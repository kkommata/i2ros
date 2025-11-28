#include <actionlib/client/simple_action_client.h>
#include <cmath>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <simulation/VehicleControl.h>
#include <string>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct Waypoint {
    int id;
    std::string name;
    geometry_msgs::Pose pose;
    bool orientation_set;
    bool is_return_point;
};

class WaypointNavigator {
  public:
    WaypointNavigator() : tf_listener_(tf_buffer_), move_base_client_("move_base", true) {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

        private_nh.param("position_tolerance", position_tolerance_, 10.0);
        private_nh.param("orientation_tolerance", orientation_tolerance_, 3.14);
        private_nh.param("retry_attempts", retry_attempts_, 3);
        private_nh.param("loop_waypoints", loop_waypoints_, false);
        private_nh.param("nav_timeout", nav_timeout_, 60.0);
        private_nh.param("update_rate", update_rate_, 10.0);
        private_nh.param("initial_move_distance", initial_move_distance_, 0.1);

        current_speed_ = 0.0;
        max_throttle_ = 1.0;
        acceleration_ = 10.0;
        target_speed_ = 3.0;

        if (!loadWaypoints(private_nh)) {
            ROS_ERROR("Failed to load waypoints! Shutting down.");
            ros::shutdown();
            return;
        }

        ROS_INFO("Waiting for move_base server...");
        if (!move_base_client_.waitForServer(ros::Duration(10.0))) {
            ROS_ERROR("Could not connect to move_base server!");
            ros::shutdown();
            return;
        }
        ROS_INFO("Connected to move_base server");

        pose_sub_ = nh.subscribe("/Unity_ROS_message_Rx/OurCar/CoM/pose", 1, &WaypointNavigator::poseCallback, this);
        marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/waypoint_markers", 1, true);
        path_pub_ = nh.advertise<nav_msgs::Path>("/planned_path", 1);
        vehicle_control_pub_ = nh.advertise<simulation::VehicleControl>("/car_command", 1);

        state_ = WAITING_FOR_INITIAL_POSE;
        current_goal_index_ = -1;
        retry_count_ = 0;
        navigation_active_ = false;
        goal_sent_ = false;
        initial_move_completed_ = false;

        update_timer_ = nh.createTimer(ros::Duration(1.0 / update_rate_), &WaypointNavigator::updateNavigation, this);

        visualizeWaypoints();
        ROS_INFO("WaypointNavigator initialized successfully.");
    }

    void run() { ros::spin(); }

  private:
    enum State { WAITING_FOR_INITIAL_POSE, INITIAL_MOVE_DIRECT, NAVIGATING, FINISHED };

    ros::Subscriber pose_sub_;
    ros::Publisher marker_pub_;
    ros::Publisher path_pub_;
    ros::Publisher vehicle_control_pub_;
    ros::Timer update_timer_;
    ros::Timer nav_timer_;
    MoveBaseClient move_base_client_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    double position_tolerance_;
    double orientation_tolerance_;
    int retry_attempts_;
    bool loop_waypoints_;
    double nav_timeout_;
    double update_rate_;
    double initial_move_distance_;

    State state_;
    int current_goal_index_;
    int retry_count_;
    bool navigation_active_;
    bool goal_sent_;
    bool initial_move_completed_;

    double current_speed_;
    ros::Time last_move_time_;
    double max_throttle_;
    double acceleration_;
    double target_speed_;

    std::vector<Waypoint> waypoints_;
    std::vector<Waypoint> completed_waypoints_;
    std::vector<Waypoint> failed_waypoints_;
    geometry_msgs::Pose current_pose_;
    geometry_msgs::Point initial_position_;
    geometry_msgs::Quaternion initial_orientation_;
    geometry_msgs::Point initial_move_start_point_;

    bool loadWaypoints(ros::NodeHandle &nh) {
        ROS_INFO("Attempting to load waypoints from parameter server...");

        XmlRpc::XmlRpcValue waypoint_list;
        if (!nh.getParam("waypoints", waypoint_list)) {
            ROS_ERROR("No 'waypoints' parameter found in namespace '%s'", nh.getNamespace().c_str());
            return false;
        }

        if (waypoint_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_ERROR("Waypoints parameter is not an array! Type: %d", waypoint_list.getType());
            return false;
        }

        ROS_INFO("Found %d waypoints in configuration", waypoint_list.size());

        for (int i = 0; i < waypoint_list.size(); ++i) {
            XmlRpc::XmlRpcValue wp = waypoint_list[i];

            if (wp.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
                ROS_ERROR("Waypoint %d is not a struct (Type: %d). Skipping...", i, wp.getType());
                continue;
            }

            Waypoint waypoint;
            waypoint.id = i;
            waypoint.orientation_set = false;
            waypoint.is_return_point = false;

            // 获取名称
            if (wp.hasMember("name")) {
                if (wp["name"].getType() == XmlRpc::XmlRpcValue::TypeString) {
                    waypoint.name = static_cast<std::string>(wp["name"]);
                } else {
                    ROS_WARN("Waypoint %d 'name' is not a string. Using default.", i);
                    waypoint.name = "Waypoint " + std::to_string(i);
                }
            } else {
                ROS_WARN("Waypoint %d missing 'name'. Using default.", i);
                waypoint.name = "Waypoint " + std::to_string(i);
            }

            // 获取位置
            if (wp.hasMember("position")) {
                if (wp["position"].getType() == XmlRpc::XmlRpcValue::TypeStruct) {
                    XmlRpc::XmlRpcValue pos = wp["position"];

                    // 检查并获取x坐标
                    if (pos.hasMember("x") && pos["x"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                        waypoint.pose.position.x = static_cast<double>(pos["x"]);
                    } else {
                        ROS_ERROR("Waypoint %d missing or invalid 'x' coordinate", i);
                        continue;
                    }

                    // 检查并获取y坐标
                    if (pos.hasMember("y") && pos["y"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                        waypoint.pose.position.y = static_cast<double>(pos["y"]);
                    } else {
                        ROS_ERROR("Waypoint %d missing or invalid 'y' coordinate", i);
                        continue;
                    }

                    // 获取z坐标（可选）
                    if (pos.hasMember("z") && pos["z"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                        waypoint.pose.position.z = static_cast<double>(pos["z"]);
                    } else {
                        waypoint.pose.position.z = 0.0; // 默认为0
                        ROS_WARN("Waypoint %d missing 'z' coordinate. Using 0.0", i);
                    }
                } else {
                    ROS_ERROR("Waypoint %d 'position' is not a struct", i);
                    continue;
                }
            } else {
                ROS_ERROR("Waypoint %d missing 'position' data", i);
                continue;
            }

            waypoints_.push_back(waypoint);
            ROS_INFO("Loaded waypoint %d: %s at (%.2f, %.2f, %.2f)", i, waypoint.name.c_str(), waypoint.pose.position.x, waypoint.pose.position.y,
                     waypoint.pose.position.z);
        }

        if (waypoints_.empty()) {
            ROS_ERROR("No valid waypoints loaded!");
            return false;
        }

        ROS_INFO("Successfully loaded %lu waypoints", waypoints_.size());
        return true;
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
        current_pose_ = msg->pose;

        if (state_ == WAITING_FOR_INITIAL_POSE) {
            initial_position_ = msg->pose.position;
            initial_orientation_ = msg->pose.orientation;
            initial_move_start_point_ = msg->pose.position;
            last_move_time_ = ros::Time::now();
            state_ = INITIAL_MOVE_DIRECT;
            ROS_INFO("Recorded initial position and orientation, starting initial move");
        }

        try {
            geometry_msgs::PoseStamped in_pose, out_pose;
            in_pose.header = msg->header;
            in_pose.pose = current_pose_;

            geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform("world", msg->header.frame_id, ros::Time(0), ros::Duration(1.0));

            tf2::doTransform(in_pose, out_pose, transform);
            current_pose_ = out_pose.pose;
        } catch (tf2::TransformException &ex) {
            ROS_WARN("TF transform failed: %s", ex.what());
        }
    }

    double calculateDistance(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2) {
        double dx = pose1.position.x - pose2.position.x;
        double dy = pose1.position.y - pose2.position.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    double calculateDistance(const geometry_msgs::Point &pt1, const geometry_msgs::Point &pt2) {
        double dx = pt1.x - pt2.x;
        double dy = pt1.y - pt2.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    double calculateAngleDifference(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2) {
        double yaw1 = tf2::getYaw(pose1.orientation);
        double yaw2 = tf2::getYaw(pose2.orientation);

        double angle_diff = std::abs(yaw1 - yaw2);
        if (angle_diff > M_PI) {
            angle_diff = 2 * M_PI - angle_diff;
        }

        return angle_diff;
    }

    bool isGoalReached() {
        if (current_goal_index_ < 0 || current_goal_index_ >= waypoints_.size()) {
            return false;
        }

        const geometry_msgs::Pose &goal_pose = waypoints_[current_goal_index_].pose;
        double distance = calculateDistance(current_pose_, goal_pose);

        double angle_diff = calculateAngleDifference(current_pose_, goal_pose);
        return (distance <= position_tolerance_) && (angle_diff <= orientation_tolerance_);
        // return distance <= position_tolerance_;
    }

    geometry_msgs::Quaternion calculateOrientationTowardsNext(int current_index) {
        geometry_msgs::Point next_point;

        if (current_index == static_cast<int>(waypoints_.size()) - 1) {
            next_point = initial_position_;
        } else {
            next_point = waypoints_[current_index + 1].pose.position;
        }

        const geometry_msgs::Point &current_point = waypoints_[current_index].pose.position;
        double dx = next_point.x - current_point.x;
        double dy = next_point.y - current_point.y;

        double yaw = std::atan2(dy, dx);
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);

        geometry_msgs::Quaternion orientation;
        tf2::convert(q, orientation);
        return orientation;
    }

    void performInitialMove() {
        double distance_moved = calculateDistance(current_pose_.position, initial_move_start_point_);

        if (distance_moved < initial_move_distance_) {
            ros::Time now = ros::Time::now();
            double dt = (now - last_move_time_).toSec();
            last_move_time_ = now;

            if (dt <= 0 || dt > 0.5)
                dt = 0.1;

            if (current_speed_ < target_speed_) {
                current_speed_ += acceleration_ * dt;
                if (current_speed_ > target_speed_) {
                    current_speed_ = target_speed_;
                }
            }

            double throttle = 0.0;
            if (current_speed_ < target_speed_) {
                throttle = max_throttle_ * (1.0 - current_speed_ / target_speed_);
            } else {
                throttle = max_throttle_ * 0.3;
            }

            simulation::VehicleControl control_msg;
            control_msg.Throttle = static_cast<float>(throttle);
            control_msg.Steering = 0.0f;
            control_msg.Brake = 0.0f;
            control_msg.Reserved = 0.0f;

            vehicle_control_pub_.publish(control_msg);
            ROS_INFO_THROTTLE(3, "Initial move: %.1f/%.1fm @ %.1fm/s (throttle: %.1f%%)", distance_moved, initial_move_distance_, current_speed_,
                              throttle * 100);
        } else {

            simulation::VehicleControl stop_msg;
            stop_msg.Throttle = 0.0f;
            stop_msg.Steering = 0.0f;
            stop_msg.Brake = 0.5f;
            stop_msg.Reserved = 0.0f;

            vehicle_control_pub_.publish(stop_msg);
            current_speed_ = 0.0;

            ROS_INFO("Initial move completed, starting waypoint navigation");
            initial_move_completed_ = true;
            state_ = NAVIGATING;
            selectNextWaypoint();
        }
    }

    void selectNextWaypoint() {
        ROS_WARN("Selecting next waypoint. Completed: %zu, Failed: %zu", completed_waypoints_.size(), failed_waypoints_.size());

        if (std::isnan(current_pose_.position.x)) {
            ROS_WARN("No position received, cannot select waypoint");
            return;
        }

        int next_index = -1;
        for (size_t i = 0; i < waypoints_.size(); ++i) {
            bool completed = false;
            for (const auto &wp : completed_waypoints_) {
                if (wp.id == waypoints_[i].id) {
                    completed = true;
                    break;
                }
            }

            bool failed = false;
            for (const auto &wp : failed_waypoints_) {
                if (wp.id == waypoints_[i].id) {
                    failed = true;
                    break;
                }
            }

            if (!completed && !failed) {
                next_index = i;
                break;
            }
        }

        if (next_index == -1) {
            if (loop_waypoints_) {
                ROS_INFO("All waypoints completed, restarting loop...");
                completed_waypoints_.clear();
                failed_waypoints_.clear();
                next_index = 0;
            } else {
                ROS_INFO("All waypoints completed, preparing to return to start!");
                prepareReturnToStart();
                return;
            }
        }

        current_goal_index_ = next_index;

        if (!waypoints_[next_index].orientation_set) {
            waypoints_[next_index].pose.orientation = calculateOrientationTowardsNext(next_index);
            waypoints_[next_index].orientation_set = true;
        }

        double distance = calculateDistance(current_pose_, waypoints_[next_index].pose);
        ROS_INFO("Selected waypoint %d (%s) - distance: %.2fm", current_goal_index_, waypoints_[current_goal_index_].name.c_str(), distance);

        sendGoalToMoveBase();
    }

    void prepareReturnToStart() {
        Waypoint return_point;
        return_point.id = waypoints_.size();
        return_point.name = "Return to Start";
        return_point.pose.position = initial_position_;
        return_point.pose.orientation = initial_orientation_;
        return_point.orientation_set = true;
        return_point.is_return_point = true;

        waypoints_.push_back(return_point);
        current_goal_index_ = waypoints_.size() - 1;

        ROS_INFO("Added return to start waypoint");
        sendGoalToMoveBase();
    }

    bool sendGoalToMoveBase() {
        if (goal_sent_) {
            ROS_WARN("Goal already sent, skipping duplicate request");
            return false;
        }

        if (current_goal_index_ < 0 || current_goal_index_ >= static_cast<int>(waypoints_.size())) {
            return false;
        }

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "world";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose = waypoints_[current_goal_index_].pose;

        move_base_client_.sendGoal(goal, boost::bind(&WaypointNavigator::goalDoneCallback, this, _1, _2));

        goal_sent_ = true;
        navigation_active_ = true;
        ROS_INFO("Sent waypoint %d to move_base", current_goal_index_);

        nav_timer_ = ros::NodeHandle().createTimer(ros::Duration(nav_timeout_), &WaypointNavigator::navTimeoutCallback, this, true);
        visualizeWaypoints();
        return true;
    }

    void navTimeoutCallback(const ros::TimerEvent &event) {
        if (navigation_active_ && goal_sent_) {
            ROS_WARN("Navigation to waypoint %d timed out!", current_goal_index_);
            move_base_client_.cancelGoal();
            handleNavigationFailure();
        }
    }

    void goalDoneCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResult::ConstPtr &result) {
        goal_sent_ = false;
        nav_timer_.stop();

        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Reached waypoint %d", current_goal_index_);

            if (waypoints_[current_goal_index_].is_return_point) {
                ROS_INFO("Returned to start, navigation complete!");
                state_ = FINISHED;
                navigation_active_ = false;
                return;
            }

            completed_waypoints_.push_back(waypoints_[current_goal_index_]);
            retry_count_ = 0;

            selectNextWaypoint();
        } else {
            ROS_WARN("Failed to reach waypoint %d (state: %s)", current_goal_index_, state.toString().c_str());
            handleNavigationFailure();
        }
    }

    void handleNavigationFailure() {
        retry_count_++;

        if (retry_count_ >= retry_attempts_) {
            ROS_ERROR("Waypoint %d retries exhausted, marking as failed", current_goal_index_);
            failed_waypoints_.push_back(waypoints_[current_goal_index_]);
            retry_count_ = 0;
            navigation_active_ = false;
            selectNextWaypoint();
        } else {
            ROS_INFO("retry point: %d (%d/%d)", current_goal_index_, retry_count_, retry_attempts_);
            sendGoalToMoveBase();
        }
    }

    void visualizeWaypoints() {
        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::Marker delete_marker;
        delete_marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(delete_marker);

        for (size_t i = 0; i < waypoints_.size(); ++i) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time::now();
            marker.ns = "waypoints";
            marker.id = i;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose = waypoints_[i].pose;
            marker.scale.x = 0.5;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;

            if (static_cast<int>(i) == current_goal_index_ && navigation_active_) {
                // green
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.color.a = 1.0;
            } else if (isWaypointCompleted(waypoints_[i].id)) {
                // blue
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
                marker.color.a = 0.5;
            } else if (isWaypointFailed(waypoints_[i].id)) {
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
                marker.color.a = 0.7;
            } else {
                marker.color.r = 1.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.color.a = 1.0;
            }

            marker.lifetime = ros::Duration();
            marker_array.markers.push_back(marker);

            visualization_msgs::Marker label;
            label.header.frame_id = "world";
            label.header.stamp = ros::Time::now();
            label.ns = "labels";
            label.id = i;
            label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            label.action = visualization_msgs::Marker::ADD;
            label.pose.position.x = waypoints_[i].pose.position.x;
            label.pose.position.y = waypoints_[i].pose.position.y;
            label.pose.position.z = 0.5;
            label.text = waypoints_[i].name;
            label.scale.z = 0.3;
            label.color = marker.color;
            marker_array.markers.push_back(label);
        }

        visualization_msgs::Marker start_marker;
        start_marker.header.frame_id = "world";
        start_marker.header.stamp = ros::Time::now();
        start_marker.ns = "start_point";
        start_marker.id = 0;
        start_marker.type = visualization_msgs::Marker::CYLINDER;
        start_marker.action = visualization_msgs::Marker::ADD;
        start_marker.pose.position = initial_position_;
        start_marker.pose.orientation.w = 1.0;
        start_marker.scale.x = 0.3;
        start_marker.scale.y = 0.3;
        start_marker.scale.z = 0.1;
        start_marker.color.r = 0.0;
        start_marker.color.g = 1.0;
        start_marker.color.b = 0.0;
        start_marker.color.a = 0.7;
        start_marker.lifetime = ros::Duration();
        marker_array.markers.push_back(start_marker);

        visualization_msgs::Marker start_label;
        start_label.header.frame_id = "world";
        start_label.header.stamp = ros::Time::now();
        start_label.ns = "start_label";
        start_label.id = 0;
        start_label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        start_label.action = visualization_msgs::Marker::ADD;
        start_label.pose.position.x = initial_position_.x;
        start_label.pose.position.y = initial_position_.y;
        start_label.pose.position.z = 0.5;
        start_label.text = "Start Point";
        start_label.scale.z = 0.3;
        start_label.color.r = 0.0;
        start_label.color.g = 1.0;
        start_label.color.b = 0.0;
        start_label.color.a = 1.0;
        start_label.lifetime = ros::Duration();
        marker_array.markers.push_back(start_label);

        marker_pub_.publish(marker_array);
    }

    void publishPath() {
        if (waypoints_.empty() || current_goal_index_ < 0) {
            return;
        }

        nav_msgs::Path path;
        path.header.frame_id = "world";
        path.header.stamp = ros::Time::now();

        geometry_msgs::PoseStamped start_pose;
        start_pose.header = path.header;
        start_pose.pose = current_pose_;
        path.poses.push_back(start_pose);

        for (int i = current_goal_index_; i < static_cast<int>(waypoints_.size()); ++i) {
            if (!isWaypointCompleted(waypoints_[i].id) && !isWaypointFailed(waypoints_[i].id)) {
                geometry_msgs::PoseStamped pose;
                pose.header = path.header;
                pose.pose = waypoints_[i].pose;
                path.poses.push_back(pose);
            }
        }

        path_pub_.publish(path);
    }

    void updateNavigation(const ros::TimerEvent &event) {
        switch (state_) {
        case WAITING_FOR_INITIAL_POSE:
            ROS_INFO_THROTTLE(5, "Waiting for initial pose...");
            break;

        case INITIAL_MOVE_DIRECT:
            performInitialMove();
            break;

        case NAVIGATING:
            // if (isGoalReached()) {
            //     ROS_INFO("Reached waypoint %d", current_goal_index_);

            //     if (waypoints_[current_goal_index_].is_return_point) {
            //         ROS_INFO("Returned to start, navigation complete!");
            //         state_ = FINISHED;
            //         break;
            //     }

            //     completed_waypoints_.push_back(waypoints_[current_goal_index_]);
            //     retry_count_ = 0;
            //     selectNextWaypoint();
            // }
            break;

        case FINISHED:
            break;
        }

        visualizeWaypoints();
        publishPath();
    }

    bool isWaypointCompleted(int id) {
        for (const auto &wp : completed_waypoints_) {
            if (wp.id == id)
                return true;
        }
        return false;
    }

    bool isWaypointFailed(int id) {
        for (const auto &wp : failed_waypoints_) {
            if (wp.id == id)
                return true;
        }
        return false;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "waypoint_navigator");
    WaypointNavigator navigator;
    navigator.run();
    return 0;
}