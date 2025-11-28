#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

class OdomTfPublisher {
  public:
    OdomTfPublisher() {
        pose_sub_ = nh_.subscribe("/Unity_ROS_message_Rx/OurCar/CoM/pose", 10, &OdomTfPublisher::poseCallback, this);
        odom_initialized_ = false;
        startup_time_ = ros::Time::now();
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg) {
        if (!odom_initialized_) {
            if ((ros::Time::now() - startup_time_).toSec() < 2.0) {
                ROS_WARN_THROTTLE(1.0, "Waiting before initializing odom...");
                return;
            }

            // 设置初始原点
            initial_pose_.header.stamp = pose_msg->header.stamp;
            initial_pose_.header.frame_id = "world";
            initial_pose_.pose.position.x = 12.277;
            initial_pose_.pose.position.y = -62.799;
            initial_pose_.pose.position.z = -0.837;

            tf2::Quaternion q;
            q.setRPY(0, 0, 0.0); // 无初始朝向旋转
            initial_pose_.pose.orientation = tf2::toMsg(q);

            odom_initialized_ = true;
            ROS_INFO("Odom origin set to fixed pose: x=%.3f, y=%.3f, z=%.3f", initial_pose_.pose.position.x, initial_pose_.pose.position.y,
                     initial_pose_.pose.position.z);

            // 立即广播一次 world -> odom（含90度旋转）
            geometry_msgs::TransformStamped world_to_odom;
            world_to_odom.header.stamp = pose_msg->header.stamp;
            world_to_odom.header.frame_id = "world";
            world_to_odom.child_frame_id = "odom";

            tf2::Transform tf_initial, tf_world_to_odom;
            tf2::fromMsg(initial_pose_.pose, tf_initial);

            tf2::Quaternion q_rot90;
            q_rot90.setRPY(0, 0, -M_PI_2); // 绕Z轴旋转90度
            tf2::Transform tf_rot90(q_rot90);

            tf_world_to_odom = (tf_initial * tf_rot90).inverse(); // 先平移再旋转，再取逆
            world_to_odom.transform = tf2::toMsg(tf_world_to_odom);

            tf_broadcaster_.sendTransform(world_to_odom);
            return;
        }

        // 发布 world -> odom（含90度旋转）
        geometry_msgs::TransformStamped world_to_odom;
        world_to_odom.header.stamp = pose_msg->header.stamp;
        world_to_odom.header.frame_id = "world";
        world_to_odom.child_frame_id = "odom";

        tf2::Transform tf_initial, tf_world_to_odom;
        tf2::fromMsg(initial_pose_.pose, tf_initial);

        tf2::Quaternion q_rot90;
        q_rot90.setRPY(0, 0, -M_PI_2);
        tf2::Transform tf_rot90(q_rot90);

        tf_world_to_odom = (tf_initial * tf_rot90).inverse();
        world_to_odom.transform = tf2::toMsg(tf_world_to_odom);

        // 使用 world->base_link 的姿态和位置
        tf2::Transform tf_world_to_base_link;
        tf2::fromMsg(pose_msg->pose, tf_world_to_base_link);

        // 计算 odom -> base_link
        tf2::Transform tf_odom_to_base_link = tf_world_to_odom.inverse() * tf_world_to_base_link;

        geometry_msgs::TransformStamped odom_to_base_link;
        odom_to_base_link.header.stamp = pose_msg->header.stamp;
        odom_to_base_link.header.frame_id = "odom";
        odom_to_base_link.child_frame_id = "base_link";

        // 构造一个以 base_link 为局部坐标系的变换：
        tf2::Transform tf_base_adjust;
        tf_base_adjust.setOrigin(tf2::Vector3(0.0, 0.0, -0.71));
        tf_base_adjust.setRotation(tf2::Quaternion::getIdentity());

        // 在 base_link 坐标系中应用这个偏移（局部变换）
        tf2::Transform tf_adjusted = tf_odom_to_base_link * tf_base_adjust;

        // 广播两个 TF
        tf_broadcaster_.sendTransform(world_to_odom);
        odom_to_base_link.transform = tf2::toMsg(tf_adjusted);
        tf_broadcaster_.sendTransform(odom_to_base_link);

        // ➕ 新增 TF：base_link → rear_axle_link
        // geometry_msgs::TransformStamped base_to_rear_axle;
        // base_to_rear_axle.header.stamp = pose_msg->header.stamp;
        // base_to_rear_axle.header.frame_id = "base_link";
        // base_to_rear_axle.child_frame_id = "rear_axle_link";

        // // rear_axle 在 base_link 坐标系下的位置
        // base_to_rear_axle.transform.translation.x = -1.35;
        // base_to_rear_axle.transform.translation.y = 0.0;
        // base_to_rear_axle.transform.translation.z = 0.0;

        // tf2::Quaternion q_identity;
        // q_identity.setRPY(0, 0, 0);
        // base_to_rear_axle.transform.rotation = tf2::toMsg(q_identity);

        // tf_broadcaster_.sendTransform(base_to_rear_axle);
    }

  private:
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    geometry_msgs::PoseStamped initial_pose_;
    bool odom_initialized_;
    ros::Time startup_time_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "odom_tf_publisher");
    OdomTfPublisher node;
    ros::spin();
    return 0;
}
