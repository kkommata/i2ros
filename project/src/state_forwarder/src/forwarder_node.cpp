#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

class StateForwarder {
  public:
    StateForwarder(ros::NodeHandle &nh) {
        pose_sub_ = nh.subscribe("/Unity_ROS_message_Rx/OurCar/CoM/pose", 1, &StateForwarder::poseCallback, this);
        twist_sub_ = nh.subscribe("/Unity_ROS_message_Rx/OurCar/CoM/twist", 1, &StateForwarder::twistCallback, this);
        odom_pub_ = nh.advertise<nav_msgs::Odometry>("/forward_state", 1);
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
        last_pose_ = *msg;
        publishOdom();
    }

    void twistCallback(const geometry_msgs::TwistStamped::ConstPtr &msg) {
        last_twist_ = *msg;
        publishOdom();
    }

    void publishOdom() {
        // only publish if we have both pose and twist
        if (last_pose_.header.stamp.toSec() == 0 || last_twist_.header.stamp.toSec() == 0)
            return;

        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "world";
        odom.child_frame_id = "odom";
        odom.pose.pose = last_pose_.pose;
        odom.twist.twist = last_twist_.twist;

        odom_pub_.publish(odom);
    }

  private:
    ros::Subscriber pose_sub_, twist_sub_;
    ros::Publisher odom_pub_;
    geometry_msgs::PoseStamped last_pose_;
    geometry_msgs::TwistStamped last_twist_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "state_forwarder_node");
    ros::NodeHandle nh;
    StateForwarder forwarder(nh);
    ros::spin();
    return 0;
}
