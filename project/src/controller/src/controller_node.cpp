#include <ros/console.h>
#include <ros/ros.h>

#include <eigen_conversions/eigen_msg.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
// #include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
// #include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <perception_yolo/TrafficLightState.h>
#include <simulation/VehicleControl.h>
#include <std_msgs/Bool.h>

#define PI M_PI

#include <eigen3/Eigen/Dense>

// If you choose to use Eigen, tf provides useful functions to convert tf
// messages to eigen types and vice versa, have a look to the documentation:
// http://docs.ros.org/melodic/api/eigen_conversions/html/namespacetf.html

class controllerNode {
    ros::NodeHandle nh;

    ros::Subscriber current_state;
    ros::Subscriber cmd_vel_sub, light_sub;
    ros::Publisher car_commands;
    ros::Timer timer;
    ros::Time red_light_until_;

    // Controller internals (you will have to set them below)
    // Current state
    Eigen::Vector3d x;     // current position of the UAV's c.o.m. in the world frame
    Eigen::Vector3d v;     // current velocity of the UAV's c.o.m. in the world frame
    Eigen::Matrix3d R;     // current orientation of the UAV
    Eigen::Vector3d omega; // current angular velocity of the UAV's c.o.m. in the *body* frame
    Eigen::Vector3d v_forward;

    double ackermann_cmd_steering_angle;
    double ackermann_cmd_vel;
    double wheelbase;

    // 车辆物理参数
    double max_steering_angle; // max turn abgle (rad)
    double max_accel;          // max accel (m/s²)
    double max_decel;          // max_decel (m/s²)
    double max_forward_speed;  // max_f_speed (m/s)
    double max_reverse_speed;  // max_r_speed (m/s)

    double hz;

    // PID控制器增益
    double kp; // 比例增益，增大加速响应，容易振荡，减小降低响应，更稳定，车辆加速缓慢 → 增大 Kp，速度反复震荡 → 减小 Kp
    double ki;             // 积分增益，消除稳态误差，速度始终低于目标值 → 增大 Ki，车辆加速后反复“过冲”
    double kd;             // 微分增益，抑制速度变化率，车辆加速后明显超调 → 增大 Kd， 速度抖动明显 → 减小 Kd
    double prev_error_vel; // PID控制器状态变量: 上一时刻速度误差
    double integral_vel;   // PID控制器状态变量: 积分项累加值
    bool red_light_stop_;
    simulation::VehicleControl control_msg;

  public:
    controllerNode() : hz(100.0), ackermann_cmd_steering_angle{0.0}, ackermann_cmd_vel{0.0}, prev_error_vel(0.0), integral_vel(0.0), red_light_stop_(false) {
        // 从参数服务器获取参数
        nh.param("/vehicle/max_steering_angle", max_steering_angle, 0.349);    // 默认+-20度
        nh.param("/vehicle/max_accel", max_accel, 1.89);                       // 默认1.89 m/s²
        nh.param("/vehicle/max_decel", max_decel, 2.0);                        // 默认2.0 m/s²
        nh.param("/vehicle/max_forward_speed", max_forward_speed, 8.5);        // 最大速度8.5m/s
        nh.param("/vehicle/max_reverse_speed", max_reverse_speed, -1.0);       // 默认倒车速度1m/s
        wheelbase = nh.param("/move_base/TebLocalPlannerROS/wheelbase", 2.63); // 轴距

        // PID参数 (可从参数服务器配置)
        nh.param("controller/kp", kp, 2.0);
        nh.param("controller/ki", ki, 0.1);
        nh.param("controller/kd", kd, 0.1);

        current_state = nh.subscribe("/forward_state", 1, &controllerNode::onCurrentState, this);
        cmd_vel_sub = nh.subscribe("/cmd_vel", 1, &controllerNode::onVelConvert, this);
        light_sub = nh.subscribe("/perception/yolo/traffic_light", 1, &controllerNode::lightCallback, this);
        car_commands = nh.advertise<simulation::VehicleControl>("/car_command", 1);
        timer = nh.createTimer(ros::Rate(hz), &controllerNode::controlLoop, this);
        wheelbase = nh.param("/move_base/TebLocalPlannerROS/wheelbase", 2.63);
    }

    void onCurrentState(const nav_msgs::Odometry &cur_state) {
        x << cur_state.pose.pose.position.x, cur_state.pose.pose.position.y, cur_state.pose.pose.position.z;
        v << cur_state.twist.twist.linear.x, cur_state.twist.twist.linear.y, cur_state.twist.twist.linear.z;
        omega << cur_state.twist.twist.angular.x, cur_state.twist.twist.angular.y, cur_state.twist.twist.angular.z;
        Eigen::Quaterniond q;
        tf::quaternionMsgToEigen(cur_state.pose.pose.orientation, q);
        R = q.toRotationMatrix();

        v_forward = R.transpose() * v;
        omega = R.transpose() * omega; // Rotate omega
    }

    void onVelConvert(const geometry_msgs::Twist &cmd_vel) {
        if (red_light_stop_) {
            ROS_WARN("[controller] Ignoring cmd_vel due to RED LIGHT. ");
            return; // 关键：红灯时不处理目标速度
        }

        ackermann_cmd_vel = std::clamp(cmd_vel.linear.x, max_reverse_speed, max_forward_speed);

        if (ackermann_cmd_vel == 0 || cmd_vel.angular.z == 0) {
            ackermann_cmd_steering_angle = 0.0;
        } else {
            ackermann_cmd_steering_angle = atan(wheelbase * cmd_vel.angular.z / ackermann_cmd_vel);

            // 限制转向角度在物理范围内
            ackermann_cmd_steering_angle = std::clamp(ackermann_cmd_steering_angle, -max_steering_angle, max_steering_angle);
        }
    }

    void lightCallback(const perception_yolo::TrafficLightState &msg) {
        ros::Time now = ros::Time::now();

        if (msg.state == "red") {
            if (!red_light_stop_) {
                red_light_until_ = now + ros::Duration(3.0);
                ROS_WARN("[controller] RED LIGHT triggered. Forcing stop at least 3 seconds.");
            }
            red_light_stop_ = true;
        } else {
            // 绿灯来临，但要等够 3 秒
            if (now >= red_light_until_) {
                if (red_light_stop_) {
                    ROS_INFO("[controller] GREEN LIGHT - Resuming.");
                }
                red_light_stop_ = false;
            }
        }
    }

    void controlLoop(const ros::TimerEvent &t) {
        if (red_light_stop_) {
            ROS_WARN_THROTTLE(2.0, "[controller] RED LIGHT active - holding position.");
            control_msg.Throttle = 0.0;
            control_msg.Steering = 0.0;
            control_msg.Brake = 1.0;
            control_msg.Reserved = 0.0;
            car_commands.publish(control_msg);
            return;
        }

        double current_vel = v_forward[0]; // 车体前向速度
        double target_vel = ackermann_cmd_vel;

        // 计算速度误差, 注意方向
        double error_vel = target_vel - current_vel; // 目标速度和当前车辆速度误差

        // PID控制器计算原始加速度指令
        double derivative_vel = (error_vel - prev_error_vel) * hz;
        integral_vel += error_vel * (1.0 / hz);
        double raw_accel = kp * error_vel + ki * integral_vel + kd * derivative_vel;
        prev_error_vel = error_vel;

        // 方向相反时强制刹停
        if (target_vel * current_vel < -0.1) { // 速度方向相反且超过阈值
            control_msg.Throttle = 0.0;
            control_msg.Steering = 0.0;
            control_msg.Brake = 1.0;
            control_msg.Reserved = 0.0;
            car_commands.publish(control_msg);
            return;
        }

        // 控制指令初始化
        control_msg.Throttle = 0.0;
        control_msg.Steering = 0.0;
        control_msg.Brake = 0.0;
        control_msg.Reserved = 0.0;

        // 转向控制 (归一化到[-1, 1])
        if (max_steering_angle > 0.001) {
            control_msg.Steering = -ackermann_cmd_steering_angle / max_steering_angle;
        }

        // 零速死区处理
        constexpr double ZERO_VEL_THRESH = 0.1; // 0.1 m/s
        if (fabs(target_vel) < ZERO_VEL_THRESH && fabs(current_vel) < ZERO_VEL_THRESH) {
            control_msg.Brake = 0.0; // 释放刹车
            car_commands.publish(control_msg);
            return;
        }

        // 油门/刹车控制分配
        if (target_vel >= 0) {   // 前进模式
            if (raw_accel > 0) { // 需要加速
                control_msg.Throttle = std::clamp(raw_accel / max_accel, 0.0, 1.0);
            } else { // 需要减速
                control_msg.Brake = std::clamp(-raw_accel / max_decel, 0.0, 1.0);
            }
        } else {                 // 倒车模式
            if (raw_accel < 0) { // 需要加速倒车
                control_msg.Throttle = std::clamp(raw_accel / max_accel, -1.0, 0.0);
            } else { // 需要减速 (倒车时刹车)
                control_msg.Brake = std::clamp(raw_accel / max_decel, 0.0, 1.0);
            }
        }

        // 最终输出限幅
        control_msg.Throttle = std::clamp<float>(control_msg.Throttle, -1.0, 1.0);
        control_msg.Steering = std::clamp<float>(control_msg.Steering, -1.0, 1.0);
        control_msg.Brake = std::clamp<float>(control_msg.Brake, 0.0, 1.0);

        car_commands.publish(control_msg);

        // // 计算积分和微分
        // integral_vel += error_vel * (1.0 / hz);
        // double derivative_vel = (error_vel - prev_error_vel) * hz;

        // // PID控制器输出
        // double control_output_vel = kp * error_vel + ki * integral_vel + kd * derivative_vel;

        // // 更新前一时刻误差
        // prev_error_vel = error_vel;

        // control_msg.Throttle = control_output_vel;                  // 使用PID控制器计算的加速度命令;
        // control_msg.Steering = -ackermann_cmd_steering_angle * 1.5; // Turning angle
        // if (v_forward[0] == 0.0 || red_light_stop_) {
        //     control_msg.Brake = 0.0;
        // } else {
        //     control_msg.Brake = std::min((ackermann_cmd_vel - v_forward[0]) / v_forward[0] * 0.0, 0.0); // Braking
        // }
        // control_msg.Reserved = 0.0;
        // car_commands.publish(control_msg);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "controller_node");
    ROS_INFO_NAMED("controller", "Controller started!");
    controllerNode n;
    ros::spin();
}

