// Copyright (C) 2025 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//
/*****************************************************************************
 * Example of how to convert between different frame conventions.
 ****************************************************************************/
#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <ros_frame_convention/frame_convention.hpp>

class FrameConventionNode : public rclcpp::Node
{
public:
    FrameConventionNode() : Node("frame_convention_node")
    {
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/robot/odometry", rclcpp::SensorDataQoS(), std::bind(&FrameConventionNode::CallbackOdometry, this, std::placeholders::_1));
    }

private:
    void CallbackOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Only process new messages once a second.
        if(msg->header.stamp.sec <= _prev_time.sec)
        {
            return;
        }

        _prev_time = msg->header.stamp;

        // Print the received odometry message.
        RCLCPP_INFO(this->get_logger(),
                    "Odometry received [ENU/FLU]: \n"
                    " - Position: [%.2f, %.2f, %.2f] \n"
                    " - Orientation (quaternion): [%.2f, %.2f, %.2f, %.2f] \n"
                    " - Linear Velocity: [%.2f, %.2f, %.2f], \n"
                    " - Angular Velocity: [%.2f, %.2f, %.2f]",
                    msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z,
                    msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,
                    msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z,
                    msg->twist.twist.angular.x, msg->twist.twist.angular.y,
                    msg->twist.twist.angular.z);

        // Convert the received odometry from ENU/FLU to NED/FRD frame convention.
        nav_msgs::msg::Odometry odom_ned_frd = creos_ros::ConvertEnuFluToNedFrd(*msg);

        // Print the converted odometry message.
        RCLCPP_INFO(this->get_logger(),
                    "Odometry converted [NED/FRD]: \n"
                    " - Position: [%.2f, %.2f, %.2f] \n"
                    " - Orientation (quaternion): [%.2f, %.2f, %.2f, %.2f] \n"
                    " - Linear Velocity: [%.2f, %.2f, %.2f], \n"
                    " - Angular Velocity: [%.2f, %.2f, %.2f]",
                    odom_ned_frd.pose.pose.position.x, odom_ned_frd.pose.pose.position.y,
                    odom_ned_frd.pose.pose.position.z, odom_ned_frd.pose.pose.orientation.w,
                    odom_ned_frd.pose.pose.orientation.x, odom_ned_frd.pose.pose.orientation.y,
                    odom_ned_frd.pose.pose.orientation.z, odom_ned_frd.twist.twist.linear.x,
                    odom_ned_frd.twist.twist.linear.y, odom_ned_frd.twist.twist.linear.z,
                    odom_ned_frd.twist.twist.angular.x, odom_ned_frd.twist.twist.angular.y,
                    odom_ned_frd.twist.twist.angular.z);
    }

    // ROS Subscriptions
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

    // Last received time.
    builtin_interfaces::msg::Time _prev_time;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FrameConventionNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
