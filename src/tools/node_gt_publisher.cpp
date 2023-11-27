#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
extern rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

//Call Back Function of motion captrure system
void MC_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = msg->header.stamp;
    odom.header.frame_id = "world";
    odom.pose.pose.orientation.w = msg->pose.orientation.w;
    odom.pose.pose.orientation.x = msg->pose.orientation.x;
    odom.pose.pose.orientation.y = msg->pose.orientation.y;
    odom.pose.pose.orientation.z = msg->pose.orientation.z;
    odom.pose.pose.position.x = msg->pose.position.x;
    odom.pose.pose.position.y = msg->pose.position.y;
    odom.pose.pose.position.z = msg->pose.position.z;
    odom_pub->publish(odom);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto listener_node = std::make_shared<rclcpp::Node>("listener");
    odom_pub = listener_node->create_publisher<nav_msgs::msg::Odometry>("odom_gt", 1000);
    auto sub = listener_node->create_subscription<geometry_msgs::msg::PoseStamped>
        ("vicon", 1000, MC_callback);
    rclcpp::spin(listener_node);
    rclcpp::shutdown();
    return 0;
}
