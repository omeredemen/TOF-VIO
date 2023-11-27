#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <fstream>

using namespace  std;

std::ofstream fd;

//Call Back Function of motion captrure system
void callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    fd << setprecision(6)
         << msg->header.stamp.nanosec << " "
         << setprecision(9)
         << msg->pose.pose.position.x << " "
         << msg->pose.pose.position.y << " "
         << msg->pose.pose.position.z << " "
         << msg->pose.pose.orientation.w << " "
         << msg->pose.pose.orientation.x << " "
         << msg->pose.pose.orientation.y << " "
         << msg->pose.pose.orientation.z << std::endl;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node= std::make_shared<rclcpp::Node>("topic2file");

    std::string filepath;
    node->declare_parameter<std::string>("filepath", "home/omer/Log");
    filepath = node->get_parameter("filepath").as_string();
    fd.open(filepath.c_str());

    auto sub = node->create_subscription<nav_msgs::msg::Odometry>("odom", 10, callback);
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
