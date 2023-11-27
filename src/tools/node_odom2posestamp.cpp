#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <fstream>

using namespace  std;

geometry_msgs::msg::PoseStamped latest_pose;
bool first_msg = true;
uint64_t lastMsgTime;
double time_gap = 0.1;
string odom_topic_in;
string pose_topic_out;
int frequency = 10;

rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;

//Call Back Function of motion captrure system
void callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if(first_msg)
    {
        lastMsgTime = msg->header.stamp.sec;
        latest_pose.header.frame_id = "world";
        latest_pose.header.stamp=msg->header.stamp;
        latest_pose.pose.orientation.w = msg->pose.pose.orientation.w;
        latest_pose.pose.orientation.x = msg->pose.pose.orientation.x;
        latest_pose.pose.orientation.y = msg->pose.pose.orientation.y;
        latest_pose.pose.orientation.z = msg->pose.pose.orientation.z;
        latest_pose.pose.position.x = msg->pose.pose.position.x;
        latest_pose.pose.position.y = msg->pose.pose.position.y;
        latest_pose.pose.position.z = msg->pose.pose.position.z;
        pose_pub->publish(latest_pose);
        first_msg = false;
    }
    else
    {
        if((msg->header.stamp.sec - lastMsgTime) > time_gap)
        {
            lastMsgTime = msg->header.stamp.sec;
            latest_pose.header.frame_id = "world";
            latest_pose.header.stamp=msg->header.stamp;
            latest_pose.pose.orientation.w = msg->pose.pose.orientation.w;
            latest_pose.pose.orientation.x = msg->pose.pose.orientation.x;
            latest_pose.pose.orientation.y = msg->pose.pose.orientation.y;
            latest_pose.pose.orientation.z = msg->pose.pose.orientation.z;
            latest_pose.pose.position.x = msg->pose.pose.position.x;
            latest_pose.pose.position.y = msg->pose.pose.position.y;
            latest_pose.pose.position.z = msg->pose.pose.position.z;
            pose_pub->publish(latest_pose);
        }
    }
    return;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("odom2posestamp");

    node->declare_parameter<std::string>("odom_topic_in");
    node->declare_parameter<std::string>("pose_topic_out");
    odom_topic_in = node->get_parameter("odom_topic_in").as_string();
    pose_topic_out = node->get_parameter("pose_topic_out").as_string();

    auto sub = node->create_subscription<nav_msgs::msg::Odometry>(odom_topic_in, 2, callback);
    pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_out, 2);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
