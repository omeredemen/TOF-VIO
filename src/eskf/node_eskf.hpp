#ifndef ESKF_HPP_
#define ESKF_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "eskf/eskf_imu.h"

using namespace std;
using namespace Eigen;

using odom_msg = nav_msgs::msg::Odometry;
using imu_msg = sensor_msgs::msg::Imu;

#define PI (3.14159265358)

class ESKF : public rclcpp::Node
{
public:
    ESKF();
    void imu_callback(const imu_msg::SharedPtr msg);
    void odom_callback_vo(const odom_msg::SharedPtr msg);
    void declareParameters();

private:
    ESKF_IMU *eskf_imu;
    bool initialized=false;

    // PUB
    rclcpp::Publisher<odom_msg>::SharedPtr odom_pub;

    // SUB
    rclcpp::Subscription<odom_msg>::SharedPtr odom_sub;
    rclcpp::Subscription<imu_msg>::SharedPtr imu_sub;
};
#endif  // ESKF_HPP_
