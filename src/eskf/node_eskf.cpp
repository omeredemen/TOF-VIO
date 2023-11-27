#include <stdio.h>
#include <math.h>
#include <iostream>
#include <deque>

#include <Eigen/Eigen>

#include "include/euler_q_rmatrix.h"
#include "eskf/eskf_imu.h"

#include "node_eskf.hpp"

ESKF::ESKF() : Node("eskf")
{
    declareParameters();

    // You should also tune these parameters
    double ng, na, nbg, nba, n_vo_p, n_vo_q, vo_delay;
    ng = this->get_parameter("eskf/ng").as_double();
    na = this->get_parameter("eskf/na").as_double();
    nbg = this->get_parameter("eskf/nbg").as_double();
    nba = this->get_parameter("eskf/nba").as_double();
    n_vo_p = this->get_parameter("eskf/n_vo_p").as_double();
    n_vo_q = this->get_parameter("eskf/n_vo_q").as_double();
    vo_delay = this->get_parameter("eskf/vo_delay").as_double();

    cout << "ng     :" << ng << endl;
    cout << "na     :" << na << endl;
    cout << "nbg    :" << nbg << endl;
    cout << "nba    :" << nba << endl;
    cout << "n_vo_p :"  << n_vo_p << endl;
    cout << "n_vo_q :"  << n_vo_q << endl;

    eskf_imu = new ESKF_IMU(na,
                            ng,
                            nba,
                            nbg,
                            n_vo_q,
                            n_vo_q,
                            vo_delay);

    odom_pub = this->create_publisher<odom_msg>("eskf_odom", 10);

    odom_sub = this->create_subscription<odom_msg>(
        "vo", 10, std::bind(&ESKF::odom_callback_vo, this, std::placeholders::_1));
    imu_sub = this->create_subscription<imu_msg>(
        "imu", 10, std::bind(&ESKF::imu_callback, this, std::placeholders::_1));
}


void ESKF::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  eskf_imu->read_imu_msg(msg->header.stamp.sec,
                         msg->linear_acceleration.x,
                         msg->linear_acceleration.y,
                         msg->linear_acceleration.z,
                         msg->angular_velocity.x,
                         msg->angular_velocity.y,
                         msg->angular_velocity.z);
  if(!initialized)
  {
    //use the first vo input at the init value
    eskf_imu->states.push_back(eskf_imu->curr_state);
    if(eskf_imu->states.size()>=200)
    {
      eskf_imu->states.pop_front();
    }
  }
  else {//initialzed
    eskf_imu->update_Nominal_Error_Cov();
    nav_msgs::msg::Odometry eskf_odom;
    eskf_odom.header.stamp = msg->header.stamp;
    eskf_odom.header.frame_id = "world";
    SYS_STATE xt=eskf_imu->states.back();
    eskf_odom.pose.pose.position.x = xt.n_state[4];
    eskf_odom.pose.pose.position.y = xt.n_state[5];
    eskf_odom.pose.pose.position.z = xt.n_state[6];
    eskf_odom.twist.twist.linear.x = xt.n_state[7];
    eskf_odom.twist.twist.linear.y = xt.n_state[8];
    eskf_odom.twist.twist.linear.z = xt.n_state[9];
    Quaterniond odom_q;
    eskf_odom.pose.pose.orientation.w = xt.n_state[0];
    eskf_odom.pose.pose.orientation.x = xt.n_state[1];
    eskf_odom.pose.pose.orientation.y = xt.n_state[2];
    eskf_odom.pose.pose.orientation.z = xt.n_state[3];
    odom_pub->publish(eskf_odom);
    //cout << "imu update process" << endl;
  }
}


void ESKF::odom_callback_vo(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  //cout << endl << "in vo callback:" << endl;
  if (msg->pose.pose.position.x == 0.012345)
  {
    return;
  }
  else
  {
    if(!initialized)
    {
      //use the first vo input at the init value
      eskf_imu->init_from_vo(msg->header.stamp.sec,
                             msg->pose.pose.orientation.w,
                             msg->pose.pose.orientation.x,
                             msg->pose.pose.orientation.y,
                             msg->pose.pose.orientation.z,
                             msg->pose.pose.position.x,
                             msg->pose.pose.position.y,
                             msg->pose.pose.position.z);
      initialized = true;
    }
    else {//initialzed
      static int count = 0;
      eskf_imu->read_vo_msg(msg->header.stamp.sec,
                            msg->pose.pose.orientation.w,
                            msg->pose.pose.orientation.x,
                            msg->pose.pose.orientation.y,
                            msg->pose.pose.orientation.z,
                            msg->pose.pose.position.x,
                            msg->pose.pose.position.y,
                            msg->pose.pose.position.z);
      eskf_imu->innovate_ErrorState();
      eskf_imu->innovate_Inject_Reset();
      eskf_imu->innovate_reintegrate();
      nav_msgs::msg::Odometry eskf_odom;
      eskf_odom.header.stamp = msg->header.stamp;
      eskf_odom.header.frame_id = "world";
      SYS_STATE xt=eskf_imu->states.back();
      eskf_odom.pose.pose.position.x = xt.n_state[4];
      eskf_odom.pose.pose.position.y = xt.n_state[5];
      eskf_odom.pose.pose.position.z = xt.n_state[6];
      eskf_odom.twist.twist.linear.x = xt.n_state[7];
      eskf_odom.twist.twist.linear.y = xt.n_state[8];
      eskf_odom.twist.twist.linear.z = xt.n_state[9];
      Quaterniond odom_q;
      eskf_odom.pose.pose.orientation.w = xt.n_state[0];
      eskf_odom.pose.pose.orientation.x = xt.n_state[1];
      eskf_odom.pose.pose.orientation.y = xt.n_state[2];
      eskf_odom.pose.pose.orientation.z = xt.n_state[3];
      odom_pub->publish(eskf_odom);
      count++;
    }
  }
}

void ESKF::declareParameters()
{
    // Q imu covariance matrix;
    this->declare_parameter<double>("eskf/ng", 0);
    this->declare_parameter<double>("eskf/na", 0);
    this->declare_parameter<double>("eskf/nbg", 0);
    this->declare_parameter<double>("eskf/nba", 0);
    /* pnp position and orientation noise */
    this->declare_parameter<double>("eskf/vo_p", 0);
    this->declare_parameter<double>("eskf/vo_q", 0);
    this->declare_parameter<double>("eskf/vo_delay_ms", 0);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ESKF>());
  rclcpp::shutdown();
  return 0;
}
