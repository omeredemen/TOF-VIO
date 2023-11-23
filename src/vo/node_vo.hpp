#ifndef VISUALODOM_NODE_VO_HPP_
#define VISUALODOM_NODE_VO_HPP_

//ROS
#include <pluginlib/class_list_macros.hpp>
// #include <nodelet/nodelet.h>
// #include <tf/transform_broadcaster.h>
#include "tf2_ros/transform_broadcaster.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"



//Eigen
#include <Eigen/Dense>
#include <Eigen/Eigen>
//CV
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
//PCL
// #include <pcl/io/io.h>
#include <pcl/common/io.h>
// #include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/transformation_from_correspondences.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/ndt.h>
//USER
#include <include/common.h>
#include <include/tic_toc_ros.h>
#include <include/euler_q_rmatrix.h>
#include <vo/salientpts.h>
#include <vo/icp.h>
#include <vo/tof_frame.h>



using namespace cv;
using namespace std;
using namespace sensor_msgs;
using namespace Eigen;

using point_cloud2 = sensor_msgs::msg::PointCloud2;
using float32_array = std_msgs::msg::Float32MultiArray;
using odometry_msg = nav_msgs::msg::Odometry;
using image_msg = sensor_msgs::msg::Image;
using imu_msg = sensor_msgs::msg::Imu;
using pose_stamped_msg = geometry_msgs::msg::PoseStamped;

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace nodelet_ns
{
class NICP : public rclcpp::Node
{
public:
    NICP();

    void mc_callback(const pose_stamped_msg::SharedPtr msg);
    void imu_callback(const imu_msg::SharedPtr msg);
    void findNearestKeyframe(void);

    void addFrameToMap(TOF_Frame::Ptr frame);
    void visualizeDepthImg(cv::Mat& visualized_depth, cv::Mat d_img_in);
    void publish_tf(Eigen::Affine3d T_cw);
    void publish_pose(Eigen::Affine3d T_cw, rclcpp::Time pose_stamp);
    void publishPC(CloudTPtr PCptr, string frame_id, rclcpp::Publisher<point_cloud2>::SharedPtr pub);
    void tof_callback(const point_cloud2::SharedPtr pcPtr,
                        const image_msg::SharedPtr mono8Ptr,
                        const image_msg::SharedPtr depthPtr);
    void declareParameters();

private:
    TOF_Frame::Ptr curr_frame,prev_frame,key_frame;
    std::vector<TOF_Frame> keyframes;
    //map
    CloudTPtr mapcloud;

    Eigen::Affine3d T_cw_init;
    Eigen::Affine3d T_cw_imu_last;
    Eigen::Affine3d T_cw_gt_last;
    Eigen::Affine3d T_ic,T_ci;//T_ic: camera to imu

    //Salient Pts extractor and ICP alignment
    SalientPts*    salient_pts_extractor;
    ICP_ALIGNMENT* icp_alignment;

    //Flag
    bool init_by_IMU, init_by_MC, vo_mode;
    int icp_init;
    int receive_mc_data;
    int receive_imu_data;
    int FrameCount;
    int kf_criteria;
    int use_other_icp;
    bool use_orig_pts;
    bool use_ransomdownsample_pts;
    bool use_salient_pts;
    bool use_robust_w;

    // stastic
    double time_sum;

    // PUB
    rclcpp::Publisher<point_cloud2>::SharedPtr pub_cloudin;
    rclcpp::Publisher<point_cloud2>::SharedPtr pub_keyframe;
    rclcpp::Publisher<point_cloud2>::SharedPtr pub_sailentpts;
    rclcpp::Publisher<point_cloud2>::SharedPtr pub_mapcloud;
    rclcpp::Publisher<float32_array>::SharedPtr pub_tf_array;
    rclcpp::Publisher<odometry_msg>::SharedPtr pub_odom;

    image_transport::Publisher pub_colored_dimg;

    // SUB
    rclcpp::Subscription<pose_stamped_msg>::SharedPtr mc_sub;
    rclcpp::Subscription<imu_msg>::SharedPtr imu_sub; 
    // SYNC SUB
    message_filters::Subscriber<point_cloud2> pc_sub;
    message_filters::Subscriber<image_msg>    grey_sub;
    message_filters::Subscriber<image_msg>    depth_sub;
    std::shared_ptr<message_filters::TimeSynchronizer<point_cloud2, image_msg, image_msg>> temp_sync;
};
}  // namespace /* namespace_name */
#endif  // VISUALODOM_NODE_VO_HPP_

