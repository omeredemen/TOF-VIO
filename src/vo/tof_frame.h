#ifndef TOF_FRAME_H
#define TOF_FRAME_H

#include "include/common.h"
#include <Eigen/Dense>
#include <Eigen/Eigen>

#include <pcl/io/io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/transformation_from_correspondences.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace cv;
using point_cloud2 = sensor_msgs::msg::PointCloud2;
using image_msg = sensor_msgs::msg::Image;

class TOF_Frame
{
public:
  typedef std::shared_ptr<TOF_Frame> Ptr;

  Eigen::Affine3d T_cw;
  Eigen::Affine3d T_wc;
  CloudTPtr       cloud;
  CloudTPtr       sailent_cloud;
  Mat             i_img;

  TOF_Frame();

  void clear();
  void read_PC_Iimg_FromROSMsg(const point_cloud2::SharedPtr &pcPtr,
                               const image_msg::SharedPtr &mono8Ptr);
  static void copy(TOF_Frame &frome, TOF_Frame &to);

};



#endif // TOF_FRAME_H
