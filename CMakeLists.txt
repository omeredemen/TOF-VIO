cmake_minimum_required(VERSION 3.8)
project(tof_vio)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL 1.3 REQUIRED)

find_package (OpenCV 4 REQUIRED)
find_package (Eigen3 REQUIRED)
find_package (cv_bridge REQUIRED)

include_directories(SYSTEM
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    "${CMAKE_CURRENT_SOURCE_DIR}/src/"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/include/"
    )

##################################################################################
add_library(nodelet_icp
    # src/vo/node_vo.cpp
    # src/vo/salientpts.cpp
    # src/vo/icp.cpp
    src/vo/tof_frame.cpp
    )
target_link_libraries(nodelet_icp
    ${OpenCV_LIBRARIES})
ament_target_dependencies(nodelet_icp
  rclcpp
  sensor_msgs                  
  geometry_msgs
  tf2_ros
  pcl_ros
  pcl_conversions
  cv_bridge
  )

install(TARGETS nodelet_icp
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()