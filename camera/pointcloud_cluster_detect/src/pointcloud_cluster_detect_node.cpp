// ROS Includes
#include <ros/ros.h>

// User defined includes
#include <pointcloud_cluster_detect/pointcloud_cluster_detect.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud_cluster_detect");
  pointcloud_cluster_detect::RSCameraNode rsc;
  ros::spin();

  return 0;
}
