#ifndef CAMERA_CLUSTER_DETECT_H
#define CAMERA_CLUSTER_DETECT_H

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// PointCloud includes
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

namespace pointcloud_cluster_detect
{
class RSCameraNode
{
public:
  RSCameraNode();
  ~RSCameraNode();

private:
  // handle
  ros::NodeHandle nh_;

  double target_height_threshold_;
  double target_width_threshold_;

  typedef pcl::PointXYZ PointT;

  // publisher
  ros::Publisher detected_object_list_pub;
  ros::Publisher detected_object_cloud_pub;

  // subscriber
  ros::Subscriber pointcloud_sub;

  // callbacks
  void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& input_cloud);

  // initializer
  void initForROS();

};

}  

#endif  
