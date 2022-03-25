#include <pointcloud_cluster_detect/pointcloud_cluster_detect.h>

#include <pointcloud_cluster_detect/DetectedObjectArray.h>
#include <pointcloud_cluster_detect/DetectedObject.h>


namespace pointcloud_cluster_detect
{
// Constructor
RSCameraNode::RSCameraNode()
    : nh_("~")
{
    initForROS();
}

RSCameraNode::~RSCameraNode()
{}

void RSCameraNode::initForROS()
{
    pointcloud_sub_ = nh_.subscribe("/camera/depth/color/points", 5, &RSCameraNode::pointcloud_callback, this);

    detected_object_list_pub = nh_.advertise<pointcloud_cluster_detect::DetectedObjectArray>("/detected_object_list", 1);
    detected_object_cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("/detected_object_cloud", 1);

    nh_.param("target_height_threshold", target_height_threshold_, 0.05);
    nh_.param("target_width_threshold", target_width_threshold_, 0.05);
}

void RSCameraNode::pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
    // Read in the cloud data
    pcl::PointCloud<PointT>::Ptr current_sensor_cloud_ptr(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*cloud, *current_sensor_cloud_ptr);

    // Build a passthrough filter to remove spurious NaNs
    pcl::PassThrough<PointT> pass;
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_f(new pcl::PointCloud<PointT>);
    pass.setInputCloud(current_sensor_cloud_ptr);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, 8);
    pass.filter(*cloud_filtered);

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud_filtered);
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.filter(*cloud_filtered);

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.02);

    int nr_points = (int) cloud_filtered->size ();
    while (cloud_filtered->size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);

        // Get the points associated with the planar surface
        extract.filter(*cloud_plane);
        // std::cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative(true);
        extract.filter(*cloud_f);
        *cloud_filtered = *cloud_f;
    }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(0.02); // 2cm
    ec.setMinClusterSize(500);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    sensor_msgs::PointCloud2 output_cloud;
    output_cloud.header = cloud->header;
    pcl::toROSMsg(*cloud_filtered, output_cloud);
    detected_object_cloud_pub.publish(output_cloud);

    pointcloud_cluster_detect::DetectedObjectArray detected_object_list;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->push_back((*cloud_filtered)[*pit]); //*

        double max_x = -DBL_MAX, max_y = -DBL_MAX, max_z = -DBL_MAX;
        double min_x = DBL_MAX, min_y = DBL_MAX, min_z = DBL_MAX;

        for (int i = 0; i < cloud_cluster->size(); i++)
        {
            if ((*cloud_cluster)[i].x > max_x) max_x = (*cloud_cluster)[i].x;
            if ((*cloud_cluster)[i].x < min_x) min_x = (*cloud_cluster)[i].x;
            if ((*cloud_cluster)[i].y > max_y) max_y = (*cloud_cluster)[i].y;
            if ((*cloud_cluster)[i].y < min_y) min_y = (*cloud_cluster)[i].y;
            if ((*cloud_cluster)[i].z > max_z) max_z = (*cloud_cluster)[i].z;
            if ((*cloud_cluster)[i].z < min_z) min_z = (*cloud_cluster)[i].z;
        }

        pointcloud_cluster_detect::DetectedObject object;

        object.pose.position.x = (max_x + min_x) / 2;
        object.pose.position.y = (max_y + min_y) / 2;
        object.pose.position.z = (max_z + min_z) / 2;

        object.height = max_y - min_y;
        object.width = max_x - min_x;

        double target_height = 0.2;
        if (object.height < target_height && object.height > target_height - target_height_threshold_)
        {
            for (int i = 0; i < 5;i++)
            {
                double target_width = 0.2 + 0.075 * i;
                if (object.width < target_width && object.width > target_width - target_width_threshold_)
                object.order = 5 - i;
            }
        }

        detected_object_list.objects.push_back(object);
    }
    detected_object_list_pub.publish(detected_object_list);
}

}