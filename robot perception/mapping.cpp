#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>

#include <ros/ros.h>
#include <boost/make_shared.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>

class OccupancyGridMapper
{
public:
  OccupancyGridMapper(ros::NodeHandle nh): tf_listener_(tf_buffer_)
  {
    // Subscribe to the PointCloud2 topic
    sub_ = nh.subscribe<sensor_msgs::PointCloud2>("/camera1/points", 1, &OccupancyGridMapper::cloudCallback, this);

    occupancy_grid_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/occupancy_grid", 1);
  }

private:
  ros::Subscriber sub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  nav_msgs::OccupancyGrid occupancy_grid;

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_msg)
  {
    // Convert PointCloud2 to PointCloud
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(*input_msg, pcl_cloud);

    // Lookup transform from base_link to input_msg frame
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tf_buffer_.lookupTransform("map", input_msg->header.frame_id,
                                                   input_msg->header.stamp);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      return;
    }

    // Transform PointCloud to base_link frame
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud_transformed;
    pcl_ros::transformPointCloud(pcl_cloud, pcl_cloud_transformed, transformStamped.transform);

    // Convert PointCloud2 message to PointCloud<PointXYZ> type
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(pcl_cloud_transformed);

    // Filter point cloud data using VoxelGrid filter
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(0.1, 0.1, 0.1);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_grid.filter(*filtered_cloud);

    // Segment point cloud data using EuclideanClusterExtraction algorithm
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclidean_cluster;
    euclidean_cluster.setInputCloud(filtered_cloud);
    euclidean_cluster.setClusterTolerance(0.2);
    euclidean_cluster.setMinClusterSize(100);
    euclidean_cluster.setMaxClusterSize(10000);
    std::vector<pcl::PointIndices> cluster_indices;
    euclidean_cluster.extract(cluster_indices);

    // Create occupancy grid map
    occupancy_grid.header.frame_id = "map";
    occupancy_grid.info.resolution = 0.1;
    occupancy_grid.info.width = 200;
    occupancy_grid.info.height = 200;
    occupancy_grid.info.origin.position.x = -5;
    occupancy_grid.info.origin.position.y = -5;
    occupancy_grid.data.resize(occupancy_grid.info.width * occupancy_grid.info.height, -1);

    // Iterate over each point in the point cloud data and mark the corresponding cell in the occupancy grid map as occupied
    for (auto& indices : cluster_indices) 
    {
      for (auto& index : indices.indices) 
      {
        float z = filtered_cloud->points[index].z;
        if (z > 0.23) {  // Ignore points with z-coordinate 
            int x = (filtered_cloud->points[index].x - occupancy_grid.info.origin.position.x) / occupancy_grid.info.resolution;
            int y = (filtered_cloud->points[index].y - occupancy_grid.info.origin.position.y) / occupancy_grid.info.resolution;
            int index_1d = y * occupancy_grid.info.width + x;
            if (index_1d >= 0 && index_1d < occupancy_grid.data.size()) {
                occupancy_grid.data[index_1d] = 100;
            }
        }
      }
    }

    // Publish occupancy grid map
    occupancy_grid_pub_.publish(occupancy_grid);
}

ros::NodeHandle nh_;
ros::Subscriber pcl_sub_;
ros::Publisher occupancy_grid_pub_;

};

int main(int argc, char** argv) {
  ros::init(argc, argv, "occupancy_grid_mapper");
  ros::NodeHandle nh_;
  
  OccupancyGridMapper occupancy_grid_mapper(nh_);

  ros::spin();
  return 0;
}
