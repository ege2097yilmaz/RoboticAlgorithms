#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_ros/transforms.h>

class PointCloudHandler
{
public:
  PointCloudHandler(ros::NodeHandle& nh) : nh_(nh), tf_listener_(tf_buffer_)
    {
        // Subscribe to PointCloud2 topic
        sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/points2", 1, &PointCloudHandler::pointcloudCallback, this);

        // Initialize occupancy grid map message
        map_.header.frame_id = "base_link";
        map_.header.stamp = ros::Time::now();
        map_.info.resolution = 0.1;  
        map_.info.width = 40;  
        map_.info.height = 40;  
        map_.info.origin.position.x = -2.0; 
        map_.info.origin.position.y = -2.0;
        map_.data.resize(map_.info.width * map_.info.height);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    nav_msgs::OccupancyGrid map_;
    std::vector<int8_t> map_data_;
    float decay_rate_ = 0.5;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        // Convert ROS PointCloud2 message to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(*msg, pcl_cloud);

        // Get the transform from camera frame to base link
        try {
            geometry_msgs::TransformStamped transformStamped = tf_buffer_.lookupTransform("base_link", msg->header.frame_id,
                msg->header.stamp, ros::Duration(1.0));
                
            // Transform the point cloud to base link frame
            pcl_ros::transformPointCloud(pcl_cloud, pcl_cloud, transformStamped.transform);
        }
        catch (tf2::TransformException& ex) {
            ROS_WARN("Could not transform point cloud: %s", ex.what());
        }

        // Update occupancy grid map with pointcloud data
        for (pcl::PointCloud<pcl::PointXYZ>::iterator it = pcl_cloud.begin(); it != pcl_cloud.end(); ++it)
        {   
            float x = it->x;
            float y = it->y;
            float z = it->z;

            // Convert x, y coordinates to occupancy grid map indices
            int i = (x - map_.info.origin.position.x) / map_.info.resolution;
            int j = (y - map_.info.origin.position.y) / map_.info.resolution;

            // Check if i and j indices are within range of the data vector
            if (i >= 0 && i < map_.info.width && j >= 0 && j < map_.info.height)
            {
                // Check if z value is greater than 0.1 meters
                if (z > 0.02 && z < 0.5)
                {
                    // Mark corresponding cell in occupancy grid map as occupied
                    map_.data[j * map_.info.width + i] = 100;
                    inflate_map(map_, j * map_.info.width + i);
                }
                else
                {
                    // Mark corresponding cell in occupancy grid map as free
                    map_.data[j * map_.info.width + i] = 0;
                }
            }
        }

        // decay rate for celaring costmap
        applyDecay(map_);
        
        // Publish occupancy grid map message
        map_.header.stamp = ros::Time::now();
        map_pub_.publish(map_);
    }

    void applyDecay(nav_msgs::OccupancyGrid& mapdata)
    {
        // Apply decay rate to mep_data list
        for (int i = 0; i < mapdata.data.size(); i++)
        {
        mapdata.data[i] *= decay_rate_;
        }
    }

    void inflate_map(nav_msgs::OccupancyGrid& map, int index)
    {
        // Define the inflation radius and kernel size in cells
        int radius = 3;
        int kernel_size = 2 * radius + 1;

        // Create a circular mask for inflation
        std::vector<std::vector<int>> mask(kernel_size, std::vector<int>(kernel_size, 0));
        for (int i = 0; i < kernel_size; i++) 
        {
            for (int j = 0; j < kernel_size; j++) 
            {
                if (sqrt((i - radius) * (i - radius) + (j - radius) * (j - radius)) <= radius) {
                    mask[i][j] = 1;
                }
            }
        }

        // Get the row and column indices of the occupied cell
        int row = index / map.info.width;
        int col = index % map.info.width;

        // Inflate the cells around the occupied cell
        for (int i = -radius; i <= radius; i++) 
        {
            for (int j = -radius; j <= radius; j++) 
            {
                int r = row + i;
                int c = col + j;
                if (r >= 0 && r < map.info.height && c >= 0 && c < map.info.width) {
                    int idx = r * map.info.width + c;
                    if (map.data[idx] != 100) {  // check if the cell is not already occupied
                        map.data[idx] += mask[i + radius][j + radius];
                        if (map.data[idx] > 100) {
                            map.data[idx] = 100;
                        }
                    }
                }
            }
        }
    }
    
    ros::Publisher map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/pointcloudcostmap", 1);
};

int main(int argc, char** argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "pointcloud_node");
    ROS_INFO("pointcloud costmap is starting ...");

    ros::NodeHandle nh;

    // Create instance of PointCloudHandler class
    PointCloudHandler pc_handler(nh);

    // Spin ROS event loop
    ros::spin();

    return 0;
}
