#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>

class LidarTransformNode
{
public:
    LidarTransformNode()
    {
        ROS_INFO("scan merger is starting...");

        // advertise publisher
        scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan_combined", 10);

        // subscribe to LIDAR data topic
        lidar_sub_ = nh_.subscribe("/scan_1", 10, &LidarTransformNode::lidarCallback, this);
        lidar_sub_2_ = nh_.subscribe("/scan_2", 10, &LidarTransformNode::lidarCallback, this);
    }

    ~LidarTransformNode() {}

    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
    {
        // create a tf listener to get the transform from LIDAR frame to base_link frame
        tf::StampedTransform transform;

        try {
            tf_listener_.waitForTransform("base_link", scan_msg->header.frame_id, scan_msg->header.stamp, ros::Duration(1.0));
            tf_listener_.lookupTransform("base_link", scan_msg->header.frame_id, scan_msg->header.stamp, transform);
        } catch (tf::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            return;
        }

        // convert sensor_msgs::LaserScan to pcl pointcloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (int i = 0; i < scan_msg->ranges.size(); ++i) 
        {
            float range = scan_msg->ranges[i];
            float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
            pcl::PointXYZ point;
            point.x = range * std::cos(angle);
            point.y = range * std::sin(angle);
            point.z = 0.0;
            cloud->push_back(point);
        }

        // transform LIDAR data to base_link frame
        pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
        pcl_ros::transformPointCloud(*cloud, transformed_cloud, transform);

        // add the transformed cloud to the combined point cloud
        combined_cloud_ += transformed_cloud;

        // check if it's time to reset the combined point cloud
        // clear the obstacle and updating combined pointcloud
        if (combined_cloud_.width >= reset_threshold_) {
            combined_cloud_.clear();
            combined_cloud_.width = 0;
        }

        // publish the combined point cloud
        sensor_msgs::PointCloud2 combined_cloud_msg;
        pcl::toROSMsg(combined_cloud_, combined_cloud_msg);

        // convert combined point cloud to laser scan
        sensor_msgs::LaserScan scan_msg_combined;
        scan_msg_combined.header.frame_id = "base_link";
        scan_msg_combined.header.stamp = combined_cloud_msg.header.stamp;
        scan_msg_combined.angle_min = -M_PI;
        scan_msg_combined.angle_max = M_PI;
        scan_msg_combined.angle_increment = 2.0 * M_PI / num_scan_points_;
        scan_msg_combined.time_increment = 0.0;
        scan_msg_combined.scan_time = 0.1;
        scan_msg_combined.range_min = 0.0;
        scan_msg_combined.range_max = 100.0;

        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(combined_cloud_msg, pcl_cloud);

        for (int i = 0; i < num_scan_points_; ++i) 
        {
            float angle = scan_msg_combined.angle_min + i * scan_msg_combined.angle_increment;
            float range = std::numeric_limits<float>::infinity();

            for (auto point : pcl_cloud) 
            {
                float point_angle = std::atan2(point.y, point.x);
                if (std::abs(point_angle - angle) < scan_msg_combined.angle_increment / 2.0) {
                    float point_range = std::sqrt(point.x * point.x + point.y * point.y);
                    if (point_range < range) {
                        range = point_range;
                    }
                }
            }

            scan_msg_combined.ranges.push_back(range);
        }

        // publish the combined laser scan
        scan_pub_.publish(scan_msg_combined);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher transformed_pub_;
    ros::Publisher scan_pub_;
    ros::Subscriber lidar_sub_;
    ros::Subscriber lidar_sub_2_;
    tf::TransformListener tf_listener_;
    pcl::PointCloud<pcl::PointXYZ> combined_cloud_;
    int reset_threshold_ = 1500; // number of points before resetting the combined point cloud
    int num_scan_points_ = 360; // number of scan points in the output laser scan
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scan_merger");

    // lidar merger
    LidarTransformNode node;

    ros::spin();

    return 0;
}