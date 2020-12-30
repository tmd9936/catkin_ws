#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
// roscd visualization_msgs
#include "visualization_msgs/MarkerArray.h"
// roscd geometry_msgs
#include "geometry_msgs/Point.h"

#include "object_detection.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>

#include <laser_geometry/laser_geometry.h>

ros::Publisher filtered_cloud_publisher;
sensor_msgs::PointCloud2 filtered_cloud_msg;

laser_geometry::LaserProjection projector_;


void msgCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    
    

    projector_.projectLaser(*scan_msg, filtered_cloud_msg);

    //ROS_INFO("filtered_cloud_msg %d", filtered_cloud_msg.height);
    filtered_cloud_publisher.publish(filtered_cloud_msg);

}

// void msgCallback(const visualization_msgs::MarkerArray::ConstPtr& marker_array_msg)
// {
//     ROS_INFO("markers");
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_data_process");

    ros::NodeHandle nh;
    // ros::NodeHandle nhp;

    // ros::Subscriber marker_array_subscriber = nh.subscribe("marker_array", 100, msgCallback);

    ros::Subscriber raw_data_sub = nh.subscribe("raw_lidar", 1, msgCallback);
    
    filtered_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 100);

    ros::spin();
}
