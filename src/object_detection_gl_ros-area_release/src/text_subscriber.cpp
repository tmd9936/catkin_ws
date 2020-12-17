#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
// roscd visualization_msgs
#include "visualization_msgs/MarkerArray.h"
// roscd geometry_msgs
#include "geometry_msgs/Point.h"

#include "object_detection.h"

// void msgCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
// {
//     ROS_INFO("range_max=%lf", scan_msg->range_max);
// }

// python 퍼블리셔 생성해서 받기

void msgCallback(const visualization_msgs::MarkerArray::ConstPtr& marker_array_msg)
{
    ROS_INFO("marker_array_msg->markers[0].color.r = %f", marker_array_msg->markers[0].color.r);
    ROS_INFO("marker_array_msg->markers[0].color.g = %f", marker_array_msg->markers[0].color.g);
    ROS_INFO("marker_array_msg->markers[0].color.b = %f", marker_array_msg->markers[0].color.b);

    ROS_INFO("markers");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "text_subscriber");
    ros::NodeHandle nh;

    // ros::Subscriber text_subscriber = nh.subscribe("scan", 100, msgCallback);

    ros::Subscriber text_subscriber = nh.subscribe("marker_array", 100, msgCallback);


    ros::spin();

    return 0;

    
}