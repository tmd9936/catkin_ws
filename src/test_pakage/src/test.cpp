#include <ros/ros.h>
#include <std_msgs/Int16.h>

#include "camera_opencv/TrafficState.h"

void msgCallback(const camera_opencv::TrafficState::ConstPtr& msg)
{
    ROS_INFO("line_state = %d", msg->line_state);
    ROS_INFO("station_area = %d", msg->station_area);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_node");

    ros::NodeHandle nh;

    ros::Subscriber test_sub = nh.subscribe("traffic_state", 1, msgCallback);

    ros::spin();

    return 0;
}