#include "ros/ros.h"
#include "package_20201210/Data_msg.h"

void msgCallback(const package_20201210::Data_msg::ConstPtr& msg)
{
	ROS_INFO("recieve msg = %d", msg->Data1);
	ROS_INFO("recieve msg = %d", msg->Data2);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Read_Data");
	
	ros::NodeHandle nh;
	
	ros::Subscriber sub = nh.subscribe("test", 100, msgCallback);
	
	ros::spin();
	

	
	return 0;
}
