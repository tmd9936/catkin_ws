#include "ros/ros.h"
#include "package_20201210/Data_msg.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Send_Data");
	ros::NodeHandle nh;
	
	ros::Publisher msg_pub = nh.advertise<package_20201210::Data_msg>("test", 100);
	
	ros::Rate loop_rate(10);
	
	package_20201210::Data_msg msg;
	int count = 0;
	
	while (ros::ok())
	{
		if (count * count > 4294967290)
		{
			count = 0;
		}
		
		msg.Data1 = count;
		msg.Data2 = count * count;
		
		ROS_INFO("send msg = %d", msg.Data1);
		ROS_INFO("send msg = %d", msg.Data2);
		
		msg_pub.publish(msg);
		
		loop_rate.sleep();
		
		count++;
		
		
	}
	
	
	return 0;
}
