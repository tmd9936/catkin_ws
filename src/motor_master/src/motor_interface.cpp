#include <ros/ros.h>
#include "motor_master/Motor.h"
#include "object_detection_gl_ros/Distance.h"
#include <std_msgs/UInt16.h>

#include <iostream> 
#include <mutex>
#include <std_msgs/Header.h>

// 메인 모드
#define DRIVE_MODE 0
#define BLOCK_MODE 1
#define TRAFFIC_LIGHT_MODE 2
#define STATION_MODE 3

// BLOCK_MODE
#define DIST_INIT_MODE 0
#define RIGHT_TURN_MODD 1
#define RT_AND_GO_MODE 2
#define BLOCK_LEFT_TURN_MODE 3
#define BLOCK_LT_AND_GO_MODE 4
#define BLOCK_PASS_MODE 5
#define LINE_LEFT_TURN_MODE 6
#define LINE_LT_AND_GO_MODE 7
#define LINE_CHECK_MODE 8

// TRAFFIC_LIGHT_MODE

// STATION_MODE

int front_dist = 0;
int left_dist = 0;

int flag = 0;
int sub_flag = 0;

float duration_sec = 0;

std::mutex  m; 

void LidarCallback(const object_detection_gl_ros::Distance::ConstPtr &msg) {
	front_dist = msg->front_dist;
	left_dist = msg->left_dist;
	//ROS_INFO("front_distttt = %d cm", front_dist);
	//ROS_INFO("left_distttt = %d cm", left_dist);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "motor_interface");
	ros::NodeHandle nh;

	ros::Publisher station_stop_state = nh.advertise<std_msgs::UInt16>("station_stop_state", 1);
	ros::Publisher motor_val = nh.advertise<std_msgs::UInt16>("motor_val", 1);
	ros::Publisher servo_val = nh.advertise<std_msgs::UInt16>("servo_val", 1);

	std_msgs::UInt16 motor_msg;
	std_msgs::UInt16 servo_msg;

	ros::Subscriber distance_sub = nh.subscribe("distances", 1, LidarCallback);	

	ros::AsyncSpinner spinner(1);  //spin
    spinner.start();
    ros::Rate loop_rate(20);	

	ros::Time begin_s;

	while(ros::ok())
	{
		std::lock_guard<std::mutex> lock(m);

		begin_s = ros::Time::now();
		if(flag == DRIVE_MODE){
			duration_sec = 0;
			if(front_dist < 40 && front_dist > 0){
				flag = BLOCK_MODE;
				sub_flag = 0;
			}
			//else if(traffic_light = 1)
			//{
			//	flag = TRAFFIC_LIGHT_MODE;
			//}
			//else if(station_on == 1)
			//{
			//	flag = STATION_MODE;
			//}
			else
			{
				// 서보모터 각도 받아서 바꾸기
				servo_msg.data = 75;
				motor_msg.data = 30;
			}
		}

		if(flag == BLOCK_MODE)
		{
			if(sub_flag == DIST_INIT_MODE)
			{
				if(front_dist > 30 && front_dist != 0)
				{
					motor_msg.data = 30;
					duration_sec = 0;
				}
				else
				{
					motor_msg.data = 0;
					sub_flag = RIGHT_TURN_MODD;
					duration_sec = 0.1;
				}
			}
			else if(sub_flag == RIGHT_TURN_MODD)
			{
				motor_msg.data = 0;
				servo_msg.data = 120;
				duration_sec = 1;
				sub_flag = RT_AND_GO_MODE;
				
			}
			else if(sub_flag == RT_AND_GO_MODE)
			{
				motor_msg.data = 30;
				duration_sec = 0.5;
				sub_flag = BLOCK_LEFT_TURN_MODE;
			}
			else if(sub_flag == BLOCK_LEFT_TURN_MODE)
			{
				motor_msg.data = 0;
				servo_msg.data = 30;
				duration_sec = 1;
				sub_flag = BLOCK_LT_AND_GO_MODE;
			}
			else if(sub_flag == BLOCK_LT_AND_GO_MODE)
			{
				motor_msg.data = 30;
				duration_sec = 0.5;
				sub_flag = BLOCK_PASS_MODE;
			}
			else if(sub_flag == BLOCK_PASS_MODE)
			{
				servo_msg.data = 75;
				if(front_dist > 0 && front_dist < 15)
				{
					motor_msg.data = 15;
					duration_sec = 0;
				}
				else
				{
					motor_msg.data = 0;
					duration_sec = 0.5;
					sub_flag = LINE_LEFT_TURN_MODE;
				}
			}
			else if(sub_flag == LINE_LEFT_TURN_MODE)
			{
				motor_msg.data = 0;
				servo_msg.data = 30;
				duration_sec = 1;
				sub_flag = LINE_LT_AND_GO_MODE;
			}
			else if(sub_flag == LINE_LT_AND_GO_MODE)
			{
				motor_msg.data = 30;
				duration_sec = 0.5;
				sub_flag = LINE_CHECK_MODE;
			}
			else if(sub_flag == LINE_CHECK_MODE)
			{
				motor_msg.data = 15;
				servo_msg.data = 120;

				// if(line_state == 0) // opencv에서 받은 라인의 각도가 0이 되면 빠져나가기
				//{
				ros::Duration(3).sleep();
				servo_msg.data = 75;
				ros::Duration(1).sleep();
				sub_flag = 0;
				flag = DRIVE_MODE;
				//}

			}
		
		}
		ROS_INFO("flag = %d,  sub_flag = %d, sec = %f ", flag, sub_flag, ros::Time::now() - begin_s);

		motor_val.publish(motor_msg);
		servo_val.publish(servo_msg);
		front_dist = 0; 
		ros::Duration(duration_sec).sleep();
		
		ros::spinOnce();
	    loop_rate.sleep();
		
	
	}

	spinner.stop();

	//ros::spin();

	return 0;
}



