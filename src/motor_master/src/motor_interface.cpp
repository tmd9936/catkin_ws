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
#define RIGHT_TURN_MODE 1
#define RT_AND_GO_MODE 2
#define BLOCK_LEFT_TURN_MODE 3
#define BLOCK_LT_AND_GO_MODE 4
#define BLOCK_PASS_MODE 5
#define LINE_LEFT_TURN_MODE 6
#define LINE_LT_AND_GO_MODE 7
#define LINE_CHECK_INIT_MODE 8
#define LINE_CHECK_MODE 9

double right_turn_mode_duration;
double rt_and_go_mode_duration;
double block_left_turn_mode_duration;
double block_lt_and_go_mode_duration;
double block_pass_mode_duration;
double line_left_turn_mode_duration;
double line_lt_and_go_mode_duration;
double line_check_init_mode_duration;

// TRAFFIC_LIGHT_MODE

// STATION_MODE

int front_dist = 0;
int left_dist = 0;

int flag = 0;
int sub_flag = 0;

float duration_sec = 0;

int basic_motor_pwm = 30;

std::mutex m; 

void initParams(ros::NodeHandle *nh_priv)
{
	nh_priv->param("right_turn_mode_duration", right_turn_mode_duration, right_turn_mode_duration);
	nh_priv->param("rt_and_go_mode_duration", rt_and_go_mode_duration, rt_and_go_mode_duration);
	nh_priv->param("block_left_turn_mode_duration", block_left_turn_mode_duration, block_left_turn_mode_duration);
	nh_priv->param("block_lt_and_go_mode_duration", block_lt_and_go_mode_duration, block_lt_and_go_mode_duration);
	nh_priv->param("block_pass_mode_duration", block_pass_mode_duration, block_pass_mode_duration);
	nh_priv->param("line_left_turn_mode_duration", line_left_turn_mode_duration, line_left_turn_mode_duration);
	nh_priv->param("line_lt_and_go_mode_duration", line_lt_and_go_mode_duration, line_lt_and_go_mode_duration);
	nh_priv->param("line_check_init_mode_duration", line_check_init_mode_duration, line_check_init_mode_duration);
	
	nh_priv->param("basic_motor_pwm", basic_motor_pwm, basic_motor_pwm);

}

void LidarCallback(const object_detection_gl_ros::Distance::ConstPtr &msg) 
{
	front_dist = msg->front_dist;
	left_dist = msg->left_dist;
	//ROS_INFO("front_distttt = %d cm", front_dist);
	//ROS_INFO("left_distttt = %d cm", left_dist);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "motor_interface");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv{"~"};

	ros::Publisher station_stop_state = nh.advertise<std_msgs::UInt16>("station_stop_state", 1);
	ros::Publisher motor_val = nh.advertise<std_msgs::UInt16>("motor_val", 1);
	ros::Publisher servo_val = nh.advertise<std_msgs::UInt16>("servo_val", 1);

	std_msgs::UInt16 motor_msg;
	std_msgs::UInt16 servo_msg;

	ros::Subscriber distance_sub = nh.subscribe("distances", 1, LidarCallback);	

	ros::AsyncSpinner spinner(1);  //spin
    spinner.start();
    ros::Rate loop_rate(50);	

	ros::Time begin_s;

	initParams(&nh_priv);

	while(ros::ok())
	{
		std::lock_guard<std::mutex> lock(m);

		begin_s = ros::Time::now();
		if(flag == DRIVE_MODE){
			duration_sec = 0;
			if(front_dist < 30 && front_dist > 0){
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
				// opencv에서 차선 각도 받아서 서보모터 각도 바꾸기
				servo_msg.data = 75;
				motor_msg.data = basic_motor_pwm;
			}
		}

		if(flag == BLOCK_MODE)
		{
			if(sub_flag == DIST_INIT_MODE)
			{
				motor_msg.data = 0;
				sub_flag = RIGHT_TURN_MODE;
				duration_sec = 1.5;
			}
			else if(sub_flag == RIGHT_TURN_MODE)
			{
				motor_msg.data = 0;
				servo_msg.data = 120;
				duration_sec = right_turn_mode_duration;
				sub_flag = RT_AND_GO_MODE;
				
			}
			else if(sub_flag == RT_AND_GO_MODE)
			{
				motor_msg.data = basic_motor_pwm;
				duration_sec = rt_and_go_mode_duration;
				sub_flag = BLOCK_LEFT_TURN_MODE;
			}
			else if(sub_flag == BLOCK_LEFT_TURN_MODE)
			{
				motor_msg.data = 0;
				servo_msg.data = 30;
				duration_sec = block_left_turn_mode_duration;
				sub_flag = BLOCK_LT_AND_GO_MODE;
			}
			else if(sub_flag == BLOCK_LT_AND_GO_MODE)
			{
				motor_msg.data = basic_motor_pwm;
				duration_sec = block_lt_and_go_mode_duration;
				sub_flag = BLOCK_PASS_MODE;
			}
			else if(sub_flag == BLOCK_PASS_MODE)
			{
				servo_msg.data = 75;
				if(left_dist > 0 && left_dist <= 15)
				{
					motor_msg.data = basic_motor_pwm;
					duration_sec = 0;
				}
				else if(left_dist > 15)
				{
					motor_msg.data = 0;
					duration_sec = block_pass_mode_duration;
					sub_flag = LINE_LEFT_TURN_MODE;
				}
				else {}
			}
			else if(sub_flag == LINE_LEFT_TURN_MODE)
			{
				motor_msg.data = 0;
				servo_msg.data = 30;
				duration_sec = line_left_turn_mode_duration;
				sub_flag = LINE_LT_AND_GO_MODE;
			}
			else if(sub_flag == LINE_LT_AND_GO_MODE)
			{
				motor_msg.data = basic_motor_pwm;
				duration_sec = line_lt_and_go_mode_duration;
				sub_flag = LINE_CHECK_INIT_MODE;
			}
			else if(sub_flag == LINE_CHECK_INIT_MODE)
			{
				motor_msg.data = basic_motor_pwm;
				servo_msg.data = 120;
				duration_sec = line_check_init_mode_duration; //차선 보일때까진 움직이기
				sub_flag = LINE_CHECK_MODE
			}
			else if(sub_flag == LINE_CHECK_MODE)
			{				
				// if(line_state == 0) // opencv에서 받은 라인의 각도가 0이 되면 빠져나가기
				//{
					motor_msg.data = 0;
					servo_msg.data = 75;
					duration_sec = 1;
					sub_flag = 0;
					flag = DRIVE_MODE;
				//}
			}
		
		}
		ROS_INFO("%f", block_left_turn_mode_duration);
		ROS_INFO("flag = %d,  sub_flag = %d, sec = %f ", flag, sub_flag, ros::Time::now() - begin_s);

		motor_val.publish(motor_msg);
		servo_val.publish(servo_msg);
		front_dist = 0; 
		ros::Duration(duration_sec).sleep();
		ROS_INFO("left distance %d", left_dist);

		ros::spinOnce();
	    loop_rate.sleep();
		
	
	}

	spinner.stop();

	//ros::spin();

	return 0;
}



