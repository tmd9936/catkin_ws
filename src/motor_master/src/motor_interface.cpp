#include <ros/ros.h>
#include "motor_master/Motor.h"
#include "object_detection_gl_ros/Distance.h"
#include <std_msgs/UInt16.h>

#include <iostream>
#include <mutex>
#include <std_msgs/Header.h>

#include <camera_opencv/TrafficState.h>

// 메인 모드
#define DRIVE_MODE 0
#define BLOCK_MODE 1
#define TRAFFIC_LIGHT_MODE 2
#define STATION_MODE 3

// 주행모드
#define USUALLY_DRIVE 0
#define LEFT_OUT 1
#define RIGHT_OUT 2

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

// 정류 모드
#define FIND_STATION 0
#define GETTING_ON_STOP_MODE 1

// 신호등 색상
#define RED 0
#define GREEN 1

#define STATION_AREA_OFF 0
#define STATION_AREA_ON 4

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

// 라이다 관련 변수
int front_dist = 0;
int left_dist = 0;

// opencv 관령 변수
int line_state = 0;
int left_line_count = 0;
int right_line_count = 0;
int station_area = 0;
int traffic_color = GREEN;

// 모드
int flag = 0;
// 서브모드
int sub_flag = 0;

// 정지할 시간(초)
float duration_sec = 0;
// 기본 모터 속력
int basic_motor_pwm = 15;

int boundary = 5;

// PD제어 관련 변수
ros::Time now;
ros::Time last_time(0.);
int error;
int lastError = 0;
double dt = 0.0;

double kp = 0.4;
double kd_val = 0.65;
double kd = kp * 0.65;

double derivative = 0.0;
double proportional = 0.0;
int PD = 0; 

int pre_servo_val = 15;
int pre_left_line_count = 1;
int pre_right_line_count = 1;

bool start_drive = false;

int people_in_station = 0;

int pre_left_distance = 0;

// launch 파일 파라미터
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

	nh_priv->param("boundary", boundary, boundary);

	nh_priv->param("kp", kp, kp);
	nh_priv->param("kd_val", kd_val, kd_val);

	pre_servo_val = basic_motor_pwm;

}

void lidarCallback(const object_detection_gl_ros::Distance::ConstPtr &msg)
{
	front_dist = msg->front_dist;
	left_dist = msg->left_dist;
	//ROS_INFO("front_dist    =     %d cm", front_dist);
	//ROS_INFO("left_dist     =     %d cm", left_dist);
}

void cameraCallback(const camera_opencv::TrafficState::ConstPtr &msg)
{
	line_state = msg->line_state;
	station_area = msg->station_area;
	traffic_color = msg->traffic_color;
	left_line_count = msg->left_line_count;
	right_line_count = msg->right_line_count;

}

void gettingOnStopCallback(const std_msgs::UInt16::ConstPtr &msg)
{
	people_in_station = msg->data;
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

	camera_opencv::TrafficState traffic_state_msg;

	ros::Subscriber distance_sub = nh.subscribe("distances", 1, lidarCallback);
	ros::Subscriber camera_opencv_sub = nh.subscribe("traffic_state", 1, cameraCallback);
	ros::Subscriber getting_on_stop_sub = nh.subscribe("getting_on_and_off_state", 1, gettingOnStopCallback);

	ros::AsyncSpinner spinner(1); //spin
	spinner.start();
	ros::Rate loop_rate(50);

	initParams(&nh_priv);

	kp = kd * kd_val;

	while (ros::ok())
	{
		//std::lock_guard<std::mutex> lock(m);

		now = ros::Time::now();

		if (!start_drive)
		{
			start_drive = true;
			pre_left_line_count = left_line_count;
			pre_right_line_count = pre_left_line_count;
		}

		// 주행모드
		if (flag == DRIVE_MODE)
		{
			motor_msg.data = basic_motor_pwm;
			duration_sec = 0;
			if (traffic_color == RED)
			{
				flag = TRAFFIC_LIGHT_MODE;
				motor_msg.data = 0;
			}
			// 앞의 장애물에 멈추고 장애물 감지 모드로 변경
			else if (front_dist < 45 && front_dist > 0 && traffic_color != RED && traffic_color != GREEN)
			{
				flag = BLOCK_MODE;
				sub_flag = DIST_INIT_MODE;
				motor_msg.data = 0;
			}
			else if(station_area == STATION_AREA_ON)
			{
				flag = STATION_MODE;
				sub_flag = FIND_STATION;
				duration_sec = 2.4;
			}
			else
			{
				// opencv에서 차선 degree 받아서 서보모터 degree 바꾸기
				// 이상치가 오면 전의 value로 대체
				
				// boundary == 경계
				dt = now.toSec() - last_time.toSec();
				error = abs(line_state);

				derivative = kd * (error - lastError) / dt;
				proportional = kp * error;
				PD = int(derivative + proportional);
				if(line_state < 0)
					PD *= (-1);
				//ROS_INFO("PD %d,  dt %f", PD, dt);
				lastError = error;

				if (sub_flag == USUALLY_DRIVE)
				{
					// 왼쪽 탈선
					if ( (pre_left_line_count == 1 && pre_right_line_count == 0 && 
						left_line_count == 0 && right_line_count == 1) || (
						pre_left_line_count == 1 && pre_right_line_count == 0 && 
						left_line_count == 0 && right_line_count == 0))
					{
						sub_flag = LEFT_OUT;
						// 오른쪽으로 복귀
						servo_msg.data = 140;
						//ROS_INFO("left out");
					}
					// 오른쪽 탈선
					else if ( pre_left_line_count == 0 && pre_right_line_count == 1 && 
						left_line_count == 1 && right_line_count == 0 || (
						pre_left_line_count == 0 && pre_right_line_count == 1 && 
						left_line_count == 0 && right_line_count == 0))
					{
						sub_flag = RIGHT_OUT;
						// 왼쪽으로 복귀
						servo_msg.data = 10;
						//ROS_INFO("right out");
					}
					else
					{
						if (PD <= boundary && PD >= boundary*(-1))
						{
							servo_msg.data = 75;
						}
						else
						{
							if (75 + PD > 150)
							{
								servo_msg.data = 150;
							}
							else if(75 + PD < 0)
							{
								servo_msg.data = 0;
							}
							else
							{
								servo_msg.data = 75 + PD;
							}
						}
					}
					pre_servo_val = servo_msg.data;
					pre_left_line_count = left_line_count;
					pre_right_line_count = right_line_count;
				}
				else if (sub_flag == LEFT_OUT)
				{
					if(right_line_count == 1)
					{
						sub_flag = USUALLY_DRIVE;
						servo_msg.data = 75;
						if (front_dist > 60)
							duration_sec = 0.1;
						else
							duration_sec = 0.0;
					}
					else 
					{
						servo_msg.data = 140;
					}
				}
				else if (sub_flag == RIGHT_OUT)
				{
					if(left_line_count == 1)
					{
						sub_flag = USUALLY_DRIVE;
						servo_msg.data = 75;
						if (front_dist > 60)
							duration_sec = 0.1;
						else
							duration_sec = 0.0;
					}
					else 
					{
						servo_msg.data = 10;
					}
				}
			}
		}
		// 장애물 만날경우
		else if (flag == BLOCK_MODE)
		{
			// 멈추기
			if (sub_flag == DIST_INIT_MODE)
			{
				motor_msg.data = 0;
				sub_flag = RIGHT_TURN_MODE;
				duration_sec = 1.5;
			}
			//오른쪽으로 턴
			else if (sub_flag == RIGHT_TURN_MODE)
			{
				motor_msg.data = 0;
				servo_msg.data = 150;
				duration_sec = right_turn_mode_duration;
				sub_flag = RT_AND_GO_MODE;
			}
			// 오른쪽으로 턴상태에서 움직이기
			else if (sub_flag == RT_AND_GO_MODE)
			{
				motor_msg.data = basic_motor_pwm;
				duration_sec = rt_and_go_mode_duration;
				sub_flag = BLOCK_LEFT_TURN_MODE;
			}
			// 왼쪽으로 턴
			else if (sub_flag == BLOCK_LEFT_TURN_MODE)
			{
				motor_msg.data = 0;
				servo_msg.data = 0;
				duration_sec = block_left_turn_mode_duration;
				sub_flag = BLOCK_LT_AND_GO_MODE;
			}
			// 왼쪾으로 턴 상태에서 움직이기
			else if (sub_flag == BLOCK_LT_AND_GO_MODE)
			{
				motor_msg.data = basic_motor_pwm;
				duration_sec = block_lt_and_go_mode_duration;
				sub_flag = BLOCK_PASS_MODE;
			}
			// 장애물 지나가기
			else if (sub_flag == BLOCK_PASS_MODE)
			{
				servo_msg.data = 75;
				// 라이다에서 받은 맨왼쪽의 값이 55이하면 계속진행
				if (left_dist > 0 && left_dist <= 55)
				{
					if (left_dist > 32)
					{
						servo_msg.data = 25;
					}
					else if(left_dist < 27)
					{
						servo_msg.data = 120;
					}
					else
					{
						servo_msg.data = 75;
					}
					motor_msg.data = basic_motor_pwm;
					duration_sec = 0;
					pre_left_distance = left_dist;
				}
				// 55이상이면 장애물이 없다고 판단.
				else if (left_dist > 55)
				{
					motor_msg.data = 0;
					duration_sec = block_pass_mode_duration;
					sub_flag = LINE_LEFT_TURN_MODE;
				}
				else
				{
				}
			}
			// 장애물 넘기위해 왼쪽으로 턴
			else if (sub_flag == LINE_LEFT_TURN_MODE)
			{
				motor_msg.data = 0;
				servo_msg.data = 0;
				duration_sec = line_left_turn_mode_duration;
				sub_flag = LINE_LT_AND_GO_MODE;
			}
			// 왼쪽으로 턴하고 움직이기
			else if (sub_flag == LINE_LT_AND_GO_MODE)
			{
				motor_msg.data = basic_motor_pwm;
				duration_sec = line_lt_and_go_mode_duration * pre_left_distance;
				sub_flag = LINE_CHECK_INIT_MODE;
			}
			//차선 보일때까진 움직이기
			else if (sub_flag == LINE_CHECK_INIT_MODE)
			{
				motor_msg.data = basic_motor_pwm;
				servo_msg.data = 150;
				duration_sec = line_check_init_mode_duration;
				sub_flag = LINE_CHECK_MODE;
			}
			else if (sub_flag == LINE_CHECK_MODE)
			{
				// 차선이 2개 보이면 빠져나가기
				if(left_line_count == 1 && right_line_count == 1) 
				{
					// motor_msg.data = 0;
					// servo_msg.data = 75;
					// duration_sec = 1;
					sub_flag = USUALLY_DRIVE;
					flag = DRIVE_MODE;
				}
			}
		}
		// 빨간불을 만날 경우 대기 모드
		else if (flag == TRAFFIC_LIGHT_MODE)
		{
			motor_msg.data = 0;
			if (traffic_color == GREEN)
			{
				flag = DRIVE_MODE;
				sub_flag = USUALLY_DRIVE;
			}
			else {}

		}
		else if (flag == STATION_MODE)
		{
			if (sub_flag == FIND_STATION)
			{
				duration_sec = 1.5;
				motor_msg.data = 0;
				sub_flag = GETTING_ON_STOP_MODE;
			}
			else if (sub_flag == GETTING_ON_STOP_MODE)
			{
				// 초음파 받아서 사람있는지 판별
				// 있으면 계속 대기
				if (people_in_station == 1)
					motor_msg.data = 0;
				// 없으면 조금 대기했다가 출발
				else 
				{
					duration_sec = 0.5;
					flag = DRIVE_MODE;
					sub_flag = USUALLY_DRIVE;
				}
			}
		}
		else {}

		ROS_INFO("flag = %d,  sub_flag = %d, sec = %f ", flag, sub_flag, ros::Time::now().toSec() - now.toSec());

		motor_val.publish(motor_msg);
		servo_val.publish(servo_msg);
		front_dist = 0;
		ros::Duration(duration_sec).sleep();

		last_time = ros::Time::now();
		// ROS_INFO("last_time %f", last_time.toSec());
		ros::spinOnce();
		loop_rate.sleep();
	}

	spinner.stop();

	//ros::spin();

	return 0;
}
