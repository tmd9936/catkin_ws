#include "ros/ros.h" // ROS기본 헤더 파일
#include "beginner_tutorial/MsgTutorial.h" // MsgTutorial메세지 파일 헤더 (빌드 후 자동생성)

// 메세지 콜백함수로서, 아래서 설정한 "ros_tutorial_msg"란 이름의 토픽, 메세지를 수신하였을때 동작하는 함수
// 입력 메세지로는 beginner_tutorial패키지의 MsgTutorial메세지를 받도록 되어있음
void msgCallback(const beginner_tutorial::MsgTutorial::ConstPtr& msg)
{
	ROS_INFO("recievemsg=%d", msg->stamp.sec);
	ROS_INFO("recievemsg=%d", msg->stamp.nsec);
	ROS_INFO("recievemsg=%d", msg->data);
		
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "topic_subscriber"); // 노드명 초기화
	ros::NodeHandle nh; // ROS 시스템과 통신을위한 노드 핸들 선언
	
	// 서브스크라이버 선언, "beginner_tutorial"패키기의 MsgTutorial 메세지 파일을 이용한 서브스크라이버
	// rostutorial_sub을 작성한다.
	// topic명은 ros_tutorial_msg이며 Queue 사이즈를 100개로 설정한다.
	ros::Subscriber ros_tutorial_sub=nh.subscribe("beginner_tutorial", 100, msgCallback);
	
	// 콜백함수 호출을 위한 함수로 메세지가 수신되기를 대기, 수신되었을 경우 콜백함수를 실행
	ros::spin();
	return 0;
}
