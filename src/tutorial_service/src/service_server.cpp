#include "ros/ros.h"
#include "tutorial_service/SrvTutorial.h" // 서비스 파일 헤더 (자동생성됨)

// 서비스 요청이 있을 때, 아래 처리를 수행한다.
// 서비스 요청은 req, 서비스 응답은 res로 설정함.
bool calculation(tutorial_service::SrvTutorial::Request &req, 
				 tutorial_service::SrvTutorial::Response &res)
{
	res.result = req.a + req.b;
	ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
	ROS_INFO("sendig back response:%ld", (long int)res.result);
	
	return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "service_server"); // 노드명 초기화
	ros::NodeHandle nh; // 노드핸들 선언
	
	// 서비스 서버 선언, "tutorial_servce"패키지의 "SrvTutorial"서비스 파일을 이용한 서비스 서버 'server'를 선언한다
	// 서비스명은 '   '이며 서비스 요청이 있을 때 calculation함수를 실행 하라고 설정.
	
	ros::ServiceServer server = nh.advertiseService("ros_tutorial_srv", calculation);
	
	ROS_INFO("ready service server!!");
	
	ros::spin();
	
	return 0;

}
