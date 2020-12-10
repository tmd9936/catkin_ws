#include "ros/ros.h"
#include "tutorial_service/SrvTutorial.h"
#include <cstdlib> // atoll 함수 사용을 위한 라이브러리

int main(int argc, char** argv)
{
	ros::init(argc, argv, "service_client"); // 노드명 초기화
	
	if (argc != 3) // 입력값 오류 처리
	{
		ROS_INFO("cmd : rosrun tutorial_service service_client arg0 arg1");
		ROS_INFO("arg0 : double number, arg1 : double number");
		return 1;
	}
	
	ros::NodeHandle nh; // ROS 시스템과 통신을 위한 노드핸들 선언
	
	// 서비스 클라이언트 선언, "tutorial_service"패키지의  "SrvTutorial"서비스 파일을 이용한 
	// 서비스 클라이언트 'client'를 선언한다.
	// 서비스명은 'ros_tutorial_srv'이다.
	ros::ServiceClient client = nh.serviceClient<tutorial_service::SrvTutorial>("ros_tutorial_srv");
	 
	tutorial_service::SrvTutorial srv; // srv라는 이름으로 SrvTutorial서비스 파일을 이용하는 서비스 선언
	
	// 서비스 요청 값으로 노드가 실행될 때 입력으로 사용된 매개변수를 각각의 a,b에 저장
	srv.request.a = atoll(argv[1]);
	srv.request.b = atoll(argv[2]);
	
	if(client.call(srv))
	{
		ROS_INFO("send srv, srv.Request.a and b: %ld, %ld", (long int)srv.request.a, (long int)srv.request.b);
		ROS_INFO("send srv, srv.Request.result : %ld", (long int)srv.response.result);		
	}
	else
	{
		ROS_ERROR("Failed to call service ros_tutorial_srv");
		return 1;
	}
	
	
	return 0;
}


