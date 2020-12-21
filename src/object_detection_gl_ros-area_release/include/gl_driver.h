
#ifndef GLDRIVER_H
#define GLDRIVER_H

#include <stdio.h> 
#include <unistd.h> 
#include <stdlib.h>
#include <string.h>
#include <chrono>
#include <thread>
#include <iostream>
#include <iomanip>
#include <vector>
#include <math.h>

// http://docs.ros.org/en/jade/api/serial/html/index.html
// RS-232와 같은 직렬 통신을 위한 ROS 라이브러리
// 타임아웃제어
// 핸드쉐이킹(데이터교환) 상태(CTS, DSR, RI, CD 및 RTS, DTR) 확인 및 설정
// 핸드쉐이킹 라인 변경, 차단
// I/O를 개별적으로 플러쉬하고 모든 쓰기가 완료 될 때까지 차단
#include "serial/serial.h"

// serial.h
// 


// /msg, srv 폴더에 놓이는 메세지, 서비스, 액션 (*.msg, *.srv, *.action)은 헤더 파일로 변환후
// 구조체 타입으로 변환되기 때문에 바로 구조체로 바꿔도 괜찮나?
class Gl
{
public:
	struct framedata_t
	{
		std::vector<double> angle;
		std::vector<double> distance;
		std::vector<double> pulse_width;
	};

// baudrate 보드 속도 (비트/초의 단위를 사용하는 데이터 전송 속도)
public:
	Gl(std::string &port, uint32_t baudrate);
	Gl();
	~Gl();

	void OpenSerial(std::string &port, uint32_t baudrate);

	std::string GetSerialNum(void);
	framedata_t ReadFrameData(void);
	void SetFrameDataEnable(uint8_t framedata_enable);

private:
	void ThreadCallBack(void);
};

#endif