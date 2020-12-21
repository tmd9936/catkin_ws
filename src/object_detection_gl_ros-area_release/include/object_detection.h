
#ifndef OBJDETECT_H
#define OBJDETECT_H


#include "gl_driver.h"

#define STATE_UP		1
#define STATE_UPDATE	2
#define STATE_DOWN		3

// 라이다로 검출된 오브젝트들
class ObjDetect
{
public:
	struct ObjInfo
	{
		int id = 0;
		long unsigned int total_point_count = 0;
		std::vector<double> cur_point{0.0,0.0};
		std::vector<double> cur_vel{0.0,0.0};
		std::vector<std::vector<double>> raw_point_vector;
		int raw_point_idx = 0;
		std::vector<std::vector<double>> avr_point_vector;
		int avr_point_idx = 0;
		std::vector<double> lefttop_point{0.0,0.0};
    	std::vector<double> rightbottom_point{0.0,0.0};
		
		int state = STATE_UP;
		int count = 0;
	};

public:
	int min_point_num = 3;
	double dist_thr = 0.1;
	int max_mean_point = 5;
	int max_traj_point = 1000;

	std::string serial_port_name = std::string("/dev/ttyUSB0");
    
	std::vector<ObjInfo> obj_info_vector;
	

public:
	~ObjDetect();

	void InitObjDetect(void);
	Gl::framedata_t GetRawData(void);
	void Detection(bool ref_onoff=false);

};


#endif
