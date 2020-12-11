#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Point.h"

#include "object_detection.h"


class ObjectDetectionGlRosNode
{
public:
	ObjectDetectionGlRosNode();

    void InitROS(void);
    
    void Run(void);
    void PubRawLidar(Gl::framedata_t frame_data);
    void PubMarker(void);
    void InitArea(void);

private:
    // ros
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv{"~"};

    ros::Publisher data_pub;
    ros::Publisher rviz_pub;

    // Launch variables
    std::string frame_id = std::string("laser");
    std::string pub_topicname_lidar = std::string("scan");

    int max_ref_count = 40;

    std::string pub_topicname_marker = std::string("marker_array");

    // Local variables
    ObjDetect obj_detect;

    bool area_on = false;
    std::vector<double> area_pos = {0.0, 0.0, 0.0}; //x,y,z
    std::vector<std::vector<double>> area_rgb_vector; //r,g,b
    std::vector<double> area_scale = {0.0, 0.0, 0.001}; //x,y,z
    std::vector<double> border_x = {-9.65, -7.0, 0.0, 2.65};
    std::vector<double> border_y = {0.01, 10.0};
    visualization_msgs::Marker area_msg;
    std::vector<bool> area_flag;



    int timer_loop_count = 0;
};


ObjectDetectionGlRosNode::ObjectDetectionGlRosNode()
{
    InitROS();
    InitArea();
    obj_detect.InitObjDetect();
}


void ObjectDetectionGlRosNode::InitROS(void)
{
    nh_priv.param("serial_port_name", obj_detect.serial_port_name, obj_detect.serial_port_name);
    nh_priv.param("frame_id", frame_id, frame_id);
    nh_priv.param("pub_topicname_lidar", pub_topicname_lidar, pub_topicname_lidar);

    nh_priv.param("min_point_num", obj_detect.min_point_num, obj_detect.min_point_num);
    nh_priv.param("dist_thr", obj_detect.dist_thr, obj_detect.dist_thr);
    nh_priv.param("max_mean_point", obj_detect.max_mean_point, obj_detect.max_mean_point);
    nh_priv.param("max_traj_point", obj_detect.max_traj_point, obj_detect.max_traj_point);

    nh_priv.param("max_ref_count", max_ref_count, max_ref_count);
    
    nh_priv.param("pub_topicname_marker", pub_topicname_marker, pub_topicname_marker);

    nh_priv.param("area_on", area_on, area_on);
    
    data_pub = nh.advertise<sensor_msgs::LaserScan>(pub_topicname_lidar, 10);
    rviz_pub = nh.advertise<visualization_msgs::MarkerArray>(pub_topicname_marker, 10);
}

void ObjectDetectionGlRosNode::InitArea(void)
{
    std::vector<double> green {0.0,0.2,0.0};
    std::vector<double> red {0.2,0.0,0.0};
    area_rgb_vector.push_back(green);
    area_rgb_vector.push_back(red);
    area_rgb_vector.push_back(green);
}

void ObjectDetectionGlRosNode::Run(void)
{
    if(timer_loop_count<max_ref_count)
    {
        obj_detect.Detection(false);
        PubMarker();

        timer_loop_count++;
    }
    else
    {
        obj_detect.Detection(true);
        PubMarker();
    }

    Gl::framedata_t frame_data = obj_detect.GetRawData();
    PubRawLidar(frame_data);
}


void ObjectDetectionGlRosNode::PubRawLidar(Gl::framedata_t frame_data)
{
    sensor_msgs::LaserScan scan_msg;

    int num_data = frame_data.distance.size();
    if(num_data>0)
    {
        scan_msg.header.stamp = ros::Time::now();
        scan_msg.header.frame_id = frame_id;
        scan_msg.angle_min = frame_data.angle[0];
        scan_msg.angle_max = frame_data.angle[num_data-1];
        scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (double)(num_data-1);
        scan_msg.range_min = 0.1;
        scan_msg.range_max = 30.0;
        scan_msg.ranges.resize(num_data);
        for(int i=0; i<num_data; i++)
        {
            scan_msg.ranges[i] = frame_data.distance[i];
        }
        
        data_pub.publish(scan_msg);
    }
}


void ObjectDetectionGlRosNode::PubMarker(void) 
{
    //////////////////////////////////////////////////////////////
    // clear markers
    //////////////////////////////////////////////////////////////
    visualization_msgs::MarkerArray marker_array_msg;
    visualization_msgs::Marker marker_msg;
    marker_msg.header.frame_id = frame_id;
    marker_msg.header.stamp = ros::Time::now();
    marker_msg.ns = "obj";
    marker_msg.id = 0;
    marker_msg.action = visualization_msgs::Marker::DELETEALL;
    marker_array_msg.markers.push_back(marker_msg);
    rviz_pub.publish(marker_array_msg);

    area_flag.clear();
    for(int i=0;i<area_rgb_vector.size();i++) area_flag.push_back(false);


    //////////////////////////////////////////////////////////////
    // publish markers
    //////////////////////////////////////////////////////////////
    
    //if(obj_detect.obj_info_vector.size()==0) return; 
    
    for(int i=0; i<obj_detect.obj_info_vector.size(); i++)
    {
        //////////////////////////////////////////////////////////////
        // Sphere marker
        //////////////////////////////////////////////////////////////
        visualization_msgs::Marker marker_msg;
        marker_msg.header.frame_id = frame_id;
        marker_msg.header.stamp = ros::Time::now();
        marker_msg.ns = "obj";
        marker_msg.id = obj_detect.obj_info_vector[i].id;
        marker_msg.type = visualization_msgs::Marker::SPHERE;
        marker_msg.action = visualization_msgs::Marker::ADD;
        marker_msg.pose.position.x = obj_detect.obj_info_vector[i].cur_point[0];
        marker_msg.pose.position.y = obj_detect.obj_info_vector[i].cur_point[1];
        marker_msg.pose.position.z = 0;
        marker_msg.pose.orientation.w = 1.0;
        marker_msg.scale.x = 0.2;
        marker_msg.scale.y = 0.2;
        marker_msg.scale.z = 0.2;
        marker_msg.color.a = 1.0; // Don't forget to set the alpha!
        marker_msg.color.r = 0.0;
        marker_msg.color.g = 1.0;
        marker_msg.color.b = 0.0;

        marker_array_msg.markers.push_back(marker_msg);


        //////////////////////////////////////////////////////////////
        // box marker
        //////////////////////////////////////////////////////////////
        visualization_msgs::Marker box_msg;
        box_msg.header.frame_id = frame_id;
        box_msg.header.stamp = ros::Time::now();
        box_msg.ns = "box";
        box_msg.id = obj_detect.obj_info_vector[i].id;
        box_msg.type = visualization_msgs::Marker::LINE_STRIP;
        box_msg.action = visualization_msgs::Marker::ADD;
        box_msg.pose.orientation.w = 1.0;
        box_msg.scale.x = 0.05;
        box_msg.color.a = 1.0; // Don't forget to set the alpha!
        box_msg.color.r = 0.0;
        box_msg.color.g = 1.0; 
        box_msg.color.b = 0.0;

        geometry_msgs::Point box_point_msg;
        box_point_msg.x = obj_detect.obj_info_vector[i].lefttop_point[0];
        box_point_msg.y = obj_detect.obj_info_vector[i].lefttop_point[1];
        box_msg.points.push_back(box_point_msg);

        box_point_msg.x = obj_detect.obj_info_vector[i].rightbottom_point[0];
        box_point_msg.y = obj_detect.obj_info_vector[i].lefttop_point[1];
        box_msg.points.push_back(box_point_msg);

        box_point_msg.x = obj_detect.obj_info_vector[i].rightbottom_point[0];
        box_point_msg.y = obj_detect.obj_info_vector[i].rightbottom_point[1];
        box_msg.points.push_back(box_point_msg);

        box_point_msg.x = obj_detect.obj_info_vector[i].lefttop_point[0];
        box_point_msg.y = obj_detect.obj_info_vector[i].rightbottom_point[1];
        box_msg.points.push_back(box_point_msg);

        box_point_msg.x = obj_detect.obj_info_vector[i].lefttop_point[0];
        box_point_msg.y = obj_detect.obj_info_vector[i].lefttop_point[1];
        box_msg.points.push_back(box_point_msg);
        
        marker_array_msg.markers.push_back(box_msg);


        //////////////////////////////////////////////////////////////
        // Line marker
        //////////////////////////////////////////////////////////////
        visualization_msgs::Marker line_msg;
        line_msg.header.frame_id = frame_id;
        line_msg.header.stamp = ros::Time::now();
        line_msg.ns = "trj";
        line_msg.id = obj_detect.obj_info_vector[i].id;
        line_msg.type = visualization_msgs::Marker::LINE_STRIP;
        line_msg.action = visualization_msgs::Marker::ADD;
        line_msg.pose.orientation.w = 1.0;
        line_msg.scale.x = 0.1;
        line_msg.color.a = 1.0; // Don't forget to set the alpha!
        line_msg.color.r = 0.0;
        line_msg.color.g = 1.0;
        line_msg.color.b = 0.0;

        for(int j=0; j<obj_detect.obj_info_vector[i].avr_point_vector.size(); j++)
        {
            int idx = obj_detect.obj_info_vector[i].avr_point_idx - j;
            if(idx<0) idx += obj_detect.max_traj_point;
            
            geometry_msgs::Point line_point_msg;
            line_point_msg.x = obj_detect.obj_info_vector[i].avr_point_vector[idx][0];
            line_point_msg.y = obj_detect.obj_info_vector[i].avr_point_vector[idx][1];

            if((abs(line_point_msg.x)>0.5) || (abs(line_point_msg.y)>0.5)) line_msg.points.push_back(line_point_msg);
        }

        if(line_msg.points.size()>1) marker_array_msg.markers.push_back(line_msg);

        //////////////////////////////////////////////////////////////
        // Text marker
        //////////////////////////////////////////////////////////////
        visualization_msgs::Marker text_msg;
        text_msg.header.frame_id = frame_id;
        text_msg.header.stamp = ros::Time::now();
        text_msg.ns = "text";
        text_msg.id = obj_detect.obj_info_vector[i].id;
        text_msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_msg.action = visualization_msgs::Marker::ADD;
        text_msg.pose.position.x = obj_detect.obj_info_vector[i].cur_point[0];
        text_msg.pose.position.y = obj_detect.obj_info_vector[i].cur_point[1]+0.5;
        text_msg.scale.z = 0.3;
        text_msg.color.a = 1.0; // Don't forget to set the alpha!
        text_msg.color.r = 0.0;
        text_msg.color.g = 1.0;
        text_msg.color.b = 0.0;

        std::string text_string = "[" + std::to_string(text_msg.id) + "]\nVel=(" + std::to_string(obj_detect.obj_info_vector[i].cur_vel[0]) + ", " + std::to_string(obj_detect.obj_info_vector[i].cur_vel[1]) + ")";
        text_msg.text = text_string;

        marker_array_msg.markers.push_back(text_msg);

        //////////////////////////////////////////////////////////////
        // Arrow marker
        //////////////////////////////////////////////////////////////
        visualization_msgs::Marker arrow_msg;
        arrow_msg.header.frame_id = frame_id;
        arrow_msg.header.stamp = ros::Time::now();
        arrow_msg.ns = "arrow";
        arrow_msg.id = obj_detect.obj_info_vector[i].id;
        arrow_msg.type = visualization_msgs::Marker::ARROW;
        arrow_msg.action = visualization_msgs::Marker::ADD;
        arrow_msg.pose.orientation.w = 1.0;
        arrow_msg.scale.x = 0.1;
        arrow_msg.scale.y = 0.3;
        arrow_msg.scale.z = 0.3;
        arrow_msg.color.a = 1.0; // Don't forget to set the alpha!
        arrow_msg.color.r = 0.0;
        arrow_msg.color.g = 0.0;
        arrow_msg.color.b = 1.0;

        bool condition1 = obj_detect.obj_info_vector[i].total_point_count>obj_detect.max_mean_point*2;
        bool condition2 = abs(obj_detect.obj_info_vector[i].cur_vel[0])>0.0001;
        bool condition3 = abs(obj_detect.obj_info_vector[i].cur_vel[1])>0.0001;

        if( condition1 && (condition2||condition3) )
        {
            geometry_msgs::Point arrow_point_msg;
            arrow_point_msg.x = obj_detect.obj_info_vector[i].cur_point[0];
            arrow_point_msg.y = obj_detect.obj_info_vector[i].cur_point[1];
            arrow_msg.points.push_back(arrow_point_msg);

            arrow_point_msg.x = obj_detect.obj_info_vector[i].cur_point[0] + obj_detect.obj_info_vector[i].cur_vel[0];
            arrow_point_msg.y = obj_detect.obj_info_vector[i].cur_point[1] + obj_detect.obj_info_vector[i].cur_vel[1];
            arrow_msg.points.push_back(arrow_point_msg);

            marker_array_msg.markers.push_back(arrow_msg);
        }


        //////////////////////////////////////////////////////////////
        // Area detection
        //////////////////////////////////////////////////////////////

        double cur_x = obj_detect.obj_info_vector[i].cur_point[0];
        double cur_y = obj_detect.obj_info_vector[i].cur_point[1];
        if((cur_y<border_y[1] && cur_y>border_y[0])){
            for(int i_x = 0; i_x < (border_x.size()-1); i_x++){
                if(cur_x>border_x[i_x] && cur_x<border_x[i_x+1]){
                    area_flag[i_x] = true;
                }
            }
        }

    }

    //////////////////////////////////////////////////////////////
    // Background area marker
    //////////////////////////////////////////////////////////////

    area_scale[1] = border_y[1]-border_y[0];
    area_pos[1] = area_scale[1]/2.0; 

    for(int area_idx = 0; area_idx<area_rgb_vector.size(); area_idx++){

        // background rgb
        std::vector<double> area_rgb {0.0,0.0,0.0};
        area_rgb[0] = area_rgb_vector[area_idx][0];
        area_rgb[1] = area_rgb_vector[area_idx][1];
        area_rgb[2] = area_rgb_vector[area_idx][2];

        if((int)area_flag[area_idx] == 1){
            area_rgb[0] += 0.2;
            area_rgb[1] += 0.2;
            area_rgb[2] += 0.2;
        }

        area_scale[0] = border_x[area_idx+1]-border_x[area_idx]; 
        area_pos[0] = (border_x[area_idx+1]+border_x[area_idx])/2.0; 

        area_msg.header.frame_id = frame_id;
        area_msg.header.stamp = ros::Time();
        area_msg.id = area_idx;
        area_msg.ns = "area";

        area_msg.type = visualization_msgs::Marker::CUBE;
        area_msg.lifetime = ros::Duration(0.1);
        area_msg.action = visualization_msgs::Marker::ADD;

        area_msg.pose.position.x = area_pos[0];
        area_msg.pose.position.y = area_pos[1];
        area_msg.pose.position.z = area_pos[2];
        area_msg.pose.orientation.x = 0.0;
        area_msg.pose.orientation.y = 0.0;
        area_msg.pose.orientation.z = 0.0;
        area_msg.pose.orientation.w = 1.0;

        area_msg.scale.x = area_scale[0];
        area_msg.scale.y = area_scale[1];
        area_msg.scale.z = area_scale[2];

        area_msg.color.r = area_rgb[0];
        area_msg.color.g = area_rgb[1];
        area_msg.color.b = area_rgb[2];
        area_msg.color.a = 0.5;

        if( area_on ) marker_array_msg.markers.push_back(area_msg);
    }

    rviz_pub.publish(marker_array_msg);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_detection_gl_ros_node");

    ObjectDetectionGlRosNode obj;

    // loop
    ros::Rate loop_rate(20);
    while(ros::ok())
    {
        obj.Run();

        ros::spinOnce();
        loop_rate.sleep();
    }
    

    return 0;
}