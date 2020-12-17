#include <iostream>
#include <string.h>
#include <fstream>
#include <algorithm>
#include <iterator>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include "opencv2/video/tracking.hpp"
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/geometry.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
 
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <limits>
#include <utility>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_publisher");
    ros::NodeHandle nh;

    ros::Publisher test_publisher = nh.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 100);

    ros::Rate loop_rate(10);
    sensor_msgs::PointCloud2 sensor_msgs;
    
    int height = 0;

    while(ros::ok())
    {
        sensor_msgs.height = height;

        test_publisher.publish(sensor_msgs);

        loop_rate.sleep();

        height++;
    }

    return 0;
}