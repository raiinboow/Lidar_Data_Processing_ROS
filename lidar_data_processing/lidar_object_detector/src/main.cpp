/*****************************************************************
Author:                 J. Gong
Date:                      2021-02-04
Description:        Node 'lidar_object_detector'
*****************************************************************/

// ROS
#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>

// Custom
#include "lidar_object_detector/LidarObjectDetector.h"


int main(int argc, char** argv){
    
    // initialize node
    ros::init(argc, argv, "lidar_object_detector");
    ros::NodeHandle nh;

    // create LidarObjectDetector object
    LidarObjectDetector lod(nh);

    // use callback function of LidarObjectDetector
    ros::Subscriber sub = nh.subscribe("lslidar_point_cloud", 10, &LidarObjectDetector::detectionCallBack, &lod);

    // start spinning
    ros::spin();

    return 0;

}