/*****************************************************************
Author:                 J. Gong
Date:                      2021-02-05
Description:        Node 'lidar_surface_detector'
*****************************************************************/

// ROS
#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>

// Custom
#include "lidar_surface_detector/LidarSurfaceDetector.h"


int main(int argc, char** argv){

    // initialize node
    ros::init(argc, argv, "lidar_surface_detector");
    ros::NodeHandle nh;

    // create LidarSurfaceDetector object
    LidarSurfaceDetector lbd(nh);

    // use callback function of LidarSurfaceDetector
    ros::Subscriber sub2 = nh.subscribe("lslidar_point_cloud", 10, &LidarSurfaceDetector::detectionCallBack, &lbd);

    // start spinning
    ros::spin();

    return 0;

}