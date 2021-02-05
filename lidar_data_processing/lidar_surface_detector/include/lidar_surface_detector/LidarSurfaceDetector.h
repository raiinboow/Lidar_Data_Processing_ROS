/*****************************************************************
Author:                 J. Gong
Date:                      2021-02-05
Description:       LidarSurfaceDetector
*****************************************************************/

#pragma once

// System
#include<map>
#include<vector>

// ROS
#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<std_msgs/Float32.h>

// PCL conversion
#include<pcl_conversions/pcl_conversions.h>

// PCL 
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>

//Custom
#include "lidar_surface_detector/Utility.h"
#include "lidar_surface_detector/Grid.h"


// define point type
typedef pcl::PointXYZ PointT;


using namespace std;


/**************************************************************************
 * Class Name: LidarSurfaceDetector
 * Description: Road surface detection through asymmetrical grid
 **************************************************************************/
class LidarSurfaceDetector{

    private:

        // configuration
        float MOUNT_HEIGHT;
        float MOUNT_ANGLE;
        float TARGET_WIDTH;
        float TARGET_LOOKAHEAD;
        float TARGET_WIDTH_RESOLUTION;

        // Gridmap
        GridMap* gm;

        // publisher
        vector<vector<ros::Publisher*>> publisher_matrix;


        /** 
         * Function Name: initialize_variable
         * Description: initialize grid map and publisher
         * Input:
         *          - ros::NodeHandle nh: ROS NodeHandel
         * Output: void
         */     
        void initialize_variable(ros::NodeHandle nh);


        /** 
         * Function Name: publish_delta
         * Description: publish delta of each cell in grid map
         * Input: void
         * Output: void
         */     
        void publish_delta();


    public:

        /** 
         * Function Name: LidarSurfaceDetector
         * Description: Constructor
         * Input:
         *          - ros::NodeHandle nh: ROS NodeHandel
         * 
         * ######################TODO##########################
         *          - read configuration from ROS parameter server
         * #####################################################
         */     
        LidarSurfaceDetector(ros::NodeHandle nh);


        /** 
         * Function Name: detectionCallBack
         * Description:
         *          - convert to PCL pointcloud Gen1
         *          - update grid map
         *          - publish delta of each cell
         *          - visualize grid map in the console
         * Input:
         *          - const sensor_msgs::PointCloud2ConstPtr& input: ROS pointcloud message
         * Output: void
         */     
        void detectionCallBack(const sensor_msgs::PointCloud2ConstPtr& input);

};