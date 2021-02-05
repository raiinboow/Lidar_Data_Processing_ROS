/*****************************************************************
Author:                 J. Gong
Date:                      2021-02-05
Description:       LidarSurfaceDetector
*****************************************************************/

// Custom
#include "lidar_surface_detector/LidarSurfaceDetector.h"
#include  "lidar_surface_detector/Utility.h"
#define PI 3.14159

// System
#include<cmath>
#include<iostream>
#include <iomanip>

// PCL conversion
#include<pcl_conversions/pcl_conversions.h>



/**************************************************************************
 * Class Name: LidarSurfaceDetector
 * Description: Road surface detection through asymmetrical grid
 **************************************************************************/

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
LidarSurfaceDetector::LidarSurfaceDetector(ros::NodeHandle nh){

    MOUNT_HEIGHT = 0.3;
    MOUNT_ANGLE = 0;
    TARGET_WIDTH = 2;
    TARGET_WIDTH_RESOLUTION = 0.1;
    TARGET_LOOKAHEAD = 5;

    // initialize 
    initialize_variable(nh);
    
}


/** 
 * Function Name: initialize_variable
 * Description: initialize grid map and publisher
 * Input:
 *          - ros::NodeHandle nh: ROS NodeHandel
 * Output: void
 */     
void LidarSurfaceDetector::initialize_variable(ros::NodeHandle nh){

    // grid map
    gm = new GridMap();
    *gm = GridMap(MOUNT_HEIGHT, MOUNT_ANGLE, TARGET_LOOKAHEAD, TARGET_WIDTH, TARGET_WIDTH_RESOLUTION);

    // publisher
    for(int i=0; i<gm->map_rows(); i++){

        vector<ros::Publisher*> publisher_row;

        for(int j=0; j<gm->map_cols(); j++){
            ros::Publisher *pub = new ros::Publisher;
            std::string topic_str = "delta_" + std::to_string(i)+"_"+std::to_string(j);
            const char* topic = topic_str.c_str();
            *pub = nh.advertise<std_msgs::Float32>(topic, 1000);
            publisher_row.push_back(pub);
        }

        publisher_matrix.push_back(publisher_row);

    }

}


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
void LidarSurfaceDetector::detectionCallBack(const sensor_msgs::PointCloud2ConstPtr& input){

    cout<<"########################################################"<<endl;

    // time stamp
    ros::Time ts = input->header.stamp;
    double timestamp_ns = ts.toNSec();
    
    // convert to PointCloud Gen2
    pcl::PCLPointCloud2::Ptr inPtr(new pcl::PCLPointCloud2); 
    pcl_conversions::toPCL(*input, *inPtr);

    // convert to PointCloud Gen1
    pcl::PointCloud<PointT>::Ptr XYZPtr( new pcl::PointCloud<PointT>);
    pcl::fromPCLPointCloud2(*inPtr, *XYZPtr); 
    
    // update grid
    gm->Update(XYZPtr, timestamp_ns);

    //publish
    publish_delta(gm);
    
    //visualize
    gm->Plot();
   
}


/** 
 * Function Name: publish_delta
 * Description: publish delta of each cell in grid map
 * Input: void
 * Output: void
 */     
void LidarSurfaceDetector::publish_delta(){

    for(int i=0; i<gm->map_rows(); i++){
        for(int j=0; j<gm->map_cols(); j++){
            publisher_matrix[i][j]->publish(gm->delta(i, j));
        }
    }

}
