/*****************************************************************
Author:                 J. Gong
Date:                      2021-02-04
Description:        PatternDetector, LeishenDecoder
*****************************************************************/

// Custom
#include "lidar_surface_detector/Utility.h"

// System
#include <math.h>
#include<iostream>

// ROS
#include<ros/ros.h>

using namespace std;


/**************************************************************************
 * Class Name: PatternDetector
 * Description: Detect a pattern of 'peak-trough'
 **************************************************************************/

/** 
 * Function Name: PatternDetector
 * Description: Constructor
 * Input: void
 */     
PatternDetector::PatternDetector(){

    initialize_parameter();

}


/** 
 * Function Name: initialize_parameter
 * Description: Initialize parameters and variables
 * Input: void
 * Output: void
 * 
 * ####################TODO######################
 *          - Read parameters from ROS parameter server
 * ###############################################
 */       
void PatternDetector::initialize_parameter(){

    // Hard-coded parameter                                                 
    PATTERN_WINDOW=2;       
    PATTERN_INTERVAL = 0.1;
    SCAN_FREQUENCY = 10;
    AMPLITUDE_THRESHOLD = 0.025;
    POSITIVE_FIRST = true;

    // Conver to ns
    PATTERN_WINDOW_NS = PATTERN_WINDOW * 1e9;
    PATTERN_INTERVAL_NS = PATTERN_INTERVAL * 1e9;

    // initialize detector
    reset_peak_1();
    reset_peak_2();

}


/** 
 * Function Name: Detect
 * Description:
 *          - Take the latest input
 *          - Always detect the first peak
 *          - Reset first peak if the second peak comes later than PATTERN_WINDOW
 *          - Cool down for PATTERN_INTERVAL after a sucussful detection
 * Input:
 *          - float input: input value for detection
 *          - double time_stamp: time stamp in ns
 * Output: 
 *          - bool: true if the input completes a pattern
 * 
 * ####################TODO######################
 *          - Seperate upward/downward edge detection
 *          - Dynamic window size
 * ###############################################
 */       
bool PatternDetector::Detect(float input, double time_stamp){

    // cool out if a pattern is just detected
    if(peak_detect_1 && peak_detect_2){

        // reset after an interval
        if(is_out_interval(peak_ts_2, time_stamp)){
            reset_peak_1();
            reset_peak_2();
        }
        // still in interval, return false to avoid repeated detection
        else return false;

    }

    // ready to detect new pattern
    else{

        // always detect peak1
        if(is_peak_1(input)){
            ROS_DEBUG("Detect first peak!");
            peak_detect_1 = true;
            peak_ts_1 = time_stamp;
        }

        // not peak1
        else{
            
            // peak1 already detected, waiting for peak2
            if(peak_detect_1){

                // detect peak2, compare timestamp
                if(is_peak_2(input)){

                    // peak2 within window
                    if(is_in_window(peak_ts_1, time_stamp)){
                        peak_detect_2 = true;
                        peak_ts_2 = time_stamp;
                        ROS_DEBUG("Detect second peak!");
                    }

                    // out of window, reset peak 1
                    else{ 
                        reset_peak_1(); 
                        ROS_DEBUG("Second peak out of time window, reset peak1");
                    }

                }

                // not peak2, reset peak1 if not within timewindow
                else if(!is_in_window(peak_ts_1, time_stamp)) {
                    reset_peak_1();
                    ROS_DEBUG("No second peak, reset peak1");
                }

            }
        }
    }

    // return result
    if(peak_detect_1 && peak_detect_2) return true;
    else return false;

}


/** 
 * Function Name: is_peak_1
 * Description: Detect first peak/trough 
 * Input: 
 *          - float input: input value for detection
 * Output: 
 *          - bool: true if the input is the first peak/trough 
 */       
bool PatternDetector::is_peak_1(float input){

    bool result;

    if((POSITIVE_FIRST && isgreater(input, AMPLITUDE_THRESHOLD)) ||
        (!POSITIVE_FIRST && isless(input, -1 *AMPLITUDE_THRESHOLD))){   
            result = true;
    }
    else result = false;

    return result;

}


/** 
 * Function Name: is_peak_2
 * Description: Detect second peak/trough 
 * Input: 
 *          - float input: input value for detection
 * Output: 
 *          - bool: true if the input is the second peak/trough 
 */    
bool PatternDetector::is_peak_2(float input){

    bool result;

    if((POSITIVE_FIRST && isless(input, -1*AMPLITUDE_THRESHOLD)) ||
        (!POSITIVE_FIRST && isgreater(input, AMPLITUDE_THRESHOLD))){   
            result = true;
    }
    else result = false;

    return result;

}


/** 
 * Function Name: is_in_window
 * Description: Whether interval between two time stamps is within PATTERN_WINDOW_NS
 * Input: 
 *          - double ts_last: last time stamp
 *          - double ts_new: new time stamp
 * Output: 
 *          - bool: true if ts_new within PATTERN_WINDOW_NS of ts_last
 */  
bool PatternDetector::is_in_window(double ts_last, double ts_new){

    double duration = ts_new - ts_last;

    if (isgreater(duration,PATTERN_WINDOW_NS)) return false;
    else return true;

}


/** 
 * Function Name: is_out_interval
 * Description: Whether interval between two time stamps is out of PATTERN_INTERVAL_NS
 * Input: 
 *          - double ts_last: last time stamp
 *          - double ts_new: new time stamp
 * Output: 
 *          - bool: true if ts_new out of PATTERN_INTERVAL_NS of ts_last
 */  
bool PatternDetector::is_out_interval(double ts_last, double ts_new){

     double duration = ts_new - ts_last;

    if (isgreater(duration,PATTERN_INTERVAL_NS)) return true;
    else return false;

}


/** 
 * Function Name: reset_peak_1
 * Description: Reset indicator and time stamp of first peak
 * Input: void
 * Output: void
 */  
void PatternDetector::reset_peak_1(){

    peak_detect_1 = false;
    peak_ts_1 = 0;

}


/** 
 * Function Name: reset_peak_2
 * Description: Reset indicator and time stamp of second peak
 * Input: void
 * Output: void
 */  
void PatternDetector::reset_peak_2(){

    peak_detect_2 = false;
    peak_ts_2 = 0;

}



/**************************************************************************
 * Class Name: LeishenDecoder
 * Description: Access Lidar point
 **************************************************************************/

/** 
 * Function Name: LeishenDecoder
 * Description: Constructor
 * Input: void
 */     
LeishenDecoder::LeishenDecoder(){

    AZIMUTH_RESOLUTION = 0.18;

    MAP_ALTITUDE_SCAN= {
        {-16, 0}, {0, 1}, {-15,2}, {1,3}, {-14,4}, {2,5}, {-13,6}, {3,7}, {-12,8}, 
        {4,9}, {-11,10}, {5,11}, {-10,12}, {6,13}, {-9,14}, {7,15},
        {-8,16}, {8,17}, {-7,18}, {9,19}, {-6,20}, {10,21}, {-5,22}, {11,23}, 
        {-4,24}, {12,25}, {-3,26}, {13,27}, {-2,28}, {14,29}, {-1,30}, {15,31}, 
    };

}


/** 
 * Function Name: get_point_azimuth_altitude2
 * Description: Access point from point cloud Gen2 by azimuth and altitude
 * Input: 
 *          - pcl::PCLPointCloud2Ptr inPtr: pointer to a Gen2 point cloud
 *          - float azimuth: azimuth(degree)
 *          - int altitude: altitude(degree) 
 * Output:
 *          - PointT p: point at the specified position
 * */     
PointT LeishenDecoder::get_point_azimuth_altitude2(pcl::PCLPointCloud2Ptr inPtr, float azimuth, int altitude){

    PointT p;

    // Convert to point cloud Gen1
    pcl::PointCloud<PointT>::Ptr XYZPtr( new pcl::PointCloud<PointT>);
    pcl::fromPCLPointCloud2(*inPtr, *XYZPtr); 

    // ROS_INFO("width, height: %d, %d", inPtr->width, inPtr->height);

    // column
    int width = inPtr->width;
    int col = int(azimuth/AZIMUTH_RESOLUTION);

    // row
    int row;
    if(altitude<-16 || altitude>15) {
        altitude = 0;
        ROS_INFO("Invalid altitude input, set to 0 instead!");
    }
    // map altitude to row
    std::map<int,int>::iterator iter;
    iter = MAP_ALTITUDE_SCAN.find(altitude);
    if(iter != MAP_ALTITUDE_SCAN.end()) row = iter->second;

    //ROS_INFO("Point of azimuth %f, altitude %d -> (%d, %d)", azimuth, altitude, row, col);

    // retrive point
    p = (*XYZPtr)(col, row);

    // ROS_INFO("Point x:%f, y:%f, z:%f", p.x, p.y, p.z);

    return p;

}


/** 
 * Function Name: get_point_azimuth_altitude
 * Description: Access point from point cloud Gen1 by azimuth and altitude
 * Input: 
 *          - pcl::PointCloud<PointT>::Ptr inPtr: pointer to a Gen1 point cloud
 *          - float azimuth: azimuth(degree)
 *          - int altitude: altitude(degree) 
 * Output:
 *          - PointT p: point at the specified position
 * */   
PointT LeishenDecoder::get_point_azimuth_altitude(pcl::PointCloud<PointT>::Ptr inPtr, float azimuth, int altitude){

    PointT p;

     // width
    int width = inPtr->width;
    int col = int(azimuth/AZIMUTH_RESOLUTION);

    // height
    int row;
    if(altitude<-16 || altitude>15) {
        ROS_INFO("Invalid altitude input %d, set to 0 instead!", altitude);
        altitude = 0;
    }
    std::map<int,int>::iterator iter;
    iter = MAP_ALTITUDE_SCAN.find(altitude);
    if(iter != MAP_ALTITUDE_SCAN.end()) row = iter->second;

    //ROS_INFO("Point of azimuth %f, altitude %d -> (%d, %d)", azimuth, altitude, width, height);

    p = (*inPtr)(col, row);

    return p;

}


/** 
 * Function Name: calculate_scan_angle
 * Description: 
 *          - calculate the max scan angle
 *          - the intersection point with ground is within max_distance 
 * Input: 
 *          - float height: height from the ground(m)
 *          - float max_distance: max lookahead distance(m)
 * Output:
 *          - int result: scan angle(degree)
 * */   
int LeishenDecoder::calculate_scan_angle(float height, float max_distance){

    float angle_rad = atan2(-height, max_distance);
    int result = int(angle_rad * 180 / PI) - 1;

    return result;

}


/** 
 * Function Name: calculate_scan_angle
 * Description: 
 *          - calculate the max scan angle 
 *          - the intersection point with ground is within max_distance
 *          - x coordinate of the intersection point with ground is fixed
 * Input: 
 *          - float x: x coordinate of the intersection point with ground (m)
 *          - float height: height from the ground(m)
 *          - float max_distance: max lookahead distance(m)
 * Output:
 *          - int result: scan angle(degree)
 * */   
int LeishenDecoder::calculate_scan_angle(float x, float height, float max_distance){

    float projection = sqrt(x*x + max_distance*max_distance);
    float angle_rad = atan2(-height, projection);
    int result = int(angle_rad * 180 / PI) - 1;
    
    return result;

}


/** 
 * Function Name: calculate_azimuth
 * Description: calculate azimuth from x and distance(-y)
 * Input: 
 *          - float x: x-coordinate(m)
 *          - float max_distance: (-y)-coordinate(m)
 * Output:
 *          - float result: azimuth(degree)
 * */   
float LeishenDecoder::calculate_azimuth(float x, float max_distance){

    float angle_rad = PI + atan2(x, max_distance);
    float result = angle_rad * 180/PI;

    return result;

}


/** 
 * Function Name: azimuth_resolution
 * Description: access AZIMUTH_RESOLUTION
 * Input: void
 * Output:
 *          - float AZIMUTH_RESOLUTION (degree)
 * */   
float LeishenDecoder::azimuth_resolution(){

    return AZIMUTH_RESOLUTION;

}


/** 
 * Function Name: calculate_distance
 * Description: calculate distance from height and angle(degree)
 * Input: 
 *          - float height: height from ground(m)
 *          - float angle: pitch angle(degree)
 * Output:
 *          - float distance(m)
 * */   
float LeishenDecoder::calculate_distance(float height, float angle){

    float d;

    d = height / tan(angle * PI / 180 + 1e-6);

    return d;

}