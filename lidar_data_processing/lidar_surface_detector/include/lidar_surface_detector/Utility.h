/*****************************************************************
Author:                 J. Gong
Date:                      2021-02-04
Description:        PatternDetector, LeishenDecoder
*****************************************************************/

#pragma once
#define PI 3.14159

// System
#include<map>
#include <queue>

// ROS
#include<sensor_msgs/PointCloud2.h>

// PCL conversion
#include<pcl_conversions/pcl_conversions.h>

// PCL 
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>

// define point type
typedef pcl::PointXYZ PointT;


/**************************************************************************
 * Class Name: PatternDetector
 * Description: Detect a pattern of 'peak-trough'
 **************************************************************************/
class PatternDetector{

    private:

        // parameter
        float PATTERN_WINDOW;                       // pattern window(s)
        float PATTERN_INTERVAL;                     // interval between patterns (s)
        int SCAN_FREQUENCY;                            // Working frequency of Lidar (Hz)
        float AMPLITUDE_THRESHOLD;          // Amplitude threshold (m)
        bool POSITIVE_FIRST;                              // Pattern direction

        double PATTERN_WINDOW_NS;         // pattern window(ns), comply with ROS time stamp
        double PATTERN_INTERVAL_NS;       // interval between patterns (ns), comply with ROS time stamp

        bool peak_detect_1;                                 // Indicator for first peak
        bool peak_detect_2;                                 // Indicator for second peak
        double peak_ts_1;                                     // Timestamp of first peak
        double peak_ts_2;                                     // Timestamp of second peak


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
        void initialize_parameter();


        /** 
         * Function Name: is_peak_1
         * Description: Detect first peak/trough 
         * Input: 
         *          - float input: input value for detection
         * Output: 
         *          - bool: true if the input is the first peak/trough 
         */    
        bool is_peak_1(float input);


        /** 
         * Function Name: is_peak_2
         * Description: Detect second peak/trough 
         * Input: 
         *          - float input: input value for detection
         * Output: 
         *          - bool: true if the input is the second peak/trough 
         */    
        bool is_peak_2(float input);


        /** 
         * Function Name: is_in_window
         * Description: Whether interval between two time stamps is within PATTERN_WINDOW_NS
         * Input: 
         *          - double ts_last: last time stamp
         *          - double ts_new: new time stamp
         * Output: 
         *          - bool: true if ts_new within PATTERN_WINDOW of ts_last
         */  
        bool is_in_window(double ts_last, double ts_new);


        /** 
         * Function Name: is_out_interval
         * Description: Whether interval between two time stamps is out of PATTERN_INTERVAL_NS
         * Input: 
         *          - double ts_last: last time stamp
         *          - double ts_new: new time stamp
         * Output: 
         *          - bool: true if ts_new out of PATTERN_INTERVAL_NS of ts_last
         */  
        bool is_out_interval(double ts_last, double ts_new);


        /** 
         * Function Name: reset_peak_1
         * Description: Reset indicator and time stamp of first peak
         * Input: void
         * Output: void
         */  
        void reset_peak_1();


        /** 
         * Function Name: reset_peak_2
         * Description: Reset indicator and time stamp of second peak
         * Input: void
         * Output: void
         */  
        void reset_peak_2();


    public:

        /** 
         * Function Name: PatternDetector
         * Description: Constructor
         * Input: void
         */    
        PatternDetector();


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
        bool Detect(float input, double time_stamp);

};



/**************************************************************************
 * Class Name: LeishenDecoder
 * Description: Access Lidar point
 **************************************************************************/
class LeishenDecoder{

    private:

        float AZIMUTH_RESOLUTION;                                           // Azimuth resolution(degree), depends on scanning frequency
        std::map<int,int> MAP_ALTITUDE_SCAN;                    // Map between altitude(degree) and scan sequence


    public:

        /** 
         * Function Name: LeishenDecoder
         * Description: Constructor
         * Input: void
         */     
        LeishenDecoder();

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
        PointT get_point_azimuth_altitude2(pcl::PCLPointCloud2Ptr inPtr, float azimuth, int altitude);


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
        PointT get_point_azimuth_altitude(pcl::PointCloud<PointT>::Ptr inPtr, float azimuth, int altitude);


        /** 
         * Function Name: calculate_scan_angle
         * Description: Calculate the max scan angle, the intersection point with ground is within max_distance 
         * Input: 
         *          - float height: height from the ground(m)
         *          - float max_distance: max lookahead distance(m)
         * Output:
         *          - int result: scan angle(degree)
         * */   
        int calculate_scan_angle(float height, float max_distance);


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
        int calculate_scan_angle(float x, float height, float max_distance);


        /** 
         * Function Name: calculate_azimuth
         * Description: calculate azimuth from x and distance(-y)
         * Input: 
         *          - float x: x-coordinate(m)
         *          - float max_distance: (-y)-coordinate(m)
         * Output:
         *          - float result: azimuth(degree)
         * */   
        float calculate_azimuth(float x, float max_distance);


        /** 
         * Function Name: azimuth_resolution
         * Description: access AZIMUTH_RESOLUTION
         * Input: void
         * Output:
         *          - float AZIMUTH_RESOLUTION (degree)
         * */   
        float azimuth_resolution();


        /** 
         * Function Name: calculate_distance
         * Description: calculate distance from height and angle(degree)
         * Input: 
         *          - float height: height from ground(m)
         *          - float angle: pitch angle(degree)
         * Output:
         *          - float distance(m)
         * */   
        float calculate_distance(float height, float angle);

};