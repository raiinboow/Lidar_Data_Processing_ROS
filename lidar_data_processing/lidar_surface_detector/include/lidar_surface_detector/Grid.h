/*****************************************************************
Author:                 J. Gong
Date:                      2021-02-05
Description:        GridCell, GridRow, GridMap
*****************************************************************/

#pragma once

// System
#include <map>
#include <vector>

// PCL 
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>

// Custom
#include "Utility.h"

// define point type
typedef pcl::PointXYZ PointT;

using namespace std;


/**************************************************************************
 * Class Name: GridCell
 * Description: Minimal unit for surface description
 **************************************************************************/
class GridCell{

    private:

        //scan
        float AZIMUTH_MIN;
        float AZIMUTH_MAX;
        int ALTITUDE_MIN;
        int ALTITUDE_MAX;

        //resolution
        float AZIMUTH_RESOLUTION;
        int COUNT_SCAN;

        //trust
        float CONFIDENCE_THRESHOLD;

        //decoder
        LeishenDecoder ls_decoder;

        //list for data
        std::vector<PatternDetector*> pd_ptr_list;
        // std::vector<float>target_altitude_list;              //###small range, altitude considered constant to decouple with mount_height
        std::vector<float> azimuth_list;
        std::vector<float> delta_list;
        std::vector<bool> detection_list;
        std::vector<float> distance_list;

        // output
        float distance_output;
        bool detection_output;

        /** 
         * Function Name: initialize_list
         * Description: initialize internal lists
         * Input: void
         * Output: void
         */     
        void initialize_list();


        /** 
         * Function Name: calculate_delta
         * Description: calculate the diffrence in given direction, points accessed by altitude and azimuth, min-max
         * Input:
         *          - pcl::PointCloud<PointT>::Ptr inPtr: pointer to a point cloud Gen1
         *          - float altitude_min: min altitude(degree)
         *          - float altitude_max: max altitude(degree)
         *          - float azimuth_min: min azimuth(degree)
         *          - float azimuth_max: max azimuth(degree) 
         *          - char direction: 'x', 'y', 'z'
         * Output:
         *          - float delta: calculate by point_min - point_max
         */     
         float calculate_delta(pcl::PointCloud<PointT>::Ptr inPtr, float altitude_min, float altitude_max, float azimuth_min, float azimuth_max, char direction);


        /** 
         * Function Name: update_list
         * Description: update internal lists from point cloud
         *          - delta_list
         *          - detection_list
         *          - distance_list
         * Input:
         *          - pcl::PointCloud<PointT>::Ptr inPtr: pointer to a point cloud Gen1
         *          - double timestamp_ns: ROS time stamp
         * Output: void
         */     
        void update_list(pcl::PointCloud<PointT>::Ptr inPtr, double timestamp_ns);


        /** 
         * Function Name: calculate_confidence
         * Description: update output values
         *          - confidence
         *          - distance_output
         * Input: void
         * Output:
         *          - float confidence: pattern detection rate
         */   
        float calculate_confidence();     

       


    public:

        //position and ID, info only
        float CELL_X_MIN;
        float CELL_X_MAX;
        float CELL_Y;
        int CELL_ROW_ID;
        int CELL_COLUMN_ID;


        /** 
         * Function Name: GridCell
         * Description: Constructor
         * Input: void
         */     
        GridCell();


        /** 
         * Function Name: GridCell
         * Description: Constructor
         * Input: 
         *          - float x_max: x-coordinate of the cell(m)
         *          - float size: resolution in x direction(m)
         *          - float y: y-coordinate of the cell(m)
         *          - int altitude_min: lower scan angle(degree)
         *          - int altitude_max: upper scan angle(degree)
         *          - float azimuth_min: min azimuth(degree) 
         *          - float azimuth_max: max azimuth(degree)
         */     
        GridCell(float x_max, float size, float y, int altitude_min, int altitude_max, float azimuth_min, float azimuth_max);


        /** 
         * Function Name: Detect
         * Description: 
         *          - update list
         *          - calculate confidence and distance
         *          - positive detection if confidence beyond CONFIDENCE_THRESHOLD
         * Input:
         *          - pcl::PointCloud<PointT>::Ptr inPtr: pointer to a point cloud Gen1
         *          - double timestamp_ns: ROS time stamp
         * Output:
         *          - bool detection_output: true if confidence beyond CONFIDENCE_THRESHOLD
         */     
        bool Detect(pcl::PointCloud<PointT>::Ptr inPtr, double timestamp_ns);  


        /** 
         * Function Name: delta
         * Description: get a random sample of delta list
         * Input: void
         * Output:
         *          - float delta: a random sample of delta list
         */     
        float delta();


        /** 
         * Function Name: distance
         * Description: get current distance
         * Input: void
         * Output:
         *          - float output_distance: current distance
         */     
        float distance();


        /** 
         * Function Name: detection
         * Description: get detection result
         * Input: void
         * Output:
         *          - bool detection_output: detection result
         */     
        bool detection();

};


/**************************************************************************
 * Class Name: GridRow
 * Description: Intagration of  a row of GridCells
 **************************************************************************/
class GridRow{

    private:

        // altitude
        int ALTITUDE_MIN;
        int ALTITUDE_MAX;
        
        // lateral information
        float TARGET_WIDTH;
        float TARGET_OFFSET_X;
        float TARGET_WIDTH_RESOLUTION;
        int COUNT_CELL_LATERAL;

        // internal lists
        vector<float> target_azimuth_list;
        vector<float> delta_list;
        vector<bool> detection_list;
        vector<float> distance_list;
        vector<GridCell*>  cell_ptr_list;

        // decoder
        LeishenDecoder ls_decoder;


        /** 
         * Function Name: initialize_variable
         * Description: initialize internal variables and lists
         * Input: void
         * Output: void
         */     
        void initialize_variable();


    public:

        // position and ID, info only
        int ROW_ID;
        float ROW_Y;

        /** 
         * Function Name: GridRow
         * Description: Constructor
         * Input: void
         */     
        GridRow();


        /** 
         * Function Name: GridRow
         * Description: Constructor
         * Input:
         *          - float target_width: width of ROI(m) 
         *          - float target_width_resolution: resolution/cell size(m)
         *          - float y: y-coordinate of the row(m)
         *          - int altitude_min: lower scan(degree)
         *          - int altitude_max: upper scan(degree)
         *          - int row_id: row ID, info only
         */     
        GridRow(float target_width, float target_width_resolution, float y, int altitude_min, int altitude_max, int row_id);


        /** 
         * Function Name: Update
         * Description: update internal lists from point cloud
         *          - cell_ptr_list
         *          - delta_list
         *          - detection_list
         *          - distance_list: distance set to -200 if no pattern detected
         * Input:
         *          - pcl::PointCloud<PointT>::Ptr inPtr: pointer to a point cloud Gen1
         *          - double timestamp_ns: ROS time stamp
         * Output: void
         */    
        void Update(pcl::PointCloud<PointT>::Ptr inPtr, double timestamp_ns);


        /** 
         * Function Name: delta
         * Description: get delta at index 'column'
         * Input: 
         *          - int column: inqury index
         * Output:
         *          - float delta: delta at index 'column'
         */  
        float delta(int column);


        /** 
         * Function Name: detection
         * Description: get detection result at index 'column'
         * Input: 
         *          - int column: inqury index
         * Output:
         *          - bool detection: detection result at index 'column'
         */  
        bool detection(int column);

};


/**************************************************************************
 * Class Name: GridMap
 * Description: Intagration of  GridRows
 **************************************************************************/
class GridMap{

    private:

        // decoder
        LeishenDecoder ls_decoder;

        // configuration
        float MOUNT_HEIGHT;
        float MOUNT_ANGLE;
        float TARGET_WIDTH;
        float TARGET_LOOKAHEAD;
        float TARGET_WIDTH_RESOLUTION;
        float TARGET_OFFSET_X;

        // Grid
        int COUNT_CELL_LATERAL;
        int COUNT_SCAN;
        int COUNT_CELL_LONGITUDINAL;

        // internal lists
        vector<int> target_altitude_list;             // calculated from TARGET_LOOKAHEAD
        vector<float> target_y_list;
        std::vector<PatternDetector*> pd_ptr_list;
        std::vector<float> target_azimuth_list;
        std::vector<float> delta_list;
        std::vector<bool> detection_list;
        std::vector<float> distance_list;
        vector<GridRow*> row_ptr_list;

        /** 
         * Function Name: initialize_variable
         * Description: initialize internal variables and lists
         * Input: void
         * Output: void
         */     
        void initialize_variable();

    
    public:

        /** 
         * Function Name: GridMap
         * Description: Constructor
         * Input: void
         */     
        GridMap();


        /** 
         * Function Name: GridMap
         * Description: Constructor
         * Input:
         *          - float mount_height: height to ground(m)
         *          - float mount_angle: pitch angle of Lidar(degree)
         *          - float target_lookahead: farest intersection point with the ground(m)
         *          - float target_width: width of ROI(m) 
         *          - float target_width_resolution: resolution/cell size(m)
         */     
        GridMap(float mount_height, float mount_angle, float target_lookahead, float target_width, float target_width_resolution);


        /** 
         * Function Name: Update
         * Description: update each row
         * Input:
         *          - pcl::PointCloud<PointT>::Ptr inPtr: pointer to a point cloud Gen1
         *          - double timestamp_ns: ROS time stamp
         * Output: void
         */    
        void Update(pcl::PointCloud<PointT>::Ptr inPtr, double timestamp_ns);


        /** 
         * Function Name: Plot
         * Description: Visualize grid map in the console
         * Input: void
         * Output: void
         */  
        void Plot();


        /** 
         * Function Name: delta
         * Description: get delta at index 'column'
         * Input: 
         *          - int row: inqury index
         *          - int column: inqury index
         * Output:
         *          - float delta: delta at (row, column)
         */  
        float delta(int row, int column);


        /** 
         * Function Name: map_rows
         * Description: get number of rows
         * Input: void
         * Output:
         *          - int: number of rows
         */  
        int map_rows();

        /** 
         * Function Name: map_cols
         * Description: get number of columns
         * Input: void
         * Output:
         *          - int: number of columns
         */  
        int map_cols();

};