/*****************************************************************
Author:                 J. Gong
Date:                      2021-02-05
Description:        GridCell, GridRow, GridMap
*****************************************************************/

// Custom
#include "lidar_surface_detector/Grid.h"

// System
#include<iostream>


/**************************************************************************
 * Class Name: GridCell
 * Description: Minimal unit for surface description
 **************************************************************************/

/** 
 * Function Name: GridCell
 * Description: Constructor
 * Input: void
 */     
GridCell::GridCell(){}


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
GridCell::GridCell(float x_max, float size, float y, int altitude_min, int altitude_max, float azimuth_min, float azimuth_max){

    CELL_X_MAX = x_max;
    CELL_X_MIN = CELL_X_MAX - size;

    CELL_Y = y;

    ALTITUDE_MIN = altitude_min;
    ALTITUDE_MAX = altitude_max;

    AZIMUTH_MIN = azimuth_min;
    AZIMUTH_MAX = azimuth_max;

    AZIMUTH_RESOLUTION = ls_decoder.azimuth_resolution();

    COUNT_SCAN = int((AZIMUTH_MAX-AZIMUTH_MIN)/AZIMUTH_RESOLUTION);

    initialize_list();

}


/** 
 * Function Name: initialize_list
 * Description: initialize internal lists
 * Input: void
 * Output: void
 */     
void GridCell::initialize_list(){

    CONFIDENCE_THRESHOLD=0.5;

    for(int i=0; i<COUNT_SCAN; i++){

        // pattern detector
        PatternDetector *pd = new PatternDetector;
        pd_ptr_list.push_back(pd);

        // azimuth
        azimuth_list.push_back(AZIMUTH_MIN + i * AZIMUTH_RESOLUTION);

        // delta
        delta_list.push_back(0.);

        // distance
        distance_list.push_back(0.);

        // detection
        detection_list.push_back(false);

    }


}


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
float GridCell::calculate_delta(pcl::PointCloud<PointT>::Ptr inPtr, float altitude_min, float altitude_max, float azimuth_min, float azimuth_max, char direction){

    PointT p1 = ls_decoder.get_point_azimuth_altitude(inPtr, azimuth_min, altitude_min);
    PointT p2 = ls_decoder.get_point_azimuth_altitude(inPtr, azimuth_max, altitude_max);

    float delta;

    // X
    if (direction == 88|| direction ==120)
        delta= p1.x - p2.x;
    // Y
    else if (direction == 89|| direction ==121)
        delta= p1.y - p2.y;
    // Z
    else if (direction == 90|| direction ==122)
        delta= p1.z - p2.z;
    // Invalid
    else
        delta = 0;

    return delta;

}


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
void GridCell::update_list(pcl::PointCloud<PointT>::Ptr inPtr, double timestamp_ns){

    // loop through scans
    for(int i =0; i<COUNT_SCAN; i++){

        //delta
        delta_list[i] = calculate_delta(inPtr,  ALTITUDE_MIN, ALTITUDE_MAX, azimuth_list[i], azimuth_list[i], 'z');
       
       //detection
        detection_list[i] = pd_ptr_list[i]->Detect(delta_list[i], timestamp_ns);

        //distance
        PointT p = ls_decoder.get_point_azimuth_altitude(inPtr, azimuth_list[i], ALTITUDE_MIN);
        distance_list[i] = -1 *p.y;

    }

}


/** 
 * Function Name: calculate_confidence
 * Description: update output values
 *          - confidence
 *          - distance_output
 * Input: void
 * Output:
 *          - float confidence: pattern detection rate
 */     
float GridCell::calculate_confidence(){

    float confidence = 0.;
    int positive_count = 0;
    int positive_count_valid = 0;
    float distance_sum = 0.;

    //find positive count
    for(int i =0; i<COUNT_SCAN; i++){
        if(detection_list[i]){
            positive_count++;
            if(!isnan(distance_list[i])){
                positive_count_valid++;
                distance_sum += distance_list[i];
            }
        }
    }

    //calculate confidence if there is a positive count
    if(positive_count>0){

        confidence = (positive_count+0.0)/COUNT_SCAN;

        //calculate average distance if there is a valid positive count
        if(positive_count_valid>0) distance_output = distance_sum / positive_count_valid;
        else distance_output = CELL_Y;

    }  
    // no positive count
    else confidence = 0.;

    return confidence;

}


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
bool GridCell::Detect(pcl::PointCloud<PointT>::Ptr inPtr, double timestamp_ns){

    // update internal lists
    update_list(inPtr, timestamp_ns);

    // calculate confidence
    float confidence;
    confidence = calculate_confidence();

    // set output
    if(std::isgreater(confidence,CONFIDENCE_THRESHOLD)){
        detection_output = true;
    }
    else{
        detection_output = false;
    }

    // return output
    return detection_output;

}

 
 /** 
 * Function Name: delta
 * Description: get a random sample of delta list
 * Input: void
 * Output:
 *          - float delta: a random sample of delta list
 */     
 float GridCell::delta(){

    return delta_list[rand()%COUNT_SCAN];

 }


 /** 
 * Function Name: distance
 * Description: get current distance
 * Input: void
 * Output:
 *          - float output_distance: current distance
 */     
float GridCell::distance(){

    return distance_output;

}


 /** 
 * Function Name: detection
 * Description: get detection result
 * Input: void
 * Output:
 *          - bool detection_output: detection result
 */     
bool GridCell::detection(){

    return detection_output;

}



/**************************************************************************
 * Class Name: GridRow
 * Description: Intagration of  a row of GridCells
 **************************************************************************/

/** 
 * Function Name: GridRow
 * Description: Constructor
 * Input: void
 */     
GridRow::GridRow(){}


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
GridRow::GridRow(float target_width, float target_width_resolution, float y, int altitude_min, int altitude_max, int row_id){

    TARGET_WIDTH = target_width;
    TARGET_WIDTH_RESOLUTION = target_width_resolution;

    ROW_Y = y;

    ALTITUDE_MAX = altitude_max;
    ALTITUDE_MIN = altitude_min;

    ROW_ID = row_id;

    initialize_variable();

}


/** 
 * Function Name: initialize_variable
 * Description: initialize internal variables and lists
 * Input: void
 * Output: void
 */     
void GridRow::initialize_variable(){

    TARGET_OFFSET_X = TARGET_WIDTH / 2;
    COUNT_CELL_LATERAL = int(TARGET_WIDTH / TARGET_WIDTH_RESOLUTION);

    // loop through all cells
    for(int i=0; i<COUNT_CELL_LATERAL; i++){

        // distance
        distance_list.push_back(0.);

        // detection
        detection_list.push_back(false);

        // delta
        delta_list.push_back(0.);

       // cell
        GridCell *cell = new GridCell;
        float cell_x = TARGET_OFFSET_X - i * TARGET_WIDTH_RESOLUTION;
        float azimuth_max= ls_decoder.calculate_azimuth(cell_x, abs(ROW_Y));
        float azimuth_min = ls_decoder.calculate_azimuth(cell_x-TARGET_WIDTH_RESOLUTION, abs(ROW_Y));
        *cell = GridCell(cell_x, TARGET_WIDTH_RESOLUTION, abs(ROW_Y), ALTITUDE_MIN, ALTITUDE_MAX, azimuth_min, azimuth_max);
        cell->CELL_ROW_ID = this->ROW_ID;
        cell->CELL_COLUMN_ID = i;
        cell_ptr_list.push_back(cell);

    }

}


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
void GridRow::Update(pcl::PointCloud<PointT>::Ptr inPtr, double timestamp_ns){

    // loop through all cells
    for(int i=0; i<COUNT_CELL_LATERAL; i++){

        // cell
       bool detection = cell_ptr_list[i]->Detect(inPtr, timestamp_ns);

       // detection
       detection_list[i] = detection;

       //delta
       delta_list[i] =  cell_ptr_list[i]->delta();

        //distance
        if(detection) {
           distance_list[i] = cell_ptr_list[i]->distance();
           ROS_INFO("Cell (%d,%d) detected a pattern, location(x,-y) is (%f, %f)", cell_ptr_list[i]->CELL_ROW_ID, cell_ptr_list[i]->CELL_COLUMN_ID, cell_ptr_list[i]->CELL_X_MAX, distance_list[i]);
        }
       else distance_list[i] = -200;

   }

}


 /** 
 * Function Name: delta
 * Description: get delta at index 'column'
 * Input: 
 *          - int column: inqury index
 * Output:
 *          - float delta: delta at index 'column'
 */  
float GridRow::delta(int column){

    return cell_ptr_list[column]->delta();

}


 /** 
 * Function Name: detection
 * Description: get detection result at index 'column'
 * Input: 
 *          - int column: inqury index
 * Output:
 *          - bool detection: detection result at index 'column'
 */  
bool GridRow::detection(int column){

    return cell_ptr_list[column]->detection();

}



/**************************************************************************
 * Class Name: GridMap
 * Description: Intagration of  GridRows
 **************************************************************************/

/** 
 * Function Name: GridMap
 * Description: Constructor
 * Input: void
 */     
GridMap::GridMap(){}


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
GridMap::GridMap(float mount_height, float mount_angle, float target_lookahead, float target_width, float target_width_resolution){

    MOUNT_HEIGHT = mount_height;
    MOUNT_ANGLE = mount_angle;
    TARGET_WIDTH = target_width;
    TARGET_WIDTH_RESOLUTION = target_width_resolution;
    TARGET_LOOKAHEAD = target_lookahead;

    initialize_variable();

}


/** 
 * Function Name: initialize_variable
 * Description: initialize internal variables and lists
 * Input: void
 * Output: void
 */     
void GridMap::initialize_variable(){

    // lateral
    TARGET_OFFSET_X = TARGET_WIDTH / 2;
    COUNT_CELL_LATERAL = int(TARGET_WIDTH/TARGET_WIDTH_RESOLUTION);

    // longitudinal
    int max_altitude = ls_decoder.calculate_scan_angle(0, MOUNT_HEIGHT, TARGET_LOOKAHEAD);
    int max_scan_altitude = max_altitude - MOUNT_ANGLE;         // consider mountage
    int temp_scan = -15;

    while(temp_scan<=max_scan_altitude+1){
        target_altitude_list.push_back(temp_scan);
        target_y_list.push_back(ls_decoder.calculate_distance(MOUNT_HEIGHT, temp_scan+MOUNT_ANGLE));
        temp_scan++;
    }

    COUNT_SCAN = target_y_list.size();
    COUNT_CELL_LONGITUDINAL = COUNT_SCAN - 1;
    
    // INFO
    cout<<"Longitudinal initialization"<<endl;
    for(int j=0; j<COUNT_SCAN; j++){
        cout<<target_altitude_list[j]<<" degree: "<<target_y_list[j]<<endl;
    }

    // GridRow
    for(int i = 0; i<COUNT_CELL_LONGITUDINAL; i++){
        
        GridRow *gr = new GridRow();
        *gr = GridRow(TARGET_WIDTH, TARGET_WIDTH_RESOLUTION, target_y_list[i], target_altitude_list[i], target_altitude_list[i+1], i);
        row_ptr_list.push_back(gr);

        cout<<"Initialize row "<<i<<endl;

    }

}


/** 
 * Function Name: Update
 * Description: update each row
 * Input:
 *          - pcl::PointCloud<PointT>::Ptr inPtr: pointer to a point cloud Gen1
 *          - double timestamp_ns: ROS time stamp
 * Output: void
 */    
void GridMap::Update(pcl::PointCloud<PointT>::Ptr inPtr, double timestamp_ns){

    for(int i=0; i<COUNT_CELL_LONGITUDINAL; i++){
        cout<<i<<endl;
        row_ptr_list[i]->Update(inPtr, timestamp_ns);
    }

}


/** 
 * Function Name: Plot
 * Description: Visualize grid map in the console
 * Input: void
 * Output: void
 */  
void GridMap::Plot(){

    cout<<setiosflags(ios::fixed)<<setprecision(3);

    cout<<endl;
    cout<<"\t";
    for(int j=0; j<COUNT_CELL_LATERAL; j++){
        if(j%5 == 0) cout<<j;
        else if((j>9 && j%5>1) || (j<=9))cout<<" ";
    } 
    cout<<"\tROW\tSCAN\t-Y"<<endl;

    for(int i=COUNT_CELL_LONGITUDINAL-1; i>=0; i--){
        cout<<"\t";
        for(int j=0; j<COUNT_CELL_LATERAL; j++){
            if(row_ptr_list[i]->detection(j)){
                cout<<"X";
            }
            else cout<<"_";
        }
        cout<<'\t'<<i<<'\t'<<target_altitude_list[i]<<"\t"<< -1*row_ptr_list[i]->ROW_Y<<endl;
    }

    cout<<"   r↑"<<endl;
    cout<<"  (0,0)→  c"<<endl;

    cout<<"\t";
    for(int i=0; i < int(COUNT_CELL_LATERAL/2)-3; i++) cout<<" ";
    cout<<"LIDAR↑"<<endl;

    cout<<"\t";
    for(int i=0; i < int(COUNT_CELL_LATERAL/2)-3; i++) cout<<" ";
    cout<<"X←"<<endl;

    cout<<"\t";
    for(int i=0; i < int(COUNT_CELL_LATERAL/2)-3; i++) cout<<" ";
    cout<<"  ↓ Y"<<endl;

    cout<<endl;

}


/** 
 * Function Name: delta
 * Description: get delta at index 'column'
 * Input: 
 *          - int row: inqury index
 *          - int column: inqury index
 * Output:
 *          - float delta: delta at (row, column)
 */  
float GridMap::delta(int row, int column){

    return row_ptr_list[row]->delta(column);

}



/** 
 * Function Name: map_rows
 * Description: get number of rows
 * Input: void
 * Output:
 *          - int: number of rows
 */  
int GridMap::map_rows(){

    return COUNT_CELL_LONGITUDINAL;

}


/** 
 * Function Name: map_cols
 * Description: get number of columns
 * Input: void
 * Output:
 *          - int: number of columns
 */  
int GridMap::map_cols(){

    return COUNT_CELL_LATERAL;

}






