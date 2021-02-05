/****************************************************************
* Author:                 J. Gong                                                                                  
* Date:                      2021-02-04                                                                           
* Description:        LidarObjectDetector                                                       
*****************************************************************/

// Custom
#include "lidar_object_detector/LidarObjectDetector.h"
#define PI 3.14159

// System
#include<vector>
#include<Eigen/Core>

// ROS
#include<std_msgs/Float32.h>
#include<std_msgs/Float32MultiArray.h>

// PCL conversion
#include<pcl_conversions/pcl_conversions.h>

// PCL
#include<pcl/common/common.h>   
#include<pcl/common/transforms.h>
#include<pcl/features/moment_of_inertia_estimation.h>
#include<pcl/filters/passthrough.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl/filters/crop_box.h>
#include<pcl/filters/extract_indices.h>
#include<pcl/filters/project_inliers.h>
#include<pcl/kdtree/kdtree_flann.h>
#include<pcl/ModelCoefficients.h>
#include<pcl/segmentation/extract_clusters.h>
#include<pcl/segmentation/sac_segmentation.h>



/**************************************************************************
 * Class Name: LidarObjectDetector
 * Description: Object detection through RANSAC and Euclidean clustering
 **************************************************************************/

/** 
 * Function Name: LidarObjectDetector
 * Description: Constructor with ROS node
 * Input:
 *          - ros::NodeHandle nh: ROS node
 */
LidarObjectDetector::LidarObjectDetector(ros::NodeHandle nh){

    // register topic
    pub = nh.advertise<std_msgs::Float32MultiArray>("lidar_object_list", 1000);
    
    // initialize viewer_euclidean
    viewer_euclidean.setWindowName("Euclidean Clustering");
    if(!VISUAL_EUCLIDEAN) viewer_euclidean.close();

    // initialize viewer_ransac
    // viewer_ransac.setWindowName("RANSAC");
    // if(!VISUAL_RANSAC) viewer_ransac.close();

    get_parameter();    

    initialize_variable();

}


/** 
 * Function Name: LidarObjectDetector
 * Description: Default constructor
 * Input: void
 */
LidarObjectDetector::LidarObjectDetector(){
    
    // initialize viewer_euclidean
    viewer_euclidean.setWindowName("Euclidean Clustering");
    if(!VISUAL_EUCLIDEAN) viewer_euclidean.close();

    // initialize viewer_ransac
    // viewer_ransac.setWindowName("RANSAC");
    // if(!VISUAL_RANSAC) viewer_ransac.close();

    get_parameter();    

    initialize_variable();

}


/** 
 * Function Name: detectionCallBack
 * Description:
 *          - Convert ROS message to point cloud
 *          - Downsample
 *          - Crop ROI
 *          - RANSAC to find ground
 *          - Euclidean clustering and BB generation
 *          - publish output
 * Input: 
 *          - const sensor_msgs::PointCloud2ConstPtr& input: input ROS message
 * Output: void
 */
void LidarObjectDetector::detectionCallBack(const sensor_msgs::PointCloud2ConstPtr& input){

    ROS_INFO("#####################################################################################");

    // declear pointers  
    pcl::PCLPointCloud2::Ptr inPtr(new pcl::PCLPointCloud2);                // input
    pcl::PCLPointCloud2::Ptr vgPtr( new pcl::PCLPointCloud2);               // voxel grid
    pcl::PCLPointCloud2::Ptr cbPtr(new pcl::PCLPointCloud2);                // crop box
    pcl::PCLPointCloud2::Ptr ransacPtr(new pcl::PCLPointCloud2);        // RANSAC

    // convert to PCL
    pcl_conversions::toPCL(*input, *inPtr);
    ROS_INFO("Original:\t%d * %d,\t%d points", inPtr->width, inPtr->height, inPtr->width * inPtr->height);

    // downsample
    voxel_grid_filter(inPtr, vgPtr);
    ROS_INFO("VoxelGrid:\t%d*%d,\t%d points", vgPtr->width, vgPtr->height, vgPtr->width*vgPtr->height);

    // crop ROI
    crop_box_filter(vgPtr, cbPtr);
    ROS_INFO("CropBox:\t%d*%d,\t\t%d points", cbPtr->width, cbPtr->height, cbPtr->width*cbPtr->height);

    // run RANSAC
    ransac_ground_filter(cbPtr, ransacPtr);
    ROS_INFO("RANSAC:\t\t%d*%d,\t\t%d points", ransacPtr->width, ransacPtr->height, ransacPtr->width*ransacPtr->height);

    // run Euclidean clustering
    euclidean_cluster(ransacPtr);
    
    // publish output
    pub.publish(output);

}


/** 
 * Function Name: get_parameter
 * Description: Retrieve parameter from ROS parameter server, use default value when fails
 * Input: void
 * Output: void
 */
void LidarObjectDetector::get_parameter(){

    ros::param::param<std::string>("/lidar_object_detection/BOUNDING_BOX_TYPE", s_bounding_box_type, "AABB");
    if(s_bounding_box_type == "AABB") bounding_box_type = AABB;
    else if (s_bounding_box_type == "OBB") bounding_box_type = OBB;
    else bounding_box_type = AABB;
    ROS_INFO("BOUNDING_BOX_TYPE: %d", bounding_box_type);

    // visualization
    ros::param::param<bool>("/lidar_object_detection/VISUAL_RANSAC", VISUAL_RANSAC, false);
    ros::param::param<bool>("/lidar_object_detection/VISUAL_EUCLIDEAN", VISUAL_EUCLIDEAN, true);

    // voxel grid
    ros::param::param<float>("/lidar_object_detection/leaf_size_x", leaf_size_x, 0.02);
    ros::param::param<float>("/lidar_object_detection/leaf_size_y", leaf_size_y, 0.02);
    ros::param::param<float>("/lidar_object_detection/leaf_size_z", leaf_size_z, 0.02);

    // ROI
    ros::param::param<float>("/lidar_object_detection/cb_width", cb_width, 2.0);
    ros::param::param<float>("/lidar_object_detection/cb_height", cb_height, 0.5);
    ros::param::param<float>("/lidar_object_detection/cb_lookahead", cb_lookahead, 10);
    ros::param::param<float>("/lidar_object_detection/cb_lookback", cb_lookback, 0);
    
    // ground ROI
    ros::param::param<float>("/lidar_object_detection/mount_height", mount_height, 0.9);
    ros::param::param<float>("/lidar_object_detection/ground_lower_tolerance", ground_lower_tolerance, 0.1);
    ros::param::param<float>("/lidar_object_detection/ground_upper_tolerance", ground_upper_tolerance, 0.1);

    // RANSAC
    ros::param::param<int>("/lidar_object_detection/ransac_max_iter", ransac_max_iter, 100);
    ros::param::param<float>("/lidar_object_detection/ransac_distance_thresh", ransac_distance_thresh, 0.05);

    // Euclidean clustering
    ros::param::param<float>("/lidar_object_detection/ec_cluster_tolerance", ec_cluster_tolerance, 0.3);
    ros::param::param<int>("/lidar_object_detection/ec_min_cluster_size", ec_min_cluster_size, 20);
    ros::param::param<int>("/lidar_object_detection/ec_max_cluster_size", ec_max_cluster_size, 2000);

    // Lidar size
    ros::param::param<float>("/lidar_object_detection/ls_radius", ls_radius, 0.06);
    ros::param::param<float>("/lidar_object_detection/ls_height", ls_height, 0.11);   

}


/** 
 * Function Name: initialize_variable
 * Description: Initialize variables from parameters
 * Input: void
 * Output: void
 */
void LidarObjectDetector::initialize_variable(){

    // ROI
    cb_x_min = -0.5 * cb_width;
    cb_x_max = 0.5 * cb_width;
    cb_y_min = -1 * cb_lookahead;
    cb_y_max = cb_lookback;
    cb_z_min = -1 * mount_height - ground_lower_tolerance;
    cb_z_max = cb_z_min + cb_height;

    // ground
    ground_x_min = -0.5 * cb_width;
    ground_x_max = 0.5 * cb_width;
    ground_y_min = cb_y_min;
    ground_y_max = cb_y_max;
    ground_z_min = cb_z_min;
    ground_z_max = -1 * mount_height + ground_upper_tolerance;

}


/** 
 * Function Name: initialize_viewer
 * Description: Initialize PCL viewer with ROI, axis and Lidar
 * Input: 
 *          - pcl::visualization::PCLVisualizer& viewer: viewer pointer
 * Output: void
 */
void LidarObjectDetector::initialize_viewer(pcl::visualization::PCLVisualizer& viewer){

    // clear
    viewer.removeAllPointClouds();
    viewer.removeAllShapes();
    viewer.removeAllCoordinateSystems();
    
    // add ROI
    viewer.addCube(cb_x_min, cb_x_max, cb_y_min, cb_y_max, cb_z_min, cb_z_max, 0.0, 1.0, 1.0, "ROI");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "ROI");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "ROI");
    
    // add axis
    viewer.addCoordinateSystem();

    // add Lidar
    pcl::ModelCoefficients cylinder_coeff;
    cylinder_coeff.values.resize (7);    // 7 values to define a cylinder
    cylinder_coeff.values[0] = 0;
    cylinder_coeff.values[1] = 0;
    cylinder_coeff.values[2] = -1*ls_height/2;
    cylinder_coeff.values[3] = 0;
    cylinder_coeff.values[4] = 0;
    cylinder_coeff.values[5] = ls_height;
    cylinder_coeff.values[6] = ls_radius;
    viewer.addCylinder (cylinder_coeff, "lidar");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.2, 0.2, 0.2, "lidar");

}


/** 
 * Function Name: voxel_grid_filter
 * Description: Apply voxel grid filter
 * Input: 
 *          - pcl::PCLPointCloud2Ptr inPtr: input pointcloud pointer
 * Output: void
 *          - pcl::PCLPointCloud2Ptr outPtr: filtered pointcloud pointer
 */
void LidarObjectDetector::voxel_grid_filter(pcl::PCLPointCloud2Ptr inPtr, pcl::PCLPointCloud2Ptr outPtr){

    pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
    vg.setInputCloud(inPtr);
    vg.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
    vg.filter(*outPtr);

}


/** 
 * Function Name: crop_box_filter
 * Description: Apply crop box
 * Input: 
 *          - pcl::PCLPointCloud2Ptr inPtr: input pointcloud pointer
 * Output: void
 *          - pcl::PCLPointCloud2Ptr outPtr: filtered pointcloud pointer
 */
void LidarObjectDetector::crop_box_filter(pcl::PCLPointCloud2Ptr inPtr, pcl::PCLPointCloud2Ptr outPtr){

    pcl::CropBox<pcl::PCLPointCloud2> cb;
    cb.setMin(Eigen::Vector4f(cb_x_min, cb_y_min, cb_z_min, 1.0));
    cb.setMax(Eigen::Vector4f(cb_x_max, cb_y_max, cb_z_max, 1.0));
    cb.setInputCloud(inPtr);
    cb.filter(*outPtr);

}


/** 
 * Function Name: crop_box_filter
 * Description:
 *          - Find the biggest horizontal cluster through RANSAC, considered groud
 *          - Filter ground from the point cloud
 * Input: 
 *          - pcl::PCLPointCloud2Ptr inPtr: input pointcloud pointer
 * Output: void
 *          - pcl::PCLPointCloud2Ptr outPtr: filtered pointcloud pointer, ground remove
 */
void LidarObjectDetector::ransac_ground_filter(pcl::PCLPointCloud2Ptr inPtr, pcl::PCLPointCloud2Ptr outPtr){

    // declear ground detect bool variable
    bool ground_detected = false;

    // clear viewer_ransac
    // if(VISUAL_RANSAC && !viewer_ransac.wasStopped()) viewer_ransac.removeAllPointClouds();

    // clear viewer_euclidean
     if(VISUAL_EUCLIDEAN){
         initialize_viewer(viewer_euclidean);
     } 

    // crop ground
    pcl::CropBox<pcl::PCLPointCloud2> cb;
    cb.setMin(Eigen::Vector4f(ground_x_min, ground_y_min, ground_z_min, 1.0));
    cb.setMax(Eigen::Vector4f(ground_x_max, ground_y_max, ground_z_max, 1.0));
    cb.setInputCloud(inPtr);
    cb.setKeepOrganized(true);
    pcl::PCLPointCloud2::Ptr temp(new pcl::PCLPointCloud2);
    cb.filter(*temp);

    // convert from PCLPointCloud2 to PointCloud<PointT>
    pcl::PointCloud<PointT>::Ptr cbXYZPtr( new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr originalXYZPtr( new pcl::PointCloud<PointT>);
    pcl::fromPCLPointCloud2(*temp, *cbXYZPtr);          // from cropped input
    pcl::fromPCLPointCloud2(*inPtr, *originalXYZPtr);

    // initialize RANSAC object
    pcl::SACSegmentation<PointT> sac;
    sac.setInputCloud(cbXYZPtr);
    sac.setMethodType(ransac_method_type);
    sac.setModelType(ransac_model_type);
    sac.setMaxIterations(ransac_max_iter);
    sac.setDistanceThreshold(ransac_distance_thresh);

    // declear variables
    pcl::ModelCoefficients::Ptr coefficient(new pcl::ModelCoefficients);                                    // model coefficients
    pcl::PointIndices::Ptr inlier (new pcl::PointIndices);                                                                      // indices of inliers
    pcl::PointCloud<PointT>::Ptr ext_cloud(new pcl::PointCloud<PointT>);                            // pointer to segemented point cloud
    pcl::PointCloud<PointT>::Ptr ext_cloud_rest(new pcl::PointCloud<PointT>);                  // pointer to rest point cloud
    pcl::PointCloud<PointT>::Ptr ext_output(new pcl::PointCloud<PointT>);                           // pointer to rest point cloud
    pcl::ExtractIndices<PointT> ext;                                                                                                             // object to extract point cloud of certain indices

    // initialize loop
    int i = cbXYZPtr->size(), j = 0;
    
    // start loop if current point cloud has at least 3 valid points
    while(cbXYZPtr->size() > 3 && !ground_detected){
        
        // update remaining point cloud
        ext.setInputCloud(cbXYZPtr);

        // run RANSAC, break if there is no inlier
        sac.segment(*inlier, *coefficient);
        if(inlier->indices.size()==0) break;

        // report plane information
        ROS_INFO("Plane %d:\ta=%f\tb=%f\tc=%f,\td=%f", j+1, coefficient->values[0], coefficient->values[1], coefficient->values[2], coefficient->values[3]);

        // set extracter indices
        ext.setIndices(inlier);

        // extract inlier to ex_cloud
        ext.setNegative(false);
        ext.filter(*ext_cloud);

        //extract outlier to ex_cloud_rest
        ext.setNegative(true);
        ext.filter(*ext_cloud_rest);

        // update remaining point cloud
        *cbXYZPtr = *ext_cloud_rest;

        // horizontal plane and ground not detected yet
        if(isgreater(abs(coefficient->values[2]), 0.98) && (!ground_detected)){

            // ground detected
            ground_detected = true;

            // visualize ground in viewer_ransac
            // if(VISUAL_RANSAC && !viewer_ransac.wasStopped()){
            //     pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb1(ext_cloud, 255, 255, 255);
            //     viewer_ransac.addPointCloud(ext_cloud, rgb1, "pc_ground");
            // }

            // visualize ground in viewer_euclidean
             if(VISUAL_EUCLIDEAN){
                pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb1(ext_cloud, 255, 255, 255);
                viewer_euclidean.addPointCloud(ext_cloud, rgb1, "pc_ground");
            }

            // use current indices to filter out ground from original point cloud
            ext.setInputCloud(originalXYZPtr);
            ext.setNegative(true);
            ext.filter(*ext_output);

            // convert filtered point cloud to ROS msg
            pcl::toPCLPointCloud2(*ext_output, *outPtr);

        }

        // all other planes, visualize in viewer_ransac only
        // else if(VISUAL_RANSAC && !viewer_ransac.wasStopped()){

        //     // get random color and create ColorHandler
        //     int* rgb = rand_rgb();
        //     pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb1(ext_cloud, rgb[0], rgb[1], rgb[2]);
        //     delete[] rgb;
        //     viewer_ransac.addPointCloud(ext_cloud, rgb1, std::to_string(j));

        // }

        // couting variable to avoid name conflict
        j++;

    }

    // update viewer_ransac once (ROS node is spinning)
    // if(VISUAL_RANSAC && !viewer_ransac.wasStopped()) viewer_ransac.spinOnce();   

}


/** 
 * Function Name: write_to_output
 * Description: Generate a output vector, input in AABB form
 * Input: 
 *          - PointT minPoint: min point of an AABB bounding box
 *          - PointT maxPoint: max point of an AABB bounding box
 * Output: 
 *          - std::vector<float>& output_data: pointer to output data vector
 */
void LidarObjectDetector::write_to_output(std::vector<float>& output_data, PointT minPoint, PointT maxPoint){

    // object center
    output_data.push_back(0.5* (minPoint.x + maxPoint.x));
    output_data.push_back(0.5 * (minPoint.y + maxPoint.y));
    output_data.push_back(0.5 * (minPoint.z + maxPoint.z));

    // object size
    output_data.push_back(maxPoint.x - minPoint.x);
    output_data.push_back(maxPoint.y - minPoint.y);
    output_data.push_back(maxPoint.z - minPoint.z);

    // rotation in angle(grad)
    output_data.push_back(0.0);
    output_data.push_back(0.0);
    output_data.push_back(0.0);
    
}


/** 
 * Function Name: write_to_output
 * Description: Generate a output vector, input in OBB form
 * Input: 
 *          - PointT minPoint: min point of an OBB bounding box
 *          - PointT maxPoint: max point of an OBB bounding box
 *          - Eigen::Vector3f &position: position of an OBB
 *          - Eigen::Quaternionf &q: orientation of an OBB in quaternion form
 * Output: 
 *          - std::vector<float>& output_data: pointer to output data vector
 */
void LidarObjectDetector::write_to_output(std::vector<float>& output_data, PointT minPoint, PointT maxPoint, Eigen::Vector3f &position, Eigen::Quaternionf &q ){

    // object center
    output_data.push_back(position(0));
    output_data.push_back(position(1));
    output_data.push_back(position(2));

    // object size
    output_data.push_back(maxPoint.x - minPoint.x);
    output_data.push_back(maxPoint.y - minPoint.y);
    output_data.push_back(maxPoint.z - minPoint.z);

    Eigen::Vector3f euler = q.matrix().eulerAngles(0,1,2);
    // rotation in angle(degree)
    output_data.push_back(euler(0)/PI*180);
    output_data.push_back(euler(1)/PI*180);
    output_data.push_back(euler(2)/PI*180);
    
}


/** 
 * Function Name: euclidean_cluster
 * Description: 
 *          - Apply Euclidean clustering to a point cloud
 *          - Generate a bounding box for each cluster
 * Input: 
 *          - pcl::PCLPointCloud2Ptr inPt: pointer to input pointcloud
 * Output: void
 */
void LidarObjectDetector::euclidean_cluster(pcl::PCLPointCloud2Ptr inPtr){

    // clear output
    output.data.clear();
    
    // convert from PCLPointCloud2 to PointCloud<PointT>
    pcl::PointCloud<PointT>::Ptr XYZPtr( new pcl::PointCloud<PointT>);
    pcl::fromPCLPointCloud2(*inPtr, *XYZPtr);

    // declear variables
    std::vector<pcl::PointIndices>  ece_inlier;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    pcl::EuclideanClusterExtraction<PointT> ece;
    pcl::ExtractIndices<PointT> ext;

    // initialize Euclidean Cluster object
    ece.setInputCloud(XYZPtr);
    ece.setClusterTolerance(ec_cluster_tolerance);
    ece.setMinClusterSize(ec_min_cluster_size);
    ece.setMaxClusterSize(ec_max_cluster_size);
    ece.setSearchMethod(tree);
    ece.extract(ece_inlier);

    // initialize extractor object
    ext.setInputCloud(XYZPtr);
    ext.setNegative(true);

    ROS_INFO("Found %d clusters", ece_inlier.size());

    // loop through clusters
    for(int i=0; i<ece_inlier.size(); i++){

        // extract cluster point cloud
        pcl::PointCloud<PointT>::Ptr ext_cloud(new pcl::PointCloud<PointT>);
        std::vector<int> ece_inlier_ext = ece_inlier[i].indices;
        pcl::copyPointCloud(*XYZPtr, ece_inlier_ext, *ext_cloud);       // "ext.setIndices" requires a Boost shared_ptr, use "copyPointCloud" instead
        
        // generate bounding box
        generate_bounding_box(ext_cloud, i);

    }

    // update viewer_euclidean once
     if(VISUAL_EUCLIDEAN) viewer_euclidean.spinOnce();
     viewer_euclidean.spinOnce();

}


/** 
 * Function Name: visualize_point_cloud
 * Description: Visualize a point cloud
 * Input: 
 *          - pcl::visualization::PCLVisualizer& viewer: PCL viewer
 *          - pcl::PointCloud<PointT>::Ptr cloud: point cloud
 *          - int id: identifer, converted to string when add to viewer
 * Output: void
 */
void LidarObjectDetector::visualize_point_cloud(pcl::visualization::PCLVisualizer& viewer, pcl::PointCloud<PointT>::Ptr cloud, int id){

    pcl::visualization::PointCloudColorHandlerRandom<PointT> rc(cloud);
    viewer.addPointCloud(cloud, rc, std::to_string(id));
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, std::to_string(id));
            
}


/** 
 * Function Name: visualize_cube
 * Description: Visualize an AABB
 * Input: 
 *          - pcl::visualization::PCLVisualizer& viewer: PCL viewer
 *          - PointT minPoint: min point of an AABB bounding box
 *          - PointT maxPoint: max point of an AABB bounding box
 * Output: void
 */
void LidarObjectDetector::visualize_cube(pcl::visualization::PCLVisualizer& viewer, PointT minPoint, PointT maxPoint, int id){

    // visualize transparent cube
    viewer.addCube(minPoint.x, maxPoint.x, minPoint.y, maxPoint.y, minPoint.z, maxPoint.z, 1.0, 0.0, 0.0, std::to_string(id+100));
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, std::to_string(id+100));

    //vidusalize cube wireframe
    viewer.addCube(minPoint.x, maxPoint.x, minPoint.y, maxPoint.y, minPoint.z, maxPoint.z, 1.0, 0.0, 0.0, std::to_string(id+200));
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, std::to_string(id+200));
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, std::to_string(id+200));

}


/** 
 * Function Name: visualize_cube
 * Description: Visualize an OBB
 * Input: 
 *          - pcl::visualization::PCLVisualizer& viewer: PCL viewer
 *          - PointT minPoint: min point of an OBB bounding box
 *          - PointT maxPoint: max point of an OBB bounding box
 *          - Eigen::Vector3f &position: position of an OBB
 *          - Eigen::Quaternionf &q: orientation of an OBB in quaternion form
 * Output: void
 */
void LidarObjectDetector::visualize_cube(pcl::visualization::PCLVisualizer& viewer, PointT minPoint, PointT maxPoint, Eigen::Vector3f position, Eigen::Quaternionf q, int id){

    // visualize transparent cube
    viewer.addCube(position, q, maxPoint.x-minPoint.x, maxPoint.y-minPoint.y, maxPoint.z-minPoint.z, std::to_string(id+100));
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, std::to_string(id+100));
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, std::to_string(id+100));

    //vidusalize cube wireframe
    viewer.addCube(position, q, maxPoint.x-minPoint.x, maxPoint.y-minPoint.y, maxPoint.z-minPoint.z, std::to_string(id+200));
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, std::to_string(id+200));
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, std::to_string(id+200));
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, std::to_string(id+200));

}


/** 
 * Function Name: generate_bounding_box
 * Description: 
 *          - Generate a bounding box
 *          - Write to output list
 *          - Visualize point cloud and bounding box
 * Input: 
 *          - pcl::PointCloud<PointT>::Ptr inPtr: input point cloud
 *          - int id: identifier for visulization
 * Output: void
 */
void LidarObjectDetector::generate_bounding_box(pcl::PointCloud<PointT>::Ptr inPtr, int id){

    // both types need to find AABB
    pcl::MomentOfInertiaEstimation<PointT> fe_original;
    fe_original.setInputCloud(inPtr);
    fe_original.compute();
    PointT minAABB, maxAABB;
    fe_original.getAABB(minAABB, maxAABB);

    // visualize point cloud
    if(VISUAL_EUCLIDEAN) visualize_point_cloud(viewer_euclidean, inPtr, id);
    
    // AABB, visualize and write to output
    if(bounding_box_type == AABB){
        
        // visualize cube
        if(VISUAL_EUCLIDEAN) visualize_cube(viewer_euclidean, minAABB, maxAABB, id);

        // add to output
        write_to_output(output.data, minAABB, maxAABB);

    }

    // OBB, compute projected OBB, visualize and write to output
    else if (bounding_box_type == OBB){

        // project to horizontal
        pcl::PointCloud<PointT>::Ptr projectedPtr( new pcl::PointCloud<PointT>);
        project_horizontal(inPtr, projectedPtr);

        // get OBB from feature extractor
        pcl::MomentOfInertiaEstimation<PointT> fe_projected;
        fe_projected.setInputCloud(projectedPtr);
        fe_projected.compute();
        PointT minOBB, maxOBB, positionOBB;
        Eigen::Matrix3f rotationOBB;
        fe_projected.getOBB(minOBB, maxOBB, positionOBB, rotationOBB);

        // use height infromation from AABB
        maxOBB.z = maxAABB.z;
        minOBB.z = minAABB.z;
        Eigen::Vector3f position(positionOBB.x, positionOBB.y, 0.5* (maxOBB.z+minOBB.z));
        Eigen::Quaternionf q(rotationOBB);

        // visualize
        if(VISUAL_EUCLIDEAN) visualize_cube(viewer_euclidean, minOBB, maxOBB, position, q, id);

        // add to output
        write_to_output(output.data, minOBB, maxOBB, position, q);

    }

}


/** 
 * Function Name: project_horizontal
 * Description: Project point cloud to plane z=0
 * Input: 
 *          - pcl::PointCloud<PointT>::Ptr inPtr: pointer to input point cloud
 *          - pcl::PointCloud<PointT>::Ptr outPtr: pointer to projected point cloud
 * Output: void
 */
void LidarObjectDetector::project_horizontal(pcl::PointCloud<PointT>::Ptr inPtr, pcl::PointCloud<PointT>::Ptr outPtr){

    pcl::ModelCoefficients::Ptr horizontal_plane_ptr(new pcl::ModelCoefficients());
    horizontal_plane_ptr->values.resize(4);
    horizontal_plane_ptr->values[0] = 0;
    horizontal_plane_ptr->values[1] = 0;
    horizontal_plane_ptr->values[2] = 1;
    horizontal_plane_ptr->values[3] = 0;
    
    pcl::ProjectInliers<PointT> proj;

    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(inPtr);
    proj.setModelCoefficients(horizontal_plane_ptr);
    proj.filter(*outPtr);

}



