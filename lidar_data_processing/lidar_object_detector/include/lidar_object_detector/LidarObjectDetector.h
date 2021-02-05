/****************************************************************
* Author:                 J. Gong                                                                                  
* Date:                      2021-02-04                                                                           
* Description:        LidarObjectDetector                                                       
*****************************************************************/

#pragma once

// System
#include<string>

// ROS
#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<std_msgs/Float32.h>
#include<std_msgs/Float32MultiArray.h>

// PCL 
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/sample_consensus/method_types.h>
#include<pcl/sample_consensus/model_types.h>

// Eigen
#include<Eigen/Core>


// define point type
typedef pcl::PointXYZ PointT;


/**************************************************************************
 * Class Name: LidarObjectDetector
 * Description: Object detection through RANSAC and Euclidean clustering
 **************************************************************************/

class LidarObjectDetector{

    private:

        ros::Publisher pub;
        // pcl::visualization::PCLVisualizer viewer_ransac;
        pcl::visualization::PCLVisualizer viewer_euclidean; 
        std_msgs::Float32MultiArray output;

        // enum
        enum BOUNDING_BOX_TYPE{AABB, OBB};

        // visualization control variables
        bool VISUAL_RANSAC; 
        bool VISUAL_EUCLIDEAN;

        // bounding box type
        std::string s_bounding_box_type;
        int bounding_box_type;

        // voxel grid
        float leaf_size_x; 
        float leaf_size_y;
        float leaf_size_z;

        // crop box (Ground)
        float mount_height;
        float ground_lower_tolerance;
        float ground_upper_tolerance;
        float ground_x_min; 
        float ground_y_min; 
        float ground_z_min; 
        float ground_x_max;
        float ground_y_max; 
        float ground_z_max; 

        // crop box (ROI)
        float cb_width;
        float cb_height;
        float cb_lookahead;
        float cb_lookback;
        float cb_x_min; 
        float cb_y_min; 
        float cb_z_min; 
        float cb_x_max; 
        float cb_y_max; 
        float cb_z_max; 

        // RANSAC
        int ransac_method_type = pcl:: SAC_RANSAC;
        int ransac_model_type = pcl:: SACMODEL_PLANE;
        int ransac_max_iter;
        float ransac_distance_thresh;

        // Euclidean clustering
        float ec_cluster_tolerance;
        int ec_min_cluster_size;
        int ec_max_cluster_size;

        // Lidar size
        float ls_radius;
        float ls_height;

        // visualization parameter
        int point_size = 2;


        /** 
         * Function Name: get_parameter
         * Description: Retrieve parameter from ROS parameter server, use default value when fails
         * Input: void
         * Output: void
         */
        void get_parameter();


        /** 
         * Function Name: initialize_variable
         * Description: Initialize variables from parameters
         * Input: void
         * Output: void
         */       
        void initialize_variable();


        /** 
         * Function Name: initialize_viewer
         * Description: Initialize PCL viewer with ROI, axis and Lidar
         * Input: 
         *          - pcl::visualization::PCLVisualizer& viewer: viewer pointer
         * Output: void
         */
        void initialize_viewer(pcl::visualization::PCLVisualizer& viewer);


        /** 
         * Function Name: voxel_grid_filter
         * Description: Apply voxel grid filter
         * Input: 
         *          - pcl::PCLPointCloud2Ptr inPtr: input pointcloud pointer
         * Output: void
         *          - pcl::PCLPointCloud2Ptr outPtr: filtered pointcloud pointer
         */
        void voxel_grid_filter(pcl::PCLPointCloud2Ptr inPtr, pcl::PCLPointCloud2Ptr outPtr);


        /** 
         * Function Name: crop_box_filter
         * Description: Apply crop box
         * Input: 
         *          - pcl::PCLPointCloud2Ptr inPtr: input pointcloud pointer
         * Output: void
         *          - pcl::PCLPointCloud2Ptr outPtr: filtered pointcloud pointer
         */
        void crop_box_filter(pcl::PCLPointCloud2Ptr inPtr, pcl::PCLPointCloud2Ptr outPtr);


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
        void ransac_ground_filter(pcl::PCLPointCloud2Ptr inPtr, pcl::PCLPointCloud2Ptr outPtr);


        /** 
         * Function Name: write_to_output
         * Description: Generate a output vector, input in AABB form
         * Input: 
         *          - PointT minPoint: min point of an AABB bounding box
         *          - PointT maxPoint: max point of an AABB bounding box
         * Output: 
         *          - std::vector<float>& output_data: pointer to output data vector
         */
        void write_to_output(std::vector<float>& output_data, PointT minPoint, PointT maxPoint);


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
        void write_to_output(std::vector<float>& output_data, PointT minPoint, PointT maxPoint, Eigen::Vector3f &position, Eigen::Quaternionf &q );


        /** 
         * Function Name: euclidean_cluster
         * Description: 
         *          - Apply Euclidean clustering to a point cloud
         *          - Generate a bounding box for each cluster
         * Input: 
         *          - pcl::PCLPointCloud2Ptr inPt: pointer to input pointcloud
         * Output: void
         */
        void euclidean_cluster(pcl::PCLPointCloud2Ptr inPtr);


        /** 
         * Function Name: visualize_point_cloud
         * Description: Visualize a point cloud
         * Input: 
         *          - pcl::visualization::PCLVisualizer& viewer: PCL viewer
         *          - pcl::PointCloud<PointT>::Ptr cloud: point cloud
         *          - int id: identifer, converted to string when add to viewer
         * Output: void
         */
        void visualize_point_cloud(pcl::visualization::PCLVisualizer& viewer, pcl::PointCloud<PointT>::Ptr cloud, int id);


        /** 
         * Function Name: visualize_cube
         * Description: Visualize an AABB
         * Input: 
         *          - pcl::visualization::PCLVisualizer& viewer: PCL viewer
         *          - PointT minPoint: min point of an AABB bounding box
         *          - PointT maxPoint: max point of an AABB bounding box
         * Output: void
         */
        void visualize_cube(pcl::visualization::PCLVisualizer& viewer, PointT minPoint, PointT maxPoint, int id);


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
        void visualize_cube(pcl::visualization::PCLVisualizer& viewer, PointT minPoint, PointT maxPoint, Eigen::Vector3f position, Eigen::Quaternionf q, int id);


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
        void generate_bounding_box(pcl::PointCloud<PointT>::Ptr inPtr, int id);


        /** 
         * Function Name: project_horizontal
         * Description: Project point cloud to plane z=0
         * Input: 
         *          - pcl::PointCloud<PointT>::Ptr inPtr: pointer to input point cloud
         *          - pcl::PointCloud<PointT>::Ptr outPtr: pointer to projected point cloud
         * Output: void
         */
        void project_horizontal(pcl::PointCloud<PointT>::Ptr inPtr, pcl::PointCloud<PointT>::Ptr outPtr);



    public:

        /** 
         * Function Name: LidarObjectDetector
         * Description: Default constructor
         * Input: void
         */
        LidarObjectDetector();


        /** 
         * Function Name: LidarObjectDetector
         * Description: Constructor with ROS node
         * Input:
         *          - ros::NodeHandle nh: ROS node
         */        
        LidarObjectDetector(ros::NodeHandle nh);


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
        void detectionCallBack(const sensor_msgs::PointCloud2ConstPtr& input);

};