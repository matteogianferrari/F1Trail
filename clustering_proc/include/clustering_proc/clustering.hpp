#ifndef CLUSTERING_H_
#define CLUSTERING_H_

#include "../include/cluster_data.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"



// struct VoxelConf {
//     float lx;
//     float ly;
//     float lz;
// };


// struct CropBoxConf {
//     Eigen::Vector4f minPoint;
//     Eigen::Vector4f maxPoint;
// };


// struct ClusteringConf {
//     double ecTolerance;
//     size_t minClusterSize;
//     size_t maxClusterSize;
// };


/**
 * @class 			ClusteringNode
 * @brief 			ROS2 node that performs point cloud clustering using a 2D Lidar scan.
 * 
 * @details  		node subscribes to a Lidar scan topic,
 * 					converts it into a 3D point cloud,
 * 					and processes it using downsampling, cropping,
 * 					and clustering techniques to detect objects in the scan.
 */
class ClusteringNode : public rclcpp::Node
{
public:
	/**
	* @fn     		ClusteringNode
	* 
	* @brief  		Constructs a new ClusteringNode object.
	*
	* @details		Initializes the node, declares parameters,
	*				and sets up the necessary publishers and subscribers.
	*/
    ClusteringNode();


private:
	/**
	 * @fn 			laserScanCallback
	 * 
	 * @brief		Callback function for receiving and processing Lidar scan data.
	 *
	 * @details 	Converts the 2D Lidar scan to a 3D point cloud, filters it,
	 * 				and applies clustering to detect objects.
	 * 
	 * @param[in]	msg LaserScan message containing the Lidar data.
	 */
	void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);


	/**
	 * @fn 			readLidarScan
	 * 
	 * @brief		Converts a 2D Lidar scan into a 3D point cloud.
	 *
	 * @details 	Each point in the Lidar scan is projected into 3D space
	 * 				by calculating its X and Y coordinates
	 * 				using polar coordinates, while the Z coordinate is set to zero.
	 * 
	 * @param[in]	scan LaserScan message containing the Lidar data.
	 * @param[in] 	outputPC Pointer to the output 3D point cloud.
	 */
	void readLidarScan(const sensor_msgs::msg::LaserScan::SharedPtr scan, pcl::PointCloud<pcl::PointXYZ>::Ptr& outputPC);


	/**
	 * @fn			reduceDensityPC
	 *
	 * @brief 		Reduces the density of the input point cloud using a VoxelGrid filter.
	 * 
	 * @details 	Downsamples the point cloud to reduce the number of points,
	 * 				improving processing speed without significantly losing geometric information.
	 *
	 * @param[in]	inputPC Pointer to the input point cloud.
	 * @param[in] 	outputPC Pointer to the output 3D point cloud.
	 */
	void reduceDensityPC(pcl::PointCloud<pcl::PointXYZ>::Ptr& inputPC, pcl::PointCloud<pcl::PointXYZ>::Ptr& outputPC);


	/**
	 * @fn 			cropFOVPC
	 *
	 * @brief 		Crops the input point cloud to focus on the region of interest.
	 * 
	 * @details		Applies a CropBox filter to remove points outside the desired field of view,
	 * 				defined by the min and max points in 3D space.
	 *
	 * @param[in]	inputPC Pointer to the input point cloud.
	 * @param[in] 	outputPC Pointer to the output 3D point cloud.
	 */
    void cropFOVPC(pcl::PointCloud<pcl::PointXYZ>::Ptr& inputPC, pcl::PointCloud<pcl::PointXYZ>::Ptr& outputPC);


    /**
	 * @fn 			applyClustering
	 *
	 * @brief 		Applies Euclidean Clustering to group points into distinct clusters.
	 * 
	 * @details		Segments the point cloud into clusters based on
	 * 				the Euclidean distance between points.
     * 				Each cluster represents a potential object in the scene.
	 *
	 * @param[in]	inputPC Pointer to the input point cloud.
	 * @param[in] 	clusterIndices Vector of indices corresponding to points in each cluster.
	 * @param[in] 	clusters Vector of ClusterData objects, representing the detected clusters.
	 */
    void applyClustering(pcl::PointCloud<pcl::PointXYZ>::Ptr& inputPC,
                        std::vector<pcl::PointIndices>& clusterIndices,
                        std::vector<struct ClusterData>& clusters
    );


    /**
	 * @fn 			detectObject
	 *
	 * @brief 		Detects and identifies the object closest to a specified target point.
	 * 
	 * @details		Searches through the clusters to find the one closest to the target point.
	 * 				The index of the detected object is returned.
	 *
	 * @param[in]	clusters Vector of ClusterData objects representing detected clusters.
	 * @param[in] 	target The target point XYZ to which the object distance is measured.
	 * @param[in] 	objectIndex Index of the closest object in the clusters vector.
	 */
    void detectObject(std::vector<struct ClusterData>& clusters, pcl::PointXYZ target, int& objectIndex);



	// Member variables for processing
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid_;                               /**< VoxelGrid filter for downsampling the point cloud. */
    pcl::CropBox<pcl::PointXYZ> cropBox_;                                   /**< CropBox filter for limiting the point cloud to a region of interest. */
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclideanClustering_;    /**< Euclidean Clustering object for detecting point cloud clusters. */

	
	std::array<double,3> voxelConf_;      /**< VoxelGrid leaf sizes for downsampling in X, Y, and Z axes. */
    Eigen::Vector4f cb_minPoint_;         /**< Minimum point for the CropBox filter. */
    Eigen::Vector4f cb_maxPoint_;         /**< Maximum point for the CropBox filter. */
    double ecTolerance;                   /**< Tolerance for Euclidean Clustering (maximum distance between points in a cluster). */
    size_t minClusterSize;                /**< Minimum number of points required to form a cluster. */
    size_t maxClusterSize;                /**< Maximum number of points allowed in a cluster. */
};

#endif  // CLUSTERING_H_
