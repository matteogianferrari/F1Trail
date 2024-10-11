#ifndef CLUSTERING_H_
#define CLUSTERING_H_

#include "../include/cluster_data.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "rclcpp/rclcpp.hpp"

//#include "sensor_msgs/msg/laser_scan.hpp"


struct VoxelConf {
    float lx;
    float ly;
    float lz;
};


struct CropBoxConf {
    Eigen::Vector4f minPoint;
    Eigen::Vector4f maxPoint;
};


struct ClusteringConf {
    double ecTolerance;
    size_t minClusterSize;
    size_t maxClusterSize;
};


class ClusteringNode : public rclcpp::Node
{
public:
	/**
	* @fn     ClusteringNode
	* 
	* @brief  Constructs a new ClusteringNode object.
	*/
    ClusteringNode();

private:
	/**
	 * @fn 			readPCDFile
	 * 
	 * @brief		Callback function for reading 2D point cloud from PCD file.
	 *
	 * @details 	This callback is called when two images with the same timestamp
	 *				are published on their respective topics.
	 * 
	 * @param[in]	fileName PCD file name.
	 * @param[in] 	outputPC output cloud.
	 */
	void readPCDFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZ>::Ptr& outputPC);;

	/**
	 * @fn			reduceDensityPC
	 *
	 * @brief 		Target detection.
	 * 
	 * @details		Detection is performed with opencv's ArUco marker library.
	 *				The marker needs to be detected in both images.
	 *				The projected center of the marker is found for each image
	 *				and it is used to compute the disparity.
	 *
	 * @param[in]	inputPC input cloud.
	 * @param[in] 	outputPC output cloud.
	 */
	void reduceDensityPC(pcl::PointCloud<pcl::PointXYZ>::Ptr& inputPC, pcl::PointCloud<pcl::PointXYZ>::Ptr& outputPC);

	/**
	 * @fn 			cropFOVPC
	 *
	 * @brief 		Computes target's center.
	 * 
	 * @details		The center is computed as the mean of corners' coordinates.
	 *
	 * @param[in]	inputPC input cloud.
	 * @param[in] 	outputPC output cloud.
	 */
    void cropFOVPC(pcl::PointCloud<pcl::PointXYZ>::Ptr& inputPC, pcl::PointCloud<pcl::PointXYZ>::Ptr& outputPC);

    /**
	 * @fn 			applyClustering
	 *
	 * @brief 		Computes target's center.
	 * 
	 * @details		The center is computed as the mean of corners' coordinates.
	 *
	 * @param[in]	inputPC input cloud.
	 * @param[in] 	clusterIndices cluster points indices.
	 * @param[in] 	clusters vector of point clouds.
	 */
    void applyClustering(pcl::PointCloud<pcl::PointXYZ>::Ptr& inputPC,
                        std::vector<pcl::PointIndices>& clusterIndices,
                        std::vector<struct ClusterData>& clusters
    );

    /**
	 * @fn 			detectObject
	 *
	 * @brief 		Computes target's center.
	 * 
	 * @details		The center is computed as the mean of corners' coordinates.
	 *
	 * @param[in]	clusters vector of point clouds.
	 * @param[in] 	target Point XYZ.
	 * @param[in] 	objectIndex index of object inside vector.
	 */
    void detectObject(std::vector<struct ClusterData>& clusters, pcl::PointXYZ target, int& objectIndex);

    /**
	 * @fn 			publishObjectCoords
	 *
	 * @brief 		Computes target's center.
	 * 
	 * @details		The center is computed as the mean of corners' coordinates.
	 */
    void publishObjectCoords();


    pcl::PCDReader reader_;                                                 /**< */
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid_;                               /**< */
    pcl::CropBox<pcl::PointXYZ> cropBox_;                                   /**< */
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclideanClustering_;    /**< Focal length.*/

	std::array<double,3> voxelConf_;
	Eigen::Vector4f cb_minPoint_;
    Eigen::Vector4f cb_maxPoint_;
	double ecTolerance;
    size_t minClusterSize;
    size_t maxClusterSize;
};

#endif  // CLUSTERING_H_
