#ifndef CLUSTERING_H_
#define CLUSTERING_H_

#include "../include/cluster_data.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

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


class Clustering {
 public:
    Clustering(struct VoxelConf vConf, struct CropBoxConf cbConf, struct ClusteringConf cConf);
    ~Clustering();

    // void readLidarScan(const sensor_msgs::msg::LaserScan::SharedPtr scan, pcl::PointCloud<pcl::PointXYZ>::Ptr& outputPC);
    void readPCDFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZ>::Ptr& outputPC);

    void reduceDensityPC(pcl::PointCloud<pcl::PointXYZ>::Ptr& inputPC, pcl::PointCloud<pcl::PointXYZ>::Ptr& outputPC);
    
    void cropFOVPC(pcl::PointCloud<pcl::PointXYZ>::Ptr& inputPC, pcl::PointCloud<pcl::PointXYZ>::Ptr& outputPC);

    void applyClustering(pcl::PointCloud<pcl::PointXYZ>::Ptr& inputPC,
                        std::vector<pcl::PointIndices>& clusterIndices,
                        std::vector<struct ClusterData>& clusters
    );

    void detectObject(std::vector<struct ClusterData>& clusters, pcl::PointXYZ target, int& objectIndex);
    
    void publishObjectCoords();

 private:
    pcl::PCDReader reader_;
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid_;
    pcl::CropBox<pcl::PointXYZ> cropBox_;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclideanClustering_;

    struct VoxelConf voxelConf_;
    struct CropBoxConf cropBoxConf_;
    struct ClusteringConf clusteringConf_;
};

#endif  // CLUSTERING_H_
