#include "clustering.hpp"
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <limits>


Clustering::Clustering(struct VoxelConf vConf, struct CropBoxConf cbConf, struct ClusteringConf cConf):
    voxelConf_ {vConf}, cropBoxConf_ {cbConf}, clusteringConf_ {cConf} {

}


Clustering::~Clustering() {

}


void Clustering::readLidarScan(const sensor_msgs::msg::LaserScan::SharedPtr scan, pcl::PointCloud<pcl::PointXYZ>::Ptr& outputPC) {
    // Point's angle used to compute (x; y) coordinates
    double currentAngle = scan->angle_min;
    
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        float range = scan->ranges[i];

        // Checks for range constraints
        if (range >= scan->range_min && range <= scan->range_max) {
            pcl::PointXYZ point;

            // The Lidar returns a 2D point, thus it's converted to 3D by adding 0 to the Z-axis
            point.x = range * cos(currentAngle);
            point.y = range * sin(currentAngle);
            point.z = 0.f;

            // Adds the point to the output point cloud
            outputPC->points.push_back(point);
        }

        // Increments the angle 
        currentAngle += scan->angle_increment;
    }

    // Fills additional point cloud information (not needed for the project)
    outputPC->width = outputPC->points.size();    // Unit measure: [Point]
    outputPC->height = 1;                         // Unit measure: [Point]
    outputPC->is_dense = true;                    // No points are invalid
}


void Clustering::readPCDFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZ>::Ptr& outputPC) {
    pcl::PointCloud<pcl::PointXY>::Ptr inputPC {new pcl::PointCloud<pcl::PointXY>};

    // Reads the 2D point cloud from the PCD file
    reader_.read(fileName, *inputPC);

    // Reserves space for the 3D point cloud
    outputPC->points.resize(inputPC->points.size());

    // Converts each XY point to XYZ point
    for (size_t i = 0; i < inputPC->points.size(); ++i) {
        pcl::PointXYZ point;

        // The Lidar returns a 2D point, thus it's converted to 3D by adding 0 to the Z-axis
        point.x = inputPC->points[i].x;
        point.y = inputPC->points[i].y;
        point.z = 0.f;

        // Adds the point to the output point cloud
        outputPC->points[i] = point;
    }

    // Fills additional point cloud information (not needed for the project)
    outputPC->width = inputPC->width;           // Unit measure: [Point]
    outputPC->height = inputPC->height;         // Unit measure: [Point]
    outputPC->is_dense = inputPC->is_dense;     // No points are invalid
}


void Clustering::reduceDensityPC(pcl::PointCloud<pcl::PointXYZ>::Ptr& inputPC, pcl::PointCloud<pcl::PointXYZ>::Ptr& outputPC) {
    // Sets the input cloud pointer
    voxelGrid_.setInputCloud(inputPC);

    // Sets the leaf size to create the grid
    voxelGrid_.setLeafSize(voxelConf_.lx, voxelConf_.ly, voxelConf_.lz);

    // Apply Voxel filtering
    voxelGrid_.filter(*outputPC);
}


void Clustering::cropFOVPC(pcl::PointCloud<pcl::PointXYZ>::Ptr& inputPC, pcl::PointCloud<pcl::PointXYZ>::Ptr& outputPC) {
    // Sets the input cloud pointer
    cropBox_.setInputCloud(inputPC);

    // Sets the minimum and maximum point of the crop box
    cropBox_.setMin(cropBoxConf_.minPoint);
    cropBox_.setMax(cropBoxConf_.maxPoint);

    // Apply crop box filtering
    cropBox_.filter(*outputPC);
}


void Clustering::applyClustering(pcl::PointCloud<pcl::PointXYZ>::Ptr& inputPC,
                                std::vector<pcl::PointIndices>& clusterIndices,
                                std::vector<struct ClusterData>& clusters) {
    // K-D Tree object creation used for neighbors search
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree {new pcl::search::KdTree<pcl::PointXYZ>};

    // Sets the input cloud pointer
    tree->setInputCloud(inputPC);

    // Sets the spatial cluster tolerance
    euclideanClustering_.setClusterTolerance(clusteringConf_.ecTolerance);

    // Sets the minimum and maximum number of points for a cluster
    euclideanClustering_.setMinClusterSize(clusteringConf_.minClusterSize); 
    euclideanClustering_.setMaxClusterSize(clusteringConf_.maxClusterSize);   

    // Sets the search method
    euclideanClustering_.setSearchMethod(tree);

    // Sets the input cloud pointer
    euclideanClustering_.setInputCloud(inputPC);

    // Performs Euclidean clustering
    euclideanClustering_.extract(clusterIndices);

    // Clears the vector of point clouds (in case it already contains data)
    clusters.clear();

    // For each cluster, extracts the points and creates a new point cloud
    for (const auto& currIndices : clusterIndices) {
        // Creates a new point cloud for the cluster
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);

        // Adds the points corresponding to the current cluster to the new point cloud
        for (const auto& i : currIndices.indices) {
            cluster->points.push_back(inputPC->points[i]);
        }

        cluster->width = cluster->points.size();
        cluster->height = 1; 
        cluster->is_dense = true;

        // Gets the minimum point and the maximum point from the cluster
        pcl::PointXYZ minPt, maxPt;
        pcl::getMinMax3D(*cluster, minPt, maxPt);

        // Box creation for rendering the bounding box
        BoundingBox box {minPt, maxPt};

        // Computes the current cluster centroid
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cluster, centroid);

        struct ClusterData currCluster {cluster, box, centroid};
        clusters.push_back(currCluster);
    }
}


void Clustering::detectObject(std::vector<struct ClusterData>& clusters, pcl::PointXYZ target, int& objectIndex) {
    int closestIndex = -1;
    float minDistance = std::numeric_limits<float>::max();

    for (int i = 0; i < clusters.size(); ++i) {
        float distance = std::sqrt(std::pow(target.x - clusters[i].centroid[0], 2) +
                     std::pow(target.y - clusters[i].centroid[1], 2) +
                     std::pow(target.z - clusters[i].centroid[2], 2));

        if (distance < minDistance) {
            minDistance = distance;
            closestIndex = i;
        }
    }

    objectIndex = closestIndex;
}
