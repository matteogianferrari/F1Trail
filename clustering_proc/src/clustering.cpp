#include "clustering.hpp"
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <limits>
#include <array>



ClusteringNode::ClusteringNode() : Node("clustering_node") {
    // Gets all potential parameters
    // Voxel Conf
    this->declare_parameter<std::vector<double>>("voxel", {0.,0.,0.}); // maybe change the default to {0.05, 0.05, 0.f}
	auto VoxelConf = this->get_parameter("voxel").get_value<std::vector<double>>();
    
    // maybe throw exception if the array has not exaclty 3 entries
    if (VoxelConf.size() != 3) {
        throw std::runtime_error("Parameter 'voxel' must contain exactly 3 entries.");
    }

    // Set up VoxelGrid filter parameters
    voxelGrid_.setLeafSize(voxelConf_[0], voxelConf_[1], voxelConf_[2]);



    // CropBox Conf
    this->declare_parameter<std::vector<double>>("crop_box_min", {0.0, 0.0, 0.0, 0.0}); // maybe change the default to {0.f, -3.f, 0.f, 1.f}
    this->declare_parameter<std::vector<double>>("crop_box_max", {0.0, 0.0, 0.0, 0.0}); // maybe change the default to {5.f, 3.f, 0.f, 1.}
    auto minPointParam = this->get_parameter("crop_box_min").get_value<std::vector<double>>();
    auto maxPointParam = this->get_parameter("crop_box_max").get_value<std::vector<double>>();

    // Size check
    if (minPointParam.size() != 4) {
        throw std::runtime_error("Parameter 'crop_box_min' must contain exactly 4 entries.");
    }
    if (maxPointParam.size() != 4) {
        throw std::runtime_error("Parameter 'crop_box_max' must contain exactly 4 entries.");
    }

    // Convert to Eigen::Vector4f (float)
    Eigen::Vector4f minPoint(static_cast<float>(minPointParam[0]), static_cast<float>(minPointParam[1]),
                             static_cast<float>(minPointParam[2]), static_cast<float>(minPointParam[3]));
    Eigen::Vector4f maxPoint(static_cast<float>(maxPointParam[0]), static_cast<float>(maxPointParam[1]),
                             static_cast<float>(maxPointParam[2]), static_cast<float>(maxPointParam[3]));

    // Create CropBoxConf and assign the min and max points
    // Not sure this is necessary
    CropBoxConf cropBoxConf_;
    cropBoxConf_.minPoint = minPoint;
    cropBoxConf_.maxPoint = maxPoint;



    // Clustering Conf
    // Declare clustering configuration parameters
    this->declare_parameter<std::vector<size_t>>("ClusteringConf_size_topic", {0, 0});  // Min and Max cluster sizes
    this->declare_parameter("ClusteringConf_tolerance_topic", 0.0);  // Tolerance value
    // maybe change the default to {10, 300} and 0.35f

    // Retrieve the size parameters (min and max cluster sizes)
    auto ClusteringConf_size_topic = this->get_parameter("ClusteringConf_size_topic").get_value<std::vector<size_t>>();
    if (ClusteringConf_size_topic.size() != 2) {
        throw std::runtime_error("Parameter 'ClusteringConf_size_topic' must contain exactly 2 entries.");
    }

    // Retrieve the tolerance parameter
    double ClusteringConf_tolerance_topic = this->get_parameter("ClusteringConf_tolerance_topic").as_double();

    // Assign values to your ClusteringConf struct
    // Not sure this is necessary
    ClusteringConf clusteringConf_;
    clusteringConf_.minClusterSize = ClusteringConf_size_topic[0];
    clusteringConf_.maxClusterSize = ClusteringConf_size_topic[1];
    clusteringConf_.ecTolerance = ClusteringConf_tolerance_topic;


    // Defines quality of service: all messages that you want to receive must have the same
	rclcpp::QoS custom_qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
	.history(rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST)
	.keep_last(10)
	.reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
	.durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE)
	.avoid_ros_namespace_conventions(false);


    // Create publisher for clustered point cloud output
    pub_clusters_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output_clusters", 10);


    //                                             NEED TO MODIFY LIDAR SCAN LOCATION
    // Create subscription to LaserScan data instead of PointCloud2
    point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "lidar_scan", 10, std::bind(&ClusteringNode::laserScanCallback, this, std::placeholders::_1));

}


// callback
void ClusteringNode::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received a LaserScan message");

    // Convert the LaserScan into a PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);
    readLidarScan(msg, inputCloud);

    // Proceed with voxel filtering, cropping, and clustering (same as before)
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
    reduceDensityPC(inputCloud, filteredCloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr croppedCloud(new pcl::PointCloud<pcl::PointXYZ>);
    cropFOVPC(filteredCloud, croppedCloud);

    std::vector<pcl::PointIndices> clusterIndices;
    std::vector<ClusterData> clusters;
    applyClustering(croppedCloud, clusterIndices, clusters);

    pcl::PointXYZ target;
    target.x = 0.0;
    target.y = 0.0;
    target.z = 0.0;

    int objectIndex;
    detectObject(clusters, target, objectIndex);
    
    RCLCPP_INFO(this->get_logger(), "Detected object index: %d", objectIndex);

    // Publish the clustered point cloud
    sensor_msgs::msg::PointCloud2 outputMsg;
    pcl::toROSMsg(*croppedCloud, outputMsg);
    outputMsg.header.stamp = this->get_clock()->now();
    pub_clusters_->publish(outputMsg);
}


void ClusteringNode::readLidarScan(const sensor_msgs::msg::LaserScan::SharedPtr scan, pcl::PointCloud<pcl::PointXYZ>::Ptr& outputPC) {
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


void ClusteringNode::reduceDensityPC(pcl::PointCloud<pcl::PointXYZ>::Ptr& inputPC, pcl::PointCloud<pcl::PointXYZ>::Ptr& outputPC)
{
    // Sets the input cloud pointer
    voxelGrid_.setInputCloud(inputPC);

    // Sets the leaf size to create the grid -> already made in constructor

    // Apply Voxel filtering
    voxelGrid_.filter(*outputPC);
}


void ClusteringNode::cropFOVPC(pcl::PointCloud<pcl::PointXYZ>::Ptr& inputPC, pcl::PointCloud<pcl::PointXYZ>::Ptr& outputPC)
{
    // Sets the input cloud pointer
    cropBox_.setInputCloud(inputPC);
    
    // Sets the minimum and maximum point of the crop box
    cropBox_.setMin(cropBoxConf_.minPoint);
    cropBox_.setMax(cropBoxConf_.maxPoint);

    // Apply crop box filtering
    cropBox_.filter(*outputPC);
}


void ClusteringNode::applyClustering(pcl::PointCloud<pcl::PointXYZ>::Ptr& inputPC,
                                      std::vector<pcl::PointIndices>& clusterIndices,
                                      std::vector<ClusterData>& clusters)
{
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


void ClusteringNode::detectObject(std::vector<ClusterData>& clusters, pcl::PointXYZ target, int& objectIndex)
{
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



// Clustering::Clustering(struct VoxelConf vConf, struct CropBoxConf cbConf, struct ClusteringConf cConf):
//     voxelConf_ {vConf}, cropBoxConf_ {cbConf}, clusteringConf_ {cConf} {

// }


// Clustering::~Clustering() {

// }


// void Clustering::readPCDFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZ>::Ptr& outputPC) {
//     pcl::PointCloud<pcl::PointXY>::Ptr inputPC {new pcl::PointCloud<pcl::PointXY>};

//     // Reads the 2D point cloud from the PCD file
//     reader_.read(fileName, *inputPC);

//     // Reserves space for the 3D point cloud
//     outputPC->points.resize(inputPC->points.size());

//     // Converts each XY point to XYZ point
//     for (size_t i = 0; i < inputPC->points.size(); ++i) {
//         pcl::PointXYZ point;

//         // The Lidar returns a 2D point, thus it's converted to 3D by adding 0 to the Z-axis
//         point.x = inputPC->points[i].x;
//         point.y = inputPC->points[i].y;
//         point.z = 0.f;

//         // Adds the point to the output point cloud
//         outputPC->points[i] = point;
//     }

//     // Fills additional point cloud information (not needed for the project)
//     outputPC->width = inputPC->width;           // Unit measure: [Point]
//     outputPC->height = inputPC->height;         // Unit measure: [Point]
//     outputPC->is_dense = inputPC->is_dense;     // No points are invalid
// }


// void Clustering::reduceDensityPC(pcl::PointCloud<pcl::PointXYZ>::Ptr& inputPC, pcl::PointCloud<pcl::PointXYZ>::Ptr& outputPC) {
//     // Sets the input cloud pointer
//     voxelGrid_.setInputCloud(inputPC);

//     // Sets the leaf size to create the grid
//     voxelGrid_.setLeafSize(voxelConf_.lx, voxelConf_.ly, voxelConf_.lz);

//     // Apply Voxel filtering
//     voxelGrid_.filter(*outputPC);
// }


// void Clustering::cropFOVPC(pcl::PointCloud<pcl::PointXYZ>::Ptr& inputPC, pcl::PointCloud<pcl::PointXYZ>::Ptr& outputPC) {
//     // Sets the input cloud pointer
//     cropBox_.setInputCloud(inputPC);
    
//     // Sets the minimum and maximum point of the crop box
//     cropBox_.setMin(cropBoxConf_.minPoint);
//     cropBox_.setMax(cropBoxConf_.maxPoint);

//     // Apply crop box filtering
//     cropBox_.filter(*outputPC);
// }


// void Clustering::applyClustering(pcl::PointCloud<pcl::PointXYZ>::Ptr& inputPC,
//                                 std::vector<pcl::PointIndices>& clusterIndices,
//                                 std::vector<struct ClusterData>& clusters) {
//     // K-D Tree object creation used for neighbors search
//     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree {new pcl::search::KdTree<pcl::PointXYZ>};

//     // Sets the input cloud pointer
//     tree->setInputCloud(inputPC);

//     // Sets the spatial cluster tolerance
//     euclideanClustering_.setClusterTolerance(clusteringConf_.ecTolerance);

//     // Sets the minimum and maximum number of points for a cluster
//     euclideanClustering_.setMinClusterSize(clusteringConf_.minClusterSize); 
//     euclideanClustering_.setMaxClusterSize(clusteringConf_.maxClusterSize);   

//     // Sets the search method
//     euclideanClustering_.setSearchMethod(tree);

//     // Sets the input cloud pointer
//     euclideanClustering_.setInputCloud(inputPC);

//     // Performs Euclidean clustering
//     euclideanClustering_.extract(clusterIndices);

//     // Clears the vector of point clouds (in case it already contains data)
//     clusters.clear();

//     // For each cluster, extracts the points and creates a new point cloud
//     for (const auto& currIndices : clusterIndices) {
//         // Creates a new point cloud for the cluster
//         pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);

//         // Adds the points corresponding to the current cluster to the new point cloud
//         for (const auto& i : currIndices.indices) {
//             cluster->points.push_back(inputPC->points[i]);
//         }

//         cluster->width = cluster->points.size();
//         cluster->height = 1; 
//         cluster->is_dense = true;

//         // Gets the minimum point and the maximum point from the cluster
//         pcl::PointXYZ minPt, maxPt;
//         pcl::getMinMax3D(*cluster, minPt, maxPt);

//         // Box creation for rendering the bounding box
//         BoundingBox box {minPt, maxPt};

//         // Computes the current cluster centroid
//         Eigen::Vector4f centroid;
//         pcl::compute3DCentroid(*cluster, centroid);

//         struct ClusterData currCluster {cluster, box, centroid};
//         clusters.push_back(currCluster);
//     }
// }


// void Clustering::detectObject(std::vector<struct ClusterData>& clusters, pcl::PointXYZ target, int& objectIndex) {
//     int closestIndex = -1;
//     float minDistance = std::numeric_limits<float>::max();

//     for (int i = 0; i < clusters.size(); ++i) {
//         float distance = std::sqrt(std::pow(target.x - clusters[i].centroid[0], 2) +
//                      std::pow(target.y - clusters[i].centroid[1], 2) +
//                      std::pow(target.z - clusters[i].centroid[2], 2));

//         if (distance < minDistance) {
//             minDistance = distance;
//             closestIndex = i;
//         }
//     }

//     objectIndex = closestIndex;
// }


int main(int argc, char ** argv) {
	rclcpp::init(argc, argv);

	auto node = std::make_shared<ClusteringNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	
	return 0;
}