#include "clustering.hpp"
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <limits>
#include <string>



ClusteringNode::ClusteringNode() : Node("clustering_node") {
    // parameter for lidar scan topic
    this->declare_parameter<std::string>("scan_topic", "/scan");
    // Gets all potential parameters
    // Voxel Conf
    // Maybe change the default to {0.05, 0.05, 0.f}
    this->declare_parameter<std::vector<double>>("voxel", {0.05, 0.05, 0.0});
	voxelConf_ = this->get_parameter("voxel").get_value<std::vector<double>>();
    
    // Throw exception if the array has not exaclty 3 entries
    if (voxelConf_.size() != 3) {
        throw std::runtime_error("Parameter 'voxel' must contain exactly 3 entries.");
    }


    // CropBox Conf
    this->declare_parameter<std::vector<double>>("crop_box_min", {0., -3., 0., 1.});
    this->declare_parameter<std::vector<double>>("crop_box_max", {5., 3., 0., 1.});
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
    // Set cropbox parameters
    cb_minPoint_ = minPoint;
    cb_maxPoint_ = maxPoint;


    // Clustering Conf
    // Declare clustering configuration parameters
    this->declare_parameter<int64_t>("max_cluster_size", std::numeric_limits<size_t>::max());
    this->declare_parameter<int64_t>("min_cluster_size", 0);
    this->declare_parameter<double>("ec_tollerance", 0.35);

    // retrieve bound values for cluster size
    minClusterSize_ = static_cast<size_t>(this->get_parameter("min_cluster_size").get_value<int64_t>());
    maxClusterSize_ = static_cast<size_t>(this->get_parameter("max_cluster_size").get_value<int64_t>());
    if (maxClusterSize_ <= minClusterSize_) {
        throw std::runtime_error("Parameter 'max_cluster_size' must be greater than 'min_cluster_size'.");
    }
    
    // Retrieve the tolerance parameter
    ecTolerance_ = this->get_parameter("ec_tollerance").as_double();

    // Defines quality of service: all messages that you want to receive must have the same
	rclcpp::QoS custom_qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
	.history(rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST)
	.keep_last(10)
	.reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
	.durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE)
	.avoid_ros_namespace_conventions(false);

    // Create publisher for a vector of centroids
    centroid_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/cluster_centroids", custom_qos_profile);

    // Create subscription to LaserScan data
    auto scan_topic = this->get_parameter("scan_topic").get_value<std::string>();
    lidar_scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic, custom_qos_profile, std::bind(&ClusteringNode::laserScanCallback, this, std::placeholders::_1));
}


void ClusteringNode::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Received a LaserScan message");

    // Convert the LaserScan into a PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);
    readLidarScan(msg, inputCloud);

    // Proceed with voxel filtering, cropping, and clustering (same as before)
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
    reduceDensityPC(inputCloud, filteredCloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr croppedCloud(new pcl::PointCloud<pcl::PointXYZ>);
    cropFOVPC(filteredCloud, croppedCloud);

    // Get clusters centroids
    auto cluster_centroids = applyClustering(croppedCloud);

    // Publish vector of centroids
    // Set timestamp and set frame ID (should match your TF frame)
    geometry_msgs::msg::PoseArray centroidArrayMsg;
    centroidArrayMsg.header.stamp = this->now();
    centroidArrayMsg.header.frame_id = "lidar_link";

    for (const auto& centroid : cluster_centroids) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = centroid[0];
        pose.position.y = centroid[1];
        pose.position.z = centroid[2];

        // Set a neutral quaternion for orientation (you can ignore this later)
        // No rotation (neutral quaternion)
        pose.orientation.w = 1.0;

        centroidArrayMsg.poses.push_back(pose);
    }

    // Publish the centroid array
    centroid_publisher_->publish(centroidArrayMsg);
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
    outputPC->width = outputPC->points.size();
    outputPC->height = 1;
    outputPC->is_dense = true;
}


void ClusteringNode::reduceDensityPC(pcl::PointCloud<pcl::PointXYZ>::Ptr& inputPC, pcl::PointCloud<pcl::PointXYZ>::Ptr& outputPC)
{
    // Sets the input cloud pointer
    voxelGrid_.setInputCloud(inputPC);
    // Set up VoxelGrid filter parameters
    voxelGrid_.setLeafSize(voxelConf_[0], voxelConf_[1], voxelConf_[2]);
    // Apply Voxel filtering
    voxelGrid_.filter(*outputPC);
}


void ClusteringNode::cropFOVPC(pcl::PointCloud<pcl::PointXYZ>::Ptr& inputPC, pcl::PointCloud<pcl::PointXYZ>::Ptr& outputPC)
{
    // Sets the input cloud pointer
    cropBox_.setInputCloud(inputPC);
    
    // Sets the minimum and maximum point of the crop box
    cropBox_.setMin(cb_minPoint_);
    cropBox_.setMax(cb_maxPoint_);

    // Apply crop box filtering
    cropBox_.filter(*outputPC);
}


std::vector<Eigen::Vector4f> ClusteringNode::applyClustering(pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputPC) {
    // K-D Tree object creation used for neighbors search
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree {new pcl::search::KdTree<pcl::PointXYZ>};

    // Sets the input cloud pointer
    tree->setInputCloud(inputPC);

    // Sets the spatial cluster tolerance
    euclideanClustering_.setClusterTolerance(ecTolerance_);

    // Sets the minimum and maximum number of points for a cluster
    euclideanClustering_.setMinClusterSize(minClusterSize_); 
    euclideanClustering_.setMaxClusterSize(maxClusterSize_);   

    // Sets the search method
    euclideanClustering_.setSearchMethod(tree);

    // Sets the input cloud pointer
    euclideanClustering_.setInputCloud(inputPC);

    // Performs Euclidean clustering
    std::vector<pcl::PointIndices> clusterIndices;
    euclideanClustering_.extract(clusterIndices);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Found %d clusters.", clusterIndices.size());

    // Vector to store centroids
    std::vector<Eigen::Vector4f> centroids;

    // For each cluster, extracts the points and creates a new point cloud
    for (const auto& currIndices : clusterIndices) {
        Eigen::Vector4f centroid;
        // We need to obtain an iterator on the points of a specific cluster
        // There is a handy class: pcl::CloudIterator
        auto pcConstIter = pcl::ConstCloudIterator<pcl::PointXYZ>(*inputPC, currIndices);
        
        // Computes the current cluster centroid
        pcl::compute3DCentroid(pcConstIter, centroid);
        centroids.push_back(centroid);
    }
    return centroids;
}

int main(int argc, char ** argv) {
	rclcpp::init(argc, argv);

	auto node = std::make_shared<ClusteringNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	
	return 0;
}