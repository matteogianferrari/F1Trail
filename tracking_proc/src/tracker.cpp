#include "tracker.hpp"
#include <string>


// Used for parameter binding in callbacks to be registered with subscribers.
using std::placeholders::_1;


TrackerNode::TrackerNode(): Node("tracking_module") {
    isInitialized_ = false;

    // Gets all potential parameters
    this->declare_parameter("target_location_topic", "/target_loc");
    std::string target_location_topic = this->get_parameter("target_location_topic").as_string();
    
    this->declare_parameter("cluster_centroids_topic", "/cluster_centroids");
    std::string cluster_centroids_topic = this->get_parameter("cluster_centroids_topic").as_string();
    
    // Defines quality of service: all messages that you want to receive must have the same
    rclcpp::QoS custom_qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
    .history(rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST)
    .keep_last(10)
    .reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
    .durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE)
    .avoid_ros_namespace_conventions(false);
    
    // Subscribes to target topic
    subTargetLoc_ = this->create_subscription<geometry_msgs::msg::Point>(
        target_location_topic, custom_qos_profile, std::bind(&TrackerNode::target_callback, this, std::placeholders::_1));

    // Subscribes to the clustering topic
    subClusterCentroids_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        cluster_centroids_topic, custom_qos_profile, std::bind(&TrackerNode::centroids_callback, this, std::placeholders::_1));

    // create publisher for target location point
    pub_ = this->create_publisher<geometry_msgs::msg::Point>("/tracked_obj_loc", custom_qos_profile);
}


void TrackerNode::target_callback(const geometry_msgs::msg::Point::SharedPtr stereo_msg) {
    RCLCPP_INFO(this->get_logger(), "Received target location from aruco_loc node.");

    // Stores the target location received from the Aruco node
    currAlpha_ = pointToEigen(stereo_msg);

    // Print the target location (assuming currAlpha_ is an Eigen vector)
    RCLCPP_INFO(this->get_logger(), "Target Location: [x: %f, y: %f, z: %f]", 
                currAlpha_[0], currAlpha_[1], currAlpha_[2]);

    // Edge case: No centroids have been received yet
    // if (centroids_.empty()) {
    //     RCLCPP_WARN(this->get_logger(), "No centroids available yet.");
    //     return;
    // }

    // Tracks the object
    // trackerCore();
}


void TrackerNode::centroids_callback(const geometry_msgs::msg::PoseArray::SharedPtr cluster_msg) {
    RCLCPP_INFO(this->get_logger(), "Received clusters centroids from clustering node.");

    // Stores the centroids received from the Clustering node
    centroids_ = poseArrayToEigen(cluster_msg);

    // Print the centroids (assuming centroids_ is a vector of Eigen vectors)
    RCLCPP_INFO(this->get_logger(), "Centroids:");
    for (size_t i = 0; i < centroids_.size(); ++i) {
        RCLCPP_INFO(this->get_logger(), "Centroid %zu: [x: %f, y: %f, z: %f]", 
                    i, centroids_[i][0], centroids_[i][1], centroids_[i][2]);
    }

    // Edge case: No target location has been received yet
    // if (currAlpha_.size() == 0) {
    //     RCLCPP_WARN(this->get_logger(), "No target location received yet.");
    //     return;
    // }

    // Tracks the object
    // trackerCore();
}


void TrackerNode::trackerCore() {
    // Applies sensor fusion between lidar and stereo camera
    // Selects the centroid that minimizes the distance w.r.t. the target location currAlpha
    beta_ = selectCentroid();

    if (!isInitialized_) {
        isInitialized_ = true;
        kalman_.setState(beta_);
    }

    dt = ;
    kalman_.updateMatrices(dt);

    // Estimates the future state of the tracked object
    kalman_.predict();

    // Updates the estimation with the measurement
    kalman_.update(beta_);

    //
    betaPred_ = kalman_.getState();
    currAlpha_ = betaPred_;

    // Publish the position of the tracked object
}


Eigen::Vector3d TrackerNode::selectCentroid() {
    // Initialize variables to keep track of the closest centroid and minimum distance
    double minDistance = std::numeric_limits<double>::max(); 
    Eigen::Vector3d closestCentroid = Eigen::Vector3d::Zero();

    // Iterate over all centroids and calculate the distance to the target location
    for (const auto& centroid : centroids_) {
        // Calculate Euclidean distance between the current centroid and the target location
        double distance = (centroid - currAlpha_).norm();

        // If the distance is smaller than the current minimum, update the closest centroid
        if (distance < minDistance) {
            minDistance = distance;
            closestCentroid = centroid;
        }
    }

    // Log the closest centroid and the minimum distance for debugging purposes
    RCLCPP_INFO(this->get_logger(), "Closest centroid found with distance: %f", minDistance);
    RCLCPP_INFO(this->get_logger(), "Closest Centroid: [x: %f, y: %f, z: %f]", 
                closestCentroid[0], closestCentroid[1], closestCentroid[2]);

    return closestCentroid;
}


Eigen::VectorXd TrackerNode::pointToEigen(const geometry_msgs::msg::Point& point) {
    // Creates an Eigen::VectorXd of size 3
    Eigen::VectorXd vec(3);
    vec << point.x, point.y, point.z;

    return vec;
}


std::vector<Eigen::VectorXd> TrackerNode::poseArrayToEigen(const geometry_msgs::msg::PoseArray& centroids) {
    std::vector<Eigen::VectorXd> eigenCentroids;

    for (const auto& point : centroids.poses) {
        Eigen::VectorXd centroid(3);
        
        // Assign the position (x, y, z)
        centroid << point.position.x, point.position.y, point.position.z;

        eigenCentroids.push_back(centroid);
    }

    return eigenCentroids;
}


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TrackerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}