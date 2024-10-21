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
    

    // Subscribes to target topic
    subTargetLoc_.subscribe(this, target_location_topic, custom_qos_profile.get_rmw_qos_profile());

    // Subscribes to the clustering topic
    subClusterCentroids_.subscribe(this, cluster_centroids_topic, custom_qos_profile.get_rmw_qos_profile());

    // create publisher for target location point
    pub_ = this->create_publisher<geometry_msgs::msg::Point>("/tracked_obj_loc", 10);
}


void TrackerNode::target_callback(const geometry_msgs::msg::Point::SharedPtr stereo_msg) {
    RCLCPP_INFO(this->get_logger(), "Received target location from aruco_loc node.");

    currAlpha_ = pointToEigen(stereo_msg);

    if () {
        return;
    }

    trackerCore();
}


void TrackerNode::centroids_callback(const geometry_msgs::msg::PoseArray::SharedPtr cluster_msg) {
    RCLCPP_INFO(this->get_logger(), "Received clusters centroids from clustering node.");

    centroids_ = poseArrayToEigen(cluster_msg);

    if () {
        return;
    }

    trackerCore();
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

    auto node = std::make_shared<Tracker>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}