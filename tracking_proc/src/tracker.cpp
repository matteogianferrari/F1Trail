#include "tracker.hpp"


// Used for parameter binding in callbacks to be registered with subscribers.
using std::placeholders::_1;


Tracker::Tracker(): Node("tracking_module") {
	isInitialized_ = false;

	// Gets all potential parameters
	this->declare_parameter("target_location_topic", "/target_loc");
	
	this->declare_parameter("cluster_centroids_topic", "/cluster_centroids");


	// Subscribes to target topic
	subTargetLoc_.subscribe(this, target_location_topic, custom_qos_profile.get_rmw_qos_profile());

	// Subscribes to the clustering topic
	subClusterCentroids_.subscribe(this, cluster_centroids_topic, custom_qos_profile.get_rmw_qos_profile());

	// create publisher for target location point
	pub_ = this->create_publisher<geometry_msgs::msg::Point>("/tracked_obj_loc", 10);
}


void Tracker::target_callback(const ) {
	RCLCPP_INFO(this->get_logger(), "Received target location from aruco_loc node.");

	currAlpha_ = pointToEigen(stereo_msg);

	if () {
		return;
	}

	trackerCore();
}


void Tracker::centroids_callback(const) {
	RCLCPP_INFO(this->get_logger(), "Received clusters centroids from clustering node.");

	centroids_ = cluster_msg;

	if () {
		return;
	}

	trackerCore();
}


void Tracker::trackerCore() {
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


Eigen::Vector3d Tracker::selectCentroid() {

}



Eigen::VectorXd pointToEigen(const geometry_msgs::msg::Point& point) {
    // Creates an Eigen::VectorXd of size 3
    Eigen::VectorXd vec(3);
	vec << point.x, point.y, point.z;

    return vec;
}


int main(int argc, char ** argv) {
	rclcpp::init(argc, argv);

	auto node = std::make_shared<Tracker>();
	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}