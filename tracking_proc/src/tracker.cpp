#include "tracker.hpp"


// Used for parameter binding in callbacks to be registered with subscribers.
using std::placeholders::_1;


Tracker::Tracker(): Node("tracking_module") {
	isInitialized_ = false;

	// Gets all potential parameters
	this->declare_parameter("target_location_topic", "/target_loc");

	// Defines quality of service: all messages that you want to receive must have the same
	rclcpp::QoS custom_qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
	.history(rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST)
	.keep_last(10)
	.reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
	.durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE)
	.avoid_ros_namespace_conventions(false);

	// Subscribes to target topic
	subTargetLoc_.subscribe(this, target_location_topic, custom_qos_profile.get_rmw_qos_profile());

}


void Tracker::target_callback(const ) {
	currAlpha_ = stereo_msg;

	if () {
		return;
	}

	trackerCore();
}


void Tracker::centroids_callback(const) {
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


int main(int argc, char ** argv) {
	rclcpp::init(argc, argv);

	auto node = std::make_shared<Tracker>();
	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}