#include "split_stereo.hpp"


// used for parameter binding in callbacks to be registered with subscribers.
using std::placeholders::_1;


SLStereoSplit::SLStereoSplit(): Node("split_stereo") {
	// Explicitly create a custom QoS profile to allow synchronization on published image topics by subscribers
	// Subscribers must refer to these options when synchronizing topics
	rclcpp::QoS custom_qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
	.history(rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST)
	.keep_last(10)
	.reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
	.durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE)
	.avoid_ros_namespace_conventions(false);

	// Topic where we want to receive the stereo image
	this->declare_parameter("stereo_img_topic", "/topic");
	std::string stereo_img_topic = this->get_parameter("stereo_img_topic").as_string();
	
	// Subscribes to the stereo image topic
	RCLCPP_INFO(this->get_logger(), "Subscribing to image topic: %s", stereo_img_topic.c_str());
	sub_img_ = this->create_subscription<sensor_msgs::msg::Image>(stereo_img_topic, 
	10, std::bind(&SLStereoSplit::img_callback, this, _1));

	// Creates the publisher for the left and right image separately
	this->declare_parameter("right_output_topic", "/right_image");
	this->declare_parameter("left_output_topic", "/left_image");
	std::string right_output = this->get_parameter("right_output_topic").as_string();
	std::string left_output = this->get_parameter("left_output_topic").as_string();
	pub_right_ = this->create_publisher<sensor_msgs::msg::Image>(right_output, custom_qos_profile);
	pub_left_ = this->create_publisher<sensor_msgs::msg::Image>(left_output, custom_qos_profile);
}


void SLStereoSplit::img_callback(const sensor_msgs::msg::Image::SharedPtr img) {
	// Data structures to hold images
	cv::Mat left, right;

	RCLCPP_INFO(this->get_logger(), "Received stereo image.");
	auto cv_img = cv_bridge::toCvShare(img);

	// Headers are created starting from the stereo image header
	auto cv_right = cv_bridge::CvImage(img->header, img->encoding);
	auto cv_left = cv_bridge::CvImage(img->header, img->encoding);
	split_stereo(cv_img, cv_left.image, cv_right.image);

	// Publishes resulting images
	pub_left_->publish(*cv_left.toImageMsg());
	pub_right_->publish(*cv_right.toImageMsg());
}


void SLStereoSplit::split_stereo(cv_bridge::CvImageConstPtr img, cv::Mat &left, cv::Mat &right) {
	// Image will always need to have an even number of columns
	rcpputils::require_true((img->image.cols % 2) == 0, "Stereo image has an odd number of columns!");
	
	auto half = img->image.cols / 2;
	RCLCPP_DEBUG(this->get_logger(), "Splitting stereo image at column: %d", half);

	// This operation does not copy assign. only copies a modified header, the underlying data is that of the original image
	left = img->image(cv::Range::all(), cv::Range(0, half));
	right = img->image(cv::Range::all(), cv::Range(half, 2*half));
	RCLCPP_INFO(this->get_logger(), "Stereo image has been split. Dimension (h,w): %d,%d", img->image.rows, half);
}


int main(int argc, char ** argv) {
	rclcpp::init(argc, argv);

	auto node = std::make_shared<SLStereoSplit>();
	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}
