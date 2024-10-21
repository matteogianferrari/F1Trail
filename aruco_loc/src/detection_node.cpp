#include <string>
#include "detection_node.hpp"


// Used for parameter binding of callbacks (see registration of callbacks in cosntructor)
using std::placeholders::_1;
using std::placeholders::_2;


ImageDetectionNode::ImageDetectionNode() : Node("detection_node") {
	// Gets all potential parameters
	this->declare_parameter("left_img_topic", "/left_image");
	std::string left_img_topic = this->get_parameter("left_img_topic").as_string();

	this->declare_parameter("right_img_topic", "/right_image");
	std::string right_img_topic = this->get_parameter("right_img_topic").as_string();
	
	// Supposing horizontal displacement only between stereo images (value in cm)
	this->declare_parameter("horizontal_d", 119.994);
	B = this->get_parameter("horizontal_d").as_double();

	// Focal length (value in pixels)
	this->declare_parameter("focal_length", 699.74);
	f = this->get_parameter("focal_length").as_double();

	// Parameters for aruco detector
	this->declare_parameter("marker_idx", 16);
	marker_idx_ = this->get_parameter("marker_idx").as_int();

	this->declare_parameter("aruco_dict", static_cast<int>(cv::aruco::DICT_6X6_250));
	aruco_dict_ = static_cast<cv::aruco::PredefinedDictionaryType>(this->get_parameter("aruco_dict").as_int());

	// Defines quality of service: all messages that you want to receive must have the same
	rclcpp::QoS custom_qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
	.history(rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST)
	.keep_last(10)
	.reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
	.durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE)
	.avoid_ros_namespace_conventions(false);

	// Subscribes to image topics
	sub_right_img_.subscribe(this, right_img_topic, custom_qos_profile.get_rmw_qos_profile());
	sub_left_img_.subscribe(this, left_img_topic, custom_qos_profile.get_rmw_qos_profile());

	// Creates exact synchronization based on timestamp data in message headers
	sync_exact_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image,sensor_msgs::msg::Image>>(sub_left_img_, sub_right_img_, 10);
	auto Func = std::bind(&ImageDetectionNode::exact_sync_callback, this, _1, _2);
	sync_exact_->registerCallback(Func);

	// create publisher for target location point
	pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/target_loc", 10);
	
	// Selects dict of markers to use
	cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(aruco_dict_);

	// Leaving detection parameters as defaults
	detector_ = cv::aruco::ArucoDetector(dictionary);
}


void ImageDetectionNode::exact_sync_callback(const sensor_msgs::msg::Image::ConstSharedPtr left_img, const sensor_msgs::msg::Image::ConstSharedPtr right_img) {
	RCLCPP_INFO(this->get_logger(), "Received synchronized stereo images.");
	stereo_target_detect(cv_bridge::toCvShare(left_img), cv_bridge::toCvShare(right_img));
}


void ImageDetectionNode::stereo_target_detect(cv_bridge::CvImageConstPtr left_img, cv_bridge::CvImageConstPtr right_img) {
	// Contains the ids of markers found each image
	std::vector<int> left_markerIds;
	std::vector<int> right_markerIds;
	// Corner location of found markers. order is based on list of ids
	std::vector<std::vector<cv::Point2f>> left_markerCorners;
	std::vector<std::vector<cv::Point2f>> right_markerCorners;
	cv::Mat left, right;
	// We accept the standard color format from stereolabs cameras. detector accepts color images without alpha component.
	cv::cvtColor(left_img->image, left, cv::COLOR_RGBA2RGB);
	cv::cvtColor(right_img->image, right, cv::COLOR_BGRA2BGR);

	detector_.detectMarkers(left, left_markerCorners, left_markerIds);
	detector_.detectMarkers(right, right_markerCorners, right_markerIds);
	// Look for our marker. if not found, do not proceed further!
	auto left_mark_it = std::find(left_markerIds.begin(), left_markerIds.end(), marker_idx_);
	auto right_mark_it = std::find(right_markerIds.begin(), right_markerIds.end(), marker_idx_);
	if (left_mark_it == left_markerIds.end()) {
		RCLCPP_WARN(this->get_logger(), "No marker found in left image!");
	}
	else if (right_mark_it == right_markerIds.end()) {
		RCLCPP_WARN(this->get_logger(), "No marker found in right image!");
	}
	else {
		cv::Point2f left_center = target_center(left_markerCorners[std::distance(left_markerIds.begin(), left_mark_it)]);
		cv::Point2f right_center = target_center(right_markerCorners[std::distance(right_markerIds.begin(), right_mark_it)]);
		
		geometry_msgs::msg::PointStamped pt_msg;
		pt_msg.point.x = left_center.x;
		pt_msg.point.y = left_center.y;
		
		// Depth computation assumes no rotation (otherwise epipolar geometry is needed)
		pt_msg.point.z = B*f / (std::abs(left_center.x - right_center.x));

		// Set point header
		pt_msg.header.stamp = this->now();
		pt_msg.header.frame_id = "camera_link";
		
		RCLCPP_INFO(this->get_logger(), "Marker location found!");
		pub_->publish(pt_msg);
	}

}


cv::Point2f ImageDetectionNode::target_center(std::vector<cv::Point2f> const &corners) const {
	cv::Point2f center;

	for (auto const &corner : corners) {
		center.x += corner.x / 4;
		center.y += corner.y / 4;
	}

	return center;
}


int main(int argc, char ** argv) {
	rclcpp::init(argc, argv);

	auto node = std::make_shared<ImageDetectionNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	
	return 0;
}
