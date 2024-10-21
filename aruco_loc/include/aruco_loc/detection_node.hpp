/**
 * @file    detection_node.hpp
 * 
 * @author  Pietro Moriello
 *
 * @date    2024-10-08
 * 
 * @brief   Header file for the ROS Image Detection node.
 */
#ifndef DETECTION_NODE_HPP_
#define DETECTION_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <memory>
#include <iostream>
#include <functional>
#include <cmath>

/**
 * @class   ImageDetectionNode 
 * 
 * @brief   Node for marker localization with stereo video frames.
 * 
 * @details This node estimate the position (3D spatial point) of an ArUco
 *			marker relative to the camera reference frame.
 */
class ImageDetectionNode : public rclcpp::Node
{
public:
	/**
	* @fn     ImageDetectionNode
	* 
	* @brief  Constructs a new ImageDetectionNode object.
	*/
    ImageDetectionNode();

private:
	/**
	 * @fn 			exact_sync_callback
	 * 
	 * @brief		Callback function for synchronizing stereo images.
	 *
	 * @details 	This callback is called when two images with the same timestamp
	 *				are published on their respective topics.
	 * 
	 * @param[in]	left_img Image message for left stereo component.
	 * @param[in] 	right_img Image message for right stereo component.
	 */
	void exact_sync_callback(const sensor_msgs::msg::Image::ConstSharedPtr left_img, const sensor_msgs::msg::Image::ConstSharedPtr right_img);

	/**
	 * @fn			stereo_target_detect
	 *
	 * @brief 		Target detection.
	 * 
	 * @details		Detection is performed with opencv's ArUco marker library.
	 *				The marker needs to be detected in both images.
	 *				The projected center of the marker is found for each image
	 *				and it is used to compute the disparity.
	 *
	 * @param[in]	left_img cv_bridge wrap of the received left image message.
	 * @param[in] 	right_img cv_bridge wrap of the received right image message.
	 */
	void stereo_target_detect(cv_bridge::CvImageConstPtr left_img, cv_bridge::CvImageConstPtr right_img);

	/**
	 * @fn 			target_center
	 *
	 * @brief 		Computes target's center.
	 * 
	 * @details		The center is computed as the mean of corners' coordinates.
	 *
	 * @param[in] 	corners Image plane pixel coordinates of detected marker's corners
	 *
	 * @return 		Target center in image plane pixel coordinates.
	 */
    cv::Point2f target_center(std::vector<cv::Point2f> const &corners) const;


	/**
	 * @brief	Synchronizes left and right image topics based on exact matching of timestamps.
	 */
	std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image,sensor_msgs::msg::Image>> sync_exact_;
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_;	/**< Publisher of detected target location.*/
	message_filters::Subscriber<sensor_msgs::msg::Image> sub_right_img_;	/**< Receives the right image stream.*/
	message_filters::Subscriber<sensor_msgs::msg::Image> sub_left_img_;		/**< Receives the left image stream.*/
	
	/**
	 * @brief	ArUco dictionary of markers to be used.
	 *			See OpenCV's documentation on corresponding enum entries.
	 */
	cv::aruco::PredefinedDictionaryType aruco_dict_;
	cv::aruco::ArucoDetector detector_;		/**< ArUco marker detector.*/
	
	int marker_idx_;	/**< Index of ArUco marker to detect.*/
	double B;			/**< Horizontal displacement of stereo cameras (assume no rotations).*/
    double f;			/**< Focal length.*/
	double cx, cy;      /**< Center of left pinhole camera reference w.r.t. left image reference*/
};

#endif	// DETECTION_NODE_HPP_
