/**
 * @file    split_stereo.hpp
 * 
 * @author  Pietro Moriello
 *
 * @date    2024-10-08
 * 
 * @brief   Header file for SLStereoSplit node.
 */
#ifndef SPLIT_STEREO_HPP_
#define SPLIT_STEREO_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include "rcpputils/asserts.hpp"

#include <string>
#include <memory>
#include <iostream>
#include <functional>
#include <exception>
#include <algorithm>
#include <queue>
#include <cmath>

/**
 * @class   SLStereoSplit 
 * 
 * @brief   Node for pre-processing of stereo images produced by a StereoLabs camera.
 * 
 * @details This node takes as input the left-right concateneted frame that are published
 *          on the stereo image topic of StereoLabs cameras, divides them into two distinct
 *          images and then publishes them to their respective topic. 
 */
class SLStereoSplit : public rclcpp::Node
{
public:
	/**
	* @fn     SLStereoSplit
	* 
	* @brief  Constructs a new SLStereoSplit object.
	*/
	SLStereoSplit();

private:
    /**
     * @fn          img_callback
     *
     * @brief       Callback to recieve stereo image from a Zed camera.
     * 
     * @details     This callback expects a rectified color image from a Zed camera.
     *
     * @param[in]   img Received image. It is supposed to be rectified and colored. 
     */
	void img_callback(const sensor_msgs::msg::Image::SharedPtr img);
    
    /**
     * @fn          split_stereo
     * 
     * @brief       Splits stereo image into left and right images.
     * 
     * @details     Zed stereo images are published as a single frame image with left and right view concatenated width-wise. 
     *              This functions splits the image into two different views. 
     *              It expects a frame with an even number of columns, otherwise an exception is thrown.
     *
     * @param[in]   img Original image wrapped by cv_bridge.
     * @param[out]  left Stores the resulting left image after the split.
     * @param[out]  right Stores the resulting right image after the split.
     */
    void split_stereo(cv_bridge::CvImageConstPtr img, cv::Mat &left, cv::Mat &right);


    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_;  /**< Receives stereo image published by a Zed camera topic.*/
  	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_left_;    /**< Publisher for the left image component.*/
  	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_right_;   /**< Publisher for the right image component.*/
};

#endif  // SPLIT_STEREO_HPP_
