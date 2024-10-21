/**
 * @file    tracker.hpp
 * 
 * @author  Matteo Gianferrari
 *
 * @date    2024-10-17
 * 
 * @brief   Header file for Tracking node.
 */
#ifndef TRACKER_HPP_
#define TRACKER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <eigen3/Eigen/Dense>

#include "kalman_filter.hpp"
#include <vector>
#include <memory>

/**
 * @class   TrackerNode 
 * 
 * @brief   Node for tracking an object and estimating its future state.
 * 
 * @details  
 */
class TrackerNode : public rclcpp::Node
{
public:
    /**
     * @fn      TrackerNode
     * 
     * @brief   Construct a new TrackerNode object.
     */
    TrackerNode();

private:
    /**
     * @fn          target_callback
     * 
     * @brief 
     * 
     * @details
     *
     * @param[in]
     */
    void target_callback(const geometry_msgs::msg::PointStamped::SharedPtr stereo_msg);

    /**
     * @fn          centroids_callback
     * 
     * @brief 
     * 
     * @details
     *
     * @param[in]
     */
    void centroids_callback(const geometry_msgs::msg::PoseArray::SharedPtr cluster_msg);
    
    /**
     * @brief Construct a new tracker Core object
     * 
     */
    void trackerCore(const rclcpp::Time& timestamp);

    /**
     * @brief Construct a new select Centroid object
     * 
     */
    Eigen::Vector3d selectCentroid();

    /**
     * @brief 
     * 
     * @param point 
     * @return Eigen::VectorXd 
     */
    Eigen::VectorXd pointToEigen(const geometry_msgs::msg::PointStamped& point);

    /**
     * @brief 
     * 
     * @param centroids 
     * @return 
     */
    std::vector<Eigen::VectorXd> poseArrayToEigen(const geometry_msgs::msg::PoseArray& centroids);


    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subTargetLoc_;  /**< Subscription for target location (LaserScan message). */
    
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subClusterCentroids_;  /**< Subscription for cluster centroids. */

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_;           /**< Publisher of detected target location.*/

    KalmanFilter kalman_;                       /**< */
    
    bool isInitialized_;                        /**< */
    
    Eigen::VectorXd currAlpha_;                 /**< */
    Eigen::VectorXd beta_;                      /**< */
    Eigen::VectorXd betaPred_;                  /**< */
    
    std::vector<Eigen::VectorXd> centroids_;    /**< */
    rclcpp::Time previousTime_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_{nullptr};                 /**< Buffer to get available static transforms */
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};   /**< Transform listener */
};

#endif  // TRACKER_HPP_
