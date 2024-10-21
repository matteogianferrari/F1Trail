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
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose"
#include <eigen3/Eigen/Dense>

#include "kalman_filter.hpp"
#include <vector>

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
    void target_callback(const geometry_msgs::msg::Point::SharedPtr stereo_msg);

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
    void trackerCore();

    /**
     * @brief Construct a new select Centroid object
     * 
     */
    selectCentroid();

    /**
     * @brief 
     * 
     * @param point 
     * @return Eigen::VectorXd 
     */
    Eigen::VectorXd pointToEigen(const geometry_msgs::msg::Point& point);

    /**
     * @brief 
     * 
     * @param centroids 
     * @return 
     */
    std::vector<Eigen::VectorXd> poseArrayToEigen(const geometry_msgs::msg::PoseArray& centroids);


    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subTargetLoc_;  /**< Subscription for target location (LaserScan message). */
    
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subClusterCentroids_;  /**< Subscription for cluster centroids. */

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_;           /**< Publisher of detected target location.*/

    KalmanFilter kalman_;                       /**< */
    
    bool isInitialized_;                        /**< */
    
    Eigen::VectorXd currAlpha_;                 /**< */
    Eigen::VectorXd beta_;                      /**< */
    Eigen::VectorXd betaPred_;                  /**< */
    
    std::vector<Eigen::VectorXd> centroids_;   /**< */
}

#endif  // TRACKER_HPP_
