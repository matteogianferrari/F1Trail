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

/**
 * @class   Tracker 
 * 
 * @brief   Node for tracking an object and estimating its future state.
 * 
 * @details  
 */
class Tracker : public rclcpp::Node
{
public:
    /**
     * @fn      Tracker
     * 
     * @brief   Construct a new Tracker object.
     */
    Tracker();

    /**
     * @fn          target_callback
     * 
     * @brief 
     * 
     * @details
     *
     * @param[in]
     */
    void target_callback(const ensor_msgs::msg::LaserScan::SharedPtr stereo_msg);

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
    
private:
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
     * @return Eigen::MatrixXd 
     */
    Eigen::MatrixXd poseArrayToEigen(const geometry_msgs::msg::PoseArray& centroids);


    rclcpp::Subscription<geometry_msgs::msg::Point> subTargetLoc_;

    rclcpp::Subscription<geometry_msgs::msg::PoseArray> subClusterCentroids_;

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_;			/**< Publisher of detected target location.*/

    KalmanFilter kalman_;                       /**< */
    
    bool isInitialized_;                        /**< */
    
    Eigen::VectorXd currAlpha_;                 /**< */
    Eigen::VectorXd beta_;                      /**< */
    Eigen::VectorXd betaPred_;                  /**< */
    
    Eigen::MatrixXd centroids_;   /**< */
}

#endif  // TRACKER_HPP_
