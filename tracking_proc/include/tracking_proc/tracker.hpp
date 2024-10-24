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
 * @details The node receives messages containing a target location and cluster centroids,
 *          performs sensor fusion to identify the object of interest, and tracks it over time
 *          using a Kalman Filter to predict its future state.
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
     * @brief       Callback function for target location incoming from the vision node.
     * 
     * @details     Transforms the incoming target position to the "base_link" frame using the tf2 buffer.
     *              If the transformation is successful, the target is stored.
     *              If the received target location is the first message received from this node, there 
     *              are no centroids available yet, thus sensor fusion cannot be applied, the function
     *              handle this edge case and returns without performing sensor fusion and tracking.
     *              Otherwise, the `trackerCore` function is called to perform sensor fusion and tracking.
     *
     * @param[in]   target_msg Point (x, y, z) containing the target position.
     */
    void target_callback(const geometry_msgs::msg::PointStamped::SharedPtr target_msg);

    /**
     * @fn          centroids_callback
     * 
     * @brief       Callback function for clusters centroids incoming from the clustering node.
     * 
     * @details     Transforms the incoming clusters centroids to the "base_link" frame using the tf2 buffer.
     *              If the transformation is successful, the target is stored.
     *              If the received centroids are the first message received from this node, there 
     *              is no target location available yet, thus sensor fusion cannot be applied, the function
     *              handle this edge case and returns without performing sensor fusion and tracking.
     *              Otherwise, the `trackerCore` function is called to perform sensor fusion and tracking.
     *
     * @param[in]   clusters_msg Array of poses containing the clusters centroids.
     */
    void centroids_callback(const geometry_msgs::msg::PoseArray::SharedPtr clusters_msg);
    
    /**
     * @fn          trackerCore
     *
     * @brief       Executes the logic of the tracking node.
     * 
     * @details     This function performs sensor fusion between the target location and the centroids,
     *              applies the Kalman Filter for object tracking, and predicts the future position of
     *              the tracked object. The tracked position is then published as a message.
     */
    void trackerCore();

    /**
     * @fn      sensorFusion
     *
     * @brief   Performs sensor fusion between vision and clustering nodes messages.
     *
     * @details The algorithm peforms sensor fusion by creating a minimum problem.
     *          The target location received from the vision node is used as a reference to
     *          find the correct centroid in the vector of centroids received from the clustering node.
     *          By minimizing the Euclidean distance between the target location and the
     *          centroids, the right cluster is selected as the target object.
     *
     * @return  Eigen::Vector3d Centroid (x, y, z) that minimizes the distances with the target location.
     */
    Eigen::Vector3d sensorFusion();

    /**
     * @fn          pointToEigen
     * 
     * @brief       Converts a PointStamped message into an Eigen::VectorXd point.
     * 
     * @param[in]   pt PointStamped message.
     *
     * @return      Eigen::VectorXd The point converted into an Eigen vector.
     */
    Eigen::VectorXd pointToEigen(const geometry_msgs::msg::PointStamped& pt);

    /**
     * @fn          poseArrayToEigen
     * 
     * @brief       Converts a PoseArray message into a vector of Eigen::VectorXd points.
     * 
     * @param[in]   centroids PoseArray message.
     *
     * @return      std::vector<Eigen::VectorXd> A std vector of centroids converted into Eigen vectors.
     */
    std::vector<Eigen::VectorXd> poseArrayToEigen(const geometry_msgs::msg::PoseArray& centroids);


    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subTargetLoc_;        /**< Subscriver to target location.*/
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subClusterCentroids_;    /**< Subscriber to clusters centroids.*/
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_;                           /**< Publisher of detected target location.*/

    KalmanFilter kalman_;   /**< Kalman Filter object to peform the tracking.*/
    
    bool isInitialized_;    /**< Variable used to check for initialization of the tracked object.*/
    
    rclcpp::Time previousTime_; /**< Previous timestamp used to update the Kalman Filter matrices.*/

    Eigen::VectorXd currAlpha_;                 /**< Current target location used to perform sensor fusion.*/
    Eigen::VectorXd beta_;                      /**< Input of the Kalman Filter.*/
    Eigen::VectorXd betaPred_;                  /**< Output of the Kalman Filter (tracked object position).*/
    std::vector<Eigen::VectorXd> centroids_;    /**< Vector of centroids (x, y, z) used in sensor fusion.*/

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;                /**< Buffer to get available static transforms.*/
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;   /**< Transform listener.*/
};

#endif  // TRACKER_HPP_
