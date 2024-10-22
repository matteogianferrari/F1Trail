#include "tracker.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <string>


// Used for parameter binding in callbacks to be registered with subscribers.
using std::placeholders::_1;


TrackerNode::TrackerNode(): Node("tracking_module") {
    isInitialized_ = false;
    tf_buffer_ {nullptr};            
    tf_listener_ {nullptr};

    // Gets all potential parameters
    this->declare_parameter("target_location_topic", "/target_loc");
    std::string target_location_topic = this->get_parameter("target_location_topic").as_string();
    
    this->declare_parameter("cluster_centroids_topic", "/cluster_centroids");
    std::string cluster_centroids_topic = this->get_parameter("cluster_centroids_topic").as_string();
    
    // Defines quality of service: all messages that you want to receive must have the same
    rclcpp::QoS custom_qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
    .history(rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST)
    .keep_last(10)
    .reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
    .durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE)
    .avoid_ros_namespace_conventions(false);
    
    // create transform buffer and listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscribes to target topic
    subTargetLoc_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        target_location_topic, custom_qos_profile, std::bind(&TrackerNode::target_callback, this, std::placeholders::_1));

    // Subscribes to the clustering topic
    subClusterCentroids_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        cluster_centroids_topic, custom_qos_profile, std::bind(&TrackerNode::centroids_callback, this, std::placeholders::_1));

    // create publisher for target location point
    pub_ = this->create_publisher<geometry_msgs::msg::Point>("/tracked_obj_loc", custom_qos_profile);

    // set initial time to compute delta
    previousTime_ = this->now();
}


void TrackerNode::target_callback(const geometry_msgs::msg::PointStamped::SharedPtr target_msg) {
    RCLCPP_INFO(this->get_logger(), "Received target location from camera tracking node.");

    // Performs frame transform on incoming data
    geometry_msgs::msg::PointStamped tf_target_msg;
    try {
        tf_target_msg = tf_buffer_->transform(*target_msg, "base_link");
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s",
                    "base_link", target_msg->header.frame_id.c_str(), ex.what());

        return;
    }

    // Stores the target location received from the vision node
    currAlpha_ = pointToEigen(tf_target_msg);

    // Edge case: No centroids have been received yet.
    // Sensor fusion can only be applied if both target location and centroids are available.
    // See the function details for an in-depth explanation.
    if (centroids_.empty()) {
        RCLCPP_WARN(this->get_logger(), "No centroids available yet.");
        return;
    }
    
    // Tracks the object
    trackerCore(tf_target_msg.header.stamp);
}


void TrackerNode::centroids_callback(const geometry_msgs::msg::PoseArray::SharedPtr clusters_msg) {
    RCLCPP_INFO(this->get_logger(), "Received clusters centroids from clustering node.");

    // Performs frame transform on incoming data (ROS at its best here)
    geometry_msgs::msg::PoseArray tf_clusters_msg;
    try {
        // Either we do this, or we implement a conversion following tf2 templates between Pose and PoseStamped
        geometry_msgs::msg::PoseStamped out;

        for (const auto& pose: clusters_msg->poses) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = clusters_msg->header;
            ps.pose = pose;
            tf_clusters_msg.poses.push_back(tf_buffer_->transform(ps, out, "base_link").pose);
        }
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s",
                    "base_link", clusters_msg->header.frame_id.c_str(), ex.what());
        return;
    }

    // Stores the centroids received from the clustering node
    centroids_ = poseArrayToEigen(tf_clusters_msg);

    // Edge case: No target location has been received yet.
    // Sensor fusion can only be applied if both target location and centroids are available.
    // See the function details for an in-depth explanation.
    if (currAlpha_.size() == 0) {
        RCLCPP_WARN(this->get_logger(), "No target location received yet.");
        return;
    }

    // Tracks the object
    trackerCore(tf_clusters_msg.header.stamp);
}


void TrackerNode::trackerCore(const rclcpp::Time& timestamp) {
    // Applies sensor fusion between vision and clustering nodes
    // Selects the centroid that minimizes the distance w.r.t. the target location currAlpha
    beta_ = sensorFusion();

    // Checks if the kalman filter has been already initialized
    if (!isInitialized_) {
        isInitialized_ = true;
        kalman_.setState(beta_);
    }

    // Computes the delta time between messages
    rclcpp::Duration dt = timestamp - previousTime_;
    previousTime_ = timestamp;

    // Updates the kalman filter matrices
    kalman_.updateMatrices(dt.seconds());

    // Estimates the future state of the tracked object
    kalman_.predict();

    // Updates the estimation with the measurement
    kalman_.update(beta_);

    // Gets the object state (x, y, z) from kalman filter
    betaPred_ = kalman_.getState();
    currAlpha_ = betaPred_;

    // Transforms Eigen to Point
    geometry_msgs::msg::Point tracked;
    tracked.x = betaPred_[0];
    tracked.y = betaPred_[1];
    tracked.z = betaPred_[2];

    RCLCPP_INFO(this->get_logger(), "Target Location: [x: %f, y: %f, z: %f]", 
                betaPred_[0], betaPred_[1], betaPred_[2]);

    // Publish the position of the tracked object
    pub_->publish(tracked);
}


Eigen::Vector3d TrackerNode::sensorFusion() {
    // Initializes variables to keep track of the closest centroid and minimum distance
    double minDistance = std::numeric_limits<double>::max(); 
    Eigen::Vector3d closestCentroid = Eigen::Vector3d::Zero();

    // Iterates over all centroids and computes the distance to the target location
    for (const auto& centroid : centroids_) {
        // Computes Euclidean distance between the current centroid and the target location
        double distance = (centroid - currAlpha_).norm();

        if (distance < minDistance) {
            minDistance = distance;
            closestCentroid = centroid;
        }
    }

    return closestCentroid;
}


Eigen::VectorXd TrackerNode::pointToEigen(const geometry_msgs::msg::PointStamped& pt) {
    // Creates an Eigen::VectorXd of size 3
    Eigen::VectorXd vec(3);
    vec << pt.point.x, pt.point.y, pt.point.z;

    return vec;
}


std::vector<Eigen::VectorXd> TrackerNode::poseArrayToEigen(const geometry_msgs::msg::PoseArray& centroids) {
    std::vector<Eigen::VectorXd> eigenCentroids;

    for (const auto& point : centroids.poses) {
        Eigen::VectorXd centroid(3);
        
        // Assign the position (x, y, z)
        centroid << point.position.x, point.position.y, point.position.z;

        eigenCentroids.push_back(centroid);
    }

    return eigenCentroids;
}


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TrackerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
