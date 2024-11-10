#include "longitudinal_pid.hpp"


// Used for parameter binding in callbacks to be registered with subscribers.
using std::placeholders::_1;


LongitudinalPIDNode::LongitudinalPIDNode() : Node("longitudinal_ctr"),
    Kp(3.0f), Ki(0.05f), Kd(0.2f),
    integrator(0.0f), integrator_min(-0.3f), integrator_max(2.0f),
    prev_error(0.0f), differentiator(0.0f), prev_measurement(0.0f),
    pid_output(0.0f), current_speed(0.0f),
    current_position{0.0f, 0.0f}, target_position{0.0f, 0.0f}           
{
    // Gets all potential parameters
	this->declare_parameter("threshold_distance", 2.0);
	threshold_distance = this->get_parameter("threshold_distance").as_double();

    // Defines quality of service: all messages that you want to receive must have the same
	rclcpp::QoS custom_qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
	.history(rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST)
	.keep_last(10)
	.reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
	.durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE)
	.avoid_ros_namespace_conventions(false);

    // Subscribes to odometry to track car's position
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/ego_racecar/odom", custom_qos_profile, std::bind(&LongitudinalPIDNode::odom_callback, this, std::placeholders::_1));

    // Subscribes to target position messages
    target_position_sub = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/tracked_obj_loc", custom_qos_profile, std::bind(&LongitudinalPIDNode::target_position_callback, this, std::placeholders::_1));

    // Publisher for throttle command
    throttle_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

    prev_time = this->now();
}


void LongitudinalPIDNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Updates the current position and speed of the vehicle
    current_position.x = msg->pose.pose.position.x; 
    current_position.y = msg->pose.pose.position.y; 
    current_speed = msg->twist.twist.linear.x;
}


void LongitudinalPIDNode::target_position_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg_tracker) {
    // Updates the target position
    target_position.x = msg_tracker->point.x; 
    target_position.y = msg_tracker->point.y;

    // Executes the PID control loop
    control_loop();
}


void LongitudinalPIDNode::control_loop() {
    // Computes the distance to the target position
    float distance = sqrt(pow((target_position.x - current_position.x), 2) +
                          pow((target_position.y - current_position.y), 2));

    // Updates PID controller with the current distance to the target
    float throttle = update_pid(distance, current_speed);

    // Creates and publish throttle command
    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    drive_msg.drive.speed = throttle; 
    throttle_pub->publish(drive_msg); 
}


float LongitudinalPIDNode::update_pid(float distance_to_target, float current_speed) {
    // Calculate the error as the difference between distance to target and threshold distance to keep from target
    float error = (distance_to_target - threshold_distance);
    double now = this->now();

    // If target is forward and error is small, stop the car and reset PID to avoid instability
    if (error > 0 && error < 0.1) {
        RCLCPP_INFO(this->get_logger(), "Car is near the target, stopping %f", distance_to_target);

        // Stops the car (throttle = 0) and resets the integrator to avoid windup and previous error to avoid derivative spikes
        pid_output = 0.0f;
        integrator = 0.0f;
        prev_error = 0.0f;
        prev_time = now;
        return pid_output;
    }
    double dt = prev_time - this->now();
    prev_time = now;

    // Proportional term
    float proportional = Kp * error;

    // Integral term with anti-windup using the trapezoidal rule
    integrator += 0.5f * Ki * dt * (error + prev_error);

    // Clamp integrator to its min and max values
    if (integrator > integrator_max) {
        integrator = integrator_max;
    }
    else if (integrator < integrator_min) {
        integrator = integrator_min;
    }

    // Derivative term (based on change in error over time)
    differentiator = Kd * (error - prev_error) / dt;

    // Compute final PID output (throttle) and apply limits.
    // Subtracting current speed for smooth control.
    pid_output = proportional + integrator + differentiator - current_speed;

    // Stores the current error for the next update
    prev_error = error;

    return pid_output;
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<LongitudinalPIDNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}