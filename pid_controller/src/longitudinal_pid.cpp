#include "longitudinal_pid.hpp"

// Used for parameter binding in callbacks to be registered with subscribers.
using std::placeholders::_1;

LongitudinalPIDNode::LongitudinalPIDNode()
    : Node("longitudinal_ctr"),
      Kp(0.2f), Ki(0.01f), Kd(0.15f), // PID coefficients
      delta_time(0.5f),  // Time constant for PID control
      integrator(0.0f), integrator_min(-0.4f),
      integrator_max(5.0f), prev_error(0.0f),
      differentiator(0.0f), prev_measurement(0.0f),
      pid_output(0.0f), current_speed(0.0f), // Initialize current speed
      current_position{0.0f, 0.0f},          // Initialize Position struct for current position
      target_position{0.0f, 0.0f}            // Initialize Position struct for target position
{
    // Initialize PID controller
    init_pid();

    // Defines quality of service: all messages that you want to receive must have the same
	rclcpp::QoS custom_qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
	.history(rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST)
	.keep_last(10)
	.reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
	.durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE)
	.avoid_ros_namespace_conventions(false);

    // Subscribe to odometry to track car's position
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/ego_racecar/odom", 10, std::bind(&LongitudinalPIDNode::odom_callback, this, std::placeholders::_1));

    // Subscribe to target position messages
    target_position_sub = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/tracked_obj_loc", custom_qos_profile, std::bind(&LongitudinalPIDNode::target_position_callback, this, std::placeholders::_1));

    // Publisher for throttle command
    throttle_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
}

// Callback for odometry data to update current position
void LongitudinalPIDNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_position.x = msg->pose.pose.position.x; // Update current x position
    current_position.y = msg->pose.pose.position.y; // Update current y position
    current_speed = msg->twist.twist.linear.x;      // Update current speed
}

// Callback for target position
void LongitudinalPIDNode::target_position_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg_tracker)
{
    target_position.x = msg_tracker->point.x; // Set target x position
    target_position.y = msg_tracker->point.y; // Set target y position

    control_loop();
}

// Control loop method that runs periodically to update the throttle command
void LongitudinalPIDNode::control_loop()
{
    // Calculate distance to the target position
    float distance = sqrt(pow((target_position.x - current_position.x), 2) +
                          pow((target_position.y - current_position.y), 2));

    // Update PID controller with the current distance to the target
    float throttle = update_pid(distance, current_speed);

    // Create and publish throttle command
    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    drive_msg.drive.speed = throttle; // Set throttle based on PID output
    throttle_pub->publish(drive_msg); // Publish the drive command
}

// Initialize PID variables
void LongitudinalPIDNode::init_pid()
{
    integrator = 0.0f;
    prev_error = 0.0f;
    differentiator = 0.0f;
    prev_measurement = 0.0f;
    pid_output = 0.0f;
}

// Update PID output based on distance to target and current speed
float LongitudinalPIDNode::update_pid(float distance_to_target, float current_speed)
{
    // Threshold for stopping
    const float stop_threshold = 2.0f; // Distance threshold to stop (2 meters)

    // If the car is near the target, set throttle to zero and reset integrator
    if (distance_to_target < stop_threshold)
    {
        RCLCPP_INFO(this->get_logger(), "Car is near the target, stopping %f", distance_to_target);
        pid_output = 0.0f; // Stop the car (throttle = 0)
        integrator = 0.0f; // Reset the integrator to avoid windup
        return pid_output; // Return the output immediately
    }

    // Calculate the error (distance to target)
    float error = distance_to_target;

    // Proportional term
    float proportional = Kp * error;

    // Integral term with anti-windup using the trapezoidal rule
    integrator += 0.5f * Ki * delta_time * (error + prev_error);

    // Clamp integrator to its min and max values
    if (integrator > integrator_max)
    {
        integrator = integrator_max;
    }
    else if (integrator < integrator_min)
    {
        integrator = integrator_min;
    }

    // Derivative term (based on change in error over time)
    differentiator = Kd * (error - prev_error) / delta_time;

    // Compute final PID output (throttle) and apply limits
    pid_output = proportional + integrator + differentiator - current_speed; // Subtracting current speed for smooth control

    // Store the current error for the next update
    prev_error = error;

    return pid_output; // Return the calculated throttle
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<LongitudinalPIDNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}