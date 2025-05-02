#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

using namespace std::chrono_literals;

class EndEffectorVelocityTracker : public rclcpp::Node
{
public:
    EndEffectorVelocityTracker()
    : Node("ee_velocity_tracker", 
           rclcpp::NodeOptions()
             .automatically_declare_parameters_from_overrides(true)
             .append_parameter_override("use_sim_time", true))
    {
        // Publisher for velocity data
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "ee_velocity", 100);
        
        // Set up TF listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Create a timer for tracking velocity
        timer_ = this->create_wall_timer(
            5ms,  // 200Hz
            std::bind(&EndEffectorVelocityTracker::track_velocity, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "Simple EE velocity tracker started (200Hz)");
    }

private:
    void track_velocity()
    {
        try {
            // Get current transform
            geometry_msgs::msg::TransformStamped transform;
            try {
                transform = tf_buffer_->lookupTransform(
                    "world",
                    "needle",
                    tf2::TimePointZero
                );
            } catch (const tf2::TransformException& ex) {
                return; // Silently return if transform not available
            }
            
            // If first reading, just store it and return
            if (!have_prev_transform_) {
                prev_transform_ = transform;
                prev_time_ = this->now();
                have_prev_transform_ = true;
                return;
            }
            
            // Current time
            auto current_time = this->now();
            
            // Calculate time difference in seconds
            double dt = (current_time - prev_time_).seconds();
            if (dt < 0.001) {
                return; // Skip if time difference too small
            }
            
            // Calculate linear velocity components directly
            double vx = (transform.transform.translation.x - prev_transform_.transform.translation.x) / dt;
            double vy = (transform.transform.translation.y - prev_transform_.transform.translation.y) / dt;
            double vz = (transform.transform.translation.z - prev_transform_.transform.translation.z) / dt;
            
            // Create velocity message
            geometry_msgs::msg::TwistStamped velocity_msg;
            velocity_msg.header = transform.header;
            velocity_msg.twist.linear.x = vx;
            velocity_msg.twist.linear.y = vy;
            velocity_msg.twist.linear.z = vz;
            
            // Angular velocity is left at zeros - we don't need it
            
            // Publish velocity
            velocity_publisher_->publish(velocity_msg);
            
            // Store current values for next iteration
            prev_transform_ = transform;
            prev_time_ = current_time;
        }
        catch (const std::exception& e) {
            // Silently handle errors
        }
    }
    
    // Components
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // State tracking
    bool have_prev_transform_ = false;
    geometry_msgs::msg::TransformStamped prev_transform_;
    rclcpp::Time prev_time_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EndEffectorVelocityTracker>());
    rclcpp::shutdown();
    return 0;
} 