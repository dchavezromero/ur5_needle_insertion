#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>
#include <cmath>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "path_planning/srv/plan_motion.hpp"
#include "path_planning/srv/start_recording.hpp"

using namespace std::chrono_literals;

class MotionPlanningService : public rclcpp::Node
{
public:
  MotionPlanningService()
  : Node("motion_planning_service", 
         rclcpp::NodeOptions()
           .automatically_declare_parameters_from_overrides(true)
           .use_intra_process_comms(true)
           .append_parameter_override("use_sim_time", true))
  {
    // Initialize TF buffer and listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Create service for motion planning
    planning_service_ = this->create_service<path_planning::srv::PlanMotion>(
      "plan_motion",
      std::bind(&MotionPlanningService::handle_plan_motion, this, 
                std::placeholders::_1, std::placeholders::_2));
    
    // Create clients for data recorder services
    start_recording_client_ = this->create_client<path_planning::srv::StartRecording>("start_recording");
    stop_recording_client_ = this->create_client<std_srvs::srv::Trigger>("stop_recording");
    
    // Initialize MoveGroupInterface with a retry mechanism
    RCLCPP_INFO(this->get_logger(), "Waiting for move_group to become available...");
    
    // Create a timer to try initializing MoveIt periodically
    init_timer_ = this->create_wall_timer(
      std::chrono::seconds(3),
      std::bind(&MotionPlanningService::try_init_moveit, this)
    );
    
    // Subscribe to joint states to track the robot's current state independently
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states",
      rclcpp::SensorDataQoS(),
      std::bind(&MotionPlanningService::joint_state_callback, this, std::placeholders::_1)
    );
    
    RCLCPP_INFO(this->get_logger(), "Motion planning service is ready to handle requests");
    RCLCPP_INFO(this->get_logger(), "MoveIt initialization will continue in the background");
  }

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // Store the latest joint states
    latest_joint_state_ = *msg;
    have_joint_state_ = true;
  }
  
  void try_init_moveit() {
    if (is_moveit_ready_) {
      // Already initialized successfully
      init_timer_->cancel();
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Attempting to initialize MoveGroupInterface...");
    
    try {
      move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "ur5_arm");
      
      // Test if we can get joint names (this shouldn't throw)
      auto joint_names = move_group_->getJointNames();
      if (!joint_names.empty()) {
        RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized successfully");
        RCLCPP_INFO(this->get_logger(), "Available planning group: %s", 
                   move_group_->getName().c_str());
        
        // Display joint names
        std::string joint_names_str = "";
        for (const auto& name : joint_names) {
          joint_names_str += name + " ";
        }
        RCLCPP_INFO(this->get_logger(), "Joint names: %s", joint_names_str.c_str());
        
        // Store joint names for later use
        joint_names_ = joint_names;
        is_moveit_ready_ = true;
        init_timer_->cancel();
      } else {
        RCLCPP_WARN(this->get_logger(), "MoveGroupInterface created but joint names empty, retrying...");
      }
    }
    catch(const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "Failed to initialize MoveGroupInterface: %s. Will retry...", e.what());
    }
  }

  // Get default joint values (home position) when no joint states are available
  std::vector<double> get_default_joint_values() {
    // Default joint values for UR5 in a neutral position
    return {0.0, -1.57, 0.0, -1.57, 0.0, 0.0};
  }
  
  // Get current joint values from latest joint state or use defaults
  std::vector<double> get_current_joint_values() {
    if (!have_joint_state_ || latest_joint_state_.name.empty()) {
      RCLCPP_WARN(this->get_logger(), "No joint states received yet, using default joint values");
      return get_default_joint_values();
    }
    
    // Map latest joint state to the expected joint order
    std::vector<double> joint_values(joint_names_.size(), 0.0);
    
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      const auto& name = joint_names_[i];
      auto it = std::find(latest_joint_state_.name.begin(), latest_joint_state_.name.end(), name);
      if (it != latest_joint_state_.name.end()) {
        size_t index = std::distance(latest_joint_state_.name.begin(), it);
        if (index < latest_joint_state_.position.size()) {
          joint_values[i] = latest_joint_state_.position[index];
        }
      }
    }
    
    return joint_values;
  }

  // Get the pose of a point in the target frame relative to the source frame
  geometry_msgs::msg::Pose get_pose_from_tf(
      const std::string& target_frame,
      const std::string& source_frame)
  {
    geometry_msgs::msg::PoseStamped pose_stamped;
    geometry_msgs::msg::PoseStamped transformed_pose;
    
    // Initialize with identity pose
    pose_stamped.pose.orientation.w = 1.0;  // Identity quaternion
    pose_stamped.header.frame_id = source_frame;
    pose_stamped.header.stamp = this->get_clock()->now();
    
    try {
      // Wait for the transform to be available
      if (tf_buffer_->canTransform(target_frame, source_frame, 
                                 rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0)))
      {
        // Transform the pose to the target frame
        transformed_pose = tf_buffer_->transform(pose_stamped, target_frame);
        RCLCPP_INFO(this->get_logger(), "Successfully got transform from %s to %s", 
                   source_frame.c_str(), target_frame.c_str());
        return transformed_pose.pose;
      }
      else {
        RCLCPP_WARN(this->get_logger(), "Transform from %s to %s not available", 
                   source_frame.c_str(), target_frame.c_str());
        return pose_stamped.pose;  // Return original pose if transform is not available
      }
    }
    catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform pose: %s", ex.what());
      return pose_stamped.pose;  // Return original pose on error
    }
  }

  // Service callback for planning motion
  void handle_plan_motion(
    const std::shared_ptr<path_planning::srv::PlanMotion::Request> request,
    std::shared_ptr<path_planning::srv::PlanMotion::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Received motion planning request for target frame: %s", 
               request->target_frame.c_str());
    
    // Check if MoveIt is ready
    if (!is_moveit_ready_) {
      response->success = false;
      response->message = "MoveIt is not initialized or ready. Make sure move_group is running (launch ur5_moveit.launch.py first)";
      return;
    }
    
    // Get current joint values (from our tracked state, not MoveGroupInterface)
    std::vector<double> current_joint_values = get_current_joint_values();
    
    RCLCPP_INFO(this->get_logger(), "Current joint values: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
               current_joint_values[0], current_joint_values[1], current_joint_values[2],
               current_joint_values[3], current_joint_values[4], current_joint_values[5]);
    
    // Get target pose from TF with retry logic
    geometry_msgs::msg::Pose target_pose;
    bool has_valid_target_pose = false;
    
    // Try up to 3 times to get the transform
    for (int attempt = 0; attempt < 3 && !has_valid_target_pose; ++attempt) {
      try {
        target_pose = get_pose_from_tf("world", request->target_frame);
        
        // Validate the pose (checking for NaN values)
        if (std::isfinite(target_pose.position.x) && 
            std::isfinite(target_pose.position.y) && 
            std::isfinite(target_pose.position.z)) {
          
          RCLCPP_INFO(this->get_logger(), "Target pose: position [%.3f, %.3f, %.3f], orientation [%.3f, %.3f, %.3f, %.3f]",
                     target_pose.position.x, target_pose.position.y, target_pose.position.z,
                     target_pose.orientation.x, target_pose.orientation.y, 
                     target_pose.orientation.z, target_pose.orientation.w);
          has_valid_target_pose = true;
          break;
        } else {
          RCLCPP_WARN(this->get_logger(), "Received invalid target pose with NaN values (attempt %d/3)", 
                     attempt + 1);
          std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
      }
      catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to get target pose (attempt %d/3): %s", 
                   attempt + 1, e.what());
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    }
    
    // Check if we were able to get a valid target pose
    if (!has_valid_target_pose) {
      response->success = false;
      response->message = "Failed to get valid target pose after multiple attempts. "
                         "Make sure the target frame exists.";
      return;
    }
    
    // Configure the planning pipeline
    RCLCPP_INFO(this->get_logger(), "Planning with algorithm %s and timeout %.1f seconds",
               request->planning_algorithm.c_str(), request->planning_timeout);
    
    // Update the start state manually
    moveit::core::RobotStatePtr start_state(new moveit::core::RobotState(move_group_->getRobotModel()));
    start_state->setToDefaultValues();
    
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      start_state->setJointPositions(joint_names_[i], &current_joint_values[i]);
    }
    
    move_group_->setStartState(*start_state);
    
    move_group_->setPlannerId(request->planning_algorithm);
    move_group_->setPlanningTime(request->planning_timeout);
    move_group_->setNumPlanningAttempts(3);
    move_group_->setMaxVelocityScalingFactor(0.5);
    move_group_->setMaxAccelerationScalingFactor(0.5);
    
    // Set goal tolerances
    move_group_->setGoalOrientationTolerance(0.001);  // [rad]
    move_group_->setGoalPositionTolerance(0.001);     // [m]
    move_group_->setGoalJointTolerance(0.001);        // [rad]
    
    // Set the pose target
    move_group_->setPoseTarget(target_pose);
    
    // Plan the motion
    auto start_time = this->now();
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    
    bool success = false;
    try {
      success = static_cast<bool>(move_group_->plan(plan));
      
      if (success) {
        auto end_time = this->now();
        
        // Calculate planning time
        response->planning_time = (end_time - start_time).seconds();
        response->success = true;
        response->message = "Motion planning succeeded";
        response->trajectory_points = static_cast<int32_t>(plan.trajectory_.joint_trajectory.points.size());
        
        // Calculate expected execution time
        if (!plan.trajectory_.joint_trajectory.points.empty()) {
          response->trajectory_time = 
            plan.trajectory_.joint_trajectory.points.back().time_from_start.sec +
            plan.trajectory_.joint_trajectory.points.back().time_from_start.nanosec * 1e-9;
        }
        
        RCLCPP_INFO(this->get_logger(), "Planning succeeded in %.2f seconds with %d trajectory points",
                   response->planning_time, response->trajectory_points);
        
        // Execute the plan if requested
        if (request->execute_plan) {
          // Start recording data before execution
          if (start_recording_client_->service_is_ready()) {
            auto recording_request = std::make_shared<path_planning::srv::StartRecording::Request>();
            recording_request->planning_time = response->planning_time;
            recording_request->target_frame = request->target_frame;
            
            RCLCPP_INFO(this->get_logger(), "Starting data recording before trajectory execution...");
            RCLCPP_INFO(this->get_logger(), "Planning time: %.3f seconds, Target frame: %s", 
                       response->planning_time, request->target_frame.c_str());
            
            start_recording_client_->async_send_request(recording_request,
              [this](rclcpp::Client<path_planning::srv::StartRecording>::SharedFuture future) {
                auto result = future.get();
                if (result->success) {
                  RCLCPP_INFO(this->get_logger(), "Data recording started: %s", result->message.c_str());
                } else {
                  RCLCPP_WARN(this->get_logger(), "Failed to start data recording: %s", result->message.c_str());
                }
              });
          } else {
            RCLCPP_WARN(this->get_logger(), "Data recorder service not available");
          }
          
          // Execute the plan
          RCLCPP_INFO(this->get_logger(), "Executing motion plan...");
          auto execution_result = move_group_->execute(plan);
          
          // Stop recording after execution completes
          if (stop_recording_client_->service_is_ready()) {
            auto stop_request = std::make_shared<std_srvs::srv::Trigger::Request>();
            RCLCPP_INFO(this->get_logger(), "Stopping data recording after trajectory execution...");
            
            stop_recording_client_->async_send_request(stop_request,
              [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                auto result = future.get();
                if (result->success) {
                  RCLCPP_INFO(this->get_logger(), "Data recording stopped: %s", result->message.c_str());
                } else {
                  RCLCPP_WARN(this->get_logger(), "Failed to stop data recording: %s", result->message.c_str());
                }
              });
          }
          
          if (execution_result == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Plan executed successfully");
            response->message += ", execution completed successfully";
          } else {
            RCLCPP_ERROR(this->get_logger(), "Plan execution failed");
            response->message += ", but execution failed";
          }
        }
      } else {
        response->success = false;
        response->message = "Motion planning failed - could not find a valid path";
        RCLCPP_ERROR(this->get_logger(), "Planning failed!");
      }
    }
    catch (const std::exception& e) {
      response->success = false;
      response->message = std::string("Motion planning failed with exception: ") + e.what();
      RCLCPP_ERROR(this->get_logger(), "Planning exception: %s", e.what());
    }
  }

  // MoveIt and TF components
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // Joint state tracking
  sensor_msgs::msg::JointState latest_joint_state_;
  bool have_joint_state_ = false;
  std::vector<std::string> joint_names_;
  
  // Service, subscribers and clients
  rclcpp::Service<path_planning::srv::PlanMotion>::SharedPtr planning_service_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Client<path_planning::srv::StartRecording>::SharedPtr start_recording_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_recording_client_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  
  // State tracking
  bool is_moveit_ready_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionPlanningService>());
  rclcpp::shutdown();
  return 0;
} 