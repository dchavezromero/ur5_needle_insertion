#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <thread>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <std_srvs/srv/trigger.hpp>  // For data recorder services

// Get the pose of a point in the target frame relative to the source frame
geometry_msgs::msg::Pose get_pose_from_tf(
    tf2_ros::Buffer* tf_buffer,
    const std::shared_ptr<rclcpp::Node>& node,
    const std::string& target_frame,
    const std::string& source_frame)
{
  geometry_msgs::msg::PoseStamped pose_stamped;
  geometry_msgs::msg::PoseStamped transformed_pose;
  
  // Initialize with identity pose
  pose_stamped.pose.orientation.w = 1.0;  // Identity quaternion
  pose_stamped.header.frame_id = source_frame;
  pose_stamped.header.stamp = node->get_clock()->now();
  
  try
  {
    // Wait for the transform to be available
    if (tf_buffer->canTransform(target_frame, source_frame, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0)))
    {
      // Transform the pose to the target frame
      transformed_pose = tf_buffer->transform(pose_stamped, target_frame);
      RCLCPP_INFO(node->get_logger(), "Successfully got transform from %s to %s", 
                 source_frame.c_str(), target_frame.c_str());
      return transformed_pose.pose;
    }
    else
    {
      RCLCPP_WARN(node->get_logger(), "Transform from %s to %s not available", 
                 source_frame.c_str(), target_frame.c_str());
      return pose_stamped.pose;  // Return original pose if transform is not available
    }
  }
  catch (const tf2::TransformException &ex)
  {
    RCLCPP_WARN(node->get_logger(), "Could not transform pose: %s", ex.what());
    return pose_stamped.pose;  // Return original pose on error
  }
}

// Simplified function to compute IK and create a plan (but not execute)
bool compute_ik_and_plan(
    std::shared_ptr<rclcpp::Node> node,
    moveit::planning_interface::MoveGroupInterface& move_group,
    const std::vector<double>& current_joint_values,
    const geometry_msgs::msg::Pose& target_pose,
    moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const std::string& planning_algorithm = "RRTstar",
    const float& planning_time = 10.0)
{
  RCLCPP_INFO(node->get_logger(), "Computing IK for target pose");
  
  // Define joint names in the order MoveIt expects them
  std::vector<std::string> joint_names = {
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
  };
  
  // *** Skip IK computation and directly use MoveGroup's IK capabilities ***
  
  if (current_joint_values.size() != 6) {
    RCLCPP_ERROR(node->get_logger(), "Invalid joint values (got %zu values, expected 6)", current_joint_values.size());
    return false;
  }
  
  // Log the current joint values
  RCLCPP_INFO(node->get_logger(), "Current state - Shoulder pan: %.4f, Shoulder lift: %.4f, Elbow: %.4f, Wrist1: %.4f, Wrist2: %.4f, Wrist3: %.4f", 
             current_joint_values[0], current_joint_values[1], current_joint_values[2], 
             current_joint_values[3], current_joint_values[4], current_joint_values[5]);
  
  RCLCPP_INFO(node->get_logger(), "Setting target pose with position [%.3f, %.3f, %.3f]",
             target_pose.position.x, target_pose.position.y, target_pose.position.z);
  
  // Configure the planning pipeline
  move_group.setPlannerId(planning_algorithm);
  move_group.setPlanningTime(planning_time);
  move_group.setNumPlanningAttempts(3);
  move_group.setMaxVelocityScalingFactor(0.5);
  move_group.setMaxAccelerationScalingFactor(0.5);
  
  // Set goal tolerances
  move_group.setGoalOrientationTolerance(0.001);  // [rad]
  move_group.setGoalPositionTolerance(0.001);    // [m]
  move_group.setGoalJointTolerance(0.001);       // [rad]
  
  // Set the pose target directly
  move_group.setPoseTarget(target_pose);
  
  // Plan (but don't execute)
  bool success = static_cast<bool>(move_group.plan(plan));
  
  if (success) {
    RCLCPP_INFO(node->get_logger(), "Planning succeeded! Plan has %zu waypoints over %.2f seconds", 
               plan.trajectory_.joint_trajectory.points.size(),
               plan.trajectory_.joint_trajectory.points.back().time_from_start.sec + 
               plan.trajectory_.joint_trajectory.points.back().time_from_start.nanosec * 1e-9);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Planning failed!");
  }
  
  return success;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  // Create node with sim_time enabled
  auto node = std::make_shared<rclcpp::Node>("move_ur5", 
                              rclcpp::NodeOptions()
                                .automatically_declare_parameters_from_overrides(true)
                                .use_intra_process_comms(true)
                                .append_parameter_override("use_sim_time", true));
  
  // Check if use_sim_time is correctly set
  bool use_sim_time = false;
  node->get_parameter("use_sim_time", use_sim_time);
  
  RCLCPP_INFO(node->get_logger(), "SimTimeJointStateNode initialized with use_sim_time=%s", 
             use_sim_time ? "true" : "false");
  
  // Initialize tf2 buffer and listener
  auto tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  
  // Create clients for data recorder services
  auto start_recording_client = node->create_client<std_srvs::srv::Trigger>("start_recording");
  auto stop_recording_client = node->create_client<std_srvs::srv::Trigger>("stop_recording");
  
  // Wait for data recorder service to be available
  while (!start_recording_client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for data recorder service.");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for data recorder services to be available...");
  }
  RCLCPP_INFO(node->get_logger(), "Data recorder services are available.");
  
  RCLCPP_INFO(node->get_logger(), "Waiting for MoveIt to initialize...");
  rclcpp::sleep_for(std::chrono::seconds(5));
  
  // Initialize MoveGroupInterface
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface = nullptr;
  bool moveit_ready = false;
  
  try {
    RCLCPP_INFO(node->get_logger(), "Initializing MoveGroupInterface for UR5...");
    // Initialize the move group for the UR5 arm
    move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      node, "ur5_arm");
    
    if (move_group_interface) { 
      RCLCPP_INFO(node->get_logger(), "MoveGroupInterface initialized successfully");
      moveit_ready = true;
    }
  }
  catch(const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize MoveGroupInterface: %s", e.what());
    moveit_ready = false;
  }
  
  // Flags to track planning status
  bool has_valid_joint_values = false;
  bool has_valid_target_pose = false;
  bool plan_created = false;
  bool execution_started = false;
  bool execution_completed = false;
  bool shutdown_requested = false;
  bool recording_started = false;  // Track if recording has started
  bool recording_stopped = false;  // Track if recording has stopped
  std::vector<double> current_joint_values;
  geometry_msgs::msg::Pose target_pose;
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool planning_in_progress = false;
  
  // To check when execution is done
  auto execution_start_time = std::chrono::steady_clock::now();
  double expected_execution_time = 0.0;
  
  // Create main loop - declare timer as a member variable
  rclcpp::TimerBase::SharedPtr main_timer;
  
  main_timer = node->create_wall_timer(
    std::chrono::seconds(1),
    [&]() {
      // Skip processing if shutdown has been requested
      if (shutdown_requested) {
        return;
      }
      
      // Get joint values directly from MoveGroupInterface if available
      if (moveit_ready) {
        try {
          if (!has_valid_joint_values) {
            current_joint_values = move_group_interface->getCurrentJointValues();
            
            if (current_joint_values.size() == 6) {  // UR5 has 6 joints
              RCLCPP_INFO(node->get_logger(), "Joint values obtained:");
              RCLCPP_INFO(node->get_logger(), "  shoulder_pan_joint: %.6f rad", current_joint_values[0]);
              RCLCPP_INFO(node->get_logger(), "  shoulder_lift_joint: %.6f rad", current_joint_values[1]);
              RCLCPP_INFO(node->get_logger(), "  elbow_joint: %.6f rad", current_joint_values[2]);
              RCLCPP_INFO(node->get_logger(), "  wrist_1_joint: %.6f rad", current_joint_values[3]);
              RCLCPP_INFO(node->get_logger(), "  wrist_2_joint: %.6f rad", current_joint_values[4]);
              RCLCPP_INFO(node->get_logger(), "  wrist_3_joint: %.6f rad", current_joint_values[5]);
              has_valid_joint_values = true;
              
              // Try getting a transform from TF if we have valid joint values
              if (!has_valid_target_pose) {
                target_pose = get_pose_from_tf(tf_buffer.get(), node, "world", "torso6_insertion_point");
                // target_pose.position.z += 0.10; // Account for needle length
                
                RCLCPP_INFO(node->get_logger(), "Target pose acquired at [%.3f, %.3f, %.3f]",
                           target_pose.position.x, target_pose.position.y, target_pose.position.z);
                has_valid_target_pose = true;
              }
            } else {
              RCLCPP_WARN(node->get_logger(), "Unexpected number of joint values: %zu", current_joint_values.size());
            }
          }
          
          // If we have both valid joint values and a valid target pose, compute IK and plan
          // Only execute this code once to avoid repeated planning
          if (has_valid_joint_values && has_valid_target_pose && !plan_created && !planning_in_progress) {
            planning_in_progress = true;
            RCLCPP_INFO(node->get_logger(), "Computing IK and planning motion to target pose...");
            
            // Wrap in a try-catch to capture any exceptions specifically from planning
            try {
              bool success = compute_ik_and_plan(node, *move_group_interface, current_joint_values, 
                                               target_pose, plan, "RRTstar", 5.0);
              if (success) {
                plan_created = true;
                
                // Calculate expected execution time from the trajectory
                expected_execution_time = plan.trajectory_.joint_trajectory.points.back().time_from_start.sec +
                                         plan.trajectory_.joint_trajectory.points.back().time_from_start.nanosec * 1e-9;
                
                RCLCPP_INFO(node->get_logger(), "Successfully created a motion plan! Trajectory has %zu points over %.2f seconds.",
                           plan.trajectory_.joint_trajectory.points.size(), expected_execution_time);
              } else {
                RCLCPP_ERROR(node->get_logger(), "Failed to create a motion plan.");
                // Reset planning_in_progress to allow another attempt
                planning_in_progress = false;
              }
            }
            catch(const std::exception& e) {
              RCLCPP_ERROR(node->get_logger(), "Exception during planning: %s", e.what());
              planning_in_progress = false;
            }
          }
          
          // Start recording before execution
          if (plan_created && !recording_started && !execution_started) {
            RCLCPP_INFO(node->get_logger(), "Starting data recording...");
            
            // Call the start_recording service
            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
            
            auto future = start_recording_client->async_send_request(request,
              [&node](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                auto response = future.get();
                if (response->success) {
                  RCLCPP_INFO(node->get_logger(), "Data recording started: %s", response->message.c_str());
                } else {
                  RCLCPP_ERROR(node->get_logger(), "Failed to start data recording: %s", response->message.c_str());
                }
              });
              
            recording_started = true;
            
            // Give a short delay to ensure recording has started before execution
            rclcpp::sleep_for(std::chrono::milliseconds(500));
          }
          
          // Execute the plan if we have one and haven't started execution yet
          if (plan_created && recording_started && !execution_started && !execution_completed) {
            RCLCPP_INFO(node->get_logger(), "Executing motion plan...");
            execution_started = true;
            execution_start_time = std::chrono::steady_clock::now();
            
            try {
              moveit::core::MoveItErrorCode execution_result = move_group_interface->execute(plan);
              if (execution_result == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(node->get_logger(), "Plan execution started successfully. Expected duration: %.2f seconds", 
                           expected_execution_time);
              } else {
                RCLCPP_ERROR(node->get_logger(), "Plan execution failed with error code: %d", execution_result.val);
                // Allow retry if execution fails
                execution_started = false;
              }
            }
            catch(const std::exception& e) {
              RCLCPP_ERROR(node->get_logger(), "Exception during execution: %s", e.what());
              execution_started = false;
            }
          }
          
          // Check if execution has completed and stop recording
          if (execution_started && !execution_completed) {
            auto now = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed = now - execution_start_time;
            
            // Add a small margin to the expected execution time
            if (elapsed.count() > expected_execution_time + 1.0) {
              execution_completed = true;
              RCLCPP_INFO(node->get_logger(), "Motion execution completed after %.2f seconds", elapsed.count());
              
              // Stop recording if it hasn't been stopped yet
              if (!recording_stopped) {
                RCLCPP_INFO(node->get_logger(), "Stopping data recording...");
                
                // Call the stop_recording service
                auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
                
                auto future = stop_recording_client->async_send_request(request,
                  [&node](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                    auto response = future.get();
                    if (response->success) {
                      RCLCPP_INFO(node->get_logger(), "Data recording stopped: %s", response->message.c_str());
                    } else {
                      RCLCPP_ERROR(node->get_logger(), "Failed to stop data recording: %s", response->message.c_str());
                    }
                  });
                  
                recording_stopped = true;
                
                // Give a short delay before shutdown to ensure recording has been saved
                rclcpp::sleep_for(std::chrono::seconds(1));
                
                RCLCPP_INFO(node->get_logger(), "All tasks completed successfully. Shutting down...");
                
                // Set flag but don't trigger shutdown from within the callback
                shutdown_requested = true;
                
                // Start a safe shutdown thread
                std::thread([node]() {
                  // Give a little time for logging and stabilization
                  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                  rclcpp::shutdown();
                }).detach();
              }
            }
          }
        }
        catch(const std::exception& e) {
          RCLCPP_ERROR(node->get_logger(), "Error getting joint values from MoveGroupInterface: %s", e.what());
        }
      } else {
        // Only show this warning occasionally to avoid log spam
        RCLCPP_WARN_THROTTLE(node->get_logger(), 
                           *node->get_clock(),
                           5000, // Only print warning every 5 seconds
                           "MoveIt interface not ready yet");
      }
    });
  
  // Spin the node
  rclcpp::spin(node);
  
  RCLCPP_INFO(node->get_logger(), "Node has been shut down");
  return 0;
}