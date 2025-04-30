// imports
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <sensor_msgs/msg/joint_state.hpp>
#include <mutex>
#include <condition_variable>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <moveit_msgs/srv/get_position_ik.hpp>

// Global variables for joint state recording
std::mutex joint_state_mutex;
std::condition_variable joint_state_cv;
bool recording_complete = false;
std::vector<sensor_msgs::msg::JointState> recorded_states;

// Get current timestamp for filename
std::string getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S");
    return ss.str();
}

// Function to save trajectory data to CSV
void saveTrajectoryToCSV(const moveit_msgs::msg::RobotTrajectory& trajectory,
                        const std::vector<sensor_msgs::msg::JointState>& actual_states,
                        const std::string& planner_name,
                        const rclcpp::Logger& logger) {
    std::string timestamp = getCurrentTimestamp();
    std::string filename = "trajectory_" + planner_name + "_" + timestamp + ".csv";
    
    std::ofstream csv_file(filename);
    if (!csv_file.is_open()) {
        RCLCPP_ERROR(logger, "Failed to open file for writing: %s", filename.c_str());
        return;
    }

    // Write header
    csv_file << "time,planned_elbow,planned_shoulder_lift,planned_shoulder_pan,planned_wrist_1,planned_wrist_2,planned_wrist_3,";
    csv_file << "actual_elbow,actual_shoulder_lift,actual_shoulder_pan,actual_wrist_1,actual_wrist_2,actual_wrist_3\n";

    // Write trajectory points
    for (size_t i = 0; i < trajectory.joint_trajectory.points.size(); ++i) {
        const auto& point = trajectory.joint_trajectory.points[i];
        double time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
        csv_file << time << ",";
        
        // Write planned joint positions
        for (size_t j = 0; j < point.positions.size(); ++j) {
            csv_file << point.positions[j];
            if (j < point.positions.size() - 1) csv_file << ",";
        }
        
        // Write actual joint positions if available
        if (i < actual_states.size()) {
            csv_file << ",";
            const auto& state = actual_states[i];
            // Find the indices of our joints in the state message
            std::vector<size_t> joint_indices;
            for (const auto& joint_name : {"elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint",
                                         "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"}) {
                auto it = std::find(state.name.begin(), state.name.end(), joint_name);
                if (it != state.name.end()) {
                    joint_indices.push_back(std::distance(state.name.begin(), it));
                }
            }
            // Write actual joint positions
            for (size_t j = 0; j < joint_indices.size(); ++j) {
                csv_file << state.position[joint_indices[j]];
                if (j < joint_indices.size() - 1) csv_file << ",";
            }
        }
        csv_file << "\n";
    }

    RCLCPP_INFO(logger, "Saved trajectory data to %s", filename.c_str());
}

// Utility function to convert RPY angles to quaternion
geometry_msgs::msg::Quaternion rpy_to_quaternion(double roll, double pitch, double yaw) {
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  geometry_msgs::msg::Quaternion quat;
  quat.x = q.x();
  quat.y = q.y();
  quat.z = q.z();
  quat.w = q.w();
  return quat;
}

bool plan_and_execute(moveit::planning_interface::MoveGroupInterface& move_group,
                      const std::map<std::string, double>& target,
                      const rclcpp::Logger& logger,
                      const std::string& planning_algorithm = "RRTstar",
                      const float& planning_time = 20.0)
{
  // Set the planner
  move_group.setPlannerId(planning_algorithm);
  
  // Configure RRT* specific parameters
  move_group.setPlanningTime(planning_time);
  move_group.setNumPlanningAttempts(5);
  move_group.setMaxVelocityScalingFactor(0.5);
  move_group.setMaxAccelerationScalingFactor(0.5);
  
  // Set goal tolerances
  move_group.setGoalOrientationTolerance(0.1);  // [rad]
  move_group.setGoalPositionTolerance(0.01);    // [m]
  move_group.setGoalJointTolerance(0.01);       // [rad]

  move_group.setJointValueTarget(target);
  
  // Plan and execute
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(move_group.plan(plan));
  
  if (success)
  {
    RCLCPP_INFO(logger, "Planning succeeded! Executing plan...");
    
    // Clear previous recorded states and start new recording
    {
      std::lock_guard<std::mutex> lock(joint_state_mutex);
      recorded_states.clear();
      recording_complete = false;
    }
    
    // Execute the plan
    moveit::core::MoveItErrorCode execution_result = move_group.execute(plan);
    if (execution_result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(logger, "Execution completed successfully");
      
      // Wait a short time for the last joint states to be recorded
      rclcpp::sleep_for(std::chrono::milliseconds(500));
      
      // Stop recording
      {
        std::lock_guard<std::mutex> lock(joint_state_mutex);
        recording_complete = true;
      }
      joint_state_cv.notify_all();
      
      // Save both planned and actual trajectories
      saveTrajectoryToCSV(plan.trajectory_, recorded_states, planning_algorithm, logger);
    } else {
      RCLCPP_ERROR(logger, "Execution failed with error code: %d", execution_result.val);
      return false;
    }
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
  }
  return success;
}

// Function to compute IK and then plan and execute
bool compute_ik_and_execute(std::shared_ptr<rclcpp::Node> node,
                           moveit::planning_interface::MoveGroupInterface& move_group,
                           const geometry_msgs::msg::Pose& target_pose,
                           const rclcpp::Logger& logger,
                           const std::map<std::string, double>& ready_pose,
                           const std::string& planning_algorithm = "RRTstar",
                           const float& planning_time = 20.0)
{
  RCLCPP_INFO(logger, "Computing IK for target pose");
  
  // Create the IK service client
  auto client = node->create_client<moveit_msgs::srv::GetPositionIK>("/compute_ik");
  
  // Wait for service to be available
  if (!client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(logger, "Service /compute_ik not available after waiting");
    return false;
  }
  
  // Create the request
  auto request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
  
  // Set up the request with the current joint values for the seed state
  request->ik_request.group_name = "ur5_arm";
  
  // Define joint names in the order MoveIt expects them
  std::vector<std::string> joint_names = {
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
  };
  
  // Try to get current joint values directly using the MoveGroupInterface
  std::vector<double> current_joint_values = move_group.getCurrentJointValues();
  
  if (current_joint_values.size() == 6) {
    RCLCPP_INFO(logger, "Using current joint values as seed state");
    
    // Fill in the request with the current joint values
    request->ik_request.robot_state.joint_state.name = joint_names;
    request->ik_request.robot_state.joint_state.position = current_joint_values;
    
    // Log the seed values
    RCLCPP_INFO(logger, "Seed state - Shoulder pan: %.4f, Shoulder lift: %.4f, Elbow: %.4f, Wrist1: %.4f, Wrist2: %.4f, Wrist3: %.4f", 
               current_joint_values[0], current_joint_values[1], current_joint_values[2], 
               current_joint_values[3], current_joint_values[4], current_joint_values[5]);
  } else {
    RCLCPP_ERROR(logger, "Failed to get current joint values (got %zu values, expected 6)", current_joint_values.size());
    RCLCPP_INFO(logger, "Using ready pose values as seed state instead");
    
    // Use the ready_pose values as seed
    request->ik_request.robot_state.joint_state.name = joint_names;
    request->ik_request.robot_state.joint_state.position.resize(6);
    
    request->ik_request.robot_state.joint_state.position[0] = ready_pose.at("shoulder_pan_joint");
    request->ik_request.robot_state.joint_state.position[1] = ready_pose.at("shoulder_lift_joint");
    request->ik_request.robot_state.joint_state.position[2] = ready_pose.at("elbow_joint");
    request->ik_request.robot_state.joint_state.position[3] = ready_pose.at("wrist_1_joint");
    request->ik_request.robot_state.joint_state.position[4] = ready_pose.at("wrist_2_joint");
    request->ik_request.robot_state.joint_state.position[5] = ready_pose.at("wrist_3_joint");
    
    RCLCPP_INFO(logger, "Seed state from ready pose - Shoulder pan: %.4f, Shoulder lift: %.4f, Elbow: %.4f, Wrist1: %.4f, Wrist2: %.4f, Wrist3: %.4f", 
               request->ik_request.robot_state.joint_state.position[0], 
               request->ik_request.robot_state.joint_state.position[1],
               request->ik_request.robot_state.joint_state.position[2], 
               request->ik_request.robot_state.joint_state.position[3],
               request->ik_request.robot_state.joint_state.position[4], 
               request->ik_request.robot_state.joint_state.position[5]);
  }
  
  // Set the pose target
  request->ik_request.pose_stamped.header.frame_id = "world";
  request->ik_request.pose_stamped.pose = target_pose;
  
  // Set timeout
  request->ik_request.timeout.sec = 1;
  request->ik_request.timeout.nanosec = 0;
  
  // Call IK service
  auto result_future = client->async_send_request(request);
  
  // Wait for the result
  if (rclcpp::spin_until_future_complete(node, result_future) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Failed to call service /compute_ik");
    return false;
  }
  
  auto result = result_future.get();
  
  // Check if IK succeeded
  if (result->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(logger, "IK computation failed with error code: %d", result->error_code.val);
    return false;
  }
  
  // Extract joint values
  std::map<std::string, double> target_joint_values;
  for (size_t i = 0; i < result->solution.joint_state.name.size(); ++i) {
    target_joint_values[result->solution.joint_state.name[i]] = result->solution.joint_state.position[i];
    RCLCPP_INFO(logger, "Joint %s: %.4f", result->solution.joint_state.name[i].c_str(), 
               result->solution.joint_state.position[i]);
  }
  
  RCLCPP_INFO(logger, "IK solution found, planning motion to target joint configuration");
  
  // Plan and execute with the IK solution
  return plan_and_execute(move_group, target_joint_values, logger, planning_algorithm, planning_time);
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  
  auto const logger = rclcpp::get_logger("hello_moveit");
  auto error = rcutils_logging_set_logger_level(logger.get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
  if (error != RCUTILS_RET_OK)
  {
    RCLCPP_ERROR(logger, "Failed to set logging level to DEBUG");
  }

  // Create TF2 buffer and listener
  std::shared_ptr<tf2_ros::Buffer> tf_buffer = 
    std::make_shared<tf2_ros::Buffer>(node->get_clock());
  std::shared_ptr<tf2_ros::TransformListener> tf_listener = 
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  // Joint state subscriber for trajectory recording
  auto joint_state_sub = node->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [](const sensor_msgs::msg::JointState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(joint_state_mutex);
          if (!recording_complete) {
              recorded_states.push_back(*msg);
          }
      });

  // Allow time for ROS connections to establish
  RCLCPP_INFO(logger, "Initializing...");
  rclcpp::sleep_for(std::chrono::seconds(1));

  // Initialize MoveGroupInterface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur5_arm");
  RCLCPP_INFO(logger, "MoveGroup interface initialized");

  // Define poses
  const std::map<std::string, double> home_pose = {
      {"elbow_joint", 0.0},
      {"shoulder_lift_joint", -1.57},
      {"shoulder_pan_joint", 0.0},
      {"wrist_1_joint", -1.57},
      {"wrist_2_joint", 0.0},
      {"wrist_3_joint", 0.0}
  };

  const std::map<std::string, double> ready_pose = {
      {"elbow_joint", 1.267},
      {"shoulder_lift_joint", -2.1869},
      {"shoulder_pan_joint", 0.0},
      {"wrist_1_joint", -0.6596},
      {"wrist_2_joint", -1.57},
      {"wrist_3_joint", 0.0}
  };

  const std::map<std::string, double> needle_insertion_pose = {
      {"elbow_joint", 0.872665},
      {"shoulder_lift_joint", -0.663225},
      {"shoulder_pan_joint", 0.0698132},
      {"wrist_1_joint", -1.79769},
      {"wrist_2_joint", -1.5708},
      {"wrist_3_joint", 0.0698132}
  };

  RCLCPP_INFO(logger, "Starting motion planning sequence with RRT*...");

  // Wait for transforms to be available
  RCLCPP_INFO(logger, "Waiting for transforms to become available...");
  rclcpp::sleep_for(std::chrono::seconds(2));
  
  // Get the arm insertion point transform using TF2
  geometry_msgs::msg::Pose target_pose;
  
  try {
    // Look up the transform from world to arm_insertion_point
    // geometry_msgs::msg::TransformStamped transform_stamped;
    // transform_stamped = tf_buffer->lookupTransform("world", "arm_insertion_point", 
    //                                               tf2::TimePointZero);
    
    // Convert the transform to a pose
    // target_pose.position.x = transform_stamped.transform.translation.x;
    // target_pose.position.y = transform_stamped.transform.translation.y;
    // target_pose.position.z = transform_stamped.transform.translation.z;
    
    // // Set a suitable orientation for needle insertion
    // target_pose.orientation = rpy_to_quaternion(-3.14, 0.0, -1.57);

    target_pose.position.x = 0.082;
    target_pose.position.y = 0.109;
    target_pose.position.z = 0.670;
    
    // Set a suitable orientation for needle insertion
    target_pose.orientation.x = -0.702;
    target_pose.orientation.y = 0.712;
    target_pose.orientation.z = -0.005;
    target_pose.orientation.w = 0.011;
    
    RCLCPP_INFO(logger, "Planning to arm insertion point at: [%f, %f, %f]", 
               target_pose.position.x, target_pose.position.y, target_pose.position.z);
    
    // First, move to the ready pose to ensure we start from a known configuration
    // RCLCPP_INFO(logger, "Moving to ready pose first...");
    // bool ready_success = plan_and_execute(move_group_interface, ready_pose, logger, "RRTstar");
    // if (!ready_success) {
    //   RCLCPP_ERROR(logger, "Failed to move to ready pose");
    //   return 1;
    // }
    
    // Add delay to ensure robot controller has fully updated its state
    // RCLCPP_INFO(logger, "Waiting for robot to stabilize and update state...");
    // rclcpp::sleep_for(std::chrono::seconds(2));
    
    // Then use IK to plan to the target pose
    RCLCPP_INFO(logger, "Now planning to target pose...");
    bool target_success = compute_ik_and_execute(node, move_group_interface, target_pose, logger, ready_pose, "RRTstar");
    
    if (!target_success) {
      RCLCPP_ERROR(logger, "Failed to execute planning to arm insertion point");
    }
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(logger, "Could not transform from 'world' to 'arm_insertion_point': %s", ex.what());
  }

  rclcpp::shutdown();
  return 0;
}
