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

// Global variables for joint state recording
std::mutex joint_state_mutex;
std::condition_variable joint_state_cv;
bool recording_complete = false;
std::vector<sensor_msgs::msg::JointState> recorded_states;

// get current timestamp for filename
std::string getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S");
    return ss.str();
}

// function to save trajectory data to CSV
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

    // write header
    csv_file << "time,planned_elbow,planned_shoulder_lift,planned_shoulder_pan,planned_wrist_1,planned_wrist_2,planned_wrist_3,";
    csv_file << "actual_elbow,actual_shoulder_lift,actual_shoulder_pan,actual_wrist_1,actual_wrist_2,actual_wrist_3\n";

    // write trajectory points
    for (size_t i = 0; i < trajectory.joint_trajectory.points.size(); ++i) {
        const auto& point = trajectory.joint_trajectory.points[i];
        double time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
        csv_file << time << ",";
        
        // write planned joint positions
        for (size_t j = 0; j < point.positions.size(); ++j) {
            csv_file << point.positions[j];
            if (j < point.positions.size() - 1) csv_file << ",";
        }
        
        // write actual joint positions if available
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

    RCLCPP_INFO(logger, "saved trajectory data to %s", filename.c_str());
}

bool plan_and_execute(moveit::planning_interface::MoveGroupInterface& move_group,
                      const std::map<std::string, double>& target,
                      const rclcpp::Logger& logger,
                      const std::string& planning_algorithm = "RRTstar",
                      const float& planning_time = 20.0)
{
  // set the planner
  move_group.setPlannerId(planning_algorithm);
  
  // configure RRT* specific parameters
  move_group.setPlanningTime(planning_time);
  move_group.setNumPlanningAttempts(5);
  move_group.setMaxVelocityScalingFactor(0.5);
  move_group.setMaxAccelerationScalingFactor(0.5);
  
  // set goal tolerances
  move_group.setGoalOrientationTolerance(0.1);  // [rad]
  move_group.setGoalPositionTolerance(0.01);    // [m]
  move_group.setGoalJointTolerance(0.01);       // [rad]

  move_group.setJointValueTarget(target);
  
  // Plan and execute
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(move_group.plan(plan));
  
  if (success)
  {
    RCLCPP_INFO(logger, "planning succeeded! executing plan...");
    
    // Clear previous recorded states and start new recording
    {
      std::lock_guard<std::mutex> lock(joint_state_mutex);
      recorded_states.clear();
      recording_complete = false;
    }
    
    // Execute the plan
    moveit::core::MoveItErrorCode execution_result = move_group.execute(plan);
    if (execution_result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(logger, "execution completed successfully");
      
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
      RCLCPP_ERROR(logger, "execution failed with error code: %d", execution_result.val);
      return false;
    }
  }
  else
  {
    RCLCPP_ERROR(logger, "planning failed!");
  }
  return success;
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

  // Create joint state subscriber
  auto joint_state_sub = node->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [](const sensor_msgs::msg::JointState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(joint_state_mutex);
          if (!recording_complete) {
              recorded_states.push_back(*msg);
          }
      });

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur5_arm");

  // define poses
  const std::map<std::string, double> home_pose = {
      {"elbow_joint", 0.0},
      {"shoulder_lift_joint", -1.57},
      {"shoulder_pan_joint", 0.0},
      {"wrist_1_joint", -1.57},
      {"wrist_2_joint", 0.0},
      {"wrist_3_joint", 0.0}
  };

  /*
      <group_state name="ready" group="ur5_arm">
        <joint name="elbow_joint" value="1.267"/>
        <joint name="shoulder_lift_joint" value="-2.1869"/>
        <joint name="shoulder_pan_joint" value="0"/>
        <joint name="wrist_1_joint" value="-0.6596"/>
        <joint name="wrist_2_joint" value="-1.57"/>
        <joint name="wrist_3_joint" value="0"/>
    </group_state>
  */

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

  RCLCPP_INFO(logger, "starting motion planning sequence with RRT*...");
  
  // Execute ready pose
  RCLCPP_INFO(logger, "planning and executing ready pose...");
  bool ready_success = plan_and_execute(move_group_interface, ready_pose, logger, "RRTstar");
  
  if (!ready_success) {
    RCLCPP_ERROR(logger, "Failed to execute ready pose, aborting sequence");
    rclcpp::shutdown();
    return 1;
  }
  
  RCLCPP_INFO(logger, "waiting for 3 seconds before next pose...");
  rclcpp::sleep_for(std::chrono::seconds(3));
  
  // Execute needle insertion pose
  RCLCPP_INFO(logger, "planning and executing needle insertion pose...");
  bool needle_success = plan_and_execute(move_group_interface, needle_insertion_pose, logger, "RRTstar");
  
  if (!needle_success) {
    RCLCPP_ERROR(logger, "Failed to execute needle insertion pose");
  }

  rclcpp::shutdown();
  return 0;
}
