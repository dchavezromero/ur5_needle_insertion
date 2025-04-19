// imports
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <filesystem>

// get current timestamp for filename
std::string getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S");
    return ss.str();
}

// function to save trajectory data to CSV (if we want to compare different planners)
void saveTrajectoryToCSV(const moveit_msgs::msg::RobotTrajectory& trajectory,
                        const std::string& planner_name,
                        const rclcpp::Logger& logger) {
    std::string timestamp = getCurrentTimestamp();
    std::string filename = "trajectory_" + planner_name + "_" + timestamp + ".csv";
    
    std::ofstream csv_file(filename);
    if (!csv_file.is_open()) {
        RCLCPP_ERROR(logger, "Failed to open file for writing: %s", filename.c_str());
        return;
    }

    // write header (hard-coded for now)
    csv_file << "time,elbow_joint,shoulder_lift_joint,shoulder_pan_joint,wrist_1_joint,wrist_2_joint,wrist_3_joint\n";

    // write trajectory points
    for (size_t i = 0; i < trajectory.joint_trajectory.points.size(); ++i) {
        const auto& point = trajectory.joint_trajectory.points[i];
        csv_file << point.time_from_start.sec + point.time_from_start.nanosec * 1e-9 << ",";
        
        // write joint positions
        for (size_t j = 0; j < point.positions.size(); ++j) {
            csv_file << point.positions[j];
            if (j < point.positions.size() - 1) csv_file << ",";
        }
        csv_file << "\n";
    }

    RCLCPP_INFO(logger, "saved trajectory data to %s", filename.c_str());
}

bool plan_and_execute(moveit::planning_interface::MoveGroupInterface& move_group,
                      const std::map<std::string, double>& target,
                      const rclcpp::Logger& logger,
                      const std::string& planning_algorithm = "RRTstar",
                      const float& planning_time = 20.0)  // Increased planning time for RRT*
{
  // set the planner
  move_group.setPlannerId(planning_algorithm);
  
  // configure RRT* specific parameters
  // TODO(BZ): make this configurable to different planning algorithms, just hard-coding for now
  move_group.setPlanningTime(planning_time);
  move_group.setNumPlanningAttempts(5);  // Try multiple times to find a better path
  move_group.setMaxVelocityScalingFactor(0.5);  // Slower execution for more precise movements
  move_group.setMaxAccelerationScalingFactor(0.5);
  
  // set goal tolerances
  move_group.setGoalOrientationTolerance(0.1);  // [rad]
  move_group.setGoalPositionTolerance(0.01);    // [m]
  move_group.setGoalJointTolerance(0.01);       // [rad]

  std::ostringstream oss;
  oss << "Joint value target:\n";
  for (const auto& [joint, value] : target)
  {
    RCLCPP_INFO_STREAM(logger, "  " << joint << ": " << value);
    oss << "  " << joint << ": " << value << "\n";
  }
  RCLCPP_INFO_STREAM(logger, oss.str());

  move_group.setJointValueTarget(target);
  
  // Plan and execute
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(move_group.plan(plan));
  
  if (success)
  {
    RCLCPP_INFO(logger, "planning succeeded! executing plan...");
    // get the path length for analysis
    double path_length = 0.0;
    for (size_t i = 1; i < plan.trajectory_.joint_trajectory.points.size(); ++i)
    {
      const auto& prev = plan.trajectory_.joint_trajectory.points[i-1];
      const auto& curr = plan.trajectory_.joint_trajectory.points[i];
      for (size_t j = 0; j < prev.positions.size(); ++j)
      {
        path_length += std::abs(curr.positions[j] - prev.positions[j]);
      }
    }
    RCLCPP_INFO(logger, "path length: %f", path_length);
    
    move_group.execute(plan);
    saveTrajectoryToCSV(plan.trajectory_, planning_algorithm, logger);
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
  
  // set the logging level to DEBUG to see all messages
  auto const logger = rclcpp::get_logger("hello_moveit");
  auto error = rcutils_logging_set_logger_level(logger.get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
  if (error != RCUTILS_RET_OK)
  {
    RCLCPP_ERROR(logger, "Failed to set logging level to DEBUG");
  }

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
  RCLCPP_INFO(logger, "planning and executing ready pose...");
  plan_and_execute(move_group_interface, ready_pose, logger, "RRTstar");
  rclcpp::sleep_for(std::chrono::seconds(3));
  RCLCPP_INFO(logger, "planning and executing needle insertion pose...");
  plan_and_execute(move_group_interface, needle_insertion_pose, logger, "RRTstar");

  rclcpp::shutdown();
  return 0;
}
