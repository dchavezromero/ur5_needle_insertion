#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>


#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

bool plan_and_execute(moveit::planning_interface::MoveGroupInterface& move_group,
                      const std::map<std::string, double>& target,
                      const rclcpp::Logger& logger,
                      const std::string& planning_algorithm = "RRTConnect")
{
  move_group.setPlannerId(planning_algorithm);

  std::ostringstream oss;
  oss << "Joint value target:\n";
  for (const auto& [joint, value] : target)
  {
    RCLCPP_INFO(logger, "  %s: %f", joint.c_str(), value);
    RCLCPP_ERROR(logger, " %s", joint.c_str());
    oss << "  " << joint << ": " << value << "\n";
  }
//  RCLCPP_INFO(logger, "%s"); //, oss.str().c_str());

  move_group.setJointValueTarget(target);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(move_group.plan(plan));
  if (success)
  {
    RCLCPP_INFO(logger, "TEST Planning succeeded! Executing plan...");
    move_group.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "FOO Planning failed!");
  }
  return success;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto const logger = rclcpp::get_logger("hello_moveit");

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur5_arm");

  // Define poses
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

  // Move through the poses
//  RCLCPP_INFO(logger, "planning and executing home pose...");
//  plan_and_execute(move_group_interface, home_pose, logger, "RRTConnect");
//  rclcpp::sleep_for(std::chrono::seconds(3));
    RCLCPP_INFO(logger, "planning and executing ready pose...");
  plan_and_execute(move_group_interface, ready_pose, logger, "RRTConnect");
  rclcpp::sleep_for(std::chrono::seconds(3));
  RCLCPP_INFO(logger, "planning and executing needle insertion pose...");
  plan_and_execute(move_group_interface, needle_insertion_pose, logger, "RRTConnect");

  rclcpp::shutdown();
  return 0;
}
