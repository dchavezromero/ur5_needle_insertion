#!/usr/bin/env python3

""" playground for trying to get the UR5 to move using MoveIt2"""

import rclpy
from rclpy.node import Node

# below is broken rn
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface


class UR5MotionPlanner(Node):
    def __init__(self):
        super().__init__("ur5_motion_planner")

        # initialize MoveIt components
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group = MoveGroupCommander("ur_manipulator")

        # Set planner parameters
        self.group.set_planner_id("RRTConnectkConfigDefault")
        self.group.set_num_planning_attempts(5)
        self.group.set_goal_tolerance(0.01)

        # move to a predefined pose TODO: make this intelligent
        self.move_to_pose()

    def move_to_pose(self):
        target_pose = self.group.get_current_pose().pose
        target_pose.position.x += 0.1  # move 0.1m forward?
        target_pose.position.z += 0.05

        self.group.set_pose_target(target_pose)
        success, plan, _, _ = self.group.plan()

        if success:
            self.get_logger().info("executing planned motion...")
            self.group.execute(plan, wait=True)
        else:
            self.get_logger().warn("motion planning failed!")


def main(args=None):

    # initialize the ROS2 node
    rclpy.init(args=args)
    node = UR5MotionPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
