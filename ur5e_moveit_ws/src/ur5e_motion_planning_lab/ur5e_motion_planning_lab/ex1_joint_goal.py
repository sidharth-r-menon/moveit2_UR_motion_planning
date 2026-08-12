#!/usr/bin/env python3
"""
Experiment 1: explicit joint-space target via MoveIt's /move_action server.

Prerequisite:
  ros2 launch ur5e_motion_planning_lab ur5e_fake_moveit.launch.py
"""

import rclpy
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes


PLANNING_GROUP = "ur_manipulator"

TARGET_JOINT_POSITIONS = {
    "shoulder_pan_joint": 0.0,
    "shoulder_lift_joint": -1.57,
    "elbow_joint": 1.57,
    "wrist_1_joint": -1.57,
    "wrist_2_joint": -1.57,
    "wrist_3_joint": 0.0,
}


class JointGoalExperiment:
    def __init__(self):
        self.node = rclpy.create_node("ex1_joint_goal")
        self.client = ActionClient(self.node, MoveGroup, "/move_action")

    def move_to_joint_target(self, joint_positions: dict[str, float]) -> bool:
        goal = MoveGroup.Goal()

        # Motion-plan request
        goal.request.group_name = PLANNING_GROUP
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.25
        goal.request.max_acceleration_scaling_factor = 0.25
        goal.request.start_state.is_diff = True  # use current /joint_states

        constraints = Constraints()
        for joint_name, position in joint_positions.items():
            constraint = JointConstraint()
            constraint.joint_name = joint_name
            constraint.position = position
            constraint.tolerance_above = 0.001
            constraint.tolerance_below = 0.001
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)

        goal.request.goal_constraints.append(constraints)

        # False means: ask move_group to plan AND execute.
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        self.node.get_logger().info(
            f"Sending joint goal: {joint_positions}"
        )

        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_future)
        goal_handle = send_future.result()

        if not goal_handle or not goal_handle.accepted:
            self.node.get_logger().error("MoveGroup rejected the goal.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result().result

        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.node.get_logger().info("Planning and execution succeeded.")
            return True

        self.node.get_logger().error(
            f"MoveIt failed with error code: {result.error_code.val}"
        )
        return False

    def shutdown(self):
        self.node.destroy_node()


def main():
    rclpy.init()
    experiment = JointGoalExperiment()

    try:
        if not experiment.client.wait_for_server(timeout_sec=10.0):
            experiment.node.get_logger().error(
                "/move_action is unavailable. Start the fake MoveIt launch first."
            )
            return

        experiment.move_to_joint_target(TARGET_JOINT_POSITIONS)

    finally:
        experiment.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()