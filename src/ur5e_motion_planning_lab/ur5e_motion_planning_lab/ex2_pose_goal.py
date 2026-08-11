#!/usr/bin/env python3
"""Experiment 2: pose-goal planning through MoveIt's /move_action server."""

import rclpy
from geometry_msgs.msg import PoseStamped

from ur5e_motion_planning_lab.moveit_action_client import (
    BASE_LINK,
    EE_LINK,
    MoveItActionClient,
    pose_goal_constraints,
)


def make_pose(x, y, z, qx=0.0, qy=1.0, qz=0.0, qw=0.0, frame=BASE_LINK):
    target = PoseStamped()
    target.header.frame_id = frame
    target.pose.position.x = x
    target.pose.position.y = y
    target.pose.position.z = z
    target.pose.orientation.x = qx
    target.pose.orientation.y = qy
    target.pose.orientation.z = qz
    target.pose.orientation.w = qw
    return target


def main():
    rclpy.init()
    moveit = MoveItActionClient("ex2_pose_goal")
    try:
        if not moveit.wait_for_servers():
            return

        # Tool0 points down. This is a typical top-down pick/place pose.
        target = make_pose(0.40, 0.10, 0.40)
        moveit.logger.info(
            f"Pose goal in {target.header.frame_id}: "
            f"({target.pose.position.x:.2f}, {target.pose.position.y:.2f}, "
            f"{target.pose.position.z:.2f}) for {EE_LINK}"
        )
        if moveit.plan_and_execute(pose_goal_constraints(target, EE_LINK)):
            moveit.logger.info("Pose-goal motion succeeded.")
        else:
            moveit.logger.warn("Pose-goal motion failed: inspect IK and collision state.")
    finally:
        moveit.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
