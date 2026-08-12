#!/usr/bin/env python3
"""Experiment 6: contrast deterministic Pilz PTP and LIN motions."""

import rclpy
from geometry_msgs.msg import PoseStamped

from ur5e_motion_planning_lab.moveit_action_client import (
    BASE_LINK,
    EE_LINK,
    MoveItActionClient,
    pose_goal_constraints,
)

PILZ_PIPELINE = "pilz_industrial_motion_planner"


def pose(x, y, z, qx=0.0, qy=1.0, qz=0.0, qw=0.0, frame=BASE_LINK):
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
    moveit = MoveItActionClient("ex6_pilz_linear")
    try:
        if not moveit.wait_for_servers():
            return

        start = pose(0.40, 0.20, 0.40)
        end = pose(0.40, -0.20, 0.40)

        moveit.logger.info("PTP: joint-space industrial motion to the start pose.")
        if not moveit.plan_and_execute(
            pose_goal_constraints(start, EE_LINK),
            pipeline_id=PILZ_PIPELINE,
            planner_id="PTP",
        ):
            return

        moveit.logger.info("LIN: Cartesian straight-line industrial motion.")
        if not moveit.plan_and_execute(
            pose_goal_constraints(end, EE_LINK),
            pipeline_id=PILZ_PIPELINE,
            planner_id="LIN",
        ):
            moveit.logger.warn(
                "LIN failed. Confirm Pilz is loaded and has Cartesian velocity/acceleration limits."
            )
    finally:
        moveit.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
