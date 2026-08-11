#!/usr/bin/env python3
"""Experiment 5: planning-only comparison of standard OMPL planners."""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped

from ur5e_motion_planning_lab.moveit_action_client import (
    BASE_LINK,
    EE_LINK,
    MoveItActionClient,
    pose_goal_constraints,
)

# These are standard names in MoveIt's ompl_planning.yaml. If your distro's
# file omits one, delete that entry rather than treating it as a failed plan.
PLANNERS_TO_COMPARE = [
    "RRTConnectkConfigDefault",
    "RRTstarkConfigDefault",
    "PRMstarkConfigDefault",
    "ESTkConfigDefault",
    "KPIECEkConfigDefault",
]


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


def joint_path_length(trajectory) -> float:
    points = trajectory.joint_trajectory.points
    return sum(
        math.sqrt(sum((b - a) ** 2 for a, b in zip(previous.positions, current.positions)))
        for previous, current in zip(points, points[1:])
    )


def main():
    rclpy.init()
    moveit = MoveItActionClient("ex5_planner_comparison")
    try:
        if not moveit.wait_for_servers():
            return

        goal = pose(0.35, 0.35, 0.25)
        goal_constraints = pose_goal_constraints(goal, EE_LINK)
        results = []

        # Each request uses the same live start state because this exercise
        # never executes any trajectory.
        for planner_id in PLANNERS_TO_COMPARE:
            moveit.logger.info(f"Planning with OMPL planner: {planner_id}")
            started = time.monotonic()
            result = moveit.plan(
                goal_constraints,
                planner_id=planner_id,
                planning_time=5.0,
            )
            elapsed = time.monotonic() - started

            if result is None:
                results.append((planner_id, False, elapsed, None))
            else:
                results.append(
                    (planner_id, True, elapsed, joint_path_length(result.planned_trajectory))
                )

        moveit.logger.info("=== Planning-only comparison ===")
        moveit.logger.info(f"{'planner':<25} {'success':<8} {'time(s)':<10} {'joint length':<12}")
        for planner, success, elapsed, length in results:
            length_text = "-" if length is None else f"{length:.3f}"
            moveit.logger.info(
                f"{planner:<25} {str(success):<8} {elapsed:<10.3f} {length_text:<12}"
            )
    finally:
        moveit.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
