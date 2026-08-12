#!/usr/bin/env python3
"""Experiment 3: collision-checked Cartesian interpolation through waypoints."""

import rclpy
from geometry_msgs.msg import PoseStamped

from ur5e_motion_planning_lab.moveit_action_client import (
    BASE_LINK,
    EE_LINK,
    MoveItActionClient,
    pose_goal_constraints,
)

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
    moveit = MoveItActionClient("ex3_cartesian_path")
    try:
        if not moveit.wait_for_servers():
            return

        # ``/compute_cartesian_path`` interpolates the TCP through all of
        # these points in a single Cartesian trajectory. This is different
        # from Pilz LIN: Pilz is an optional industrial pipeline, whereas
        # this service is part of normal MoveIt and is available now.
        waypoints = [
            pose(0.40, 0.10, 0.40),
            pose(0.40, 0.10, 0.25),  # approach
            pose(0.40, -0.10, 0.25),  # transfer at depth
            pose(0.40, -0.10, 0.40),  # retreat
        ]
        moveit.logger.info(f"Computing Cartesian path through {len(waypoints)} waypoints.")
        trajectory = moveit.compute_cartesian_path(waypoints)
        if trajectory is None:
            return
        if moveit.execute(trajectory):
            moveit.logger.info("Full Cartesian waypoint path succeeded.")
    finally:
        moveit.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
