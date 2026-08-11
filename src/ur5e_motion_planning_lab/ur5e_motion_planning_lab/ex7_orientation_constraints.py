#!/usr/bin/env python3
"""Experiment 7: a genuine whole-path orientation constraint with OMPL."""

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Constraints, OrientationConstraint

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


def level_path_constraint(tolerance_rad: float = 0.20) -> Constraints:
    """Keep tool0 down-facing throughout the complete trajectory."""
    orientation = OrientationConstraint()
    orientation.header.frame_id = BASE_LINK
    orientation.link_name = EE_LINK
    orientation.orientation.y = 1.0
    orientation.orientation.w = 0.0
    orientation.absolute_x_axis_tolerance = tolerance_rad
    orientation.absolute_y_axis_tolerance = tolerance_rad
    orientation.absolute_z_axis_tolerance = tolerance_rad
    orientation.weight = 1.0

    constraints = Constraints()
    constraints.orientation_constraints.append(orientation)
    return constraints


def main():
    rclpy.init()
    moveit = MoveItActionClient("ex7_orientation_constraints")
    try:
        if not moveit.wait_for_servers():
            return

        # Baseline without a path constraint.
        baseline = pose(0.35, 0.30, 0.30)
        moveit.logger.info("Baseline: unconstrained OMPL pose motion.")
        if not moveit.plan_and_execute(pose_goal_constraints(baseline, EE_LINK)):
            return

        # Same style of goal, but orientation must remain within +/- 0.20 rad
        # of down-facing for every state along the path, not only at the goal.
        constrained_target = pose(0.35, -0.30, 0.30)
        moveit.logger.info("Constrained: keep the tool level for the full path.")
        ok = moveit.plan_and_execute(
            pose_goal_constraints(constrained_target, EE_LINK),
            path_constraints=level_path_constraint(),
            planning_time=10.0,
        )
        if not ok:
            moveit.logger.warn(
                "Constrained plan failed. Loosen tolerance_rad or choose a nearer target."
            )
    finally:
        moveit.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
