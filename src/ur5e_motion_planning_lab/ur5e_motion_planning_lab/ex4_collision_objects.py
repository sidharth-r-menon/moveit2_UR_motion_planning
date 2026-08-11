#!/usr/bin/env python3
"""Experiment 4: collision-world updates and OMPL obstacle avoidance."""

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive

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


def make_box(object_id, x, y, z, size_xyz, frame=BASE_LINK):
    obj = CollisionObject()
    obj.header.frame_id = frame
    obj.id = object_id
    obj.operation = CollisionObject.ADD

    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = list(size_xyz)

    primitive_pose = Pose()
    primitive_pose.position.x = x
    primitive_pose.position.y = y
    primitive_pose.position.z = z
    primitive_pose.orientation.w = 1.0

    obj.primitives.append(primitive)
    obj.primitive_poses.append(primitive_pose)
    return obj


def main():
    rclpy.init()
    moveit = MoveItActionClient("ex4_collision_objects")
    obstacle_id = "center_obstacle"
    try:
        if not moveit.wait_for_servers():
            return

        start = pose(0.40, -0.25, 0.35)
        goal = pose(0.40, 0.25, 0.35)

        moveit.logger.info("Step 1: move to the unobstructed start pose.")
        if not moveit.plan_and_execute(pose_goal_constraints(start, EE_LINK)):
            return

        moveit.logger.info("Step 2: add a blocking box to the MoveIt world.")
        scene = PlanningScene()
        scene.world.collision_objects.append(
            make_box(obstacle_id, 0.40, 0.0, 0.35, (0.15, 0.15, 0.50))
        )
        if not moveit.apply_planning_scene(scene):
            return

        moveit.logger.info("Step 3: OMPL plans around the collision object.")
        if moveit.plan_and_execute(pose_goal_constraints(goal, EE_LINK)):
            moveit.logger.info("Obstacle-avoidance motion succeeded.")
        else:
            moveit.logger.warn(
                "No valid detour was found. Increase planning time or reduce/move the box."
            )

        moveit.logger.info("Step 4: remove the obstacle and return.")
        remove = CollisionObject()
        remove.id = obstacle_id
        remove.header.frame_id = BASE_LINK
        remove.operation = CollisionObject.REMOVE
        clean_scene = PlanningScene()
        clean_scene.world.collision_objects.append(remove)
        if moveit.apply_planning_scene(clean_scene):
            moveit.plan_and_execute(pose_goal_constraints(start, EE_LINK))
    finally:
        moveit.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
