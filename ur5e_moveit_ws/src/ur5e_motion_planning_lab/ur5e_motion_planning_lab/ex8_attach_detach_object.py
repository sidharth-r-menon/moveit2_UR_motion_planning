#!/usr/bin/env python3
"""Experiment 8: planning-scene payload attach, transport, and detach."""

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AllowedCollisionMatrix,
    AttachedCollisionObject,
    CollisionObject,
    PlanningScene,
)
from shape_msgs.msg import SolidPrimitive

from ur5e_motion_planning_lab.moveit_action_client import (
    BASE_LINK,
    EE_LINK,
    MoveItActionClient,
    pose_goal_constraints,
)

PAYLOAD_ID = "payload_box"
PAYLOAD_SIZE = (0.05, 0.05, 0.10)
TOUCH_LINKS = [EE_LINK]  # Replace/add real finger links when you add a gripper.


def pose_stamped(x, y, z, qx=0.0, qy=1.0, qz=0.0, qw=0.0, frame=BASE_LINK):
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


def attach_existing_world_object() -> AttachedCollisionObject:
    """Tell MoveIt to transfer the named world object onto ``tool0``."""
    attached = AttachedCollisionObject()
    attached.link_name = EE_LINK
    attached.object.id = PAYLOAD_ID
    attached.object.operation = CollisionObject.ADD
    attached.touch_links = TOUCH_LINKS
    return attached


def detach_object() -> AttachedCollisionObject:
    detached = AttachedCollisionObject()
    detached.link_name = EE_LINK
    detached.object.id = PAYLOAD_ID
    detached.object.operation = CollisionObject.REMOVE
    return detached


def allow_tool_payload_contact(allowed: bool) -> PlanningScene:
    """Allow only the deliberate tool0--payload contact during the final approach."""
    matrix = AllowedCollisionMatrix()
    matrix.entry_names = [EE_LINK, PAYLOAD_ID]

    tool_row = AllowedCollisionEntry()
    tool_row.enabled = [False, allowed]
    payload_row = AllowedCollisionEntry()
    payload_row.enabled = [allowed, False]
    matrix.entry_values = [tool_row, payload_row]

    scene = PlanningScene()
    scene.allowed_collision_matrix = matrix
    return scene


def main():
    rclpy.init()
    moveit = MoveItActionClient("ex8_attach_detach_object")
    try:
        if not moveit.wait_for_servers():
            return

        pick_xyz = (0.40, 0.20, 0.15)
        place_xyz = (0.40, -0.20, 0.15)
        hover_z = 0.15

        moveit.logger.info("Step 1: add payload into the collision world.")
        add_scene = PlanningScene()
        add_scene.world.collision_objects.append(
            make_box(PAYLOAD_ID, *pick_xyz, PAYLOAD_SIZE)
        )
        if not moveit.apply_planning_scene(add_scene):
            return

        moveit.logger.info("Step 2: approach the payload.")
        pregrasp = pose_stamped(pick_xyz[0], pick_xyz[1], pick_xyz[2] + hover_z)
        if not moveit.plan_and_execute(pose_goal_constraints(pregrasp, EE_LINK)):
            return

        # A collision object represents the payload. Without a gripper model,
        # asking tool0 to move into its centre is correctly rejected as a
        # collision. Permit this one intended contact pair for the approach.
        moveit.logger.info("Step 3: allow tool0-payload contact and descend.")
        if not moveit.apply_planning_scene(allow_tool_payload_contact(True)):
            return
        grasp = pose_stamped(*pick_xyz)
        if not moveit.plan_and_execute(pose_goal_constraints(grasp, EE_LINK)):
            return

        moveit.logger.info("Step 4: attach payload to tool0 in the planning scene.")
        attach_scene = PlanningScene()
        attach_scene.robot_state.is_diff = True
        attach_scene.robot_state.attached_collision_objects.append(
            attach_existing_world_object()
        )
        if not moveit.apply_planning_scene(attach_scene):
            return

        # After attaching, MoveIt uses touch_links for this permitted contact;
        # remove the temporary global collision allowance.
        moveit.apply_planning_scene(allow_tool_payload_contact(False))

        moveit.logger.info("Step 5: retreat and transfer while carrying payload.")
        retreat = pose_stamped(pick_xyz[0], pick_xyz[1], pick_xyz[2] + hover_z)
        if not moveit.plan_and_execute(pose_goal_constraints(retreat, EE_LINK)):
            return

        place_hover = pose_stamped(place_xyz[0], place_xyz[1], place_xyz[2] + hover_z)
        if not moveit.plan_and_execute(pose_goal_constraints(place_hover, EE_LINK)):
            return

        place = pose_stamped(*place_xyz)
        if not moveit.plan_and_execute(pose_goal_constraints(place, EE_LINK)):
            return

        moveit.logger.info("Step 6: detach payload and re-add it as a world object.")
        detach_scene = PlanningScene()
        detach_scene.robot_state.is_diff = True
        detach_scene.robot_state.attached_collision_objects.append(detach_object())
        detach_scene.world.collision_objects.append(
            make_box(PAYLOAD_ID, *place_xyz, PAYLOAD_SIZE)
        )
        if moveit.apply_planning_scene(detach_scene):
            moveit.logger.info("Pick, carry, and place sequence completed.")
    finally:
        moveit.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
