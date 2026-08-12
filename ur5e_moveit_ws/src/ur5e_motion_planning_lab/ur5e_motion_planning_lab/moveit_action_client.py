"""Small rclpy client layer for a running MoveIt move_group node (ROS 2 Humble)."""

from __future__ import annotations

from typing import Iterable, Mapping

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
    RobotTrajectory,
)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
from rclpy.action import ActionClient
from shape_msgs.msg import SolidPrimitive


PLANNING_GROUP = "ur_manipulator"
EE_LINK = "tool0"
BASE_LINK = "base_link"


def joint_goal_constraints(joint_positions: Mapping[str, float]) -> Constraints:
    constraints = Constraints()
    for name, position in joint_positions.items():
        joint = JointConstraint()
        joint.joint_name = name
        joint.position = float(position)
        joint.tolerance_above = 0.001
        joint.tolerance_below = 0.001
        joint.weight = 1.0
        constraints.joint_constraints.append(joint)
    return constraints


def pose_goal_constraints(
    target: PoseStamped,
    link_name: str = EE_LINK,
    position_tolerance: float = 0.002,
    orientation_tolerance: float = 0.01,
) -> Constraints:
    """Create a six-DOF pose goal using only standard MoveIt messages."""
    constraints = Constraints()

    position = PositionConstraint()
    position.header = target.header
    position.link_name = link_name
    position.weight = 1.0

    tolerance_box = SolidPrimitive()
    tolerance_box.type = SolidPrimitive.BOX
    tolerance_box.dimensions = [position_tolerance] * 3

    box_pose = Pose()
    box_pose.position = target.pose.position
    box_pose.orientation.w = 1.0

    region = BoundingVolume()
    region.primitives.append(tolerance_box)
    region.primitive_poses.append(box_pose)
    position.constraint_region = region

    orientation = OrientationConstraint()
    orientation.header = target.header
    orientation.link_name = link_name
    orientation.orientation = target.pose.orientation
    orientation.absolute_x_axis_tolerance = orientation_tolerance
    orientation.absolute_y_axis_tolerance = orientation_tolerance
    orientation.absolute_z_axis_tolerance = orientation_tolerance
    orientation.weight = 1.0

    constraints.position_constraints.append(position)
    constraints.orientation_constraints.append(orientation)
    return constraints


class MoveItActionClient:
    """Plan and execute against an already running ``move_group`` node."""

    def __init__(self, node_name: str):
        self.node = rclpy.create_node(node_name)
        self._plan_client = ActionClient(self.node, MoveGroup, "/move_action")
        self._execute_client = ActionClient(
            self.node, ExecuteTrajectory, "/execute_trajectory"
        )
        self._apply_scene_client = self.node.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )
        self._cartesian_path_client = self.node.create_client(
            GetCartesianPath, "/compute_cartesian_path"
        )

    @property
    def logger(self):
        return self.node.get_logger()

    def wait_for_servers(self, timeout_sec: float = 10.0) -> bool:
        action_checks = (
            (self._plan_client, "/move_action"),
            (self._execute_client, "/execute_trajectory"),
        )
        for client, name in action_checks:
            if not client.wait_for_server(timeout_sec=timeout_sec):
                self.logger.error(f"{name} is unavailable. Start the MoveIt launch first.")
                return False

        # ``/apply_planning_scene`` is a ROS service, not an action server.
        if not self._apply_scene_client.wait_for_service(timeout_sec=timeout_sec):
            self.logger.error(
                "/apply_planning_scene is unavailable. Start the MoveIt launch first."
            )
            return False
        if not self._cartesian_path_client.wait_for_service(timeout_sec=timeout_sec):
            self.logger.error(
                "/compute_cartesian_path is unavailable. Start the MoveIt launch first."
            )
            return False
        return True

    def plan(
        self,
        goal_constraints: Constraints,
        *,
        pipeline_id: str = "",
        planner_id: str = "",
        path_constraints: Constraints | None = None,
        planning_time: float = 5.0,
        attempts: int = 5,
    ):
        """Return a successful ``MoveGroup.Result`` or ``None`` on failure."""
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = PLANNING_GROUP
        request.goal_constraints.append(goal_constraints)
        request.start_state.is_diff = True  # take the latest /joint_states state
        request.num_planning_attempts = attempts
        request.allowed_planning_time = planning_time
        request.max_velocity_scaling_factor = 0.25
        request.max_acceleration_scaling_factor = 0.25
        request.pipeline_id = pipeline_id
        request.planner_id = planner_id
        if path_constraints is not None:
            request.path_constraints = path_constraints

        # Planning and execution are deliberately separate for experiments.
        goal.planning_options.plan_only = True

        send_future = self._plan_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.logger.error("MoveGroup rejected the planning request.")
            return None

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result().result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.logger.error(f"Planning failed (MoveIt code {result.error_code.val}).")
            return None
        return result

    def execute(self, trajectory: RobotTrajectory) -> bool:
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        send_future = self._execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.logger.error("ExecuteTrajectory rejected the trajectory.")
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result().result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.logger.error(f"Execution failed (MoveIt code {result.error_code.val}).")
            return False
        return True

    def plan_and_execute(self, goal_constraints: Constraints, **plan_kwargs) -> bool:
        result = self.plan(goal_constraints, **plan_kwargs)
        if result is None:
            return False
        return self.execute(result.planned_trajectory)

    def compute_cartesian_path(
        self,
        waypoints: Iterable[PoseStamped],
        *,
        max_step: float = 0.01,
        jump_threshold: float = 0.0,
    ) -> RobotTrajectory | None:
        """Return a collision-checked Cartesian waypoint path.

        This is MoveIt's standard Cartesian interpolation service. It does
        not require the optional Pilz pipeline; every waypoint is expressed
        in the same base frame.
        """
        waypoints = list(waypoints)
        if not waypoints:
            self.logger.error("Cartesian path needs at least one waypoint.")
            return None
        frame = waypoints[0].header.frame_id
        if any(waypoint.header.frame_id != frame for waypoint in waypoints):
            self.logger.error("All Cartesian waypoints must share one frame.")
            return None

        request = GetCartesianPath.Request()
        request.header.frame_id = frame
        request.group_name = PLANNING_GROUP
        request.link_name = EE_LINK
        request.start_state.is_diff = True
        request.waypoints = [waypoint.pose for waypoint in waypoints]
        request.max_step = max_step
        request.jump_threshold = jump_threshold
        request.avoid_collisions = True

        future = self._cartesian_path_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()
        if response is None or response.error_code.val != MoveItErrorCodes.SUCCESS:
            code = "no response" if response is None else response.error_code.val
            self.logger.error(f"Cartesian-path request failed (MoveIt code {code}).")
            return None
        if response.fraction < 0.999:
            self.logger.error(
                f"Cartesian path is only {response.fraction * 100:.1f}% complete; not executing it."
            )
            return None
        return response.solution

    def apply_planning_scene(self, scene_diff: PlanningScene) -> bool:
        scene_diff.is_diff = True
        request = ApplyPlanningScene.Request()
        request.scene = scene_diff
        future = self._apply_scene_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()
        if response is None or not response.success:
            self.logger.error("MoveIt rejected the planning-scene update.")
            return False
        return True

    def shutdown(self):
        self.node.destroy_node()
