"""
Shared setup for all experiment scripts in this package.

Every ex*.py script needs the same three things:
  1. A MoveItPy instance built from the ur_moveit_config MoveItConfigsBuilder
     (this pulls in the SRDF, kinematics.yaml, joint limits, OMPL config, and
     controller config that the UR driver repo already ships).
  2. The "ur_manipulator" PlanningComponent (the move group for the arm).
  3. A rclpy logger.

Why moveit_py and not moveit_commander:
  moveit_commander is the ROS 1-era Python API, ported but now deprecated in
  Humble/Iron. moveit_py is the actively maintained pybind11 binding directly
  onto the C++ MoveIt core (same MoveGroupInterface / PlanningSceneMonitor /
  PlanningComponent objects, no client-server round trip through move_group's
  action server for planning-only calls). It's the one you want to actually
  learn, and it's what current MoveIt tutorials use.

Usage pattern in each experiment script:

    from ur5e_motion_planning_lab.mgi_base import build_moveitpy, PLANNING_GROUP

    def main():
        moveit, logger = build_moveitpy("ex1_joint_goal")
        arm = moveit.get_planning_component(PLANNING_GROUP)
        ...
"""

from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import rclpy
import yaml
import os


PLANNING_GROUP = "ur_manipulator"
EE_LINK = "tool0"
BASE_LINK = "base_link"


def _load_named_plan_request_params() -> dict:
    """
    Loads config/moveit_py_params.yaml (the named plan_request_params blocks
    for OMPL/Pilz planner comparisons) and returns it as a plain dict, ready
    to be merged into the MoveItConfigsBuilder's config_dict.

    We read it straight off disk from this package's installed share
    directory rather than via ROS parameter substitution, since MoveItPy's
    config_dict is just a plain nested dict at construction time.
    """
    share_dir = get_package_share_directory("ur5e_motion_planning_lab")
    params_path = os.path.join(share_dir, "config", "moveit_py_params.yaml")
    with open(params_path, "r") as f:
        return yaml.safe_load(f)


def build_moveit_config():
    """
    Builds the MoveIt configuration the same way the driver's own
    ur_moveit.launch.py does, but returns the config object instead of
    launching nodes, so we can hand it directly to MoveItPy in-process.

    ur_type / robot params match what you bring up with:
      ros2 launch ur_robot_driver ur_control.launch.py \
          ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true \
          launch_rviz:=false
      ros2 launch ur_moveit_config ur_moveit.launch.py \
          ur_type:=ur5e launch_rviz:=true
    """
    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description(
            file_path="config/ur.urdf.xacro",
            mappings={
                "ur_type": "ur5e",
                "name": "ur",
                "safety_limits": "true",
            },
        )
        .robot_description_semantic(file_path="srdf/ur.srdf.xacro", mappings={"name": "ur"})
        .trajectory_execution("config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )
    return moveit_config


def build_moveitpy(node_name: str):
    """
    Initializes rclpy (if needed) and constructs a MoveItPy instance with
    both the standard UR MoveIt config AND this package's named
    plan_request_params blocks (ompl_rrtc, ompl_rrtstar, pilz_lin, etc.)
    merged in, so any ex*.py script can reference those names directly via
    PlanRequestParameters(moveit, "<name>").

    Returns (moveit_py_instance, logger). Caller must call moveit.shutdown()
    and rclpy.shutdown() when done (see the `finally` block in each ex*.py).
    """
    if not rclpy.ok():
        rclpy.init()

    logger = rclpy.logging.get_logger(node_name)
    moveit_config = build_moveit_config()

    config_dict = moveit_config.to_dict()
    config_dict.update(_load_named_plan_request_params())

    moveit = MoveItPy(node_name=node_name, config_dict=config_dict)
    logger.info(f"MoveItPy initialized for node '{node_name}'.")
    return moveit, logger


def plan_and_execute(
    moveit,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
):
    """
    Standard plan -> execute pattern, matching the official MoveIt 2 Humble
    moveit_py tutorial exactly:
      https://moveit.picknik.ai/humble/doc/examples/motion_planning_python_api/motion_planning_python_api_tutorial.html

    Execution goes through the top-level MoveItPy.execute(robot_trajectory,
    controllers=[]) call -- NOT planning_component.execute(), which does not
    exist in the current Python bindings (see moveit2_tutorials issue #643).

    Returns True on success, False otherwise. Never raises on a planning
    failure -- a failed plan is a normal, expected event when sweeping
    planners or stress-testing collision scenes, and should not crash your
    script.
    """
    logger.info("Planning...")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(multi_plan_parameters=multi_plan_parameters)
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(single_plan_parameters=single_plan_parameters)
    else:
        plan_result = planning_component.plan()

    if not plan_result:
        logger.error("Planning failed.")
        return False

    logger.info("Executing...")
    robot_trajectory = plan_result.trajectory
    moveit.execute(robot_trajectory, controllers=[])
    return True
