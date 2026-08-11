"""UR5e fake hardware + MoveIt OMPL/Pilz pipelines + RViz for ROS 2 Humble.

This launch is based on the installed Humble ``ur_moveit.launch.py`` layout,
not on MoveItConfigsBuilder. UR's Humble configuration creates an OMPL pipeline
under the ``move_group`` namespace by default; here we make the two pipeline
names explicit so a MotionPlanRequest can choose Pilz by setting
``pipeline_id='pilz_industrial_motion_planner'`` and ``planner_id='PTP'`` or
``'LIN'``.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ur_moveit_config.launch_common import load_yaml


LAB_PACKAGE = "ur5e_motion_planning_lab"


def launch_moveit_with_pilz(context, *args, **kwargs):
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    prefix = LaunchConfiguration("prefix")

    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "visual_parameters.yaml"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]), " ",
            "robot_ip:=xxx.yyy.zzz.www ",
            "joint_limit_params:=", joint_limit_params, " ",
            "kinematics_params:=", kinematics_params, " ",
            "physical_params:=", physical_params, " ",
            "visual_params:=", visual_params, " ",
            "safety_limits:=", safety_limits, " ",
            "safety_pos_margin:=", safety_pos_margin, " ",
            "safety_k_position:=", safety_k_position, " ",
            "name:=ur ", "ur_type:=", ur_type, " ",
            "script_filename:=ros_control.urscript ",
            "input_recipe_filename:=rtde_input_recipe.txt ",
            "output_recipe_filename:=rtde_output_recipe.txt ",
            "prefix:=", prefix,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_description_semantic = {
        "robot_description_semantic": Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
                PathJoinSubstitution(
                    [FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]
                ),
                " ", "name:=ur ", "prefix:=", prefix,
            ]
        )
    }
    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare("ur_moveit_config"), "config", "kinematics.yaml"]
    )
    robot_description_planning = {
        "robot_description_planning": load_yaml("ur_moveit_config", "config/joint_limits.yaml")
    }
    # Add the Cartesian limits required by Pilz LIN/CIRC to the existing UR
    # joint-limit parameter namespace.
    robot_description_planning["robot_description_planning"].update(
        load_yaml(LAB_PACKAGE, "config/pilz_cartesian_limits.yaml")[
            "robot_description_planning"
        ]
    )

    adapter_chain = (
        "default_planner_request_adapters/AddTimeOptimalParameterization "
        "default_planner_request_adapters/FixWorkspaceBounds "
        "default_planner_request_adapters/FixStartStateBounds "
        "default_planner_request_adapters/FixStartStateCollision "
        "default_planner_request_adapters/FixStartStatePathConstraints"
    )
    ompl_pipeline = {
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": adapter_chain,
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_pipeline["ompl"].update(load_yaml("ur_moveit_config", "config/ompl_planning.yaml"))

    pilz_pipeline = {
        "pilz_industrial_motion_planner": load_yaml(
            LAB_PACKAGE, "config/pilz_industrial_motion_planner_planning.yaml"
        )
    }
    planning_pipelines = {
        "planning_pipelines": {
            "pipeline_names": ["ompl", "pilz_industrial_motion_planner"],
            "default_planning_pipeline": "ompl",
        }
    }

    controllers_yaml = load_yaml("ur_moveit_config", "config/controllers.yaml")
    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        "trajectory_execution.execution_duration_monitoring": False,
    }
    planning_scene_monitor = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            {"publish_robot_description_semantic": True},
            robot_description_kinematics,
            robot_description_planning,
            planning_pipelines,
            ompl_pipeline,
            pilz_pipeline,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor,
            {"use_sim_time": False},
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=[
            "-d",
            PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "rviz", "view_robot.rviz"]),
        ],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            planning_pipelines,
            ompl_pipeline,
            pilz_pipeline,
            {"use_sim_time": False},
        ],
    )
    return [move_group, rviz]


def generate_launch_description():
    ur_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": LaunchConfiguration("ur_type"),
            "robot_ip": "yyy.yyy.yyy.yyy",
            "use_fake_hardware": "true",
            "launch_rviz": "false",
            "initial_joint_controller": "scaled_joint_trajectory_controller",
        }.items(),
    )

    arguments = [
        DeclareLaunchArgument("ur_type", default_value="ur5e"),
        DeclareLaunchArgument("safety_limits", default_value="true"),
        DeclareLaunchArgument("safety_pos_margin", default_value="0.15"),
        DeclareLaunchArgument("safety_k_position", default_value="20"),
        DeclareLaunchArgument("prefix", default_value=""),
    ]
    # Delay MoveIt until fake ros2_control is publishing joint states.
    start_moveit = TimerAction(
        period=5.0,
        actions=[OpaqueFunction(function=launch_moveit_with_pilz)],
    )
    return LaunchDescription(arguments + [ur_control, start_moveit])