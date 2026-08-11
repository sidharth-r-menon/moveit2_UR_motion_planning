"""
Bring up UR5e fake (mock) hardware + MoveIt 2 + RViz in one command.

This wraps the two commands documented by the UR driver into a single
launch file:

    ros2 launch ur_robot_driver ur_control.launch.py \
        ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy \
        use_fake_hardware:=true launch_rviz:=false
    ros2 launch ur_moveit_config ur_moveit.launch.py \
        ur_type:=ur5e launch_rviz:=true

robot_ip is required by ur_control.launch.py's argument parser even in fake
mode (it's simply unused when use_fake_hardware:=true) -- yyy.yyy.yyy.yyy is
the driver docs' own placeholder value, kept here for consistency.

Usage:
    ros2 launch ur5e_motion_planning_lab ur5e_fake_moveit.launch.py

Then in a separate terminal, run any experiment script, e.g.:
    ros2 run ur5e_motion_planning_lab ex1_joint_goal

NOTE: if your ur_robot_driver / ur_moveit_config versions expose different
argument names than shown here (driver launch API has shifted across
Humble point releases), check with:
    ros2 launch ur_robot_driver ur_control.launch.py --show-args
    ros2 launch ur_moveit_config ur_moveit.launch.py --show-args
and adjust the LaunchConfiguration/IncludeLaunchDescription args below to
match.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type_arg = DeclareLaunchArgument(
        "ur_type", default_value="ur5e", description="UR robot model"
    )

    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": LaunchConfiguration("ur_type"),
            "robot_ip": "yyy.yyy.yyy.yyy",  # unused with fake hardware
            "use_fake_hardware": "true",
            "launch_rviz": "false",  # RViz comes from ur_moveit.launch.py instead
            "initial_joint_controller": "scaled_joint_trajectory_controller",
        }.items(),
    )

    # ur_moveit.launch.py expects the driver's controller manager to already
    # be up and publishing /joint_states before it starts planning-scene
    # monitoring -- a short delay avoids a race on first launch. If you still
    # see "didn't receive robot state" warnings, increase this.
    ur_moveit_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"]
                    )
                ),
                launch_arguments={
                    "ur_type": LaunchConfiguration("ur_type"),
                    "use_fake_hardware": "true",
                    "launch_rviz": "true",
                    "initial_joint_controller": "scaled_joint_trajectory_controller",
                }.items(),
            )
        ],
    )

    return LaunchDescription(
        [
            ur_type_arg,
            ur_control_launch,
            ur_moveit_launch,
        ]
    )
