#!/usr/bin/env bash
# Do not enable `set -u` here: Humble's setup.bash reads optional environment
# variables that may be unset in a fresh container.
set -eo pipefail

source /opt/ros/humble/setup.bash

echo "[ur_ros] Building the mounted lab workspace..."
cd /ws
colcon --log-base log_docker build \
  --packages-select ur5e_motion_planning_lab \
  --symlink-install \
  --build-base build_docker \
  --install-base install_docker

echo "[ur_ros] Starting UR driver against URSim..."
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5 \
  robot_ip:=192.168.56.101 \
  reverse_ip:=192.168.56.102 \
  headless_mode:=true \
  launch_rviz:=false \
  initial_joint_controller:=scaled_joint_trajectory_controller &
driver_pid=$!

cleanup() {
  kill "$driver_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# Give ros2_control time to start its controller manager and joint-state
# broadcaster before move_group starts monitoring the robot state.
sleep 15

echo "[ur_ros] Starting MoveIt and RViz..."
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur5 \
  launch_rviz:=true
