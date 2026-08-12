FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Runtime dependencies for the UR ROS 2 driver, its MoveIt configuration, and
# building/running the Python experiment package mounted at /ws.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ur-robot-driver \
    ros-humble-ur-moveit-config \
    ros-humble-moveit \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-xacro \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-yaml \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws

COPY start_ros_stack.sh /usr/local/bin/start_ros_stack.sh
RUN chmod +x /usr/local/bin/start_ros_stack.sh

# The workspace is deliberately mounted at runtime.  This keeps source code
# editable on the Windows host and avoids baking user files into the image.
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
