# syntax=docker/dockerfile:1
# MISSION X rover - ROS 2 Jazzy image for simulation, SLAM / Nav2 tests and the
# headless evaluation run. Build context = repository root (docker-compose.yml):
#   docker compose -f docker/docker-compose.yml build
# (the real robot runs natively on the Raspberry Pi 4, see docs/ROS2.md)
FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    RCUTILS_COLORIZED_OUTPUT=0

# ros-base already contains colcon, rosdep, rosidl generators and rosbag2
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-navigation2 \
        ros-jazzy-nav2-bringup \
        ros-jazzy-slam-toolbox \
        ros-jazzy-robot-localization \
        ros-jazzy-xacro \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-joint-state-publisher \
        ros-jazzy-teleop-twist-keyboard \
        python3-serial \
        python3-numpy \
        python3-yaml \
        python3-pytest \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY ros2_ws/src /ws/src
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --event-handlers console_direct+ \
         --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf /ws/build /ws/log

COPY docker/entrypoint.sh /entrypoint.sh
COPY docker/scripts /ws/scripts
# tolerate CRLF checkouts on Windows hosts
RUN sed -i 's/\r$//' /entrypoint.sh /ws/scripts/*.sh \
    && chmod +x /entrypoint.sh /ws/scripts/*.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
