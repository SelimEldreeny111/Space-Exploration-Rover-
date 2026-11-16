#!/usr/bin/env bash
# Source ROS 2 Jazzy and the MISSION X workspace, then run the command.
set -e
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source /ws/install/setup.bash
exec "$@"
