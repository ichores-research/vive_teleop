#!/bin/bash
set -euo pipefail

# Matches bridge/Dockerfile: Noetic + Foxy + ros1_bridge + vive_head_pose + tiago mapper workspace.
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash
source /bridge_ws/install/setup.bash
source /ros2_vive_ws/install/setup.bash
source /catkin_ws/devel/setup.bash

exec "$@"
