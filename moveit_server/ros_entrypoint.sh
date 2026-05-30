#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

if [ -f /moveit_ws/install/setup.bash ]; then
  source /moveit_ws/install/setup.bash
fi

if [ -n "${EXTRA_ROS_SETUP:-}" ] && [ -f "${EXTRA_ROS_SETUP}" ]; then
  source "${EXTRA_ROS_SETUP}"
fi

exec "$@"
