#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /ros1_bridge_ws/install/setup.bash

exec "$@"
