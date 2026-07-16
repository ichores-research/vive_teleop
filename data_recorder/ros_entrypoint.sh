#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /recorder_ws/install/setup.bash

exec "$@"
