#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file:///ros2_ws/cyclonedds.xml}"
export ROS_DOMAIN_ID=0

TOPIC="/xtion/rgb/image_raw"

echo "Waiting for ROS2 bridge topic: $TOPIC"

for i in {1..90}; do
    echo "Attempt $i/90"

    if ros2 topic list 2>/dev/null | grep -q "$TOPIC"; then
        echo "Bridge topic detected"
        exit 0
    fi

    sleep 2
done

echo "Timeout waiting for bridge topic: $TOPIC"
exit 1
