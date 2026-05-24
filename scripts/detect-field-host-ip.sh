#!/usr/bin/env bash
set -euo pipefail

if [[ -n "${ROS_FIELD_HOST_IP:-}" ]]; then
  printf '%s\n' "$ROS_FIELD_HOST_IP"
  exit 0
fi

robot_ip="${ROBOT_IP:-10.68.0.1}"
ip -4 route get "$robot_ip" | awk '{
  for (i = 1; i <= NF; i++) {
    if ($i == "src") {
      print $(i + 1)
      exit
    }
  }
}'
