#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd -- "$script_dir/.." && pwd)"
ros_test_image="${ROS_TEST_IMAGE:-ros:humble}"

"$script_dir/check-static.sh"

if [[ "${1:-}" == "--static" ]]; then
  exit 0
fi

docker run --rm --entrypoint bash \
  -v "$repo_dir:/workspace:ro" \
  -w /workspace \
  "$ros_test_image" \
  -lc 'source /opt/ros/humble/setup.bash && PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=/workspace/moveit_server/src/vive_moveit_server:/workspace/webrtc_server/src/image_listener:$PYTHONPATH python3 -m pytest -p no:cacheprovider -q moveit_server/src/vive_moveit_server/test webrtc_server/src/image_listener/test'

printf '%s\n' 'Software-only ROS tests passed.'
