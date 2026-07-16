#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd -- "$script_dir/.." && pwd)"
ros_test_image="${ROS_TEST_IMAGE:-ros:humble}"
compose_test_project="${COMPOSE_TEST_PROJECT_NAME:-vive_teleop_test}"

usage() {
  cat <<'EOF'
Usage: ./scripts/test-software.sh [--all|--static|--ros|--unity]

  --all     Run static checks, ROS tests, and compile the Unity Linux player.
            This is the default.
  --static  Run non-hardware static checks only.
  --ros     Run static checks, build the complete Compose stack, and run ROS
            C++ and Python tests.
  --unity   Run static checks and compile the Unity Linux player.
EOF
}

mode="${1:---all}"
if (( $# > 1 )); then
  usage >&2
  exit 2
fi

run_ros=false
run_unity=false
case "$mode" in
  --all)
    run_ros=true
    run_unity=true
    ;;
  --static)
    ;;
  --ros)
    run_ros=true
    ;;
  --unity)
    run_unity=true
    ;;
  --help|-h)
    usage
    exit 0
    ;;
  *)
    printf 'Unknown test mode: %s\n' "$mode" >&2
    usage >&2
    exit 2
    ;;
esac

"$script_dir/check-static.sh"

if [[ "$run_ros" == "true" ]]; then
  env \
    CYCLONEDDS_HOST_CONFIG=/tmp/vive-teleop-ci-cyclonedds.xml \
    WEBRTC_TURN_URLS='turn:127.0.0.1:3478?transport=udp' \
    WEBRTC_HOST_IP=192.0.2.10 \
    WEBRTC_PUBLIC_TURN_URLS='turn:192.0.2.10:3478?transport=udp' \
    ROS_FIELD_HOST_IP=192.0.2.20 \
    docker compose \
      --project-name "$compose_test_project" \
      -f "$repo_dir/docker-compose.yml" \
      -f "$repo_dir/docker-compose.wifi.yml" \
      build

  printf '%s\n' 'Docker Compose images built successfully.'

  moveit_test_image="${compose_test_project}-moveit_server:latest"
  if ! docker image inspect "$moveit_test_image" >/dev/null 2>&1; then
    printf '%s\n' 'Could not resolve the built MoveIt server image.' >&2
    exit 1
  fi

  docker run --rm --entrypoint bash "$moveit_test_image" \
    -lc 'source /opt/ros/humble/setup.bash && source /moveit_ws/install/setup.bash && cd /moveit_ws && colcon test --packages-select vive_moveit_server --event-handlers console_direct+ && colcon test-result --verbose'

  docker run --rm --entrypoint bash \
    -v "$repo_dir:/workspace:ro" \
    -w /workspace \
    "$ros_test_image" \
    -lc 'source /opt/ros/humble/setup.bash && PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=/workspace/webrtc_server/src/image_listener:$PYTHONPATH python3 -m pytest -p no:cacheprovider -q webrtc_server/src/image_listener/test'

  docker run --rm --entrypoint bash \
    -v "$repo_dir/data_recorder/src:/recorder_ws/src:ro" \
    "$ros_test_image" \
    -lc 'apt-get update >/dev/null && apt-get install -y --no-install-recommends ros-humble-ament-cmake-gtest ros-humble-rosbag2 ros-humble-sensor-msgs ros-humble-tf2-msgs python3-colcon-common-extensions >/dev/null && source /opt/ros/humble/setup.bash && cd /recorder_ws && colcon build && colcon test --event-handlers console_direct+ && colcon test-result --verbose'

  printf '%s\n' 'Software-only ROS tests passed.'
fi

if [[ "$run_unity" == "true" ]]; then
  "$script_dir/build-unity-vr-linux.sh"
  printf '%s\n' 'Unity Linux compilation passed.'
fi
