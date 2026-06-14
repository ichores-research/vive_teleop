#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd -- "$script_dir/.." && pwd)"
unity_project="${repo_dir}/unity-vr-headset"
unity_player="${unity_project}/Builds/Linux/vive-teleop"
unity_build_stamp="${unity_player}.build-stamp"

unity_build_required=false
if [[ "${VIVE_TELEOP_FORCE_UNITY_BUILD:-0}" == "1" ||
      ! -x "$unity_player" ||
      ! -f "$unity_build_stamp" ||
      "$script_dir/build-unity-vr-linux.sh" -nt "$unity_build_stamp" ]]; then
  unity_build_required=true
else
  newer_unity_source="$(
    find \
      "$unity_project/Assets" \
      "$unity_project/Packages" \
      "$unity_project/ProjectSettings" \
      -type f -newer "$unity_build_stamp" -print -quit
  )"
  if [[ -n "$newer_unity_source" ]]; then
    unity_build_required=true
  fi
fi

if [[ "${VIVE_TELEOP_SKIP_UNITY_BUILD:-0}" == "1" ]]; then
  printf 'Skipping automatic Unity build.\n'
elif [[ "$unity_build_required" == "true" ]]; then
  printf 'Unity sources changed; building the Linux player.\n'
  "$script_dir/build-unity-vr-linux.sh" "$unity_player"
else
  printf 'Unity Linux player is up to date.\n'
fi

"$script_dir/up-wifi-webrtc.sh" -d

host_ip="$("$script_dir/detect-webrtc-host-ip.sh")"
config_url="http://${host_ip}:8088/config"
robot_ip="${ROBOT_IP:-10.68.0.1}"
camera_topic="${ROBOT_CAMERA_TOPIC:-/head_front_camera/rgb/image_raw}"
camera_wait_seconds="${ROBOT_CAMERA_WAIT_SECONDS:-45}"

printf 'Waiting for WebRTC signaling at %s\n' "$config_url"
for _attempt in $(seq 1 60); do
  if curl --fail --silent --max-time 1 "$config_url" >/dev/null; then
    break
  fi
  sleep 1
done

if ! curl --fail --silent --max-time 2 "$config_url" >/dev/null; then
  printf 'WebRTC signaling did not become ready: %s\n' "$config_url" >&2
  exit 1
fi

printf 'Waiting for robot camera publisher on %s\n' "$camera_topic"
camera_publisher_count=""
for _attempt in $(seq 1 "$camera_wait_seconds"); do
  camera_publisher_count="$(
    timeout 3 docker exec ros2_app_wifi bash -lc \
      'source /opt/ros/humble/setup.bash && ros2 topic info "$1"' \
      _ "$camera_topic" 2>/dev/null |
      awk '/Publisher count:/ { print $3; exit }' || true
  )"
  if [[ "$camera_publisher_count" =~ ^[1-9][0-9]*$ ]]; then
    break
  fi
  sleep 1
done

if [[ ! "$camera_publisher_count" =~ ^[1-9][0-9]*$ ]]; then
  if ping -c 1 -W 1 "$robot_ip" >/dev/null 2>&1; then
    printf 'Robot %s is online, but ROS2 has no publisher for %s.\n' \
      "$robot_ip" "$camera_topic" >&2
    printf 'Start the robot ROS2 bringup/camera application at http://%s, then rerun this script.\n' \
      "$robot_ip" >&2
  else
    printf 'Robot %s is unreachable, so camera video cannot start.\n' \
      "$robot_ip" >&2
  fi
  exit 1
fi

printf 'Robot camera publisher is available.\n'

WEBRTC_HOST_IP="$host_ip" "$script_dir/check-teleop-runtime.sh"

if ! pgrep -f '/SteamVR/.*/vrserver' >/dev/null 2>&1; then
  printf 'Starting SteamVR through Steam...\n'
  steam -applaunch 250820 >/dev/null 2>&1 &
else
  printf 'SteamVR is already running.\n'
fi

printf 'Waiting for SteamVR server...\n'
for _attempt in $(seq 1 60); do
  if pgrep -f '/SteamVR/.*/vrserver' >/dev/null 2>&1; then
    break
  fi
  sleep 1
done

if ! pgrep -f '/SteamVR/.*/vrserver' >/dev/null 2>&1; then
  printf 'SteamVR did not become ready within 60 seconds.\n' >&2
  exit 1
fi

cd "$repo_dir"
exec "$script_dir/run-unity-vr-linux.sh" "$@"
