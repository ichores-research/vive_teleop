#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd -- "$script_dir/.." && pwd)"
log_root="${VIVE_TELEOP_LOG_ROOT:-${repo_dir}/logs}"
run_stamp="$(date +%Y%m%d-%H%M%S)"
run_log_dir="${VIVE_TELEOP_RUN_LOG_DIR:-${log_root}/${run_stamp}}"
startup_log="${run_log_dir}/start-vive-teleop.log"
run_started_epoch="$(date +%s)"
unity_project="${repo_dir}/unity-vr-headset"
unity_player="${unity_project}/Builds/Linux/vive-teleop"
unity_build_stamp="${unity_player}.build-stamp"
log_follow_pids=()
launch_lock="${XDG_RUNTIME_DIR:-/tmp}/vive-teleop-start.lock"

exec 9>"$launch_lock"
if ! flock --nonblock 9; then
  printf 'Another vive_teleop launcher is already running (lock: %s).\n' \
    "$launch_lock" >&2
  exit 75
fi

mkdir -p "$run_log_dir"
exec > >(tee -a "$startup_log") 2>&1

log() {
  printf '[%(%Y-%m-%dT%H:%M:%S%z)T] %s\n' -1 "$*"
}

run_and_log() {
  local label="$1"
  local log_path="$2"
  shift 2

  log "Starting ${label}; detailed log: ${log_path}"
  "$@" > >(tee -a "$log_path") 2> >(tee -a "$log_path" >&2)
  log "Finished ${label}"
}

start_container_log() {
  local container="$1"
  local log_path="${run_log_dir}/${container}.log"

  : > "$log_path"
  if ! docker ps --format '{{.Names}}' | grep -Fxq "$container"; then
    log "Container ${container} is not running; log capture skipped"
    return
  fi

  log "Capturing ${container} output to ${log_path}"
  docker logs --timestamps --follow --since "$run_started_epoch" "$container" \
    >>"$log_path" 2>&1 &
  log_follow_pids+=("$!")
}

stop_background_logs() {
  local status=$?

  if [[ "${#log_follow_pids[@]}" -gt 0 ]]; then
    log "Stopping background Docker log followers"
    for pid in "${log_follow_pids[@]}"; do
      kill "$pid" >/dev/null 2>&1 || true
    done
    wait "${log_follow_pids[@]}" >/dev/null 2>&1 || true
  fi

  log "Run log directory: ${run_log_dir}"
  log "start-vive-teleop exiting with status ${status}"
}

trap stop_background_logs EXIT

log "vive_teleop startup begin"
log "Repository: ${repo_dir}"
log "Run log directory: ${run_log_dir}"

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
  log "Skipping automatic Unity build"
elif [[ "$unity_build_required" == "true" ]]; then
  log "Unity sources changed; building the Linux player"
  run_and_log \
    "Unity Linux player build" \
    "${run_log_dir}/unity-build.log" \
    "$script_dir/build-unity-vr-linux.sh" "$unity_player"
else
  log "Unity Linux player is up to date"
fi

run_and_log \
  "Docker Compose Wi-Fi WebRTC stack" \
  "${run_log_dir}/docker-compose-up.log" \
  "$script_dir/up-wifi-webrtc.sh" -d

start_container_log "webrtc_server_wifi"
start_container_log "moveit_server_wifi"
start_container_log "coturn_wifi"

host_ip="$("$script_dir/detect-webrtc-host-ip.sh")"
config_url="http://${host_ip}:8088/config"
robot_ip="${ROBOT_IP:-10.68.0.1}"
camera_topic="${ROBOT_CAMERA_TOPIC:-/head_front_camera/rgb/image_raw}"
camera_wait_seconds="${ROBOT_CAMERA_WAIT_SECONDS:-45}"

log "Waiting for WebRTC signaling at ${config_url}"
for _attempt in $(seq 1 60); do
  if curl --fail --silent --max-time 1 "$config_url" >/dev/null; then
    break
  fi
  sleep 1
done

if ! curl --fail --silent --max-time 2 "$config_url" >/dev/null; then
  log "WebRTC signaling did not become ready: ${config_url}"
  exit 1
fi
log "WebRTC signaling is ready"

log "Waiting for robot camera publisher on ${camera_topic}"
camera_publisher_count=""
for _attempt in $(seq 1 "$camera_wait_seconds"); do
  camera_publisher_count="$(
    timeout 3 docker exec webrtc_server_wifi bash -lc \
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
    log "Robot ${robot_ip} is online, but ROS2 has no publisher for ${camera_topic}"
    log "Start the robot ROS2 bringup/camera application at http://${robot_ip}, then rerun this script"
  else
    log "Robot ${robot_ip} is unreachable, so camera video cannot start"
  fi
  exit 1
fi

log "Robot camera publisher is available"

run_and_log \
  "teleop runtime validation" \
  "${run_log_dir}/check-teleop-runtime.log" \
  env WEBRTC_HOST_IP="$host_ip" "$script_dir/check-teleop-runtime.sh"

if ! pgrep -f '/SteamVR/.*/vrserver' >/dev/null 2>&1; then
  log "Starting SteamVR through Steam; detailed log: ${run_log_dir}/steamvr-start.log"
  steam -applaunch 250820 >"${run_log_dir}/steamvr-start.log" 2>&1 &
else
  log "SteamVR is already running"
fi

log "Waiting for SteamVR server and compositor"
for _attempt in $(seq 1 60); do
  if pgrep -f '/SteamVR/.*/vrserver' >/dev/null 2>&1 &&
     pgrep -f '/SteamVR/.*/vrcompositor' >/dev/null 2>&1; then
    break
  fi
  sleep 1
done

if ! pgrep -f '/SteamVR/.*/vrserver' >/dev/null 2>&1; then
  log "SteamVR server did not become ready within 60 seconds"
  exit 1
fi

if ! pgrep -f '/SteamVR/.*/vrcompositor' >/dev/null 2>&1; then
  log "SteamVR compositor did not become ready within 60 seconds"
  log "Check ~/.local/share/Steam/logs/vrcompositor.txt for direct-display or DRM lease errors"
  exit 1
fi
log "SteamVR server and compositor are ready"

cd "$repo_dir"
log "Starting Unity player; wrapper log: ${run_log_dir}/unity-wrapper.log"
run_and_log \
  "Unity player" \
  "${run_log_dir}/unity-wrapper.log" \
  env VIVE_TELEOP_LOG_DIR="$run_log_dir" \
    "$script_dir/run-unity-vr-linux.sh" "$@"
