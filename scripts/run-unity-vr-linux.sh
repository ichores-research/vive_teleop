#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd -- "$script_dir/.." && pwd)"
build_dir="${repo_dir}/unity-vr-headset/Builds/Linux"
player="${VIVE_TELEOP_PLAYER:-${build_dir}/vive-teleop}"

if [[ ! -x "$player" && -z "${VIVE_TELEOP_PLAYER:-}" &&
      -x "${build_dir}/vive-teleop.x86_64" ]]; then
  player="${build_dir}/vive-teleop.x86_64"
  printf 'Using legacy Unity player name: %s\n' "$player" >&2
fi

if [[ ! -x "$player" ]]; then
  printf 'Unity Linux player not found: %s\n' "$player" >&2
  printf 'Build it first with ./scripts/build-unity-vr-linux.sh\n' >&2
  exit 1
fi

host_ip="$("$script_dir/detect-webrtc-host-ip.sh")"
if [[ -z "$host_ip" ]]; then
  printf 'Could not detect the WebRTC host IP. Set WEBRTC_HOST_IP or WEBRTC_NIC.\n' >&2
  exit 1
fi

config_url="${VIVE_TELEOP_WEBRTC_CONFIG_URL:-http://${host_ip}:8088/config}"
recording_dir="${VIVE_TELEOP_RECORDING_DIR:-${repo_dir}/recordings}"
record_controller="${VIVE_TELEOP_RECORD_CONTROLLER:-1}"
log_dir="${VIVE_TELEOP_LOG_DIR:-${repo_dir}/logs}"
log_path="${log_dir}/unity-player.log"

mkdir -p "$recording_dir" "$log_dir"

printf 'Unity WebRTC config: %s\n' "$config_url"
printf 'Unity player: %s\n' "$player"
printf 'Unity player log: %s\n' "$log_path"
printf 'Controller recordings: %s\n' "$recording_dir"
printf 'Gripper: swipe the right control up to open or down to close.\n'
printf "Tracking: press P to adopt the robot's current pose and restart tracking.\n"

cd "$build_dir"
exec env \
  VIVE_TELEOP_WEBRTC_CONFIG_URL="$config_url" \
  VIVE_TELEOP_RECORD_CONTROLLER="$record_controller" \
  VIVE_TELEOP_RECORDING_DIR="$recording_dir" \
  "$player" \
  "--webrtc-config-url=${config_url}" \
  -force-glcore \
  -screen-fullscreen 0 \
  -screen-width 1280 \
  -screen-height 720 \
  -logFile "$log_path" \
  "$@"
