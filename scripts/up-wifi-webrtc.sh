#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd -- "$script_dir/.." && pwd)"

host_ip="$("$script_dir/detect-webrtc-host-ip.sh")"
if [[ -z "$host_ip" ]]; then
  printf 'Could not detect a WebRTC host IP. Set WEBRTC_HOST_IP or WEBRTC_NIC.\n' >&2
  exit 1
fi

export WEBRTC_HOST_IP="$host_ip"
export TURN_USER="${TURN_USER:-dummy}"
export TURN_PASSWORD="${TURN_PASSWORD:-dummy}"
export WEBRTC_TURN_URLS="${WEBRTC_TURN_URLS:-turn:${WEBRTC_HOST_IP}:3478?transport=udp,turn:${WEBRTC_HOST_IP}:3478?transport=tcp}"

printf 'Using WebRTC host IP: %s\n' "$WEBRTC_HOST_IP"
printf 'Signaling URL: http://%s:8088\n' "$WEBRTC_HOST_IP"
printf 'TURN URLs: %s\n' "$WEBRTC_TURN_URLS"

cd "$repo_dir"
exec docker compose -f docker-compose.yml -f docker-compose.wifi.yml up --build "$@"
