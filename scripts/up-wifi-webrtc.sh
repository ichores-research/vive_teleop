#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd -- "$script_dir/.." && pwd)"

host_ip="$("$script_dir/detect-webrtc-host-ip.sh")"
if [[ -z "$host_ip" ]]; then
  printf 'Could not detect a WebRTC host IP. Set WEBRTC_HOST_IP or WEBRTC_NIC.\n' >&2
  exit 1
fi

field_host_ip="$("$script_dir/detect-field-host-ip.sh")"
if [[ -z "$field_host_ip" ]]; then
  printf 'Could not detect the field-network host IP. Set ROS_FIELD_HOST_IP.\n' >&2
  exit 1
fi

export WEBRTC_HOST_IP="$host_ip"
export ROS_FIELD_HOST_IP="$field_host_ip"
export ROBOT_IP="${ROBOT_IP:-10.68.0.1}"
export ROS2_DDS_INTERFACE="${ROS2_DDS_INTERFACE:-$ROS_FIELD_HOST_IP}"
export ROS2_DDS_ALLOW_MULTICAST="${ROS2_DDS_ALLOW_MULTICAST:-true}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-67}"
export TURN_USER="${TURN_USER:-dummy}"
export TURN_PASSWORD="${TURN_PASSWORD:-dummy}"
export VIVE_TELEOP_RECORD_DATASET="${VIVE_TELEOP_RECORD_DATASET:-0}"
export VIVE_TELEOP_RECORDING_ROOT="${VIVE_TELEOP_RECORDING_ROOT:-${repo_dir}/recordings}"
export VIVE_TELEOP_RECORDING_MODE="${VIVE_TELEOP_RECORDING_MODE:-deadman_window}"
export VIVE_TELEOP_RECORDING_POST_ROLL_SEC="${VIVE_TELEOP_RECORDING_POST_ROLL_SEC:-0.75}"
export VIVE_TELEOP_RECORDING_GATE_TIMEOUT_SEC="${VIVE_TELEOP_RECORDING_GATE_TIMEOUT_SEC:-0.25}"
export VIVE_TELEOP_RECORDING_MIN_FREE_BYTES="${VIVE_TELEOP_RECORDING_MIN_FREE_BYTES:-20000000000}"
export VIVE_TELEOP_RECORDING_UID="${VIVE_TELEOP_RECORDING_UID:-$(id -u)}"
export VIVE_TELEOP_RECORDING_GID="${VIVE_TELEOP_RECORDING_GID:-$(id -g)}"
export WEBRTC_PUBLIC_TURN_URLS="${WEBRTC_PUBLIC_TURN_URLS:-turn:${WEBRTC_HOST_IP}:3478?transport=udp,turn:${WEBRTC_HOST_IP}:3478?transport=tcp}"
export WEBRTC_TURN_URLS="${WEBRTC_TURN_URLS:-turn:127.0.0.1:3478?transport=udp,turn:127.0.0.1:3478?transport=tcp}"

case "$ROS2_DDS_ALLOW_MULTICAST" in
  true|false) ;;
  *)
    printf 'ROS2_DDS_ALLOW_MULTICAST must be true or false, got: %s\n' \
      "$ROS2_DDS_ALLOW_MULTICAST" >&2
    exit 1
    ;;
esac

case "$VIVE_TELEOP_RECORD_DATASET" in
  0|1) ;;
  *)
    printf 'VIVE_TELEOP_RECORD_DATASET must be 0 or 1, got: %s\n' \
      "$VIVE_TELEOP_RECORD_DATASET" >&2
    exit 1
    ;;
esac

python3 - "$ROBOT_IP" "$ROS2_DDS_INTERFACE" <<'PY'
import ipaddress
import sys

for label, value in zip(("ROBOT_IP", "ROS2_DDS_INTERFACE"), sys.argv[1:]):
    try:
        ipaddress.ip_address(value)
    except ValueError as error:
        raise SystemExit(f"{label} must be an IP address: {error}") from error
PY

if [[ -z "${CYCLONEDDS_HOST_CONFIG:-}" ]]; then
  dds_config_suffix="$(printf '%s' "$ROS2_DDS_INTERFACE" | tr -c '[:alnum:]_.-' '_')"
  umask 077
  CYCLONEDDS_HOST_CONFIG="$(
    mktemp "/tmp/vive_teleop_cyclonedds_${dds_config_suffix}_XXXXXX.xml"
  )"
  export CYCLONEDDS_HOST_CONFIG
fi

if [[ -L "$CYCLONEDDS_HOST_CONFIG" ]] || \
  [[ -e "$CYCLONEDDS_HOST_CONFIG" && ! -f "$CYCLONEDDS_HOST_CONFIG" ]]; then
  dds_config_suffix="$(printf '%s' "$ROS2_DDS_INTERFACE" | tr -c '[:alnum:]_.-' '_')"
  fallback_config="$(mktemp "/tmp/vive_teleop_cyclonedds_${dds_config_suffix}_XXXXXX.xml")"
  printf 'CycloneDDS config path is not a regular file: %s; using %s\n' \
    "$CYCLONEDDS_HOST_CONFIG" "$fallback_config" >&2
  export CYCLONEDDS_HOST_CONFIG="$fallback_config"
elif [[ -f "$CYCLONEDDS_HOST_CONFIG" && ! -w "$CYCLONEDDS_HOST_CONFIG" ]]; then
  dds_config_suffix="$(printf '%s' "$ROS2_DDS_INTERFACE" | tr -c '[:alnum:]_.-' '_')"
  fallback_config="$(mktemp "/tmp/vive_teleop_cyclonedds_${dds_config_suffix}_XXXXXX.xml")"
  printf 'CycloneDDS config path is not writable: %s; using %s\n' \
    "$CYCLONEDDS_HOST_CONFIG" "$fallback_config" >&2
  export CYCLONEDDS_HOST_CONFIG="$fallback_config"
fi

mkdir -p "$(dirname -- "$CYCLONEDDS_HOST_CONFIG")"
umask 077

cat > "$CYCLONEDDS_HOST_CONFIG" <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cyclonedds.io/xml">
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface address="${ROS2_DDS_INTERFACE}"/>
      </Interfaces>
      <AllowMulticast>${ROS2_DDS_ALLOW_MULTICAST}</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>
      <Peers>
        <Peer Address="${ROBOT_IP}"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF

printf 'Using WebRTC host IP: %s\n' "$WEBRTC_HOST_IP"
printf 'Using ROS2 robot IP: %s\n' "$ROBOT_IP"
printf 'Using field host IP for ROS2 robot access: %s\n' "$ROS_FIELD_HOST_IP"
printf 'Using DDS interface for direct robot discovery: %s\n' "$ROS2_DDS_INTERFACE"
printf 'Using ROS2 domain ID: %s\n' "$ROS_DOMAIN_ID"
printf 'CycloneDDS config: %s\n' "$CYCLONEDDS_HOST_CONFIG"
printf 'Signaling URL: http://%s:8088\n' "$WEBRTC_HOST_IP"
printf 'Client config URL: http://%s:8088/config\n' "$WEBRTC_HOST_IP"
printf 'Client TURN URLs: %s\n' "$WEBRTC_PUBLIC_TURN_URLS"
printf 'Server TURN URLs: %s\n' "$WEBRTC_TURN_URLS"

services=(webrtc_server_wifi moveit_server_wifi coturn_wifi)
if [[ "$VIVE_TELEOP_RECORD_DATASET" == "1" ]]; then
  if [[ -z "${VIVE_TELEOP_SESSION_ID:-}" ]]; then
    VIVE_TELEOP_SESSION_ID="$(date -u +%Y%m%dT%H%M%SZ)-$RANDOM"
    export VIVE_TELEOP_SESSION_ID
  fi
  mkdir -p "$VIVE_TELEOP_RECORDING_ROOT"
  if [[ ! -w "$VIVE_TELEOP_RECORDING_ROOT" ]]; then
    printf 'Recording root is not writable: %s\n' "$VIVE_TELEOP_RECORDING_ROOT" >&2
    exit 1
  fi
  printf 'Dataset recording session: %s\n' "$VIVE_TELEOP_SESSION_ID"
  printf 'Dataset recording root: %s\n' "$VIVE_TELEOP_RECORDING_ROOT"
  services+=(data_recorder_wifi)
fi

cd "$repo_dir"
docker compose -f docker-compose.yml -f docker-compose.wifi.yml stop \
  webrtc_server moveit_server data_recorder coturn >/dev/null 2>&1 || true

exec docker compose -f docker-compose.yml -f docker-compose.wifi.yml up --build --remove-orphans "$@" \
  "${services[@]}"
