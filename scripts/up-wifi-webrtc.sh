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
export ROS2_DDS_INTERFACE="${ROS2_DDS_INTERFACE:-lo}"
export ROS2_DDS_ALLOW_MULTICAST="${ROS2_DDS_ALLOW_MULTICAST:-true}"
export TURN_USER="${TURN_USER:-dummy}"
export TURN_PASSWORD="${TURN_PASSWORD:-dummy}"
export WEBRTC_PUBLIC_TURN_URLS="${WEBRTC_PUBLIC_TURN_URLS:-turn:${WEBRTC_HOST_IP}:3478?transport=udp,turn:${WEBRTC_HOST_IP}:3478?transport=tcp}"
export WEBRTC_TURN_URLS="${WEBRTC_TURN_URLS:-turn:127.0.0.1:3478?transport=udp,turn:127.0.0.1:3478?transport=tcp}"
export CYCLONEDDS_HOST_CONFIG="${CYCLONEDDS_HOST_CONFIG:-/tmp/vive_teleop_cyclonedds_${ROS2_DDS_INTERFACE}_host.xml}"

if [[ -e "$CYCLONEDDS_HOST_CONFIG" && ! -f "$CYCLONEDDS_HOST_CONFIG" ]]; then
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

cat > "$CYCLONEDDS_HOST_CONFIG" <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cyclonedds.io/xml">
  <Domain Id="any">
    <General>
      <NetworkInterfaceAddress>${ROS2_DDS_INTERFACE}</NetworkInterfaceAddress>
      <AllowMulticast>${ROS2_DDS_ALLOW_MULTICAST}</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF

printf 'Using WebRTC host IP: %s\n' "$WEBRTC_HOST_IP"
printf 'Using field host IP for ROS1 robot access: %s\n' "$ROS_FIELD_HOST_IP"
printf 'Using DDS interface for local ROS2 bridge/app discovery: %s\n' "$ROS2_DDS_INTERFACE"
printf 'CycloneDDS config: %s\n' "$CYCLONEDDS_HOST_CONFIG"
printf 'Signaling URL: http://%s:8088\n' "$WEBRTC_HOST_IP"
printf 'Client config URL: http://%s:8088/config\n' "$WEBRTC_HOST_IP"
printf 'Client TURN URLs: %s\n' "$WEBRTC_PUBLIC_TURN_URLS"
printf 'Server TURN URLs: %s\n' "$WEBRTC_TURN_URLS"

cd "$repo_dir"
docker compose -f docker-compose.yml -f docker-compose.wifi.yml stop \
  ros1_bridge ros2_app moveit_server coturn >/dev/null 2>&1 || true

exec docker compose -f docker-compose.yml -f docker-compose.wifi.yml up --build "$@" \
  ros1_bridge_wifi ros2_app_wifi moveit_server_wifi coturn_wifi
