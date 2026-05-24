#!/usr/bin/env bash
set -euo pipefail

if [[ -n "${WEBRTC_HOST_IP:-}" ]]; then
  printf '%s\n' "$WEBRTC_HOST_IP"
  exit 0
fi

if [[ -n "${WEBRTC_NIC:-}" ]]; then
  ip -4 -o addr show dev "$WEBRTC_NIC" scope global | awk '{split($4, addr, "/"); print addr[1]; exit}'
  exit 0
fi

for candidate in /sys/class/net/*; do
  iface="$(basename "$candidate")"
  if [[ -d "$candidate/wireless" || "$iface" == wl* || "$iface" == wlan* ]]; then
    ip_addr="$(ip -4 -o addr show dev "$iface" scope global | awk '{split($4, addr, "/"); print addr[1]; exit}')"
    if [[ -n "$ip_addr" ]]; then
      printf '%s\n' "$ip_addr"
      exit 0
    fi
  fi
done

route_target="${WEBRTC_ROUTE_TARGET:-1.1.1.1}"
ip -4 route get "$route_target" | awk '{
  for (i = 1; i <= NF; i++) {
    if ($i == "src") {
      print $(i + 1)
      exit
    }
  }
}'
