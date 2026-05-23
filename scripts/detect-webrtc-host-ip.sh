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

route_target="${WEBRTC_ROUTE_TARGET:-1.1.1.1}"
ip -4 route get "$route_target" | awk '{
  for (i = 1; i <= NF; i++) {
    if ($i == "src") {
      print $(i + 1)
      exit
    }
  }
}'
