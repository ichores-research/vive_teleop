#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd -- "$script_dir/.." && pwd)"
host_ip="$("$script_dir/detect-webrtc-host-ip.sh")"
port="${1:-8000}"

printf 'Serving debug client on http://0.0.0.0:%s\n' "$port"
if [[ -n "$host_ip" ]]; then
  printf 'Open from Wi-Fi clients: http://%s:%s\n' "$host_ip" "$port"
fi

cd "$repo_dir"
exec python3 -m http.server "$port" --bind 0.0.0.0
