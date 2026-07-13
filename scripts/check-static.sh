#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd -- "$script_dir/.." && pwd)"

cd "$repo_dir"

docker compose config --quiet
CYCLONEDDS_HOST_CONFIG=/tmp/vive-teleop-ci-cyclonedds.xml \
WEBRTC_TURN_URLS='turn:127.0.0.1:3478?transport=udp' \
WEBRTC_HOST_IP=192.0.2.10 \
WEBRTC_PUBLIC_TURN_URLS='turn:192.0.2.10:3478?transport=udp' \
ROS_FIELD_HOST_IP=192.0.2.20 \
  docker compose -f docker-compose.yml -f docker-compose.wifi.yml \
  config --quiet

while IFS= read -r script; do
  bash -n "$script"
done < <(find scripts -maxdepth 1 -type f -name '*.sh' -print)
bash -n moveit_server/ros_entrypoint.sh
bash -n data_recorder/ros_entrypoint.sh

unexpected_recorded_user_topics="$(
  grep -E '^[[:space:]]+- /vive/' \
    data_recorder/src/vive_dataset_recorder/config/recorder.yaml |
    grep -vE '^[[:space:]]+- /vive/hand_target_active$' || true
)"
if [[ -n "$unexpected_recorded_user_topics" ]]; then
  printf 'Recorder whitelist contains forbidden user input:\n%s\n' \
    "$unexpected_recorded_user_topics" >&2
  exit 1
fi

if command -v shellcheck >/dev/null 2>&1; then
  mapfile -t shell_scripts < <(
    find scripts -maxdepth 1 -type f -name '*.sh' -print
  )
  shellcheck --exclude=SC1090,SC1091,SC2016 \
    moveit_server/ros_entrypoint.sh data_recorder/ros_entrypoint.sh "${shell_scripts[@]}"
fi

python3 - <<'PY'
import ast
import json
from pathlib import Path
import xml.etree.ElementTree as ET

for root in (
    Path("moveit_server/src/vive_moveit_server"),
    Path("webrtc_server/src/image_listener"),
):
    for path in root.rglob("*.py"):
        ast.parse(path.read_text(encoding="utf-8"), filename=str(path))

for path in Path("unity-vr-headset").rglob("*.json"):
    if "Library" not in path.parts:
        with path.open(encoding="utf-8") as stream:
            json.load(stream)

for path in (
    Path("moveit_server/src/vive_moveit_server/package.xml"),
    Path("webrtc_server/src/image_listener/package.xml"),
    Path("data_recorder/src/vive_dataset_recorder/package.xml"),
):
    ET.parse(path)
PY

python3 scripts/check-markdown-links.py

if command -v node >/dev/null 2>&1; then
  node <<'JS'
const fs = require('fs');
const html = fs.readFileSync('index.html', 'utf8');
const match = html.match(/<script>([\s\S]*?)<\/script>/);
if (!match) {
    throw new Error('inline browser script not found');
}
new Function(match[1]);
JS
fi

printf '%s\n' 'Static checks passed.'
