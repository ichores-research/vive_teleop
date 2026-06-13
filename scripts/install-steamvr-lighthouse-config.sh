#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd -- "$script_dir/.." && pwd)"
source_file="${1:-${VIVE_LIGHTHOUSE_CONFIG_SOURCE:-${repo_dir}/config/steamvr/lighthouse/lighthousedb.json}}"
steam_config_dir="${STEAMVR_CONFIG_DIR:-${HOME}/.local/share/Steam/config}"
destination_dir="${steam_config_dir}/lighthouse"
destination_file="${destination_dir}/lighthousedb.json"

if [[ ! -f "$source_file" ]]; then
  printf 'SteamVR lighthouse source does not exist: %s\n' "$source_file" >&2
  exit 1
fi

python3 - "$source_file" <<'PY'
import json
import sys

path = sys.argv[1]
with open(path, encoding="utf-8") as stream:
    data = json.load(stream)

required = ("base_stations", "known_universes", "revision")
missing = [key for key in required if key not in data]
if missing:
    raise SystemExit(
        f"{path}: missing required lighthouse keys: {', '.join(missing)}"
    )
if not data["base_stations"] or not data["known_universes"]:
    raise SystemExit(f"{path}: lighthouse calibration is empty")
PY

mkdir -p "$destination_dir"

if [[ -f "$destination_file" ]] && cmp -s "$source_file" "$destination_file"; then
  printf 'SteamVR lighthouse configuration is already current: %s\n' \
    "$destination_file"
  exit 0
fi

if [[ "${VIVE_LIGHTHOUSE_ALLOW_RUNNING:-0}" != "1" ]] \
  && pgrep -f '/SteamVR/.*/vrserver' >/dev/null 2>&1; then
  printf 'SteamVR is running. Stop SteamVR before replacing %s\n' \
    "$destination_file" >&2
  exit 1
fi

if [[ -f "$destination_file" ]]; then
  backup_file="${destination_file}.before-vive-teleop-$(date +%Y%m%d-%H%M%S)"
  cp --preserve=mode,timestamps "$destination_file" "$backup_file"
  printf 'Backed up existing lighthouse configuration to %s\n' "$backup_file"
fi

install -m 600 "$source_file" "$destination_file"
printf 'Installed SteamVR lighthouse configuration: %s\n' "$destination_file"
