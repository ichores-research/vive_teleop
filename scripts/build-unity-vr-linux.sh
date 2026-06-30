#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
project_dir="${repo_root}/unity-vr-headset"
project_version="$(
  sed -n 's/^m_EditorVersion: //p' \
    "${project_dir}/ProjectSettings/ProjectVersion.txt"
)"
unity_editor="${UNITY_EDITOR:-${HOME}/Unity/Hub/Editor/${project_version}/Editor/Unity}"
output_path="${1:-${project_dir}/Builds/Linux/vive-teleop}"

if [[ ! -x "${unity_editor}" ]]; then
  printf 'Unity editor not found: %s\n' "${unity_editor}" >&2
  printf 'Set UNITY_EDITOR to the Unity executable path.\n' >&2
  exit 1
fi

mkdir -p "$(dirname "${output_path}")"

"${unity_editor}" \
  -batchmode \
  -nographics \
  -quit \
  -projectPath "${project_dir}" \
  -buildLinux64Player "${output_path}" \
  -logFile -

if [[ ! -x "${output_path}" ]]; then
  printf 'Unity reported success, but the player is missing: %s\n' \
    "${output_path}" >&2
  exit 1
fi

touch "${output_path}.build-stamp"
printf 'Linux player built: %s\n' "${output_path}"
