#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
project_dir="${repo_root}/unity-vr-headset"
project_version="$(
  sed -n 's/^m_EditorVersion: //p' \
    "${project_dir}/ProjectSettings/ProjectVersion.txt"
)"
unity_editor="${UNITY_EDITOR:-${HOME}/Unity/Hub/Editor/${project_version}/Editor/Unity}"
steamvr_dir="${STEAMVR_DIR:-${HOME}/.local/share/Steam/steamapps/common/SteamVR}"
openvr_provider="$(
  find "${project_dir}/Library/PackageCache" \
    -path '*/com.valvesoftware.unity.openvr@*/Runtime/x64/libXRSDKOpenVR.so' \
    -print -quit 2>/dev/null || true
)"
webrtc_library="$(
  find "${project_dir}/Library/PackageCache" \
    -path '*/com.unity.webrtc@*/Runtime/Plugins/x86_64/libwebrtc.so' \
    -print -quit 2>/dev/null || true
)"

failures=0

check_file() {
  local label="$1"
  local path="$2"
  if [[ -e "${path}" ]]; then
    printf 'OK   %s: %s\n' "${label}" "${path}"
  else
    printf 'FAIL %s: %s\n' "${label}" "${path}" >&2
    failures=$((failures + 1))
  fi
}

check_command() {
  local command_name="$1"
  if command -v "${command_name}" >/dev/null 2>&1; then
    printf 'OK   command: %s\n' "$(command -v "${command_name}")"
  else
    printf 'FAIL command not found: %s\n' "${command_name}" >&2
    failures=$((failures + 1))
  fi
}

check_file "Unity ${project_version}" "${unity_editor}"
check_file "SteamVR server" "${steamvr_dir}/bin/linux64/vrserver"
check_file "SteamVR compositor" "${steamvr_dir}/bin/linux64/vrcompositor-launcher"
check_file \
  "OpenVR Linux provider" \
  "${openvr_provider:-${project_dir}/Library/PackageCache/<openvr>/Runtime/x64/libXRSDKOpenVR.so}"
check_file \
  "Unity WebRTC Linux library" \
  "${webrtc_library:-${project_dir}/Library/PackageCache/<webrtc>/Runtime/Plugins/x86_64/libwebrtc.so}"
check_command "steam"
check_command "vulkaninfo"

if command -v vulkaninfo >/dev/null 2>&1; then
  if vulkaninfo --summary >/dev/null 2>&1; then
    printf 'OK   Vulkan runtime responds\n'
  else
    printf 'FAIL Vulkan runtime did not initialize\n' >&2
    failures=$((failures + 1))
  fi
fi

if [[ "${failures}" -ne 0 ]]; then
  printf '\nLinux VR preflight failed with %d issue(s).\n' "${failures}" >&2
  exit 1
fi

printf '\nLinux VR preflight passed. Launch SteamVR before entering Play mode.\n'
