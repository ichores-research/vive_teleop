#!/usr/bin/env bash
set -euo pipefail

steam_root="${STEAM_ROOT:-$HOME/.local/share/Steam}"
steam_compat_root="${STEAM_COMPAT_ROOT:-$HOME/.steam}"

printf '%s\n' '=== STEAM ROOT ==='
ls -ld "$steam_compat_root" "$steam_root"

printf '%s\n' '=== STEAM RUNTIME ==='
ls -l "$steam_compat_root/steam/ubuntu12_64/steamclient.so"

printf '%s\n' '=== STEAMVR DIR ==='
ls -ld "$steam_root/steamapps/common/SteamVR"

printf '%s\n' '=== STEAMVR BIN ==='
ls -l "$steam_root/steamapps/common/SteamVR/bin/linux64/"

printf '%s\n' '=== FIND vrserver ==='
find "$steam_root" -name vrserver -type f 2>/dev/null
