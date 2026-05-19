#!/bin/bash
set +e

# ── 1. Clean old X11 locks ────────────────────────────────────────────────────
echo "[steamvr] Cleaning up old X11 locks..."
rm -f /tmp/.X99-lock /tmp/.X11-unix/X99

# ── 2. Virtual display ────────────────────────────────────────────────────────
echo "[steamvr] Starting virtual display on :99 ..."
Xvfb :99 -screen 0 1920x1080x24 +extension GLX +render -noreset &
XVFB_PID=$!

for i in $(seq 1 20); do
    [ -e /tmp/.X11-unix/X99 ] && break
    sleep 0.5
done
export DISPLAY=:99

# ── 3. VNC ────────────────────────────────────────────────────────────────────
x11vnc -display :99 -nopw -forever -quiet &

# ── 4. DBus (required by PipeWire and SteamVR) ───────────────────────────────
echo "[steamvr] Starting dbus..."
export XDG_RUNTIME_DIR=/tmp/runtime-root
mkdir -p "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR"

eval $(dbus-launch --sh-syntax)
export DBUS_SESSION_BUS_ADDRESS

# ── 4b. udev (helps SteamVR enumerate USB/HID devices) ────────────────────────
echo "[steamvr] Starting udev..."
mkdir -p /run/udev
# If /run/udev is bind-mounted from the host (as in docker-compose.yml),
# the host udevd is already providing the control socket. Starting a second
# udevd in-container will fail with "Address already in use".
if [ -S /run/udev/control ]; then
    echo "[steamvr] Detected existing /run/udev/control; skipping in-container udevd."
else
    (/lib/systemd/systemd-udevd --daemon --resolve-names=never) || true
    udevadm control --reload-rules || true
    udevadm trigger || true
    udevadm settle || true
fi

# ── 5. PipeWire audio stack ───────────────────────────────────────────────────
echo "[steamvr] Starting PipeWire..."
export PIPEWIRE_RUNTIME_DIR="$XDG_RUNTIME_DIR"

pipewire &
sleep 0.5
wireplumber &
pipewire-pulse &
sleep 1

# ── 6. PipeWire config ────────────────────────────────────────────────────────
# Don't override PIPEWIRE_CONFIG_DIR unless providing a complete config tree.
# Setting it to a minimal stub can prevent PipeWire from loading protocol/modules.
unset PIPEWIRE_CONFIG_DIR

# ── 7. SteamVR settings — enable null driver ──────────────────────────────────
echo "[steamvr] Generating VR Path Registry..."
mkdir -p /root/openvr/logs
mkdir -p /root/.config/openvr

cat <<EOF > /root/.config/openvr/openvrpaths.vrpath
{
  "config": [ "/root/openvr" ],
  "external_drivers": [ ],
  "log":    [ "/root/openvr/logs" ],
  "runtime": [ "/opt/steamvr" ],
  "version": 1
}
EOF
cp /root/.config/openvr/openvrpaths.vrpath /root/openvr/openvrpaths.vrpath

cat <<EOF > /root/.config/openvr/steamvr.vrsettings
{
   "steamvr" : {
      "activateMultipleDrivers" : false,
      "requireHmd" : true
   }
}
EOF

# ── 8. SDK symlinks (use writable path, not the ro bind mount) ───────────────
echo "[steamvr] Creating SDK symlinks..."
STEAM_LIB="/root/.local/share/Steam/linux64"

# Use /tmp for shims (container-writable). SteamVR may still try ~/.steam/sdk64;
# if the host bind mount doesn't include it, we'll warn below.
SDK64=/tmp/steam-sdk64
SDK32=/tmp/steam-sdk32
mkdir -p "$SDK64" "$SDK32"

ln -sf "${STEAM_LIB}/steamclient.so"  "$SDK64/steamclient.so"
ln -sf "${STEAM_LIB}/steamclient.so"  "$SDK32/steamclient.so"
ln -sf "${STEAM_LIB}/steamservice.so" "$SDK64/steamservice.so" || true

# Also populate the canonical paths SteamVR probes (may be a writable overlay mount).
mkdir -p /root/.steam/sdk64 /root/.steam/sdk32 2>/dev/null || true
ln -sf "${STEAM_LIB}/steamclient.so" /root/.steam/sdk64/steamclient.so 2>/dev/null || true
ln -sf "${STEAM_LIB}/steamclient.so" /root/.steam/sdk32/steamclient.so 2>/dev/null || true

# Point SteamVR at the new locations
export STEAM_SDK64="$SDK64"
export LD_LIBRARY_PATH="${SDK64}:${SDK32}:${STEAM_LIB}:\
/opt/steamvr/bin/linux64:\
/opt/steamvr/bin/vrclient/linux64:\
${LD_LIBRARY_PATH:-}"

# If SteamVR still insists on loading from /root/.steam/sdk64, make that visible
# via the host bind mount (preferred), or you'll see dlopen failures.
if [ ! -e /root/.steam/sdk64/steamclient.so ]; then
    echo "[steamvr] WARNING: /root/.steam/sdk64/steamclient.so not found (host Steam install may be missing sdk64)."
fi

# ── 9. steam-runtime shim ────────────────────────────────────────────────────
if ! command -v steam-runtime-launch-client &>/dev/null; then
    echo "[steamvr] WARNING: steam-runtime-launch-client not found, installing shim..."
    cat <<'SHIM' > /usr/local/bin/steam-runtime-launch-client
#!/bin/sh
# Minimal compat shim for SteamVR scripts that call:
#   steam-runtime-launch-client --alongside-steam -- <cmd> <args...>
# We ignore launcher flags and execute the command after `--`.
while [ $# -gt 0 ]; do
  case "$1" in
    --) shift; break ;;
    *) shift ;;
  esac
done
exec "$@"
SHIM
    chmod +x /usr/local/bin/steam-runtime-launch-client
fi

# ── 10. Environment ───────────────────────────────────────────────────────────
export VR_OVERRIDE=/opt/steamvr
export VR_CONFIG_PATH=/root/openvr
export VR_LOG_PATH=/root/openvr/logs
export XDG_CONFIG_HOME=/root/.config
export STEAMVR_LD_ORIGINAL_PRELOAD=""
# Pretend the "scout" runtime is already active so vrsetup.sh doesn't try to exec
# ~/.steam/bin/steam-runtime/run.sh (not present in this container).
export STEAM_RUNTIME=1
# vrstartup.sh wants a writable Steam base for logs; /root/.steam is bind-mounted ro.
export STEAM_BASE_FOLDER=/tmp/steam-base
mkdir -p "${STEAM_BASE_FOLDER}/logs" || true

# Keep the earlier LD_LIBRARY_PATH (which prioritizes /tmp/steam-sdk* shims).
# Only add ~/.steam/sdk64 if it exists (it may be a read-only host bind mount).
if [ -d /root/.steam/sdk64 ]; then
    export LD_LIBRARY_PATH="/root/.steam/sdk64:${LD_LIBRARY_PATH:-}"
fi

# ── 11. Launch ────────────────────────────────────────────────────────────────
echo "[steamvr] Launching SteamVR..."

cleanup() {
    echo "[steamvr] Shutting down..."
    kill "$XVFB_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [ -x /opt/steamvr/bin/vrstartup.sh ]; then
    echo "[steamvr] Using vrstartup.sh to start full stack..."
    # We are running inside a container, not under Steam Linux Runtime (Pressure Vessel).
    # Allow vrstartup.sh to proceed without the runtime environment.
    /opt/steamvr/bin/vrstartup.sh --valve-skip-runtime-safety 2>&1 | tee /root/openvr/logs/vrstartup.txt
    echo "[steamvr] vrstartup.sh exited: ${PIPESTATUS[0]}"
else
    echo "[steamvr] vrstartup.sh not found; falling back to vrserver..."
    /opt/steamvr/bin/linux64/vrserver --keepalive 2>&1 | tee /root/openvr/logs/vrserver.txt
    echo "[steamvr] vrserver exited: ${PIPESTATUS[0]}"
fi

echo "[steamvr] Keeping container alive for debugging..."
sleep infinity