#!/bin/bash
set -e

# 1. Clean up old locks
echo "[steamvr] Cleaning up old X11 locks..."
rm -f /tmp/.X99-lock /tmp/.X11-unix/X99

# 2. Start virtual display
echo "[steamvr] Starting virtual display on :99 ..."
Xvfb :99 -screen 0 1920x1080x24 +extension GLX +render -noreset &
XVFB_PID=$!
sleep 2

# 3. VNC
x11vnc -display :99 -nopw -forever -quiet &

# 4. Audio
echo "[steamvr] Starting PulseAudio ..."
pulseaudio --start --exit-idle-time=-1 || true

# 5. Fix Paths & SDK
echo "[steamvr] Generating VR Path Registry..."
# Match the path SteamVR is actually looking for based on your logs
mkdir -p /etc/openvr
cat <<EOF > /etc/openvr/openvrpaths.vrpath
{
  "config": [ "/etc/openvr" ],
  "external_drivers": null,
  "log": [ "/etc/openvr/logs" ],
  "runtime": [ "/opt/steamvr" ],
  "version": 1
}
EOF

echo "[steamvr] Creating SDK symlinks..."
mkdir -p /root/.steam/sdk64

ln -sf /root/.local/share/Steam/linux64/steamclient.so \
       /root/.steam/sdk64/steamclient.so

# 6. Launch
echo "[steamvr] Launching SteamVR..."
export DISPLAY=:99
export VR_OVERRIDE=/opt/steamvr
export XDG_CONFIG_HOME="/etc"

export LD_LIBRARY_PATH="/root/.local/share/Steam/linux64:/opt/steamvr/bin/linux64:/opt/steamvr/bin/vrclient/linux64:/root/.steam/sdk64:$LD_LIBRARY_PATH"

VRSERVER_PATH="/opt/steamvr/bin/linux64/vrserver"
exec /opt/steamvr/bin/linux64/vrstartup