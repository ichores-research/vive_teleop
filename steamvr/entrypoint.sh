#!/bin/bash
set -e

echo "[steamvr] Starting virtual display on :99 ..."
Xvfb :99 -screen 0 1920x1080x24 +extension GLX +render -noreset &
XVFB_PID=$!
sleep 2

# Optional: expose the virtual desktop via VNC for debugging
x11vnc -display :99 -nopw -forever -quiet &

echo "[steamvr] Starting PulseAudio ..."
pulseaudio --start --exit-idle-time=-1 || true

echo "[steamvr] Launching SteamVR ..."
export DISPLAY=:99
export VR_OVERRIDE=/opt/steamvr

# vrserver is the SteamVR compositor process; running it directly
# avoids needing a full Steam UI session
exec /opt/steamvr/bin/linux64/vrserver \
    --keepalive \
    2>&1 | tee /var/log/steamvr.log