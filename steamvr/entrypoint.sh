#!/usr/bin/env bash
set -e

export USER=steam
export HOME=/home/steam
export XDG_RUNTIME_DIR=/tmp/runtime-steam

mkdir -p $XDG_RUNTIME_DIR
chmod 700 $XDG_RUNTIME_DIR

# ── Core services ───────────────────────────────────────────────
dbus-daemon --system --fork

udevd --daemon
udevadm trigger
udevadm settle

chmod -R 777 /dev/bus/usb || true

# ── Display ─────────────────────────────────────────────────────
if [ -z "$DISPLAY" ]; then
    Xvfb :99 -screen 0 1920x1080x24 &
    export DISPLAY=:99
fi

x11vnc -display $DISPLAY -nopw -forever -shared &

# ── Wine + Vive lens server ─────────────────────────────────────
export WINEPREFIX=/home/steam/.wine
mkdir -p $WINEPREFIX

wine64 /opt/vive/driver/viveVR/lens-server/lens-server.exe &

sleep 2

# ── Monado ──────────────────────────────────────────────────────
monado-service &

sleep 2

echo "Runtime ready"

# Keep container alive
tail -f /dev/null