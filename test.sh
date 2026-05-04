echo "=== STEAM ROOT ==="
ls -ld /home/robot/.steam
ls -ld /home/robot/.local/share/Steam

echo "=== STEAM RUNTIME ==="
ls -l /home/robot/.steam/steam/ubuntu12_64/steamclient.so

echo "=== STEAMVR DIR ==="
ls -ld /home/robot/.local/share/Steam/steamapps/common/SteamVR

echo "=== STEAMVR BIN ==="
ls -l /home/robot/.local/share/Steam/steamapps/common/SteamVR/bin/linux64/

echo "=== FIND vrserver ==="
find /home/robot/.local/share/Steam -name vrserver 2>/dev/null