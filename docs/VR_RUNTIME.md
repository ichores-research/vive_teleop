# Vive Pro tracking prerequisites (SteamVR / OpenVR)

This stack reads headset pose through **OpenVR**, which requires the **SteamVR** runtime on the same machine as `vive_head_pose`.

## Required hardware

- **Base stations (Lighthouses)** — Vive tracking is optical; without base stations you will not get stable 6-DoF pose. Only the cable-connected headset is not enough.
- Headset tracking must show as **OK** in the SteamVR status window.

## Software on the teleop PC

1. Install **Steam** (native `.deb`, not Flatpak/Snap if you want fewer USB/runtime issues).
2. Install **SteamVR** from the Steam library.
3. Complete **room setup** and ensure the HMD is tracked (green in SteamVR).
4. Install Python OpenVR bindings used by the ROS 2 nodes:

```bash
python3 -m pip install --user openvr
```

## Linux udev (USB devices)

If the headset or receivers are not detected, add Valve’s udev rules (see SteamVR / SteamVR for Linux documentation) and reload rules:

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## ROS 2 discovery with `ros1_bridge`

The bridge container uses **Cyclone DDS** (`rmw_cyclonedds_cpp`). On the host running `vive_head_pose`, use the **same** `ROS_DOMAIN_ID` as the bridge (default `0` unless you changed it) so ROS 2 topics are visible to `dynamic_bridge`.

If discovery fails across subnets/VLANs, align `CYCLONEDDS_URI` or use `ROS_LOCALHOST_ONLY=0` as appropriate for your network.

## Where to run `vive_head_pose`

**Recommended:** run `vive_head_pose` on the **same PC as SteamVR** (GPU + USB + runtime). Running SteamVR inside Docker is possible but fragile; the provided Docker image mainly supports CI/build and advanced users who mount SteamVR paths and devices explicitly.

See [README.md](../README.md) for compose profiles and host-run examples.

## Quick stability verification

After launching `vive_head_pose`, confirm both signal presence and update rate:

```bash
ros2 topic echo /vive/head_pose --no-arr
ros2 topic hz /vive/head_pose
```

A steady publish rate (near your `rate_hz` parameter) and smoothly changing poses indicate tracking/runtime are healthy.

## One-command headset diagnostics

From repo root, run:

```bash
bash check_vive_headset.sh
```

Optional arguments:

```bash
bash check_vive_headset.sh <sample_count> <sample_sleep_ms>
```

The script builds `vive_head_pose` if needed, then runs a diagnostic executable that:
- checks whether OpenVR initializes,
- detects the HMD,
- prints model/manufacturer/serial/tracking system/battery (if available),
- samples pose validity and position.
