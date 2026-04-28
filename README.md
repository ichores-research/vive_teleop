# vive_teleop

ROS **ROS 2 → `ros1_bridge` → ROS 1** pipeline for **HTC Vive Pro** head tracking and **PAL TIAGO** head motion mirroring.

## Contents

| Path | Purpose |
|------|---------|
| [`bridge/Dockerfile`](bridge/Dockerfile) | Ubuntu 20.04 image: Noetic + Foxy, `ros1_bridge`, OpenVR, `vive_head_pose` (ROS 2), `tiago_head_from_vive` (ROS 1) |
| [`docker-compose.yml`](docker-compose.yml) | Runs `dynamic_bridge --bridge-all-topics` toward the TIAGO ROS master |
| [`ros2_ws/src/vive_head_pose`](ros2_ws/src/vive_head_pose) | OpenVR node publishing `geometry_msgs/PoseStamped` on `/vive/head_pose` |
| [`ros1_ws/src/tiago_head_from_vive`](ros1_ws/src/tiago_head_from_vive) | Maps pose → head commands (default `std_msgs/Float64MultiArray` on `/head_controller/command`, optional `JointTrajectory`) |
| [`docs/VR_RUNTIME.md`](docs/VR_RUNTIME.md) | SteamVR, lighthouses, DDS notes |
| [`docs/VALIDATION.md`](docs/VALIDATION.md) | Staged checks before enabling motion |

## Prerequisites

- **Lighthouse base stations** must be set up. Headset USB alone does **not** provide full 6-DoF tracking.
- **Steam + SteamVR** on the teleop PC; room-scale calibration completed.
- Network reachability from the teleop PC to the TIAGO **`roscore`** (`ROS_MASTER_URI`).

## Docker: build

From this directory (build context **must** be repo root so `ros2_ws` / `ros1_ws` are included):

```bash
docker compose build
```

## Docker: run `ros1_bridge`

Edit `ROS_MASTER_URI` / IP in [`docker-compose.yml`](docker-compose.yml) for your site. Then:

```bash
docker compose up ros1_bridge
```

The container sources Noetic, Foxy, `/bridge_ws/install`, `/ros2_vive_ws/install`, and `/catkin_ws/devel` (see [`bridge/ros_entrypoint.sh`](bridge/ros_entrypoint.sh)).

## ROS 2: publish Vive head pose (recommended: host PC)

**Recommended:** run SteamVR and `vive_head_pose` on the **same machine** as the headset (GPU/USB/runtime). Inside Docker, SteamVR is usually impractical unless you mount devices and the SteamVR tree explicitly.

On the host (after building `ros2_ws` with Foxy):

```bash
cd ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select vive_head_pose
source install/setup.bash
export ROS_DOMAIN_ID=0   # match bridge / fleet
ros2 launch vive_head_pose vive_head_pose.launch.py
```

Published topics:

- `/vive/head_pose` (`geometry_msgs/PoseStamped`) — parameter `pose_topic`
- TF (optional): `world_frame` → `frame_id` (default `vive_world` → `vive_hmd`) when `publish_tf:=true`

## ROS 1: TIAGO head mapper

Run on any ROS 1 machine that shares the TIAGO master (often the robot PC or your laptop with `ROS_MASTER_URI` set):

```bash
cd ros1_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
roslaunch tiago_head_from_vive tiago_head_mapper.launch start_enabled:=false
```

**Safety:** default `start_enabled:=false`. Enable only after validation:

```bash
rostopic pub /tiago_head_mapper/enabled std_msgs/Bool "data: true" -1
```

Recalibrate neutral pose:

```bash
rosservice call /tiago_head_mapper/calibrate
```

Then return your head to neutral; the next pose sample becomes zero yaw/pitch.

### Controller topic compatibility

Default mapper output is **`std_msgs/Float64MultiArray`** to `/head_controller/command`.
The payload is `[head_1_joint, head_2_joint]` positions in radians.

If your stack uses trajectories instead, launch with:

```bash
roslaunch tiago_head_from_vive tiago_head_mapper.launch \
  output_mode:=joint_trajectory \
  output_command_topic:=/head_controller/joint_trajectory
```

Before enabling motion on a new robot image, detect the real command interface:

```bash
rostopic info /head_controller/command
rostopic type /head_controller/command
```

## Parameters (mapper highlights)

See [`launch/tiago_head_mapper.launch`](ros1_ws/src/tiago_head_from_vive/launch/tiago_head_mapper.launch): joint names (`head_1_joint`, `head_2_joint`), yaw/pitch limits, smoothing, deadband, sign flips.

## Validation

Follow [`docs/VALIDATION.md`](docs/VALIDATION.md).
