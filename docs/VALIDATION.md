# Staged validation (Vive → ROS 2 → bridge → ROS 1 → TIAGO)

Follow in order. Keep **`start_enabled:=false`** until step 4 passes.

## 1. SteamVR / hardware

- Base stations powered and paired; HMD shows as tracked in SteamVR.
- On Linux, confirm USB permissions (udev) if devices do not appear.

## 2. ROS 2 head pose only (teleop PC)

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash   # after colcon build of vive_head_pose
ros2 launch vive_head_pose vive_head_pose.launch.py
```

In another terminal:

```bash
ros2 topic echo /vive/head_pose --no-arr
```

Pose fields should change when you move your head. If `VR_Init` fails, SteamVR is not running or `libopenvr_api` cannot load the SteamVR runtime.

## 3. ros1_bridge + ROS 1 visibility

With `dynamic_bridge` running and `ROS_MASTER_URI` pointing at the TIAGO master:

```bash
export ROS_MASTER_URI=http://<robot-or-master-host>:11311
rostopic echo /vive/head_pose
```

If the topic is missing, check DDS domain (`ROS_DOMAIN_ID`), firewall, and that a ROS 2 publisher is active so `dynamic_bridge` can match types.

## 4. Mapper dry-run (no motion)

On a ROS 1 machine connected to the same master:

```bash
roslaunch tiago_head_from_vive tiago_head_mapper.launch start_enabled:=false
rostopic echo /head_controller/command
```

Toggle enable only after verifying limits:

```bash
rostopic pub /tiago_head_mapper/enabled std_msgs/Bool "data: true" -1
```

Watch proposed joint positions; **emergency stop** the robot if anything unexpected appears.

Confirm the command topic type on your TIAGO image before enabling:

```bash
rostopic type /head_controller/command
rostopic info /head_controller/command
```

If it is not `std_msgs/Float64MultiArray`, switch mapper output mode/topic accordingly.

## 5. Calibration

With the operator looking at a neutral pose (facing the robot / desired “straight ahead”):

```bash
rosservice call /tiago_head_mapper/calibrate
```

Move head back to neutral; the next `PoseStamped` becomes the reference.

## 6. On-robot test

- Reduce motion: lower `limit_yaw_rad`, `limit_pitch_rad`, increase `deadband_rad`, slower `trajectory_time_s`.
- Stand clear; enable teleop briefly; verify head follows smoothly without oscillation.
