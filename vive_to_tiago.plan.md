---
name: Vive To TIAGO
overview: Add a ROS 2 head-tracking pipeline for the Vive Pro, bridge the resulting topics into ROS 1, and convert that pose into TIAGO head commands. The plan assumes the current `vive_teleop/` repo remains the integration point and treats lighthouse setup as a hard prerequisite for true 6-DoF head position.
todos:
  - id: vr-runtime-prereq
    content: "Verify and document the Vive Pro tracking prerequisite: add lighthouse/base-station setup and choose SteamVR/OpenVR as the initial runtime."
    status: completed
  - id: ros2-head-publisher
    content: Design a ROS 2 package that publishes the Vive headset pose on a standard topic such as `/vive/head_pose` and optional TF frames.
    status: in_progress
  - id: ros1-head-mapper
    content: Design a ROS 1 TIAGO mapper node that converts the bridged head pose into safe head pan/tilt commands.
    status: pending
  - id: bridge-runtime-cleanup
    content: Update the bridge/runtime layout so the Vive ROS 2 node and ros1_bridge run consistently, and fix the current Foxy/Humble entrypoint mismatch.
    status: pending
  - id: validation-safety
    content: Define calibration, smoothing, joint limits, and staged validation from VR runtime to ROS 2 to ROS 1 to TIAGO motion.
    status: pending
isProject: false
---

# Vive Pro Head Pose To TIAGO Plan

## What I found
The current repo only contains ROS bridge infrastructure, not Vive tracking or robot control nodes:

- [/home/biba/Documents/tiago/vive_teleop/docker-compose.yml](/home/biba/Documents/tiago/vive_teleop/docker-compose.yml) starts `ros2 run ros1_bridge dynamic_bridge --bridge-all-topics` against the TIAGO ROS 1 master at `http://10.68.0.1:11311`.
- [/home/biba/Documents/tiago/vive_teleop/bridge/Dockerfile](/home/biba/Documents/tiago/vive_teleop/bridge/Dockerfile) builds a Noetic + Foxy `ros1_bridge` image.
- [/home/biba/Documents/tiago/vive_teleop/bridge/ros_entrypoint.sh](/home/biba/Documents/tiago/vive_teleop/bridge/ros_entrypoint.sh) appears stale because it sources Humble and `/ros1_bridge_ws`, while the Docker build uses Foxy and `/bridge_ws`.

A full head-position pipeline is therefore a new addition, not a small patch to existing teleop logic.

## Important constraint
With `headset_only` and no lighthouse/base stations configured, you should not expect reliable Vive Pro 6-DoF head position. For the robot to copy head motion in position + orientation, the plan must include bringing up a tracked VR runtime first. If you proceed without base stations, at best you may get limited inertial orientation, which is not enough for accurate teleoperation.

## Recommended architecture
Use the simplest practical stack first: SteamVR/OpenVR on the PC, a dedicated ROS 2 node that publishes the headset pose, then a ROS 1-side conversion node that turns pose into TIAGO head commands.

```mermaid
flowchart LR
    vivePro[ViveProHMD] --> steamVr[SteamVR_OpenVR]
    steamVr --> headPoseNode[ros2_head_pose_publisher]
    headPoseNode --> ros2Pose[/vive/head_pose]
    ros2Pose --> bridge[ros1_bridge]
    bridge --> ros1Pose[/vive/head_pose]
    ros1Pose --> headMapper[tiago_head_mapper]
    headMapper --> headCmd[/head_controller/command_or_joint_trajectory]
    headMapper --> tfDebug[/tf_debug_optional]
```

## Implementation plan
1. Add a small ROS 2 package under [/home/biba/Documents/tiago/vive_teleop](/home/biba/Documents/tiago/vive_teleop) for Vive ingestion.
   - Create a node that reads the HMD pose from SteamVR/OpenVR and publishes:
     - `geometry_msgs/msg/PoseStamped` on `/vive/head_pose`
     - optionally `tf2_msgs/msg/TFMessage` or a `TransformStamped` broadcaster for RViz/debugging
   - Prefer standard message types so the existing bridge can forward them without custom message work.
   - Keep the published frame explicit, for example `vive_world -> vive_hmd`.

2. Add a ROS 1 mapper node that converts head pose into TIAGO head pan/tilt.
   - Subscribe to the bridged `/vive/head_pose` topic on ROS 1.
   - Transform the headset orientation into TIAGO-compatible yaw/pitch commands.
   - Publish either:
     - `trajectory_msgs/JointTrajectory` to `/head_controller/joint_trajectory`, or
     - the controller command expected by your TIAGO image, commonly `/head_controller/command` with `head_1_joint` and `head_2_joint`.
   - Add limits, rate limiting, and smoothing so headset jitter does not produce unsafe motion.

3. Update the container/runtime layout so both the bridge and the new publisher can run repeatably.
   - Extend [/home/biba/Documents/tiago/vive_teleop/bridge/Dockerfile](/home/biba/Documents/tiago/vive_teleop/bridge/Dockerfile) or add a sibling service image for the ROS 2 Vive node.
   - Update [/home/biba/Documents/tiago/vive_teleop/docker-compose.yml](/home/biba/Documents/tiago/vive_teleop/docker-compose.yml) so the ROS 2 publisher and the bridge share the same ROS 2 environment.
   - Either fix or remove the mismatch in [/home/biba/Documents/tiago/vive_teleop/bridge/ros_entrypoint.sh](/home/biba/Documents/tiago/vive_teleop/bridge/ros_entrypoint.sh) to avoid Foxy/Humble confusion.

4. Replace broad bridge behavior with an explicit topic bridge once the pipeline works.
   - Start with the current `--bridge-all-topics` for bring-up.
   - Then switch to a parameterized bridge config for only the topics you need, especially `/vive/head_pose` and any TF topics.
   - If `/tf_static` is used, configure ROS 2 durability correctly.

5. Add calibration and safety logic before commanding the robot.
   - Define a neutral headset pose that maps to TIAGO looking straight ahead.
   - Clamp yaw/pitch to TIAGO head joint limits.
   - Add deadband and filtering to suppress small tracking noise.
   - Add a software enable/disable gate so the robot does not move as soon as tracking appears.

6. Validate in layers.
   - First: verify the PC can see the Vive runtime and that the headset pose updates at a stable rate.
   - Second: verify ROS 2 publishes `/vive/head_pose` correctly in RViz.
   - Third: verify the bridge exposes the topic on ROS 1.
   - Fourth: verify the ROS 1 mapper produces the expected pan/tilt commands without moving the robot.
   - Fifth: test on TIAGO with conservative limits and low speed.

## Likely files to change
- [/home/biba/Documents/tiago/vive_teleop/docker-compose.yml](/home/biba/Documents/tiago/vive_teleop/docker-compose.yml)
- [/home/biba/Documents/tiago/vive_teleop/bridge/Dockerfile](/home/biba/Documents/tiago/vive_teleop/bridge/Dockerfile)
- [/home/biba/Documents/tiago/vive_teleop/bridge/ros_entrypoint.sh](/home/biba/Documents/tiago/vive_teleop/bridge/ros_entrypoint.sh)
- New ROS 2 package for Vive head pose publishing under [/home/biba/Documents/tiago/vive_teleop](/home/biba/Documents/tiago/vive_teleop)
- New ROS 1 node or script for TIAGO head mapping under [/home/biba/Documents/tiago/vive_teleop](/home/biba/Documents/tiago/vive_teleop)

## Assumptions
- SteamVR/OpenVR is the default source stack because your headset is connected to the PC but not yet configured.
- You want standard ROS messages (`PoseStamped`, `JointTrajectory`, TF) rather than custom interfaces.
- TIAGO remains on ROS 1 and the current bridge continues to be the cross-version transport layer.
- Full 6-DoF head pose is blocked until base stations are added and calibrated.