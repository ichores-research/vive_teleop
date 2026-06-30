# vive_teleop Project Context

## Purpose

`vive_teleop` is a direct teleoperation stack for a TIAGo robot. It streams the robot head camera to WebRTC clients, receives VR/debug input over a WebRTC data channel, publishes typed ROS 2 teleop topics, and converts those topics into head, wrist, and gripper controller commands.

The implemented architecture is split into five operational areas:

- `webrtc_server`: WebRTC signaling/media/input gateway and live robot state snapshot API.
- `moveit_server`: ROS 2 teleop control, MoveIt Servo runtime, pose-to-twist bridge, and direct head/gripper trajectory output.
- `unity-vr-headset`: intended VR operator client.
- `index.html`: browser debug client and manual input surface.
- `scripts`, `docker-compose*.yml`, `coturn`: runtime networking, startup, validation, and TURN relay.

A sixth area is designed but not implemented:

- `data_recorder`: an observational ROS 2/rosbag2 service for ML dataset
  capture. The future design uses deadman-delimited capture windows, records
  robot-space actions and measured outcomes, and must never affect control.

## Runtime Flow

1. Robot publishes camera images, joint states, and TF on the ROS 2 graph.
2. `webrtc_server` subscribes to camera frames and exposes WebRTC signaling on port `8088`.
3. Unity or the browser connects to `/offer` for video and `/input_offer` for data-channel input.
4. `webrtc_server` parses input JSON and publishes:
   - `/vive/head_pose`
   - `/vive/hand_target_pose`
   - `/vive/hand_target_active`
   - `/vive/gripper_opening`
   - `/vive/base_command`
   - `/vive/base_active`
5. `moveit_server` consumes those topics:
   - HMD orientation -> `/head_controller/joint_trajectory`
   - wrist target + deadman -> `/servo_node/pose_target_cmds` and `/servo_node/pose_target_active`
   - normalized gripper opening -> `/gripper_controller/joint_trajectory`
   - clicked joystick axes -> guarded differential-drive velocity on `/key_vel`
6. `servo_pose_bridge` converts absolute wrist poses into `TwistStamped` commands for MoveIt Servo.
7. MoveIt Servo publishes arm trajectories to `/arm_controller/joint_trajectory`.

## Data Stability Contract

This project is latency-sensitive and command-data-sensitive. Stable behavior depends on these invariants:

- Only fresh robot state should initialize or command control.
- Pose data must contain finite position values and valid quaternions.
- Wrist target frames must match the configured planning frame or be transformed explicitly.
- The deadman signal must be explicit and must immediately halt pursuit when false or stale.
- Base driving must require the joystick/trackpad click, time out on stale input,
  publish repeated zero commands when halted, and require a new physical click
  edge after a timeout.
- High-rate pose streams should keep only the newest command; old queued commands should not replay.
- Runtime checks must validate robot state, MoveIt group composition, and Servo command gating before operation.

## Key Architecture Docs

- `docs/architecture/README.md`: diagram index and rendering notes.
- `docs/architecture/deployment/deployment.puml`: runtime placement and top-level network connections.
- `docs/architecture/component/overview.puml`: readable top-level component communication.
- `docs/architecture/component/*.puml`: focused component views for gateway, MoveIt, and ROS topic boundaries.
- `docs/architecture/communication/data-flow.puml`: brief end-to-end data flow.
- `docs/architecture/class/**/*.puml`: class diagrams for runtime node internals.

## Future Data Recording Docs

- `docs/data-recording/README.md`: design status, decisions, and document index.
- `docs/data-recording/architecture-and-lifecycle.md`: proposed container,
  recorder state machine, startup, shutdown, and failure behavior.
- `docs/data-recording/dataset-contract.md`: observations, actions, outcomes,
  topic whitelist, episode identity, timestamps, and metadata.
- `docs/data-recording/implementation-plan.md`: phased repository changes and
  concrete integration points.
- `docs/data-recording/validation-plan.md`: automated and manual acceptance
  criteria for recorder and dataset integrity.
- `.agents/data-recorder-context.md`: concise context for future implementation
  work.

## Change Guidance

- Prefer keeping schema, frame, and timing changes backward-compatible or explicitly versioned.
- If a change adds a new payload field, update Unity, `index.html`, `input_publisher.py`, this `.agents` context, and the client/input diagram together.
- If a change affects command topics or frames, update `README.md`, `docs/architecture/*.puml`, runtime checks, and relevant launch/config files.
- Treat `unity-vr-headset/Assets/SteamVR` as vendored unless the task is specifically about SteamVR integration.
- Treat `data_recorder` as a future subsystem until its implementation plan is
  completed. Documentation must not imply that dataset recording currently
  starts with the stack.
