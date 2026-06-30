# Future data_recorder Context

## Status

This subsystem is designed but not implemented. Its detailed specification is
under `docs/data-recording/`. Do not describe it as available in runtime
instructions until the implementation and validation gates are complete.

## Responsibility

`data_recorder` will be a separate, observational ROS 2 Humble container that
creates synchronized machine-learning datasets from teleoperation sessions. It
must capture what the robot observed, what robot-space action was requested,
what command was emitted, and what state resulted.

Recorder failure, slow storage, disk exhaustion, or invalid configuration must
never block, delay, publish to, or otherwise alter the teleoperation control
path.

## Core Design Decisions

- Run one rosbag2 writer for the whole application session.
- Do not start and kill `ros2 bag record` for every deadman press.
- Use `/vive/hand_target_active` as the default capture gate because the gateway
  publishes it before the hand target, giving the recorder the earliest chance
  to resume.
- Record `/servo_node/pose_target_active` as the authoritative effective-action
  mask after downstream control gating.
- Keep the writer open and pause high-volume capture while no action is active.
- Record a short post-roll after release. Add pre-roll only after the basic
  recorder is stable.
- Record explicit event messages for session, capture-window, action-segment,
  timeout, and shutdown boundaries.
- Use an explicit topic whitelist. Never default to `ros2 bag record -a`.
- Store robot-space actions as primary policy labels. Raw Unity controller data
  is optional provenance, not the primary action target.
- Keep immutable rosbag/MCAP data as the source archive. Convert it into an ML
  training format offline.
- Store one bag per application session. Deadman intervals are indexed inside
  that session rather than creating many small bags.

## Identity Model

- Session: one startup-to-shutdown run and one bag directory.
- Capture window: an interval during which rosbag writes high-volume topics.
- Action segment: one exact deadman-active interval.
- Task episode: one semantically complete attempt, such as one pickup. A task
  episode may contain several action segments and should eventually carry a
  success/failure annotation.

Do not use these terms interchangeably in code, manifests, or logs.

## Recommended ML Contract

At logical sample time `t`, preserve:

- Observation: RGB image, joint state, wrist transform, gripper state, and any
  additional verified robot sensors used by the future policy.
- Action: the robot-space pose target or Cartesian twist and gripper target at
  the interface the learned policy will control.
- Outcome: the next measured joint, wrist, and gripper state.
- Mask/context: effective action gate, operator deadman, session ID, segment ID,
  task episode ID, timestamps, and task result.

Camera and robot-state trajectories alone are not sufficient action labels. A
stationary state can result from no command, controller lag, contact, clipping,
or blocked motion. Preserve both the command and measured outcome.

## Current Topic Chain

Robot observations:

- `/head_front_camera/rgb/image_raw`
- `/joint_states`
- `/tf`
- `/tf_static`

Operator/gateway intent:

- `/vive/head_pose`
- `/vive/hand_target_pose`
- `/vive/hand_target_active`
- `/vive/gripper_opening`

Robot-space arm command stages:

- `/servo_node/pose_target_cmds`
- `/servo_node/pose_target_active`
- `/servo_node/delta_twist_cmds`
- `/servo_node/status`
- `/arm_controller/joint_trajectory`

Direct controller commands:

- `/head_controller/joint_trajectory`
- `/gripper_controller/joint_trajectory`

Runtime topic types and QoS must be probed and saved in the session manifest.
The proposed whitelist is in `docs/data-recording/dataset-contract.md`.

## Deadman Semantics

The wrist deadman identifies requested arm action, not a complete manipulation
task or proof that Servo accepted a command. Head and gripper commands can occur
while wrist deadman is false. The default deadman-window mode is therefore
appropriate for arm-centric data, but a future whole-task dataset needs a
separate task/demo-active signal or manual task episode annotations.

The recorder must not rely only on an explicit `false` message. It needs a
watchdog because a WebRTC/data-channel failure can stop samples while the last
observed deadman state was true. The effective MoveIt gate already falls
inactive after the current `hand_target_timeout_sec` of `0.12` seconds; the
recorder watchdog is an independent storage guard and must be configurable.

Do not require downstream Servo pose publishers during recorder bootstrap. The
current MoveIt node creates those publisher interfaces lazily after first input,
so waiting for them before Unity starts would create a startup deadlock.

## Time Contract

- Unity currently sends `Time.realtimeSinceStartup` as a floating-point
  `timestamp`. It is monotonic relative to that Unity process, not UTC.
- `webrtc_server` currently ignores that timestamp and stamps ROS pose messages
  at gateway receipt time.
- Robot camera/joint/TF headers may use the robot clock, while bag receive time
  uses the recorder host clock.
- Preserve source stamp, gateway receipt stamp, and bag receive stamp when
  available.
- Synchronize robot and host clocks and record the measured offset. Do not
  silently align training rows from unsynchronized header clocks.

## Deployment Contract

The future container must:

- Use ROS 2 Humble, the same `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, and
  CycloneDDS config as the other ROS containers.
- Use host networking in the Wi-Fi profile.
- Use an available ipvlan address in field-network mode; `10.68.0.135` is the
  current proposal and must be checked for conflicts before use.
- Mount a dedicated recording volume and never write bags into the container
  layer.
- Have no control-topic publishers apart from namespaced recorder event/health
  topics.
- Finalize the bag on `SIGTERM` and receive enough Compose stop grace time.
- Emit a separate startup/runtime log collected by
  `scripts/start-vive-teleop.sh`.

## Humble-Specific Constraint

The installed Humble `ros2 bag record` supports `--start-paused`, but its CLI
process does not expose recorder pause/resume services in the current image.
The future implementation should embed `rosbag2_transport::Recorder` in a C++
node and call its pause/resume API. Do not control a CLI process by emulating
keyboard input or repeatedly spawning processes.

The current image only exposes SQLite3 storage. The recorder image must install
`ros-humble-rosbag2-storage-mcap` before selecting MCAP.

## Data Safety

Bags contain live controller command topics. Never replay an unrestricted bag
on the robot ROS domain. A future replay helper must use an isolated
`ROS_DOMAIN_ID` or explicit observation-only topic selection and command-topic
remapping.

## Required Implementation Updates

When implementing this subsystem, update together:

- `data_recorder/` Docker image, ROS package, config, and tests.
- `docker-compose.yml` and `docker-compose.wifi.yml`.
- `scripts/start-vive-teleop.sh` service startup, readiness, shutdown, and logs.
- `scripts/up-wifi-webrtc.sh` generated environment/service selection.
- Unity and `webrtc_server` if raw input, sequence, or session fields are added.
- `.gitignore` so generated bags cannot be committed.
- README and all data-recording documentation.
- Deployment, component, communication, and recorder class diagrams.

## Authoritative Detailed Docs

- `docs/data-recording/architecture-and-lifecycle.md`
- `docs/data-recording/dataset-contract.md`
- `docs/data-recording/implementation-plan.md`
- `docs/data-recording/validation-plan.md`
