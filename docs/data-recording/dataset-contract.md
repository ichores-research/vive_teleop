# Dataset Contract

## Status

Proposed contract for future implementation. Topic availability, types, QoS,
camera properties, and rates must be confirmed against the live TIAGo graph
before the contract is frozen.

## Dataset Objective

The first target is a dataset suitable for learning robot manipulation behavior
from teleoperated demonstrations. The archive must preserve enough information
to train different policy interfaces later without recollecting the same
demonstrations.

The dataset should distinguish:

- What the robot observed.
- What the operator intended.
- What robot-space target was accepted after mapping and constraints.
- What executable command was generated.
- What the robot actually did.
- Whether the action and task were valid and successful.

## Logical Training Record

At logical sample time `t`:

```text
observation_t
  RGB image
  joint positions and velocities
  end-effector pose
  gripper state
  optional additional sensors

action_t
  effective action-active mask
  robot-space pose target or Cartesian twist
  gripper target
  optional head target

outcome_t_plus_delta
  measured next joint state
  measured next end-effector pose
  measured next gripper state

context
  session/capture/action/task identifiers
  source and receive timestamps
  task/object labels
  success/failure and failure reason
  effective runtime configuration
```

Rosbag messages do not have to be converted into this row shape during
recording. The online recorder should preserve authoritative timestamped
streams. An offline exporter should align and resample them.

## Why Robot Action Is Required

Camera and robot state show the outcome but do not uniquely identify the action
that caused it. For example, an unchanged wrist pose can mean:

- No motion command was issued.
- A command was issued but the controller had not responded yet.
- The end effector was blocked by contact.
- A target was clipped by workspace or joint constraints.
- A command expired or deadman became inactive.

State differencing also amplifies sensor noise and hides commanded motion during
contact. Preserve robot-space commands even if the initial model is trained
from observations only.

## Action-Layer Selection

The future policy's output interface should determine the primary training
label. Record all inexpensive action stages now, then choose one during export.

| Layer | Current topic | Meaning | Dataset role |
| --- | --- | --- | --- |
| Operator target | `/vive/hand_target_pose` | Unity/browser target accepted by gateway | Optional intent/provenance |
| Effective gate | `/servo_node/pose_target_active` | Whether bridge accepts pose pursuit | Required action-valid mask |
| Mapped robot target | `/servo_node/pose_target_cmds` | Clutch-relative, workspace-constrained robot pose | Recommended high-level action source |
| Executable Cartesian command | `/servo_node/delta_twist_cmds` | Feedback/feed-forward physical twist sent to Servo | Recommended executed-action source |
| Low-level arm command | `/arm_controller/joint_trajectory` | Servo output sent to arm controller | Required diagnostic/low-level action |
| Gripper target | `/vive/gripper_opening` | Normalized requested opening | Recommended high-level gripper action |
| Gripper controller command | `/gripper_controller/joint_trajectory` | Physical finger target/duration | Required executed gripper action |
| Head controller command | `/head_controller/joint_trajectory` | Physical head pan/tilt trajectory | Optional unless head is part of policy |

### Recommended Initial Policy Interface

If the learned policy will retain the existing Servo/control stack, train it to
produce a robot-relative end-effector target/delta plus normalized gripper
target. Derive this offline from:

- `/servo_node/pose_target_cmds`
- measured wrist TF at the same time
- `/vive/gripper_opening`
- `/servo_node/pose_target_active`

Keep `/servo_node/delta_twist_cmds` and final trajectories as diagnostics and
alternative labels. Avoid training directly on raw VR coordinates unless the
deployment policy will intentionally reproduce Unity's calibration and clutch
mapping.

## Topic Whitelist

### Required Version 1 Topics

| Topic | Expected type | Producer | Role | Capture policy |
| --- | --- | --- | --- | --- |
| `/head_front_camera/rgb/image_raw` | `sensor_msgs/msg/Image` | TIAGo camera | Primary visual observation | Bootstrap, active, post-roll |
| `/joint_states` | `sensor_msgs/msg/JointState` | TIAGo | Measured positions/velocities/effort | Bootstrap, active, post-roll |
| `/tf` | `tf2_msgs/msg/TFMessage` | Robot/MoveIt | Dynamic measured frame state | Bootstrap, active, post-roll |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Robot/MoveIt | Camera/wrist kinematic context | Always capture during bootstrap |
| `/vive/hand_target_active` | `std_msgs/msg/Bool` | `webrtc_server` | Operator deadman and capture gate | Controller always observes; record while writer active |
| `/servo_node/pose_target_active` | `std_msgs/msg/Bool` | `moveit_server` | Effective arm-action mask | Record while writer active; do not require at bootstrap |
| `/servo_node/pose_target_cmds` | `geometry_msgs/msg/PoseStamped` | `moveit_server` | Mapped high-level robot action | Active, post-roll |
| `/servo_node/delta_twist_cmds` | `geometry_msgs/msg/TwistStamped` | pose bridge | Executable Cartesian command | Active, post-roll |
| `/arm_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | MoveIt Servo | Low-level arm command | Active, post-roll |
| `/vive/gripper_opening` | `std_msgs/msg/Float64` | `webrtc_server` | Normalized gripper intent | Active, post-roll |
| `/gripper_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | `moveit_server` | Physical gripper command | Active, post-roll |
| `/teleop/recording/events` | Versioned event message or JSON | recorder controller | Boundaries and reasons | Every event while writer active |

### Recommended Context Topics

| Topic | Expected type | Reason |
| --- | --- | --- |
| `/vive/hand_target_pose` | `geometry_msgs/msg/PoseStamped` | Compare operator/gateway target with mapped robot target. |
| `/vive/head_pose` | `geometry_msgs/msg/PoseStamped` | Preserve head intent and visual-attention context. |
| `/head_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | Preserve executed head action. |
| `/servo_node/status` | Verify at runtime | Identify Servo warnings, singularities, and halts. |
| `/vive/raw_input_json` | `std_msgs/msg/String`, proposed | Preserve Unity-only controller/calibration fields. |
| `/parameter_events` | `rcl_interfaces/msg/ParameterEvent` | Detect parameter changes during a session. |

### Runtime-Discovery Candidates

These are not currently referenced by application code. Probe the live graph
and add only when they support the intended learning objective:

- RGB camera info/intrinsics, commonly a `sensor_msgs/msg/CameraInfo` topic.
- Arm, head, and gripper controller-state topics.
- Force/torque sensors.
- Odometry and base velocity if base motion becomes part of the task.
- Depth, laser, or point-cloud observations if the policy consumes them.
- Robot diagnostics and hardware fault topics.
- Task/object pose or perception outputs.

Record the exact runtime name and type in configuration. Do not assume a topic
exists because it is conventional on another TIAGo deployment.

### Excluded by Default

- `ros2 bag record -a` and unrestricted regex capture.
- `/rosout`, unless debugging a specific incident.
- WebRTC media packets. Record the source ROS camera topic instead.
- Repeated high-volume topics not used by the target policy or validation.
- Secrets, TURN credentials, environment dumps containing credentials, and
  unrelated host telemetry.

## "Full Robot State" Definition

"Full" should mean all state required to reconstruct the demonstrated control
problem, not every topic on the ROS graph.

For current arm/head/gripper behavior, the minimum is:

- All controlled joint positions.
- Joint velocity and effort when the robot publishes them.
- Dynamic and static transforms needed to reconstruct wrist and camera poses.
- Actual gripper finger positions.
- Camera image and camera calibration.
- Controller status relevant to command acceptance.
- Robot-space commands at the selected action layers.

The dataset validator must verify that `/joint_states` includes at least:

```text
arm_1_joint ... arm_7_joint
head_1_joint
head_2_joint
gripper_left_finger_joint
gripper_right_finger_joint
```

If torso/base state affects reachable motion, include and validate those joints
or odometry even though the current Servo group intentionally excludes
`torso_lift_joint`.

## Current Unity Payload Inventory

Unity currently builds `PosePayload` at `poseSendRateHz`, configured as `30 Hz`,
and sets `timestamp = Time.realtimeSinceStartup`.

### Envelope

- `type`
- `timestamp`

### HMD

- `hmdAvailable`
- `hmdFrame`
- `hmdPx`, `hmdPy`, `hmdPz`
- `hmdRx`, `hmdRy`, `hmdRz`, `hmdRw`
- `headsetRecenter`

### Wrist and Workspace

- `wristAvailable`
- `wristCommandEnabled`
- `wristSource`
- `wristWorkspace`
- `wristPositionScale`
- `wristWorkspaceAnchorAvailable`
- `wristWorkspaceAnchorPx`, `wristWorkspaceAnchorPy`,
  `wristWorkspaceAnchorPz`
- `wristWorkspaceAnchorRx`, `wristWorkspaceAnchorRy`,
  `wristWorkspaceAnchorRz`, `wristWorkspaceAnchorRw`
- `wristFrame`
- `wristPx`, `wristPy`, `wristPz`
- `wristRx`, `wristRy`, `wristRz`, `wristRw`
- `robotWristFrame`
- `robotWristPx`, `robotWristPy`, `robotWristPz`
- `robotWristRx`, `robotWristRy`, `robotWristRz`, `robotWristRw`

### Controller and Gripper

- `joystickAxisX`, `joystickAxisY`
- `joystickTrigger`
- `joystickGrip`
- `joystickPrimaryButton`
- `gripperAvailable`
- `gripperOpening`

The local JSONL recorder writes this complete payload when manual recording is
active and a wrist sample is available.

## Current ROS Representation Gap

`webrtc_server` currently converts only selected payload fields into:

- `/vive/head_pose`
- `/vive/hand_target_pose`
- `/vive/hand_target_active`
- `/vive/gripper_opening`

The following useful provenance does not have a ROS representation:

- Raw controller source and pose.
- Workspace mode and anchor.
- Position scale.
- Joystick axes, trigger, grip, and primary button.
- Headset recenter event.
- Unity source timestamp.
- A sequence number, schema version, source ID, and session ID.

### Pragmatic Version 1 Solution

Publish each validated accepted input object on a proposed
`/vive/raw_input_json` topic using `std_msgs/msg/String`. Before publishing,
extend the object with:

```json
{
  "schemaVersion": 1,
  "sessionId": "...",
  "sourceId": "...",
  "sequence": 123,
  "gatewayReceivedUnixNs": 0
}
```

Preserve the original Unity monotonic `timestamp`. Bag receive time provides an
additional recorder-side timestamp.

Advantages:

- No shared custom interface package is required for version 1.
- All current Unity fields remain available.
- The browser can use the same schema.
- Offline export can validate JSON against a versioned schema.

Limitations:

- Field-level ROS typing is lost.
- Consumers must parse JSON.
- `std_msgs/String` has no header; source/gateway times must be inside JSON.

A future `vive_teleop_msgs/TeleopInputSample` message can replace it after the
payload contract and repository build contexts stabilize.

## Event Contract

Recorder events must exist inside the bag and in an external easy-to-read
index. A typed custom message is preferred because only the recorder/exporter
need to depend on it.

Proposed logical fields:

```text
header.stamp
schema_version
session_id
fragment_id
capture_window_id
action_segment_id
task_episode_id
event_type
reason
source_id
operator_deadman
effective_action
details_json
```

Required event types:

```text
SESSION_START
SESSION_END
BOOTSTRAP_COMPLETE
WINDOW_START
WINDOW_END
SEGMENT_START
SEGMENT_END
TASK_EPISODE_START
TASK_EPISODE_END
RECORDER_WARNING
RECORDER_FAILURE
```

Required segment/window end reasons:

```text
explicit_release
gate_stale
source_changed
operator_stop
shutdown
disk_low
writer_error
configuration_error
```

Write the same event as one JSON line in `events.jsonl` after successful bag
publication. The bag is authoritative for stream alignment; the JSONL index is
for discovery and recovery.

## Task and Success Annotation

Action segments alone do not indicate whether the robot picked up an object.
Each task episode should eventually include:

- Task name and version.
- Object identifier/category.
- Initial object placement or scenario identifier when available.
- Operator/source identifier using a non-sensitive pseudonym.
- Success, failure, aborted, or unknown result.
- Failure reason such as miss, slip, unreachable, operator abort, or system
  fault.
- Optional quality/rating and free-form note.

Do not mark every completed recording window as a successful pickup. Default
result must be `unknown` until explicitly annotated or determined by a verified
automatic signal.

## Idle and Terminal Samples

Removing all inactive data reduces storage but can produce a policy that sees
only movement and does not learn waiting or stopping.

The initial exporter should support:

- Keeping the release sample and configured post-roll.
- Keeping a bounded number of zero-action samples after settling.
- Sampling a small fraction of inactive session state as negative examples.
- Emitting an `action_valid` mask so inactive samples are not mislabeled as
  expert movement.

Storage gating and training filtering are separate decisions. Preserve enough
boundary context to revisit the filtering policy later.

## Timestamp Contract

### Available Clocks

- Unity source monotonic time: current `PosePayload.timestamp`.
- Gateway ROS receipt time: used to stamp outgoing `PoseStamped` messages.
- Original robot message header time: camera, joint, and TF publishers.
- Rosbag receive/storage time: recorder host ROS clock.

### Required Additions

- Add an unsigned monotonic `sequence` to every client payload.
- Add `schemaVersion`, `sessionId`, and `sourceId`.
- Preserve gateway receipt time in raw input JSON.
- Record robot/host clock synchronization status and measured offset in the
  manifest.

### Alignment Rules

- Never align Unity `Time.realtimeSinceStartup` directly to UTC.
- Use sequence to detect client loss/reordering.
- Use gateway receipt and bag timestamps to estimate transport/recorder delay.
- Prefer original ROS header time for sensor-time alignment only after clock
  synchronization has been verified.
- Preserve raw timestamps. Do not overwrite source timestamps during export.
- Store the exporter's chosen time basis and interpolation rules in exported
  dataset metadata.

### Clock Readiness

Before production collection:

- Synchronize host and robot clocks using the site's supported NTP/PTP setup.
- Measure offset before and after each session.
- Define a maximum acceptable offset and drift for the target control rate.
- Mark sessions outside the limit as quarantined rather than silently fixing
  them.

## Frame Contract

Current important frames:

- Planning/reference frame: `base_footprint`.
- End effector: `arm_tool_link`.
- Camera: `head_front_camera_link`.
- Raw Unity frame: `unity_world`.

Every pose-derived export must include source and target frame. Never combine
positions from different frames based only on field name.

For a default robot-relative action export:

1. Read current `base_footprint -> arm_tool_link` TF at `t`.
2. Read `/servo_node/pose_target_cmds` in `base_footprint`.
3. Compute translation and shortest-path quaternion delta.
4. Preserve both absolute target and derived delta.
5. Reject or flag missing/stale TF and invalid quaternions.

## QoS Contract

The recorder must use explicit QoS overrides for sensor and transient-local
topics where automatic negotiation is insufficient.

At minimum validate:

- Camera compatibility with sensor-data/best-effort publication.
- `/joint_states` compatibility with sensor-data QoS.
- `/tf_static` transient-local durability so bootstrap receives static frames.
- Reliable delivery for low-rate event and gate topics where compatible.

Save effective offered/requested QoS in the manifest. A subscriber existing in
the graph is not proof that it receives every required sample.

## Storage Format

### Recommended Archive

Use MCAP with chunking, indexes, CRCs, and Zstd compression. Install
`ros-humble-rosbag2-storage-mcap` in the recorder image; it is not available in
the current MoveIt image probe.

Do not select the `fastwrite` profile for long-term source data without an
explicit post-processing step because it trades away integrity/index features.

SQLite3 remains an acceptable fallback for the first functional prototype.
Whichever format is selected must be recorded in the manifest and tested for
graceful and interrupted recovery.

### Camera Capacity

Raw image storage dominates. Approximate uncompressed rate:

```text
width * height * channels * frames_per_second
```

For illustration, `640 * 480 * 3 * 30` is about `27.6 MB/s`, or approximately
`99.5 GB/hour`, before rosbag overhead. Do not assume this is the deployed
camera's actual resolution or rate. Measure it with the live topic and a timed
recording.

Choose between raw images, image-transport compression, and bag-level
compression based on:

- Model quality requirements.
- CPU budget on the teleoperation workstation.
- Disk throughput and capacity.
- Decode throughput during training.
- Whether lossy compression is acceptable.

## Session Directory Layout

Proposed layout:

```text
recordings/
  2026-06-27/
    session_<session_id>/
      manifest.json
      events.jsonl
      bag/
        metadata.yaml
        session_<session_id>_0.mcap
      parameters/
        vive_moveit_server.yaml
        servo_pose_bridge.yaml
        servo_node.yaml
      config/
        topics.yaml
        qos-overrides.yaml
        storage.yaml
      unity/
        optional_controller_payloads.jsonl
      validation.json
      checksums.sha256
```

Write temporary manifest/validation files and atomically rename them only after
successful serialization.

## Session Manifest

Required fields:

### Identity

- Manifest schema version.
- Session ID and fragment ID.
- UTC start/end time and stop reason.
- Host, robot, and source identifiers using non-sensitive stable IDs.

### Source Version

- Git commit.
- Dirty-worktree flag and optional diff hash.
- Docker image names and immutable IDs/digests.
- Unity editor/player version and build stamp/hash.
- ROS distribution, RMW implementation, ROS domain, and DDS config hash.

### Recording Configuration

- Recording mode.
- Topic whitelist and discovered runtime types.
- Offered/requested QoS.
- Storage plugin, compression, chunk/cache/split settings.
- Roll/watchdog/bootstrap values.
- Output filesystem and free-space values, excluding sensitive mount details.

### Robot/Control Configuration

- Relevant ROS parameter dumps.
- SHA-256 hashes of MoveIt/Servo/teleop YAML files.
- Important frame and joint names.
- Camera encoding, dimensions, nominal rate, and camera-info source.

### Timing

- Clock sources.
- Robot-host offset and measurement method.
- Start/end offset and estimated drift.
- Gateway and recorder host identity if they differ.

### Quality Summary

- Message counts per topic.
- First/last timestamp per topic.
- Missing/dropped sequence ranges.
- Maximum active-period gap per required topic.
- Dataset validity status and reasons.
- Task-episode annotation completeness.

## Offline Export Contract

The exporter should be a separate process and versioned independently from raw
capture. It should:

1. Refuse unrestricted command-topic playback.
2. Read bags directly.
3. Select a declared time basis and fixed sample rate.
4. Associate each image with state/action using documented interpolation.
5. Preserve masks for missing, stale, interpolated, and inactive data.
6. Compute robot-relative action labels without destroying absolute originals.
7. Split train/validation/test by session/scenario, not random adjacent frames.
8. Save source session IDs and exporter version in every generated shard.

Possible downstream formats can be selected later. The rosbag/MCAP plus
manifest remains the immutable source of truth.
