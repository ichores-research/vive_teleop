# Implementation Plan

## Status

No implementation exists yet. Follow the phases in order. Do not wire the
recorder into the default startup path until the synthetic integration tests and
graceful-shutdown tests pass.

## Target Repository Layout

```text
data_recorder/
  Dockerfile
  cyclonedds.xml
  src/
    vive_dataset_recorder/
      CMakeLists.txt
      package.xml
      include/vive_dataset_recorder/
        recorder_controller.hpp
        recorder_events.hpp
        session_manifest.hpp
      msg/
        RecordingEvent.msg
      src/
        main.cpp
        recorder_controller.cpp
        recorder_events.cpp
        session_manifest.cpp
      config/
        recorder.yaml
        topics.yaml
        qos-overrides.yaml
        mcap-writer-options.yaml
      test/
        test_recorder_state_machine.cpp
        test_session_manifest.cpp
        test_event_contract.cpp

tools/
  dataset/
    README.md
    validate_session.py
    export_session.py
    requirements.txt

docs/data-recording/
  ... existing design documents ...
```

The exact C++ file split can follow local preference, but keep state transitions
testable without a live ROS graph or writer.

## Phase 0: Measure and Freeze the Initial Contract

Before writing recorder code, collect one short live-system inventory.

### Topic Inventory

Run in the ROS environment:

```bash
ros2 topic list -t
ros2 topic info /head_front_camera/rgb/image_raw -v
ros2 topic info /joint_states -v
ros2 topic info /tf -v
ros2 topic info /tf_static -v
ros2 topic info /servo_node/pose_target_active -v
ros2 topic info /servo_node/status -v
```

Record exact topic names, types, publisher counts, and offered QoS. Confirm
whether camera-info and controller-state topics exist.

### Bandwidth Inventory

```bash
ros2 topic hz /head_front_camera/rgb/image_raw
ros2 topic bw /head_front_camera/rgb/image_raw
ros2 topic hz /joint_states
ros2 topic hz /servo_node/delta_twist_cmds
ros2 topic hz /servo_node/delta_joint_cmds
```

Capture at least one minute under actual teleoperation. Use measured p95/p99
rates and bandwidth to choose cache, disk, and post-roll settings.

### Clock Inventory

Determine:

- Robot clock synchronization source.
- Host clock synchronization source.
- Measured offset and drift.
- Whether camera, joint-state, and TF header stamps share one clock.

Do not begin production dataset collection until a repeatable offset check
exists.

### Initial Decisions to Record

- Primary policy action interface: pose target or Cartesian twist.
- Camera representation: raw, compressed transport, or bag compression.
- MCAP or temporary SQLite3 prototype.
- Required and optional topics.
- Maximum accepted active-period sample gap per required topic.
- Disk reserve and retention policy.

## Phase 1: Build a Standalone Recorder Image

### Docker Image

Pin the recorder to the same ROS 2 Humble base selected for the rest of the
project. The current application images use floating `ros:humble` tags, so image
pinning should be completed before reproducible dataset collection. Install at
least:

```text
ros-humble-rmw-cyclonedds-cpp
ros-humble-rosbag2
ros-humble-rosbag2-storage-mcap
python3-colcon-common-extensions
build-essential
cmake
```

ROS package dependencies should include:

```text
rclcpp
rosbag2_cpp
rosbag2_transport
rosbag2_storage
rosbag2_storage_mcap
std_msgs
diagnostic_msgs
builtin_interfaces
rosidl_default_generators
```

Add only message dependencies required by the event/status contract. Generic
rosbag subscriptions should handle selected application topic types.

The image must fail its build if:

```bash
ros2 bag record --help
ros2 pkg prefix rosbag2_storage_mcap
```

do not succeed after sourcing the workspace.

### Storage User and Permissions

Do not rely on root-owned output files. Either:

- Build/run with the host user's UID/GID passed at build/runtime, or
- Create a fixed container user and ensure the mounted recording directory is
  writable by that user.

Add an image-level smoke test that creates, closes, inspects, and deletes a
small bag in a temporary directory.

## Phase 2: Implement the Recorder Controller

### Why C++

The Humble `ros2 bag record` CLI in the current image supports
`--start-paused`, but a runtime probe exposes no recorder pause/resume services
and keyboard handling is disabled without a terminal. The Humble C++
`rosbag2_transport::Recorder` API exposes `pause()`, `resume()`, and `stop()`.

Embed that API. Do not automate terminal keyboard input.

### Separation of Concerns

Implement two logical objects:

1. Pure `RecorderStateMachine`
   - Inputs are gate events, time, writer results, disk status, and shutdown.
   - Outputs are state changes and commands such as resume, pause, emit event,
     and stop.
   - No ROS dependencies in core transition tests.

2. ROS `RecorderController`
   - Owns or coordinates the rosbag2 recorder.
   - Subscribes to gate signals independently from paused bag subscriptions.
   - Executes state-machine commands.
   - Publishes events/status and updates manifest/index files.

### Threading Model

The implementation must ensure:

- Bag recording cannot block gate callbacks.
- Pause/resume/stop calls are serialized.
- Only one state transition mutates IDs/timers at a time.
- Shutdown can interrupt bootstrap and post-roll safely.
- Event publication occurs before the corresponding pause.
- The final writer stop is called exactly once.

The implemented model uses a multi-threaded controller executor and a dedicated
single-threaded executor for the embedded rosbag2 node. Humble's
`Recorder::record()` initializes recording and returns; the dedicated thread
must spin the recorder node rather than treating that return as writer exit.
During shutdown the recorder executor remains alive long enough to consume
terminal event messages, then it is cancelled and joined before `stop()` closes
the writer. Pause/resume/stop calls remain serialized by a narrow mutex, while
state transitions use one mutually exclusive callback group.

### Proposed Parameters

`config/recorder.yaml`:

```yaml
vive_dataset_recorder:
  ros__parameters:
    recording_mode: deadman_window
    capture_gate_topic: /vive/hand_target_active
    effective_action_topic: /servo_node/pose_target_active
    event_topic: /teleop/recording/events
    status_topic: /teleop/recording/status
    post_roll_sec: 0.75
    pre_roll_sec: 0.0
    capture_gate_stale_timeout_sec: 0.25
    bootstrap_timeout_sec: 30.0
    status_publish_rate_hz: 1.0
    disk_check_period_sec: 2.0
    minimum_free_space_bytes: 20000000000
    output_root: /recordings
    storage_id: mcap
    max_bag_size_bytes: 0
    max_bag_duration_sec: 0
```

Validate all values before opening a writer. Reject negative durations,
unknown modes, an empty session ID, duplicate topics, command topics used as
event topics, and an output root outside the mounted dataset root.

### Proposed Topic Configuration

`config/topics.yaml` should be project-owned data rather than a command-line
string:

```yaml
schema_version: 1
bootstrap_required:
  - /head_front_camera/rgb/image_raw
  - /joint_states
  - /tf
  - /tf_static
  - /vive/hand_target_active
  - /teleop/recording/events
active_required:
  - /servo_node/pose_target_active
  - /servo_node/pose_target_cmds
  - /servo_node/delta_twist_cmds
  - /servo_node/delta_joint_cmds
  - /arm_controller/joint_trajectory
  - /vive/gripper_opening
  - /gripper_controller/joint_trajectory
optional:
  - /vive/raw_input_json
  - /vive/hand_target_pose
  - /vive/head_pose
  - /head_controller/joint_trajectory
  - /servo_node/status
  - /parameter_events
```

The recorder may start with missing optional and active-required topics. The
current Servo pose publishers are created lazily after first input, so waiting
for them before Unity starts would deadlock startup. Topic policy should
distinguish:

- Missing bootstrap-required topic at bootstrap.
- Missing active-required topic after capture begins.
- Discovered after bootstrap.
- Disappeared while active.
- Reappeared with a different type or QoS.

A type change during one session invalidates the affected stream.

### QoS Overrides

Create `config/qos-overrides.yaml` after Phase 0 measurement. Expected shape:

```yaml
/head_front_camera/rgb/image_raw:
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 10
/joint_states:
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 10
/tf_static:
  reliability: reliable
  durability: transient_local
  history: keep_last
  depth: 1
/teleop/recording/events:
  reliability: reliable
  durability: volatile
  history: keep_last
  depth: 100
```

Do not copy these values blindly. Compare them with live publishers and validate
message receipt.

### MCAP Options

Start with integrity and index features enabled:

```yaml
noChunkCRC: false
noAttachmentCRC: false
enableDataCRC: false
noSummaryCRC: false
noChunking: false
noMessageIndex: false
noSummary: false
chunkSize: 786432
compression: Zstd
compressionLevel: Fast
forceCompression: false
```

Benchmark CPU and output size before changing compression level. Preserve the
effective storage options in the manifest.

### State-Machine Skeleton

```text
on_start:
  validate()
  create_session()
  recorder.record()
  state = BOOTSTRAP

on_bootstrap_ready_or_timeout:
  emit(BOOTSTRAP_COMPLETE)
  recorder.pause()
  state = PAUSED

on_capture_gate(true):
  refresh_watchdog()
  if state == PAUSED:
    recorder.resume()
    open_window()
    open_segment()
  elif state == POST_ROLL:
    cancel_post_roll()
    open_segment()
  state = RECORDING

on_capture_gate(false):
  if state == RECORDING:
    close_segment(explicit_release)
    start_post_roll()
    state = POST_ROLL

on_gate_watchdog:
  if last_capture_gate_value == true:
    close_segment(gate_stale)
    start_post_roll()
    state = POST_ROLL

on_post_roll_expired:
  close_window()
  recorder.pause()
  state = PAUSED

on_shutdown:
  close_open_ids(shutdown)
  recorder.stop()
  finalize_manifest()
```

`on_effective_action(value)` updates the effective-action mask/status and event
context but does not open the writer. This avoids losing the first command to a
race with a lazily created downstream publisher.

Account for repeated booleans, a false message in `PAUSED`, true during
`BOOTSTRAP`, re-press during post-roll, and shutdown from every state.

## Phase 3: Session Identity and Raw Input Provenance

### Generate Identity in the Launcher

Generate `VIVE_TELEOP_SESSION_ID` once near the start of
`scripts/start-vive-teleop.sh`, before Compose or Unity starts. Reuse it for:

- Recorder output/manifest.
- Recorder log prefix.
- Unity payloads and optional local JSONL filename.
- `webrtc_server` raw input publication.
- Any task-annotation tool.

Do not let each process independently generate a session ID.

### Extend Unity Payload

Add to `PosePayload` in
`unity-vr-headset/Assets/ViveTeleopWebRtcClient.cs`:

```text
schemaVersion
sessionId
sourceId
sequence
```

Requirements:

- Increment `sequence` once per generated payload, including payloads not sent
  because the channel is unavailable only if that behavior is documented.
- Prefer sequence over floating-point time for drop/reorder detection.
- Keep `timestamp = Time.realtimeSinceStartup` for source-relative timing.
- Read session/source identity from controlled environment or startup config.
- Include the same fields in local JSONL.

Update browser payload builders in `index.html` to the same schema.

### Publish Raw Input on ROS

In `webrtc_server/src/image_listener/image_listener/input_publisher.py`:

1. Parse and validate the object.
2. Verify schema/session/source ownership policy.
3. Add gateway receipt time without changing the original source timestamp.
4. Publish versioned JSON on `/vive/raw_input_json`.
5. Publish the existing typed command topics.

Whether rejected payloads are recorded should be a separate diagnostics policy.
Do not put malformed untrusted payloads into the training stream by default.

## Phase 4: Manifest and Event Index

### Host Seed Manifest

The launcher should create a seed manifest before Compose starts containing
host-owned information:

- Git commit and dirty flag.
- Unity build/version information.
- Selected Compose files and profiles.
- Config file hashes.
- Session ID and UTC start time.

Mount it into the recorder container read-only. Do not capture `.env` contents
or TURN credentials.

### Recorder Final Manifest

The recorder enriches and finalizes the manifest with:

- Runtime topic types/QoS and publishers.
- Bag storage settings and file list.
- ROS parameter dumps or hashes.
- Clock-check result.
- Message counts/time ranges.
- State transitions and errors.
- UTC end time, stop reason, and validation state.

Use atomic temp-file replacement. Preserve a provisional manifest after a
crash whenever possible.

### Event Message

Create `RecordingEvent.msg` using the logical fields in
`dataset-contract.md`. Use numeric constants for event types while preserving a
human-readable reason string.

Assign IDs before event publication. Never reuse an action-segment ID within a
session.

## Phase 5: Compose Integration

### Base Field Profile

Add `data_recorder` to `docker-compose.yml`:

- Build `./data_recorder`.
- Mount the field CycloneDDS config.
- Join `field_net` with proposed address `10.68.0.135` after conflict check.
- Mount `${VIVE_TELEOP_RECORDING_ROOT:-./recordings}:/recordings`.
- Pass session ID and recorder configuration.
- Set `init: true` and `stop_grace_period: 30s`.
- Do not make control services depend on recorder health.

### Wi-Fi Profile

Add `data_recorder_wifi` to `docker-compose.wifi.yml`:

- Use `network_mode: host`.
- Mount the generated `${CYCLONEDDS_HOST_CONFIG}` read-only.
- Use the same ROS domain/RMW variables as gateway and MoveIt.
- Mount the same dedicated output root.
- Use a distinct container name and log filename.

### Restart Policy

Avoid silent continuity. Either:

- Use `restart: "no"` and require explicit operator restart, or
- Use `on-failure` only after fragment IDs and restart manifests are tested.

Never append a restarted writer into an existing bag directory.

## Phase 6: Startup Script Integration

Update `scripts/start-vive-teleop.sh` only after the standalone service passes
integration tests.

Required changes:

1. Generate/export session ID.
2. Resolve and create recording root/session directory.
3. Check free space and write permissions.
4. Create seed manifest.
5. Include recorder service when `VIVE_TELEOP_RECORD_DATASET=1`.
6. Capture Compose output in existing `docker-compose-up.log`.
7. Follow recorder logs into `data_recorder_wifi.log`.
8. Wait for recorder bootstrap status with a configurable timeout.
9. Warn and continue by default on recorder failure.
10. Fail startup only when `VIVE_TELEOP_REQUIRE_RECORDER=1`.
11. On cleanup, stop Unity/input first, then let recorder post-roll/finalize,
    then stop the ROS containers.
12. Wait for graceful recorder termination before forced Compose cleanup.

Proposed environment variables:

```text
VIVE_TELEOP_RECORD_DATASET=0
VIVE_TELEOP_REQUIRE_RECORDER=0
VIVE_TELEOP_RECORDING_ROOT=<repo>/recordings
VIVE_TELEOP_RECORDING_MODE=deadman_window
VIVE_TELEOP_RECORDING_POST_ROLL_SEC=0.75
VIVE_TELEOP_RECORDING_GATE_TIMEOUT_SEC=0.25
VIVE_TELEOP_RECORDING_STORAGE=mcap
VIVE_TELEOP_RECORDING_MIN_FREE_BYTES=20000000000
VIVE_TELEOP_SESSION_ID=<generated>
```

Do not overload the existing `VIVE_TELEOP_RECORD_CONTROLLER`; it controls the
Unity-local JSONL recorder and has different semantics.

Update `scripts/up-wifi-webrtc.sh` service lists and environment propagation in
the same change.

## Phase 7: Health and Operator Feedback

At minimum, make recorder state visible in:

- `data_recorder_wifi.log` or `data_recorder.log`.
- `/teleop/recording/status`.
- Session manifest.
- Startup summary.

The status should distinguish:

```text
ready-paused
recording
post-roll
stopping
failed
```

Do not report "recording" merely because the container is running.

Later, Unity can show a small recording-state indicator driven by server/ROS
status, but UI work should not block the first recorder implementation.

## Phase 8: Offline Validator and Exporter

Implement `tools/dataset/validate_session.py` before collecting production
data. It should read the bag and manifest directly and produce
`validation.json`.

Then implement `export_session.py` with explicit options for:

- Time basis.
- Output sample rate.
- Primary action layer.
- Image encoding/resize.
- State interpolation.
- Idle sample policy.
- Action delay/offset.
- Included task result classes.

Never default to replaying command topics through ROS.

## Phase 9: Documentation and Diagrams

After implementation behavior is stable:

- Update `README.md` with operator commands and storage locations.
- Change all "future" wording in this folder to implemented behavior where
  accurate.
- Add recorder to deployment/component/communication PlantUML diagrams.
- Add a class diagram for recorder controller/state machine/manifest writer.
- Update `.agents/project-context.md` and runtime/network context.
- Add generated `recordings/` output to `.gitignore` while retaining any
  intentionally tracked schema/examples.

## Implementation Order Summary

1. Measure live topic/QoS/bandwidth/clock behavior.
2. Build the standalone image and verify MCAP.
3. Implement and unit-test pure state transitions.
4. Integrate embedded rosbag2 recorder.
5. Add events and manifests.
6. Run synthetic ROS integration tests.
7. Add session identity/raw input provenance.
8. Integrate Compose and startup logging.
9. Implement direct bag validation/export.
10. Run controlled real-robot data validation.
11. Make dataset capture available by an explicit opt-in flag.

## Definition of Done

Version 1 is complete only when:

- Recorder capture starts/stops correctly for explicit release and stale input.
- No recorder publisher exists on any control topic.
- A slow/full disk does not alter teleoperation timing or availability.
- Bags finalize on normal shutdown and are recoverable after forced failure.
- Required topics, types, QoS, event IDs, timestamps, and manifests validate.
- Camera/state/action streams can be aligned and exported deterministically.
- Every action segment has one start and one terminal event.
- An observation-only replay/export workflow cannot command the live robot.
- Documentation and diagrams match the implemented behavior.
