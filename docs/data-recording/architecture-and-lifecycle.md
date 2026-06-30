# Recorder Architecture and Lifecycle

## Status and Scope

This document specifies a future subsystem. It does not describe current
runtime behavior.

The first implementation should collect arm-centric teleoperation
demonstrations without affecting the control stack. It should be extensible to
whole-task pickup episodes, but it should not attempt to solve task annotation,
dataset conversion, and policy training in its first version.

## Constraints From the Current System

- ROS 2 distribution: Humble.
- Middleware: `rmw_cyclonedds_cpp`.
- Default ROS domain: `67`.
- Wi-Fi runtime: ROS containers use host networking and a generated
  CycloneDDS configuration.
- Field runtime: ROS containers use the `10.68.0.0/24` ipvlan network.
- Unity sends input at a configured `poseSendRateHz`, currently `30 Hz`.
- `moveit_server` evaluates pending hand targets on a `0.02 s` timer.
- Both `hand_target_timeout_sec` and the bridge `target_timeout_sec` are
  currently `0.12 s`.
- The deployed Servo pose bridge publishes at `100 Hz`.
- Head commands are produced at `20 Hz`.
- Gripper input is independent of the wrist deadman.
- Unity's current JSONL recording is controlled manually and is not aligned to
  a rosbag session.

These values are configuration, not permanent protocol constants. The recorder
must read or preserve the effective configuration instead of embedding them in
dataset code.

## Proposed Deployment

```text
Unity / browser
      |
      v
webrtc_server ------ /vive/* ------------------+
      |                                         |
      | camera/state                            v
      |                                  moveit_server
      |                                         |
      |                                         v
      +---------------- ROS 2 graph ------ Servo/controllers
                                                |
                                                v
                                      robot command topics

all selected observation, action, outcome, and gate topics
      |
      v
data_recorder container
  - recorder_controller node
  - embedded rosbag2_transport::Recorder
  - event/health publishers only
      |
      v
dedicated host recording volume
```

The recorder is a consumer of existing topics. It must not sit between any
producer and consumer in the control path.

## Failure-Isolation Rule

The control stack must not depend on recorder health. Specifically:

- `webrtc_server` and `moveit_server` must not `depends_on` recorder readiness.
- Recorder subscriptions must not change publisher QoS or block publishers.
- A full or slow disk must fail the recording session, not teleoperation.
- Recorder restart must create a new fragment or session identity. It must not
  silently append data with ambiguous continuity.
- Recorder health may be visible in logs and UI, but it must not be connected
  to deadman or Servo enable logic.

## Identity and Boundary Model

### Session

One invocation of `scripts/start-vive-teleop.sh` through shutdown. A session has
one stable `session_id`, one manifest, and normally one rosbag directory. If the
recorder crashes and restarts, each resulting bag is a separately identified
fragment of the same session.

Recommended ID shape:

```text
20260627T012345.678Z_<host>_<random-suffix>
```

Do not rely on the timestamp alone for uniqueness.

### Capture Window

An interval during which high-volume selected topics are being written. A
window includes the active interval and configured roll time. Capture windows
are a storage concept.

### Action Segment

One exact rising-to-falling interval of the upstream operator wrist deadman. A
new deadman press creates a new segment even if it occurs during the previous
window's post-roll and no writer pause occurs between the segments. The
downstream effective-action gate is recorded as a mask inside the segment.

### Task Episode

One semantic attempt, such as one object pickup. It can contain multiple
action segments, gripper actions, pauses, and recovery motions. Task episodes
need a separate annotation mechanism because the wrist deadman alone cannot
identify them reliably.

## Trigger Signals

### Capture Gate

Use `/vive/hand_target_active` as the default storage trigger for arm-centric
capture.

Reasons:

- It is the explicit wrist deadman state parsed by the gateway.
- `input_publisher.py` publishes it before `/vive/hand_target_pose` for each
  accepted sample, allowing the recorder to resume before later command stages.
- It exists at the gateway independently of lazy Servo pose publishers.
- It directly matches the operator's requested capture behavior.

Production recording must require the current explicit
`wristCommandEnabled` field and versioned payload schema. Do not accept the
legacy missing-field fallback as a trustworthy dataset trigger.

### Effective Action Gate

Record `/servo_node/pose_target_active` as the effective-action mask.

This is downstream of gateway parsing and `moveit_server` clutch/timeout logic.
It allows offline measurement of gateway/control latency and distinguishes
requested input from pose pursuit accepted by the Servo bridge. It must not be
the capture trigger because the publisher is created lazily and its first
message can race the first pose command.

### Recording Enable

Separate the concepts of "dataset collection is enabled" and "the arm action
is active".

Proposed control:

- `VIVE_TELEOP_RECORD_DATASET=0|1` determines whether the recorder is started.
- A future `/teleop/recording/session_enabled` command may permit operator UI
  control, but is not required for the first version.
- The upstream wrist deadman controls capture windows only while the session
  recorder is enabled.

## Recording Modes

The controller should have an explicit `recording_mode` rather than implicit
behavior.

### `deadman_window`

Recommended first mode. Capture starts on effective arm-gate activation and
pauses after release plus post-roll. This minimizes inactive video and creates
clean arm-action segments.

Limitations:

- It can omit head and gripper actions performed while the arm gate is false.
- It does not define a complete pickup attempt.
- It under-samples idle/no-op behavior unless explicit idle samples are added.

### `manual_episode`

Future mode. A dedicated operator signal opens and closes a complete task
episode. Deadman remains an action-valid mask inside the recording. This is the
preferred mode for whole pickup demonstrations containing multiple arm moves,
gripper actions, and deliberate pauses.

### `continuous_session`

Diagnostic mode. Record the explicit topic whitelist from startup to shutdown.
Use only for short debugging runs or storage-throughput measurements.

## Recorder State Machine

The first implementation should use explicit states and a single serialized
transition path.

| State | Writer | Meaning |
| --- | --- | --- |
| `STARTING` | Closed | Validate config, output path, and storage capacity. |
| `BOOTSTRAP` | Recording | Discover topics and capture static/config context. |
| `PAUSED` | Open, paused | Session is healthy but high-volume samples are skipped. |
| `RECORDING` | Recording | At least one operator deadman segment is active. |
| `POST_ROLL` | Recording | Capture gate is false; preserve terminal outcome for a short interval. |
| `STOPPING` | Flushing | Stop discovery, flush, close writer, and finalize metadata. |
| `FAILED` | Closed or degraded | Dataset session is invalid; control stack continues. |

### Startup Transition

```text
STARTING
  -> validate destination and free-space threshold
  -> create session directory and provisional manifest
  -> create recorder and subscriptions
  -> BOOTSTRAP
  -> capture /tf_static, topic types/QoS, initial state, and session event
  -> PAUSED
```

Do not start the writer paused before `/tf_static` and session metadata have a
chance to be captured. Humble does not provide the newer repeat-transient-local
behavior in the installed CLI, and a paused recorder skips messages as they
arrive.

The bootstrap completion condition should be explicit:

- Bootstrap-required observation topics have publishers and subscriptions.
- At least one valid camera frame and joint-state sample have been observed.
- Required static transforms are present, or a configurable bootstrap timeout
  has expired and the session is marked incomplete.
- The session-start event and manifest have been written.

Do not require `/servo_node/pose_target_active` or pose command publishers to
exist before bootstrap completes. The current `moveit_server` creates those
interfaces lazily after the first input, while Unity is intentionally launched
after runtime readiness.

### Rising Gate Transition

```text
PAUSED + capture_gate=true
  -> resume recorder
  -> create capture_window_id and action_segment_id
  -> publish WINDOW_START and SEGMENT_START events
  -> RECORDING
```

The recorder-controller subscription must stay active while the rosbag writer
is paused. Resume the recorder before publishing recorder events so those
events are included in the bag.

### Repeated True Samples

Repeated true messages are heartbeats. They update the gate watchdog but do not
create new IDs or duplicate start events.

### Falling Gate Transition

```text
RECORDING + capture_gate=false
  -> publish SEGMENT_END(reason=explicit_release)
  -> start post-roll timer
  -> POST_ROLL

POST_ROLL + timer expired
  -> publish WINDOW_END
  -> allow event to enter writer
  -> pause recorder
  -> PAUSED
```

The current post-roll candidate is `0.75 s`. It must be configurable and tuned
after measuring command-to-state settling time.

### Re-press During Post-Roll

Do not pause the writer. End the old action segment, create a new segment ID,
publish a new segment start, cancel the post-roll timer, and return to
`RECORDING`. The capture-window ID can remain unchanged because storage was
continuous.

### Watchdog Transition

If the last capture-gate heartbeat is true but no update arrives within
`capture_gate_stale_timeout_sec`, synthesize an action-segment end with reason
`gate_stale` and enter post-roll.

An initial candidate is `0.25 s`, which is longer than the current `0.12 s`
control timeout while still preventing indefinite recording after a component
failure. This is an independent recorder guard, not a replacement for the
control timeout.

### Shutdown Transition

On `SIGTERM`, `SIGINT`, or normal application shutdown:

1. Reject new segment starts.
2. If recording, emit segment/window end events with reason `shutdown`.
3. Stop recorder discovery.
4. Flush buffered messages.
5. Close the writer and finalize rosbag metadata.
6. Update the external session manifest atomically.
7. Write checksums only after all files are closed.

Compose must provide a generous stop grace period, initially `30 s`. The
startup script must wait for recorder exit instead of immediately killing the
container.

## Humble Recorder Implementation

The locally installed Humble CLI exposes these relevant options:

- `--start-paused`
- `--snapshot-mode`
- `--max-cache-size`
- `--qos-profile-overrides-path`
- `--storage-config-file`
- `--max-bag-size`
- `--max-bag-duration`
- file/message Zstd compression for compatible storage

However, probing the current image shows that a CLI recorder exposes parameter
services only; it does not expose recorder pause/resume services. Keyboard
control is also disabled when stdin is not a terminal.

Therefore the recommended implementation is a C++ ROS 2 node that owns an
instance of `rosbag2_transport::Recorder` and calls its `record()`, `pause()`,
`resume()`, and `stop()` API from the state machine. The controller and recorder
must run in a concurrency model that keeps gate callbacks and shutdown handling
responsive while recording.

Do not implement control by:

- Writing space characters into a pseudo-terminal.
- Sending undocumented signals to the CLI.
- Starting and killing a recorder process for every deadman interval.
- Creating one bag directory per 30 Hz input sample or per short clutch press.

## Roll Strategy

### Version 1

- Bootstrap context at session start.
- No pre-roll.
- Resume immediately on upstream deadman/capture-gate rise.
- Keep `0.75 s` configurable post-roll.
- Keep a small configurable number of explicit idle/terminal examples.

This is implementable with recorder pause/resume and avoids unbounded memory.

### Future Pre-Roll

Pre-roll is useful because the first action often depends on the immediately
preceding view and state. It requires buffering while disk writing is paused.

Potential approaches:

- A bounded in-memory circular buffer for selected topics, flushed on gate
  activation.
- Separate always-on low-bandwidth state recording plus a camera-specific
  buffer.
- A custom writer path combining snapshot buffering with active streaming.

Do not add pre-roll to version 1 unless tests prove buffer ordering, memory
bounds, and exact event timestamps.

## Head and Gripper Behavior

The wrist deadman is not a complete demonstration gate:

- Head commands can continue independently.
- Gripper joystick commands explicitly do not require wrist deadman.
- A pickup may require gripper closure after arm motion stops.

For the initial arm-centric mode, document an operator procedure requiring any
gripper action intended for the demonstration to occur while the capture window
is open or during post-roll.

For a full pickup dataset, implement `manual_episode` or another explicit
task-level signal. Do not silently claim that deadman segments represent whole
pickups.

## Container and Compose Integration

Proposed service names:

- Field profile: `data_recorder`
- Wi-Fi profile: `data_recorder_wifi`

Required environment:

- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- `ROS_DOMAIN_ID`, defaulting consistently with the stack
- `CYCLONEDDS_URI`
- `VIVE_TELEOP_SESSION_ID`
- recorder mode, output root, storage format, roll times, watchdog, and size
  limits

Required mounts:

- Recorder configuration, read-only.
- Generated CycloneDDS configuration, read-only in Wi-Fi mode.
- Dedicated dataset output root, read-write.
- Optional source/config paths, read-only, if their hashes are captured by the
  container rather than passed in the manifest.

Field mode can tentatively reserve `10.68.0.135`, following the existing
`.132` gateway, `.133` TURN, and `.134` MoveIt addresses. Validate network
ownership before committing this address.

The recorder needs no host HTTP port. Health should be exposed through ROS and
container health checks rather than a public network API unless operational
experience shows an HTTP endpoint is useful.

## Health and Observability

Publish or log at least:

- Session ID and fragment ID.
- Recorder state.
- Current capture-window/action-segment IDs.
- Selected storage plugin and compression.
- Current output path.
- Bytes written and available disk bytes.
- Required topic discovery status.
- Last camera, joint-state, TF, action, and gate sample age.
- Writer/cache dropped-message counts when available.
- Last error and whether the session is considered valid.

Proposed ROS namespace:

```text
/teleop/recording/status
/teleop/recording/events
/teleop/recording/session_enabled
```

Only status/events are required publishers. The recorder must never publish
messages to `/vive/*`, `/servo_node/*`, or controller command topics.

## Failure Behavior

| Failure | Recorder behavior | Teleoperation behavior |
| --- | --- | --- |
| Output path not writable | Fail before capture; log and mark invalid. | Continue unless operator explicitly required data capture. |
| Disk below threshold | Close/finalize if possible; mark incomplete. | Continue. |
| Writer exception | Enter `FAILED`; preserve diagnostics. | Continue. |
| Camera absent | Remain unready or mark observation stream incomplete. | Existing startup policy decides teleop availability. |
| Gate topic absent | Stay paused and report unready. | Continue. |
| Gate stream disappears while true | Watchdog ends segment and post-roll. | Existing control timeout halts pursuit. |
| Container restart | Create new fragment ID and bag. | Continue. |
| Host shutdown | Gracefully flush within stop grace period. | Normal stack shutdown. |
| Clock offset exceeds limit | Record data only as invalid/quarantined, based on policy. | Continue. |

## Multi-Client and Ownership Constraint

The current gateway permits multiple input channels to publish to the same ROS
topics. Before collecting production datasets, add or enforce a single active
operator/session owner. Otherwise action segments can mix sources without a
reliable label.

At minimum, future raw input and episode events need `source_id` and
`session_id`. Reject or quarantine sessions in which the active source changes
without an explicit boundary.

## Replay Safety

The bag whitelist contains live arm, head, and gripper commands. Replaying the
whole bag on the robot domain can command physical motion.

Required policy:

- Never run unrestricted `ros2 bag play` while connected to the robot domain.
- Use a dedicated offline `ROS_DOMAIN_ID` for inspection and export.
- Provide a future helper that plays observations only by default.
- Require an explicit unsafe flag to include command topics.
- Prefer offline readers for ML export rather than ROS playback.

## Initial Configuration Candidates

These are starting points, not acceptance criteria:

```yaml
recording_mode: deadman_window
capture_gate_topic: /vive/hand_target_active
effective_action_topic: /servo_node/pose_target_active
post_roll_sec: 0.75
pre_roll_sec: 0.0
capture_gate_stale_timeout_sec: 0.25
bootstrap_timeout_sec: 30.0
minimum_free_space_bytes: 20000000000
storage_id: mcap
stop_grace_period_sec: 30.0
```

Tune free space, compression, cache, and roll time only after measuring the
actual camera bandwidth and command-to-state latency on the target workstation.
