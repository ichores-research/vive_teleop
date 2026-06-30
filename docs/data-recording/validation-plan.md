# Recorder and Dataset Validation Plan

## Purpose

The recorder is useful only if it preserves synchronized, explainable data
without affecting teleoperation. Validation therefore has two independent
targets:

1. Runtime isolation and lifecycle correctness.
2. Dataset completeness, timing, and semantic correctness.

Passing one does not imply the other.

## Pragmatic Test Layers

This project is operated by one developer and depends on robot/VR hardware. Do
not build a large hosted CI system for hardware behavior. Use three layers:

### Fast Local Gate

Run on every recorder change:

- C++ format/lint as configured by the ROS package.
- Pure state-machine unit tests.
- Manifest/event serialization tests.
- Recorder config schema tests.
- Shell syntax for changed scripts.
- Compose config validation with required environment placeholders.

Target runtime: under one minute after dependencies are built.

### Container Synthetic Integration

Run before merging a recorder change:

- Start the recorder against synthetic ROS publishers.
- Exercise gate transitions and inspect the produced bag.
- No Unity, SteamVR, robot, or external network required.

### Manual Hardware Acceptance

Run before collecting valuable demonstrations or after changes to topics,
timing, middleware, camera handling, or storage.

## Unit Tests

### State-Machine Transition Matrix

Test every meaningful state/input combination.

| Initial state | Input | Expected result |
| --- | --- | --- |
| `STARTING` | valid config | Open session and enter `BOOTSTRAP`. |
| `STARTING` | invalid config/output | Enter `FAILED`; no bag left as valid. |
| `BOOTSTRAP` | required streams ready | Emit bootstrap event, pause, enter `PAUSED`. |
| `BOOTSTRAP` | timeout | Pause with incomplete reason or fail per configured policy. |
| `BOOTSTRAP` | capture gate true | Remember pending activation; start after bootstrap. |
| `PAUSED` | false heartbeat | Stay paused without events/ID changes. |
| `PAUSED` | true | Resume, create window/segment, emit starts. |
| `RECORDING` | repeated true | Refresh watchdog only. |
| `RECORDING` | false | End segment, start post-roll. |
| `RECORDING` | stale watchdog | End segment with `gate_stale`, start post-roll. |
| `POST_ROLL` | true | Cancel pause, open new segment, keep current window. |
| `POST_ROLL` | timer expiry | End window, pause, enter `PAUSED`. |
| Any live state | shutdown | Close IDs, stop exactly once, finalize. |
| `FAILED` | any gate | No writer restart or ID mutation. |

### Idempotence and Ordering

Verify:

- Repeated true does not create duplicate segment/window starts.
- Repeated false does not create duplicate ends.
- Every segment/window ID is unique within the session.
- Start event precedes data considered part of the segment.
- Segment end precedes window end.
- Window end is published before recorder pause.
- Shutdown closes each open boundary once.
- Writer `stop()` is invoked once even if shutdown signals repeat.
- A timer from an old segment cannot close a newer segment.

### Time Tests

Use a fake monotonic clock. Do not sleep in unit tests.

- Watchdog expires exactly at configured policy boundary.
- Post-roll uses monotonic time and is unaffected by wall-clock changes.
- Negative, NaN, infinite, and unreasonably large duration parameters are
  rejected.
- UTC timestamps are metadata only and do not drive state transitions.

### Manifest Tests

- Required fields cannot be omitted.
- Unknown schema versions are rejected by validator.
- Temporary file is atomically replaced.
- Secrets and `.env` values are not serialized.
- Dirty-worktree and missing-image-digest cases are represented explicitly.
- Crash/provisional manifest cannot be mistaken for a finalized valid session.
- File names and IDs reject path separators and traversal.

### Event Tests

- Numeric event constants remain stable once released.
- Unknown reason strings remain parseable but are flagged.
- Event timestamps are finite and monotonic within one source clock.
- `details_json` is valid JSON when present.
- External `events.jsonl` entries match bag event IDs.

## Synthetic Integration Environment

Create a small test publisher package or script that publishes:

- Configurable synthetic RGB images with frame sequence encoded in pixels.
- Joint states for all required joints with sequence-derived values.
- Dynamic wrist/camera transforms and one transient-local static transform.
- Upstream capture/deadman and downstream effective-action booleans.
- Pose targets, twists, and joint trajectories with shared sequence values.
- Optional intentional drops, reordering, type absence, and timing jitter.

The sequence must be recoverable from each synthetic stream so the validator
can prove alignment rather than only count messages.

## Core Integration Scenarios

### Bootstrap

1. Start static/dynamic/state publishers.
2. Start recorder.
3. Wait for bootstrap completion.
4. Verify initial image/joint/TF and `/tf_static` are present.
5. Verify recorder becomes paused and manifest reports required topics.

Repeat with publishers starting after recorder to test discovery.

Repeat without downstream Servo pose publishers. Bootstrap must still complete
because those active-required publishers are created lazily after first input.

### Basic Active Window

1. Keep the capture/deadman gate false for several seconds.
2. Publish capture gate true and action/state data for two seconds.
3. Publish capture gate false.
4. Continue state/image publication through post-roll and beyond.
5. Stop gracefully.

Verify:

- No high-volume standby interval is stored after bootstrap and before start.
- Start/end events and IDs are complete.
- Active data is present without unexpected gaps.
- Post-roll duration is within one scheduler/sample tolerance.
- Data after post-roll is absent.

### Rapid Re-press

Release and re-press before post-roll expires.

Verify:

- One continuous capture window.
- Two action-segment IDs.
- No writer pause/resume between them.
- Both release and new start transitions are represented.

### Lost Capture-Gate Stream

Stop publishing while the capture/deadman gate is true.

Verify:

- Watchdog creates one `gate_stale` segment end.
- Post-roll completes and writer pauses.
- Recorder remains healthy and can record a later new segment.

### Missing Optional Topic

Do not publish `/vive/raw_input_json` or `/servo_node/status`.

Verify recorder is ready but manifest reports optional topics as absent.

### Missing Required Topic

Do not publish a bootstrap-required camera or joint-state topic.

Verify bootstrap timeout follows configured required-topic policy and the
session cannot be marked fully valid.

### Missing Active-Required Topic

Complete bootstrap, activate the capture gate, but omit one downstream action
publisher.

Verify capture starts, the missing action stream is reported promptly, and the
window cannot be classified as a valid training segment.

### Source/Type Change

Simulate a source identity change and, where practical, topic type mismatch.

Verify a source boundary or invalidation event is emitted. Never merge two
operators into one unlabelled segment.

## Bag Inspection Tests

After every synthetic scenario, validate through a direct reader rather than
only checking process exit status.

Required checks:

- Storage plugin and metadata are readable.
- All expected topics have expected types.
- Required topic message counts are non-zero in every active window.
- First/last timestamps are inside expected window bounds.
- `/tf_static` exists and reconstructs required frame paths.
- Camera messages decode and sequence pixels match expected values.
- Joint names and sequence-derived values match.
- Pose and quaternion fields are finite and normalized within tolerance.
- Event IDs form valid, non-overlapping relationships.
- Bag and external event index agree.
- Manifest file list and checksums match disk contents.

Use `ros2 bag info` as a human diagnostic, not as the only validator.

## QoS Tests

Run synthetic publishers with the QoS profiles observed on the real graph.

- Best-effort camera and joint-state publishers must be received.
- Transient-local `/tf_static` published before recorder startup must be
  received during bootstrap.
- Event/gate topics must not silently mismatch reliability/durability.
- The manifest must distinguish requested and offered QoS.

Test at least one intentional incompatible profile and verify startup reports a
useful failure rather than a generic missing-topic timeout.

## Storage and Shutdown Tests

### Graceful Shutdown

- Stop while paused.
- Stop while recording.
- Stop during post-roll.
- Stop during bootstrap.

Every case must close the writer, finalize metadata, and produce a terminal
manifest status.

### Forced Termination

Terminate the container without grace during active writing. On restart:

- The previous output is never appended to.
- Recovery/reindex behavior is documented and tested for selected storage.
- Recovered data is marked interrupted.
- A new fragment receives a new ID/path.

### Disk Exhaustion

Use a size-limited test filesystem or configured threshold.

- Low-space threshold produces warning before writer failure.
- Recorder closes/fails cleanly when reserve is crossed.
- Teleoperation/synthetic control publishers continue at expected rates.
- Session is marked incomplete/invalid.

### Slow Storage

Throttle or simulate writer delay.

- Track cache pressure and dropped messages.
- Gate callbacks remain responsive.
- Control publishers/subscribers are unaffected.
- Recorder reports degradation rather than presenting a valid dataset.

### Permission Failure

Mount output read-only and verify failure occurs before capture with a clear
path/permission diagnostic.

## Performance and Isolation Tests

Measure the same controlled motion workload with recorder disabled and enabled.

Collect:

- Camera receive/publish rate.
- `/vive/hand_target_pose`, capture gate, and effective-action rate.
- Pose-target and twist-command rates.
- Command-to-measured-state latency distribution.
- CPU, memory, disk write throughput, and recorder cache pressure.
- DDS lost/dropped samples where observable.

Set acceptance thresholds from a recorder-disabled baseline. At minimum:

- Recorder must not introduce control-topic stalls.
- Gate release/timeout behavior must remain within existing control timing.
- Memory must remain bounded during a long active window.
- Disk throughput must sustain measured camera plus state/action bandwidth with
  margin.

Do not declare a universal latency percentage before measuring baseline jitter
on the target workstation.

## Clock and Alignment Validation

For each production-like session:

- Record robot-host clock offset before and after.
- Verify header timestamps do not jump backward.
- Compare source sequence, gateway receipt, bag time, and command time.
- Compute per-stream delay and jitter distributions.
- Detect missing or duplicate client sequences.
- Verify images, joint states, TF, and actions overlap every active window.

Create a known synthetic motion pattern and prove the exporter associates the
correct action/state/image sequences. Visual inspection alone is insufficient.

## Data Semantics Tests

### Action vs Outcome

Produce cases where:

- Command and motion both occur.
- Non-zero command produces delayed motion.
- Non-zero command produces no motion.
- Zero command follows deadman release.
- Workspace constraint changes target.

Verify the dataset keeps intended target, executable command, and actual state
separate. The exporter must not replace command with observed state delta.

### Deadman and Stop Behavior

- Preserve effective and operator gates separately.
- Preserve the release sample and halt/zero commands.
- Confirm post-roll contains measured settling state.
- Confirm the exporter can label active, terminal, and selected idle samples.

### Gripper and Head Independence

Run a gripper-only action while wrist deadman is false.

- `deadman_window` should either omit it as documented or capture it only if it
  falls inside an existing post-roll.
- `manual_episode` must preserve it.

This test prevents future code from claiming deadman windows are complete
pickup episodes.

### Task Result

- New task episodes default to `unknown`.
- Success/failure annotation is explicit and versioned.
- Aborted/system-fault demonstrations cannot be included as success by default.
- Export can filter by result while preserving raw source data.

## Real-Robot Acceptance Procedure

Use a controlled workspace and short session.

1. Verify recording root, free space, and session ID.
2. Verify host/robot clock status.
3. Start stack with recorder enabled but not required.
4. Confirm recorder reaches `ready-paused` before Unity input.
5. Hold deadman and make a small wrist motion.
6. Release and wait beyond post-roll.
7. Operate head and gripper in documented test combinations.
8. Stop stack normally and wait for bag finalization.
9. Disconnect from the robot ROS domain.
10. Run offline validation and observation-only inspection.
11. Confirm camera, wrist TF, joint state, command stages, gripper state, and
    event boundaries align.
12. Mark task result manually and rerun validation.

Never validate by playing unrestricted command topics back onto the live robot.

## Soak Test

Before production collection, run at least one session long enough to exceed
the expected demonstration duration and include many press/release cycles.

Verify:

- Memory does not grow per segment.
- File handles and threads remain stable.
- IDs remain unique.
- Bag splitting, if enabled, preserves event/context relationships.
- Shutdown duration stays within Compose grace period.
- Final validation time and checksum generation are operationally acceptable.

## Session Validity Levels

Suggested classification:

### `valid`

All required streams, timing, events, storage integrity, and annotations needed
for the selected export are present.

### `valid_with_warnings`

Core streams are complete but optional context or non-critical metadata is
missing.

### `quarantined`

Data may be recoverable, but clock offset, stream gaps, interrupted storage, or
source ambiguity exceeds policy. Exclude from training by default.

### `invalid`

Required observations/actions are missing, storage is unreadable, event
boundaries are contradictory, or task labels are knowingly wrong.

Never delete invalid raw sessions automatically. Retention policy should decide
when they are removed after diagnosis.

## Production Dataset Acceptance Checklist

A session is eligible for export only when:

- Bag and metadata open successfully.
- Checksums match.
- Manifest schema/version is supported.
- Session/source identity is unambiguous.
- Required topic types and QoS are as expected.
- Required joint names and frame paths exist.
- Camera frames decode and camera calibration is available when required.
- Action segments have complete start/end events.
- Required streams cover active windows within gap limits.
- Numeric pose/action data is finite and quaternions are valid.
- Clock offset/drift is inside configured limits.
- Task result policy is satisfied.
- Export configuration and exporter version are recorded.

## Minimal Automation Deliverables

The first implementation should include these commands or equivalents:

```text
scripts/check-data-recorder.sh
  Verify image/package/config/output prerequisites.

scripts/test-data-recorder.sh
  Run synthetic publishers, gate sequence, shutdown, and bag validation.

tools/dataset/validate_session.py <session-dir>
  Produce validation.json and non-zero exit for invalid sessions.

tools/dataset/export_session.py <session-dir> <output-dir>
  Perform deterministic observation/action alignment without ROS playback.
```

Keep hosted CI limited to fast unit/config checks. Run container integration
locally, and keep real-robot acceptance manual.
