# Prioritized Findings

## Priority definitions

- **P0**: address before expanding command authority, allowing untrusted network
  access, or describing the system as safe.
- **P1**: high-value correctness, verification and portfolio work for the next
  stable milestone.
- **P2**: maintainability, reproducibility and operational improvement.
- **P3**: polish or optional future capability.

## P0 findings

### VT-001 — Command endpoints have no authentication or operator ownership

Evidence:

- [`webrtc_server.py` lines 123-132](../../webrtc_server/src/image_listener/image_listener/webrtc_server.py#L123)
  enables wildcard CORS with credentials.
- [`webrtc_server.py` lines 157-167](../../webrtc_server/src/image_listener/image_listener/webrtc_server.py#L157)
  accepts every data channel and forwards every message.
- [`docker-compose.wifi.yml` lines 9-29](../../docker-compose.wifi.yml#L9)
  exposes the service on the host network.

Impact: any reachable browser or WebRTC peer can create an input connection and
publish robot commands. Multiple peers can command concurrently. A malicious
web page opened on the same network may also reach the permissive CORS endpoint.

Recommendation:

1. Require an authenticated session token for signaling and input setup.
2. Add a single command-authority lease with `client_id`, `session_id`, expiry,
   explicit acquire/release and server-enforced exclusivity.
3. Restrict CORS to the known debug-client origins; disable credentials unless
   they are actually needed.
4. Reject command messages from peers without the active lease.
5. Publish an explicit inactive gate when the lease expires or its channel closes.
6. Treat TLS or a trusted isolated network as a deployment requirement.

Acceptance criteria: a second client cannot command while the first holds the
lease; unauthorized offers return 401/403; channel loss clears all active gates;
an automated test covers acquire, conflict, expiry and reconnect.

### VT-002 — Browser input implicitly enables arm pursuit

Evidence:

- [`index.html` lines 1133-1155](../../index.html#L1133) builds wrist fields but
  omits `wristCommandEnabled`.
- [`input_publisher.py` lines 122-131](../../webrtc_server/src/image_listener/image_listener/input_publisher.py#L122)
  interprets the missing field as `true`.
- [`test_input_publisher.py` lines 33-39](../../webrtc_server/src/image_listener/test/test_input_publisher.py#L33)
  codifies that legacy fallback.

Impact: clicking **Input** begins an active arm stream without an explicit hold
action. The starting pose is designed not to move the arm, but editing a field
then commands motion under an implicit, persistent deadman.

Recommendation: make the field mandatory in protocol version 2, default it to
false, add a hold-to-command control in the browser, and remove the legacy
active-by-default test after a deprecation window. The server must reject a
wrist-bearing v2 command that omits the field.

Acceptance criteria: opening input never makes `/vive/hand_target_active` true;
only holding the UI deadman does; mouse-up, blur, visibility loss, channel close
and unload each publish or synthesize false.

### VT-003 — Non-finite poses can pass quaternion validation

Evidence:

- [`input_publisher.py` lines 163-198](../../webrtc_server/src/image_listener/image_listener/input_publisher.py#L163)
  converts positions without checking `isfinite`.
- [`input_publisher.py` lines 217-226](../../webrtc_server/src/image_listener/image_listener/input_publisher.py#L217)
  accepts a NaN norm because `NaN < threshold` is false.
- The same pattern exists in
  [`arm_movement.py` lines 21-36](https://github.com/ichores-research/vive_teleop/blob/0fb7718f66f2fbfb8f1029233b7a7547dbcc120d/moveit_server/src/vive_moveit_server/vive_moveit_server/arm_movement.py#L21-L36),
  [`servo_pose_bridge.py` lines 15-30](https://github.com/ichores-research/vive_teleop/blob/0fb7718f66f2fbfb8f1029233b7a7547dbcc120d/moveit_server/src/vive_moveit_server/vive_moveit_server/servo_pose_bridge.py#L15-L30),
  and [`vive_moveit_server.py` lines 48-63](https://github.com/ichores-research/vive_teleop/blob/0fb7718f66f2fbfb8f1029233b7a7547dbcc120d/moveit_server/src/vive_moveit_server/vive_moveit_server/vive_moveit_server.py#L48-L63).

Impact: NaN or infinity may propagate into targets, feedback error and twist
commands. Downstream behavior is controller-dependent and must not be relied on.

Recommendation: centralize finite vector/quaternion validation, reject every
non-finite component before normalization, and validate calculated deltas and
final outgoing commands as a last barrier.

Acceptance criteria: property/fuzz tests cover NaN, positive/negative infinity,
zero norm, huge finite values and malformed JSON; no invalid input publishes a
pose, gate, trajectory or twist.

### VT-004 — Servo feedback accepts an arbitrarily stale wrist transform

Evidence: [`servo_pose_bridge.py` lines 317-331](https://github.com/ichores-research/vive_teleop/blob/0fb7718f66f2fbfb8f1029233b7a7547dbcc120d/moveit_server/src/vive_moveit_server/vive_moveit_server/servo_pose_bridge.py#L317-L331)
looks up the latest transform but does not inspect its timestamp. The snapshot
API does perform an age check in
[`robot_state.py` lines 223-243](../../webrtc_server/src/image_listener/image_listener/robot_state.py#L223).

Impact: if TF publication stalls while the transform remains buffered, the
feedback controller can continue calculating velocity against stale state.

Recommendation: add `max_tf_age_sec`, reject future-skewed and stale dynamic
transforms, clear target state, emit zero commands, and publish a reason-coded
status. Decide explicitly how zero-stamped static transforms are treated.

Acceptance criteria: an integration test freezes TF while target messages
continue and verifies that command output reaches zero within the configured
bound.

### VT-005 — The default arm profile removes two important motion bounds

Evidence:

- [`servo_pose_bridge.yaml` lines 15-18](https://github.com/ichores-research/vive_teleop/blob/0fb7718f66f2fbfb8f1029233b7a7547dbcc120d/moveit_server/src/vive_moveit_server/config/servo_pose_bridge.yaml#L15-L18)
  disables linear and angular caps.
- [`tiago_servo.yaml` line 42](../../moveit_server/src/vive_moveit_server/config/tiago_servo.yaml#L42)
  disables collision checking.
- [`check-teleop-runtime.sh` lines 57-73](../../scripts/check-teleop-runtime.sh#L57)
  fails unless the bridge caps are disabled.

Impact: Servo still applies joint-limit and singularity scaling, but those are
not substitutes for Cartesian speed limits or collision avoidance. The runtime
check currently enforces the risk rather than merely reporting it.

Recommendation:

- create a conservative `lab_safe` profile with measured Cartesian caps and
  collision checking enabled;
- retain an explicitly named `performance_experimental` profile only if the lab
  requires it;
- require an acknowledgement environment flag to launch the experimental profile;
- measure tracking quality before deciding that proximity scaling is unusable;
- investigate collision-scene quality and thresholds instead of treating the
  global disable as the final solution.

Acceptance criteria: the ordinary start command selects bounded behavior; the
runtime report prints the active safety profile; an experimental launch is
impossible without an explicit flag; maximum observed velocity is tested.

### VT-006 — Input-channel loss does not immediately release command state

Evidence:

- [`webrtc_server.py` lines 157-166](../../webrtc_server/src/image_listener/image_listener/webrtc_server.py#L157)
  registers only a message handler.
- [`ViveTeleopWebRtcClient.cs` lines 455-459](../../unity-vr-headset/Assets/ViveTeleopWebRtcClient.cs#L455)
  only changes local state on close.
- [`ViveTeleopWebRtcClient.cs` lines 193-207](../../unity-vr-headset/Assets/ViveTeleopWebRtcClient.cs#L193)
  closes without sending a release.

Impact: the 120 ms ROS timeout is currently the final fallback. That is useful,
but a known close event should release immediately and provide an observable
reason.

Recommendation: command ownership belongs on the server. On channel close,
peer failure, lease expiry or parse-failure threshold, publish false for every
gate owned by that peer. Keep the downstream timeout as an independent guard.

### VT-007 — The debug-client server exposes the repository over Wi-Fi

Evidence: [`serve-debug-client.sh` lines 9-15](../../scripts/serve-debug-client.sh#L9)
binds `python3 -m http.server` to `0.0.0.0` from the repository root.

Impact: clients may retrieve source files, `.git` objects and the ignored local
`.env` file. This is unnecessary exposure on the operator network.

Recommendation: serve a dedicated static directory containing only built web
assets, or serve the page from the gateway with a strict route set. Add cache
and security headers. Never use the repository root as a web root.

Acceptance criteria: requests for `/.git/HEAD`, `/.env`, Docker files and source
paths return 404.

## P1 findings

### VT-008 — No versioned command schema, sequence number or source timestamp

The flat payload in [`ViveTeleopWebRtcClient.cs` lines 1688-1739](../../unity-vr-headset/Assets/ViveTeleopWebRtcClient.cs#L1688)
has a `type` and float timestamp but no schema version, sequence, stable client
identity or session identity. The gateway replaces the timestamp with receive
time in [`input_publisher.py` line 189](../../webrtc_server/src/image_listener/image_listener/input_publisher.py#L189).

Define a versioned schema with strict types, `client_id`, `session_id`, monotonic
`sequence`, source monotonic time, source wall/UTC time, receive time, capability
flags and explicit command gates. Preserve raw envelopes for diagnosis/recording.

### VT-009 — Command QoS is implicit and can queue stale samples

Command publishers use integer depth 10 or system defaults, for example
[`input_publisher.py` lines 23-42](../../webrtc_server/src/image_listener/image_listener/input_publisher.py#L23)
and [`arm_movement.py` lines 261-270](https://github.com/ichores-research/vive_teleop/blob/0fb7718f66f2fbfb8f1029233b7a7547dbcc120d/moveit_server/src/vive_moveit_server/vive_moveit_server/arm_movement.py#L261-L270).
For streaming teleoperation, explicitly choose QoS. Evaluate reliable versus
best-effort on the actual network, but prefer keep-last depth 1, volatile
durability and a lifespan/deadline compatible with the command timeout so old
commands cannot build a queue.

### VT-010 — Gripper control uses joint positions without freshness tracking

[`vive_moveit_server.py` lines 327-397](https://github.com/ichores-research/vive_teleop/blob/0fb7718f66f2fbfb8f1029233b7a7547dbcc120d/moveit_server/src/vive_moveit_server/vive_moveit_server/vive_moveit_server.py#L327-L397)
caches positions indefinitely and uses them to suppress/size trajectories.
Track receipt time, validate finite values, reject stale state, and report why
a command was rejected.

### VT-011 — The browser and Unity disagree about readiness

The browser requires `snapshot.ready` in [`index.html` lines 1001-1034](../../index.html#L1001).
Unity intentionally continues with a valid wrist even when `ready` is false in
[`ViveTeleopWebRtcClient.cs` lines 366-401](../../unity-vr-headset/Assets/ViveTeleopWebRtcClient.cs#L366).
The technical guide says the client will not open in that case. Replace one
global boolean with per-capability readiness (`video`, `head`, `wrist`,
`gripper`, future `base`) and make both clients enforce the same contract.

### VT-012 — Robot snapshot values are not comprehensively validated

[`robot_state.py` lines 131-145](../../webrtc_server/src/image_listener/image_listener/robot_state.py#L131)
stores joint positions without finite checks and
[`robot_state.py` lines 37-52](../../webrtc_server/src/image_listener/image_listener/robot_state.py#L37)
serializes TF without validating translation or quaternion norm. Reject invalid
samples and expose per-capability freshness and error codes.

### VT-013 — Runtime health is configuration-oriented, not end-to-end

`/healthz` is currently an alias for config in
[`webrtc_server.py` lines 146-149](../../webrtc_server/src/image_listener/image_listener/webrtc_server.py#L146).
The startup script checks camera publisher count but not receipt of a frame.
Add liveness and readiness endpoints with last-frame age, joint/TF age, command
lease status, ROS publisher/subscriber counts and recorder state. Keep liveness
independent of robot availability so container restart policy is meaningful.

### VT-014 — No observable control-status contract

The system mainly communicates faults through logs. Publish a typed status
message containing active source, gate state, last command age, TF age,
workspace clipping, Servo status, collision profile, rejection counters and
last halt reason. This is needed for UI, bags, incident analysis and tests.

### VT-015 — Current tests cover only a narrow slice of behavior

The 15 tests pass, but there are no tests for head conversion, gripper
freshness, workspace limits, NaN handling, TF age, WebRTC offers, ICE cleanup,
multi-client arbitration, robot snapshots, browser protocol, Unity transforms,
launch composition or process shutdown. See the dedicated testing document.

### VT-016 — No CI or enforced format/lint policy

There is no `.github/workflows` tree, `ament_*` test dependency, Ruff/Black
configuration, ShellCheck step, Unity test job or Markdown link check. Add a
fast pull-request pipeline and a slower image/integration pipeline.

### VT-017 — Public default branch does not show the audited portfolio state

The local and remote default is `main`, while the polished README and renamed
gateway are on `qa`. Merge through a reviewed PR, set branch protection, and
verify the anonymous GitHub landing page. This is a portfolio blocker rather
than a runtime defect.

### VT-018 — Package metadata and repository licensing are incomplete

Both package manifests identify the maintainer as `you`, and the gateway remains
named/described as a simple `image_listener`; see
[`webrtc package.xml` lines 3-8](../../webrtc_server/src/image_listener/package.xml#L3)
and [`moveit package.xml` lines 3-8](../../moveit_server/src/vive_moveit_server/package.xml#L3).
Add a root `LICENSE`, correct identities, meaningful descriptions, test
dependencies, repository URL and explicit third-party notices.

### VT-019 — The repository contains generated and accidental artifacts

Tracked files include two `.pyc` files, `mono_crash.mem.*.blob`,
`UpgradeLog.htm`, and `AutosavedBeforeTeleop.unity`. Remove them from the index
and adopt the maintained GitHub Unity ignore patterns. Ignoring a file does not
untrack an already committed file.

### VT-020 — Unity dependency management is unusually large and not locked

1,519 of 1,729 tracked files are under `Assets/SteamVR`. The full interaction
samples and example action sets are used to obtain a small number of actions.
Meanwhile `Packages/packages-lock.json` is ignored. Retain license-compliant,
necessary SDK material only, define a project-specific action set, remove
unused template packages, and commit the package lock.

### VT-021 — Runtime images are mutable, rootful and development-heavy

Both Dockerfiles include compilers, Git, curl and nano in their final image;
Compose uses `coturn/coturn:latest`; services run as root and mount source over
built packages. Add `.dockerignore`, separate development and runtime targets,
pin base/image versions or digests, remove unnecessary tools, run non-root where
ROS/network constraints permit, add health checks and use read-only filesystems
and dropped capabilities where practical.

### VT-022 — TURN and signaling defaults are appropriate only for an isolated lab

`dummy:dummy`, plaintext HTTP signaling, and disabled TURN TLS/DTLS are repeated
across scripts, Compose and the browser. WebRTC media remains DTLS/SRTP protected,
but credentials, signaling and control authorization are not. Generate secrets,
avoid returning them to unauthenticated callers, and document the trust boundary.

### VT-023 — Data recording design is strong but unimplemented and untracked

The `docs/data-recording/` design correctly separates observation, intent,
accepted action, executable command and outcome. It was untracked at audit
time, and there is no recorder, message package, exporter or validator. Commit
the design first, then implement the phased MVP in the roadmap.

### VT-024 — Documentation contains measurable contradictions

The data-recording design says the pose bridge publishes at 50 Hz, while
[`servo_pose_bridge.yaml` line 9](https://github.com/ichores-research/vive_teleop/blob/0fb7718f66f2fbfb8f1029233b7a7547dbcc120d/moveit_server/src/vive_moveit_server/config/servo_pose_bridge.yaml#L9)
sets 100 Hz. The guide contains user-specific absolute paths and describes
readiness differently from Unity. Generate a configuration reference from
declared parameters or test documentation claims against loaded YAML.

## P2 findings

### VT-025 — Large files combine unrelated responsibilities

- `ViveTeleopWebRtcClient.cs`: 1,758 lines covering configuration, signaling,
  video, two input APIs, transforms, command state, gripper gestures, recording,
  haptics and DTOs.
- `index.html`: 1,303 lines combining style, DOM, signaling, interpolation,
  protocol and state initialization.
- `servo_pose_bridge.py`: 600 lines combining math, target estimation, service
  lifecycle, TF feedback and ROS I/O.
- `vive_moveit_server.py`: 447 lines combining parameter parsing, head control,
  gripper control and process lifecycle.

Split by stable responsibility and place math/state machines in ROS/Unity-free
units that can be tested cheaply.

### VT-026 — Quaternion and copy helpers are duplicated

Normalization, multiplication, inversion, copying and coordinate transforms are
reimplemented in multiple Python modules and both clients. Create one Python
math module and one protocol/transform specification with golden vectors shared
across Python, JavaScript and C# tests.

### VT-027 — Fallback defaults disagree with deployed YAML

`ViveMoveItServer` defaults `arm_group` to `arm_torso` at
[`vive_moveit_server.py` line 76](https://github.com/ichores-research/vive_teleop/blob/0fb7718f66f2fbfb8f1029233b7a7547dbcc120d/moveit_server/src/vive_moveit_server/vive_moveit_server/vive_moveit_server.py#L76),
while the deployed profile uses `arm`. Head signs and command duration also
differ between code defaults and YAML. Defaults should be conservative and
consistent, or required parameters should fail startup when absent.

### VT-028 — Topic names are only partly parameterized

The gateway constructor parameterizes two topics but hard-codes active and
gripper topics. Arm Servo topics are module constants. Use ROS parameters and
normal remapping consistently; group them in a protocol configuration object.

### VT-029 — Input parsing fails silently and lacks rate/size abuse controls

Invalid JSON returns `None` without a metric or throttled warning. Add schema
rejection counters, maximum useful payload size, per-peer rate limits and a
disconnect threshold for repeated invalid messages. Never log unlimited raw
payloads.

### VT-030 — Recording performs synchronous file flushes on the Unity main path

[`ViveTeleopWebRtcClient.cs` lines 1421-1444](../../unity-vr-headset/Assets/ViveTeleopWebRtcClient.cs#L1421)
writes and periodically flushes in the pose loop. Move file I/O behind a bounded
queue and background writer, record monotonic and UTC timestamps, and expose
dropped-sample counts. Rosbag2 should remain the authoritative synchronized
recording path.

### VT-031 — Startup does not provide transactional cleanup

If the combined launcher fails after Compose starts, it stops log followers but
leaves services running. Define whether that is intentional. Prefer an explicit
`--keep-services-on-failure` option and otherwise stop only services started by
that run. Store a run manifest with image IDs, Git commit and effective config.

### VT-032 — Generated CycloneDDS files use a predictable writable `/tmp` path

The Wi-Fi script can overwrite a symlink target at its default predictable path.
Always create a private `mktemp` file or runtime directory with restrictive
permissions, validate IP/boolean inputs before XML interpolation, and clean it
on exit after Compose no longer needs it.

### VT-033 — `test.sh` is not a repository test

The file has no shebang or strict mode and only lists hard-coded paths under
`/home/robot`. Rename it to a diagnostic script with arguments or delete it;
provide one documented `./scripts/test.sh` that actually runs project checks.

### VT-034 — Video and control performance are asserted but not measured

The visitor README says “low latency,” but the repository publishes no
glass-to-glass delay, input-to-command delay, command jitter, tracking error,
stop latency, packet-loss behavior or CPU usage. Rosbag-derived plots and a
repeatable benchmark would turn this claim into evidence.

### VT-035 — No simulation or replay path is available to external reviewers

The project requires specific TIAGo and Vive hardware. Add a fake-state/replay
profile that drives the gateway and controller against recorded or synthetic
topics without publishing to physical controllers. A portfolio reviewer should
be able to exercise protocol and state-machine behavior from a clean machine.

### VT-036 — The action manifest is derived from unrelated SteamVR samples

The client reads `/actions/platformer/in/Move` for teleoperation and ships
buggy, mixed-reality, teleport and skeleton actions. Define `/actions/teleop`
with explicit wrist deadman, gripper, base drive, recording and haptic actions.
This makes input intent auditable and avoids coupling production control to
sample content.

### VT-037 — Connection failures can leave partially initialized peers

Unity offer routines log and `yield break` without a success result or immediate
cleanup, while `ConnectRoutine` can continue to another subsystem. Model each
peer as a state machine with timeout, cancellation, retry/backoff and disposal.

### VT-038 — No data-channel backpressure or sequence handling exists

The clients send at fixed intervals without monitoring buffered amount; the
gateway does not detect gaps, duplicates or reordering. Add a monotonic sequence,
bounded sender queue, `bufferedAmountLowThreshold`, stale-message rejection and
metrics. Reliable ordered delivery does not eliminate queue-latency risk.

## P3 opportunities

- Add left-controller base driving only after P0 command ownership and safety.
- Add task/episode annotation and offline LeRobot/RLDS export.
- Add live Rerun/Foxglove plots for input, targets, state and gate reasons.
- Add optional haptic feedback for workspace clipping, singularity scaling,
  stale state, command rejection and gripper contact.
- Add a small C++ ROS 2 component where determinism matters, such as base command
  guarding or the rosbag2 recorder controller.
- Add simulation fixtures, benchmark releases and a short design report.
