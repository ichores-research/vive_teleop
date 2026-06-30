# Code Readability and Maintainability Review

## General strengths

- Names usually describe domain intent rather than implementation detail.
- Early returns keep many callbacks readable.
- Safety-relevant transitions have comments and logs.
- Configuration is mostly externalized into ROS parameters and YAML.
- The arm path retains only the latest target rather than replaying a queue.
- Math helpers are small enough to unit test.
- Shell entry points generally use `set -euo pipefail` and quote variables.
- Architecture sources are versioned alongside rendered diagrams.

The central maintainability problem is not poor formatting. It is that state,
transport, transformations, ROS I/O and lifecycle are often implemented in the
same class. That makes changes hard to verify without hardware.

## `moveit_server`

### `arm_movement.py`

Good:

- `_on_hand_target_active`, `_reset_hand_target_pursuit` and
  `_maybe_send_latest_target` make the deadman lifecycle visible.
- The controller and robot anchors are separate and named clearly.
- A failed TF lookup leaves the pending target available for a later timer tick.
- Workspace clipping is centralized rather than scattered across callbacks.

Recommendations:

1. Replace `ArmMovementMixin` with composition. A mixin that assumes dozens of
   host attributes has an undocumented interface and requires `object.__new__`
   test construction. Use `ArmTeleopController` for pure state/math and a ROS
   adapter for publishers, TF and services.
2. Move quaternion/vector helpers into `teleop_math.py`; use immutable tuples or
   dataclasses internally and convert only at ROS boundaries.
3. Represent deadman behavior as an explicit enum state: `INACTIVE`,
   `WAITING_FOR_ANCHOR`, `WAITING_FOR_SERVO_MODE`, `TRACKING`, `HALTING`,
   `FAULTED`. Current booleans permit unclear combinations.
4. Do not publish the active gate before validating the first controller pose.
5. Make Servo topics parameters/remaps rather than constants.
6. Distinguish “clamped” from “rejected” targets in status output.
7. Validate orientation components, final pose and parameter vectors as finite.
8. Add a minimum radial workspace, per-axis bounds and configurable orientation
   limits if required by the real workcell.
9. Explain frame multiplication order with a short equation and golden-vector
   test; this is interview-critical robotics logic.
10. Replace repeated `hasattr` lazy-interface checks with construction-time
    interfaces unless measured startup behavior requires laziness.

### `servo_pose_bridge.py`

Good:

- Feedback and feed-forward are visibly separated.
- Vector magnitude clamping preserves direction.
- Quaternion error chooses the shortest path.
- Target timeout and repeated halt messages are independent safeguards.
- Target-velocity reset after a gap avoids carrying stale feed-forward state.

Recommendations:

1. Split into `PoseTrackingController` (pure math/state), `ServoLifecycle`, and
   `ServoPoseBridgeNode` (ROS I/O).
2. Use a parameter dataclass validated once at startup. Reject non-finite gains,
   nonsensical rates, negative deadbands and timeouts inconsistent with the
   incoming command rate.
3. Sample `now` once per update instead of multiple clock reads.
4. Add TF freshness and future-skew checks.
5. Add acceleration/jerk limiting or document why downstream Servo output is
   sufficient. Unlimited feed-forward makes this especially important.
6. Publish controller diagnostics: pose error norm, command norm, target age,
   TF age, feed-forward active, clamp ratio and halt reason.
7. Store the timer handle; explicit ownership improves tests and shutdown.
8. Handle `KeyboardInterrupt`/external shutdown consistently with the other node.
9. Avoid `Optional`/`Dict` legacy typing in new code; the project already uses
   Python 3.10 unions elsewhere.
10. Define whether command stamps or receive time drive velocity estimation.
    Receive time is robust to unsynchronized clients but includes network jitter;
    source time requires clock alignment and validation.

### `vive_moveit_server.py` and `teleop_data.py`

Good:

- `TeleopDataReceiver` gives subscription wiring a narrow home.
- Head and gripper publishing are straightforward and readable.
- Gripper duration honors a physical velocity limit.
- Head commands use bounded pan/tilt and a deadband.

Recommendations:

1. Split head and gripper into independent controller classes/nodes. Arm,
   gripper and head should not share one mutable node merely because they share
   an input source.
2. Replace dictionaries of parameter names with typed configuration objects and
   parameter descriptors/ranges.
3. Make fallback values identical to deployed YAML or fail when a required
   deployment parameter is absent.
4. Validate `JointState.name`/`position` length and finite values; track sample age.
5. Add head-input freshness and controller availability diagnostics.
6. Derive head limits from the robot model when practical instead of duplicating
   hardware values in application YAML.
7. Add velocities/accelerations only if the physical controller contract expects
   them; otherwise document why position-only points are correct.
8. Rename `TeleopDataReceiver` to describe that it owns ROS subscriptions, not
   data, and retain subscription handles explicitly.
9. Remove `arm_group` from the node if it is log-only, or validate it against the
   Servo group so configuration cannot claim one thing while controlling another.

### Launch files and configuration

- The SRDF seven-joint validation is a strong fail-fast check.
- `vive_moveit_server.launch.py` is long because it forwards many optional TIAGo
  arguments. Replace repetitive declarations with a helper-generated list and
  tests that inspect the launch description.
- Keep one authoritative source for `arm`, frames, topics and rates. Generate
  documentation tables from it.
- Provide named profiles: simulation, conservative real robot, experimental
  performance and replay/no-output.
- Add parameter descriptors and startup summaries with the effective values.

## `webrtc_server`

### `webrtc_server.py`

Good:

- Signaling is separated from ROS nodes.
- Peer connections are tracked and closed on shutdown.
- ICE candidate summaries are useful operational diagnostics.
- Video and input routes share a generic offer flow.

Recommendations:

1. Add typed request/response models and explicit HTTP error mapping.
2. Separate public config, liveness, readiness and authenticated session routes.
3. Replace `print` with structured logging and stable event IDs.
4. Add peer IDs and log correlation across offer, ICE, channel and close events.
5. Add offer/request timeout, request-size policy and per-IP/session rate limits.
6. Close disconnected peers for `disconnected` after a grace period, not only
   `failed`/`closed`.
7. Make CORS an injected configuration with a deny-by-default deployment value.
8. Avoid a module-import-time ICE configuration singleton; construct it at
   application startup after validating environment variables.
9. Attach input-channel open/close handlers to command-authority lifecycle.
10. Test cleanup when setup, remote description, answer creation, local
    description or ICE gathering fails.

### `input_publisher.py`

1. Rename the package from `image_listener`; it is now a teleoperation gateway.
2. Validate a versioned envelope before any field-level work.
3. Require actual JSON booleans. `bool("false")` must never become an active gate.
4. Centralize finite pose parsing and return reason-coded validation errors.
5. Parameterize all topics and define explicit command QoS.
6. Preserve source and receive timestamps plus sequence/client IDs.
7. Publish a raw validated envelope for recording/diagnosis only when enabled;
   never make control depend on that diagnostic topic.
8. Use throttled structured counters instead of silently dropping invalid JSON.
9. Consider one typed custom message rather than four loosely correlated topics,
   while retaining standard ROS messages at controller boundaries.

### `robot_state.py`, `image_subscriber.py`, `video_track.py`

- Replace global `ready` with per-capability readiness and stable error codes.
- Add finite checks and explicit age fields to the snapshot.
- Report last camera frame age and frame dimensions.
- Keep only the newest decoded image and measure decode/encode delay. If CPU is a
  bottleneck, investigate compressed transport or avoiding unnecessary BGR copies.
- Annotate thread ownership. ROS callbacks, aiohttp and the video event loop run
  on different threads; locks/queues should make crossings explicit.
- Make frame delivery cancellation-safe when the peer closes.
- Add typing to callbacks and avoid broad `except Exception` without counters or
  traceback context.

## Browser client

The UI is useful and more complete than a throwaway debug page. Its primary
problem is that protocol code, connection lifecycle and DOM rendering live in a
single 1,303-line HTML file.

Recommended split:

```text
web/
  index.html
  styles.css
  src/config.js
  src/protocol.js
  src/peer.js
  src/input-state.js
  src/teleop-controller.js
  src/ui.js
  test/*.test.js
```

Additional changes:

- explicit hold-to-command button and visible lease owner;
- capability-specific readiness and stale-state banners;
- `AbortController` timeouts for fetch/signaling;
- proper ICE credentials from `/config`, not only TURN URLs;
- fallback `new MediaStream([event.track])` when `event.streams` is empty;
- bounded logs and disabled raw-payload logging by default;
- schema-generated payload builders and unit tests;
- reconnect/backoff that never restores command-active state automatically;
- no external Google STUN dependency unless explicitly configured.

## Unity client

`ViveTeleopWebRtcClient` demonstrates substantial work, but it currently has at
least nine responsibilities. Suggested classes:

```text
ViveTeleopCoordinator
ServerConfigClient
WebRtcVideoClient
WebRtcInputClient
TeleopProtocolEncoder
HeadPoseSource
OpenVrControllerSource / UnityXrControllerSource
WristWorkspaceMapper
GripperGestureController
CommandGateStateMachine
JsonlDiagnosticRecorder
```

Place these in an assembly definition and add EditMode tests for pure mapping,
calibration and protocol code. Add PlayMode tests for connection lifecycle with
a fake signaling server.

Other recommendations:

- remove hard-coded private IP defaults from code and scene;
- use `WaitForSecondsRealtime` for control sampling;
- guard against concurrent `Connect()` calls and cancel in-flight requests;
- make offer routines return success/failure and clean up failed peers;
- set HTTP timeouts and authenticate signaling;
- use `double` or integer nanoseconds for time, not a long-running float;
- add sequence and session identity;
- avoid synchronous recording flushes on the main thread;
- unsubscribe video callbacks and release material/track resources explicitly;
- replace sample OpenVR action names with a project-specific action manifest;
- expose haptic feedback through an interface so safety/status events can use it;
- validate inspector configuration in `OnValidate` and fail visibly at startup.

## Shell, Docker and repository

- Shell scripts are generally better structured than the top-level `test.sh`.
- Extract common path/log/network helpers into a sourced library tested with
  Bats, or keep scripts independent but run ShellCheck and shfmt.
- Avoid nested Bash/Python inside quoted `docker exec`; install a proper health
  checker in the image and invoke it.
- Add transactional cleanup, run manifests and a `doctor` command.
- Separate Compose profiles for development bind mounts and immutable runtime.
- Add `.dockerignore`, non-root runtime users, health checks, stop grace periods,
  resource/log limits and pinned image identities.
- Remove generated artifacts and unused vendor samples; commit lockfiles.
- Add a root `LICENSE`, accurate package metadata and any required third-party
  notices before presenting the repository publicly.
