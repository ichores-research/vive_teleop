# Rosbag2 and Driving Roadmap

## Recommendation

Implement recording before driving. Recording supplies the replay, timing and
failure evidence needed to add base motion responsibly. Driving without that
observability broadens the demo but does not deepen engineering quality.

The existing `docs/data-recording/` design is a strong specification. Preserve
its separation between observation, operator intent, accepted robot-space
target, executable command and measured outcome. Implement it in increments so
the first useful bag does not wait for the entire ML pipeline.

## Rosbag2 phases

### Phase R0 — commit and verify the design

- Commit the currently untracked recording documents.
- Confirm every proposed topic name, type, publisher, QoS and rate on the live
  TIAGo graph.
- Correct the documented bridge rate (configuration currently says 100 Hz).
- Decide whether Humble's installed rosbag2/MCAP versions support every planned
  API; test the actual image rather than rolling documentation.
- Record storage throughput for raw and compressed camera topics.

Deliverable: a checked-in topic/QoS inventory captured from the robot and a
short storage benchmark.

### Phase R1 — continuous diagnostic recorder

Start with a separate recorder service using a fixed whitelist and MCAP. Record
short sessions continuously. Include:

- camera image or selected compressed image;
- `/joint_states`, `/tf`, `/tf_static`;
- upstream active gate and wrist target;
- mapped pose target, executable twist and arm trajectory;
- head/gripper intent and trajectory;
- future teleop status/rejection events;
- `/rosout` only if privacy/storage policy permits.

Add QoS overrides, output volume, free-space preflight, split limits, graceful
30-second stop, lost-message statistics and a session manifest. The recorder
must never be in the control dependency chain.

Deliverable: start/stop script, one validated bag, `ros2 bag info`, manifest and
an automated reader test.

### Phase R2 — analysis and replay

Before complex trigger logic, make data useful:

- reader/validator for types, counts, timestamp order, finite fields and TF path;
- no-output replay launch;
- command/state alignment;
- tracking-error, rate/jitter and stop-latency plots;
- source-versus-receive timestamp comparison;
- report missing topics and lost-message events as hard validity failures where
  appropriate.

Deliverable: `analyze_session` produces JSON metrics and plots from one command.

### Phase R3 — action segments and task episodes

Implement the designed recorder controller and event messages. Keep one writer
for the session. Support:

- bootstrap capture for static/config context;
- pause/resume or snapshot behavior validated on Humble;
- arm action segments from explicit deadman transitions;
- post-roll;
- manual task episode start/end and outcome labels;
- restart fragments with unambiguous identity;
- atomic final manifest and checksums.

Do not equate one deadman hold with a complete task. A pick-and-place attempt
usually contains multiple arm holds, gripper commands and pauses.

### Phase R4 — dataset export

Export immutable bags offline into a versioned schema such as LeRobot/RLDS/Zarr
only after defining the future policy interface. Recommended initial action:
robot-relative end-effector delta/target plus gripper target and active mask.
Keep twists and joint trajectories as diagnostic/alternative labels.

Split datasets by session/task/operator, not random adjacent frames. Record
licenses, consent/privacy, object/task metadata and train/validation leakage
rules. Visualize random episodes before training.

## Driving architecture

### Input allocation

- Right controller: wrist deadman and gripper, as today.
- Left controller: base planar command.
- Left grip/trigger or a dedicated action: hold-to-drive deadman.
- Menu/system action: never overloaded with motion.
- Optional mode switch: only if physical controls cannot remain unambiguous.

Create a project-specific SteamVR `/actions/teleop` map rather than extending
the sample `platformer` map.

### Protocol

Add explicit fields or a typed nested command:

```text
schema_version
client_id / session_id / sequence
source_monotonic_time / source_utc_time
base.available
base.command_enabled
base.frame                 # base or captured-headset-yaw
base.linear_x
base.linear_y              # only if the TIAGo base/controller supports it
base.angular_z
```

The gateway publishes operator intent, not directly to the hardware controller.
Suggested ROS boundary:

```text
/vive/base_command          geometry_msgs/TwistStamped
/vive/base_active           std_msgs/Bool
/teleop/base_status         custom status
```

A dedicated `base_teleop_guard` validates the lease, state age, command age,
finite values and frame; applies deadzone, limits and acceleration smoothing;
then publishes to the verified TIAGo base controller topic.

### Base guard state machine

```text
INACTIVE
  -> ARMED       valid lease + fresh base state + deadman press
ARMED
  -> DRIVING     first valid command
DRIVING
  -> DRIVING     newest command, rate/acceleration limited
DRIVING/ARMED
  -> HALTING     release, timeout, lease loss, stale state, obstacle or fault
HALTING
  -> INACTIVE    zero output confirmed/published for configured interval
```

Never restore `DRIVING` after reconnect or state recovery without a new physical
deadman edge.

### Driving limits and collision path

Use separate ordinary/turbo scales only if turbo has its own explicit button.
Adopt Nav2 velocity-smoother concepts: per-axis max/min velocity, acceleration,
deceleration, deadband and velocity timeout. Put a collision monitor downstream
of smoothing and upstream of the controller when compatible with the TIAGo
safety stack. Confirm whether the physical base controller already implements
watchdogs and obstacle intervention.

Add an arm-extension interaction policy. A conservative first rule is:

- inhibit driving outside a tested wrist envelope, or
- scale base speed down continuously as reach/height increases.

Do not infer stability from “the command was accepted.” Test representative arm
poses with the robot owner.

## Recording changes required by driving

The current arm-deadman capture trigger would omit most base motion. Add manual
task episodes as the primary whole-body mode, or open a capture window when the
union of arm/base active gates is true while retaining separate segment IDs.

Record:

- raw left-controller axes and base deadman as optional provenance;
- `/vive/base_command` and `/vive/base_active` as intent;
- guarded/smoothed effective base command as the policy/action candidate;
- final controller command;
- `/odom`, base TF and localization pose;
- laser/depth/collision-monitor state when available;
- intervention, clipping, stale-state and halt reasons;
- arm-extension speed scale.

## Driving acceptance tests

1. Opening/reconnecting the client produces zero base command.
2. A second client cannot acquire command authority.
3. Releasing the deadman, closing the channel and removing Wi-Fi each stop within
   a measured bound.
4. NaN, infinity, extreme axes, duplicate/out-of-order sequences and stale
   timestamps produce zero output and a rejection reason.
5. Acceleration and velocity never exceed configured limits.
6. Obstacle intervention stops or limits motion as configured.
7. Arm-extension policy is enforced.
8. Recorder captures intent, effective command, odometry and halt event with
   aligned timestamps.
9. Replay in no-output mode reproduces the same guard decisions.

## Portfolio result

The strongest final demonstration is not merely “the joystick drives the base.”
Show one whole task: drive toward an object, look around, manipulate it, drive
away, and then show synchronized plots of command, odometry, wrist tracking and
deadman/guard events. That demonstrates mobile manipulation, human-interface
design, safety reasoning, data engineering and quantitative validation.
