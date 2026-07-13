# Future Teleoperation Dataset Recorder

## Status

The initial online recorder is implemented under `data_recorder/`. It provides
an embedded Humble rosbag2 recorder, MCAP storage, deadman-window and continuous
modes, watchdog/post-roll handling, typed events, per-camera-frame deadman
labels, a manifest/event index, and opt-in Compose integration. The offline
validator/exporter and live-robot topic/QoS inventory are still pending, so a
hardware session must be inspected before treating a bag as production-valid.

The implemented privacy boundary is stricter than the earlier provenance
proposal: `/vive/hand_target_active` is the only operator-input topic allowed in
the bag. Raw JSON, poses, axes, buttons, head input, gripper input, and base
input are not recorded. The compact default archive contains only the head RGB
view/calibration, robot motion, effective robot commands, and recorder labels;
depth/IR, point clouds, IMU, laser, force, health, power, and statistics are
excluded.

The existing Unity client can write local JSONL controller samples, but those
files are not synchronized rosbag datasets and do not contain complete robot
state or camera data.

## Purpose

The future subsystem will record demonstrations for machine learning while
keeping data capture isolated from robot control. The target dataset must answer
four questions:

1. What did the robot observe?
2. What robot-space action was requested or executed?
3. What state resulted from that action?
4. Which samples belong to a valid action segment and successful task attempt?

Raw operator/controller motion is useful provenance but is not the primary
training action. The primary label should be the robot-space action at the same
interface the future policy will control.

## Decisions

- Add a separate observational recorder container.
- Keep one recorder alive for the whole application session.
- Use the upstream wrist deadman to open capture before robot-space commands
  arrive; record the downstream Servo gate as the effective-action mask.
- Do not start a new recorder process or bag for each button press.
- Preserve a post-roll and explicit boundaries around every active segment.
- Keep selected idle/terminal samples so a policy can learn stopping behavior.
- Record an explicit whitelist rather than every discovered ROS topic.
- Record both robot commands and measured robot outcomes.
- Store immutable bags first; build training tensors/tables in an offline export
  step.
- Treat one deadman hold as an action segment, not automatically as a complete
  pickup episode.
- Keep data recording independent from teleoperation availability and safety.

## Documents

- `architecture-and-lifecycle.md`: proposed deployment, states, trigger logic,
  failure isolation, startup, shutdown, and operational modes.
- `dataset-contract.md`: exact current topic chain, proposed topic whitelist,
  ML sample definition, identity, time alignment, metadata, and storage layout.
- `implementation-plan.md`: phased code/config changes with target repository
  paths and Humble-specific implementation constraints.
- `validation-plan.md`: unit, integration, replay, storage, timing, and dataset
  acceptance tests.
- `.agents/data-recorder-context.md`: condensed implementation context for
  future coding agents.

## External References

- ROS 2 Humble bag tutorial:
  <https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html>
- ROS 2 Humble recorder API, including pause/resume:
  <https://docs.ros.org/en/ros2_packages/humble/api/rosbag2_transport/generated/classrosbag2__transport_1_1Recorder.html>
- ROS 2 rosbag QoS overrides:
  <https://docs.ros.org/en/humble/How-To-Guides/Overriding-QoS-Policies-For-Recording-And-Playback.html>
- ROS 2 Humble MCAP storage plugin:
  <https://docs.ros.org/en/humble/p/rosbag2_storage_mcap/index.html>
- Behavioral Cloning from Observation, explaining the additional machinery
  needed when expert action labels are absent:
  <https://arxiv.org/abs/1805.01954>
