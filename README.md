<div align="center">

<h1>Vive Teleop</h1>

<h3>VR teleoperation for the TIAGo mobile manipulator</h3>

<p>Look through the robot's camera, drive its base, guide its seven-axis arm,<br>
and move its head using a Vive headset and controller.</p>

<p>
  <img src="https://img.shields.io/badge/ROS_2-Humble-22314E?style=flat-square&logo=ros" alt="ROS 2 Humble">
  <img src="https://img.shields.io/badge/Unity-6-000000?style=flat-square&logo=unity" alt="Unity 6">
  <img src="https://img.shields.io/badge/Streaming-WebRTC-333333?style=flat-square&logo=webrtc" alt="WebRTC">
  <img src="https://img.shields.io/badge/Motion-MoveIt_Servo-2E7D32?style=flat-square" alt="MoveIt Servo">
  <img src="https://img.shields.io/badge/Runtime-Docker-2496ED?style=flat-square&logo=docker&logoColor=white" alt="Docker">
</p>

<p><a href="#demo">See the demo</a> · <a href="#how-it-works">How it works</a> · <a href="docs/technical-guide.md">Technical documentation</a></p>

</div>

## Demo

<table>
  <tr>
    <td align="center" width="50%">
      <a href="docs/assets/arm-teleop-demo.gif">
        <img src="docs/assets/arm-teleop-demo.gif" alt="Operator moving the TIAGo arm with a Vive controller" width="320">
      </a>
    </td>
    <td align="center" width="50%">
      <a href="docs/assets/head-teleop-demo.gif">
        <img src="docs/assets/head-teleop-demo.gif" alt="TIAGo matching the operator's headset movement" width="320">
      </a>
    </td>
  </tr>
  <tr>
    <td align="center"><strong>6-DoF arm teleoperation</strong><br>The robot wrist follows clutch-relative controller movement.</td>
    <td align="center"><strong>Natural head control</strong><br>The robot mirrors headset pan and tilt in real time.</td>
  </tr>
</table>

<p align="center"><sub>Click either preview to open the GIF at full size.</sub></p>

## The project

Vive Teleop turns a consumer VR system into an immersive robot-control station.
The operator sees the TIAGo camera feed inside the headset and controls the head,
arm, and gripper through familiar physical gestures—without a separate control
panel interrupting the task.

| Operator input | Robot response |
| --- | --- |
| Turn or tilt the headset | Pan or tilt the robot head |
| Move the right controller while holding the deadman | Move and rotate the robot wrist, with controller top mapped to tool top |
| Swipe the controller trackpad or joystick without clicking | Open or close the gripper |
| Hold the trackpad/joystick click and push in any direction | Drive and steer the differential base continuously |
| Release the deadman | Stop arm pursuit and clear the active target |

## How it works

```mermaid
flowchart LR
    O["Operator<br/>Vive headset + controller"]
    U["Unity VR client"]
    W["WebRTC gateway"]
    T["C++ ROS 2 teleop<br/>100 Hz Cartesian control"]
    S["MoveIt Servo"]
    R["TIAGo<br/>head · arm · gripper · base · camera"]

    O -->|head and hand motion| U
    U -->|pose + deadman + gripper + base<br/>WebRTC data channel| W
    W -->|typed /vive/* topics| T
    T -->|velocity-limited Cartesian twist| S
    T -->|head + gripper trajectories<br/>guarded base velocity| R
    S -->|7-joint arm trajectory| R
    R -->|camera + live robot state| W
    R -->|joint state + TF| T
    W -->|live video| U

    classDef human fill:#6C63FF,color:#fff,stroke:#4B44C4
    classDef client fill:#00A8A8,color:#fff,stroke:#007878
    classDef robot fill:#F28C28,color:#fff,stroke:#B85E00
    class O human
    class U,W,T,S client
    class R robot
```

The control path separates transport from robot motion. WebRTC carries video and
controller data across the network; the gateway converts input into typed ROS 2
messages; the C++ controller closes the wrist pose loop from live TF at 100 Hz;
MoveIt Servo resolves that Cartesian motion through all seven arm joints.

### Arm command lifecycle

```mermaid
stateDiagram-v2
    [*] --> Idle
    Idle --> Anchored: deadman pressed
    Anchored --> Tracking: wrist TF and controller pose are available
    Tracking --> Tracking: newest 6-DoF target replaces the last
    Tracking --> Halted: deadman released or input times out
    Halted --> Idle: target cleared and zero-twist commands sent
```

Each press anchors the controller's configured top frame to the robot's current
tool frame, so engaging control does not cause a jump. Local controller
orientation changes are applied in the tool's local frame while translation
remains robot-base aligned. Tracking uses the headset frame captured at that
moment, allowing the operator to keep looking around without steering the arm.

## Engineering highlights

- Bidirectional WebRTC: live robot video in one direction and VR input in
  the other, with a TURN relay for the field-network setup.
- Clutch-relative 6-DoF control: controller translation and rotation are mapped
  from a fresh robot pose rather than an old absolute target, with explicit
  controller-top alignment and tracked-origin offset calibration.
- Real-time-oriented C++ arm loop: 100 Hz latest-value pose feedback,
  target-velocity feed-forward, low-pass filtering, Cartesian speed/acceleration
  limits, stale-input/TF watchdogs, memory locking, and `SCHED_FIFO` scheduling.
- Layered motion handling: deadman release, stale-input timeout, workspace bounds,
  joint-limit scaling, singularity scaling, smoothing, and immediate target clear.
- Independent head, arm, gripper, and base paths: each control can stay responsive
  without coupling unrelated operator movement.
- Containerized deployment: Dockerized ROS 2 services, automated runtime checks,
  timestamped logs, and a browser client for debugging without VR hardware.
- Opt-in rosbag2 datasets: MCAP capture records the head RGB view, robot
  motion, effective robot commands, and a deadman label for every camera frame.
  Other operator inputs are rejected by recorder configuration validation.

### Record a dataset

Dataset recording is off by default. Set `VIVE_TELEOP_RECORD_DATASET=1` before
starting the normal Wi-Fi launcher:

```bash
VIVE_TELEOP_RECORD_DATASET=1 ./scripts/start-vive-teleop.sh
```

The default `deadman_window` mode records bootstrap context, each deadman-active
window, and 0.75 seconds of post-roll. Bags and their `manifest.json` and
`events.jsonl` indexes are written below `recordings/<session-id>/`. Set
`VIVE_TELEOP_RECORDING_MODE=continuous_session` when an entire session is
required. Never replay these bags without isolating or remapping their command
topics from the live robot ROS domain.

## Safety scope

This is a supervised research prototype, not a safety-certified robot-control
product. The current experimental Servo profile disables collision checking and
must be used only with the lab's physical emergency-stop and operating
procedure. Signaling and command ingress also assume a trusted, isolated
network; do not expose them directly to the internet.
The base path enforces a deadman, limits, and a timeout, but it does not add an
obstacle collision monitor; maintain line of sight and clear operating space.

## Explore the implementation

- [Technical guide](docs/technical-guide.md) — setup, operation, configuration,
  networking, API behavior, and troubleshooting.
- [Architecture diagrams](docs/architecture/README.md) — deployment, component,
  communication, and class-level views with PlantUML sources.
- [Project documentation](docs/README.md) — documentation index and future dataset
  recording design.
- [Engineering audit](docs/audit-2026-06-28/README.md) — prioritized safety,
  testing, recording, and portfolio roadmap plus the low-risk fix report.
