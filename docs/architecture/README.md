# Architecture Diagrams

This folder contains the PlantUML architecture views for `vive_teleop`.
The folder is organized by diagram type so rendered images stay readable.

## Deployment

- `deployment/deployment.puml`: where the system runs and the highest-level network/runtime connections.

## Components

- `component/overview.puml`: small top-level component view.
- `component/ros2-app.puml`: gateway internals and published `/vive` topics.
- `component/moveit-server.puml`: teleop, Servo, and controller command internals.
- `component/ros-topic-boundary.puml`: narrow ROS topic contract between subsystems.

## Classes

- `class/ros2_app/image-subscriber-node.puml`: camera subscriber and WebRTC video track internals.
- `class/ros2_app/webrtc-server.puml`: HTTP signaling, peer lifecycle, ICE config, and route registration internals.
- `class/ros2_app/webrtc-input-publisher-node.puml`: data-channel payload parsing and ROS topic publishing.
- `class/ros2_app/robot-input-state-node.puml`: live robot state snapshot internals.
- `class/moveit_server/vive-moveit-server-node.puml`: main teleop control node internals.
- `class/moveit_server/servo-pose-bridge-node.puml`: Servo pose-to-twist bridge internals.
- `class/unity/unity-client.puml`: Unity client payload, calibration, and WebRTC class structure.

## Communication

- `communication/data-flow.puml`: brief data-flow sequence for startup, video, input, and robot command output.

## Rendering

Render all diagrams with:

```bash
find docs/architecture -name '*.puml' -print0 | xargs -0 plantuml
```

Rendered PNGs are generated next to their `.puml` files.
