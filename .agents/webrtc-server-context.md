# webrtc_server Context

## Responsibility

`webrtc_server` is the gateway between WebRTC clients and ROS 2. It owns HTTP signaling, ICE configuration, camera media relay, input JSON parsing, and live robot state snapshots for safe client initialization.

## Key Files

- `webrtc_server/src/image_listener/image_listener/teleop_webrtc.py`: composition entry point.
- `webrtc_server/src/image_listener/image_listener/webrtc_server.py`: aiohttp routes, peer lifecycle, ICE/TURN configuration, data-channel routing.
- `webrtc_server/src/image_listener/image_listener/image_subscriber.py`: camera ROS 2 subscriber.
- `webrtc_server/src/image_listener/image_listener/video_track.py`: latest-frame aiortc video track.
- `webrtc_server/src/image_listener/image_listener/input_publisher.py`: WebRTC payload -> typed ROS 2 topics.
- `webrtc_server/src/image_listener/image_listener/robot_state.py`: `/robot_state` snapshot from joint states and TF.

## Public HTTP API

- `GET /config`: returns server URLs and ICE servers.
- `GET /healthz`: same handler as `/config`; basic signaling availability.
- `GET /robot_state`: returns readiness, head state, wrist pose, and gripper state.
- `POST /offer`: WebRTC video offer/answer.
- `POST /input_offer`: WebRTC data-channel offer/answer.

## ROS 2 Inputs

- `/head_front_camera/rgb/image_raw`: camera stream.
- `/joint_states`: head and gripper state for `/robot_state`.
- TF: `base_footprint -> arm_tool_link` and `base_footprint -> head_front_camera_link`.

## ROS 2 Outputs

- `/vive/head_pose`: `geometry_msgs/PoseStamped`.
- `/vive/hand_target_pose`: `geometry_msgs/PoseStamped`.
- `/vive/hand_target_active`: `std_msgs/Bool`.
- `/vive/gripper_opening`: `std_msgs/Float64`, normalized `0.0..1.0`.

## Data Stability Notes

- `RobotInputState` checks joint state and TF freshness before returning `ready: true`.
- `InputPublisher` currently accepts loose JSON. Any hardening work should start here with a versioned schema, finite-number checks, strict booleans, frame validation, and sequence/timestamp handling.
- The video path intentionally stores only the latest frame; this is correct for low-latency teleoperation.
- Multiple input data channels can currently publish to the same ROS topics. Add source/session ownership before allowing multi-operator scenarios.

## Future Dataset Recording

The proposed recorder should preserve the accepted raw Unity/browser payload in
addition to typed control topics. The pragmatic version 1 design publishes a
versioned `/vive/raw_input_json` `std_msgs/String` containing source/session
identity, sequence, source monotonic time, and gateway receipt time.

This is not implemented. When adding it, publish only validated accepted input,
keep command-topic behavior unchanged, and enforce one active source per
session. See `docs/data-recording/dataset-contract.md`.

## Update Checklist

When changing WebRTC input fields:

- Update `input_publisher.py`.
- Update Unity `PosePayload`.
- Update browser payload builders in `index.html`.
- Update `.agents/unity-client-context.md`, `.agents/browser-debug-client-context.md`, `docs/architecture/component/webrtc-server.puml`, `docs/architecture/component/ros-topic-boundary.puml`, `docs/architecture/communication/data-flow.puml`, and the relevant class diagrams.
- Add tests in `webrtc_server/src/image_listener/test/`.
