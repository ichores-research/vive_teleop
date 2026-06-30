# Unity VR Client Context

## Responsibility

`unity-vr-headset` is the intended headset frontend. It connects to the WebRTC server, renders camera video, samples HMD/controller state, computes robot wrist targets, controls the gripper from controller input, and sends `unity_teleop_pose` JSON over a WebRTC data channel.

## Key Files

- `unity-vr-headset/Assets/ViveTeleopWebRtcClient.cs`: first-party WebRTC, input, calibration, and recording logic.
- `unity-vr-headset/Assets/StreamingAssets/SteamVR/*.json`: SteamVR action bindings.
- `unity-vr-headset/Packages/manifest.json`: Unity package dependencies.
- `scripts/build-unity-vr-linux.sh`: Linux player build.
- `scripts/run-unity-vr-linux.sh`: player launch wrapper.

## Client Startup Flow

1. Resolve `/config` from `VIVE_TELEOP_WEBRTC_CONFIG_URL`, command-line arguments, or inspector defaults.
2. Connect video through `/offer` if enabled.
3. Fetch `/robot_state` before input.
4. Adopt the current robot wrist pose as the wrist target anchor.
5. Open `/input_offer` and start sending pose payloads at `poseSendRateHz`.

## Payload Model

Unity sends a `PosePayload` with:

- `type = "unity_teleop_pose"`
- HMD availability and quaternion fields.
- wrist availability, deadman state, raw wrist pose, robot wrist target pose, workspace metadata.
- joystick trigger/grip/axis state.
- optional normalized gripper opening.

## Data Stability Notes

- Unity computes robot wrist targets client-side from calibrated controller deltas.
- `wristCommandEnabled` becomes true only when robot wrist state, wrist calibration, and deadman are all active.
- A resync releases wrist control, reloads `/robot_state`, and recalibrates from the current headset/controller pose.
- The client accepts a partial `/robot_state` if wrist pose is valid, even when `ready` is false. Keep this intentional behavior documented or make it stricter if gripper/head readiness should also gate input.
- Unity timestamps are not currently used by the Python server. If ordering is added, update both sides.

## Current and Future Recording

Unity can currently write local JSONL containing the complete `PosePayload`,
but that file is not synchronized with robot camera/state rosbag data. The
future dataset design keeps raw operator telemetry as optional provenance and
uses robot-space command topics as primary ML action candidates.

Future payload work proposes adding `schemaVersion`, `sessionId`, `sourceId`,
and monotonic `sequence` while preserving the existing
`Time.realtimeSinceStartup` timestamp. The same identity must be generated once
by the startup script and shared with Unity, `webrtc_server`, and the recorder.

See `docs/data-recording/dataset-contract.md` before changing recording fields.

## Update Checklist

When changing payload shape:

- Update `PosePayload` in `ViveTeleopWebRtcClient.cs`.
- Update `input_publisher.py`.
- Update browser debug payload builders.
- Update `docs/architecture/component/overview.puml`, `docs/architecture/communication/data-flow.puml`, and `docs/architecture/class/unity/unity-client.puml`.
- Consider adding a sample JSON payload to `docs/architecture/README.md` if the schema grows.
