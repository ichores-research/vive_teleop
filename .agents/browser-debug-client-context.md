# Browser Debug Client Context

## Responsibility

`index.html` is a lightweight WebRTC debug client. It receives camera video, reads live robot state, exposes manual controls, and sends a 20 Hz interpolated input stream over the same `/input_offer` data-channel path used by Unity.

## Key File

- `index.html`

## Runtime Flow

1. Load `/config`.
2. Create a video peer and POST `/offer`.
3. Before input, poll `/robot_state` until head, wrist, and gripper are all ready.
4. Render the robot state into form controls.
5. Open `/input_offer`.
6. Start a 20 Hz stream using interpolated target state.

## Data Produced

The browser can send:

- Combined `unity_teleop_pose` payloads for head, wrist, and gripper.
- Head-only debug payloads.
- Wrist-only debug payloads.
- Gripper-only debug payloads.

The Python side currently ignores the `type` value and routes by availability flags.

## Data Stability Notes

- The browser is stricter than Unity about `/robot_state`; it waits for full readiness before opening input.
- Number fields are normalized with `Number(...)` and non-finite values fall back to `0`.
- Quaternion interpolation normalizes values before sending.
- The debug client should remain conservative. Avoid adding shortcuts that bypass `/robot_state` readiness unless they are clearly marked as unsafe test tools.

## Update Checklist

When changing controls or payload fields:

- Update the control capture/render path.
- Update `buildCombinedPayload`, `headFields`, `wristFields`, and `gripperFields`.
- Update `input_publisher.py` and Unity payload shape if the field is part of the shared contract.
- Update `docs/architecture/component/overview.puml` and `docs/architecture/communication/data-flow.puml`.
