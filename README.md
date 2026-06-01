# vive_teleop

`vive_teleop` bridges robot ROS1 topics into ROS2, serves the camera over WebRTC, accepts WebRTC input, turns HMD pose into TIAGo head controller commands, and turns wrist pose targets into TIAGo arm controller commands through MoveIt IK.

The Unity VR client is still the intended headset frontend, but `index.html` can be used as a lightweight browser debug client without launching Unity or SteamVR.

## Architecture

The current system has five main pieces:

- `ros1_bridge`: ROS1 Noetic to ROS2 Foxy dynamic bridge. It connects to the robot ROS master and exposes ROS1 topics into ROS2.
- `ros2_app`: ROS2 Humble application. It waits for `/xtion/rgb/image_raw`, runs the WebRTC HTTP signaling server, serves camera video on `/offer`, and accepts data-channel input on `/input_offer`.
- `moveit_server`: ROS2 MoveIt teleoperation node. It consumes typed WebRTC pose topics, converts raw HMD orientation into head joint trajectories, and uses seeded MoveIt IK for small joystick-style wrist updates.
- `coturn`: TURN relay used by WebRTC peers in the current network setup.
- `index.html` / `unity-vr-headset`: WebRTC clients. The browser page is for debugging; Unity is the VR client.

The WebRTC server code is separated from ROS subscriber/publisher logic:

- `image_listener/webrtc_server.py`: aiohttp signaling, peer lifecycle, ICE config, media relay, and data-channel routing.
- `image_listener/image_subscriber.py`: ROS2 image subscriber for `/xtion/rgb/image_raw`.
- `image_listener/video_track.py`: aiortc video track backed by the latest ROS image frame.
- `image_listener/input_publisher.py`: ROS2 publisher for typed WebRTC input messages on `/vive/head_pose` and `/vive/hand_target_pose`.
- `image_listener/teleop_webrtc.py`: composition entry point used through `image_subscriber`.

See [architecture.puml](architecture.puml) for the PlantUML source. Regenerate [architecture.png](architecture.png) from it when a rendered diagram is needed.

## Network Layout

Runtime containers use the `field_net` ipvlan network:

- Robot / ROS master: `10.68.0.1`
- `ros1_bridge`: `10.68.0.131`
- `ros2_app`: `10.68.0.132`
- `coturn`: `10.68.0.133`
- `moveit_server`: `10.68.0.134`

`ros2_app` publishes `8088:8088`, but direct host access can still depend on the ipvlan host-interface setup. If the browser cannot reach `http://localhost:8088`, create the host ipvlan interface shown at the bottom of `docker-compose.yml` and try direct container access at `http://10.68.0.132:8088`.

Both Docker builds use `network: host` so package installs do not depend on Docker's default build bridge network.

## WebRTC API

`ros2_app` listens on `0.0.0.0:8088`.

### `POST /offer`

Accepts a browser/client WebRTC offer and returns an answer. The peer should receive video only.

Client expectation:

- Create `RTCPeerConnection`
- Add a `video` transceiver with `recvonly`
- POST local SDP to `/offer`
- Set the returned answer as the remote description

### `POST /input_offer`

Accepts a WebRTC offer for an input data channel and returns an answer.

String payloads are parsed as JSON. Binary payloads are decoded as UTF-8 before parsing.

The browser debug client snapshots the displayed input values when the input data channel opens, then streams a `unity_teleop_pose` payload at 10 Hz. Use the number-input arrows to make small changes while the stream continues.

When a payload includes pose fields, `ros2_app` publishes standard ROS2 messages:

- `/vive/head_pose`: `geometry_msgs/PoseStamped` copied from the HMD pose.
- `/vive/hand_target_pose`: `geometry_msgs/PoseStamped` using the joystick wrist position and calibrated `robotWristR*` orientation.

## MoveIt server

The separate `moveit_server` container joins the same CycloneDDS graph as `ros2_app`, so it receives the WebRTC topics directly on the ROS2 side of the bridge. It is implemented in Python. By default the container starts Humble's `tiago_moveit_config` `move_group.launch.py`, starts `robot_state_publisher`, and then starts the teleop node.

Default behavior:

- Subscribes to `/vive/head_pose`, converts raw Unity HMD orientation into TIAGo head pan/tilt joints, and publishes `trajectory_msgs/JointTrajectory` commands to `/head_controller/command`.
- Publishes head commands at a fixed 20 Hz by default, with `time_from_start` set to `0.06` seconds so the TIAGo `joint_trajectory_controller` can interpolate smoothly.
- Applies a `0.01` rad head deadband and clamps pan/tilt to 90% of configured joint limits before publishing, so small HMD jitter and startup extremes do not continuously drive the motors.
- Subscribes to `/vive/hand_target_pose` for 6-DoF joystick/controller wrist targets.
- Uses `execution_mode: ik_topic`, which calls MoveIt's `/compute_ik` service instead of running a full OMPL plan for every 10 Hz input update.
- Uses MoveIt group `arm` by default so `torso_lift_joint` is not used.
- Seeds IK from live `/joint_states`, limited to the active MoveIt group joints so non-MoveIt joints from the robot do not crash MoveIt.
- Publishes short `trajectory_msgs/JointTrajectory` commands to `/arm_controller/command`, which the ROS1 bridge can forward to the robot controller.
- Overlays TIAGo's `kinematics.yaml` with `moveit_server/tiago_pick_ik_kinematics.yaml`, using `pick_ik` in local, one-attempt mode for small repeated joystick moves.
- Ramps IK output after startup or a pause with `ik_warmup_sec`, `ik_warmup_min_scale`, and `ik_warmup_reset_after_sec` so the first stationary target does not jerk at the full joint-delta limit.
- If exact 6-DoF IK returns `NO_IK_SOLUTION` (`-31`) and a previous reachable wrist orientation exists, it retries the same position with that last reachable orientation.

Parameters live in `moveit_server/src/vive_moveit_server/config/tiago_single_params.yaml`. For a real robot, check at least:

- `arm_group`: currently `arm` to force no torso. `arm_torso` allows torso motion if you deliberately want it.
- `end_effector_link`: the TIAGo wrist/tool link used for IK. The default is `arm_tool_link`.
- `head_command_topic`: trajectory topic bridged to the ROS1 head controller, default `/head_controller/command`.
- `head_joint_names`: must match the robot's head joints, typically `head_1_joint` and `head_2_joint` for this TIAGo.
- `head_publish_rate_hz` and `head_command_duration_sec`: head command rate and matching trajectory point duration. Defaults are `20.0` Hz and `0.06` seconds.
- `head_deadband_rad`: suppresses tiny pan/tilt updates; default `0.01` rad.
- `head_pan_limits_rad`, `head_tilt_limits_rad`, and `head_limit_scale`: clamp output to a safe fraction of the real controller limits; default scale is `0.9`.
- `head_pan_sign` and `head_tilt_sign`: sign calibration knobs if runtime testing shows Unity yaw or pitch inverted.
- `pose_reference_frame`: defaults to `base_footprint`; adjust if your controller calibration publishes another robot frame.
- `ik_service_name`: defaults to `/compute_ik`.
- `joint_state_topic`: defaults to `/joint_states`.
- `arm_command_topic`: trajectory topic bridged to the ROS1 arm controller.
- `max_hand_target_distance_m`, `min_hand_target_z_m`, `max_hand_target_z_m`: safety bounds for rejecting obviously uncalibrated wrist targets before IK.
- `hand_position_scale` and `hand_position_offset`: calibration from Unity/controller coordinates into the robot frame.
- `max_joint_delta_rad`, `joint_smoothing_alpha`, and `command_duration_sec`: smoothness/responsiveness tuning for the direct controller trajectory output.
- `ik_warmup_sec`, `ik_warmup_min_scale`, and `ik_warmup_reset_after_sec`: startup/resume ramp tuning.

`NO_IK_SOLUTION` (`-31`) does not necessarily mean the `xyz` point is visually impossible. In `ik_topic` mode MoveIt is solving the full `end_effector_link` pose for the arm-only group, including orientation, joint limits, current seed state, and the fact that torso is intentionally locked out.

To disable the bundled TIAGo MoveIt launch and wait for an external MoveGroup server instead:

```bash
MOVEIT_SERVER_LAUNCH_ARGS="moveit_launch_enabled:=false" \
  sudo docker compose up --build moveit_server
```

To pass robot variant arguments through to TIAGo MoveIt, set `moveit_arm`, `moveit_arm_type`, `moveit_base_type`, `moveit_end_effector`, and/or `moveit_ft_sensor` in `MOVEIT_SERVER_LAUNCH_ARGS`. `moveit_allow_trajectory_execution` defaults to `False`; the teleop node commands the real ROS1 robot by publishing short IK-generated trajectories to the bridged controller topics instead.

## Running

Start the containers:

```bash
sudo docker compose up --build
```

This starts `ros1_bridge`, `ros2_app`, `moveit_server`, and `coturn`. The MoveIt container needs a TIAGo MoveIt/robot description configuration available on the ROS2 graph before arm planning can succeed.

In another terminal, serve the debug client:

```bash
python3 -m http.server 8000
```

Open:

```text
http://localhost:8000
```

### Wi-Fi WebRTC access

The robot-facing Docker network can stay on the static Ethernet `10.68.0.0/24` layout while WebRTC signaling and TURN are exposed through the Docker host's current Wi-Fi/default-route IP.

Start the containers with the Wi-Fi overlay:

```bash
./scripts/up-wifi-webrtc.sh
```

If the default route is not the Wi-Fi interface, specify the interface instead of hard-coding the address:

```bash
WEBRTC_NIC=wlan0 ./scripts/up-wifi-webrtc.sh
```

The script detects the current Wi-Fi host IP, exports `WEBRTC_HOST_IP`, detects the field-network host IP as `ROS_FIELD_HOST_IP`, generates a matching CycloneDDS config, and starts host-network Wi-Fi variants:

- `ros1_bridge_wifi` on the host network, using the static robot Ethernet interface for ROS1 robot access.
- `ros2_app_wifi` on the host network, so WebRTC signaling and media are reachable through the host Wi-Fi IP.
- `moveit_server_wifi` on the host network, using the same ROS2 DDS interface as the bridge/app.
- `coturn_wifi` on the host network, listening on both the Wi-Fi IP and the field-network host IP.
- local ROS2 bridge/app discovery over loopback by default with `ROS2_DDS_INTERFACE=lo`.
- client-side ICE with `WEBRTC_PUBLIC_TURN_URLS=turn:<wifi-host-ip>:3478?...`.
- server-side ICE with `WEBRTC_TURN_URLS=turn:127.0.0.1:3478?...` inside the host-network ROS2 app.

This avoids passing WebRTC media through Docker port publishing; Unity and browser clients talk directly to the host Wi-Fi IP.

If another ROS2 node outside this host must discover the Wi-Fi bridge/app, override the DDS interface with the host's field-network IP:

```bash
ROS2_DDS_INTERFACE=10.68.0.130 ./scripts/up-wifi-webrtc.sh
```

Serve the debug client on Wi-Fi:

```bash
./scripts/serve-debug-client.sh
```

Open the printed `http://<host-ip>:8000` URL from a device on the same Wi-Fi. The debug page derives `Server` as `http://<host-ip>:8088` and UDP/TCP `TURN` URLs from the same host.

External clients such as Unity should use the WebRTC server config endpoint:

```text
http://<host-ip>:8088/config
```

It returns the signaling URLs and client-facing ICE server list.

For browser debugging:

1. Wait for `ros2_app_wifi` logs to show `======== Running on http://0.0.0.0:8088 ========`.
2. Check that `ros1_bridge_wifi` logs do not show ROS master connection errors.
3. If the debug page is served by this host, try `Host :8088` first.
4. If the debug page is served from another PC, set `Server` to `http://<host-ip>:8088` manually.
5. Click `Start Video` to connect to `/offer`.
6. Click `Input` to connect to `/input_offer`; the page streams the current input state at 10 Hz. Use the number-input arrows to adjust pose values.

For Unity:

1. Start the containers.
2. Launch Unity and open `unity-vr-headset`.
3. Open `Assets/Scenes/SampleScene.unity`.
4. Select `Quad` and set `Vive Teleop Web RTC Client > Config Url` to `http://<host-ip>:8088/config` if Unity is not running on the Docker host.
5. Launch SteamVR.
6. Press Play.

The Unity scene includes `ViveTeleopWebRtcClient` on the video quad. The quad is parented to the XR camera and kept centered in front of the headset, so it stays visible in a Vive Pro / Vive Pro 2 view. The component fetches `/config`, connects video through `/offer`, opens an input data channel through `/input_offer`, renders the received video onto the quad material, and sends teleop JSON over the data channel.

By default the input payload includes:

- HMD pose from the main camera.
- Right-hand XR controller / 6-DoF joystick wrist pose from `XRNode.RightHand`.
- A calibrated relative `robotWristR*` quaternion, used in `/vive/hand_target_pose`.
- Joystick axis, trigger, grip, and primary button values when the device exposes them through Unity XR.

Press `R` in the Unity player to recalibrate the current wrist orientation as neutral. If the joystick is represented by a custom tracked GameObject, assign it to `Vive Teleop Web RTC Client > Wrist Pose Source`; otherwise the right-hand XR node is used.

Builds can override the scene URL with either `VIVE_TELEOP_WEBRTC_CONFIG_URL` or a command-line argument:

```bash
--webrtc-config-url=http://<host-ip>:8088/config
```

## Troubleshooting

If the browser shows a fetch/network error for `/offer` or `/input_offer`, WebRTC negotiation has not started yet. The HTTP signaling endpoint is unreachable from the browser.

Check:

```bash
sudo docker compose logs ros2_app
curl -i http://localhost:8088/offer
curl -i http://10.68.0.132:8088/offer
```

`GET /offer` is not a valid signaling request, but a response from the server still proves TCP reachability. A connection failure means this is a host/container networking issue, not an SDP or ICE issue.

If direct container access is needed from the Docker host:

```bash
sudo ip link add host-ipvlan link enp2s0f0 type ipvlan mode l2
sudo ip addr add 10.68.0.200/24 dev host-ipvlan
sudo ip link set host-ipvlan up
```

Use the correct NIC if it is not `enp2s0f0`, or set `NIC=...` in `.env`.
