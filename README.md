# vive_teleop

`vive_teleop` connects directly to the robot's ROS2 graph, serves the camera over WebRTC, accepts WebRTC input, turns HMD pose into TIAGo head controller commands, turns wrist pose targets into TIAGo arm controller commands through MoveIt IK, and controls the two-finger gripper.

The Unity VR client is still the intended headset frontend, but `index.html` can be used as a lightweight browser debug client without launching Unity or SteamVR.

## Quick start on Linux

From the repository root, start Docker, SteamVR, and the built Unity VR client:

```bash
cd /home/mateusz/vive_teleop
./scripts/start-vive-teleop.sh
```

The script:

1. Detects the Wi-Fi and robot-facing Ethernet addresses.
2. Builds and starts `ros2_app_wifi`, `moveit_server_wifi`, and `coturn_wifi`.
3. Waits for `http://<wifi-ip>:8088/config`.
4. Starts SteamVR through Steam if it is not already running.
5. Warns if the robot at `10.68.0.1` is unreachable.
6. Runs the Unity player in the foreground with controller recording enabled.

Use `Ctrl+C` in that terminal to stop the Unity player. Stop the containers with:

```bash
docker compose -f docker-compose.yml -f docker-compose.wifi.yml stop \
  ros2_app_wifi moveit_server_wifi coturn_wifi
```

SteamVR is managed separately by Steam and can be closed from the SteamVR window.

If the Unity player has not been built yet:

```bash
./scripts/check-unity-vr-linux.sh
./scripts/build-unity-vr-linux.sh
./scripts/start-vive-teleop.sh
```

The Unity build is expected at:

```text
unity-vr-headset/Builds/Linux/vive-teleop.x86_64
```

### Start each part manually

Terminal 1, start the services in detached mode:

```bash
cd /home/mateusz/vive_teleop
./scripts/up-wifi-webrtc.sh -d
```

Terminal 2, start SteamVR:

```bash
steam -applaunch 250820
```

Terminal 3, start the built Unity client:

```bash
cd /home/mateusz/vive_teleop
./scripts/run-unity-vr-linux.sh
```

Useful health checks:

```bash
docker ps --format '{{.Names}}\t{{.Status}}' |
  grep -E 'ros2_app_wifi|moveit_server_wifi|coturn_wifi'

WEBRTC_HOST_IP="$(./scripts/detect-webrtc-host-ip.sh)"
curl -s "http://${WEBRTC_HOST_IP}:8088/config" | python3 -m json.tool

docker exec moveit_server_wifi bash -lc \
  'source /opt/ros/humble/setup.bash && ros2 topic info /joint_states -v'
```

Arm control remains disabled until `/joint_states` has a robot publisher.

## Architecture

The current system has four main pieces:

- `ros2_app`: ROS2 Humble application. It subscribes directly to the robot's `/head_front_camera/rgb/image_raw` topic, runs the WebRTC HTTP signaling server, serves camera video on `/offer`, and accepts data-channel input on `/input_offer`.
- `moveit_server`: ROS2 teleoperation node. It converts raw HMD orientation into head trajectories, uses seeded MoveIt IK for wrist updates, and publishes direct two-finger gripper trajectories.
- `coturn`: TURN relay used by WebRTC peers in the current network setup.
- `index.html` / `unity-vr-headset`: WebRTC clients. The browser page is for debugging; Unity is the VR client.

The WebRTC server code is separated from ROS subscriber/publisher logic:

- `image_listener/webrtc_server.py`: aiohttp signaling, peer lifecycle, ICE config, media relay, and data-channel routing.
- `image_listener/image_subscriber.py`: ROS2 image subscriber for `/head_front_camera/rgb/image_raw`.
- `image_listener/video_track.py`: aiortc video track backed by the latest ROS image frame.
- `image_listener/input_publisher.py`: ROS2 publisher for typed WebRTC input messages on `/vive/head_pose`, `/vive/hand_target_pose`, and `/vive/gripper_opening`.
- `image_listener/robot_state.py`: live robot head, wrist, and gripper snapshot provider used to initialize debug input safely.
- `image_listener/teleop_webrtc.py`: composition entry point used through `image_subscriber`.

See [architecture.puml](architecture.puml) for the PlantUML source. Regenerate [architecture.png](architecture.png) from it when a rendered diagram is needed.

## Network Layout

Runtime containers use the `field_net` ipvlan network:

- Robot / ROS2 graph: `10.68.0.1`
- `ros2_app`: `10.68.0.132`
- `coturn`: `10.68.0.133`
- `moveit_server`: `10.68.0.134`

The ROS2 containers use CycloneDDS on domain `67` by default, matching the robot, with `10.68.0.1` configured as a discovery peer. Override `ROS_DOMAIN_ID` if the robot configuration changes. The existing container and robot addresses are unchanged; `10.68.0.131` is no longer used.

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

Before opening the input data channel, the browser debug client reads `GET /robot_state`. The server takes the wrist pose from TF (`base_footprint` to `arm_tool_link`) and the head and gripper state from `/joint_states`, verifies that they are fresh, then fills the displayed controls. The input stream starts at those live values, interpolates toward edited targets, and sends a `unity_teleop_pose` payload at 20 Hz. Use the wrist XYZ, head Pan/Tilt, and normalized gripper opening controls to make small changes while the stream continues.

Gripper fields in the JSON payload are:

```json
{
  "gripperAvailable": true,
  "gripperOpening": 0.5
}
```

`gripperOpening` is normalized and clamped by the ROS bridge: `0.0` is closed and `1.0` is fully open. Clients that do not intend to command the gripper must omit these fields or set `gripperAvailable` to `false`.

### `GET /robot_state`

Returns the live debug-input starting state. The response is marked `ready: false` when head or gripper joint states, or either required TF transform, are missing or stale; the browser will not open the input channel in that case.

The gripper portion of a ready response has this form:

```json
{
  "gripper": {
    "opening": 0.42,
    "leftPosition": 0.019,
    "rightPosition": 0.019,
    "minPosition": 0.0,
    "maxPosition": 0.045
  }
}
```

`opening` is calculated from the average measured finger position. The raw left and right positions are included for diagnosis.

When a payload includes pose fields, `ros2_app` publishes standard ROS2 messages:

- `/vive/head_pose`: `geometry_msgs/PoseStamped` copied from the HMD pose.
- `/vive/hand_target_pose`: `geometry_msgs/PoseStamped` using the joystick wrist position and calibrated `robotWristR*` orientation.
- `/vive/gripper_opening`: `std_msgs/Float64` normalized opening, where `0` is closed and `1` is fully open.

## MoveIt server

The separate `moveit_server` container joins the same CycloneDDS graph as `ros2_app` and the robot. It is implemented in Python. By default the container starts Humble's `tiago_moveit_config` `move_group.launch.py`, starts `robot_state_publisher`, and then starts the teleop node.

Default behavior:

- Subscribes to `/vive/head_pose`, converts raw Unity HMD orientation into TIAGo head pan/tilt joints, and publishes `trajectory_msgs/JointTrajectory` commands to `/head_controller/joint_trajectory`.
- Publishes head commands at a fixed 20 Hz, with overlapping `0.1` second trajectory points so the TIAGo `joint_trajectory_controller` can interpolate smoothly.
- Applies a `0.002` rad head deadband and clamps pan/tilt to 90% of configured joint limits before publishing, so small HMD jitter and startup extremes do not continuously drive the motors.
- Subscribes to `/vive/hand_target_pose` for 6-DoF joystick/controller wrist targets.
- Uses `execution_mode: ik_topic`, which calls MoveIt's `/compute_ik` service instead of running a full OMPL plan for each interpolated input update.
- Uses MoveIt group `arm` by default so `torso_lift_joint` is not used.
- Seeds IK from live `/joint_states`, limited to the active MoveIt group joints so non-MoveIt joints from the robot do not crash MoveIt.
- Publishes short `trajectory_msgs/JointTrajectory` commands directly to the robot's `/arm_controller/joint_trajectory` topic.
- Treats the trigger/side-grip input as a deadman clutch. Each press anchors the current controller pose to the current robot wrist pose from MoveIt FK, so pressing the button while the controller is elsewhere does not move the arm.
- Applies only controller translation and rotation accumulated after the current deadman press. Releasing the button clears the target, clutch anchors, IK pursuit state, and queued messages.
- Keeps only the newest hand target. New samples replace the previous target instead of being replayed as a movement queue.
- Evaluates the newest target at a nominal `0.04` second cadence and pursues it through bounded Cartesian IK increments.
- Seeds repeated IK from the previous command endpoint rather than delayed measured joints, with a measured-state resynchronization guard if the physical arm falls too far behind.
- Rejects IK solutions that jump to a distant joint-space branch instead of slowly chasing the discontinuity through the joint-delta limiter.
- Publishes overlapping `0.08` second arm trajectory points at the nominal 25 Hz update rate so timing jitter does not leave command gaps.
- Starts direct arm trajectories immediately on receipt instead of timestamping them on the host, avoiding clock-skew and transport-delay loss on the robot controller.
- Subscribes to normalized commands on `/vive/gripper_opening` and publishes synchronized, velocity-aware finger trajectories to `/gripper_controller/joint_trajectory`.
- Suppresses the initial gripper command when the requested opening matches the measured robot state, then suppresses duplicate targets within the configured deadband.
- Overlays TIAGo's `kinematics.yaml` with `moveit_server/tiago_pick_ik_kinematics.yaml`, using `pick_ik` in local, one-attempt mode for small repeated joystick moves.
- Ramps IK output after startup or a pause with `ik_warmup_sec`, `ik_warmup_min_scale`, and `ik_warmup_reset_after_sec` so the first stationary target does not jerk at the full joint-delta limit.
- If exact 6-DoF IK returns `NO_IK_SOLUTION` (`-31`) and a previous reachable wrist orientation exists, it retries the same position with that last reachable orientation.

Parameters live in `moveit_server/src/vive_moveit_server/config/tiago_single_params.yaml`. For a real robot, check at least:

- `arm_group`: currently `arm` to force no torso. `arm_torso` allows torso motion if you deliberately want it.
- `end_effector_link`: the TIAGo wrist/tool link used for IK. The default is `arm_tool_link`.
- `head_command_topic`: direct ROS2 trajectory topic for the head controller, default `/head_controller/joint_trajectory`.
- `head_joint_names`: must match the robot's head joints, typically `head_1_joint` and `head_2_joint` for this TIAGo.
- `head_publish_rate_hz` and `head_command_duration_sec`: head command rate and matching trajectory point duration. Defaults are `20.0` Hz and `0.1` seconds.
- `head_deadband_rad`: suppresses tiny pan/tilt updates; configured as `0.002` rad.
- `head_pan_limits_rad`, `head_tilt_limits_rad`, and `head_limit_scale`: clamp output to a safe fraction of the real controller limits; default scale is `0.9`.
- `head_pan_sign` and `head_tilt_sign`: sign calibration knobs if runtime testing shows Unity yaw or pitch inverted.
- `pose_reference_frame`: defaults to `base_footprint`; adjust if your controller calibration publishes another robot frame.
- `ik_service_name`: defaults to `/compute_ik`.
- `fk_service_name`: defaults to `/compute_fk` and is used to capture the current robot wrist pose when the deadman is pressed.
- `joint_state_topic`: defaults to `/joint_states`.
- `arm_command_topic`: direct ROS2 trajectory topic for the arm controller, default `/arm_controller/joint_trajectory`.
- `gripper_input_topic`: normalized `std_msgs/Float64` command topic, configured as `/vive/gripper_opening`.
- `gripper_command_topic`: direct controller topic, configured as `/gripper_controller/joint_trajectory`.
- `gripper_joint_names`: controller joint order, configured as `gripper_right_finger_joint` followed by `gripper_left_finger_joint`.
- `gripper_min_position_m` and `gripper_max_position_m`: map normalized input onto each finger's physical range, configured as `0.0` to `0.045` m.
- `gripper_deadband_m`: suppresses repeated targets within `0.0005` m.
- `gripper_command_duration_sec`: minimum trajectory duration, configured as `0.15` seconds.
- `gripper_max_velocity_mps`: extends the trajectory duration when required to keep finger motion at or below `0.04` m/s.
- `hand_target_timeout_sec`: time without a gated hand target before the deadman is considered released. It is configured as `0.12` seconds.
- `cartesian_position_step_m` and `cartesian_orientation_step_rad`: maximum Cartesian increments used while pursuing the newest relative target.
- `max_hand_target_distance_m`, `min_hand_target_z_m`, `max_hand_target_z_m`: workspace limits applied before IK.
- `hand_position_scale` and `hand_position_offset`: calibration from Unity/controller coordinates into the robot frame.
- `max_joint_delta_rad`, `joint_smoothing_alpha`, `joint_command_deadband_rad`, and `command_duration_sec`: smoothness/responsiveness tuning for the direct controller trajectory output. The joint deadband stops repeated trajectory refreshes after the arm reaches its requested pose.
- `ik_seed_from_commanded_state`, `ik_command_resync_threshold_rad`, and `ik_solution_jump_threshold_rad`: keep repeated IK on the current joint-space branch while falling back to measured state if command tracking diverges.
- `ik_slow_request_warn_sec`: reports IK service round trips that consume too much of the control period.
- `ik_warmup_sec`, `ik_warmup_min_scale`, and `ik_warmup_reset_after_sec`: startup/resume ramp tuning.

`NO_IK_SOLUTION` (`-31`) does not necessarily mean the `xyz` point is visually impossible. In `ik_topic` mode MoveIt is solving the full `end_effector_link` pose for the arm-only group, including orientation, joint limits, current seed state, and the fact that torso is intentionally locked out.

### Gripper control

The browser debug client is the built-in gripper frontend:

1. Reload the debug page after updating the repository so the browser has the current JavaScript.
2. Click `Input`. The page reads both finger joints from `/robot_state` and displays their measured positions.
3. Change `Opening`, or use `Open` and `Close`. While input is connected, the target is interpolated and streamed automatically at 20 Hz.
4. `Send Gripper` sends the current gripper target immediately. `Send All` sends head, wrist, and gripper fields together.

Connecting input does not intentionally move the gripper. The stream starts from the measured opening, and the server ignores an initial target that is already within `gripper_deadband_m` of the current finger position.

The Unity client currently reports the XR controller's `joystickGrip` value, but that field is not mapped to robot gripper motion. A Unity build must send `gripperAvailable: true` and `gripperOpening` to use this command path.

To disable the bundled TIAGo MoveIt launch and wait for an external MoveGroup server instead:

```bash
MOVEIT_SERVER_LAUNCH_ARGS="moveit_launch_enabled:=false" \
  sudo docker compose up --build moveit_server
```

To pass robot variant arguments through to TIAGo MoveIt, set `moveit_arm`, `moveit_arm_type`, `moveit_base_type`, `moveit_end_effector`, and/or `moveit_ft_sensor` in `MOVEIT_SERVER_LAUNCH_ARGS`. `moveit_allow_trajectory_execution` defaults to `False`; the teleop node commands the real ROS2 robot by publishing short IK-generated trajectories directly to the controller topics instead.

## Running

For the complete VR application, use the Linux quick-start command:

```bash
./scripts/start-vive-teleop.sh
```

For Docker services only:

```bash
./scripts/up-wifi-webrtc.sh -d
```

This starts the host-network `ros2_app_wifi`, `moveit_server_wifi`, and
`coturn_wifi` services. The MoveIt container needs live robot `/joint_states`
and the TIAGo robot description before arm IK can command the physical arm.

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

- `ros2_app_wifi` on the host network, so WebRTC signaling and media are reachable through the host Wi-Fi IP.
- `moveit_server_wifi` on the host network, using the same ROS2 DDS interface as the app.
- `coturn_wifi` on the host network, listening on both the Wi-Fi IP and the field-network host IP.
- direct robot discovery through `ROS_FIELD_HOST_IP` by default, with `10.68.0.1` as the DDS peer.
- client-side ICE with `WEBRTC_PUBLIC_TURN_URLS=turn:<wifi-host-ip>:3478?...`.
- server-side ICE with `WEBRTC_TURN_URLS=turn:127.0.0.1:3478?...` inside the host-network ROS2 app.

This avoids passing WebRTC media through Docker port publishing; Unity and browser clients talk directly to the host Wi-Fi IP.

To override either the DDS interface or robot address:

```bash
ROS2_DDS_INTERFACE=10.68.0.130 ROBOT_IP=10.68.0.1 \
  ./scripts/up-wifi-webrtc.sh
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
2. Check direct robot discovery with `docker exec ros2_app_wifi bash -lc 'source /opt/ros/humble/setup.bash && ros2 topic list'`.
3. If the debug page is served by this host, try `Host :8088` first.
4. If the debug page is served from another PC, set `Server` to `http://<host-ip>:8088` manually.
5. Click `Start Video` to connect to `/offer`.
6. Click `Input`. The page first displays the current robot head, wrist, and gripper values from `/robot_state`, then connects to `/input_offer` and streams smoothly from that safe starting state at 20 Hz.
7. Adjust `Opening` between `0` and `1`, or click `Open` or `Close`. The measured left and right finger positions are read-only values captured when input connects.

For Unity on Linux:

1. Install Unity `6000.3.17f1` for Linux with Linux Build Support (x86-64).
2. Install SteamVR and verify the Vive Pro / Vive Pro 2 headset, base stations, and controllers are visible in SteamVR.
3. Run `./scripts/check-unity-vr-linux.sh`.
4. Start the containers and launch SteamVR.
5. Launch Unity and open `unity-vr-headset`.
6. Open `Assets/Scenes/SampleScene.unity`.
7. Select `Quad` and set `Vive Teleop Web RTC Client > Config Url` to `http://<host-ip>:8088/config` if Unity is not running on the Docker host.
8. Press Play.

The Unity scene includes `ViveTeleopWebRtcClient` on the video quad. The quad is parented to the XR camera and kept centered in front of the headset, so it stays visible in a Vive Pro / Vive Pro 2 view. The component fetches `/config`, connects video through `/offer`, opens an input data channel through `/input_offer`, renders the received video onto the quad material, and sends teleop JSON over the data channel.

By default the input payload includes:

- HMD pose from the main camera.
- Right-hand SteamVR/OpenVR controller 6-DoF pose, with Unity XR as a fallback.
- A robot wrist target anchored to the live `/robot_state` wrist pose.
- Trackpad, trigger, grip, and menu-button values.

`joystickGrip` is telemetry only. The current Unity client does not populate `gripperAvailable` or `gripperOpening`, so it does not actuate the robot gripper.

Hold the Vive trigger or side-grip button to command the robot wrist. On every
press, the MoveIt server anchors that controller pose to the robot's current
wrist pose. Holding the controller still causes no movement; only translation
and rotation after the press are applied. Release the button to clear the
target and clutch state. The next press creates new anchors from the robot's
then-current wrist pose.

Press `R` in the Unity player to re-anchor the current controller pose to the current robot wrist target. If the joystick is represented by a custom tracked GameObject, assign it to `Vive Teleop Web RTC Client > Wrist Pose Source`; otherwise the right-hand XR node is used.

Press `Space` or the Vive controller menu button to start or stop local 6-DoF recording. Each line in the output `.jsonl` file is the same `unity_teleop_pose` JSON object sent over WebRTC, including wrist XYZ, quaternion, calibrated robot quaternion, and controller values. On Linux, recordings default to:

```text
~/.config/unity3d/DefaultCompany/unity-vr-headset/ViveTeleopRecordings/
```

Recording can also be controlled at startup:

```bash
VIVE_TELEOP_RECORD_CONTROLLER=1 \
VIVE_TELEOP_RECORDING_DIR="$PWD/recordings" \
./unity-vr-headset/Builds/Linux/vive-teleop.x86_64
```

The recommended terminal launcher sets these variables and writes a timestamped
player log automatically:

```bash
./scripts/run-unity-vr-linux.sh
```

Builds can override the scene URL with either `VIVE_TELEOP_WEBRTC_CONFIG_URL` or a command-line argument:

```bash
--webrtc-config-url=http://<host-ip>:8088/config
```

Build a Linux player after closing the project in the Unity editor:

```bash
./scripts/build-unity-vr-linux.sh
```

## Troubleshooting

If the robot image is visible but the arm cannot move, first verify that the
robot is online and publishing joint state:

```bash
ping -c 3 10.68.0.1

docker exec moveit_server_wifi bash -lc \
  'source /opt/ros/humble/setup.bash && \
   ros2 topic info /joint_states -v'
```

`Publisher count: 0` means the robot is not currently discoverable. The
deadman clutch intentionally waits instead of commanding the arm without a
current robot pose.

Inspect the current service logs:

```bash
docker logs --tail 100 ros2_app_wifi
docker logs --tail 150 moveit_server_wifi
```

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

### Gripper diagnostics

Check that the live state contains both finger joints:

```bash
curl -s http://localhost:8088/robot_state | python3 -m json.tool
```

Check the normalized command bridge and controller connection:

```bash
docker exec moveit_server_wifi bash -lc \
  'source /opt/ros/humble/setup.bash && \
   ros2 topic info /vive/gripper_opening -v && \
   ros2 topic info /gripper_controller/joint_trajectory -v'
```

The expected graph has `webrtc_input_publisher` publishing `/vive/gripper_opening`, `vive_moveit_server` subscribing to it and publishing `/gripper_controller/joint_trajectory`, and `gripper_controller` subscribing to the trajectory topic.

Confirm the robot controller endpoint:

```bash
docker exec moveit_server_wifi bash -lc \
  'source /opt/ros/humble/setup.bash && \
   ros2 node info /gripper_controller'
```

If `Input` reports a robot-state error, verify that `/joint_states` contains `gripper_left_finger_joint` and `gripper_right_finger_joint`. If the normalized topic is connected but the trajectory topic has no `gripper_controller` subscriber, fix ROS2 discovery or the robot controller before sending commands.
