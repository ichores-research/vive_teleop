# Vive Teleop Technical Guide

`vive_teleop` connects directly to the robot's ROS2 graph, serves the camera over WebRTC, accepts WebRTC input, turns HMD pose into TIAGo head controller commands, closes a 100 Hz C++ Cartesian wrist loop through MoveIt Servo, and controls the two-finger gripper.

The Unity VR client is still the intended headset frontend, but `index.html` can be used as a lightweight browser debug client without launching Unity or SteamVR.

## Quick start on Linux

From the repository root, start Docker, SteamVR, and the built Unity VR client:

```bash
cd /path/to/vive_teleop
./scripts/start-vive-teleop.sh
```

The script:

1. Detects the Wi-Fi and robot-facing Ethernet addresses.
2. Rebuilds the Unity Linux player when its sources or project settings changed.
3. Builds and starts `webrtc_server_wifi`, `moveit_server_wifi`, and `coturn_wifi`.
4. Waits for `http://<wifi-ip>:8088/config`.
5. Waits for the robot camera, wrist TF, joint states, and gripper state.
6. Verifies the C++ arm-loop rate, Cartesian speed/acceleration caps, controller-top calibration, real-time scheduling/memory locking, deadman halt gate, automatic Servo joint-limit scaling, and the complete seven-joint `arm` MoveIt group.
7. Starts SteamVR through Steam if it is not already running.
8. Runs the Unity player in the foreground with controller recording enabled.

Use `Ctrl+C` in that terminal to stop the Unity player. Stop the containers with:

```bash
docker compose -f docker-compose.yml -f docker-compose.wifi.yml stop \
  webrtc_server_wifi moveit_server_wifi coturn_wifi
```

SteamVR is managed separately by Steam and can be closed from the SteamVR window.

If the Unity player has not been built yet:

```bash
./scripts/check-unity-vr-linux.sh
./scripts/build-unity-vr-linux.sh
./scripts/start-vive-teleop.sh
```

Running `start-vive-teleop.sh` is sufficient after source changes; manual Unity
or Docker compilation is not required. It uses Docker Compose `--build` and a
Unity build stamp to rebuild only stale artifacts. Set
`VIVE_TELEOP_FORCE_UNITY_BUILD=1` to force a Unity rebuild or
`VIVE_TELEOP_SKIP_UNITY_BUILD=1` to bypass the automatic Unity build.

The Unity build is expected at:

```text
unity-vr-headset/Builds/Linux/vive-teleop
```

`run-unity-vr-linux.sh` still accepts an older `vive-teleop.x86_64` build as a
fallback. Set `VIVE_TELEOP_PLAYER` to run a player from another path.

### Start each part manually

Terminal 1, start the services in detached mode:

```bash
cd /path/to/vive_teleop
./scripts/up-wifi-webrtc.sh -d
```

Terminal 2, start SteamVR:

```bash
steam -applaunch 250820
```

Terminal 3, start the built Unity client:

```bash
cd /path/to/vive_teleop
./scripts/run-unity-vr-linux.sh
```

Useful health checks:

```bash
docker ps --format '{{.Names}}\t{{.Status}}' |
  grep -E 'webrtc_server_wifi|moveit_server_wifi|coturn_wifi'

WEBRTC_HOST_IP="$(./scripts/detect-webrtc-host-ip.sh)"
curl -s "http://${WEBRTC_HOST_IP}:8088/config" | python3 -m json.tool

./scripts/check-teleop-runtime.sh
```

Arm control remains disabled until the runtime check can read fresh robot state
and validate the seven-joint Servo configuration.

## Architecture

The current system has four main pieces:

- `webrtc_server`: ROS2 Humble application. It subscribes directly to the robot's `/head_front_camera/rgb/image_raw` topic, runs the WebRTC HTTP signaling server, serves camera video on `/offer`, and accepts data-channel input on `/input_offer`.
- `moveit_server`: ROS2 teleoperation node. It converts raw HMD orientation into head trajectories, sends wrist pose targets through MoveIt Servo, and publishes direct two-finger gripper trajectories.
- `coturn`: TURN relay used by WebRTC peers in the current network setup.
- `index.html` / `unity-vr-headset`: WebRTC clients. The browser page is for debugging; Unity is the VR client.

The WebRTC server code is separated from ROS subscriber/publisher logic:

- `image_listener/webrtc_server.py`: aiohttp signaling, peer lifecycle, ICE config, media relay, and data-channel routing.
- `image_listener/image_subscriber.py`: ROS2 image subscriber for `/head_front_camera/rgb/image_raw`.
- `image_listener/video_track.py`: aiortc video track backed by the latest ROS image frame.
- `image_listener/input_publisher.py`: ROS2 publisher for typed WebRTC input messages on `/vive/head_pose`, `/vive/hand_target_pose`, `/vive/hand_target_active`, and `/vive/gripper_opening`.
- `image_listener/robot_state.py`: live robot head, wrist, and gripper snapshot provider used to initialize debug input safely.
- `image_listener/teleop_webrtc.py`: composition entry point used through `image_subscriber`.

The architecture diagram set lives in [architecture](architecture):

- `deployment/deployment.puml`: where the system runs and the highest-level connections.
- `component/overview.puml`: small top-level component view.
- `component/*.puml`: focused component views for gateway, MoveIt, and ROS topic boundaries.
- `communication/data-flow.puml`: brief startup, video, input, and command data flow.
- `class/**/*.puml`: internals for the runtime nodes and Unity client class.

Regenerate the PNGs in that folder from their `.puml` files when rendered
diagrams are needed.

## Runtime logs

`scripts/start-vive-teleop.sh` writes each run to `logs/<timestamp>/` by
default. Override the parent directory with `VIVE_TELEOP_LOG_ROOT` or set an
exact run directory with `VIVE_TELEOP_RUN_LOG_DIR`.

Each run captures separate files for startup, Docker Compose, `webrtc_server_wifi`,
`moveit_server_wifi`, `coturn_wifi`, runtime validation, SteamVR launch, Unity
wrapper output, and the Unity player log.

## Network Layout

Runtime containers use the `field_net` ipvlan network:

- Robot / ROS2 graph: `10.68.0.1`
- `webrtc_server`: `10.68.0.132`
- `coturn`: `10.68.0.133`
- `moveit_server`: `10.68.0.134`

The ROS2 containers use CycloneDDS on domain `67` by default, matching the robot, with `10.68.0.1` configured as a discovery peer. Override `ROS_DOMAIN_ID` if the robot configuration changes. The existing container and robot addresses are unchanged; `10.68.0.131` is no longer used.

`webrtc_server` publishes `8088:8088`, but direct host access can still depend on the ipvlan host-interface setup. If the browser cannot reach `http://localhost:8088`, create the host ipvlan interface shown at the bottom of `docker-compose.yml` and try direct container access at `http://10.68.0.132:8088`.

Both Docker builds use `network: host` so package installs do not depend on Docker's default build bridge network.

## WebRTC API

`webrtc_server` listens on `0.0.0.0:8088`.

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

Before opening the input data channel, the browser debug client reads `GET /robot_state`. The server takes the wrist pose from TF (`base_footprint` to `arm_tool_link`) and the head and gripper state from `/joint_states`, verifies that they are fresh, then fills the displayed controls. The input stream starts at those live values, interpolates toward edited targets, and sends a `unity_teleop_pose` payload at 20 Hz. Use the wrist XYZ, head Pan/Tilt, and normalized gripper opening controls to make small changes while the stream continues. Wrist commands remain inactive unless the operator is holding the browser's arm deadman button.

Gripper fields in the JSON payload are:

```json
{
  "gripperAvailable": true,
  "gripperOpening": 0.5
}
```

`gripperOpening` is normalized and clamped by the C++ ROS node: `0.0` is closed and `1.0` is fully open. Clients that do not intend to command the gripper must omit these fields or set `gripperAvailable` to `false`.

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

When a payload includes pose fields, `webrtc_server` publishes standard ROS2 messages:

- `/vive/head_pose`: `geometry_msgs/PoseStamped` copied from the HMD pose.
- `/vive/hand_target_pose`: `geometry_msgs/PoseStamped` using the joystick wrist position and calibrated `robotWristR*` orientation.
- `/vive/hand_target_active`: `std_msgs/Bool` deadman state, published on every wrist sample so release is immediate rather than inferred only from a timeout.
- `/vive/gripper_opening`: `std_msgs/Float64` normalized opening, where `0` is closed and `1` is fully open.
- `/vive/base_active`: `std_msgs/Bool` trackpad/joystick click state.
- `/vive/base_command`: `geometry_msgs/TwistStamped` differential-drive intent. Joystick Y maps to `linear.x`; negative joystick X maps to positive `angular.z`, so left turns left. A radial deadzone is removed without dividing the pad into directional sectors.

## MoveIt server

The separate `moveit_server` container joins the same CycloneDDS graph as
`webrtc_server` and the robot. The motion path is compiled C++17/rclcpp; Python
is used only for ROS launch descriptions. By default the container starts
TIAGo's robot description, `move_group`, MoveIt Servo, and one
`vive_moveit_server` process. There is no separate pose-to-twist bridge.

The relevant files are:

- `src/vive_moveit_server.cpp`: subscriptions, deadman state machines, live TF feedback, 100 Hz Cartesian control, Servo startup, and head/gripper/base output.
- `src/control_math.cpp` and `include/vive_moveit_server/control_math.hpp`: pure quaternion, top-frame clutch mapping, filtering, workspace, and slew-limit math.
- `test/test_control_math.cpp`: deterministic unit tests for the controller math.
- `config/tiago_single_params.yaml`: teleop, Cartesian controller, watchdog, and real-time settings.
- `config/tiago_servo.yaml`: Servo timing, seven-joint group, frames, joint/singularity handling, and controller output.
- `launch/servo_runtime.launch.py`: starts Servo and rejects an `arm` group that is missing any of `arm_1_joint` through `arm_7_joint` or includes the torso.

### Arm behavior

- Unity samples at 90 Hz. Its ordered input data channel gives motion packets a 50 ms maximum lifetime, preventing an old retransmit from holding newer poses in a backlog.
- The C++ node uses depth-one latest-value input. A new pose replaces the previous pose; input is never replayed as a motion queue.
- Trigger or side grip is the arm deadman. A rising edge captures the latest valid controller pose and the live `base_footprint -> arm_tool_link` TF. The first target is therefore the measured tool pose, not an old client target.
- Translation after clutching remains aligned with `base_footprint`. Orientation change is measured in the controller's configured local top frame and applied in the end-effector's local frame. This is the top-of-controller to top-of-tool mapping.
- A steady-clock wall timer runs the Cartesian loop at 100 Hz. It low-pass filters new targets, estimates target velocity for feed-forward, computes residual pose error from nonblocking latest TF, and emits physical linear/angular velocity on `/servo_node/delta_twist_cmds`.
- Linear and angular vector norms are capped at `0.35 m/s` and `1.2 rad/s`; command slew is capped at `2.0 m/s²` and `6.0 rad/s²`. These explicit limits are applied before Servo rather than relying on whichever arm joint saturates first.
- `/servo_node/pose_target_cmds` carries the mapped, filtered target for diagnostics/recording. `/servo_node/pose_target_active` carries the effective downstream gate. They are observability outputs; the same C++ node directly generates the executable twist.
- Servo runs as its registered `moveit_servo::ServoNode` component with intra-process communication enabled and is started through `/servo_node/start_servo`. In this Humble build it accepts Cartesian twists directly; it uses Jacobian/differential inverse kinematics over all seven arm joints, then publishes `trajectory_msgs/JointTrajectory` to `/arm_controller/joint_trajectory`.
- Servo retains per-joint velocity scaling, singularity scaling, joint-limit margins, Butterworth output smoothing, and its own `0.10 s` command timeout.
- Deadman release, input older than `0.12 s`, invalid data, or tool TF older than `0.20 s` clears anchors/filter/feed-forward state and publishes four zero twists. After a timeout, the operator must release and press again.
- The spherical distance limit and Z interval are enforced simultaneously. An invalid/non-finite target fails closed.

The loop is real-time-oriented, not end-to-end hard real-time. Compose grants
`SYS_NICE`, `IPC_LOCK`, an RT-priority ulimit, and unlimited memlock. The node
attempts `mlockall(MCL_CURRENT | MCL_FUTURE)` and `SCHED_FIFO` priority 40 for
its single-threaded executor, exposes the actual result as ROS parameters, and
the runtime check requires both to be active. VR sampling, WebRTC, DDS, MoveIt
Servo, the TIAGo controller, and the host kernel still contribute latency.

Servo collision checking remains intentionally disabled because its proximity
scaling is too aggressive for this robot and deployment. This configuration is
not a replacement for the physical emergency stop, supervised low-speed
commissioning, or a clear workspace.

### Head, gripper, and base behavior

- HMD orientation becomes head pan/tilt at 20 Hz with overlapping `0.10 s` trajectory points, a `0.002 rad` deadband, and 90% joint-limit scaling.
- Normalized gripper input becomes synchronized, velocity-aware finger trajectories. Commands require fresh measured finger positions and duplicate targets are suppressed.
- Clicked trackpad/joystick input becomes a guarded differential-drive command on `/key_vel`. Defaults are `0.25 m/s`, `0.6 rad/s`, and a `0.15 s` timeout. Release stops immediately; timeout requires a new physical click edge.

### Parameters to verify on the real robot

Teleop and Cartesian settings live in
`moveit_server/src/vive_moveit_server/config/tiago_single_params.yaml`; Servo
settings live in `config/tiago_servo.yaml`.

- `arm_group` must remain `arm` for the seven-axis, no-torso path. The launch validates its exact joint membership.
- `end_effector_link` and `pose_reference_frame` default to `arm_tool_link` and `base_footprint`; they must match the TF used by `/robot_state` and Servo.
- `arm_control_rate_hz`, `hand_target_timeout_sec`, and `robot_state_timeout_sec` define the 100 Hz loop and its two independent freshness guards.
- `controller_to_tool_rotation_rpy_rad` rotates the virtual controller-top local axes into the desired tool convention. It is radians in roll, pitch, yaw order.
- `controller_top_offset_m` is the vector from the tracked controller origin to the physical control point, expressed in controller axes. An incorrect value makes pure controller rotation translate the tool.
- `hand_position_scale` defaults to one-to-one translation. Keep it positive unless an axis reversal is deliberate.
- `linear_gain`, `angular_gain`, target filter cutoff, feed-forward settings, and pose deadbands tune tracking lag versus jitter.
- `max_linear_velocity_mps`, `max_angular_velocity_radps`, `max_linear_acceleration_mps2`, and `max_angular_acceleration_radps2` are mandatory positive Cartesian limits. Bad values fall back to safe defaults, and runtime checks reject the deployment.
- `max_hand_target_distance_m`, `min_hand_target_z_m`, and `max_hand_target_z_m` define the Cartesian workspace intersection.
- `arm_halt_command_count` and `require_deadman_repress_after_timeout` define halt/rearm behavior.
- `enable_realtime_scheduling`, `realtime_priority`, `lock_memory`, and `cpu_affinity` control process-level latency optimizations. Leave CPU affinity at `-1` until an isolated CPU has been deliberately provisioned.
- `move_group_name`, planning/command/tool frames, `publish_period`, `incoming_command_timeout`, singularity thresholds, `joint_limit_margin`, and `command_out_topic` in `tiago_servo.yaml` must agree with the live TIAGo model and controller.
- `command_in_type` is `speed_units`; Servo's `scale.*` values affect only unitless input. `override_velocity_scaling_factor: 0.0` leaves automatic per-joint scaling active.
- `head_*`, `gripper_*`, and `base_*` settings retain their direct controller-topic, range, deadband, timeout, and slew meanings.

### Calibrate controller top to tool top

The default alignment and offset are zero because Unity already sends a
calibrated `robotWrist` target in the `arm_tool_link` convention. Start there;
do not add a second correction without observing an axis mismatch.

1. Use the physical emergency stop procedure and reduce Cartesian limits to commissioning values, for example `0.05 m/s` and `0.2 rad/s`.
2. Hold the controller in a comfortable pose, align its physical top with the desired top of the end effector, then press the deadman. The clutch deliberately makes this initial pose motionless.
3. Rotate one controller axis at a time without translating. Verify that tool roll, pitch, and yaw follow the corresponding local top-frame axes and signs.
4. If axes are swapped or inverted, release the deadman and adjust only `controller_to_tool_rotation_rpy_rad`. Rebuild/restart, re-clutch, and repeat one axis at a time.
5. If the desired control point is not the tracking origin, measure the origin-to-top vector on the actual Vive controller and enter it as `controller_top_offset_m`. Verify that rotation about that physical point produces the expected tool arc.
6. Increase speed, acceleration, filter cutoff, and gains gradually while watching `/servo_node/status`, measured TF, and controller trajectories. Stop increasing gain when jitter or overshoot appears.

Do not infer a physical offset from the controller shell or a generic Vive
model. The tracked origin and axes must be confirmed with the actual SteamVR
device/driver and hardware in use.

Useful Servo health checks:

```bash
./scripts/check-teleop-runtime.sh

docker exec moveit_server_wifi bash -lc \
  'source /opt/ros/humble/setup.bash &&
   ros2 service list | grep /servo_node/start_servo &&
   ros2 topic info /servo_node/delta_twist_cmds -v &&
   ros2 topic info /arm_controller/joint_trajectory -v'
```

### Gripper control

The browser debug client is the built-in gripper frontend:

1. Reload the debug page after updating the repository so the browser has the current JavaScript.
2. Click `Input`. The page reads both finger joints from `/robot_state` and displays their measured positions.
3. Change `Opening`, or use `Open` and `Close`. While input is connected, the target is interpolated and streamed automatically at 20 Hz.
4. `Send Gripper` sends the current gripper target immediately. `Send All` sends head, wrist, and gripper fields together.

Wrist commands use a separate hold-to-run control. Hold `Hold Arm Deadman` to
enable them. Pointer or keyboard release, moving focus away from the window,
hiding the page, a channel error/close, or disconnecting clears the browser's
active state. A lost channel still relies on the ROS command timeout because it
can no longer carry a release message.

Connecting input does not intentionally move the gripper. The stream starts from the measured opening, and the server ignores an initial target that is already within `gripper_deadband_m` of the current finger position.

The Unity client initializes its gripper target from `/robot_state` and sends
`gripperAvailable: true` with a normalized `gripperOpening`. Swipe the right
Vive trackpad or joystick upward to open the gripper and downward to close it.
The wrist deadman is not required for gripper motion. Each swipe starts from
the current latched opening, and the vertical displacement commands an opening
relative to that anchor. Returning to center latches the result for the next
swipe. `gripperJoystickDeadzone` suppresses center jitter, while
`gripperJoystickTravelForFullRange` sets the displacement needed to reach fully
open or closed. Clicking the trackpad switches that gesture to base driving;
gripper input stays suppressed until the click is released and the control
returns to center.

To disable the bundled TIAGo MoveGroup launch while leaving the Servo runtime enabled:

```bash
MOVEIT_SERVER_LAUNCH_ARGS="moveit_launch_enabled:=false" \
  sudo docker compose up --build moveit_server
```

To disable the bundled Servo runtime, add `servo_launch_enabled:=false`. To pass robot variant arguments through to TIAGo MoveIt and Servo, set `moveit_arm_type`, `moveit_base_type`, `moveit_end_effector`, and/or `moveit_ft_sensor` in `MOVEIT_SERVER_LAUNCH_ARGS`. `moveit_allow_trajectory_execution` defaults to `False`; Servo publishes the arm controller trajectories.

## Running

For the complete VR application, use the Linux quick-start command:

```bash
./scripts/start-vive-teleop.sh
```

For Docker services only:

```bash
./scripts/up-wifi-webrtc.sh -d
```

This starts the host-network `webrtc_server_wifi`, `moveit_server_wifi`, and
`coturn_wifi` services. The MoveIt container needs live robot `/joint_states`
and the TIAGo robot description before Servo can command the physical arm.
Validate those services and the live robot state with:

```bash
./scripts/check-teleop-runtime.sh
```

The check waits up to 60 seconds by default. Set
`TELEOP_RUNTIME_WAIT_SECONDS` to change that startup timeout.

In another terminal, serve only the debug-client asset:

```bash
./scripts/serve-debug-client.sh
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

Copy `.env.example` to `.env` to keep stable local network values and replace
the example TURN credentials before using a network with untrusted clients.
The Wi-Fi launcher validates DDS IP/boolean values and generates its CycloneDDS
file at a private, unpredictable temporary path.

If the default route is not the Wi-Fi interface, specify the interface instead of hard-coding the address:

```bash
WEBRTC_NIC=wlan0 ./scripts/up-wifi-webrtc.sh
```

The script detects the current Wi-Fi host IP, exports `WEBRTC_HOST_IP`, detects the field-network host IP as `ROS_FIELD_HOST_IP`, generates a matching CycloneDDS config, and starts host-network Wi-Fi variants:

- `webrtc_server_wifi` on the host network, so WebRTC signaling and media are reachable through the host Wi-Fi IP.
- `moveit_server_wifi` on the host network, using the same ROS2 DDS interface as the app.
- `coturn_wifi` on the host network, listening on both the Wi-Fi IP and the field-network host IP.
- direct robot discovery through `ROS_FIELD_HOST_IP` by default, with `10.68.0.1` as the DDS peer.
- client-side ICE with `WEBRTC_PUBLIC_TURN_URLS=turn:<wifi-host-ip>:3478?...`.
- server-side ICE with `WEBRTC_TURN_URLS=turn:127.0.0.1:3478?...` inside the host-network WebRTC server.

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

1. Wait for `webrtc_server_wifi` logs to show `======== Running on http://0.0.0.0:8088 ========`.
2. Check direct robot discovery with `docker exec webrtc_server_wifi bash -lc 'source /opt/ros/humble/setup.bash && ros2 topic list'`.
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

`joystickGrip` remains telemetry and part of the wrist deadman. Swiping the
right trackpad or joystick vertically controls the gripper independently and
populates `gripperAvailable` and `gripperOpening`; it does not require the wrist
deadman. Each swipe is relative to the opening latched when the gesture starts.
Up opens and down closes.

Hold the trackpad or joystick click to drive the base. Push up for forward,
down for reverse, left for positive-yaw left rotation, and right for right
rotation. Diagonal positions retain both components, producing a proportional
forward/reverse arc instead of choosing one of four discrete directions.
Releasing the click stops the base immediately.

Hold the Vive trigger or side-grip button to command the robot wrist. On every
press, Unity records the current headset position and yaw and the current
controller pose. The ROS server independently anchors the incoming target to
the robot's measured wrist pose from TF. Controller motion is then mapped
one-to-one in the fixed headset frame captured at the press. Moving or rotating
the headset afterward does not change the arm target, so looking around remains
independent from wrist control. Release the button to clear the target and
clutch state. The next press captures a new headset/controller frame and the
robot's then-current wrist pose.

Press `R` in the Unity player to request a new headset/controller workspace
anchor. The new anchor is captured when valid headset, controller, and robot
wrist states are available. If the joystick is represented by a custom tracked
GameObject, assign it to `Vive Teleop Web RTC Client > Wrist Pose Source`;
otherwise the right-hand XR node is used.

After manually repositioning the robot, release the wrist deadman and press `P`.
Unity immediately releases wrist pursuit, reloads the measured wrist and
gripper from `/robot_state`, and adopts them as the new command targets. The
next valid headset/controller sample becomes the new workspace anchor, so
tracking can restart without driving back toward the old target. If the robot
state request fails, wrist and joystick gripper commands remain disabled rather
than using stale state. `R` remains the lightweight re-anchor that keeps the
last commanded robot target; `P` is the full measured-state resynchronization.

Press `Space` or the Vive controller menu button to start or stop local 6-DoF recording. Each line in the output `.jsonl` file is the same `unity_teleop_pose` JSON object sent over WebRTC, including wrist XYZ, quaternion, calibrated robot quaternion, workspace mode, captured headset anchor pose, position scale, and controller values. On Linux, recordings default to:

```text
~/.config/unity3d/DefaultCompany/unity-vr-headset/ViveTeleopRecordings/
```

Recording can also be controlled at startup:

```bash
VIVE_TELEOP_RECORD_CONTROLLER=1 \
VIVE_TELEOP_RECORDING_DIR="$PWD/recordings" \
./unity-vr-headset/Builds/Linux/vive-teleop
```

The recommended terminal launcher sets these variables and writes a timestamped
player log automatically:

```bash
./scripts/run-unity-vr-linux.sh
```

### Planned synchronized ML dataset recording

A separate ROS 2/rosbag2 dataset recorder is designed but not implemented. The
plan records selected camera, robot state, robot-space action, command, outcome,
and episode-event streams while the wrist deadman is active, with a short
terminal post-roll. The downstream Servo gate remains a separate effective
action-validity label. Raw Unity controller data remains optional provenance
rather than the primary training action.

The implementation, topic contract, storage layout, and validation plan are in
[`data-recording/`](data-recording/README.md). The current Unity JSONL
recording remains the only implemented recording feature.

Builds can override the scene URL with either `VIVE_TELEOP_WEBRTC_CONFIG_URL` or a command-line argument:

```bash
--webrtc-config-url=http://<host-ip>:8088/config
```

Run every software check, including ROS tests and a Unity Linux compilation:

```bash
./scripts/test-software.sh
```

Individual test modes are available as `--static`, `--ros`, and `--unity`.
The ROS mode builds every buildable service in the default and Wi-Fi Compose
configuration, runs the MoveIt package's C++ `colcon` tests in its compiled
image, and runs the Python gateway tests; it does not start the runtime stack.
The Unity mode uses the editor version declared in
`ProjectSettings/ProjectVersion.txt`, or the `UNITY_EDITOR` override.

To build only the Linux player after closing the project in the Unity editor:

```bash
./scripts/build-unity-vr-linux.sh
```

## Troubleshooting

If the robot image is visible but the arm cannot move, first verify that the
robot is online and publishing joint state:

```bash
./scripts/check-teleop-runtime.sh

ping -c 3 10.68.0.1

docker exec moveit_server_wifi bash -lc \
  'source /opt/ros/humble/setup.bash && \
   ros2 topic info /joint_states -v'
```

The runtime check also verifies the C++ loop rate, positive Cartesian
speed/acceleration caps, controller-top parameter shape, active
`SCHED_FIFO`/memory locking, the deadman queue-clear gate, Servo's automatic
joint-limit scaling, all seven `arm` joints with no torso, and fresh
wrist/gripper state.

`Publisher count: 0` means the robot is not currently discoverable. The
deadman clutch intentionally waits instead of commanding the arm without a
current robot pose.

Inspect the current service logs:

```bash
docker logs --tail 100 webrtc_server_wifi
docker logs --tail 150 moveit_server_wifi
```

If the browser shows a fetch/network error for `/offer` or `/input_offer`, WebRTC negotiation has not started yet. The HTTP signaling endpoint is unreachable from the browser.

Check:

```bash
sudo docker compose logs webrtc_server
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
