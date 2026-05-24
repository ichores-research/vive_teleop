# vive_teleop

`vive_teleop` bridges a robot ROS1 camera topic into ROS2, serves it over WebRTC, and accepts WebRTC input data that can later become teleoperation commands.

The Unity VR client is still the intended headset frontend, but `index.html` can be used as a lightweight browser debug client without launching Unity or SteamVR.

## Architecture

The current system has four main pieces:

- `ros1_bridge`: ROS1 Noetic to ROS2 Foxy dynamic bridge. It connects to the robot ROS master and exposes ROS1 topics into ROS2.
- `ros2_app`: ROS2 Humble application. It waits for `/xtion/rgb/image_raw`, runs the WebRTC HTTP signaling server, serves camera video on `/offer`, and accepts data-channel input on `/input_offer`.
- `coturn`: TURN relay used by WebRTC peers in the current network setup.
- `index.html` / `unity-vr-headset`: WebRTC clients. The browser page is for debugging; Unity is the VR client.

The WebRTC server code is separated from ROS subscriber/publisher logic:

- `image_listener/webrtc_server.py`: aiohttp signaling, peer lifecycle, ICE config, media relay, and data-channel routing.
- `image_listener/image_subscriber.py`: ROS2 image subscriber for `/xtion/rgb/image_raw`.
- `image_listener/video_track.py`: aiortc video track backed by the latest ROS image frame.
- `image_listener/input_publisher.py`: ROS2 publisher for raw WebRTC input messages on `/vive/input_mock` and calibrated wrist orientation commands on `/vive/robot_wrist_orientation`.
- `image_listener/teleop_webrtc.py`: composition entry point used through `image_subscriber`.

See [architecture.puml](architecture.puml) for the PlantUML source. Existing rendered reference: [architecture.png](architecture.png).

## Network Layout

Runtime containers use the `field_net` ipvlan network:

- Robot / ROS master: `10.68.0.1`
- `ros1_bridge`: `10.68.0.131`
- `ros2_app`: `10.68.0.132`
- `coturn`: `10.68.0.133`

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

Accepts a WebRTC offer for an input data channel and returns an answer. Messages received on the data channel are forwarded to the raw ROS2 debug topic `/vive/input_mock`.

String payloads are published as-is. Binary payloads are encoded as JSON with base64 data.

When a Unity payload includes `wristAvailable: true` and `robotWristR*` quaternion fields, `ros2_app` also publishes `geometry_msgs/QuaternionStamped` on `/vive/robot_wrist_orientation`. Subscribe the robot wrist controller to that topic to copy the calibrated joystick wrist rotation.

```bash
ros2 topic echo /vive/robot_wrist_orientation
```

## Running

Start the containers:

```bash
sudo docker compose up --build
```

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
- `coturn_wifi` on the host network, listening on both the Wi-Fi IP and the field-network host IP.
- local ROS2 bridge/app discovery over loopback by default with `ROS2_DDS_INTERFACE=lo`.
- client-side ICE with `WEBRTC_PUBLIC_TURN_URLS=turn:<wifi-host-ip>:3478?...`.
- server-side ICE with `WEBRTC_TURN_URLS=turn:<wifi-host-ip>:3478?...`.

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
6. Click `Input` to connect to `/input_offer`, then send a payload.

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
- A calibrated relative `robotWristR*` quaternion, published by `ros2_app` as `/vive/robot_wrist_orientation`.
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
