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
- `image_listener/input_publisher.py`: mock ROS2 publisher for WebRTC input messages on `/vive/input_mock`.
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

Accepts a WebRTC offer for an input data channel and returns an answer. Messages received on the data channel are forwarded to the mock ROS2 publisher on `/vive/input_mock`.

String payloads are published as-is. Binary payloads are encoded as JSON with base64 data.

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

For browser debugging:

1. Wait for `ros2_app` logs to show `Bridge topic detected`.
2. Wait for `======== Running on http://0.0.0.0:8088 ========`.
3. In the debug page, try `Host :8088` first.
4. If fetches to `/offer` fail, try `10.68.0.132`.
5. Click `Start Video` to connect to `/offer`.
6. Click `Input` to connect to `/input_offer`, then send a payload.

For Unity:

1. Start the containers.
2. Launch Unity and open `unity-vr-headset`.
3. Launch SteamVR.
4. Press Play.

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
