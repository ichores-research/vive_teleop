# Runtime and Network Context

## Responsibility

Runtime wiring lives in Docker Compose files, shell scripts, CycloneDDS config, and TURN config. This layer detects host/robot-facing addresses, starts containers, validates ROS/Servo readiness, builds/runs Unity, and exposes WebRTC signaling.

## Key Files

- `docker-compose.yml`: ipvlan field network runtime.
- `docker-compose.wifi.yml`: host-network Wi-Fi runtime.
- `coturn/turnserver.conf`: TURN relay config for field-network mode.
- `webrtc_server/cyclonedds.xml`: ROS 2 discovery config for `webrtc_server`.
- `moveit_server/cyclonedds.xml`: ROS 2 discovery config for `moveit_server`.
- `scripts/up-wifi-webrtc.sh`: host-network Compose setup with generated CycloneDDS config.
- `scripts/start-vive-teleop.sh`: end-to-end startup.
- `scripts/check-teleop-runtime.sh`: Servo and robot state validation.
- `scripts/detect-field-host-ip.sh`: robot-facing interface detection.
- `scripts/detect-webrtc-host-ip.sh`: WebRTC-facing interface detection.
- `scripts/serve-debug-client.sh`: serves `index.html`.

## Network Modes

Field-network mode:

- `webrtc_server`: `10.68.0.132`
- `coturn`: `10.68.0.133`
- `moveit_server`: `10.68.0.134`
- Robot ROS 2 graph: `10.68.0.1`
- Docker network driver: ipvlan L2.

Wi-Fi mode:

- `webrtc_server_wifi`, `moveit_server_wifi`, and `coturn_wifi` use host networking.
- Scripts generate environment and CycloneDDS host config from detected interfaces.
- TURN listens on localhost, the WebRTC host IP, and the robot-facing host IP.

## Startup Contract

`scripts/start-vive-teleop.sh` should:

1. Build Unity when sources are newer than the build stamp.
2. Start Wi-Fi WebRTC containers.
3. Wait for `/config`.
4. Wait for the robot camera publisher.
5. Run `check-teleop-runtime.sh`.
6. Start SteamVR if needed.
7. Run the Unity player.

## Runtime Logs

`scripts/start-vive-teleop.sh` creates a per-run directory under `logs/` by default. The location can be changed with `VIVE_TELEOP_LOG_ROOT` or fixed exactly with `VIVE_TELEOP_RUN_LOG_DIR`.

Expected files:

- `start-vive-teleop.log`: top-level startup timeline.
- `unity-build.log`: Unity build output when a build is needed.
- `docker-compose-up.log`: Compose startup/build output.
- `webrtc_server_wifi.log`: WebRTC gateway container output.
- `moveit_server_wifi.log`: MoveIt/teleop container output.
- `coturn_wifi.log`: TURN relay output.
- `check-teleop-runtime.log`: runtime validation output.
- `steamvr-start.log`: Steam launch output when SteamVR is started by the script.
- `unity-wrapper.log`: Unity runner wrapper output.
- `unity-player.log`: Unity player log written by `-logFile`.

## Data Stability Notes

- `check-teleop-runtime.sh` validates the Servo group, pose active subscriber, halt command count, bridge velocity cap policy, automatic joint-limit scaling, and `/robot_state` readiness.
- Current runtime checks report the configured collision-checking state but do
  not gate startup on it; disabled checking is intentional for this robot.
- `coturn/coturn:latest` and `ros:humble` are floating image tags. Pin tags or digests when reproducibility matters.
- Avoid changing ROS domain, CycloneDDS peers, or container IPs without updating README, diagrams, and startup checks together.

## Future Dataset Recorder

The proposed `data_recorder`/`data_recorder_wifi` service is documented under
`docs/data-recording/`. It must share ROS domain/RMW/CycloneDDS configuration,
write to a dedicated host volume, and remain independent from control-service
health.

Future startup integration must generate one session ID, create a seed
manifest, optionally start the recorder, follow a separate recorder log, and
wait for graceful bag finalization during shutdown. The tentative field-network
address is `10.68.0.135` and must be conflict-checked before implementation.
