#!/usr/bin/env bash
set -euo pipefail

recorder_image="${DATA_RECORDER_TEST_IMAGE:-vive_teleop-data_recorder:latest}"
recorder_binary="/recorder_ws/install/vive_dataset_recorder/lib/vive_dataset_recorder/vive_dataset_recorder"
test_domain="${DATA_RECORDER_TEST_DOMAIN_ID:-$((100 + ($$ % 100)))}"
test_container="vive-data-recorder-test-$$"
test_session="synthetic-camera"
recording_root="$(mktemp -d /tmp/vive-data-recorder-test.XXXXXX)"
recording_uid="$(id -u)"
recording_gid="$(id -g)"

cleanup() {
  local status=$?
  if (( status != 0 )); then
    docker logs "$test_container" >&2 2>/dev/null || true
  fi
  docker rm -f "$test_container" >/dev/null 2>&1 || true
  case "$recording_root" in
    /tmp/vive-data-recorder-test.*)
      rm -rf -- "$recording_root"
      ;;
  esac
  exit "$status"
}
trap cleanup EXIT

docker run -d \
  --name "$test_container" \
  --network host \
  --user "${recording_uid}:${recording_gid}" \
  -e HOME=/tmp \
  -e ROS_LOG_DIR=/tmp/vive-data-recorder-test-logs \
  -e ROS_DOMAIN_ID="$test_domain" \
  -e VIVE_TELEOP_SESSION_ID="$test_session" \
  -v "$recording_root:/recordings" \
  "$recorder_image" \
  "$recorder_binary" \
  --ros-args \
  --params-file /recorder_ws/install/vive_dataset_recorder/share/vive_dataset_recorder/config/recorder.yaml \
  -p bootstrap_timeout_sec:=0.5 \
  -p minimum_free_space_bytes:=0 >/dev/null

recorder_paused=false
for _ in $(seq 1 50); do
  if docker logs "$test_container" 2>&1 | grep -Fq 'Pausing recording.'; then
    recorder_paused=true
    break
  fi
  if ! docker ps --format '{{.Names}}' | grep -Fxq "$test_container"; then
    break
  fi
  sleep 0.1
done
if [[ "$recorder_paused" != "true" ]]; then
  printf '%s\n' 'Dataset recorder did not reach its ready-paused state.' >&2
  exit 1
fi

docker run --rm \
  --network host \
  -e ROS_DOMAIN_ID="$test_domain" \
  --entrypoint bash \
  "$recorder_image" \
  -lc '
    source /opt/ros/humble/setup.bash
    source /recorder_ws/install/setup.bash
    ros2 topic pub -r 20 -t 40 \
      --qos-reliability reliable --qos-durability volatile \
      /vive/hand_target_active std_msgs/msg/Bool "{data: true}" >/dev/null &
    gate_publisher_pid=$!
    sleep 0.2
    ros2 topic pub -r 10 -t 6 --qos-profile sensor_data \
      /head_front_camera/rgb/image_raw sensor_msgs/msg/Image \
      "{header: auto, height: 2, width: 2, encoding: rgb8, is_bigendian: 0, step: 6, data: [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]}" >/dev/null
    ros2 topic pub -1 --qos-profile sensor_data \
      /head_front_camera/rgb/camera_info sensor_msgs/msg/CameraInfo \
      "{header: auto, height: 2, width: 2}" >/dev/null
    wait "$gate_publisher_pid"
    ros2 topic pub -1 \
      --qos-reliability reliable --qos-durability volatile \
      /vive/hand_target_active std_msgs/msg/Bool "{data: false}" >/dev/null
  '

sleep 1
docker stop --time 10 "$test_container" >/dev/null
if [[ "$(docker inspect --format '{{.State.ExitCode}}' "$test_container")" != "0" ]]; then
  printf '%s\n' 'Dataset recorder did not shut down cleanly.' >&2
  exit 1
fi

docker run --rm \
  -v "$recording_root:/recordings:ro" \
  --entrypoint bash \
  "$recorder_image" \
  -lc 'source /opt/ros/humble/setup.bash &&
    source /recorder_ws/install/setup.bash &&
    python3 - /recordings/'"$test_session"' <<'"'"'PY'"'"'
import json
from pathlib import Path
import sys

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from sensor_msgs.msg import Image
from vive_dataset_recorder.msg import RecordingEvent

session_path = Path(sys.argv[1])
reader = SequentialReader()
reader.open(
    StorageOptions(uri=str(session_path / "bag"), storage_id="mcap"),
    ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    ),
)

images = []
camera_info_count = 0
frame_label_count = 0
event_types = []
while reader.has_next():
    topic, serialized, _ = reader.read_next()
    if topic == "/head_front_camera/rgb/image_raw":
        images.append(deserialize_message(serialized, Image))
    elif topic == "/head_front_camera/rgb/camera_info":
        camera_info_count += 1
    elif topic == "/teleop/recording/deadman_frame_state":
        frame_label_count += 1
    elif topic == "/teleop/recording/events":
        event = deserialize_message(serialized, RecordingEvent)
        event_types.append(event.event_type)

assert images, "MCAP contains no camera images"
assert all(image.height == 2 and image.width == 2 for image in images)
assert all(list(image.data) == list(range(1, 13)) for image in images)
assert camera_info_count == 1, camera_info_count
assert frame_label_count == len(images), (frame_label_count, len(images))
assert RecordingEvent.SESSION_START in event_types, event_types
assert RecordingEvent.SESSION_END in event_types, event_types

manifest = json.loads((session_path / "manifest.json").read_text(encoding="utf-8"))
assert manifest["status"] == "complete", manifest
assert "/head_front_camera/rgb/image_raw" in manifest["record_topics"]
print(
    f"Dataset recorder camera integration passed: {len(images)} images, "
    f"{frame_label_count} labels"
)
PY'

