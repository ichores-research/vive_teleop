#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
container="${MOVEIT_CONTAINER:-moveit_server_wifi}"
wait_seconds="${TELEOP_RUNTIME_WAIT_SECONDS:-60}"
host_ip="${WEBRTC_HOST_IP:-$("$script_dir/detect-webrtc-host-ip.sh")}"
state_url="${VIVE_TELEOP_ROBOT_STATE_URL:-http://${host_ip}:8088/robot_state}"

if [[ -z "$host_ip" ]]; then
  printf 'Could not detect the WebRTC host IP. Set WEBRTC_HOST_IP or WEBRTC_NIC.\n' >&2
  exit 1
fi

check_servo() {
  timeout 12 docker exec "$container" bash -lc '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    source /moveit_ws/install/setup.bash
    exec python3 - <<"PY"
import math
import time
import xml.etree.ElementTree as ET

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import parameter_value_to_python


def get_parameters(node, node_name, names):
    client = node.create_client(GetParameters, node_name + "/get_parameters")
    if not client.wait_for_service(timeout_sec=2.0):
        raise SystemExit("parameter service unavailable: " + node_name)
    request = GetParameters.Request()
    request.names = names
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
    if not future.done() or future.result() is None:
        raise SystemExit("parameter request timed out: " + node_name)
    values = [parameter_value_to_python(value) for value in future.result().values]
    return dict(zip(names, values))


rclpy.init()
node = rclpy.create_node("vive_teleop_runtime_probe")

servo = get_parameters(
    node,
    "/servo_node",
    [
        "moveit_servo.move_group_name",
        "moveit_servo.override_velocity_scaling_factor",
        "moveit_servo.check_collisions",
        "robot_description_semantic",
    ],
)
bridge = get_parameters(
    node,
    "/servo_pose_bridge",
    [
        "max_linear_velocity_mps",
        "max_angular_velocity_radps",
        "pose_active_topic",
        "halt_command_count",
    ],
)
base = get_parameters(
    node,
    "/vive_moveit_server",
    [
        "base_input_topic",
        "base_active_topic",
        "base_command_topic",
        "base_input_timeout_sec",
        "base_max_linear_velocity_mps",
        "base_max_angular_velocity_radps",
        "base_halt_command_count",
    ],
)

arm_group = servo["moveit_servo.move_group_name"]
linear_cap = float(bridge["max_linear_velocity_mps"])
angular_cap = float(bridge["max_angular_velocity_radps"])
velocity_override = float(servo["moveit_servo.override_velocity_scaling_factor"])
collision_check = bool(servo["moveit_servo.check_collisions"])
pose_active_topic = bridge["pose_active_topic"]
halt_command_count = int(bridge["halt_command_count"])
base_input_topic = base["base_input_topic"]
base_active_topic = base["base_active_topic"]
base_command_topic = base["base_command_topic"]
base_timeout = float(base["base_input_timeout_sec"])
base_linear_limit = float(base["base_max_linear_velocity_mps"])
base_angular_limit = float(base["base_max_angular_velocity_radps"])
base_halt_command_count = int(base["base_halt_command_count"])

if arm_group != "arm":
    raise SystemExit("MoveIt Servo group is not arm")
if pose_active_topic != "/servo_node/pose_target_active":
    raise SystemExit("unexpected Servo deadman topic")
if not math.isclose(linear_cap, 0.0, abs_tol=1e-12):
    raise SystemExit("linear bridge velocity cap is not disabled")
if not math.isclose(angular_cap, 0.0, abs_tol=1e-12):
    raise SystemExit("angular bridge velocity cap is not disabled")
if not math.isclose(velocity_override, 0.0, abs_tol=1e-12):
    raise SystemExit("automatic Servo joint-limit scaling is overridden")
if halt_command_count < 1:
    raise SystemExit("deadman halt command count must be positive")
if not 0.02 <= base_timeout <= 0.5:
    raise SystemExit("base command timeout is outside 0.02..0.5 seconds")
if base_linear_limit <= 0.0 or base_angular_limit <= 0.0:
    raise SystemExit("base velocity limits must be positive")
if base_halt_command_count < 1:
    raise SystemExit("base halt command count must be positive")

required_topics = [pose_active_topic, base_input_topic, base_active_topic, base_command_topic]
deadline = time.monotonic() + 3.0
while True:
    missing_topics = [
        topic
        for topic in required_topics
        if not node.get_subscriptions_info_by_topic(topic)
    ]
    if not missing_topics or time.monotonic() >= deadline:
        break
    rclpy.spin_once(node, timeout_sec=0.2)
if missing_topics:
    raise SystemExit("topics without subscribers: " + ", ".join(missing_topics))

root = ET.fromstring(servo["robot_description_semantic"])
group = next(
    (item for item in root.findall("group") if item.get("name") == "arm"),
    None,
)
if group is None:
    raise SystemExit("MoveIt arm group is unavailable")
joints = {
    item.get("name")
    for item in group.findall("joint")
    if item.get("name")
}
expected = {f"arm_{index}_joint" for index in range(1, 8)}
missing = sorted(expected - joints)
if missing:
    raise SystemExit("MoveIt arm group is missing: " + ", ".join(missing))
if "torso_lift_joint" in joints:
    raise SystemExit("MoveIt arm group unexpectedly contains torso_lift_joint")

print("arm joints=" + ",".join(sorted(expected)))
print(
    f"group={arm_group} linear_cap={linear_cap} angular_cap={angular_cap} "
    f"joint_limit_scaling=automatic collision_check={str(collision_check).lower()} "
    f"deadman_halt_topic={pose_active_topic} halt_commands={halt_command_count} "
    f"base_output={base_command_topic} base_timeout={base_timeout} "
    f"base_limits={base_linear_limit},{base_angular_limit} "
    f"base_halt_commands={base_halt_command_count}"
)

node.destroy_node()
rclpy.shutdown()
PY
  '
}

check_robot_state() {
  curl --fail --silent --max-time 3 "$state_url" |
    python3 -c '
import json
import sys

state = json.load(sys.stdin)
if not state.get("ready"):
    errors = "; ".join(state.get("errors", []))
    raise SystemExit("robot state is not ready: " + errors)
if not state.get("wrist"):
    raise SystemExit("robot wrist state is unavailable")
gripper = state.get("gripper")
if not gripper:
    raise SystemExit("robot gripper state is unavailable")
print(
    "wrist=ready gripper=ready opening={:.3f}".format(
        float(gripper["opening"])
    )
)
'
}

printf 'Waiting for MoveIt Servo runtime in %s\n' "$container"
servo_report=""
wait_deadline=$((SECONDS + wait_seconds))
while (( SECONDS < wait_deadline )); do
  if servo_report="$(check_servo 2>/dev/null)"; then
    break
  fi
  sleep 1
done
if [[ -z "$servo_report" ]]; then
  printf 'MoveIt Servo runtime did not pass validation.\n' >&2
  check_servo
  exit 1
fi
printf '%s\n' "$servo_report"
if [[ "$servo_report" == *"collision_check=false"* ]]; then
  printf 'Warning: Servo collision checking is disabled for maximum speed.\n' >&2
fi

printf 'Waiting for fresh robot wrist and gripper state at %s\n' "$state_url"
robot_report=""
wait_deadline=$((SECONDS + wait_seconds))
while (( SECONDS < wait_deadline )); do
  if robot_report="$(check_robot_state 2>/dev/null)"; then
    break
  fi
  sleep 1
done
if [[ -z "$robot_report" ]]; then
  printf 'Robot state did not become ready.\n' >&2
  check_robot_state
  exit 1
fi
printf '%s\n' "$robot_report"
