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
    set -u

    parameter_value() {
      ros2 param get "$1" "$2" |
        sed -E "s/^[A-Za-z]+ value is: //"
    }

    arm_group="$(parameter_value /servo_node moveit_servo.move_group_name)"
    linear_cap="$(
      parameter_value /servo_pose_bridge max_linear_velocity_mps
    )"
    angular_cap="$(
      parameter_value /servo_pose_bridge max_angular_velocity_radps
    )"
    pose_active_topic="$(
      parameter_value /servo_pose_bridge pose_active_topic
    )"
    halt_command_count="$(
      parameter_value /servo_pose_bridge halt_command_count
    )"
    velocity_override="$(
      parameter_value \
        /servo_node \
        moveit_servo.override_velocity_scaling_factor
    )"
    collision_check="$(
      parameter_value /servo_node moveit_servo.check_collisions |
        tr "[:upper:]" "[:lower:]"
    )"

    [[ "$arm_group" == "arm" ]]
    [[ "$pose_active_topic" == "/servo_node/pose_target_active" ]]
    pose_active_subscribers="$(
      ros2 topic info "$pose_active_topic" |
        sed -n -E "s/^Subscription count: ([0-9]+)$/\1/p"
    )"
    [[ "$pose_active_subscribers" =~ ^[1-9][0-9]*$ ]]
    python3 \
      - "$linear_cap" "$angular_cap" "$velocity_override" \
      "$halt_command_count" <<"PY"
import math
import sys

linear_cap, angular_cap, velocity_override = map(float, sys.argv[1:4])
halt_command_count = int(sys.argv[4])
if not math.isclose(linear_cap, 0.0, abs_tol=1e-12):
    raise SystemExit("linear bridge velocity cap is not disabled")
if not math.isclose(angular_cap, 0.0, abs_tol=1e-12):
    raise SystemExit("angular bridge velocity cap is not disabled")
if not math.isclose(velocity_override, 0.0, abs_tol=1e-12):
    raise SystemExit("automatic Servo joint-limit scaling is overridden")
if halt_command_count < 1:
    raise SystemExit("deadman halt command count must be positive")
PY

    ros2 param get /servo_node robot_description_semantic |
      python3 -c "
import sys
import xml.etree.ElementTree as ET

value = sys.stdin.read()
prefix = \"String value is: \"
if not value.startswith(prefix):
    raise SystemExit(\"robot_description_semantic is unavailable\")
root = ET.fromstring(value[len(prefix):].strip())
group = next(
    (item for item in root.findall(\"group\") if item.get(\"name\") == \"arm\"),
    None,
)
if group is None:
    raise SystemExit(\"MoveIt arm group is unavailable\")
joints = {
    item.get(\"name\")
    for item in group.findall(\"joint\")
    if item.get(\"name\")
}
expected = {f\"arm_{index}_joint\" for index in range(1, 8)}
missing = sorted(expected - joints)
if missing:
    raise SystemExit(\"MoveIt arm group is missing: \" + \", \".join(missing))
if \"torso_lift_joint\" in joints:
    raise SystemExit(\"MoveIt arm group unexpectedly contains torso_lift_joint\")
print(\"arm joints=\" + \",\".join(sorted(expected)))
"

    printf "group=%s linear_cap=%s angular_cap=%s " \
      "$arm_group" "$linear_cap" "$angular_cap"
    printf "joint_limit_scaling=automatic collision_check=%s " \
      "$collision_check"
    printf "deadman_halt_topic=%s halt_commands=%s\n" \
      "$pose_active_topic" "$halt_command_count"
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
for _attempt in $(seq 1 "$wait_seconds"); do
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
for _attempt in $(seq 1 "$wait_seconds"); do
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
