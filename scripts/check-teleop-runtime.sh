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
        "moveit_servo.publish_period",
        "moveit_servo.incoming_command_timeout",
        "moveit_servo.override_velocity_scaling_factor",
        "moveit_servo.joint_limit_margin",
        "moveit_servo.halt_all_joints_in_joint_mode",
        "moveit_servo.check_collisions",
        "moveit_servo.cartesian_command_in_topic",
        "moveit_servo.joint_command_in_topic",
        "moveit_servo.status_topic",
        "robot_description_semantic",
    ],
)
teleop = get_parameters(
    node,
    "/vive_moveit_server",
    [
        "arm_group",
        "arm_control_rate_hz",
        "hand_target_topic",
        "hand_target_active_topic",
        "hand_target_timeout_sec",
        "servo_twist_topic",
        "servo_joint_topic",
        "servo_pose_target_topic",
        "servo_pose_active_topic",
        "servo_status_topic",
        "arm_halt_command_count",
        "linear_gain",
        "angular_gain",
        "max_linear_velocity_mps",
        "max_angular_velocity_radps",
        "max_linear_acceleration_mps2",
        "max_angular_acceleration_radps2",
        "target_filter_cutoff_hz",
        "feedforward_filter_alpha",
        "controller_to_tool_rotation_rpy_rad",
        "controller_top_offset_m",
        "arm_joint_names",
        "ik_model_ready",
        "ik_damping_threshold",
        "ik_maximum_damping",
        "ik_joint_motion_weights",
        "ik_singularity_avoidance_threshold",
        "ik_singularity_escape_velocity_radps",
        "ik_visibility_avoidance_enabled",
        "ik_visibility_model_ready",
        "ik_visibility_link_names",
        "ik_visibility_upward_activation_ratio",
        "ik_visibility_avoidance_velocity_radps",
        "ik_visibility_low_elbow_reward_weight",
        "ik_visibility_low_elbow_target_slope",
        "ik_visibility_gradient_step_rad",
        "ik_visibility_singularity_disable_threshold",
        "ik_visibility_singularity_full_threshold",
        "ik_visibility_joint_margin_disable_buffer_rad",
        "ik_visibility_joint_margin_full_buffer_rad",
        "ik_joint_limit_activation_ratio",
        "ik_joint_limit_weight",
        "ik_joint_centering_gain_radps",
        "ik_maximum_secondary_velocity_radps",
        "ik_joint_velocity_scale",
        "ik_joint_limit_margins_rad",
        "ik_joint_limit_margin_rad",
        "ik_joint_limit_lookahead_sec",
        "joint_limit_recovery_velocity_radps",
        "joint_limit_recovery_gain",
        "joint_limit_recovery_release_buffer_rad",
        "ik_unreachable_linear_residual_mps",
        "ik_unreachable_timeout_sec",
        "enable_realtime_scheduling",
        "realtime_scheduling_active",
        "lock_memory",
        "memory_lock_active",
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
server_arm_group = teleop["arm_group"]
servo_period = float(servo["moveit_servo.publish_period"])
servo_timeout = float(servo["moveit_servo.incoming_command_timeout"])
control_rate = float(teleop["arm_control_rate_hz"])
linear_gain = float(teleop["linear_gain"])
angular_gain = float(teleop["angular_gain"])
linear_cap = float(teleop["max_linear_velocity_mps"])
angular_cap = float(teleop["max_angular_velocity_radps"])
linear_acceleration = float(teleop["max_linear_acceleration_mps2"])
angular_acceleration = float(teleop["max_angular_acceleration_radps2"])
target_filter_cutoff = float(teleop["target_filter_cutoff_hz"])
feedforward_filter_alpha = float(teleop["feedforward_filter_alpha"])
velocity_override = float(servo["moveit_servo.override_velocity_scaling_factor"])
servo_joint_margin = float(servo["moveit_servo.joint_limit_margin"])
halt_all_joints = bool(servo["moveit_servo.halt_all_joints_in_joint_mode"])
collision_check = bool(servo["moveit_servo.check_collisions"])
servo_cartesian_input = servo["moveit_servo.cartesian_command_in_topic"]
servo_joint_input = servo["moveit_servo.joint_command_in_topic"]
servo_status_output = servo["moveit_servo.status_topic"]
hand_target_topic = teleop["hand_target_topic"]
hand_active_topic = teleop["hand_target_active_topic"]
hand_timeout = float(teleop["hand_target_timeout_sec"])
twist_topic = teleop["servo_twist_topic"]
joint_topic = teleop["servo_joint_topic"]
pose_target_topic = teleop["servo_pose_target_topic"]
pose_active_topic = teleop["servo_pose_active_topic"]
servo_status_topic = teleop["servo_status_topic"]
halt_command_count = int(teleop["arm_halt_command_count"])
controller_alignment = teleop["controller_to_tool_rotation_rpy_rad"]
controller_top_offset = teleop["controller_top_offset_m"]
arm_joint_names = teleop["arm_joint_names"]
ik_model_ready = bool(teleop["ik_model_ready"])
ik_damping_threshold = float(teleop["ik_damping_threshold"])
ik_maximum_damping = float(teleop["ik_maximum_damping"])
ik_motion_weights = [float(value) for value in teleop["ik_joint_motion_weights"]]
ik_singularity_threshold = float(teleop["ik_singularity_avoidance_threshold"])
ik_escape_velocity = float(teleop["ik_singularity_escape_velocity_radps"])
ik_visibility_enabled = bool(teleop["ik_visibility_avoidance_enabled"])
ik_visibility_model_ready = bool(teleop["ik_visibility_model_ready"])
ik_visibility_links = teleop["ik_visibility_link_names"]
ik_visibility_activation = float(
    teleop["ik_visibility_upward_activation_ratio"]
)
ik_visibility_velocity = float(
    teleop["ik_visibility_avoidance_velocity_radps"]
)
ik_visibility_low_elbow_reward = float(
    teleop["ik_visibility_low_elbow_reward_weight"]
)
ik_visibility_low_elbow_target = float(
    teleop["ik_visibility_low_elbow_target_slope"]
)
ik_visibility_gradient_step = float(teleop["ik_visibility_gradient_step_rad"])
ik_visibility_singularity_disable = float(
    teleop["ik_visibility_singularity_disable_threshold"]
)
ik_visibility_singularity_full = float(
    teleop["ik_visibility_singularity_full_threshold"]
)
ik_visibility_margin_disable = float(
    teleop["ik_visibility_joint_margin_disable_buffer_rad"]
)
ik_visibility_margin_full = float(
    teleop["ik_visibility_joint_margin_full_buffer_rad"]
)
ik_limit_activation = float(teleop["ik_joint_limit_activation_ratio"])
ik_limit_weight = float(teleop["ik_joint_limit_weight"])
ik_centering_gain = float(teleop["ik_joint_centering_gain_radps"])
ik_secondary_limit = float(teleop["ik_maximum_secondary_velocity_radps"])
ik_velocity_scale = float(teleop["ik_joint_velocity_scale"])
ik_limit_margins = [float(value) for value in teleop["ik_joint_limit_margins_rad"]]
ik_limit_margin = float(teleop["ik_joint_limit_margin_rad"])
ik_limit_lookahead = float(teleop["ik_joint_limit_lookahead_sec"])
joint_recovery_velocity = float(teleop["joint_limit_recovery_velocity_radps"])
joint_recovery_gain = float(teleop["joint_limit_recovery_gain"])
joint_recovery_buffer = float(
    teleop["joint_limit_recovery_release_buffer_rad"]
)
unreachable_residual = float(teleop["ik_unreachable_linear_residual_mps"])
unreachable_timeout = float(teleop["ik_unreachable_timeout_sec"])
realtime_requested = bool(teleop["enable_realtime_scheduling"])
realtime_active = bool(teleop["realtime_scheduling_active"])
memory_lock_requested = bool(teleop["lock_memory"])
memory_lock_active = bool(teleop["memory_lock_active"])
base_input_topic = teleop["base_input_topic"]
base_active_topic = teleop["base_active_topic"]
base_command_topic = teleop["base_command_topic"]
base_timeout = float(teleop["base_input_timeout_sec"])
base_linear_limit = float(teleop["base_max_linear_velocity_mps"])
base_angular_limit = float(teleop["base_max_angular_velocity_radps"])
base_halt_command_count = int(teleop["base_halt_command_count"])

if arm_group != "arm":
    raise SystemExit("MoveIt Servo group is not arm")
if server_arm_group != "arm":
    raise SystemExit("C++ teleop arm group is not arm")
if pose_active_topic != "/servo_node/pose_target_active":
    raise SystemExit("unexpected Servo deadman topic")
if twist_topic != "/servo_node/delta_twist_cmds":
    raise SystemExit("unexpected Servo twist topic")
if joint_topic != "/servo_node/delta_joint_cmds":
    raise SystemExit("unexpected Servo joint command topic")
if servo_status_topic != "/servo_node/status":
    raise SystemExit("unexpected Servo status topic")
if servo_status_output != servo_status_topic:
    raise SystemExit("C++ recovery does not consume MoveIt Servo status")
if servo_joint_input != joint_topic:
    raise SystemExit("MoveIt Servo does not consume the redundant IK joint topic")
if servo_cartesian_input == twist_topic:
    raise SystemExit("unweighted Servo Cartesian IK can override redundant IK")
if control_rate < 100.0:
    raise SystemExit("C++ Cartesian control rate is below 100 Hz")
if not math.isclose(servo_period, 0.01, abs_tol=1e-6):
    raise SystemExit("MoveIt Servo publish period is not 10 ms")
if not 0.02 <= servo_timeout <= hand_timeout:
    raise SystemExit("Servo timeout is not bounded by the arm input timeout")
if not all(
    math.isfinite(value) and value >= 0.0
    for value in (linear_gain, angular_gain)
):
    raise SystemExit("Cartesian feedback gains must be nonnegative")
if not 0.0 < linear_cap <= 0.5:
    raise SystemExit("linear Cartesian speed cap is outside 0..0.5 m/s")
if not 0.0 < angular_cap <= 2.0:
    raise SystemExit("angular Cartesian speed cap is outside 0..2 rad/s")
if linear_acceleration <= 0.0 or angular_acceleration <= 0.0:
    raise SystemExit("Cartesian acceleration limits must be positive")
if not 0.0 <= target_filter_cutoff <= 40.0:
    raise SystemExit("target filter cutoff is outside 0..40 Hz")
if not 0.0 <= feedforward_filter_alpha <= 1.0:
    raise SystemExit("feed-forward filter alpha is outside 0..1")
expected_arm_joints = [f"arm_{index}_joint" for index in range(1, 8)]
if arm_joint_names != expected_arm_joints:
    raise SystemExit("prioritized IK joint ordering is not arm_1 through arm_7")
if not ik_model_ready:
    raise SystemExit("prioritized IK KDL model is not ready")
if ik_damping_threshold <= 0.0 or ik_maximum_damping <= 0.0:
    raise SystemExit("prioritized IK singularity damping is disabled")
if len(ik_motion_weights) != 7 or not all(
    math.isfinite(value) and value > 0.0 for value in ik_motion_weights
):
    raise SystemExit("prioritized IK joint motion costs are invalid")
expected_ik_priority = [6, 5, 4, 2, 1, 3, 0]
actual_ik_priority = sorted(range(7), key=ik_motion_weights.__getitem__)
if actual_ik_priority != expected_ik_priority:
    raise SystemExit(
        "prioritized IK activation order is not J7 -> J6 -> J5 -> "
        "J3 -> J2 -> J4 -> J1"
    )
if ik_singularity_threshold <= 0.0 or ik_escape_velocity <= 0.0:
    raise SystemExit("prioritized IK singularity escape is disabled")
if not ik_visibility_enabled or not ik_visibility_model_ready:
    raise SystemExit("middle-arm visibility avoidance is not ready")
if ik_visibility_links != ["arm_3_link", "arm_4_link", "arm_5_link"]:
    raise SystemExit("middle-arm visibility links are not arm_3 through arm_5")
if not 0.0 <= ik_visibility_activation < 1.0:
    raise SystemExit("middle-arm upward-slope activation is invalid")
if ik_visibility_velocity <= 0.0 or ik_visibility_gradient_step <= 0.0:
    raise SystemExit("middle-arm visibility correction is disabled")
if ik_visibility_low_elbow_reward <= 0.0:
    raise SystemExit("bounded low-elbow preference is disabled")
if not -1.0 <= ik_visibility_low_elbow_target < ik_visibility_activation:
    raise SystemExit("bounded low-elbow target slope is invalid")
if not (
    0.0 <= ik_visibility_singularity_disable
    < ik_visibility_singularity_full
    <= ik_singularity_threshold
):
    raise SystemExit("visibility does not yield before singularity recovery")
if not 0.0 <= ik_visibility_margin_disable < ik_visibility_margin_full:
    raise SystemExit("visibility joint-margin safety gate is invalid")
if ik_visibility_velocity >= ik_escape_velocity:
    raise SystemExit("visibility correction can overpower singularity escape")
if not 0.0 < ik_limit_activation < 1.0 or ik_limit_weight <= 0.0:
    raise SystemExit("prioritized IK joint-limit avoidance is invalid")
if ik_centering_gain <= 0.0 or ik_secondary_limit <= 0.0:
    raise SystemExit("prioritized IK null-space motion is disabled")
if not 0.75 <= ik_velocity_scale <= 1.0:
    raise SystemExit("prioritized IK joint velocity scale is below the fallback-speed floor")
if len(ik_limit_margins) != 7 or not all(
    math.isfinite(value) and value >= 0.0 for value in ik_limit_margins
):
    raise SystemExit("prioritized IK per-joint margins are invalid")
minimum_margins = [0.14, 0.18, 0.14, 0.20, 0.24, 0.24, 0.24]
if any(
    actual + 1e-12 < required
    for actual, required in zip(ik_limit_margins, minimum_margins)
):
    raise SystemExit("prioritized IK margins lack Servo braking headroom")
if ik_limit_margin <= 0.0 or ik_limit_lookahead < 0.40:
    raise SystemExit("prioritized IK predictive joint margin is invalid")
if min(ik_limit_margins) <= servo_joint_margin:
    raise SystemExit("local IK guard does not precede the final Servo guard")
if joint_recovery_velocity <= 0.0 or joint_recovery_gain <= 0.0:
    raise SystemExit("joint-limit inward recovery is disabled")
if joint_recovery_buffer < 0.0:
    raise SystemExit("joint-limit recovery release buffer is invalid")
if unreachable_residual <= 0.0 or unreachable_timeout <= 0.0:
    raise SystemExit("unreachable-target anti-windup is disabled")
if not math.isclose(velocity_override, 0.0, abs_tol=1e-12):
    raise SystemExit("automatic Servo joint-limit scaling is overridden")
if halt_all_joints:
    raise SystemExit("one Servo joint bound can still halt the complete arm")
if halt_command_count < 1:
    raise SystemExit("deadman halt command count must be positive")
if len(controller_alignment) != 3 or not all(
    math.isfinite(float(value)) for value in controller_alignment
):
    raise SystemExit("controller-to-tool alignment must contain three finite values")
if len(controller_top_offset) != 3 or not all(
    math.isfinite(float(value)) for value in controller_top_offset
):
    raise SystemExit("controller top offset must contain three finite values")
if realtime_requested and not realtime_active:
    raise SystemExit("C++ teleop executor requested SCHED_FIFO but it is inactive")
if memory_lock_requested and not memory_lock_active:
    raise SystemExit("C++ teleop requested memory locking but it is inactive")
if not 0.02 <= base_timeout <= 0.5:
    raise SystemExit("base command timeout is outside 0.02..0.5 seconds")
if base_linear_limit <= 0.0 or base_angular_limit <= 0.0:
    raise SystemExit("base velocity limits must be positive")
if base_halt_command_count < 1:
    raise SystemExit("base halt command count must be positive")

required_subscriptions = [
    hand_target_topic,
    hand_active_topic,
    joint_topic,
    servo_status_topic,
    base_input_topic,
    base_active_topic,
    base_command_topic,
]
required_publishers = [
    pose_target_topic,
    pose_active_topic,
    twist_topic,
    joint_topic,
    servo_status_topic,
]
deadline = time.monotonic() + 3.0
while True:
    missing_subscriptions = [
        topic
        for topic in required_subscriptions
        if not node.get_subscriptions_info_by_topic(topic)
    ]
    missing_publishers = [
        topic
        for topic in required_publishers
        if not node.get_publishers_info_by_topic(topic)
    ]
    if (
        (not missing_subscriptions and not missing_publishers)
        or time.monotonic() >= deadline
    ):
        break
    rclpy.spin_once(node, timeout_sec=0.2)
if missing_subscriptions:
    raise SystemExit(
        "topics without subscribers: " + ", ".join(missing_subscriptions)
    )
if missing_publishers:
    raise SystemExit("topics without publishers: " + ", ".join(missing_publishers))

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
expected = set(expected_arm_joints)
missing = sorted(expected - joints)
if missing:
    raise SystemExit("MoveIt arm group is missing: " + ", ".join(missing))
if "torso_lift_joint" in joints:
    raise SystemExit("MoveIt arm group unexpectedly contains torso_lift_joint")

print("arm joints=" + ",".join(sorted(expected)))
print(
    f"group={arm_group} cpp_control_hz={control_rate} servo_period={servo_period} "
    f"gains={linear_gain},{angular_gain} "
    f"linear_cap={linear_cap} angular_cap={angular_cap} "
    f"acceleration_caps={linear_acceleration},{angular_acceleration} "
    f"target_filter_hz={target_filter_cutoff} "
    f"feedforward_alpha={feedforward_filter_alpha} "
    f"ik=prioritized_bounded_dls joint_velocity_scale={ik_velocity_scale} "
    f"joint_limit_scaling=automatic collision_check={str(collision_check).lower()} "
    f"deadman_halt_topic={pose_active_topic} halt_commands={halt_command_count} "
    f"sched_fifo={str(realtime_active).lower()} mlock={str(memory_lock_active).lower()} "
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
