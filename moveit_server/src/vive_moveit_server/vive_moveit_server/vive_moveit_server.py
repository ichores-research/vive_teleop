import math
from typing import Dict, List, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.task import Future

from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MoveItErrorCodes,
    MotionPlanRequest,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def _vector_parameter(value: object, fallback: list[float]) -> list[float]:
    if isinstance(value, (list, tuple)) and len(value) == 3:
        return [float(item) for item in value]
    return list(fallback)


def _float_pair_parameter(value: object, fallback: list[float]) -> list[float]:
    if isinstance(value, (list, tuple)) and len(value) == 2:
        return [float(item) for item in value]
    return list(fallback)


def _string_list_parameter(value: object, fallback: list[str]) -> list[str]:
    if isinstance(value, (list, tuple)):
        return [str(item) for item in value if str(item)]
    return list(fallback)


def _declare_string_list_parameter(
    node: Node,
    name: str,
    fallback: list[str],
) -> list[str]:
    default_value = fallback if fallback else Parameter.Type.STRING_ARRAY
    return _string_list_parameter(
        node.declare_parameter(name, default_value).value,
        fallback,
    )


def _select_joint_values(
    values: object,
    selected_indices: list[int],
    source_joint_count: int,
) -> list[float]:
    if len(values) != source_joint_count:
        return []
    return [float(values[index]) for index in selected_indices]


def _normalize_quaternion(quaternion: Quaternion) -> bool:
    norm_squared = (
        quaternion.x * quaternion.x
        + quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
        + quaternion.w * quaternion.w
    )
    if norm_squared < 1e-12:
        return False

    norm = math.sqrt(norm_squared)
    quaternion.x /= norm
    quaternion.y /= norm
    quaternion.z /= norm
    quaternion.w /= norm
    return True


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _position_distance(left: PoseStamped, right: PoseStamped) -> float:
    dx = left.pose.position.x - right.pose.position.x
    dy = left.pose.position.y - right.pose.position.y
    dz = left.pose.position.z - right.pose.position.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def _position_norm(message: PoseStamped) -> float:
    position = message.pose.position
    return math.sqrt(
        (position.x * position.x)
        + (position.y * position.y)
        + (position.z * position.z)
    )


def _orientation_distance(left: PoseStamped, right: PoseStamped) -> float:
    lq = left.pose.orientation
    rq = right.pose.orientation
    dot = abs((lq.x * rq.x) + (lq.y * rq.y) + (lq.z * rq.z) + (lq.w * rq.w))
    dot = max(0.0, min(1.0, dot))
    return 2.0 * math.acos(dot)


def _copy_pose_stamped(message: PoseStamped) -> PoseStamped:
    copy = PoseStamped()
    copy.header.stamp = message.header.stamp
    copy.header.frame_id = message.header.frame_id
    copy.pose.position.x = message.pose.position.x
    copy.pose.position.y = message.pose.position.y
    copy.pose.position.z = message.pose.position.z
    copy.pose.orientation.x = message.pose.orientation.x
    copy.pose.orientation.y = message.pose.orientation.y
    copy.pose.orientation.z = message.pose.orientation.z
    copy.pose.orientation.w = message.pose.orientation.w
    return copy


def _multiply_quaternions(left: Quaternion, right: Quaternion) -> Quaternion:
    result = Quaternion()
    result.x = (
        left.w * right.x
        + left.x * right.w
        + left.y * right.z
        - left.z * right.y
    )
    result.y = (
        left.w * right.y
        - left.x * right.z
        + left.y * right.w
        + left.z * right.x
    )
    result.z = (
        left.w * right.z
        + left.x * right.y
        - left.y * right.x
        + left.z * right.w
    )
    result.w = (
        left.w * right.w
        - left.x * right.x
        - left.y * right.y
        - left.z * right.z
    )
    _normalize_quaternion(result)
    return result


def _inverse_quaternion(quaternion: Quaternion) -> Quaternion:
    result = Quaternion()
    result.x = -quaternion.x
    result.y = -quaternion.y
    result.z = -quaternion.z
    result.w = quaternion.w
    _normalize_quaternion(result)
    return result


def _interpolate_quaternion(
    start: Quaternion,
    target: Quaternion,
    fraction: float,
) -> Quaternion:
    fraction = _clamp(fraction, 0.0, 1.0)
    dot = (
        start.x * target.x
        + start.y * target.y
        + start.z * target.z
        + start.w * target.w
    )
    target_sign = -1.0 if dot < 0.0 else 1.0
    result = Quaternion()
    result.x = start.x + fraction * ((target.x * target_sign) - start.x)
    result.y = start.y + fraction * ((target.y * target_sign) - start.y)
    result.z = start.z + fraction * ((target.z * target_sign) - start.z)
    result.w = start.w + fraction * ((target.w * target_sign) - start.w)
    if not _normalize_quaternion(result):
        result.w = 1.0
    return result


class ViveMoveItServer(Node):
    def __init__(self) -> None:
        super().__init__("vive_moveit_server")

        self.arm_group = self.declare_parameter("arm_group", "arm_torso").value
        self.end_effector_link = self.declare_parameter(
            "end_effector_link",
            "arm_tool_link",
        ).value

        head_input_topic = self.declare_parameter(
            "head_input_topic",
            "/vive/head_pose",
        ).value
        head_command_topic = self.declare_parameter(
            "head_command_topic",
            "/head_controller/joint_trajectory",
        ).value
        self.head_joint_names = _declare_string_list_parameter(
            self,
            "head_joint_names",
            ["head_1_joint", "head_2_joint"],
        )
        if len(self.head_joint_names) != 2:
            self.get_logger().warn(
                "head_joint_names must contain exactly pan and tilt joints; "
                "using TIAGo defaults"
            )
            self.head_joint_names = ["head_1_joint", "head_2_joint"]
        hand_target_topic = self.declare_parameter(
            "hand_target_topic",
            "/vive/hand_target_pose",
        ).value
        arm_command_topic = self.declare_parameter(
            "arm_command_topic",
            "/arm_controller/joint_trajectory",
        ).value
        torso_command_topic = self.declare_parameter(
            "torso_command_topic",
            "/torso_controller/joint_trajectory",
        ).value
        gripper_input_topic = self.declare_parameter(
            "gripper_input_topic",
            "/vive/gripper_opening",
        ).value
        gripper_command_topic = self.declare_parameter(
            "gripper_command_topic",
            "/gripper_controller/joint_trajectory",
        ).value
        joint_state_topic = self.declare_parameter(
            "joint_state_topic",
            "/joint_states",
        ).value
        self.move_group_action_name = self.declare_parameter(
            "move_group_action_name",
            "/move_action",
        ).value
        self.ik_service_name = self.declare_parameter(
            "ik_service_name",
            "/compute_ik",
        ).value
        self.fk_service_name = self.declare_parameter(
            "fk_service_name",
            "/compute_fk",
        ).value

        self.execution_mode = self.declare_parameter("execution_mode", "moveit").value
        self.async_execution = bool(
            self.declare_parameter("async_execution", True).value
        )
        self.pose_reference_frame = self.declare_parameter(
            "pose_reference_frame",
            "base_footprint",
        ).value
        self.planning_time_sec = float(
            self.declare_parameter("planning_time_sec", 0.35).value
        )
        self.max_velocity_scaling_factor = float(
            self.declare_parameter("max_velocity_scaling_factor", 0.25).value
        )
        self.max_acceleration_scaling_factor = float(
            self.declare_parameter("max_acceleration_scaling_factor", 0.25).value
        )
        self.min_plan_interval_sec = float(
            self.declare_parameter("min_plan_interval_sec", 0.25).value
        )
        self.hand_target_timeout_sec = float(
            self.declare_parameter("hand_target_timeout_sec", 0.12).value
        )
        self.cartesian_position_step_m = float(
            self.declare_parameter("cartesian_position_step_m", 0.02).value
        )
        self.cartesian_orientation_step_rad = float(
            self.declare_parameter(
                "cartesian_orientation_step_rad",
                0.12,
            ).value
        )
        self.position_deadband_m = float(
            self.declare_parameter("position_deadband_m", 0.01).value
        )
        self.orientation_deadband_rad = float(
            self.declare_parameter("orientation_deadband_rad", 0.035).value
        )
        self.goal_position_tolerance_m = float(
            self.declare_parameter("goal_position_tolerance_m", 0.01).value
        )
        self.goal_orientation_tolerance_rad = float(
            self.declare_parameter("goal_orientation_tolerance_rad", 0.035).value
        )
        self.num_planning_attempts = int(
            self.declare_parameter("num_planning_attempts", 1).value
        )
        self.wait_for_move_group_timeout_sec = float(
            self.declare_parameter("wait_for_move_group_timeout_sec", 0.0).value
        )
        self.max_hand_target_distance_m = float(
            self.declare_parameter("max_hand_target_distance_m", 1.5).value
        )
        self.min_hand_target_z_m = float(
            self.declare_parameter("min_hand_target_z_m", 0.2).value
        )
        self.max_hand_target_z_m = float(
            self.declare_parameter("max_hand_target_z_m", 1.6).value
        )
        self.ik_timeout_sec = float(
            self.declare_parameter("ik_timeout_sec", 0.03).value
        )
        self.ik_avoid_collisions = bool(
            self.declare_parameter("ik_avoid_collisions", False).value
        )
        self.command_duration_sec = float(
            self.declare_parameter("command_duration_sec", 0.12).value
        )
        self.max_joint_delta_rad = float(
            self.declare_parameter("max_joint_delta_rad", 0.08).value
        )
        self.joint_smoothing_alpha = float(
            self.declare_parameter("joint_smoothing_alpha", 0.6).value
        )
        self.joint_command_deadband_rad = float(
            self.declare_parameter("joint_command_deadband_rad", 0.003).value
        )
        self.ik_seed_from_commanded_state = bool(
            self.declare_parameter("ik_seed_from_commanded_state", True).value
        )
        self.ik_command_resync_threshold_rad = float(
            self.declare_parameter(
                "ik_command_resync_threshold_rad",
                0.25,
            ).value
        )
        self.ik_solution_jump_threshold_rad = float(
            self.declare_parameter(
                "ik_solution_jump_threshold_rad",
                0.35,
            ).value
        )
        self.ik_slow_request_warn_sec = float(
            self.declare_parameter("ik_slow_request_warn_sec", 0.025).value
        )
        self.ik_warmup_sec = float(
            self.declare_parameter("ik_warmup_sec", 1.5).value
        )
        self.ik_warmup_min_scale = float(
            self.declare_parameter("ik_warmup_min_scale", 0.15).value
        )
        self.ik_warmup_reset_after_sec = float(
            self.declare_parameter("ik_warmup_reset_after_sec", 0.5).value
        )
        self.ik_retry_last_orientation_on_no_solution = bool(
            self.declare_parameter(
                "ik_retry_last_orientation_on_no_solution",
                True,
            ).value
        )
        self.head_publish_rate_hz = float(
            self.declare_parameter("head_publish_rate_hz", 20.0).value
        )
        self.head_command_duration_sec = float(
            self.declare_parameter("head_command_duration_sec", 0.06).value
        )
        self.head_deadband_rad = float(
            self.declare_parameter("head_deadband_rad", 0.01).value
        )
        self.head_limit_scale = float(
            self.declare_parameter("head_limit_scale", 0.9).value
        )
        self.head_pan_limits_rad = _float_pair_parameter(
            self.declare_parameter(
                "head_pan_limits_rad",
                [-1.24, 1.24],
            ).value,
            [-1.24, 1.24],
        )
        self.head_tilt_limits_rad = _float_pair_parameter(
            self.declare_parameter(
                "head_tilt_limits_rad",
                [-0.98, 0.72],
            ).value,
            [-0.98, 0.72],
        )
        self.head_pan_sign = float(self.declare_parameter("head_pan_sign", 1.0).value)
        self.head_tilt_sign = float(self.declare_parameter("head_tilt_sign", 1.0).value)
        self.gripper_joint_names = _declare_string_list_parameter(
            self,
            "gripper_joint_names",
            [
                "gripper_right_finger_joint",
                "gripper_left_finger_joint",
            ],
        )
        self.gripper_min_position_m = float(
            self.declare_parameter("gripper_min_position_m", 0.0).value
        )
        self.gripper_max_position_m = float(
            self.declare_parameter("gripper_max_position_m", 0.045).value
        )
        self.gripper_deadband_m = float(
            self.declare_parameter("gripper_deadband_m", 0.0005).value
        )
        self.gripper_command_duration_sec = float(
            self.declare_parameter(
                "gripper_command_duration_sec",
                0.15,
            ).value
        )
        self.gripper_max_velocity_mps = float(
            self.declare_parameter("gripper_max_velocity_mps", 0.04).value
        )

        self.hand_position_scale = _vector_parameter(
            self.declare_parameter(
                "hand_position_scale",
                [1.0, 1.0, 1.0],
            ).value,
            [1.0, 1.0, 1.0],
        )
        self.hand_position_offset = _vector_parameter(
            self.declare_parameter(
                "hand_position_offset",
                [0.0, 0.0, 0.0],
            ).value,
            [0.0, 0.0, 0.0],
        )
        self.arm_joint_names = _string_list_parameter(
            self.declare_parameter(
                "arm_joint_names",
                [
                    "arm_1_joint",
                    "arm_2_joint",
                    "arm_3_joint",
                    "arm_4_joint",
                    "arm_5_joint",
                    "arm_6_joint",
                    "arm_7_joint",
                ],
            ).value,
            [
                "arm_1_joint",
                "arm_2_joint",
                "arm_3_joint",
                "arm_4_joint",
                "arm_5_joint",
                "arm_6_joint",
                "arm_7_joint",
            ],
        )
        self.torso_joint_names = _string_list_parameter(
            self.declare_parameter(
                "torso_joint_names",
                ["torso_lift_joint"],
            ).value,
            ["torso_lift_joint"],
        )

        self.pending_hand_target: Optional[PoseStamped] = None
        self.latest_head_pose: Optional[PoseStamped] = None
        self.last_head_pan: Optional[float] = None
        self.last_head_tilt: Optional[float] = None
        self.last_commanded_target: Optional[PoseStamped] = None
        self.last_successful_ik_target: Optional[PoseStamped] = None
        self.deadman_controller_anchor: Optional[PoseStamped] = None
        self.deadman_robot_anchor: Optional[PoseStamped] = None
        self.current_joint_positions: Dict[str, float] = {}
        self.last_commanded_joint_positions: Dict[str, float] = {}
        self.last_gripper_target_position: Optional[float] = None
        self.last_plan_started_sec = 0.0
        self.last_hand_target_received_sec = 0.0
        self.last_ik_request_sec = 0.0
        self.ik_warmup_started_sec = 0.0
        self.current_ik_motion_scale = 1.0
        self.last_log_times: Dict[str, float] = {}
        self.goal_in_flight = False
        self.fk_request_in_flight = False
        self.hand_target_active = False
        self.hand_target_generation = 0
        self.received_head_pose = False
        self.received_hand_target = False
        self.received_joint_state = False

        self.head_trajectory_publisher = self.create_publisher(
            JointTrajectory,
            head_command_topic,
            10,
        )
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            arm_command_topic,
            10,
        )
        self.gripper_trajectory_publisher = self.create_publisher(
            JointTrajectory,
            gripper_command_topic,
            10,
        )
        self.torso_trajectory_publisher = None
        if torso_command_topic:
            self.torso_trajectory_publisher = self.create_publisher(
                JointTrajectory,
                torso_command_topic,
                10,
            )
        self.move_group_action = ActionClient(
            self,
            MoveGroup,
            self.move_group_action_name,
        )
        self.ik_client = self.create_client(GetPositionIK, self.ik_service_name)
        self.fk_client = self.create_client(GetPositionFK, self.fk_service_name)

        self.create_subscription(
            PoseStamped,
            head_input_topic,
            self._on_head_pose,
            10,
        )
        self.create_subscription(
            PoseStamped,
            hand_target_topic,
            self._on_hand_target,
            1,
        )
        self.create_subscription(
            Float64,
            gripper_input_topic,
            self._on_gripper_opening,
            10,
        )
        self.create_subscription(
            JointState,
            joint_state_topic,
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        self.create_timer(0.02, self._maybe_send_latest_target)
        head_timer_period_sec = 1.0 / max(1.0, self.head_publish_rate_hz)
        self.create_timer(head_timer_period_sec, self._maybe_publish_head_command)

        self.get_logger().info(
            f"Listening for head poses on '{head_input_topic}' and hand targets "
            f"on '{hand_target_topic}'"
        )
        self.get_logger().info(
            f"Head poses publish JointTrajectory commands to '{head_command_topic}' "
            f"at {self.head_publish_rate_hz:.1f} Hz"
        )
        self.get_logger().info(
            f"Gripper opening commands on '{gripper_input_topic}' publish to "
            f"'{gripper_command_topic}'"
        )
        self.get_logger().info(
            f"MoveIt action '{self.move_group_action_name}', group "
            f"'{self.arm_group}', end effector '{self.end_effector_link}', "
            f"mode '{self.execution_mode}'"
        )
        if self.execution_mode == "trajectory_topic":
            self.get_logger().info(
                f"Planned arm trajectories publish to '{arm_command_topic}'"
                + (
                    f" and torso trajectories publish to '{torso_command_topic}'"
                    if torso_command_topic
                    else ""
                )
            )
        elif self.execution_mode == "ik_topic":
            self.get_logger().info(
                f"IK service '{self.ik_service_name}' seeds from '{joint_state_topic}' "
                f"and publishes short trajectories to '{arm_command_topic}'"
            )

    def _on_head_pose(self, message: PoseStamped) -> None:
        if not self.received_head_pose:
            self.get_logger().info(
                "Received first head pose input; buffering for fixed-rate head control"
            )
            self.received_head_pose = True
        self.latest_head_pose = message

    def _maybe_publish_head_command(self) -> None:
        if self.latest_head_pose is None:
            return

        pan_tilt = self._head_pose_to_pan_tilt(self.latest_head_pose)
        if pan_tilt is None:
            return

        pan, tilt = pan_tilt
        pan = self._clamp_head_joint(pan, self.head_pan_limits_rad)
        tilt = self._clamp_head_joint(tilt, self.head_tilt_limits_rad)

        inside_deadband = self._inside_head_deadband(pan, tilt)
        if inside_deadband:
            return

        point = JointTrajectoryPoint()
        point.positions = [pan, tilt]
        point.time_from_start = Duration(
            seconds=max(0.02, self.head_command_duration_sec)
        ).to_msg()

        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = list(self.head_joint_names)
        trajectory.points = [point]

        self.head_trajectory_publisher.publish(trajectory)
        self.last_head_pan = pan
        self.last_head_tilt = tilt

    def _head_pose_to_pan_tilt(
        self,
        message: PoseStamped,
    ) -> Optional[tuple[float, float]]:
        quaternion = Quaternion()
        quaternion.x = message.pose.orientation.x
        quaternion.y = message.pose.orientation.y
        quaternion.z = message.pose.orientation.z
        quaternion.w = message.pose.orientation.w
        if not _normalize_quaternion(quaternion):
            self._warn_throttled(
                "invalid_head_quaternion",
                "Ignoring head pose with invalid orientation quaternion",
                2.0,
            )
            return None

        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        # Unity HMD uses left-handed Y-up coordinates. The robot head only needs
        # yaw around Unity Y for pan and pitch around Unity X for tilt; roll is
        # intentionally ignored.
        pan = math.atan2(
            2.0 * ((w * y) + (x * z)),
            1.0 - (2.0 * ((y * y) + (x * x))),
        )
        tilt_value = _clamp(2.0 * ((w * x) - (z * y)), -1.0, 1.0)
        tilt = math.asin(tilt_value)
        return self.head_pan_sign * pan, self.head_tilt_sign * tilt

    def _clamp_head_joint(self, value: float, limits: list[float]) -> float:
        limit_scale = _clamp(self.head_limit_scale, 0.0, 1.0)
        lower = min(limits[0], limits[1]) * limit_scale
        upper = max(limits[0], limits[1]) * limit_scale
        return _clamp(value, lower, upper)

    def _inside_head_deadband(self, pan: float, tilt: float) -> bool:
        if self.last_head_pan is None or self.last_head_tilt is None:
            return False

        deadband = max(0.0, self.head_deadband_rad)
        return (
            abs(pan - self.last_head_pan) < deadband
            and abs(tilt - self.last_head_tilt) < deadband
        )

    def _on_hand_target(self, message: PoseStamped) -> None:
        if not self.hand_target_active:
            controller_anchor = self._apply_hand_target_adjustments(message)
            if not _normalize_quaternion(
                controller_anchor.pose.orientation
            ):
                self._warn_throttled(
                    "invalid_deadman_anchor",
                    "Ignoring deadman press with invalid controller orientation",
                    2.0,
                )
                return

            self.hand_target_active = True
            self.deadman_controller_anchor = controller_anchor
            self.get_logger().info(
                "Deadman active; anchored controller pose for clutch control"
            )
        if not self.received_hand_target:
            self.get_logger().info(
                "Received first hand target input "
                f"frame='{message.header.frame_id}' "
                f"xyz=({message.pose.position.x:.3f}, "
                f"{message.pose.position.y:.3f}, "
                f"{message.pose.position.z:.3f})"
            )
            self.received_hand_target = True
        self.pending_hand_target = _copy_pose_stamped(message)
        self.last_hand_target_received_sec = (
            self.get_clock().now().nanoseconds / 1e9
        )

    def _reset_hand_target_pursuit(self) -> None:
        self.pending_hand_target = None
        self.last_commanded_target = None
        self.last_successful_ik_target = None
        self.deadman_controller_anchor = None
        self.deadman_robot_anchor = None
        self.last_commanded_joint_positions.clear()
        self.last_plan_started_sec = 0.0
        self.last_hand_target_received_sec = 0.0
        self.last_ik_request_sec = 0.0
        self.ik_warmup_started_sec = 0.0
        self.current_ik_motion_scale = 1.0
        self.goal_in_flight = False
        self.fk_request_in_flight = False
        self.hand_target_active = False
        self.hand_target_generation += 1
        self.get_logger().info(
            "Deadman inactive; cleared hand-target pursuit state"
        )

    def _on_joint_state(self, message: JointState) -> None:
        if not self.received_joint_state:
            self.get_logger().info("Received first joint state sample")
            self.received_joint_state = True

        for name, position in zip(message.name, message.position):
            self.current_joint_positions[name] = float(position)

    def _on_gripper_opening(self, message: Float64) -> None:
        if not math.isfinite(float(message.data)):
            self._warn_throttled(
                "invalid_gripper_opening",
                "Ignoring non-finite gripper opening",
                2.0,
            )
            return

        if len(self.gripper_joint_names) != 2:
            self._warn_throttled(
                "invalid_gripper_joints",
                "gripper_joint_names must contain exactly two finger joints",
                5.0,
            )
            return

        missing_joints = [
            joint_name
            for joint_name in self.gripper_joint_names
            if joint_name not in self.current_joint_positions
        ]
        if missing_joints:
            self._warn_throttled(
                "gripper_joint_state_wait",
                "Waiting for gripper joint states; missing "
                + ", ".join(missing_joints),
                2.0,
            )
            return

        opening = _clamp(float(message.data), 0.0, 1.0)
        lower = min(self.gripper_min_position_m, self.gripper_max_position_m)
        upper = max(self.gripper_min_position_m, self.gripper_max_position_m)
        target_position = lower + ((upper - lower) * opening)
        current_positions = [
            self.current_joint_positions[joint_name]
            for joint_name in self.gripper_joint_names
        ]
        current_average = sum(current_positions) / len(current_positions)
        deadband = max(0.0, self.gripper_deadband_m)

        if (
            self.last_gripper_target_position is None
            and abs(target_position - current_average) <= deadband
        ):
            self.last_gripper_target_position = target_position
            return

        if (
            self.last_gripper_target_position is not None
            and abs(
                target_position - self.last_gripper_target_position
            ) <= deadband
        ):
            return

        duration_sec = max(0.02, self.gripper_command_duration_sec)
        max_velocity = max(0.0, self.gripper_max_velocity_mps)
        if max_velocity > 1e-9:
            duration_sec = max(
                duration_sec,
                abs(target_position - current_average) / max_velocity,
            )

        point = JointTrajectoryPoint()
        point.positions = [
            target_position
            for _joint_name in self.gripper_joint_names
        ]
        point.time_from_start = Duration(seconds=duration_sec).to_msg()

        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = list(self.gripper_joint_names)
        trajectory.points = [point]
        self.gripper_trajectory_publisher.publish(trajectory)
        self.last_gripper_target_position = target_position

    def _maybe_send_latest_target(self) -> None:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if (
            self.hand_target_active
            and self.hand_target_timeout_sec > 0.0
            and now_sec - self.last_hand_target_received_sec
            > self.hand_target_timeout_sec
        ):
            self._reset_hand_target_pursuit()
            return

        if (
            not self.hand_target_active
            or self.goal_in_flight
            or self.pending_hand_target is None
        ):
            return

        if now_sec - self.last_plan_started_sec < self.min_plan_interval_sec:
            return

        controller_pose = self._apply_hand_target_adjustments(
            self.pending_hand_target
        )

        if not _normalize_quaternion(controller_pose.pose.orientation):
            self._warn_throttled(
                "invalid_quaternion",
                "Ignoring hand target with invalid orientation quaternion",
                2.0,
            )
            self.pending_hand_target = None
            return

        if self.deadman_robot_anchor is None:
            self._request_current_end_effector_pose()
            return

        target = self._map_controller_delta_to_robot(controller_pose)
        if self.pose_reference_frame:
            target.header.frame_id = self.pose_reference_frame

        if not self._constrain_hand_target_to_workspace(target):
            self.pending_hand_target = None
            return

        if self._inside_deadband(target):
            self.pending_hand_target = None
            return

        if self.execution_mode == "ik_topic":
            self._send_ik_target(target, now_sec)
            return

        if not self.move_group_action.wait_for_server(
            timeout_sec=self.wait_for_move_group_timeout_sec
        ):
            self._warn_throttled(
                "move_group_wait",
                f"Waiting for MoveIt action server '{self.move_group_action_name}'",
                5.0,
            )
            return

        goal = self._build_move_group_goal(target)
        self.goal_in_flight = True
        self.last_plan_started_sec = now_sec
        self.pending_hand_target = None
        self.get_logger().info(
            f"Sending hand target to MoveIt frame='{target.header.frame_id}' "
            f"xyz=({target.pose.position.x:.3f}, "
            f"{target.pose.position.y:.3f}, "
            f"{target.pose.position.z:.3f})"
        )
        send_future = self.move_group_action.send_goal_async(goal)
        send_future.add_done_callback(
            lambda future: self._on_goal_response(future, target)
        )

    def _send_ik_target(self, target: PoseStamped, now_sec: float) -> None:
        active_joint_names = self._active_output_joint_names()
        missing_joints = [
            joint_name
            for joint_name in active_joint_names
            if joint_name not in self.current_joint_positions
        ]
        if missing_joints:
            self._warn_throttled(
                "joint_state_wait",
                "Waiting for joint states before IK servo command; missing "
                + ", ".join(missing_joints),
                2.0,
            )
            return

        if not self.ik_client.wait_for_service(
            timeout_sec=self.wait_for_move_group_timeout_sec
        ):
            self._warn_throttled(
                "ik_service_wait",
                f"Waiting for MoveIt IK service '{self.ik_service_name}'",
                5.0,
            )
            return

        self.current_ik_motion_scale = self._update_ik_warmup(now_sec)

        stepped_target = self._step_toward_hand_target(target)
        request = self._build_ik_request(stepped_target)
        self.goal_in_flight = True
        self.last_plan_started_sec = now_sec
        generation = self.hand_target_generation
        future = self.ik_client.call_async(request)
        future.add_done_callback(
            lambda result: self._on_ik_result(
                result,
                stepped_target,
                retry_count=0,
                generation=generation,
                request_started_sec=now_sec,
            )
        )

    def _request_current_end_effector_pose(self) -> None:
        if self.fk_request_in_flight:
            return
        if not self.fk_client.wait_for_service(
            timeout_sec=self.wait_for_move_group_timeout_sec
        ):
            self._warn_throttled(
                "fk_service_wait",
                f"Waiting for MoveIt FK service '{self.fk_service_name}'",
                5.0,
            )
            return

        request = GetPositionFK.Request()
        request.header.frame_id = self.pose_reference_frame
        request.fk_link_names = [self.end_effector_link]
        request.robot_state = self._build_current_robot_state()
        self.fk_request_in_flight = True
        self.goal_in_flight = True
        generation = self.hand_target_generation
        future = self.fk_client.call_async(request)
        future.add_done_callback(
            lambda result: self._on_current_end_effector_pose(
                result,
                generation,
            )
        )

    def _on_current_end_effector_pose(
        self,
        future: Future,
        generation: int,
    ) -> None:
        if (
            generation != self.hand_target_generation
            or not self.hand_target_active
        ):
            return

        self.fk_request_in_flight = False
        self.goal_in_flight = False
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().warn(f"MoveIt FK request failed: {error}")
            return

        if (
            response.error_code.val != MoveItErrorCodes.SUCCESS
            or not response.pose_stamped
        ):
            self._warn_throttled(
                "fk_error",
                "MoveIt could not provide the current wrist pose for IK startup",
                2.0,
            )
            return

        current_pose = _copy_pose_stamped(response.pose_stamped[0])
        if not _normalize_quaternion(current_pose.pose.orientation):
            self._warn_throttled(
                "fk_quaternion",
                "MoveIt FK returned an invalid wrist orientation",
                2.0,
            )
            return

        self.last_successful_ik_target = current_pose
        self.deadman_robot_anchor = _copy_pose_stamped(current_pose)
        self.last_commanded_target = _copy_pose_stamped(current_pose)
        self.get_logger().info(
            "Deadman clutch anchored to the current robot wrist pose"
        )

    def _map_controller_delta_to_robot(
        self,
        controller_pose: PoseStamped,
    ) -> PoseStamped:
        controller_anchor = self.deadman_controller_anchor
        robot_anchor = self.deadman_robot_anchor
        if controller_anchor is None or robot_anchor is None:
            return _copy_pose_stamped(controller_pose)

        target = _copy_pose_stamped(robot_anchor)
        target.header.stamp = controller_pose.header.stamp
        target.pose.position.x += (
            controller_pose.pose.position.x
            - controller_anchor.pose.position.x
        )
        target.pose.position.y += (
            controller_pose.pose.position.y
            - controller_anchor.pose.position.y
        )
        target.pose.position.z += (
            controller_pose.pose.position.z
            - controller_anchor.pose.position.z
        )

        controller_delta = _multiply_quaternions(
            controller_pose.pose.orientation,
            _inverse_quaternion(controller_anchor.pose.orientation),
        )
        target.pose.orientation = _multiply_quaternions(
            controller_delta,
            robot_anchor.pose.orientation,
        )
        return target

    def _step_toward_hand_target(self, target: PoseStamped) -> PoseStamped:
        start = self.last_successful_ik_target
        if start is None:
            return _copy_pose_stamped(target)

        stepped = _copy_pose_stamped(target)
        distance_m = _position_distance(start, target)
        max_position_step = max(0.0, self.cartesian_position_step_m)
        if max_position_step > 0.0 and distance_m > max_position_step:
            fraction = max_position_step / distance_m
            stepped.pose.position.x = start.pose.position.x + fraction * (
                target.pose.position.x - start.pose.position.x
            )
            stepped.pose.position.y = start.pose.position.y + fraction * (
                target.pose.position.y - start.pose.position.y
            )
            stepped.pose.position.z = start.pose.position.z + fraction * (
                target.pose.position.z - start.pose.position.z
            )

        orientation_distance = _orientation_distance(start, target)
        max_orientation_step = max(0.0, self.cartesian_orientation_step_rad)
        if (
            max_orientation_step > 0.0
            and orientation_distance > max_orientation_step
        ):
            stepped.pose.orientation = _interpolate_quaternion(
                start.pose.orientation,
                target.pose.orientation,
                max_orientation_step / orientation_distance,
            )

        return stepped

    def _build_ik_request(self, target: PoseStamped) -> GetPositionIK.Request:
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.arm_group
        request.ik_request.robot_state = self._build_current_robot_state(
            prefer_commanded=True
        )
        request.ik_request.avoid_collisions = self.ik_avoid_collisions
        request.ik_request.ik_link_name = self.end_effector_link
        request.ik_request.pose_stamped = target
        request.ik_request.timeout = Duration(seconds=self.ik_timeout_sec).to_msg()
        return request

    def _update_ik_warmup(self, now_sec: float) -> float:
        if self.ik_warmup_sec <= 0.0:
            self.last_ik_request_sec = now_sec
            return 1.0

        if (
            self.last_ik_request_sec <= 0.0
            or now_sec - self.last_ik_request_sec > self.ik_warmup_reset_after_sec
        ):
            self.ik_warmup_started_sec = now_sec

        self.last_ik_request_sec = now_sec
        elapsed_sec = max(0.0, now_sec - self.ik_warmup_started_sec)
        progress = max(0.0, min(1.0, elapsed_sec / self.ik_warmup_sec))
        min_scale = max(0.0, min(1.0, self.ik_warmup_min_scale))
        scale = min_scale + ((1.0 - min_scale) * progress)

        if progress < 1.0:
            self._info_throttled(
                "ik_warmup",
                f"IK motion warmup scale={scale:.2f}",
                0.5,
            )

        return scale

    def _build_current_robot_state(
        self,
        prefer_commanded: bool = False,
    ) -> RobotState:
        joint_positions = self._joint_reference_positions(prefer_commanded)
        robot_state = RobotState()
        robot_state.joint_state.name = [
            name
            for name in self._active_output_joint_names()
            if name in joint_positions
        ]
        robot_state.joint_state.position = [
            joint_positions[name]
            for name in robot_state.joint_state.name
        ]
        robot_state.is_diff = True
        return robot_state

    def _joint_reference_positions(
        self,
        prefer_commanded: bool,
    ) -> Dict[str, float]:
        positions = dict(self.current_joint_positions)
        if (
            not prefer_commanded
            or not self.ik_seed_from_commanded_state
            or not self.last_commanded_joint_positions
        ):
            return positions

        resync_threshold = max(0.0, self.ik_command_resync_threshold_rad)
        if resync_threshold > 0.0:
            for joint_name in self.arm_joint_names:
                if (
                    joint_name not in self.current_joint_positions
                    or joint_name not in self.last_commanded_joint_positions
                ):
                    continue
                tracking_error = abs(
                    self.last_commanded_joint_positions[joint_name]
                    - self.current_joint_positions[joint_name]
                )
                if tracking_error > resync_threshold:
                    self._warn_throttled(
                        "ik_command_resync",
                        "IK command state is ahead of measured joints by "
                        f"{tracking_error:.3f} rad on '{joint_name}'; "
                        "temporarily seeding from measured state",
                        1.0,
                    )
                    return positions

        for joint_name in self._active_output_joint_names():
            if joint_name in self.last_commanded_joint_positions:
                positions[joint_name] = self.last_commanded_joint_positions[
                    joint_name
                ]
        return positions

    def _active_output_joint_names(self) -> list[str]:
        if "torso" in str(self.arm_group):
            return [*self.torso_joint_names, *self.arm_joint_names]
        return list(self.arm_joint_names)

    def _apply_hand_target_adjustments(self, message: PoseStamped) -> PoseStamped:
        target = _copy_pose_stamped(message)
        target.pose.position.x = (
            target.pose.position.x * self.hand_position_scale[0]
        ) + self.hand_position_offset[0]
        target.pose.position.y = (
            target.pose.position.y * self.hand_position_scale[1]
        ) + self.hand_position_offset[1]
        target.pose.position.z = (
            target.pose.position.z * self.hand_position_scale[2]
        ) + self.hand_position_offset[2]
        return target

    def _inside_deadband(self, target: PoseStamped) -> bool:
        if self.last_commanded_target is None:
            return False

        return (
            _position_distance(target, self.last_commanded_target)
            < self.position_deadband_m
            and _orientation_distance(target, self.last_commanded_target)
            < self.orientation_deadband_rad
        )

    def _constrain_hand_target_to_workspace(
        self,
        target: PoseStamped,
    ) -> bool:
        position = target.pose.position
        if not all(
            math.isfinite(value)
            for value in (position.x, position.y, position.z)
        ):
            self._warn_throttled(
                "target_non_finite",
                "Ignoring hand target with non-finite position",
                2.0,
            )
            return False

        if self.min_hand_target_z_m < self.max_hand_target_z_m:
            constrained_z = _clamp(
                position.z,
                self.min_hand_target_z_m,
                self.max_hand_target_z_m,
            )
            if constrained_z != position.z:
                self._warn_throttled(
                    "target_z_limit",
                    f"Clamping hand target z={position.z:.3f} to "
                    f"{constrained_z:.3f}m",
                    2.0,
                )
                position.z = constrained_z

        distance_m = _position_norm(target)
        if (
            self.max_hand_target_distance_m > 0.0
            and distance_m > self.max_hand_target_distance_m
        ):
            max_distance = self.max_hand_target_distance_m
            position.z = _clamp(position.z, -max_distance, max_distance)
            horizontal_distance = math.hypot(position.x, position.y)
            max_horizontal_distance = math.sqrt(
                max(
                    0.0,
                    (max_distance * max_distance)
                    - (position.z * position.z),
                )
            )
            if horizontal_distance > 1e-9:
                horizontal_scale = (
                    max_horizontal_distance / horizontal_distance
                )
                position.x *= horizontal_scale
                position.y *= horizontal_scale
            self._warn_throttled(
                "target_distance_limit",
                f"Clamping hand target {distance_m:.3f}m from base to "
                f"{self.max_hand_target_distance_m:.3f}m",
                2.0,
            )

        return True

    def _build_move_group_goal(self, target: PoseStamped) -> MoveGroup.Goal:
        goal = MoveGroup.Goal()
        goal.request = self._build_motion_plan_request(target)
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = self.execution_mode in (
            "plan_only",
            "trajectory_topic",
        )
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        return goal

    def _build_motion_plan_request(self, target: PoseStamped) -> MotionPlanRequest:
        request = MotionPlanRequest()
        request.group_name = self.arm_group
        request.num_planning_attempts = self.num_planning_attempts
        request.allowed_planning_time = self.planning_time_sec
        request.max_velocity_scaling_factor = self.max_velocity_scaling_factor
        request.max_acceleration_scaling_factor = (
            self.max_acceleration_scaling_factor
        )
        request.start_state = RobotState()
        request.start_state.is_diff = True
        request.goal_constraints = [self._build_goal_constraints(target)]
        return request

    def _build_goal_constraints(self, target: PoseStamped) -> Constraints:
        constraints = Constraints()
        constraints.name = "vive_hand_target"

        position_constraint = PositionConstraint()
        position_constraint.header = target.header
        position_constraint.link_name = self.end_effector_link
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self.goal_position_tolerance_m]
        sphere_pose = Pose()
        sphere_pose.position = target.pose.position
        sphere_pose.orientation.w = 1.0
        region = BoundingVolume()
        region.primitives = [sphere]
        region.primitive_poses = [sphere_pose]
        position_constraint.constraint_region = region
        position_constraint.weight = 1.0

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = target.header
        orientation_constraint.link_name = self.end_effector_link
        orientation_constraint.orientation = target.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = (
            self.goal_orientation_tolerance_rad
        )
        orientation_constraint.absolute_y_axis_tolerance = (
            self.goal_orientation_tolerance_rad
        )
        orientation_constraint.absolute_z_axis_tolerance = (
            self.goal_orientation_tolerance_rad
        )
        orientation_constraint.weight = 1.0

        constraints.position_constraints = [position_constraint]
        constraints.orientation_constraints = [orientation_constraint]
        return constraints

    def _on_ik_result(
        self,
        future: Future,
        target: PoseStamped,
        retry_count: int,
        generation: int,
        request_started_sec: float,
    ) -> None:
        if (
            generation != self.hand_target_generation
            or not self.hand_target_active
        ):
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        request_duration_sec = max(0.0, now_sec - request_started_sec)
        if (
            self.ik_slow_request_warn_sec > 0.0
            and request_duration_sec > self.ik_slow_request_warn_sec
        ):
            self._warn_throttled(
                "slow_ik_request",
                f"MoveIt IK round trip took {request_duration_sec * 1000.0:.1f} ms",
                1.0,
            )

        try:
            response = future.result()
        except Exception as error:
            self.get_logger().warn(f"MoveIt IK request failed: {error}")
            self.goal_in_flight = False
            return

        error_code = response.error_code.val
        if error_code != MoveItErrorCodes.SUCCESS:
            if self._retry_ik_with_last_orientation(
                error_code,
                target,
                retry_count,
                generation,
            ):
                return

            self._warn_throttled(
                "ik_error",
                "MoveIt IK could not satisfy hand target, "
                f"error_code={error_code} ({self._moveit_error_name(error_code)}), "
                f"frame='{target.header.frame_id}' "
                f"xyz=({target.pose.position.x:.3f}, "
                f"{target.pose.position.y:.3f}, "
                f"{target.pose.position.z:.3f})",
                1.0,
            )
            self.goal_in_flight = False
            return

        solution_positions = {
            name: float(position)
            for name, position in zip(
                response.solution.joint_state.name,
                response.solution.joint_state.position,
            )
        }
        if not self._ik_solution_is_continuous(solution_positions):
            self.goal_in_flight = False
            return

        self.last_successful_ik_target = _copy_pose_stamped(target)
        if self._ik_solution_reached(solution_positions):
            self.last_commanded_target = _copy_pose_stamped(target)
            self.goal_in_flight = False
            return

        if self._publish_ik_joint_command(solution_positions):
            self._info_throttled(
                "ik_topic_success",
                f"Published IK servo command for '{self.arm_group}'",
                2.0,
            )

        self.goal_in_flight = False

    def _ik_solution_reached(self, solution_positions: Dict[str, float]) -> bool:
        deadband = max(0.0, self.joint_command_deadband_rad)
        reference_positions = self._joint_reference_positions(
            prefer_commanded=True
        )
        compared_joint_count = 0
        for joint_name in self._active_output_joint_names():
            if (
                joint_name not in solution_positions
                or joint_name not in reference_positions
            ):
                continue
            compared_joint_count += 1
            if (
                abs(
                    solution_positions[joint_name]
                    - reference_positions[joint_name]
                )
                > deadband
            ):
                return False

        return compared_joint_count > 0

    def _ik_solution_is_continuous(
        self,
        solution_positions: Dict[str, float],
    ) -> bool:
        jump_threshold = max(0.0, self.ik_solution_jump_threshold_rad)
        if jump_threshold <= 0.0:
            return True

        reference_positions = self._joint_reference_positions(
            prefer_commanded=True
        )
        largest_jump = 0.0
        largest_jump_joint = ""
        for joint_name in self.arm_joint_names:
            if (
                joint_name not in solution_positions
                or joint_name not in reference_positions
            ):
                continue
            jump = abs(
                solution_positions[joint_name]
                - reference_positions[joint_name]
            )
            if jump > largest_jump:
                largest_jump = jump
                largest_jump_joint = joint_name

        if largest_jump <= jump_threshold:
            return True

        self._warn_throttled(
            "ik_solution_jump",
            "Rejected discontinuous IK solution: "
            f"'{largest_jump_joint}' changed by {largest_jump:.3f} rad "
            f"(limit {jump_threshold:.3f} rad)",
            0.5,
        )
        return False

    def _retry_ik_with_last_orientation(
        self,
        error_code: int,
        target: PoseStamped,
        retry_count: int,
        generation: int,
    ) -> bool:
        if error_code != MoveItErrorCodes.NO_IK_SOLUTION:
            return False
        if retry_count > 0:
            return False
        if not self.ik_retry_last_orientation_on_no_solution:
            return False
        if self.last_successful_ik_target is None:
            return False

        retry_target = _copy_pose_stamped(target)
        retry_target.pose.orientation = self.last_successful_ik_target.pose.orientation
        request = self._build_ik_request(retry_target)
        request_started_sec = self.get_clock().now().nanoseconds / 1e9
        future = self.ik_client.call_async(request)
        future.add_done_callback(
            lambda result: self._on_ik_result(
                result,
                retry_target,
                retry_count=retry_count + 1,
                generation=generation,
                request_started_sec=request_started_sec,
            )
        )
        self._info_throttled(
            "ik_retry_last_orientation",
            "Retrying IK target position with the last reachable wrist orientation",
            1.0,
        )
        return True

    @staticmethod
    def _moveit_error_name(error_code: int) -> str:
        if error_code == MoveItErrorCodes.NO_IK_SOLUTION:
            return "NO_IK_SOLUTION"
        if error_code == MoveItErrorCodes.TIMED_OUT:
            return "TIMED_OUT"
        if error_code == MoveItErrorCodes.FRAME_TRANSFORM_FAILURE:
            return "FRAME_TRANSFORM_FAILURE"
        if error_code == MoveItErrorCodes.INVALID_ROBOT_STATE:
            return "INVALID_ROBOT_STATE"
        if error_code == MoveItErrorCodes.GOAL_IN_COLLISION:
            return "GOAL_IN_COLLISION"
        if error_code == MoveItErrorCodes.PLANNING_FAILED:
            return "PLANNING_FAILED"
        return "UNKNOWN"

    def _publish_ik_joint_command(
        self,
        solution_positions: Dict[str, float],
    ) -> bool:
        published = False
        if self._publish_joint_position_command(
            solution_positions,
            self.arm_joint_names,
            self.trajectory_publisher,
        ):
            published = True

        if "torso" in str(self.arm_group) and self.torso_trajectory_publisher:
            if self._publish_joint_position_command(
                solution_positions,
                self.torso_joint_names,
                self.torso_trajectory_publisher,
            ):
                published = True

        return published

    def _publish_joint_position_command(
        self,
        solution_positions: Dict[str, float],
        joint_names: list[str],
        publisher,
    ) -> bool:
        selected_joint_names = [
            joint_name for joint_name in joint_names if joint_name in solution_positions
        ]
        if not selected_joint_names:
            return False

        reference_positions = self._joint_reference_positions(
            prefer_commanded=True
        )
        start_positions: Dict[str, float] = {}
        missing_start_positions = []
        for joint_name in selected_joint_names:
            if joint_name in reference_positions:
                start_positions[joint_name] = reference_positions[joint_name]
            else:
                missing_start_positions.append(joint_name)

        if missing_start_positions:
            self._warn_throttled(
                "ik_missing_joint_state",
                "Cannot publish IK command without current joint positions for "
                + ", ".join(missing_start_positions),
                2.0,
            )
            return False

        motion_scale = max(0.0, min(1.0, self.current_ik_motion_scale))
        alpha = max(0.0, min(1.0, self.joint_smoothing_alpha)) * motion_scale
        max_joint_delta_rad = self.max_joint_delta_rad * motion_scale
        point = JointTrajectoryPoint()
        for joint_name in selected_joint_names:
            start = start_positions[joint_name]
            desired = solution_positions[joint_name]
            delta = (desired - start) * alpha
            if max_joint_delta_rad > 0.0:
                delta = max(
                    -max_joint_delta_rad,
                    min(max_joint_delta_rad, delta),
                )
            command = start + delta
            point.positions.append(command)
            self.last_commanded_joint_positions[joint_name] = command

        point.time_from_start = Duration(
            seconds=max(0.02, self.command_duration_sec)
        ).to_msg()

        trajectory = JointTrajectory()
        # A zero start stamp tells joint_trajectory_controller to start on
        # receipt. Host/robot clock skew must not consume a short teleop command.
        trajectory.joint_names = selected_joint_names
        trajectory.points = [point]
        publisher.publish(trajectory)
        return True

    def _on_goal_response(
        self,
        future: Future,
        target: PoseStamped,
    ) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:
            self.get_logger().warn(f"MoveIt goal request failed: {error}")
            self.goal_in_flight = False
            return

        if not goal_handle.accepted:
            self.get_logger().warn("MoveIt rejected the hand target goal")
            self.goal_in_flight = False
            return

        self.get_logger().info("MoveIt accepted hand target goal")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result: self._on_goal_result(result, target)
        )

    def _on_goal_result(self, future: Future, target: PoseStamped) -> None:
        try:
            action_result = future.result().result
        except Exception as error:
            self.get_logger().warn(f"MoveIt action failed: {error}")
            self.goal_in_flight = False
            return

        error_code = action_result.error_code.val
        if error_code != MoveItErrorCodes.SUCCESS:
            self._warn_throttled(
                "moveit_error",
                f"MoveIt could not satisfy hand target, error_code={error_code}",
                2.0,
            )
            self.goal_in_flight = False
            return

        if self.execution_mode == "trajectory_topic":
            self._publish_planned_trajectory(
                action_result.planned_trajectory.joint_trajectory
            )
        elif self.execution_mode == "plan_only":
            self._info_throttled(
                "plan_only_success",
                "MoveIt plan succeeded; execution_mode=plan_only",
                2.0,
            )

        self.last_commanded_target = target
        self.goal_in_flight = False

    def _publish_planned_trajectory(self, trajectory: JointTrajectory) -> None:
        if not trajectory.joint_names:
            self._warn_throttled(
                "empty_trajectory",
                "MoveIt returned an empty planned joint trajectory",
                2.0,
            )
            return

        published_topics = []
        arm_trajectory = self._filter_joint_trajectory(
            trajectory,
            self.arm_joint_names,
        )
        if arm_trajectory is not None:
            self.trajectory_publisher.publish(arm_trajectory)
            published_topics.append("arm")

        if self.torso_trajectory_publisher is not None:
            torso_trajectory = self._filter_joint_trajectory(
                trajectory,
                self.torso_joint_names,
            )
            if torso_trajectory is not None:
                self.torso_trajectory_publisher.publish(torso_trajectory)
                published_topics.append("torso")

        if published_topics:
            self._info_throttled(
                "trajectory_topic_success",
                f"Published planned trajectory segments: {', '.join(published_topics)} "
                f"joints={list(trajectory.joint_names)} points={len(trajectory.points)}",
                2.0,
            )
            return

        self._warn_throttled(
            "trajectory_no_matching_joints",
            "MoveIt planned joints "
            f"{list(trajectory.joint_names)} but none match configured arm/torso joint names",
            2.0,
        )

    def _filter_joint_trajectory(
        self,
        trajectory: JointTrajectory,
        selected_joint_names: list[str],
    ) -> Optional[JointTrajectory]:
        selected_indices = [
            index
            for index, joint_name in enumerate(trajectory.joint_names)
            if joint_name in selected_joint_names
        ]
        if not selected_indices:
            return None

        filtered = JointTrajectory()
        filtered.header = trajectory.header
        filtered.joint_names = [
            trajectory.joint_names[index] for index in selected_indices
        ]
        source_joint_count = len(trajectory.joint_names)

        for point in trajectory.points:
            filtered_point = JointTrajectoryPoint()
            filtered_point.positions = _select_joint_values(
                point.positions,
                selected_indices,
                source_joint_count,
            )
            filtered_point.velocities = _select_joint_values(
                point.velocities,
                selected_indices,
                source_joint_count,
            )
            filtered_point.accelerations = _select_joint_values(
                point.accelerations,
                selected_indices,
                source_joint_count,
            )
            filtered_point.effort = _select_joint_values(
                point.effort,
                selected_indices,
                source_joint_count,
            )
            filtered_point.time_from_start = point.time_from_start
            filtered.points.append(filtered_point)

        return filtered

    def _warn_throttled(self, key: str, message: str, period_sec: float) -> None:
        if self._should_log(key, period_sec):
            self.get_logger().warn(message)

    def _info_throttled(self, key: str, message: str, period_sec: float) -> None:
        if self._should_log(key, period_sec):
            self.get_logger().info(message)

    def _should_log(self, key: str, period_sec: float) -> bool:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        last_sec = self.last_log_times.get(key)
        if last_sec is not None and now_sec - last_sec < period_sec:
            return False

        self.last_log_times[key] = now_sec
        return True


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = ViveMoveItServer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
