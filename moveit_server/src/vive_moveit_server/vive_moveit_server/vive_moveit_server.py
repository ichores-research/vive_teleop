import math
from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .arm_movement import ArmMovementMixin
from .teleop_data import TeleopDataReceiver


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


class ViveMoveItServer(ArmMovementMixin, Node):
    """Initialize and compose teleoperation data and movement components."""

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
        self.execution_mode = self.declare_parameter(
            "execution_mode",
            "moveit",
        ).value
        self.async_execution = bool(
            self.declare_parameter("async_execution", True).value
        )
        self.pose_reference_frame = self.declare_parameter(
            "pose_reference_frame",
            "base_footprint",
        ).value

        float_parameters = {
            "planning_time_sec": 0.35,
            "max_velocity_scaling_factor": 0.25,
            "max_acceleration_scaling_factor": 0.25,
            "min_plan_interval_sec": 0.25,
            "hand_target_timeout_sec": 0.12,
            "cartesian_position_step_m": 0.02,
            "cartesian_orientation_step_rad": 0.12,
            "position_deadband_m": 0.01,
            "orientation_deadband_rad": 0.035,
            "goal_position_tolerance_m": 0.01,
            "goal_orientation_tolerance_rad": 0.035,
            "wait_for_move_group_timeout_sec": 0.0,
            "max_hand_target_distance_m": 1.5,
            "min_hand_target_z_m": 0.2,
            "max_hand_target_z_m": 1.6,
            "ik_timeout_sec": 0.03,
            "command_duration_sec": 0.12,
            "max_joint_delta_rad": 0.08,
            "joint_smoothing_alpha": 0.6,
            "joint_command_deadband_rad": 0.003,
            "ik_warmup_sec": 1.5,
            "ik_warmup_min_scale": 0.15,
            "ik_warmup_reset_after_sec": 0.5,
            "head_publish_rate_hz": 20.0,
            "head_command_duration_sec": 0.06,
            "head_deadband_rad": 0.01,
            "head_limit_scale": 0.9,
            "head_pan_sign": 1.0,
            "head_tilt_sign": 1.0,
            "gripper_min_position_m": 0.0,
            "gripper_max_position_m": 0.045,
            "gripper_deadband_m": 0.0005,
            "gripper_command_duration_sec": 0.15,
            "gripper_max_velocity_mps": 0.04,
        }
        for name, default in float_parameters.items():
            setattr(
                self,
                name,
                float(self.declare_parameter(name, default).value),
            )

        self.num_planning_attempts = int(
            self.declare_parameter("num_planning_attempts", 1).value
        )
        self.ik_avoid_collisions = bool(
            self.declare_parameter("ik_avoid_collisions", False).value
        )
        self.ik_retry_last_orientation_on_no_solution = bool(
            self.declare_parameter(
                "ik_retry_last_orientation_on_no_solution",
                True,
            ).value
        )

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

        self.gripper_joint_names = _declare_string_list_parameter(
            self,
            "gripper_joint_names",
            [
                "gripper_right_finger_joint",
                "gripper_left_finger_joint",
            ],
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
        self.arm_joint_names = _declare_string_list_parameter(
            self,
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
        )
        self.torso_joint_names = _declare_string_list_parameter(
            self,
            "torso_joint_names",
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

        self.data_receiver = TeleopDataReceiver(
            self,
            head_input_topic=head_input_topic,
            hand_target_topic=hand_target_topic,
            gripper_input_topic=gripper_input_topic,
            joint_state_topic=joint_state_topic,
            on_head_pose=self._on_head_pose,
            on_hand_target=self._on_hand_target,
            on_gripper_opening=self._on_gripper_opening,
            on_joint_state=self._on_joint_state,
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
        if self._inside_head_deadband(pan, tilt):
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

        # Unity is left-handed and Y-up. The robot uses HMD yaw for pan and
        # pitch for tilt; roll is intentionally ignored.
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
            and abs(target_position - self.last_gripper_target_position)
            <= deadband
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
            target_position for _joint_name in self.gripper_joint_names
        ]
        point.time_from_start = Duration(seconds=duration_sec).to_msg()

        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = list(self.gripper_joint_names)
        trajectory.points = [point]
        self.gripper_trajectory_publisher.publish(trajectory)
        self.last_gripper_target_position = target_position

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
