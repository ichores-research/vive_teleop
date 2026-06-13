import math
from typing import Dict, List, Optional

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .pose_utils import (
    clamp as _clamp,
    copy_pose_stamped as _copy_pose_stamped,
    inverse_quaternion as _inverse_quaternion,
    multiply_quaternions as _multiply_quaternions,
    normalize_quaternion as _normalize_quaternion,
    position_norm as _position_norm,
)
from .servo_controller import ServoController


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


class ViveMoveItServer(Node):
    def __init__(self) -> None:
        super().__init__("vive_moveit_server")

        self.end_effector_link = self.declare_parameter(
            "end_effector_link",
            "arm_tool_link",
        ).value
        self.pose_reference_frame = self.declare_parameter(
            "pose_reference_frame",
            "base_footprint",
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
        hand_deadman_topic = self.declare_parameter(
            "hand_deadman_topic",
            "/vive/hand_deadman",
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

        self.hand_target_timeout_sec = float(
            self.declare_parameter("hand_target_timeout_sec", 0.25).value
        )
        self.position_deadband_m = float(
            self.declare_parameter("position_deadband_m", 0.002).value
        )
        self.orientation_deadband_rad = float(
            self.declare_parameter("orientation_deadband_rad", 0.01).value
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
        self.head_publish_rate_hz = float(
            self.declare_parameter("head_publish_rate_hz", 20.0).value
        )
        self.head_command_duration_sec = float(
            self.declare_parameter("head_command_duration_sec", 0.1).value
        )
        self.head_deadband_rad = float(
            self.declare_parameter("head_deadband_rad", 0.002).value
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
        self.head_pan_sign = float(
            self.declare_parameter("head_pan_sign", 1.0).value
        )
        self.head_tilt_sign = float(
            self.declare_parameter("head_tilt_sign", 1.0).value
        )

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

        self.latest_head_pose: Optional[PoseStamped] = None
        self.last_head_pan: Optional[float] = None
        self.last_head_tilt: Optional[float] = None
        self.pending_hand_target: Optional[PoseStamped] = None
        self.hand_input_anchor: Optional[PoseStamped] = None
        self.robot_wrist_anchor: Optional[PoseStamped] = None
        self.hand_deadman_enabled = False
        self.hand_target_active = False
        self.last_hand_target_received_sec = 0.0
        self.current_joint_positions: Dict[str, float] = {}
        self.last_gripper_target_position: Optional[float] = None
        self.last_log_times: Dict[str, float] = {}
        self.received_head_pose = False
        self.received_hand_target = False
        self.received_joint_state = False

        self.head_trajectory_publisher = self.create_publisher(
            JointTrajectory,
            head_command_topic,
            10,
        )
        self.gripper_trajectory_publisher = self.create_publisher(
            JointTrajectory,
            gripper_command_topic,
            10,
        )
        self.servo = ServoController(
            node=self,
            end_effector_link=self.end_effector_link,
            pose_reference_frame=self.pose_reference_frame,
            position_deadband_m=self.position_deadband_m,
            orientation_deadband_rad=self.orientation_deadband_rad,
        )

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
            Bool,
            hand_deadman_topic,
            self._on_hand_deadman,
            10,
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

        self.create_timer(
            1.0 / max(1.0, self.servo.publish_rate_hz),
            self._maybe_publish_servo_target,
        )
        self.create_timer(
            1.0 / max(1.0, self.head_publish_rate_hz),
            self._maybe_publish_head_command,
        )

        self.get_logger().info(
            f"Listening for head poses on '{head_input_topic}' and Servo "
            f"wrist targets on '{hand_target_topic}'"
        )
        self.get_logger().info(
            f"Deadman release on '{hand_deadman_topic}' stops Servo immediately"
        )
        self.get_logger().info(
            f"MoveIt Servo twists publish to "
            f"'{self.servo.cartesian_command_topic}' at "
            f"{self.servo.publish_rate_hz:.1f} Hz"
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
        pan = math.atan2(
            2.0 * ((w * y) + (x * z)),
            1.0 - (2.0 * ((y * y) + (x * x))),
        )
        tilt = math.asin(
            _clamp(2.0 * ((w * x) - (z * y)), -1.0, 1.0)
        )
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
        target = self._apply_hand_target_adjustments(message)
        if not _normalize_quaternion(target.pose.orientation):
            self._warn_throttled(
                "invalid_hand_quaternion",
                "Ignoring hand target with invalid orientation quaternion",
                2.0,
            )
            return

        if not self._constrain_hand_target_to_workspace(target):
            return

        self.pending_hand_target = target
        self.last_hand_target_received_sec = self._now_sec()

        # Buffer a pose that arrives before the separate deadman topic. Once
        # armed, the next held pose starts tracking. This also prevents a delayed
        # pose from reactivating Servo after a deadman=false release.
        if not self.hand_deadman_enabled:
            return

        if not self.hand_target_active:
            current_pose = self.servo.lookup_current_pose()
            if current_pose is None:
                return
            self.hand_input_anchor = _copy_pose_stamped(target)
            self.robot_wrist_anchor = _copy_pose_stamped(current_pose)
            self.hand_target_active = True
            self.servo.start_hold()
            self.get_logger().info(
                "Deadman active; anchored Unity robot target to the current wrist"
            )

        if not self.received_hand_target:
            self.get_logger().info(
                "Received first Servo hand target "
                f"frame='{message.header.frame_id}' "
                f"xyz=({message.pose.position.x:.3f}, "
                f"{message.pose.position.y:.3f}, "
                f"{message.pose.position.z:.3f})"
            )
            self.received_hand_target = True

    def _on_hand_deadman(self, message: Bool) -> None:
        if message.data:
            self.hand_deadman_enabled = True
            return
        self.hand_deadman_enabled = False
        self._reset_hand_target_pursuit()

    def _reset_hand_target_pursuit(self) -> None:
        if self.hand_target_active:
            self.servo.stop_hold()
        self.pending_hand_target = None
        self.hand_input_anchor = None
        self.robot_wrist_anchor = None
        self.last_hand_target_received_sec = 0.0
        self.hand_deadman_enabled = False
        self.hand_target_active = False

    def _maybe_publish_servo_target(self) -> None:
        self.servo.ensure_started()
        if (
            not self.hand_target_active
            or self.pending_hand_target is None
            or self.hand_input_anchor is None
            or self.robot_wrist_anchor is None
        ):
            return

        now_sec = self._now_sec()
        if (
            self.hand_target_timeout_sec > 0.0
            and now_sec - self.last_hand_target_received_sec
            > self.hand_target_timeout_sec
        ):
            self._warn_throttled(
                "hand_target_timeout",
                "Servo hand target timed out; stopping wrist motion",
                1.0,
            )
            self._reset_hand_target_pursuit()
            return

        current_pose = self.servo.lookup_current_pose()
        if current_pose is None:
            return

        target = self._map_unity_target_to_robot(self.pending_hand_target)
        if self.servo.publish_target(target, current_pose):
            self._info_throttled(
                "servo_tracking",
                f"Servo tracking target xyz=({target.pose.position.x:.3f}, "
                f"{target.pose.position.y:.3f}, "
                f"{target.pose.position.z:.3f})",
                2.0,
            )

    def _map_unity_target_to_robot(
        self,
        input_target: PoseStamped,
    ) -> PoseStamped:
        input_anchor = self.hand_input_anchor
        robot_anchor = self.robot_wrist_anchor
        if input_anchor is None or robot_anchor is None:
            return _copy_pose_stamped(input_target)

        target = _copy_pose_stamped(robot_anchor)
        target.header.stamp = input_target.header.stamp
        target.header.frame_id = self.pose_reference_frame
        target.pose.position.x += (
            input_target.pose.position.x - input_anchor.pose.position.x
        )
        target.pose.position.y += (
            input_target.pose.position.y - input_anchor.pose.position.y
        )
        target.pose.position.z += (
            input_target.pose.position.z - input_anchor.pose.position.z
        )

        input_rotation_delta = _multiply_quaternions(
            input_target.pose.orientation,
            _inverse_quaternion(input_anchor.pose.orientation),
        )
        target.pose.orientation = _multiply_quaternions(
            input_rotation_delta,
            robot_anchor.pose.orientation,
        )
        return target

    def _apply_hand_target_adjustments(
        self,
        message: PoseStamped,
    ) -> PoseStamped:
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
            position.z = _clamp(
                position.z,
                self.min_hand_target_z_m,
                self.max_hand_target_z_m,
            )

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
        return True

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

    def _warn_throttled(
        self,
        key: str,
        message: str,
        period_sec: float,
    ) -> None:
        if self._should_log(key, period_sec):
            self.get_logger().warn(message)

    def _info_throttled(
        self,
        key: str,
        message: str,
        period_sec: float,
    ) -> None:
        if self._should_log(key, period_sec):
            self.get_logger().info(message)

    def _should_log(self, key: str, period_sec: float) -> bool:
        now_sec = self._now_sec()
        last_sec = self.last_log_times.get(key)
        if last_sec is not None and now_sec - last_sec < period_sec:
            return False
        self.last_log_times[key] = now_sec
        return True

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


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
