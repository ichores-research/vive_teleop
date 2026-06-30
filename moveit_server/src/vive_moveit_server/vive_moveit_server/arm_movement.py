import math

from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.qos import qos_profile_system_default
from rclpy.task import Future
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener

from .teleop_math import normalize_quaternion as _normalize_quaternion

try:
    from moveit_msgs.srv import ServoCommandType
except ImportError:
    ServoCommandType = None


SERVO_POSE_TOPIC = "/servo_node/pose_target_cmds"
SERVO_POSE_ACTIVE_TOPIC = "/servo_node/pose_target_active"
SERVO_SWITCH_COMMAND_TYPE_SERVICE = "/servo_node/switch_command_type"


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _position_norm(message: PoseStamped) -> float:
    position = message.pose.position
    return math.sqrt(
        (position.x * position.x)
        + (position.y * position.y)
        + (position.z * position.z)
    )


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


class ArmMovementMixin:
    """Arm clutching and MoveIt Servo pose-command publication."""

    def _on_hand_target_active(self, message: Bool) -> None:
        if message.data:
            self._publish_servo_pose_active(True)
            return
        if (
            self.hand_target_active
            or self.pending_hand_target is not None
            or self.deadman_controller_anchor is not None
            or self.deadman_robot_anchor is not None
            or getattr(self, "_servo_pose_commands_active", False)
        ):
            self._reset_hand_target_pursuit()
        else:
            self._publish_servo_pose_active(False)

    def _on_hand_target(self, message: PoseStamped) -> None:
        position = message.pose.position
        if not all(
            math.isfinite(value)
            for value in (position.x, position.y, position.z)
        ):
            self._warn_throttled(
                "hand_input_non_finite",
                "Ignoring hand target with non-finite position",
                2.0,
            )
            return

        if not self.hand_target_active:
            self._publish_servo_pose_active(True)
            controller_anchor = self._scale_hand_target(message)
            if not _normalize_quaternion(controller_anchor.pose.orientation):
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
        self.last_hand_target_received_sec = self.get_clock().now().nanoseconds / 1e9

    def _reset_hand_target_pursuit(self) -> None:
        self.pending_hand_target = None
        self.last_commanded_target = None
        self.deadman_controller_anchor = None
        self.deadman_robot_anchor = None
        self.last_hand_target_received_sec = 0.0
        self.hand_target_active = False
        self._publish_servo_pose_active(False)
        self.get_logger().info(
            "Deadman inactive; cleared hand-target pursuit and Servo queue"
        )

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

        if not self.hand_target_active or self.pending_hand_target is None:
            return

        controller_pose = self._scale_hand_target(
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
            if not self._capture_robot_anchor_from_tf():
                return

        target = self._map_controller_delta_to_robot(controller_pose)
        target.header.frame_id = self.pose_reference_frame
        target.header.stamp = self.get_clock().now().to_msg()

        if not self._constrain_hand_target_to_workspace(target):
            self.pending_hand_target = None
            return

        if not self._servo_twist_mode_ready():
            return

        self._servo_pose_publisher.publish(target)
        self.pending_hand_target = None
        self.last_commanded_target = _copy_pose_stamped(target)
        self._info_throttled(
            "servo_pose_success",
            f"Published MoveIt Servo pose command for '{self.arm_group}'",
            2.0,
        )

    def _capture_robot_anchor_from_tf(self) -> bool:
        self._ensure_servo_interfaces()

        try:
            transform = self._servo_tf_buffer.lookup_transform(
                self.pose_reference_frame,
                self.end_effector_link,
                Time(),
            )
        except TransformException as error:
            self._warn_throttled(
                "servo_tf_wait",
                "Waiting for wrist transform "
                f"'{self.pose_reference_frame}' -> '{self.end_effector_link}': "
                f"{error}",
                2.0,
            )
            return False

        current_pose = PoseStamped()
        current_pose.header = transform.header
        current_pose.pose.position.x = transform.transform.translation.x
        current_pose.pose.position.y = transform.transform.translation.y
        current_pose.pose.position.z = transform.transform.translation.z
        current_pose.pose.orientation = transform.transform.rotation
        if not all(
            math.isfinite(value)
            for value in (
                current_pose.pose.position.x,
                current_pose.pose.position.y,
                current_pose.pose.position.z,
            )
        ):
            self._warn_throttled(
                "servo_tf_position",
                "TF returned a non-finite wrist position",
                2.0,
            )
            return False
        if not _normalize_quaternion(current_pose.pose.orientation):
            self._warn_throttled(
                "servo_tf_quaternion",
                "TF returned an invalid wrist orientation",
                2.0,
            )
            return False

        self.deadman_robot_anchor = current_pose
        self.last_commanded_target = _copy_pose_stamped(current_pose)
        self.get_logger().info(
            "Deadman clutch anchored to the current robot wrist pose from TF"
        )
        return True

    def _ensure_servo_interfaces(self) -> None:
        if hasattr(self, "_servo_pose_publisher"):
            return

        self._servo_pose_publisher = self.create_publisher(
            PoseStamped,
            SERVO_POSE_TOPIC,
            qos_profile_system_default,
        )
        self._servo_pose_active_publisher = self.create_publisher(
            Bool,
            SERVO_POSE_ACTIVE_TOPIC,
            qos_profile_system_default,
        )
        self._servo_pose_commands_active = False
        self._servo_switch_command_type_client = None
        if ServoCommandType is not None:
            self._servo_switch_command_type_client = self.create_client(
                ServoCommandType,
                SERVO_SWITCH_COMMAND_TYPE_SERVICE,
            )
        self._servo_tf_buffer = Buffer()
        self._servo_tf_listener = TransformListener(
            self._servo_tf_buffer,
            self,
        )
        self._servo_twist_selected = False
        self._servo_switch_in_flight = False

    def _publish_servo_pose_active(self, active: bool) -> None:
        if not hasattr(self, "_servo_pose_active_publisher"):
            self._ensure_servo_interfaces()

        # Repeat both states so a newly discovered subscriber converges to the
        # current deadman state without relying on the first sample.
        self._servo_pose_active_publisher.publish(Bool(data=active))
        self._servo_pose_commands_active = active

    def _servo_twist_mode_ready(self) -> bool:
        self._ensure_servo_interfaces()
        if self._servo_twist_selected:
            return True
        if self._servo_switch_in_flight:
            return False
        if (
            ServoCommandType is None
            or self._servo_switch_command_type_client is None
        ):
            return True
        if not self._servo_switch_command_type_client.wait_for_service(
            timeout_sec=self.servo_service_wait_timeout_sec
        ):
            self._warn_throttled(
                "servo_switch_wait",
                "Waiting for MoveIt Servo command service "
                f"'{SERVO_SWITCH_COMMAND_TYPE_SERVICE}'",
                5.0,
            )
            return False

        request = ServoCommandType.Request()
        request.command_type = ServoCommandType.Request.TWIST
        self._servo_switch_in_flight = True
        future = self._servo_switch_command_type_client.call_async(request)
        future.add_done_callback(self._on_servo_command_type_response)
        return False

    def _on_servo_command_type_response(self, future: Future) -> None:
        self._servo_switch_in_flight = False
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().warn(
                f"MoveIt Servo command-type request failed: {error}"
            )
            return

        if not response.success:
            self._warn_throttled(
                "servo_switch_rejected",
                "MoveIt Servo rejected TWIST command mode",
                2.0,
            )
            return

        self._servo_twist_selected = True
        self.get_logger().info("MoveIt Servo TWIST command mode selected")

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

    def _scale_hand_target(self, message: PoseStamped) -> PoseStamped:
        target = _copy_pose_stamped(message)
        target.pose.position.x *= self.hand_position_scale[0]
        target.pose.position.y *= self.hand_position_scale[1]
        target.pose.position.z *= self.hand_position_scale[2]
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
                horizontal_scale = max_horizontal_distance / horizontal_distance
                position.x *= horizontal_scale
                position.y *= horizontal_scale
            self._warn_throttled(
                "target_distance_limit",
                f"Clamping hand target {distance_m:.3f}m from base to "
                f"{self.max_hand_target_distance_m:.3f}m",
                2.0,
            )

        return True
