import json
import math
from typing import Any

from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from std_msgs.msg import Bool, Float64


DEFAULT_HEAD_POSE_TOPIC = "/vive/head_pose"
DEFAULT_HAND_TARGET_TOPIC = "/vive/hand_target_pose"
DEFAULT_HAND_TARGET_ACTIVE_TOPIC = "/vive/hand_target_active"
DEFAULT_GRIPPER_TARGET_TOPIC = "/vive/gripper_opening"
DEFAULT_BASE_COMMAND_TOPIC = "/vive/base_command"
DEFAULT_BASE_ACTIVE_TOPIC = "/vive/base_active"
MAX_INPUT_PAYLOAD_BYTES = 64 * 1024


class WebRtcInputPublisher(Node):
    def __init__(
        self,
        head_pose_topic: str = DEFAULT_HEAD_POSE_TOPIC,
        hand_target_topic: str = DEFAULT_HAND_TARGET_TOPIC,
        base_command_topic: str = DEFAULT_BASE_COMMAND_TOPIC,
        base_active_topic: str = DEFAULT_BASE_ACTIVE_TOPIC,
    ):
        super().__init__("webrtc_input_publisher")
        self._head_pose_publisher = self.create_publisher(
            PoseStamped,
            head_pose_topic,
            10,
        )
        self._hand_target_publisher = self.create_publisher(
            PoseStamped,
            hand_target_topic,
            10,
        )
        self._hand_target_active_publisher = self.create_publisher(
            Bool,
            DEFAULT_HAND_TARGET_ACTIVE_TOPIC,
            10,
        )
        self._gripper_target_publisher = self.create_publisher(
            Float64,
            DEFAULT_GRIPPER_TARGET_TOPIC,
            10,
        )
        self._base_command_publisher = self.create_publisher(
            TwistStamped,
            base_command_topic,
            10,
        )
        self._base_active_publisher = self.create_publisher(
            Bool,
            base_active_topic,
            10,
        )
        self._base_frame = str(
            self.declare_parameter("base_frame", "base_footprint").value
        )
        self._base_joystick_deadzone = float(
            self.declare_parameter("base_joystick_deadzone", 0.15).value
        )
        self._base_max_linear_velocity_mps = float(
            self.declare_parameter(
                "base_max_linear_velocity_mps",
                0.25,
            ).value
        )
        self._base_max_angular_velocity_radps = float(
            self.declare_parameter(
                "base_max_angular_velocity_radps",
                0.6,
            ).value
        )
        self._last_input_log_sec = 0.0
        self._input_log_interval_sec = 2.0

    def publish_input(self, payload: str | bytes) -> None:
        payload_text = self._serialize_payload(payload)
        if (
            len(payload_text.encode("utf-8", errors="replace"))
            > MAX_INPUT_PAYLOAD_BYTES
        ):
            self.get_logger().warning(
                "Ignoring WebRTC input larger than 64 KiB"
            )
            return

        data = self._parse_json_object(payload_text)
        if data is not None:
            self._publish_head_pose(data)
            self._publish_hand_target_active(data)
            self._publish_hand_target(data)
            self._publish_gripper_target(data)
            self._publish_base_input(data)
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self._last_input_log_sec >= self._input_log_interval_sec:
            preview = payload_text[:200] + ("..." if len(payload_text) > 200 else "")
            self.get_logger().info(f"WebRTC input processed: {preview}")
            self._last_input_log_sec = now_sec

    @staticmethod
    def _serialize_payload(payload: str | bytes) -> str:
        if isinstance(payload, bytes):
            return payload.decode("utf-8", errors="replace")

        return payload

    @staticmethod
    def _parse_json_object(payload: str) -> dict[str, Any] | None:
        try:
            data = json.loads(payload)
        except json.JSONDecodeError:
            return None

        if not isinstance(data, dict):
            return None

        return data

    def _publish_head_pose(self, data: dict[str, Any]) -> None:
        if data.get("hmdAvailable") is not True:
            return

        message = self._extract_pose(
            data,
            position_prefix="hmdP",
            quaternion_prefix="hmdR",
            frame_id=str(data.get("hmdFrame") or "unity_world"),
            warning_context="HMD pose",
        )
        if message is None:
            return

        self._head_pose_publisher.publish(message)

    def _publish_hand_target(self, data: dict[str, Any]) -> None:
        if (
            data.get("wristAvailable") is not True
            or (
                "wristCommandEnabled" in data
                and data["wristCommandEnabled"] is not True
            )
        ):
            return

        position_prefix = (
            "robotWristP"
            if all(
                f"robotWristP{axis}" in data
                for axis in ("x", "y", "z")
            )
            else "wristP"
        )
        message = self._extract_pose(
            data,
            position_prefix=position_prefix,
            quaternion_prefix="robotWristR",
            frame_id=str(data.get("robotWristFrame") or "unity_world"),
            warning_context="hand target pose",
        )
        if message is None:
            return

        self._hand_target_publisher.publish(message)

    def _publish_hand_target_active(self, data: dict[str, Any]) -> None:
        if data.get("wristAvailable") is not True:
            return

        message = Bool()
        if "wristCommandEnabled" not in data:
            # Compatibility with the original Unity/browser payload schema.
            message.data = True
        elif not isinstance(data["wristCommandEnabled"], bool):
            self.get_logger().warning(
                "Ignoring non-boolean wrist command gate"
            )
            message.data = False
        else:
            message.data = data["wristCommandEnabled"]
        self._hand_target_active_publisher.publish(message)

    def _publish_gripper_target(self, data: dict[str, Any]) -> None:
        if data.get("gripperAvailable") is not True:
            return

        try:
            opening = float(data["gripperOpening"])
        except (KeyError, TypeError, ValueError):
            self.get_logger().warning(
                "Input payload had no valid gripper opening"
            )
            return

        if not math.isfinite(opening):
            self.get_logger().warning(
                "Ignoring non-finite gripper opening"
            )
            return

        message = Float64()
        message.data = max(0.0, min(1.0, opening))
        self._gripper_target_publisher.publish(message)

    def _publish_base_input(self, data: dict[str, Any]) -> None:
        if "joystickPrimaryButton" not in data:
            return

        active = Bool()
        button = data["joystickPrimaryButton"]
        if not isinstance(button, bool):
            self.get_logger().warning(
                "Ignoring non-boolean base command gate"
            )
            self._base_active_publisher.publish(active)
            return

        if data.get("wristAvailable") is not True or not button:
            self._base_active_publisher.publish(active)
            return

        try:
            axis_x = float(data["joystickAxisX"])
            axis_y = float(data["joystickAxisY"])
        except (KeyError, TypeError, ValueError):
            self.get_logger().warning(
                "Input payload had no valid base joystick axes"
            )
            self._base_active_publisher.publish(active)
            return

        if not all(math.isfinite(value) for value in (axis_x, axis_y)):
            self.get_logger().warning(
                "Ignoring non-finite base joystick axes"
            )
            self._base_active_publisher.publish(active)
            return

        linear_x, angular_z = self._joystick_to_base_velocity(
            axis_x,
            axis_y,
            deadzone=self._base_joystick_deadzone,
            max_linear_velocity_mps=self._base_max_linear_velocity_mps,
            max_angular_velocity_radps=self._base_max_angular_velocity_radps,
        )
        if not all(math.isfinite(value) for value in (linear_x, angular_z)):
            self.get_logger().warning(
                "Ignoring invalid base velocity configuration"
            )
            self._base_active_publisher.publish(active)
            return

        active.data = True
        self._base_active_publisher.publish(active)

        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = self._base_frame
        command.twist.linear.x = linear_x
        command.twist.angular.z = angular_z
        self._base_command_publisher.publish(command)

    @staticmethod
    def _joystick_to_base_velocity(
        axis_x: float,
        axis_y: float,
        *,
        deadzone: float,
        max_linear_velocity_mps: float,
        max_angular_velocity_radps: float,
    ) -> tuple[float, float]:
        """Map every joystick angle continuously to differential-drive motion."""
        magnitude = math.hypot(axis_x, axis_y)
        clamped_deadzone = max(0.0, min(0.95, deadzone))
        if magnitude <= clamped_deadzone or magnitude <= 1e-9:
            return 0.0, 0.0

        clamped_magnitude = min(1.0, magnitude)
        output_magnitude = (
            (clamped_magnitude - clamped_deadzone)
            / (1.0 - clamped_deadzone)
        )
        direction_x = axis_x / magnitude
        direction_y = axis_y / magnitude
        mapped_x = direction_x * output_magnitude
        mapped_y = direction_y * output_magnitude

        linear_x = mapped_y * max(0.0, max_linear_velocity_mps)
        # ROS positive angular Z is a left turn; joystick left is negative X.
        angular_z = -mapped_x * max(0.0, max_angular_velocity_radps)
        return linear_x, angular_z

    def _extract_pose(
        self,
        data: dict[str, Any],
        position_prefix: str,
        quaternion_prefix: str,
        frame_id: str,
        warning_context: str,
    ) -> PoseStamped | None:
        try:
            position = {
                "x": float(data[f"{position_prefix}x"]),
                "y": float(data[f"{position_prefix}y"]),
                "z": float(data[f"{position_prefix}z"]),
            }
        except (KeyError, TypeError, ValueError):
            self.get_logger().warning(
                f"Input payload had no valid {warning_context} position"
            )
            return None

        if not all(math.isfinite(value) for value in position.values()):
            self.get_logger().warning(
                f"Ignoring non-finite {warning_context} position"
            )
            return None

        quaternion = self._extract_quaternion(
            data,
            {
                "x": f"{quaternion_prefix}x",
                "y": f"{quaternion_prefix}y",
                "z": f"{quaternion_prefix}z",
                "w": f"{quaternion_prefix}w",
            },
            warning_context,
        )
        if quaternion is None:
            return None

        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = frame_id
        message.pose.position.x = position["x"]
        message.pose.position.y = position["y"]
        message.pose.position.z = position["z"]
        message.pose.orientation.x = quaternion["x"]
        message.pose.orientation.y = quaternion["y"]
        message.pose.orientation.z = quaternion["z"]
        message.pose.orientation.w = quaternion["w"]
        return message

    def _extract_quaternion(
        self,
        data: dict[str, Any],
        fields: dict[str, str],
        warning_context: str,
    ) -> dict[str, float] | None:
        try:
            quaternion = {
                axis: float(data[field_name])
                for axis, field_name in fields.items()
            }
        except (KeyError, TypeError, ValueError):
            self.get_logger().warning(
                f"Input payload had no valid {warning_context} quaternion"
            )
            return None

        if not all(math.isfinite(value) for value in quaternion.values()):
            self.get_logger().warning(
                f"Ignoring non-finite {warning_context} quaternion"
            )
            return None

        scale = max(abs(value) for value in quaternion.values())
        if scale == 0.0:
            self.get_logger().warning(
                f"Ignoring zero-length {warning_context} quaternion"
            )
            return None

        scaled = {
            axis: value / scale
            for axis, value in quaternion.items()
        }
        scaled_norm = math.sqrt(
            sum(value * value for value in scaled.values())
        )
        if scale < 1e-3 and (scale * scaled_norm) < 1e-3:
            self.get_logger().warning(
                f"Ignoring zero-length {warning_context} quaternion"
            )
            return None

        for axis in quaternion:
            quaternion[axis] = scaled[axis] / scaled_norm

        return quaternion
