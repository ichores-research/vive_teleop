import json
import math
from typing import Any

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String


DEFAULT_HEADSET_POSE_TOPIC = "vive/headset_pose"
TIAGO_HEAD_PAN_MIN = -1.24 * 0.9
TIAGO_HEAD_PAN_MAX = 1.24 * 0.9
TIAGO_HEAD_TILT_MIN = -0.98 * 0.9
TIAGO_HEAD_TILT_MAX = 0.79 * 0.9
DEFAULT_HEAD_POSE_TOPIC = "/vive/head_pose"
DEFAULT_HAND_TARGET_TOPIC = "/vive/hand_target_pose"


class WebRtcInputPublisher(Node):
    def __init__(
        self,
        headset_pose_topic: str = DEFAULT_HEADSET_POSE_TOPIC,
        head_pose_topic: str = DEFAULT_HEAD_POSE_TOPIC,
        hand_target_topic: str = DEFAULT_HAND_TARGET_TOPIC,
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
        self._headset_pose_publisher = self.create_publisher(
            String,
            headset_pose_topic,
            10,
        )
        self._headset_reference_inverse: dict[str, float] | None = None
        self._last_input_log_sec = 0.0
        self._input_log_interval_sec = 2.0

    def publish_input(self, payload: str | bytes) -> None:
        payload_text = self._serialize_payload(payload)
        data = self._parse_json_object(payload_text)
        if data is not None:
            self._publish_head_pose(data)
            self._publish_hand_target(data)
            self._publish_headset_pose(data)
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
        if not data.get("hmdAvailable"):
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
        if not data.get("wristAvailable"):
            return

        message = self._extract_pose(
            data,
            position_prefix="wristP",
            quaternion_prefix="robotWristR",
            frame_id=str(data.get("robotWristFrame") or "unity_world"),
            warning_context="hand target pose",
        )
        if message is None:
            return

        self._hand_target_publisher.publish(message)

    def _publish_headset_pose(self, data: dict[str, Any]) -> None:
        if not data.get("hmdAvailable"):
            return

        quaternion = self._extract_headset_quaternion(data)
        if quaternion is None:
            return

        if self._headset_reference_inverse is None or data.get("headsetRecenter"):
            self._headset_reference_inverse = self._invert_quaternion(quaternion)
            self.get_logger().info("Recentered headset pose reference")

        relative_quaternion = self._multiply_quaternions(
            self._headset_reference_inverse,
            quaternion,
        )
        raw_pan, raw_tilt = self._quaternion_to_pan_tilt(relative_quaternion)
        pan = self._clamp(raw_pan, TIAGO_HEAD_PAN_MIN, TIAGO_HEAD_PAN_MAX)
        tilt = self._clamp(raw_tilt, TIAGO_HEAD_TILT_MIN, TIAGO_HEAD_TILT_MAX)
        limited = pan != raw_pan or tilt != raw_tilt

        message = String()
        message.data = json.dumps(
            {
                "pan": pan,
                "tilt": tilt,
                "frame": "calibrated_relative",
                "limited": limited,
            }
        )
        self._headset_pose_publisher.publish(message)

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

        norm_squared = sum(value * value for value in quaternion.values())
        if norm_squared < 1e-6:
            self.get_logger().warning(
                f"Ignoring zero-length {warning_context} quaternion"
            )
            return None

        norm = norm_squared ** 0.5
        for axis in quaternion:
            quaternion[axis] /= norm

        return quaternion

    @staticmethod
    def _invert_quaternion(
        quaternion: dict[str, float],
    ) -> dict[str, float]:
        return {
            "x": -quaternion["x"],
            "y": -quaternion["y"],
            "z": -quaternion["z"],
            "w": quaternion["w"],
        }

    @staticmethod
    def _multiply_quaternions(
        first: dict[str, float],
        second: dict[str, float],
    ) -> dict[str, float]:
        x1 = first["x"]
        y1 = first["y"]
        z1 = first["z"]
        w1 = first["w"]
        x2 = second["x"]
        y2 = second["y"]
        z2 = second["z"]
        w2 = second["w"]

        return {
            "x": w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            "y": w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            "z": w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            "w": w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        }

    @staticmethod
    def _quaternion_to_pan_tilt(
        quaternion: dict[str, float],
    ) -> tuple[float, float]:
        x = quaternion["x"]
        y = quaternion["y"]
        z = quaternion["z"]
        w = quaternion["w"]
        pan = math.atan2(
            2 * (w * y + x * z),
            1 - 2 * (y * y + x * x),
        )
        tilt_value = WebRtcInputPublisher._clamp(
            2 * (w * x - z * y),
            -1.0,
            1.0,
        )
        tilt = math.asin(tilt_value)

        return pan, tilt

    @staticmethod
    def _clamp(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, value))

    def _extract_headset_quaternion(
        self,
        data: dict[str, Any],
    ) -> dict[str, float] | None:
        fields = {
            "x": "hmdRx",
            "y": "hmdRy",
            "z": "hmdRz",
            "w": "hmdRw",
        }

        try:
            quaternion = {
                axis: float(data[field_name])
                for axis, field_name in fields.items()
            }
        except (KeyError, TypeError, ValueError):
            self.get_logger().warning(
                "Input payload had hmdAvailable=true but no valid HMD quaternion"
            )
            return None

        norm_squared = sum(value * value for value in quaternion.values())
        if norm_squared < 1e-6:
            self.get_logger().warning("Ignoring zero-length HMD quaternion")
            return None

        norm = norm_squared ** 0.5
        for axis in quaternion:
            quaternion[axis] /= norm

        return quaternion
