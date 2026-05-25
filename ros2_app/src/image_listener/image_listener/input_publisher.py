import base64
import json
import math
from typing import Any

from geometry_msgs.msg import QuaternionStamped
from rclpy.node import Node
from std_msgs.msg import String


DEFAULT_INPUT_TOPIC = "/vive/input_mock"
DEFAULT_ROBOT_WRIST_TOPIC = "/vive/robot_wrist_orientation"
DEFAULT_HEADSET_POSE_TOPIC = "vive/headset_pose"
TIAGO_HEAD_PAN_MIN = -1.24 * 0.9
TIAGO_HEAD_PAN_MAX = 1.24 * 0.9
TIAGO_HEAD_TILT_MIN = -0.98 * 0.9
TIAGO_HEAD_TILT_MAX = 0.79 * 0.9


class WebRtcInputPublisher(Node):
    def __init__(
        self,
        topic: str = DEFAULT_INPUT_TOPIC,
        robot_wrist_topic: str = DEFAULT_ROBOT_WRIST_TOPIC,
        headset_pose_topic: str = DEFAULT_HEADSET_POSE_TOPIC,
    ):
        super().__init__("webrtc_input_publisher")
        self._publisher = self.create_publisher(String, topic, 10)
        self._robot_wrist_publisher = self.create_publisher(
            QuaternionStamped,
            robot_wrist_topic,
            10,
        )
        self._headset_pose_publisher = self.create_publisher(
            String,
            headset_pose_topic,
            10,
        )
        self._topic = topic
        self._headset_reference_inverse: dict[str, float] | None = None

    def publish_input(self, payload: str | bytes) -> None:
        message = String()
        message.data = self._serialize_payload(payload)
        self._publisher.publish(message)
        self._publish_robot_wrist_orientation(message.data)
        self._publish_headset_pose(message.data)
        preview = message.data[:200] + ("..." if len(message.data) > 200 else "")
        self.get_logger().info(f"WebRTC input forwarded to {self._topic}: {preview}")

    @staticmethod
    def _serialize_payload(payload: str | bytes) -> str:
        if isinstance(payload, bytes):
            return json.dumps(
                {
                    "type": "bytes",
                    "encoding": "base64",
                    "data": base64.b64encode(payload).decode("ascii"),
                }
            )

        return payload

    def _publish_robot_wrist_orientation(self, payload: str) -> None:
        try:
            data = json.loads(payload)
        except json.JSONDecodeError:
            return

        if not isinstance(data, dict) or not data.get("wristAvailable"):
            return

        quaternion = self._extract_robot_wrist_quaternion(data)
        if quaternion is None:
            return

        message = QuaternionStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = str(data.get("robotWristFrame") or "unity_world")
        message.quaternion.x = quaternion["x"]
        message.quaternion.y = quaternion["y"]
        message.quaternion.z = quaternion["z"]
        message.quaternion.w = quaternion["w"]
        self._robot_wrist_publisher.publish(message)

    def _publish_headset_pose(self, payload: str) -> None:
        try:
            data = json.loads(payload)
        except json.JSONDecodeError:
            return

        if not isinstance(data, dict) or not data.get("hmdAvailable"):
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

    def _extract_robot_wrist_quaternion(
        self,
        data: dict[str, Any],
    ) -> dict[str, float] | None:
        fields = {
            "x": "robotWristRx",
            "y": "robotWristRy",
            "z": "robotWristRz",
            "w": "robotWristRw",
        }

        try:
            quaternion = {
                axis: float(data[field_name])
                for axis, field_name in fields.items()
            }
        except (KeyError, TypeError, ValueError):
            self.get_logger().warning(
                "Input payload had wristAvailable=true but no valid robotWrist quaternion"
            )
            return None

        norm_squared = sum(value * value for value in quaternion.values())
        if norm_squared < 1e-6:
            self.get_logger().warning("Ignoring zero-length robot wrist quaternion")
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
