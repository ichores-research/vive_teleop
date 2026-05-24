import base64
import json
from typing import Any

from geometry_msgs.msg import QuaternionStamped
from rclpy.node import Node
from std_msgs.msg import String


DEFAULT_INPUT_TOPIC = "/vive/input_mock"
DEFAULT_ROBOT_WRIST_TOPIC = "/vive/robot_wrist_orientation"


class WebRtcInputPublisher(Node):
    def __init__(
        self,
        topic: str = DEFAULT_INPUT_TOPIC,
        robot_wrist_topic: str = DEFAULT_ROBOT_WRIST_TOPIC,
    ):
        super().__init__("webrtc_input_publisher")
        self._publisher = self.create_publisher(String, topic, 10)
        self._robot_wrist_publisher = self.create_publisher(
            QuaternionStamped,
            robot_wrist_topic,
            10,
        )
        self._topic = topic

    def publish_input(self, payload: str | bytes) -> None:
        message = String()
        message.data = self._serialize_payload(payload)
        self._publisher.publish(message)
        self._publish_robot_wrist_orientation(message.data)
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
