import base64
import json
from typing import Any

from geometry_msgs.msg import PoseStamped, QuaternionStamped
from rclpy.node import Node
from std_msgs.msg import String


DEFAULT_INPUT_TOPIC = "/vive/input_mock"
DEFAULT_ROBOT_WRIST_TOPIC = "/vive/robot_wrist_orientation"
DEFAULT_HEAD_POSE_TOPIC = "/vive/head_pose"
DEFAULT_WRIST_POSE_TOPIC = "/vive/wrist_pose"
DEFAULT_HAND_TARGET_TOPIC = "/vive/hand_target_pose"


class WebRtcInputPublisher(Node):
    def __init__(
        self,
        topic: str = DEFAULT_INPUT_TOPIC,
        robot_wrist_topic: str = DEFAULT_ROBOT_WRIST_TOPIC,
        head_pose_topic: str = DEFAULT_HEAD_POSE_TOPIC,
        wrist_pose_topic: str = DEFAULT_WRIST_POSE_TOPIC,
        hand_target_topic: str = DEFAULT_HAND_TARGET_TOPIC,
    ):
        super().__init__("webrtc_input_publisher")
        self._publisher = self.create_publisher(String, topic, 10)
        self._robot_wrist_publisher = self.create_publisher(
            QuaternionStamped,
            robot_wrist_topic,
            10,
        )
        self._head_pose_publisher = self.create_publisher(
            PoseStamped,
            head_pose_topic,
            10,
        )
        self._wrist_pose_publisher = self.create_publisher(
            PoseStamped,
            wrist_pose_topic,
            10,
        )
        self._hand_target_publisher = self.create_publisher(
            PoseStamped,
            hand_target_topic,
            10,
        )
        self._topic = topic
        self._last_input_log_sec = 0.0
        self._input_log_interval_sec = 2.0

    def publish_input(self, payload: str | bytes) -> None:
        message = String()
        message.data = self._serialize_payload(payload)
        self._publisher.publish(message)
        data = self._parse_json_object(message.data)
        if data is not None:
            self._publish_robot_wrist_orientation(data)
            self._publish_head_pose(data)
            self._publish_wrist_pose(data)
            self._publish_hand_target(data)
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self._last_input_log_sec >= self._input_log_interval_sec:
            preview = message.data[:200] + ("..." if len(message.data) > 200 else "")
            self.get_logger().info(
                f"WebRTC input forwarded to {self._topic}: {preview}"
            )
            self._last_input_log_sec = now_sec

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

    @staticmethod
    def _parse_json_object(payload: str) -> dict[str, Any] | None:
        try:
            data = json.loads(payload)
        except json.JSONDecodeError:
            return None

        if not isinstance(data, dict):
            return None

        return data

    def _publish_robot_wrist_orientation(self, data: dict[str, Any]) -> None:
        if not data.get("wristAvailable"):
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

    def _publish_wrist_pose(self, data: dict[str, Any]) -> None:
        if not data.get("wristAvailable"):
            return

        message = self._extract_pose(
            data,
            position_prefix="wristP",
            quaternion_prefix="wristR",
            frame_id=str(data.get("wristFrame") or "unity_world"),
            warning_context="wrist pose",
        )
        if message is None:
            return

        self._wrist_pose_publisher.publish(message)

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

        return self._extract_quaternion(data, fields, "robot wrist orientation")

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
