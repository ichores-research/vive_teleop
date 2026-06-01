from threading import Lock
from typing import Any

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


DEFAULT_CURRENT_WRIST_POSE_TOPIC = "/vive/current_wrist_pose"


class CurrentWristPoseSubscriber(Node):
    def __init__(
        self,
        current_wrist_pose_topic: str = DEFAULT_CURRENT_WRIST_POSE_TOPIC,
    ):
        super().__init__("current_wrist_pose_subscriber")
        self._current_wrist_pose_topic = current_wrist_pose_topic
        self._latest_payload: dict[str, Any] | None = None
        self._lock = Lock()
        self._received_first_pose = False

        self.create_subscription(
            PoseStamped,
            current_wrist_pose_topic,
            self._on_current_wrist_pose,
            10,
        )
        self.get_logger().info(
            f"Listening for current wrist poses on '{current_wrist_pose_topic}'"
        )

    def latest_payload(self) -> dict[str, Any]:
        with self._lock:
            if self._latest_payload is None:
                return {
                    "available": False,
                    "topic": self._current_wrist_pose_topic,
                    "message": "No current wrist pose has been received yet.",
                }

            payload = dict(self._latest_payload)

        stamp = float(payload.get("timestamp") or 0.0)
        if stamp > 0.0:
            now_sec = self.get_clock().now().nanoseconds / 1e9
            payload["ageSec"] = max(0.0, now_sec - stamp)

        return payload

    def _on_current_wrist_pose(self, message: PoseStamped) -> None:
        position = message.pose.position
        orientation = message.pose.orientation
        timestamp = message.header.stamp.sec + (message.header.stamp.nanosec / 1e9)
        frame = message.header.frame_id
        payload = {
            "available": True,
            "topic": self._current_wrist_pose_topic,
            "timestamp": timestamp,
            "frame": frame,
            "wristAvailable": True,
            "wristSource": "robot_current_pose",
            "wristFrame": frame,
            "wristPx": position.x,
            "wristPy": position.y,
            "wristPz": position.z,
            "wristRx": orientation.x,
            "wristRy": orientation.y,
            "wristRz": orientation.z,
            "wristRw": orientation.w,
            "robotWristFrame": frame,
            "robotWristRx": orientation.x,
            "robotWristRy": orientation.y,
            "robotWristRz": orientation.z,
            "robotWristRw": orientation.w,
        }

        with self._lock:
            self._latest_payload = payload

        if not self._received_first_pose:
            self.get_logger().info(
                "Received first current wrist pose "
                f"frame='{frame}' "
                f"xyz=({position.x:.3f}, {position.y:.3f}, {position.z:.3f})"
            )
            self._received_first_pose = True
