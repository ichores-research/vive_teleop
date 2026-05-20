import base64
import json

from rclpy.node import Node
from std_msgs.msg import String


DEFAULT_INPUT_TOPIC = "/vive/input_mock"


class MockInputPublisher(Node):
    def __init__(self, topic: str = DEFAULT_INPUT_TOPIC):
        super().__init__("mock_input_publisher")
        self._publisher = self.create_publisher(String, topic, 10)
        self._topic = topic

    def publish_input(self, payload: str | bytes) -> None:
        message = String()
        message.data = self._serialize_payload(payload)
        self._publisher.publish(message)
        preview = message.data[:200] + ("..." if len(message.data) > 200 else "")
        self.get_logger().info(f"Mock input forwarded to {self._topic}: {preview}")

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
