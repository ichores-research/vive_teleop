import asyncio

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


DEFAULT_IMAGE_TOPIC = "/head_front_camera/rgb/image_raw"


class ImageSubscriber(Node):
    def __init__(
        self,
        frame_sink,
        event_loop: asyncio.AbstractEventLoop,
        image_topic: str = DEFAULT_IMAGE_TOPIC,
    ):
        super().__init__("image_subscriber")
        self.bridge = CvBridge()
        self._frame_sink = frame_sink
        self._event_loop = event_loop
        self._image_topic = self.declare_parameter(
            "image_topic",
            image_topic,
        ).value
        self._subscription = self.create_subscription(
            Image,
            self._image_topic,
            self.callback,
            rclpy.qos.qos_profile_sensor_data,
        )
        self.get_logger().info(f"Subscribing to camera topic '{self._image_topic}'")

    def callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self._event_loop.call_soon_threadsafe(
                self._frame_sink.update_frame,
                cv_image,
            )
        except Exception as e:
            self.get_logger().error(f"CV Bridge Error: {e}")


def main(args=None):
    from .teleop_webrtc import main as run_teleop_webrtc

    run_teleop_webrtc(args=args)

if __name__ == "__main__":
    main()
