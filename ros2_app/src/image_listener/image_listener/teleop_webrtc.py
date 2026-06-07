import asyncio
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor

from .image_subscriber import DEFAULT_IMAGE_TOPIC, ImageSubscriber
from .input_publisher import WebRtcInputPublisher
from .robot_state import RobotInputState
from .video_track import LatestFrameVideoTrack
from .webrtc_server import WebRTCServer


def main(args=None):
    rclpy.init(args=args)

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    video_track = LatestFrameVideoTrack()
    image_subscriber = ImageSubscriber(
        frame_sink=video_track,
        event_loop=loop,
        image_topic=DEFAULT_IMAGE_TOPIC,
    )
    input_publisher = WebRtcInputPublisher()
    robot_input_state = RobotInputState()

    executor = MultiThreadedExecutor()
    executor.add_node(image_subscriber)
    executor.add_node(input_publisher)
    executor.add_node(robot_input_state)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    server = WebRTCServer(host="0.0.0.0", port=8088)
    server.add_video_route("/offer", video_track)
    server.add_input_route("/input_offer", input_publisher.publish_input)
    server.add_json_route("/robot_state", robot_input_state.get_snapshot)

    try:
        server.run(loop=loop)
    finally:
        executor.shutdown()
        image_subscriber.destroy_node()
        input_publisher.destroy_node()
        robot_input_state.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
