import asyncio
import json
import numpy as np
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from aiortc import RTCPeerConnection, RTCSessionDescription, RTCConfiguration, RTCIceServer, VideoStreamTrack
from aiortc.contrib.media import MediaRelay
from av import VideoFrame
from aiohttp import web
import aiohttp_cors

class ROSVideoTrack(VideoStreamTrack):
    def __init__(self):
        super().__init__()
        self.frame = None

    def update_frame(self, frame):
        self.frame = frame

    async def recv(self):
        pts, time_base = await self.next_timestamp()
        if self.frame is None:
            img = np.zeros((480, 640, 3), np.uint8)
        else:
            img = self.frame
        new_frame = VideoFrame.from_ndarray(img, format="bgr24")
        new_frame.pts = pts
        new_frame.time_base = time_base
        return new_frame

class WebRTCForwarder(Node):
    def __init__(self):
        super().__init__("webrtc_forwarder")
        self.bridge = CvBridge()
        self.video_track = ROSVideoTrack()
        self.create_subscription(Image, "/xtion/rgb/image_raw", self.callback, 10)

    def callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.video_track.update_frame(cv_image)

pcs = set()
node = None

ICE_CONFIG = RTCConfiguration(
    iceServers=[
        RTCIceServer(
            urls=[
                "turn:10.68.0.133:3478?transport=udp",
                "turn:10.68.0.133:3478?transport=tcp",
            ],
            username="dummy",
            credential="dummy"
        )
    ]
)

async def offer(request):
    params = await request.json()
    sdp_offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

    pc = RTCPeerConnection(configuration=ICE_CONFIG)
    pcs.add(pc)

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        if pc.connectionState == "failed":
            await pc.close()
            pcs.discard(pc)

    pc.addTrack(node.video_track)
    await pc.setRemoteDescription(sdp_offer)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    gathering_timeout = 10  # seconds
    start = asyncio.get_event_loop().time()
    while pc.iceGatheringState != "complete":
        if asyncio.get_event_loop().time() - start > gathering_timeout:
            break
        await asyncio.sleep(0.1)

    return web.Response(
        content_type="application/json",
        text=json.dumps({
            "sdp": pc.localDescription.sdp,
            "type": pc.localDescription.type,
        }),
    )

async def on_shutdown(app):
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()

def main():
    global node
    rclpy.init()
    node = WebRTCForwarder()
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()
    app = web.Application()
    cors = aiohttp_cors.setup(app, defaults={
        "*": aiohttp_cors.ResourceOptions(
            allow_credentials=True,
            expose_headers="*",
            allow_headers="*",
        )
    })
    app.on_shutdown.append(on_shutdown)
    route = app.router.add_post("/offer", offer)
    cors.add(route)
    web.run_app(app, host="0.0.0.0", port=8088)

if __name__ == "__main__":
    main()