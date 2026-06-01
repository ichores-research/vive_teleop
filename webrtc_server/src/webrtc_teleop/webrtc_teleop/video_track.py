import asyncio

import numpy as np
from aiortc import VideoStreamTrack
from av import VideoFrame


class LatestFrameVideoTrack(VideoStreamTrack):
    def __init__(self):
        super().__init__()
        self._frame: np.ndarray | None = None
        self._new_frame_event = asyncio.Event()

    def update_frame(self, frame: np.ndarray) -> None:
        self._frame = frame
        self._new_frame_event.set()

    async def recv(self) -> VideoFrame:
        await self._new_frame_event.wait()
        self._new_frame_event.clear()

        pts, time_base = await self.next_timestamp()
        frame = VideoFrame.from_ndarray(self._frame, format="bgr24")
        frame.pts = pts
        frame.time_base = time_base
        return frame
