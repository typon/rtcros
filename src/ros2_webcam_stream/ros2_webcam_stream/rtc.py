import asyncio
import json
from collections import deque
from fractions import Fraction
from pathlib import Path

import numpy as np
from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaStreamTrack
from av import VideoFrame
from ament_index_python.packages import get_package_share_directory


class VideoStreamTrack(MediaStreamTrack):
    """MediaStreamTrack that converts numpy images to video frames."""

    kind = "video"

    def __init__(self, images_queue: deque[np.ndarray], stream_rate: int, codec: str):
        super().__init__()
        self.images_queue = images_queue
        self.frame_count: int = 0
        self._start_time: float = 0
        self._latest_frame: np.ndarray = None
        self._last_log_time: float = 0
        self.stream_rate: int = stream_rate
        self.codec: str = codec

        # Create an empty frame as fallback
        self._empty_frame: np.ndarray = np.zeros((480, 640, 3), dtype=np.uint8)

        # Start the frame update task
        asyncio.ensure_future(self._update_frame_task())

    async def _update_frame_task(self) -> None:
        """Task to continuously update the latest frame from the queue"""
        while True:
            try:
                # Try to get a new frame (non-blocking)
                if len(self.images_queue) > 0:
                    self._latest_frame = self.images_queue.popleft()
            except IndexError:
                # No new frame available, continue with current frame
                pass

            # Sleep a short time before checking again
            await asyncio.sleep(0.01)  # 10ms

    async def recv(self) -> VideoFrame:
        # Get current frame (or empty frame if none available yet)
        if self._latest_frame is None:
            frame = self._empty_frame
        else:
            frame = self._latest_frame

        # Create video frame with proper timing
        video_frame: VideoFrame = VideoFrame.from_ndarray(frame, format="bgr24")

        # Initialize start time on first frame
        current_time = asyncio.get_event_loop().time()
        if not self._start_time:
            self._start_time = current_time

        # Calculate presentation timestamp
        video_frame.pts = int((current_time - self._start_time) * 90000)
        video_frame.time_base = Fraction(1, 90000)

        # Enforce frame pacing
        await asyncio.sleep(1 / self.stream_rate)

        return video_frame


class VideoRTCStreamer:
    """Class to stream video frames via WebRTC."""

    def __init__(self, images_queue: deque[np.ndarray], port: int, stream_rate: int = 30, codec: str = "h264"):
        self.images_queue = images_queue
        self.port = port
        self.stream_rate = stream_rate
        self.codec = codec
        self.pcs: set[RTCPeerConnection] = set()

        # Get package directory for static files
        self.package_shared_dir = Path(get_package_share_directory("ros2_webcam_stream"))
        self.web_dir = self.package_shared_dir / "web"
        assert self.web_dir.exists(), f"Web directory not found at {self.web_dir}"

    async def index(self, request: web.Request) -> web.Response:
        """Serve the index.html page."""
        with open(self.web_dir / "index.html") as f:
            content = f.read()
        return web.Response(content_type="text/html", text=content)

    async def javascript(self, request: web.Request) -> web.Response:
        """Serve the client.js file."""
        with open(self.web_dir / "client.js") as f:
            content = f.read()
        return web.Response(content_type="application/javascript", text=content)

    async def offer(self, request: web.Request) -> web.Response:
        """Handle WebRTC offer from client."""
        params = await request.json()
        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

        pc = RTCPeerConnection()
        self.pcs.add(pc)

        @pc.on("connectionstatechange")
        async def on_connectionstatechange() -> None:
            print(f"Connection state is {pc.connectionState}")
            if pc.connectionState == "failed":
                await pc.close()
                self.pcs.discard(pc)

        # Add our video track
        video = VideoStreamTrack(self.images_queue, self.stream_rate, self.codec)
        pc.addTrack(video)

        await pc.setRemoteDescription(offer)
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        return web.Response(
            content_type="application/json",
            text=json.dumps({"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}),
        )

    async def on_shutdown(self, app: web.Application) -> None:
        """Clean up peer connections when shutting down."""
        coros = [pc.close() for pc in self.pcs]
        await asyncio.gather(*coros)
        self.pcs.clear()

    def run(self) -> None:
        """Start the web server and WebRTC service."""
        # Set up web application
        app = web.Application()
        app.on_shutdown.append(self.on_shutdown)
        app.router.add_get("/", self.index)
        app.router.add_get("/client.js", self.javascript)
        app.router.add_post("/offer", self.offer)

        # Run web application
        print(f"Starting WebRTC video streamer on port {self.port}")
        web.run_app(app, host="0.0.0.0", port=self.port)
