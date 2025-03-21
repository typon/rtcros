import threading
import asyncio
import json
import queue
from pathlib import Path
from fractions import Fraction

import rclpy
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaStreamTrack
from av import VideoFrame
import numpy as np

# Get package directory for static files
PACKAGE_SHARED_DIR = Path(get_package_share_directory("ros2_webcam_stream"))
WEB_DIR = PACKAGE_SHARED_DIR / "web"
assert WEB_DIR.exists(), f"Web directory not found at {WEB_DIR}"

# Global variables
frame_queue = queue.Queue(maxsize=10)
pcs = set()

# ROS2 node to subscribe to Image messages
class ImageSubscriber(Node):
    def __init__(self, frame_queue: queue.Queue):
        super().__init__("ros_image_subscriber")
        self.subscription = self.create_subscription(
            Image, "image", self.listener_callback, 10
        )
        self.bridge = CvBridge()
        self.frame_queue = frame_queue
        self.get_logger().info("Waiting for images on topic 'image'...")

    def listener_callback(self, msg: Image) -> None:
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8").copy()
            
            # Make room in the queue if needed
            if self.frame_queue.full():
                try:
                    self.frame_queue.get_nowait()
                except queue.Empty:
                    pass
            
            # Add new frame
            self.frame_queue.put(cv_image)
        except Exception as e:
            self.get_logger().error(f"Conversion failed: {e}")

# Custom VideoStreamTrack that pulls frames from the ROS2 subscriber queue
class ROSVideoStreamTrack(MediaStreamTrack):
    kind = "video"

    def __init__(self, frame_queue: queue.Queue):
        super().__init__()
        self.frame_queue = frame_queue
        self.frame_count: int = 0
        self._start_time: float = 0
        self._latest_frame: np.ndarray = None
        self._last_log_time: float = 0
        
        # Create an empty frame as fallback
        self._empty_frame: np.ndarray = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # Start the frame update task
        asyncio.ensure_future(self._update_frame_task())
        
    async def _update_frame_task(self) -> None:
        """Task to continuously update the latest frame from the queue"""
        while True:
            try:
                # Try to get a new frame (non-blocking)
                new_frame = self.frame_queue.get_nowait()
                self._latest_frame = new_frame
            except queue.Empty:
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
            
        # Log frame information occasionally (every second)
        current_time = asyncio.get_event_loop().time()
        if not self._last_log_time or current_time - self._last_log_time > 1.0:
            mean_value = np.mean(frame)
            print(f"Mean value of frame: {mean_value}")
            self._last_log_time = current_time

        # Create video frame with proper timing
        video_frame: VideoFrame = VideoFrame.from_ndarray(frame, format="bgr24")
        
        # Initialize start time on first frame
        if not self._start_time:
            self._start_time = current_time
            
        # Calculate presentation timestamp
        video_frame.pts = int((current_time - self._start_time) * 90000)
        video_frame.time_base = Fraction(1, 90000)
        
        # Enforce frame pacing for ~30fps
        await asyncio.sleep(1/30)
        
        return video_frame

# Web routes
async def index(request: web.Request) -> web.Response:
    with open(WEB_DIR / "index.html") as f:
        content = f.read()
    return web.Response(content_type="text/html", text=content)

async def javascript(request: web.Request) -> web.Response:
    with open(WEB_DIR / "client.js") as f:
        content = f.read()
    return web.Response(content_type="application/javascript", text=content)

async def offer(request: web.Request) -> web.Response:
    params = await request.json()
    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

    pc = RTCPeerConnection()
    pcs.add(pc)

    @pc.on("connectionstatechange")
    async def on_connectionstatechange() -> None:
        print(f"Connection state is {pc.connectionState}")
        if pc.connectionState == "failed":
            await pc.close()
            pcs.discard(pc)

    # Add our video track
    video = ROSVideoStreamTrack(frame_queue)
    pc.addTrack(video)

    await pc.setRemoteDescription(offer)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    return web.Response(
        content_type="application/json",
        text=json.dumps(
            {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
        ),
    )

async def on_shutdown(app: web.Application) -> None:
    # Close peer connections
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()

# Run ROS node in a separate thread
def run_ros_node() -> None:
    rclpy.init()
    ros_node = ImageSubscriber(frame_queue)
    # Create MultiThreadedExecutor with 3 threads
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(ros_node)
    executor.spin()
    
    # Clean up
    ros_node.destroy_node()
    rclpy.shutdown()

def main() -> None:
    # Start ROS2 node in a separate thread
    ros_thread = threading.Thread(target=run_ros_node, daemon=True)
    ros_thread.start()
    print("ROS2 node started")

    # Set up web application
    app = web.Application()
    app.on_shutdown.append(on_shutdown)
    app.router.add_get("/", index)
    app.router.add_get("/client.js", javascript)
    app.router.add_post("/offer", offer)
    
    # Run web application
    print(f"Starting web server at http://localhost:8080")
    web.run_app(app, host="0.0.0.0", port=8080)

if __name__ == "__main__":
    main()
