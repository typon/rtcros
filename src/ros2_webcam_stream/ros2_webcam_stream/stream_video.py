import threading
from collections import deque
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np
from .rtc import VideoRTCStreamer


# ROS2 node to subscribe to Image messages
class ImageSubscriber(Node):
    def __init__(self, frame_queue: deque[np.ndarray]):
        super().__init__("ros_image_subscriber")
        # Use the frame queue passed from main
        self.frame_queue = frame_queue

        self.subscription = self.create_subscription(Image, "image", self.listener_callback, 10)
        self.bridge = CvBridge()
        self.get_logger().info("Waiting for images on topic 'image'...")

    def listener_callback(self, msg: Image) -> None:
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8").copy()
            # Add new frame to the deque (old frames are automatically removed if maxlen is reached)
            self.frame_queue.append(cv_image)
        except Exception as e:
            self.get_logger().error(f"Conversion failed: {e}")


# Run ROS node in a separate thread
def run_ros_node(frame_queue: deque[np.ndarray]) -> None:
    rclpy.init()

    # Create the ROS node
    ros_node = ImageSubscriber(frame_queue)

    # Create MultiThreadedExecutor with 3 threads
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(ros_node)

    print("ROS2 node started")

    # This will block until shutdown
    executor.spin()

    # Clean up
    ros_node.destroy_node()
    rclpy.shutdown()


def main() -> None:
    # Create shared frame queue
    frame_queue = deque(maxlen=10)

    # Start ROS node in a separate thread
    ros_thread = threading.Thread(target=run_ros_node, args=(frame_queue,), daemon=True)
    ros_thread.start()

    # Create and run the WebRTC streamer in the main thread (which supports asyncio signal handlers)
    rtc_streamer = VideoRTCStreamer(images_queue=frame_queue, port=8080, stream_rate=30, codec="h264")

    # This will block until program terminates
    rtc_streamer.run()


if __name__ == "__main__":
    main()
