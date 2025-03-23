import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from pathlib import Path
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
import numpy as np

PACKAGE_SHARED_DIR = Path(get_package_share_directory("ros2_webcam_stream"))
IMAGE_PATH = PACKAGE_SHARED_DIR / "horse.jpg"
assert IMAGE_PATH.exists(), f"Image file not found at {IMAGE_PATH}"


class DiskImagePublisher(Node):
    def __init__(self):
        super().__init__("disk_image_publisher")
        self.publisher_ = self.create_publisher(Image, "image", 2)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.timer_callback)  # Publish 1 image per second
        self.get_logger().info(f"Attempting to load image from: {IMAGE_PATH}")
        self.image = cv2.imread(str(IMAGE_PATH))
        if self.image is None:
            self.get_logger().error(f"Failed to load image from {IMAGE_PATH}")
            self.get_logger().error("Make sure the file exists and is a valid image format")
            rclpy.shutdown()
        else:
            self.get_logger().info(f"Successfully loaded image with shape: {self.image.shape}")

    def timer_callback(self):
        img_copy = self.image.copy()

        # Get current time for both display and animation
        now = datetime.now()
        timestamp = now.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

        # Clear previous drawings - create a clean slate
        # This ensures changes are more noticeable
        overlay = np.zeros_like(img_copy)

        # Add timestamp with larger font
        cv2.putText(overlay, timestamp, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)

        # Add frame counter
        if not hasattr(self, "frame_count"):
            self.frame_count = 0
        self.frame_count += 1
        cv2.putText(overlay, f"Frame: {self.frame_count}", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 0, 0), 3)

        # Add MUCH more dramatic moving element
        seconds = now.second + now.microsecond / 1000000

        # Add a moving circle
        x = int(img_copy.shape[1] / 2 + 200 * np.sin(seconds))
        y = int(img_copy.shape[0] / 2 + 150 * np.cos(seconds))
        cv2.circle(overlay, (x, y), 60, (0, 0, 255), -1)

        # Add a second, different moving element
        x2 = int(img_copy.shape[1] / 2 + 150 * np.cos(seconds * 2))
        y2 = int(img_copy.shape[0] / 2 + 100 * np.sin(seconds * 2))
        cv2.rectangle(overlay, (x2 - 40, y2 - 40), (x2 + 40, y2 + 40), (255, 255, 0), -1)

        # Add the overlay to the original image
        # Use alpha blending to ensure changes are dramatic
        alpha = 0.7
        img_copy = cv2.addWeighted(img_copy, 1 - alpha, overlay, alpha, 0)

        # Create an Image message
        msg = self.bridge.cv2_to_imgmsg(img_copy, encoding="bgr8")

        # Set the header timestamp to current time
        current_time = self.get_clock().now()
        msg.header.stamp = current_time.to_msg()
        msg.header.frame_id = "camera_frame"

        # Log periodically to verify changes
        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f"Published frame {self.frame_count} with moving elements at ({x},{y}) and ({x2},{y2})"
            )

        # Publish the message
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DiskImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
