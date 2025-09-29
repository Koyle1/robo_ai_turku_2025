#!/usr/bin/env python3
"""
Task 9: ROS2 Harris Corner Detection
- Harris corner detection with heatmap visualization
- Mark corners on original image
- Stack images in 2x2 grid with labels
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class HarrisCornerStudent(Node):
    def __init__(self):
        super().__init__('harris_corner_student')
        self.bridge = CvBridge()

        # Define the QoS profile for best effort
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # Subscribe to raw image topic
        self.create_subscription(Image, '/image_raw', self.image_callback, qos)

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            processed = self.process_cv_image(cv_image)
            cv2.imshow("Harris Corners - Student", processed)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error processing raw image: {e}")

    def process_cv_image(self, image):
        # Resize image for better performance
        frame = cv2.resize(image, (320, 240))
        
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_float = np.float32(gray)

        # Harris corner detection
        block_size = 2
        ksize = 3
        k = 0.04
        dst = cv2.cornerHarris(gray_float, blockSize=block_size, ksize=ksize, k=k)

        # Create heatmap visualization
        heatmap = cv2.normalize(dst, None, 0, 255, cv2.NORM_MINMAX)
        heatmap = np.uint8(heatmap)
        heatmap = cv2.applyColorMap(heatmap, cv2.COLORMAP_JET)
        
        # Mark corners on original image
        corners_img = frame.copy()
        corners_img[dst > 0.01 * dst.max()] = [0, 255, 0]  # Red corners

        # Add labels
        cv2.putText(frame, "Original", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        gray_bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        cv2.putText(gray_bgr, "Grayscale", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(heatmap, "Harris Heatmap", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(corners_img, "Corners Detected", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Stack images in 2x2 grid
        top = np.hstack((frame, gray_bgr))
        bottom = np.hstack((heatmap, corners_img))
        grid = np.vstack((top, bottom))

        return grid

def main(args=None):
    rclpy.init(args=args)
    node = HarrisCornerStudent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()