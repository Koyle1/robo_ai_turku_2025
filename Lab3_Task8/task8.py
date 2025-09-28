#!/usr/bin/env python3
"""
Task 8: ROS2 Image Processing with Canny Edge Detection
- Subscribe to raw or compressed image topics
- Apply Gaussian blur, Sobel gradients, and Canny edge detection
- Display results in a 2x3 grid with colored labels
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class CannyProcessor(Node):
    def __init__(self):
        super().__init__('canny_processor')
        self.bridge = CvBridge()
        self.processed_count = 0

        # Define the QoS profile for best effort
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # Create subscription for raw image
        self.create_subscription(Image, '/image_raw', self.image_callback, qos)

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            self.processed_count += 1
            processed = self.process_cv_image(cv_image)

            # Display the grid
            cv2.imshow("Canny Steps Grid", processed)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error processing raw image: {e}")

    def process_cv_image(self, image):
        """Resize, blur, apply Canny edge detection, stack grid with colored labels"""
        # Resize for better performance
        frame = cv2.resize(image, (213, 160))
        
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Compute Sobel gradients (X and Y)
        grad_x = cv2.Sobel(blurred, cv2.CV_64F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(blurred, cv2.CV_64F, 0, 1, ksize=3)
        magnitude = cv2.magnitude(grad_x, grad_y)
        magnitude = cv2.convertScaleAbs(magnitude)
        
        # Apply Canny edge detection
        edges = cv2.Canny(blurred, 50, 150)  # Adjustable thresholds
        
        # Convert to BGR for colored display
        gray_bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        blurred_bgr = cv2.cvtColor(blurred, cv2.COLOR_GRAY2BGR)
        magnitude_bgr = cv2.cvtColor(magnitude, cv2.COLOR_GRAY2BGR)
        edges_bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        
        # Add colored labels
        cv2.putText(frame, "Original", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(gray_bgr, "Grayscale", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(blurred_bgr, "Gaussian Blur", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(magnitude_bgr, "Sobel Magnitude", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(edges_bgr, "Canny Edges", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Stack images into 2x3 grid
        top_row = np.hstack((frame, gray_bgr, blurred_bgr))
        bottom_row = np.hstack((magnitude_bgr, edges_bgr, edges_bgr))  # Duplicate for 2x3
        grid = np.vstack((top_row, bottom_row))

        return grid

def main(args=None):
    rclpy.init(args=args)
    processor = CannyProcessor()
    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        pass
    finally:
        processor.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()