#!/usr/bin/env python3
"""
Task 5: ROS2 Image Processing with Sobel Filters - Student Template
- Create 2x2 grid: Original, Sobel magnitude, Sobel X, Sobel Y
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor_sobel')
        self.bridge = CvBridge()

        # Counters
        self.processed_count = 0
        self.compressed_count = 0

        # Define the QoS profile for best effort
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # Create subscriber for raw image
        self.create_subscription(Image, '/image_raw', self.image_callback, qos)
        
    # -----------------------------
    # Callbacks
    # -----------------------------
    def image_callback(self, msg):
        print("image received")
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            self.processed_count += 1
            if self.processed_count % 2 == 0:  # Skip every other frame for performance reasons
                processed = self.process_cv_image(cv_image)

                # Display 2x2 grid
                cv2.imshow("Sobel Grid", processed)
                cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error processing raw image: {e}")

    def compressed_image_callback(self, msg):
        try:
            # Convert ROS compressed image to OpenCV
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

            self.compressed_count += 1
            processed = self.process_cv_image(cv_image)

            # Display 2x2 grid
            cv2.imshow("Sobel Grid", processed)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error processing compressed image: {e}")

    # -----------------------------
    # Image processing
    # -----------------------------
    def process_cv_image(self, image):
        """Apply Sobel filters and stack 2x2 grid"""
        # Resize image (for performance reasons)
        frame = cv2.resize(image, (320, 240)) 

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Apply cv2.Sobel in X and Y directions
        sobel_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        sobel_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)

        # Compute Sobel magnitude
        sobel_mag = cv2.magnitude(sobel_x, sobel_y)

        # Convert to uint8 for display
        sobel_x = cv2.convertScaleAbs(sobel_x)
        sobel_y = cv2.convertScaleAbs(sobel_y)
        sobel_mag = cv2.convertScaleAbs(sobel_mag)

        # Convert grayscale images to BGR for stacking
        sobel_x_bgr = cv2.cvtColor(sobel_x, cv2.COLOR_GRAY2BGR)
        sobel_y_bgr = cv2.cvtColor(sobel_y, cv2.COLOR_GRAY2BGR)
        sobel_mag_bgr = cv2.cvtColor(sobel_mag, cv2.COLOR_GRAY2BGR)

        # Annotate each image
        cv2.putText(frame, "Original", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(sobel_mag_bgr, "Sobel Mag", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(sobel_x_bgr, "Sobel X", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(sobel_y_bgr, "Sobel Y", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Stack in 2x2 grid
        top_row = np.hstack((frame, sobel_mag_bgr))
        bottom_row = np.hstack((sobel_x_bgr, sobel_y_bgr))
        grid = np.vstack((top_row, bottom_row))

        return grid

def main(args=None):
    rclpy.init(args=args)
    processor = ImageProcessor()
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