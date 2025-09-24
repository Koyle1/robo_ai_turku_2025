#!/usr/bin/env python3
"""
Task 2: ROS2 Image Processing with RGB/HSV Filters - Student Template
- Subscribes to raw and/or compressed image topics
- Resizes images to 500px wide
- Applies RGB and HSV filters (red, green, blue)
- Displays results in a 2x2 grid
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import time




class ImageBagProcessor(Node):
    def __init__(self):
        super().__init__('image_bag_processor')

        self.bridge = CvBridge()
        
        # Counters
        self.processed_count = 0
        self.compressed_count = 0
        self.start_time = self.get_clock().now()

        # Define the QoS profile for best effort
        qos = QoSProfile(
            depth=1,  # Keep a shallow history
            reliability=QoSReliabilityPolicy.BEST_EFFORT  # Set to best effort
        )
        
        # TODO: Create subscribers for EITHER compressed and uncompressed images
        # Hint: self.create_subscription()
        subscriber_topic = 'image_raw' 
        self.subscription = self.create_subscription(
            Image,
            subscriber_topic,
            self.image_callback,
            qos
        )
        
        self.processed_count = 0

        self.get_logger().info("Image Bag Processor initialized")


    # USE ONLY ONE OF THE CALL BACKS DEPENDING ON THE TOPIC TYPE
    
    # -----------------------------
    # Callbacks 
    # -----------------------------
    def compressed_image_callback(self, msg):
        """Process compressed image messages"""
        try:
            # TODO: Convert ROS compressed image to OpenCV (BGR8)
            # Hint: self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

            self.compressed_count += 1
            
            # TODO: Process the image
            processed = self.process_cv_image(cv_image)
            
            # Display every 5th frame
            if self.compressed_count % 5 == 0:
                cv2.imshow('Filtering Grid', processed)
                cv2.waitKey(1)
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error (compressed): {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing compressed image: {e}')
    
    def image_callback(self, msg):
        """Process raw image messages"""
        try:
            # TODO: Convert ROS raw image to OpenCV (BGR8)
            # Hint: self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            self.processed_count += 1
            
            # TODO: Process the image
            processed = self.process_cv_image(cv_image)
            
            # Display every 5th frame
            if self.processed_count % 5 == 0:
                cv2.imshow('Filtering Grid', processed)
                cv2.waitKey(1)
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error (raw): {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing raw image: {e}')
    
    # -----------------------------
    # Image Processing
    # -----------------------------
    def process_cv_image(self, cv_image):
        """Resize, apply RGB/HSV filters, and stack in 2x2 grid"""

        new_width = 500
        height, width, _ = cv_image.shape
        aspect_ratio = height / width
        new_height = int(new_width * aspect_ratio)
        
        # Resize the image
        frame = cv2.resize(cv_image, (new_width, new_height))

        # CHOSE WHICH ONE YOU ARE USING RGB OR HSV

        # TODO: Apply RGB filters
        # results = self.apply_rgb_filters(frame)

        # TODO: Apply HSV filters
        results = self.apply_hsv_filters(frame)

        # TODO: Annotate the original image with text
        height, width = cv_image.shape[:2]
        now = self.get_clock().now()
        now_sec = now.nanoseconds / 1e9
        time_elapsed = (now - self.start_time).nanoseconds / 1e9
        
        # Text annotations
        frame_counter_text = f"Frame: {self.processed_count} (Compressed)"
        timestamp_text = f'ROS Time: {now_sec:.2f}s'
        time_elapsed_text = f"Elapsed: {time_elapsed:.2f}s"
        size_text = f"Size: {width}x{height}"
        
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1
        color = (200,200,200)
        thickness = 2
        margin = 10
        
        cv2.putText(cv_image, frame_counter_text, (margin, margin + 30), font, 2, (255,255, 255), thickness)
        cv2.putText(cv_image, size_text, (margin, margin + 60), font, font_scale, color, thickness)
        cv2.putText(cv_image, timestamp_text, (margin, margin + 90), font, font_scale, color, thickness)
        cv2.putText(cv_image, time_elapsed_text, (margin, margin + 120), font, font_scale, color, thickness)
        

        # TODO: Choose which filtered images to display in the grid
        # TODO: 2 x 2 grid display of original and 3 filtered images
        # Hint: np.hstack(image1, image2)
        grid = np.vstack([np.hstack([frame, results['red']['filtered']]),
                          np.hstack([results['green']['filtered'], results['blue']['filtered']])])
        
        return grid
    
    # -----------------------------
    # Color Filter Functions
    # -----------------------------
    def apply_rgb_filters(self, image):
        """Apply RGB color filtering (BGR in OpenCV)"""
        # Define BGR color ranges for red, green, and blue
        ranges = {
            'red': ([0, 0, 100], [100, 100, 255]), 
            'green': ([0, 100, 0], [100, 255, 100]), 
            'blue': ([100, 0, 0], [255, 100, 100])
        }

        results = {}
        for color, (lower, upper) in ranges.items():
            # Convert lists to NumPy arrays of type uint8
            lower_np = np.array(lower, dtype=np.uint8)
            upper_np = np.array(upper, dtype=np.uint8)

            # Create mask and apply bitwise_and
            mask = cv2.inRange(image, lower_np, upper_np)
            filtered = cv2.bitwise_and(image, image, mask=mask)

            results[color] = {'mask': mask, 'filtered': filtered}
        
        return results


    def apply_hsv_filters(self, image):
        """Apply HSV color filtering"""
        # Convert image from BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define HSV ranges for red, green, and blue
        # Note: Red needs two ranges because HSV wraps around
        ranges = {
            'red': [[np.array([0, 100, 100]), np.array([10, 255, 255])],
                    [np.array([170, 100, 100]), np.array([180, 255, 255])]],
            'green': [[np.array([40, 50, 50]), np.array([80, 255, 255])]],
            'blue': [[np.array([100, 150, 0]), np.array([140, 255, 255])]]
        }

        results = {}
        for color, bounds_list in ranges.items():
            mask = None
            for lower, upper in bounds_list:
                # Create mask for current range
                current_mask = cv2.inRange(hsv, lower, upper)
                if mask is None:
                    mask = current_mask
                else:
                    mask = cv2.bitwise_or(mask, current_mask)

            # Extract the color region from the original image
            filtered = cv2.bitwise_and(image, image, mask=mask)
            results[color] = {'mask': mask, 'filtered': filtered}

        return results
    

def main(args=None):
    rclpy.init(args=args)
    processor = ImageBagProcessor()
    
    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        pass
    finally:
        processor.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
