#!/usr/bin/env python3
"""
Task 2: ROS2 Image Processing from Bag - Fully Updated
Read images from a ROS bag, annotate them, save, and display in real-time
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import os


class ImageBagProcessor(Node):
    def __init__(self):
        super().__init__('image_bag_processor')
        self.bridge = CvBridge()

        # Define the QoS profile for best effort
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # Subscribe to uncompressed images
        subscriber_topic = 'camera/image_raw/compressed'
        self.subscription = self.create_subscription(
            CompressedImage,
            subscriber_topic,
            self.image_callback,
            qos
        )
        
        # Create folder for saving processed images if it doesn't exist
        self.save_dir = 'processed_images/task2'
        os.makedirs(self.save_dir, exist_ok=True)

        self.processed_count = 0
        self.start_time = self.get_clock().now()
        self.get_logger().info(f'Subscribed to {subscriber_topic} with best effort QoS')

    def image_callback(self, msg):
        """Process uncompressed image messages"""
        try:
            # Convert ROS Image to OpenCV image
            img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Process the image (add shapes and text)
            processed_image = self.process_cv_image(img)
            self.processed_count += 1
            
            # Save every 30th frame
            if self.processed_count % 30 == 0:
                filename = os.path.join(self.save_dir, f'task2_{self.processed_count:04d}.png')
                cv2.imwrite(filename, processed_image)
                self.get_logger().info(f"Saved {filename}")

            # Display the image in a window
            cv2.imshow('Processed Image', processed_image)
            cv2.waitKey(1)  # Needed for OpenCV to update the window
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    
    def process_cv_image(self, cv_image):
        """Add annotations to the image"""
        cv_image = cv_image.copy()
        
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
        
        return cv_image

def main(args=None):
    rclpy.init(args=args)
    
    processor = ImageBagProcessor()
    
    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        pass
    
    processor.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
