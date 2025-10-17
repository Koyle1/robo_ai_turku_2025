#! /usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan

from matplotlib import pyplot as plt

class FeatureExtracter(Node):
    def __init__(self):
        super().__init__('feature_extracter')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1) 
        self.scan_idx = 0
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile=qos_policy)
        self.publisher_ = self.create_publisher(LaserScan, '/feature_scan', 10)
        self.front_pub = self.create_publisher(LaserScan, '/front_scan', 10)

        # create a new publisher for /front_scan


    def scan_callback(self,msg):

        ranges = []
        for r in msg.ranges:
            if r > 2.5 or r < 1:   # Feature extraction code here
                ranges.append(0.0)
            else:
                ranges.append(r)

        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = msg.header.frame_id
        scan.angle_min = msg.angle_min
        scan.angle_max = msg.angle_max
        scan.angle_increment = msg.angle_increment
        scan.time_increment = msg.time_increment
        scan.range_min = msg.range_min
        scan.range_max = msg.range_max 
        scan.ranges = ranges
        self.publisher_.publish(scan)

        self.scan_idx += 1
        
        ranges_np = np.array(msg.ranges, dtype=float)
        n = len(ranges_np)
        angles = msg.angle_min + np.arange(n) * msg.angle_increment


        # Make a boolean mask for angles in [-60°, +60°]
        mask = (angles >= np.deg2rad(-60.0)) & (angles <= np.deg2rad(60.0))


        # Keep in-FOV points; set others to inf so LaserScan length stays identical
        front_ranges = np.where(mask, ranges_np, float('inf'))


        front_scan = LaserScan()
        front_scan.header = msg.header
        front_scan.angle_min = msg.angle_min
        front_scan.angle_max = msg.angle_max
        front_scan.angle_increment = msg.angle_increment
        front_scan.time_increment = msg.time_increment
        front_scan.scan_time = msg.scan_time
        front_scan.range_min = msg.range_min
        front_scan.range_max = msg.range_max
        front_scan.ranges = front_ranges.tolist()

        self.front_pub.publish(front_scan)

        print("Publish feature scan message idx", self.scan_idx)



def main(args=None):
    rclpy.init(args=args)
    feature_extracter = FeatureExtracter()
    rclpy.spin(feature_extracter) 
    feature_extracter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
