#!/usr/bin/env python3
import math
from math import sin, tau
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from tf_transformations import quaternion_from_euler

class ScannerBroadcaster(Node): 

    def __init__(self):
        super().__init__('scanner_broadcaster')

        #set params for the task3
        self.parent_frame = 'robot'
        self.child_frame  = 'scanner'
        self.x_offset     = 0.80    # m (forward)
        self.z_offset     = 0.35    # m (up)
        self.roll         = 0.0     # rad
        self.pitch        = 0.0     # rad
        self.yaw          = 0.0     # rad
        self.publish_rate_hz = 30.0
        # ---- Only two parameters exposed ----
        self.declare_parameter('y_amplitude', 0.30)   # meters (±30 cm)
        self.declare_parameter('frequency_hz', 0.5)   # Hz

        self.Ay   = float(self.get_parameter('y_amplitude').value)
        self.freq = float(self.get_parameter('frequency_hz').value)

        # Precompute static quaternion once (from fixed RPY)
        qx, qy, qz, qw = quaternion_from_euler(
            self.roll, self.pitch, self.yaw
        )
        self.static_quat = (qx, qy, qz, qw)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.t0 = self.get_clock().now()

        period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            f"Scanner broadcaster: {self.parent_frame} -> {self.child_frame} | "
            f"x={self.x_offset} z={self.z_offset} y=±{self.Ay} @ {self.freq} Hz"
        )

    def timer_callback(self):
        # elapsed time (s)
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        y = self.Ay * sin(tau * self.freq * t)

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.parent_frame
        tf.child_frame_id  = self.child_frame

        tf.transform.translation.x = self.x_offset
        tf.transform.translation.y = y
        tf.transform.translation.z = self.z_offset

        #do a static transformation for the ozillationg scanner, since the scanner is fix on the robot
        qx, qy, qz, qw = self.static_quat
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf)

def main(args=None):
    rclpy.init(args=args)
    node = ScannerBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()