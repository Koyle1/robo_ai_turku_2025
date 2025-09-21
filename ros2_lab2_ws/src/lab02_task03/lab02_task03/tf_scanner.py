#!/usr/bin/env python3
from math import sin, tau
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler

class TFScanner(Node):
    def __init__(self):
        super().__init__('tf_scanner')
        self.br = TransformBroadcaster(self)

        # fixed setup (no parameters)
        self.parent_frame = 'robot'
        self.child_frame  = 'scanner'
        self.x_offset = 0.80    # forward
        self.z_offset = 0.35    # up
        self.roll = 0.0         # rad
        self.pitch = 0.0        # rad
        self.yaw = 0.0          # rad
        self.y_amplitude = 0.30 # ±30 cm
        self.frequency_hz = 0.5
        self.publish_rate_hz = 30.0

        # precompute static rotation once
        self.qx, self.qy, self.qz, self.qw = quaternion_from_euler(self.roll, self.pitch, self.yaw)

        self.t0 = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._tick)

    def _tick(self):
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        y = self.y_amplitude * sin(tau * self.frequency_hz * t)

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.parent_frame
        tf.child_frame_id  = self.child_frame

        tf.transform.translation.x = self.x_offset
        tf.transform.translation.y = y
        tf.transform.translation.z = self.z_offset

        tf.transform.rotation.x = self.qx
        tf.transform.rotation.y = self.qy
        tf.transform.rotation.z = self.qz
        tf.transform.rotation.w = self.qw

        self.br.sendTransform(tf)

def main(args=None):
    rclpy.init(args=args)
    node = TFScanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()