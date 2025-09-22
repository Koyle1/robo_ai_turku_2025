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

        self.y_amplitude = 0.30
        self.frequency_hz = 0.5
        self.publish_rate_hz = 30.0

        # precompute static rotation once for the x and z movement abouve the robot
        self.qx, self.qy, self.qz, self.qw = quaternion_from_euler(0,0,0)

        self.t0 = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._tick)

    def _tick(self):
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        y = self.y_amplitude * sin(tau * self.frequency_hz * t)

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'robot'
        tf.child_frame_id  = 'scanner'

        tf.transform.translation.x = 0.80
        tf.transform.translation.y = y
        tf.transform.translation.z = 0.35  

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