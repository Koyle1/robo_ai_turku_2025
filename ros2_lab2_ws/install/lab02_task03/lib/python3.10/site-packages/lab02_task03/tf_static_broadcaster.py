#!/usr/bin/env python3
from math import radians
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from tf_transformations import quaternion_from_euler

class TFStaticBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_static_broadcaster')
        self.br = StaticTransformBroadcaster(self)
        self.timer = self.create_timer(0.01, self._once)

    def _once(self):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'robot'
        tf.child_frame_id  = 'lidar'
        tf.transform.translation.x = 0.30
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.15

        qx, qy, qz, qw = quaternion_from_euler(
            radians(0.0), radians(0.0), radians(0.0)
        )
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        self.br.sendTransform(tf)
        self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = TFStaticBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()