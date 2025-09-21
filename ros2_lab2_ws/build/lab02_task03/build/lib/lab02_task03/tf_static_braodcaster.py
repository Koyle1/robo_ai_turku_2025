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

        # fixed setup (same style: no parameters)
        self.parent_frame = 'robot'
        self.child_frame  = 'lidar'
        self.x = 0.30   # forward
        self.y = 0.00   # left
        self.z = 0.15   # up
        self.roll_deg  = 0.0
        self.pitch_deg = 0.0
        self.yaw_deg   = 0.0

    def _once(self):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.parent_frame
        tf.child_frame_id  = self.child_frame
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = self.z

        qx, qy, qz, qw = quaternion_from_euler(
            radians(self.roll_deg), radians(self.pitch_deg), radians(self.yaw_deg)
        )
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        self.br.sendTransform(tf)
        self.timer.cancel()  # publish once

def main(args=None):
    rclpy.init(args=args)
    node = TFStaticBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()