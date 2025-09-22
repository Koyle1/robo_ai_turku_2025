import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
class TFBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.get_logger().info('Node started!')
    
    def timer_callback(self):

        t = self.get_clock().now().seconds_nanoseconds()[0] + \
        self.get_clock().now().seconds_nanoseconds()[1] / 1e9
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = 'map' 
        transform_stamped.child_frame_id = 'robot'

        # here is already the more compilcated movement implemented
        R = 2.0
        w = 0.4

        x = R * math.cos(w * t)
        y = R * math.sin(w * t) * math.cos(w * t)
        dx = -R * w * math.sin(w * t)
        dy =  R * w * (math.cos(2*w*t))
        yaw = math.atan2(dy, dx)

        transform_stamped.transform.translation.x = x
        transform_stamped.transform.translation.y = y
        transform_stamped.transform.translation.z = 0.0

        q = quaternion_from_euler(0.0, 0.0, yaw)
        transform_stamped.transform.rotation.x = q[0]
        transform_stamped.transform.rotation.y = q[1]
        transform_stamped.transform.rotation.z = q[2]
        transform_stamped.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(transform_stamped)

def main(args=None):
    rclpy.init(args=args)
    tf_broadcaster = TFBroadcaster()
    rclpy.spin(tf_broadcaster)
    tf_broadcaster.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()