import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from tf_transformations import quaternion_from_euler

class Static_TFBroadcaster(Node):
    """
    Broadcasts a static transform from the moving robot frame to a lidar frame.
    """
    def __init__(self):
        super().__init__('Static_TFBroadcaster')
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.get_logger().info('Node started, will only publish one')

        self.declare_parameter('parent_frame', 'robot')
        self.declare_parameter('child_frame',  'lidar')
        self.declare_parameter('x', 0.30)   # X forward (m)
        self.declare_parameter('y', 0.0)    # Y left (m)
        self.declare_parameter('z', 0.15)   # Z up (m)
        self.declare_parameter('roll_deg',  0.0)
        self.declare_parameter('pitch_deg', 0.0)
        self.declare_parameter('yaw_deg',   0.0)

    def timer_callback(self):
        """
        Onetime call this to proadcast the static information
        """
        # get values from Parameters
        parent = self.get_parameter('parent_frame').value
        child  = self.get_parameter('child_frame').value
        x = float(self.get_parameter('x').value)
        y = float(self.get_parameter('y').value)
        z = float(self.get_parameter('z').value)
        roll  = radians(float(self.get_parameter('roll_deg').value))
        pitch = radians(float(self.get_parameter('pitch_deg').value))
        yaw   = radians(float(self.get_parameter('yaw_deg').value))


        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = parent
        transform_stamped.child_frame_id = child

        # Set translation
        transform_stamped.transform.translation.x = x
        transform_stamped.transform.translation.y = y
        transform_stamped.transform.translation.z = z

        # Set rotation using roll, pitch, and yaw
        q = quaternion_from_euler(roll, pitch, yaw)

        transform_stamped.transform.rotation.x = q[0]
        transform_stamped.transform.rotation.y = q[1]
        transform_stamped.transform.rotation.z = q[2]
        transform_stamped.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(transform_stamped)
        self.get_logger().info(f"Published static TF {parent} -> {child} at ({x}, {y}, {z})")
        #publish only once = cancel the time
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    tf_broadcaster = TFBroadcaster()
    rclpy.spin(tf_broadcaster)
    tf_broadcaster.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()