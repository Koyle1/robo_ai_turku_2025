import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('publisher')
        topic_name = '/pose'

        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,   # live data → VOLATILE
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.publisher_ = self.create_publisher(
            PoseStamped,
            topic_name,
            self.qos_profile
        )

        timer_period = 0.1 
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # straight-line motion state
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.1  
        self.vy = 0.1  
        self.last_time = self.get_clock().now()

    def timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # update position
        self.x += self.vx * dt
        self.y += self.vy * dt

        msg = PoseStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0 

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing Pose: x={self.x:.3f}, y={self.y:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()