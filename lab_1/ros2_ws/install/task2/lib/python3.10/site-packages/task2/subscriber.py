import rclpy
from rclpy.node import Node
from rclpy.time import Time
from custom_interface.msg import Person
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('subscriber')
        topic_name = '/person'

        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            Person, topic_name, self.listener_callback, self.qos_profile
        )

        self.prev_stamp = None

    def listener_callback(self, msg: Person):
        t_now = Time.from_msg(msg.header.stamp)

        hz = 0.0
        if self.prev_stamp is not None:
            dt = (t_now - self.prev_stamp).nanoseconds * 1e-9
            if dt > 0:
                hz = 1.0 / dt
        self.prev_stamp = t_now

        self.get_logger().info(
            f"Received: name={msg.name}, age={msg.age}, is_student={msg.is_student} | ~{hz:.1f} Hz"
        )

def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()