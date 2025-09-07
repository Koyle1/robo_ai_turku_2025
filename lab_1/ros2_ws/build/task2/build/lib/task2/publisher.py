import rclpy
from rclpy.node import Node
from custom_interface.msg import Person                     
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('publisher')
        topic_name = '/person'                            

        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.publisher_ = self.create_publisher(Person, topic_name, self.qos_profile)

        timer_period = 1.0 
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.age = 10
        self.increment = 1

    def timer_callback(self):
        msg = Person()
        msg.header.stamp = self.get_clock().now().to_msg() 
        msg.name = "Michael"
        msg.age = self.age
        msg.is_student = True

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: name={msg.name}, age={msg.age}, is_student={msg.is_student}')

        # keep age in [10, 18]
        self.age += self.increment
        if self.age > 17 or self.age < 11:
            self.increment *= -1

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()