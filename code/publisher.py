import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('publisher')
        topic_name = '/----' # TODO: fill with the name of the topic

        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,         # Ensure delivery of messages
            durability=DurabilityPolicy.TRANSIENT_LOCAL,    # Keep messages even if the publisher dies
            history=HistoryPolicy.KEEP_LAST,                # Only store the last N messages
            depth=10                                        # The number of messages to store (N=10)
        )

        self.publisher_ = self.create_publisher(
            String,
            topic_name,
            self.qos_profile
            )
        
        timer_period = # TODO:      # time in seconds
        self.timer = self.create_timer(timer_period,
                                        self.timer_callback)

        self.i = 0


    def timer_callback(self):
        msg = String() 
        msg.data = f''# TODO: fill out this line so using the counter provided in the constructor.
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
