import rclpy 
from rclpy.node import Node 
from std_msgs.msg import String 
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy 
class MinimalPublisher(Node): 
    def __init__(self): 
        super().__init__('publisher') 
        topic_name = 'test' 
        self.qos_profile = QoSProfile( 
            reliability=ReliabilityPolicy.RELIABLE,         
            durability=DurabilityPolicy.TRANSIENT_LOCAL,    
            history=HistoryPolicy.KEEP_LAST,                
            depth=10                                         
        ) 
        self.publisher_ = self.create_publisher( 
            String, 
            topic_name, 
            self.qos_profile 
            ) 
        timer_period = 2
        self.timer = self.create_timer(timer_period, 
                                        self.timer_callback) 
        self.i = 0 
    def timer_callback(self): 
        msg = String()  
        msg.data = f'{self.timer._clock} seconds after last call.' 
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