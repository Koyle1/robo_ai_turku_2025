import rclpy 
from rclpy.node import Node 
from std_msgs.msg import String 
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy 

class MinimalSubscriber(Node): 
    def __init__(self): 
        super().__init__('subscriber') 
        topic_name = 'lab1_topic' 
        self.qos_profile = QoSProfile( 
            reliability=ReliabilityPolicy.RELIABLE,         
            durability=DurabilityPolicy.TRANSIENT_LOCAL,    
            history=HistoryPolicy.KEEP_LAST,                 
            depth=10            
        ) 
        self.subscription = self.create_subscription( 
            String, 
            topic_name, 
            self.listener_callback, 
            self.qos_profile) 
        self.subscription 
        
    def listener_callback(self, msg): 
        self.get_logger().info(f'I heard: {msg.data}') 

def main(args=None): 
    rclpy.init(args=args) 
    minimal_subscriber = MinimalSubscriber() 
    rclpy.spin(minimal_subscriber) 
    minimal_subscriber.destroy_node() 
    rclpy.shutdown() 
    
if __name__ == '__main__': 
    main()