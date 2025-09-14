import rclpy 
from rclpy.node import Node 
from geometry_msgs.msg import PoseStamped as Ps
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy 

class MovementSubscriber(Node): 
    def __init__(self): 
        super().__init__('subscriber') 
        topic_name = 'task1_topic' 
        self.qos_profile = QoSProfile( 
            reliability=ReliabilityPolicy.RELIABLE,         
            durability=DurabilityPolicy.TRANSIENT_LOCAL,    
            history=HistoryPolicy.KEEP_LAST,                 
            depth=10            
        ) 
        self.subscription = self.create_subscription( 
            Ps, 
            topic_name, 
            self.listener_callback, 
            self.qos_profile) 
        self.subscription 
        
    def listener_callback(self, msg): 
        self.get_logger().info(f'New position: ({msg.pose.position.x}|{msg.pose.position.y})') 

def main(args=None): 
    rclpy.init(args=args) 
    movement_subscriber = MovementSubscriber() 
    rclpy.spin(movement_subscriber) 
    movement_subscriber.destroy_node() 
    rclpy.shutdown() 
    
if __name__ == '__main__': 
    main()