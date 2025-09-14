import rclpy 
from rclpy.node import Node  
from geometry_msgs.msg import PoseStamped as Ps
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy 
class MinimalPublisher(Node): 
    def __init__(self): 
        super().__init__('publisher') 
        topic_name = 'task1_topic' 
        self.qos_profile = QoSProfile( 
            reliability=ReliabilityPolicy.RELIABLE,         
            durability=DurabilityPolicy.TRANSIENT_LOCAL,    
            history=HistoryPolicy.KEEP_LAST,                
            depth=10                                         
        ) 
        self.publisher_ = self.create_publisher( 
            Ps, 
            topic_name, 
            self.qos_profile 
            ) 
        
        self.position = [0.0, 0.0, 0.0]
        
        timer_period = 2
        self.timer = self.create_timer(timer_period, 
                                        self.timer_callback) 
        
        self.get_logger().info(f'Publisher node has been started, publishing to {topic_name} topic.')
        
    def timer_callback(self): 
        self.move_straight() 
        msg = Ps()  
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # position update
        msg.pose.position.x,msg.pose.position.y, msg.pose.position.z = self.position
        
        # orientation update
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = 0.0, 0.0, 0.0, 1.0
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg}')
        
    def move_straight(self): 
        self.position[0] += 0.5  # Move 0.5 units along the x-axis
        self.get_logger().info(f'Updated position: {self.position}')
        
def main(args=None): 
    rclpy.init(args=args) 
    minimal_publisher = MinimalPublisher() 
    rclpy.spin(minimal_publisher) 
    minimal_publisher.destroy_node() 
    rclpy.shutdown() 
    
if __name__ == '__main__': 
    main()