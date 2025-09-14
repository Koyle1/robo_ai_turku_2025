import rclpy 
from rclpy.node import Node  
from custom_interface.msg import TaskMsg as Cm
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy 
from numpy import random as r
class StudentPublisher(Node): 
    def __init__(self): 
        super().__init__('publisher') 
        topic_name = 'task2_topic' 
        self.qos_profile = QoSProfile( 
            reliability=ReliabilityPolicy.RELIABLE,         
            durability=DurabilityPolicy.TRANSIENT_LOCAL,    
            history=HistoryPolicy.KEEP_LAST,                
            depth=10                                         
        ) 
        self.publisher_ = self.create_publisher( 
            Cm, 
            topic_name, 
            self.qos_profile 
            ) 
        

        timer_period = 2
        self.timer = self.create_timer(timer_period, 
                                        self.timer_callback) 
        
        self.get_logger().info(f'Publisher node has been started, publishing to {topic_name} topic.')
        
    def timer_callback(self): 
        msg = Cm()  
        
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.name = "Felix"
        msg.age = r.randint(10,18)
        msg.is_student = True
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg}')

        
def main(args=None): 
    rclpy.init(args=args) 
    movement_publisher = StudentPublisher() 
    rclpy.spin(movement_publisher) 
    movement_publisher.destroy_node() 
    rclpy.shutdown() 
    
if __name__ == '__main__': 
    main()