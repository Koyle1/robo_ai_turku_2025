import rclpy 
from rclpy.node import Node 
from custom_interface.msg import TaskMsg as Cm
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy 

class StudentSubscriber(Node): 
    def __init__(self): 
        super().__init__('subscriber') 
        topic_name = 'task2_topic' 
        self.qos_profile = QoSProfile( 
            reliability=ReliabilityPolicy.RELIABLE,         
            durability=DurabilityPolicy.TRANSIENT_LOCAL,    
            history=HistoryPolicy.KEEP_LAST,                 
            depth=10            
        ) 
        self.subscription = self.create_subscription( 
            Cm, 
            topic_name, 
            self.listener_callback, 
            self.qos_profile) 
        self.subscription 
        
    def listener_callback(self, msg): 
        self.get_logger().info(f'Messafe received at {msg.header.stamp.sec} | name: {msg.name} | age: {msg.age} | is_student: {msg.is_student}') 

def main(args=None): 
    rclpy.init(args=args) 
    movement_subscriber = StudentSubscriber() 
    rclpy.spin(movement_subscriber) 
    movement_subscriber.destroy_node() 
    rclpy.shutdown() 
    
if __name__ == '__main__': 
    main()