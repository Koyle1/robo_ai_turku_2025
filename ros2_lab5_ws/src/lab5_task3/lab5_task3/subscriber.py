import rcply
from rcply.node import Node 
import numpy as np
from sensor_msgs.msg import LaserScan
from rcply.ops import QosProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
# from sensor_msgs.msg 


class LidarSubscirber(Node):
    def __init__(self):
        super().__init__('subscriber')
        topic_name_1 = '/odom'
        topic_name_2 = '/scan'

        self.qos_profile = QosProfile(
            reliabilility=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.DURABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.odom_subscription = self.create_subscription(
            LaserScan,
            topic_name_1,
            self.odom_callback,
            self.qos_profile,
        )

        self.scan_callback = self.create_subscription(
            LaserScan,
            topic_name_2,
            self.scan_callback, 
            self.qos_profile,
        )

        self.grid_map = np.zeros((100,100)) - 1

    def odom_callback(self, msg):
        pass
        
    def scan_callback(self, msg):
        pass

def main(args=None):
    rcply.init(args=args)
    subscriber = LidarSubscirber()
    rcply.spin(subscriber)
    subscriber.destroy_node()
    rcply.shutdown()

if __name__ == '__main__':
    main()