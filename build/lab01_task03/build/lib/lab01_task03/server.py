import rclpy
from rclpy.node import Node
from custom_interface.srv import CalculateDistance as Cd
from geometry_msgs.msg import Point
from math import sqrt, pow

class DistanceServer(Node):
    def __init__(self):
        super().__init__('distance_server')
        self.server_ = self.create_service(Cd, 'distance_calculator', self.custom_callback)
        
        self.get_logger().info('Server has been created')
        
    def custom_callback(self, request, response):
        point_start = request.start_point
        point_end = request.end_point
        try:
            distance = sqrt(pow((point_end.x - point_start.x), 2) + pow((point_end.y - point_start.y), 2) + pow((point_end.z - point_start.z), 2))
            response.distance = distance
            response.success = True
            response.message = 'Distance calculated successfully'
            
        except Exception as e:
            response.distance = -1.0
            response.success = False
            response.message = f'Error occurred: {e}'
            
        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = DistanceServer()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()