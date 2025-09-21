import rclpy
from custom_interface.srv import CalculateDistance as Cd
from rclpy.node import Node
from geometry_msgs.msg import Point
from functools import partial 
from numpy import random as r

class DistanceClient(Node):
    def __init__(self):
        super().__init__('distance_client')
        self.get_logger().info('Client has been created')
        
        # Declare parameters (nested keys are flattened using dots)
        self.declare_parameter("start_point.x", r.uniform(-2,10))
        self.declare_parameter("start_point.y", r.uniform(-2,10))
        self.declare_parameter("start_point.z", r.uniform(-2,10))

        self.declare_parameter("end_point.x", r.uniform(-2,10))
        self.declare_parameter("end_point.y", r.uniform(-2,10))
        self.declare_parameter("end_point.z", r.uniform(-2,10))

        # Read parameters and create Point messages
        self.start_point = Point(
            x=self.get_parameter("start_point.x").value,
            y=self.get_parameter("start_point.y").value,
            z=self.get_parameter("start_point.z").value,
        )

        self.end_point = Point(
            x=self.get_parameter("end_point.x").value,
            y=self.get_parameter("end_point.y").value,
            z=self.get_parameter("end_point.z").value,
        )

        self.get_logger().info(f"Start point: {self.start_point}")
        self.get_logger().info(f"End point: {self.end_point}")
        
        self.send_request()
                
    def set_random_points(self):
        self.start_point.x = r.uniform(-2,10)
        self.start_point.y = r.uniform(-2,10)
        self.start_point.z = r.uniform(-2,10)
        
        self.end_point.x = r.uniform(-2,10)
        self.end_point.y = r.uniform(-2,10)
        self.end_point.z = r.uniform(-2,10)
        
    def send_request(self):
        client = self.create_client(Cd, 'distance_calculator')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting...')
        
        request = Cd.Request()
        request.start_point = self.start_point
        request.end_point = self.end_point
        
        future = client.call_async(request)
        future.add_done_callback(partial(self.custom_callback))
        
    def custom_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'success: {response.success}')
                self.get_logger().info(f'message: {response.message}')
                self.get_logger().info(f'Distance between points ({self.start_point.x}, {self.start_point.y}, {self.start_point.z}) and ({self.end_point.x}, {self.end_point.y}, {self.end_point.z}) is {response.distance}')
            else:
                self.get_logger().info(f'success: {response.success}')
                self.get_logger().info(f'message: {response.message}')
        except Exception as e:
            self.get_logger().info(f'Service call failed: {e}')
            
def main(args=None):
    rclpy.init(args=args)
    node = DistanceClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
            
        