import rclpy
from rclpy.node import Node
from custom_interface.srv import CalculateDistance
from geometry_msgs.msg import Point
from functools import partial

class DistanceClientNode(Node):

    def __init__(self):
        super().__init__('distance_client')
        self.get_logger().info('Distance Client Python node has been created')

        self.declare_parameter('x1', 0.0)
        self.declare_parameter('y1', 0.0)
        self.declare_parameter('x2', 3.0)
        self.declare_parameter('y2', 4.0)

        x1 = float(self.get_parameter('x1').value)
        y1 = float(self.get_parameter('y1').value)
        x2 = float(self.get_parameter('x2').value)
        y2 = float(self.get_parameter('y2').value)

        p1 = Point(x=x1, y=y1, z=0.0)
        p2 = Point(x=x2, y=y2, z=0.0)

        self.call_distance_server(p1, p2)

    def call_distance_server(self, p1: Point, p2: Point):
        client = self.create_client(CalculateDistance, '/calculate_distance')
        while not client.wait_for_service(1.0):
            self.get_logger().info('Waiting for the /calculate_distance service...')

        request = CalculateDistance.Request()
        request.p1 = p1
        request.p2 = p2

        future = client.call_async(request)
        future.add_done_callback(partial(self.distance_service_callback, p1=p1, p2=p2))

    def distance_service_callback(self, future, p1: Point, p2: Point):
        try:
            response = future.result()
            self.get_logger().info(
                f"Request: p1=({p1.x:.3f},{p1.y:.3f}), p2=({p2.x:.3f},{p2.y:.3f}) "
                f"-> success={response.success}, distance={response.distance:.3f}, msg='{response.message}'"
            )
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = DistanceClientNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()