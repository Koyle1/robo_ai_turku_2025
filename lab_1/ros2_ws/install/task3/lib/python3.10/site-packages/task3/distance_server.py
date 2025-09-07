import math
import rclpy
from rclpy.node import Node
from custom_interface.srv import CalculateDistance  # <-- your custom srv

class DistanceServerNode(Node):

    def __init__(self):
        super().__init__("distance_server")
        # service name kept simple and explicit
        self.server_ = self.create_service(
            CalculateDistance,
            "/calculate_distance",
            self.distance_service_callback
        )
        self.get_logger().info("Distance Service server has been created")

    def distance_service_callback(self, request, response):
        # request.p1 and request.p2 are geometry_msgs/Point
        dx = request.p2.x - request.p1.x
        dy = request.p2.y - request.p1.y
        dist = math.hypot(dx, dy)

        response.distance = float(dist)
        response.success = True
        response.message = (
            f"Distance between ({request.p1.x:.3f},{request.p1.y:.3f}) and "
            f"({request.p2.x:.3f},{request.p2.y:.3f}) is {dist:.3f}"
        )

        self.get_logger().info(response.message)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = DistanceServerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()