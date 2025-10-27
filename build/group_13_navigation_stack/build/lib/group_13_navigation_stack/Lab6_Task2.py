import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
import numpy as np
import heapq
import math

class SimplePathFollower(Node):
    def __init__(self):
        super().__init__('simple_path_follower_node')

        # Publisher for the path (list of waypoints)
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=25,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.path_publisher = self.create_publisher(Point, '/path', qos_profile)

        # Adjacency matrix for 6 points (predefined waypoints)
        self.adjacency_matrix = np.array([
        # 0  1  2  3  4  5  6  7  8  9 10
        [0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0],  # 0 (0.0, 0.0)
        [1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0],  # 1 (1.5, -1.5)
        [0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0],  # 2 (1.5, 3.0)
        [0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1],  # 3 (-0.8, 3.0)
        [1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0],  # 4 (-0.5, 0.5)
        [0, 1, 0, 0, 1, 0, 1, 0, 1, 1, 0],  # 5 (-2.2, 0.0)
        [0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0],  # 6 (-3.0, -2.0)
        [0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0],  # 7 (-4.2, -2.2)
        [0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0],  # 8 (-4.2, -0.15)
        [0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1],  # 9 (-2.8, 0.5)
        [0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0]   #10 (-2.8, 3.0)
        ])

        # Define waypoints (positions for each node)
        self.waypoints = [
            (0.0, 0.0),     # 0
            (1.5, -1.5),    # 1
            (1.5, 3.0),     # 2
            (-0.8, 3.0),    # 3
            (-0.5, 0.5),    # 4
            (-2.2, 0.0),    # 5
            (-3.0, -2.0),   # 6
            (-4.2, -2.2),   # 7
            (-4.2, -0.15),  # 8
            (-2.8, 0.5),    # 9
            (-2.8, 3.0)     # 10
        ]


        # Define start and end points (index for simplicity)
        self.start = 2  # Node 0 (0, 0)
        self.end = 7    # Node 5 (1.2, 1.7)

        # Find and publish the shortest path
        self.find_and_publish_path()

    def dijkstra(self, start, end):
        """Find the shortest path from start to end using Dijkstra's algorithm."""
        num_nodes = len(self.adjacency_matrix)
        
        # Initialize the distances to all nodes as infinity
        distances = {node: float('inf') for node in range(num_nodes)}
        distances[start] = 0
        
        # Priority queue for nodes to visit (min-heap)
        queue = [(0, start)]  # (distance, node)
        
        # Previous node tracking for path reconstruction
        previous_nodes = {node: None for node in range(num_nodes)}
        
        while queue:
            current_distance, current_node = heapq.heappop(queue)
            
            if current_node == end:
                break
            
            for neighbor in range(num_nodes):
                if self.adjacency_matrix[current_node][neighbor] == 1:
                    # Use Euclidean distance between waypoints as edge weight
                    edge_cost = math.dist(self.waypoints[current_node], self.waypoints[neighbor])
                    distance = current_distance + edge_cost
                    
                    # If a shorter path to the neighbor is found, update it
                    if distance < distances[neighbor]:
                        distances[neighbor] = distance
                        previous_nodes[neighbor] = current_node
                        heapq.heappush(queue, (distance, neighbor))
        
        # Reconstruct the path from end to start
        path = []
        current_node = end
        while current_node is not None:
            path.insert(0, current_node)
            current_node = previous_nodes[current_node]
        
        total_cost = distances[end]
        return path, total_cost

    def publish_path(self, path):
        """Publish the path as a sequence of waypoints."""
        for node in path:
            point_msg = Point()
            point_msg.x = self.waypoints[node][0]
            point_msg.y = self.waypoints[node][1]
            point_msg.z = 0.0  # Assume z is always 0 for 2D path
            self.path_publisher.publish(point_msg)
            self.get_logger().info(f"Publishing waypoint: {self.waypoints[node]}")

    def find_and_publish_path(self):
        """Find the shortest path and publish it."""
        path, total_cost = self.dijkstra(self.start, self.end)
        
        if path:
            self.get_logger().info(f"Path found: {path}")
            self.get_logger().info(f"Total path cost: {total_cost:.3f}")
            self.publish_path(path)
        else:
            self.get_logger().error("No path found!")

def main(args=None):
    rclpy.init(args=args)
    node = SimplePathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
