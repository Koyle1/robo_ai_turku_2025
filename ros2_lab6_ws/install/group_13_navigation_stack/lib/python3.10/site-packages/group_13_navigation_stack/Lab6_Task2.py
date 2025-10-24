import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import numpy as np
import heapq

class SimplePathFollower(Node):
    def __init__(self):
        super().__init__('simple_path_follower_node')

        # Publisher for the path (list of waypoints)
        self.path_publisher = self.create_publisher(Point, '/path', 10)

        # Adjacency matrix for 6 points (predefined waypoints)
        self.adjacency_matrix = np.array([
            [0, 1, 0, 1, 1, 0],  # Node 0
            [1, 0, 1, 0, 0, 0],  # Node 1
            [0, 1, 0, 1, 0, 0],  # Node 2
            [1, 0, 1, 0, 1, 0],  # Node 3
            [1, 0, 0, 1, 0, 1],  # Node 4
            [0, 0, 0, 0, 1, 0]   # Node 5
        ])

        # Define the waypoints corresponding to each node
        self.waypoints = [
            (0.0, 0.0),  # Node 0
            (1.3, 0.0),  # Node 1
            (1.3, -1.4), # Node 2
            (-0.3, -1.4),# Node 3
            (-0.3, 1.7), # Node 4
            (1.2, 1.7)   # Node 5
        ]

        # Define start and end points (index for simplicity)
        self.start = 0  # Node 0 (0, 0)
        self.end = 5    # Node 5 (1.2, 1.7)

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
                    # Calculate the distance to the neighbor
                    distance = current_distance + 1  # Assumed constant edge weight of 1
                    
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
        
        return path

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
        # Find the shortest path using Dijkstra's algorithm
        path = self.dijkstra(self.start, self.end)
        
        if path:
            self.get_logger().info(f"Path found: {path}")
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
