import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
import numpy as np
import heapq
import math


class PathFinderNode(Node):
    def __init__(self):
        super().__init__('path_finder_node')

        # Publisher for the path (list of waypoints)
        self.path_publisher = self.create_publisher(Point, '/path', 10)
        
        # Publisher for movement commands (Twist)
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

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

        # Find and publish the shortest path using A*
        self.find_and_publish_path()

    def a_star(self, start, end):
        """Find the shortest path from start to end using A* algorithm."""
        num_nodes = len(self.adjacency_matrix)

        # Initialize the g and f scores
        g_scores = {node: float('inf') for node in range(num_nodes)}
        g_scores[start] = 0
        
        f_scores = {node: float('inf') for node in range(num_nodes)}
        f_scores[start] = self.euclidean_distance(self.waypoints[start], self.waypoints[end])

        # Priority queue for nodes to visit (min-heap)
        open_set = [(f_scores[start], start)]  # (f_score, node)
        
        # Previous node tracking for path reconstruction
        previous_nodes = {node: None for node in range(num_nodes)}
        
        expanded_nodes = 0  # For counting expanded nodes

        while open_set:
            _, current_node = heapq.heappop(open_set)
            expanded_nodes += 1  # Increment the expanded node count
            
            if current_node == end:
                break

            for neighbor in range(num_nodes):
                if self.adjacency_matrix[current_node][neighbor] == 1:
                    tentative_g_score = g_scores[current_node] + 1  # Constant edge cost
                    
                    if tentative_g_score < g_scores[neighbor]:
                        previous_nodes[neighbor] = current_node
                        g_scores[neighbor] = tentative_g_score
                        f_scores[neighbor] = g_scores[neighbor] + self.euclidean_distance(self.waypoints[neighbor], self.waypoints[end])
                        heapq.heappush(open_set, (f_scores[neighbor], neighbor))
        
        # Reconstruct the path from end to start
        path = []
        current_node = end
        while current_node is not None:
            path.insert(0, current_node)
            current_node = previous_nodes[current_node]
        
        return path, expanded_nodes

    def euclidean_distance(self, point1, point2):
        """Calculate the Euclidean distance between two points."""
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

    def publish_path(self, path):
        """Publish the path as a sequence of waypoints."""
        for node in path:
            point_msg = Point()
            point_msg.x = self.waypoints[node][0]
            point_msg.y = self.waypoints[node][1]
            point_msg.z = 0.0  # Assume z is always 0 for 2D path
            self.path_publisher.publish(point_msg)
            self.get_logger().info(f"Publishing waypoint: {self.waypoints[node]}")

    def move_to_waypoint(self, current_pos, target_pos):
        """Move the robot from the current position to the target position."""
        # Calculate the difference in x and y
        dx = target_pos[0] - current_pos[0]
        dy = target_pos[1] - current_pos[1]
        distance = math.sqrt(dx**2 + dy**2)

        # Create a Twist message for linear and angular velocities
        cmd_msg = Twist()

        if distance > 0.1:  # If the target is not reached
            # Set the linear velocity proportional to the distance
            cmd_msg.linear.x = min(0.5, distance)  # Limiting speed to 0.5 m/s

            # Set the angular velocity to align with the target direction
            angle = math.atan2(dy, dx)
            cmd_msg.angular.z = angle
        else:
            # Stop the robot if the waypoint is reached
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0

        # Publish the command to move the robot
        self.cmd_publisher.publish(cmd_msg)

    def find_and_publish_path(self):
        """Find the shortest path using A* and publish it."""
        # Find the shortest path using A* algorithm
        path, expanded_nodes = self.a_star(self.start, self.end)
        
        if path:
            self.get_logger().info(f"A* Path found: {path}")
            self.get_logger().info(f"A* Expanded Nodes: {expanded_nodes}")

            # Now simulate or publish movement for each waypoint
            current_pos = self.waypoints[self.start]  # Starting position
            for node in path:
                target_pos = self.waypoints[node]
                self.move_to_waypoint(current_pos, target_pos)  # Move to the next waypoint
                current_pos = target_pos  # Update the current position
        else:
            self.get_logger().error("No path found!")

def main(args=None):
    rclpy.init(args=args)
    node = PathFinderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
