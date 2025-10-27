import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
import numpy as np
import heapq
import math

class AStarPathFollower(Node):
    def __init__(self):
        super().__init__('pathfinderA')

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

        self.start = 2
        self.end = 7

        self.find_and_publish_path()

    def astar(self, start, end):
        """A* algorithm returning path, total_cost, expanded nodes"""
        num_nodes = len(self.adjacency_matrix)
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {node: None for node in range(num_nodes)}
        g_score = {node: float('inf') for node in range(num_nodes)}
        g_score[start] = 0
        expanded_nodes = 0

        def heuristic(n1, n2):
            return math.dist(self.waypoints[n1], self.waypoints[n2])

        f_score = {node: float('inf') for node in range(num_nodes)}
        f_score[start] = heuristic(start, end)

        while open_set:
            _, current = heapq.heappop(open_set)
            expanded_nodes += 1

            if current == end:
                break

            for neighbor in range(num_nodes):
                if self.adjacency_matrix[current][neighbor] == 1:
                    tentative_g = g_score[current] + heuristic(current, neighbor)
                    if tentative_g < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g
                        f_score[neighbor] = tentative_g + heuristic(neighbor, end)
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

        # reconstruct path
        path = []
        node = end
        while node is not None:
            path.insert(0, node)
            node = came_from[node]

        total_cost = g_score[end]
        return path, total_cost, expanded_nodes

    def publish_path(self, path):
        for node in path:
            msg = Point()
            msg.x, msg.y, msg.z = *self.waypoints[node], 0.0
            self.path_publisher.publish(msg)
            self.get_logger().info(f"Publishing waypoint: {self.waypoints[node]}")

    def find_and_publish_path(self):
        path, total_cost, expanded_nodes = self.astar(self.start, self.end)
        if path:
            self.get_logger().info(f"A* Path found: {path}")
            self.get_logger().info(f"A* Total cost: {total_cost:.3f}")
            self.get_logger().info(f"A* Expanded Nodes: {expanded_nodes}")
            self.publish_path(path)
        else:
            self.get_logger().error("No path found!")

def main(args=None):
    rclpy.init(args=args)
    node = AStarPathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
 