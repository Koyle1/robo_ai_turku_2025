import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
import numpy as np
import heapq
import math

class FullGraphExplorer(Node):
    def __init__(self):
        super().__init__('full_graph_explorer')

        # --- ROS publisher ---
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=25,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.path_pub = self.create_publisher(Point, '/path', qos_profile)

        # --- Graph Definition (ensure 2+ loops) ---
        self.adj_matrix = np.array([
        # 0  1  2  3  4  5  6  7  8  9 10
        [0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0],  # 0
        [1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0],  # 1
        [0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0],  # 2
        [0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1],  # 3
        [1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0],  # 4
        [0, 1, 0, 0, 1, 0, 1, 0, 1, 1, 0],  # 5
        [0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0],  # 6
        [0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0],  # 7
        [0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0],  # 8
        [0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1],  # 9
        [0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0]   # 10
        ])

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

        # --- State ---
        self.visited = set([0])
        self.order = [0]
        self.current_node = 0

        # --- ROS timer ---
        self.publish_waypoint(0)
        self.create_timer(0.5, self.explore_next_node)
        self.get_logger().info("Starting continuous exploration (revisiting allowed)")

    def dijkstra(self, start):
        n = len(self.adj_matrix)
        dist = {i: float('inf') for i in range(n)}
        prev = {i: None for i in range(n)}
        dist[start] = 0
        queue = [(0, start)]
        while queue:
            d, node = heapq.heappop(queue)
            if d > dist[node]:
                continue
            for nb in range(n):
                if self.adj_matrix[node][nb] == 1:
                    edge = math.dist(self.waypoints[node], self.waypoints[nb])
                    new_d = d + edge
                    if new_d < dist[nb]:
                        dist[nb] = new_d
                        prev[nb] = node
                        heapq.heappush(queue, (new_d, nb))
        return dist, prev

    def shortest_path_to_nearest_unvisited(self):
        dist, prev = self.dijkstra(self.current_node)
        unvisited = [n for n in range(len(self.waypoints)) if n not in self.visited and dist[n] < float('inf')]
        if not unvisited:
            return None
        nearest = min(unvisited, key=lambda n: dist[n])
        # reconstruct path
        path = []
        node = nearest
        while node is not None:
            path.insert(0, node)
            node = prev[node]
        return path

    def publish_waypoint(self, idx):
        p = Point()
        p.x, p.y = self.waypoints[idx]
        p.z = 0.0
        self.path_pub.publish(p)
        self.get_logger().info(f"Published waypoint {idx}: {self.waypoints[idx]}")

    def explore_next_node(self):
        path = self.shortest_path_to_nearest_unvisited()
        if path is None:
            self.get_logger().info(f"Exploration complete! Order: {self.order}")
            rclpy.shutdown()
            return

        for node in path[1:]:  # skip current node
            self.publish_waypoint(node)
            if node not in self.visited:
                self.visited.add(node)
            self.order.append(node)
            self.current_node = node

        self.get_logger().info(f"Visited order so far: {self.order}")

def main(args=None):
    rclpy.init(args=args)
    node = FullGraphExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
