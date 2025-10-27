import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import heapq
import math
import time

class GlobalPlannerNode(Node):
    def __init__(self):
        super().__init__('global_planner_node')

        # QoS: keep the last message, make it latched-like so late subscribers still get it
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        # Publisher for the full path
        self.path_pub = self.create_publisher(Path, '/nav/global_path', qos_profile)

        # Graph connectivity (adjacency matrix)
        self.adjacency_matrix = np.array([
            #0 1 2 3 4 5 6 7 8 9 10
            [0,1,0,0,1,0,0,0,0,0,0],   # 0  (0.0, 0.0)
            [1,0,1,0,0,1,0,0,0,0,0],   # 1  (1.5, -1.5)
            [0,1,0,1,0,0,0,0,0,0,0],   # 2  (1.5, 3.0)
            [0,0,1,0,1,0,0,0,0,0,1],   # 3  (-0.8, 3.0)
            [1,0,0,1,0,1,0,0,0,0,0],   # 4  (-0.5, 0.5)
            [0,1,0,0,1,0,1,0,1,1,0],   # 5  (-2.2, 0.0)
            [0,0,0,0,0,1,0,1,0,0,0],   # 6  (-3.0, -2.0)
            [0,0,0,0,0,0,1,0,1,0,0],   # 7  (-4.2, -2.2)
            [0,0,0,0,0,0,0,1,0,1,0],   # 8  (-4.2, -0.15)
            [0,0,0,0,0,1,0,0,1,0,1],   # 9  (-2.8, 0.5)
            [0,0,0,1,0,0,0,0,0,1,0]    # 10 (-2.8, 3.0)
        ], dtype=int)

        # Waypoint coordinates for each node index
        self.waypoints = [
            (0.0,   0.0),    # 0
            (1.5,  -1.5),    # 1
            (1.5,   3.0),    # 2
            (-0.8,  3.0),    # 3
            (-0.5,  0.5),    # 4
            (-2.2,  0.0),    # 5
            (-3.0, -2.0),    # 6
            (-4.2, -2.2),    # 7
            (-4.2, -0.15),   # 8
            (-2.8,  0.5),    # 9
            (-2.8,  3.0)     # 10
        ]

        # pick start and goal by index in waypoints[]
        self.start_idx = 0  
        self.goal_idx  = 7   

        # Plan once and publish
        self.get_logger().info(f"Start and end: {self.start_idx, self.goal_idx}")
        path_idx_list, cost = self.dijkstra(self.start_idx, self.goal_idx)
        self.get_logger().info(f"Shortest path (by index): {path_idx_list}")
        self.get_logger().info(f"Total cost: {cost:.3f}")

        self.publish_path_msg(path_idx_list)

    def dijkstra(self, start, end):
        num_nodes = len(self.adjacency_matrix)

        # distances dict
        distances = {node: float('inf') for node in range(num_nodes)}
        distances[start] = 0.0

        # best parent tracking
        previous = {node: None for node in range(num_nodes)}

        # min-heap priority queue
        queue = [(0.0, start)]

        while queue:
            current_dist, current_node = heapq.heappop(queue)

            if current_node == end:
                break

            # check neighbors
            for neighbor in range(num_nodes):
                if self.adjacency_matrix[current_node][neighbor] == 1:
                    # edge weight = euclidean distance between waypoints
                    edge_cost = math.dist(
                        self.waypoints[current_node],
                        self.waypoints[neighbor]
                    )
                    new_dist = current_dist + edge_cost

                    if new_dist < distances[neighbor]:
                        distances[neighbor] = new_dist
                        previous[neighbor] = current_node
                        heapq.heappush(queue, (new_dist, neighbor))

        # reconstruct path
        path_nodes = []
        node = end
        while node is not None:
            path_nodes.insert(0, node)
            node = previous[node]

        return path_nodes, distances[end]

    def publish_path_msg(self, path_nodes):
        """Convert list of node indices into a nav_msgs/Path and publish it once."""
        path_msg = Path()
        # stamp with current time so TF consumers are happy
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"  # <-- IMPORTANT: use whatever global frame you use

        poses = []
        for idx in path_nodes:
            x, y = self.waypoints[idx]

            p = PoseStamped()
            p.header = path_msg.header  # same frame/time
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.position.z = 0.0

            # orientation: we don't really care here, just zero it
            p.pose.orientation.w = 1.0

            poses.append(p)

        path_msg.poses = poses

        self.path_pub.publish(path_msg)
        self.get_logger().info("Published global path with %d waypoints" % len(poses))

def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()