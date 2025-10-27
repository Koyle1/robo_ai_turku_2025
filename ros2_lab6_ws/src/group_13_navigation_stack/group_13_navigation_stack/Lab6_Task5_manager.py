import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Point

import math

class WaypointManagerNode(Node):
    def __init__(self):
        super().__init__('waypoint_manager_node')

        # QoS: latch-like for the global path (planner publishes once)
        path_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        # QoS: latch-like for the target waypoint so late subscribers still get it
        target_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        # --- Subscribers ---
        self.create_subscription(Path, '/nav/global_path', self.path_callback, path_qos)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # --- Publisher ---
        self.target_pub = self.create_publisher(Point, '/nav/target_waypoint', target_qos)

        # Internal state
        self.path_points = []   # [(x,y), (x,y), ...]
        self.current_idx = 0
        self.robot_x = None
        self.robot_y = None

        # tuning param: how close is "we reached this waypoint"
        self.reached_threshold = 0.2

        # run control loop @10 Hz
        self.timer = self.create_timer(0.1, self.loop)

    def path_callback(self, msg: Path):
        # Convert nav_msgs/Path to list of (x,y)
        self.path_points = [
            (p.pose.position.x, p.pose.position.y)
            for p in msg.poses
        ]
        self.current_idx = 0

        self.get_logger().info(f"New path with {len(self.path_points)} waypoints")
        # Immediately publish first target
        self.publish_current_waypoint()

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def loop(self):
        # Need both odom and path
        if self.robot_x is None or not self.path_points:
            return
        if self.current_idx >= len(self.path_points):
            # done with path
            return

        # current target
        tx, ty = self.path_points[self.current_idx]

        # distance to target
        dx = tx - self.robot_x
        dy = ty - self.robot_y
        dist = math.hypot(dx, dy)

        # did we reach it?
        if dist < self.reached_threshold:
            self.get_logger().info(f"Reached waypoint {self.current_idx} ({tx:.2f}, {ty:.2f})")
            self.current_idx += 1
            if self.current_idx >= len(self.path_points):
                self.get_logger().info("Final waypoint reached.")
                return

        # publish (or republish) active target waypoint
        self.publish_current_waypoint()

    def publish_current_waypoint(self):
        if self.current_idx >= len(self.path_points):
            return
        tx, ty = self.path_points[self.current_idx]

        msg = Point()
        msg.x = tx
        msg.y = ty
        msg.z = 0.0
        self.target_pub.publish(msg)

        self.get_logger().info(
            f"Current target waypoint {self.current_idx}: ({tx:.2f}, {ty:.2f})"
        )

def main(args=None):
    rclpy.init(args=args)
    node = WaypointManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()