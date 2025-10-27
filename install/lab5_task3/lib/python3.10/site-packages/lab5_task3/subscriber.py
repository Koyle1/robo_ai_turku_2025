import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from math import cos, sin
from tf_transformations import euler_from_quaternion

import matplotlib
matplotlib.use('Agg')  # non-GUI backend
import matplotlib.pyplot as plt


class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_grid_mapper')
        self.pose = None
        self.counter = 0

        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(Odometry, '/odom', self.odom_callback, self.qos_profile)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, self.qos_profile)

        self.grid_map = np.full((100, 100), -1, dtype=int)
        self.map_resolution = 0.1
        self.map_origin = (50, 50)

        # Timer to dump map periodically (non‑GUI)
        self.create_timer(1.0, self.timer_dump_map)

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        self.pose = (pos.x, pos.y, yaw)
        self.get_logger().info(f"Pose: {self.pose}")

    def scan_callback(self, msg):
        if self.pose is None:
            return
        x_robot, y_robot, theta = self.pose
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                x_rel = r * cos(angle)
                y_rel = r * sin(angle)
                x_world = x_robot + x_rel * cos(theta) - y_rel * sin(theta)
                y_world = y_robot + x_rel * sin(theta) + y_rel * cos(theta)

                i_robot = int((x_robot / self.map_resolution) + self.map_origin[0])
                j_robot = int((y_robot / self.map_resolution) + self.map_origin[1])
                i_hit = int((x_world / self.map_resolution) + self.map_origin[0])
                j_hit = int((y_world / self.map_resolution) + self.map_origin[1])

                for i, j in self.bresenham(i_robot, j_robot, i_hit, j_hit):
                    if 0 <= i < 100 and 0 <= j < 100:
                        self.grid_map[j, i] = 0
                if 0 <= i_hit < 100 and 0 <= j_hit < 100:
                    self.grid_map[j_hit, i_hit] = 1
            angle += msg.angle_increment

    def timer_dump_map(self):
        # Dump map to a PNG file each interval for inspection
        self.counter += 1
        if self.counter == 10:
            self.counter = 0
            fname = f"grid_snapshot_{self.get_clock().now().nanoseconds}.png"
            plt.imshow(self.grid_map, cmap='gray', origin='lower', interpolation='none')
            plt.title("Occupancy Grid")
            plt.savefig(fname)
            plt.clf()
            self.get_logger().info(f"Saved map snapshot to {fname}")

    def bresenham(self, x0, y0, x1, y1):
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = 1 if x1 > x0 else -1
        sy = 1 if y1 > y0 else -1
        if dx > dy:
            err = dx / 2
            while x != x1:
                yield x, y
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2
            while y != y1:
                yield x, y
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        yield x1, y1


def main(args=None):
    rclpy.init(args=args)
    node = LidarSubscriber()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()