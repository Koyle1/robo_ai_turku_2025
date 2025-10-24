import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        # Publisher and subscriber
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Point, '/path', self.path_callback, 10)
        self.create_timer(0.1, self.loop)  # 10 Hz

        # Initialize state variables
        self.path = []  # List of waypoints from the pathfinder
        self.current_wp = 0  # Index of the current waypoint
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Control state
        self.state = 'rotate'  # either 'rotate' or 'forward'

        # Parameters
        self.linear_speed = 0.15
        self.angular_speed = 0.4
        self.angle_tolerance = 0.05     # radians (~3°)
        self.distance_tolerance = 0.1   # meters

    def odom_callback(self, msg):
        # Get the robot's current position and orientation from Odometry
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.x, self.y, self.yaw = pos.x, pos.y, yaw

    def path_callback(self, msg):
        # Update the path from the pathfinder (a list of points)
        self.path.append((msg.x, msg.y))
        self.get_logger().info(f"Received new waypoint: ({msg.x}, {msg.y})")

    def loop(self):
        # Only proceed if we have waypoints in the path
        if self.current_wp >= len(self.path):
            self.cmd_pub.publish(Twist())  # Stop the robot when all waypoints are completed
            self.get_logger().info('Reached final waypoint, shutting down.')
            rclpy.shutdown()  # Shutdown the node when the last waypoint is reached
            return

        # Current goal position
        gx, gy = self.path[self.current_wp]
        dx, dy = gx - self.x, gy - self.y
        distance = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)

        # Normalize angle difference to [-pi, pi]
        angle_diff = math.atan2(math.sin(target_angle - self.yaw),
                                math.cos(target_angle - self.yaw))

        cmd = Twist()

        if self.state == 'rotate':
            # Rotate in place until facing the waypoint
            if abs(angle_diff) > self.angle_tolerance:
                cmd.angular.z = self.angular_speed * math.copysign(1.0, angle_diff)
            else:
                self.state = 'forward'  # Switch to moving forward
        elif self.state == 'forward':
            if distance > self.distance_tolerance:
                cmd.linear.x = self.linear_speed
            else:
                self.state = 'rotate'
                self.current_wp += 1  # Move to the next waypoint

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
