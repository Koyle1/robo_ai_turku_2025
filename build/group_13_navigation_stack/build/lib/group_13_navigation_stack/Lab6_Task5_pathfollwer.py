import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion
import math

class LocalControllerNode(Node):
    def __init__(self):
        super().__init__('local_controller_node')

        # --- Subscribers ---
        # current local goal (single waypoint) from waypoint_manager_node
        self.create_subscription(Point, '/nav/target_waypoint', self.waypoint_cb, 10)

        # robot pose for position + yaw
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        # lidar scan for obstacle distances
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        # --- Publisher ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- State ---
        self.goal_x = None
        self.goal_y = None

        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None

        self.scan_msg = None

        # --- Parameters / gains ---
        # how hard we pull toward the waypoint
        self.k_att = 1.0

        # how hard we push away from obstacles
        self.k_rep = 0.8

        # how close an obstacle has to be to really matter (safety bubble)
        self.repulsion_radius = 0.8  # meters

        # speed limits
        self.max_lin = 0.2   # m/s
        self.max_ang = 0.6   # rad/s

        # timer loop at 10 Hz
        self.create_timer(0.1, self.control_loop)

    def waypoint_cb(self, msg: Point):
        # save current target waypoint
        self.goal_x = msg.x
        self.goal_y = msg.y

    def odom_cb(self, msg: Odometry):
        # save robot pose
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.robot_yaw = yaw

    def scan_cb(self, msg: LaserScan):
        self.scan_msg = msg

    def control_loop(self):
        # we need everything to be valid to move
        if (
            self.goal_x is None or
            self.robot_x is None or
            self.scan_msg is None or
            self.robot_yaw is None
        ):
            return

        # --------------------------
        # 1. Attractive force
        # --------------------------
        # Vector from robot -> goal in world frame
        dx_goal = self.goal_x - self.robot_x
        dy_goal = self.goal_y - self.robot_y

        # attractive force (scaled toward goal)
        F_att_x = self.k_att * dx_goal
        F_att_y = self.k_att * dy_goal

        # --------------------------
        # 2. Repulsive force
        # --------------------------
        # We'll look at the closest obstacle in the scan.
        # If it's within repulsion_radius, push away from it.
        r_min = None
        ang_min = None

        ranges = self.scan_msg.ranges
        angle_min = self.scan_msg.angle_min
        angle_inc = self.scan_msg.angle_increment

        # find closest valid hit
        for i, r in enumerate(ranges):
            if math.isfinite(r):
                if r_min is None or r < r_min:
                    r_min = r
                    ang_min = angle_min + i * angle_inc

        # default repulsive force = 0
        F_rep_x = 0.0
        F_rep_y = 0.0

        if r_min is not None and r_min < self.repulsion_radius:
            # obstacle position in ROBOT frame
            obs_rx = r_min * math.cos(ang_min)
            obs_ry = r_min * math.sin(ang_min)

            # rotate that to WORLD frame using robot yaw
            cy = math.cos(self.robot_yaw)
            sy = math.sin(self.robot_yaw)
            obs_wx = self.robot_x + cy * obs_rx - sy * obs_ry
            obs_wy = self.robot_y + sy * obs_rx + cy * obs_ry

            # vector from robot -> obstacle in world
            d_ox = obs_wx - self.robot_x
            d_oy = obs_wy - self.robot_y
            dist_obs = math.hypot(d_ox, d_oy)

            if dist_obs > 1e-6:
                # direction pointing AWAY from obstacle
                away_x = -d_ox / dist_obs
                away_y = -d_oy / dist_obs

                # stronger when obstacle is closer
                strength = self.k_rep * (1.0 / max(dist_obs, 0.01) - 1.0 / self.repulsion_radius)
                if strength < 0.0:
                    strength = 0.0

                F_rep_x = strength * away_x
                F_rep_y = strength * away_y

        # --------------------------
        # 3. Combine forces
        # --------------------------
        F_x = F_att_x + F_rep_x
        F_y = F_att_y + F_rep_y

        # If total force is basically zero, just stop
        mag = math.hypot(F_x, F_y)
        if mag < 1e-4:
            self.publish_stop()
            return

        # desired world-frame heading
        desired_yaw = math.atan2(F_y, F_x)

        # heading error in [-pi, pi]
        yaw_err = math.atan2(
            math.sin(desired_yaw - self.robot_yaw),
            math.cos(desired_yaw - self.robot_yaw)
        )

        # --------------------------
        # 4. Turn + drive
        # --------------------------
        cmd = Twist()

        # angular command: proportional to heading error
        cmd.angular.z = max(-self.max_ang, min(self.max_ang, 1.5 * yaw_err))

        # linear command: forward speed ~ force magnitude, but slow down if we're turning hard
        forward_gain = 0.2
        turn_slowdown = max(0.0, 1.0 - (abs(yaw_err) / 1.0))  # 1.0 rad (~57°) => start slowing
        cmd.linear.x = min(self.max_lin, forward_gain * mag * turn_slowdown)

        # safety: if obstacle is extremely close (<0.2m), override and stop forward motion
        if r_min is not None and r_min < 0.2:
            cmd.linear.x = 0.0

        self.cmd_pub.publish(cmd)

    def publish_stop(self):
        stop = Twist()
        self.cmd_pub.publish(stop)


def main(args=None):
    rclpy.init(args=args)
    node = LocalControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()