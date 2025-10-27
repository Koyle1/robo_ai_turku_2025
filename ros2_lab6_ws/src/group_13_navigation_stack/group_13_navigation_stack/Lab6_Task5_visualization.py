import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor


from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, Point
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy


import numpy as np
import matplotlib.pyplot as plt




class TrajectoryVisualizer(Node):
    def __init__(self):
        super().__init__('trajectory_visualizer')


        # --- Internal storage ---
        # robot actual path over time (from odom)
        self.trajectory = []  # [(x,y), (x,y), ...]


        # planned global path waypoints (from Dijkstra)
        self.planned_path = []  # [(x,y), ...]


        # last published local target waypoint (for debugging)
        self.current_target = None  # (x,y)


        # velocity over time
        self.linear_velocities = []
        self.angular_velocities = []
        self.time_stamps = []
        self.start_time = None


        # --- QoS profiles ---
        # global path is latched, so we want TRANSIENT_LOCAL / KEEP_LAST / depth=1
        path_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )


        # normal best-effort OK for odom/cmd_vel
        default_qos = 10


        # --- Subscriptions ---


        # Robot pose (odom)
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            default_qos
        )


        # Velocity commands sent to robot
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            default_qos
        )


        # Full global path from planner
        self.create_subscription(
            Path,
            '/nav/global_path',
            self.global_path_callback,
            path_qos
        )


        # Current target waypoint from waypoint_manager
        self.create_subscription(
            Point,
            '/nav/target_waypoint',
            self.target_waypoint_callback,
            path_qos  # latch-like is useful here too
        )


        # --- Timer for plotting ---
        # save plot every N seconds
        self.plot_interval = 5.0  # seconds
        self.create_timer(self.plot_interval, self.visualize)


        self.get_logger().info("TrajectoryVisualizer started.")


    # ------------------
    # Callbacks
    # ------------------


    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        self.trajectory.append((pos.x, pos.y))


    def cmd_vel_callback(self, msg: Twist):
        now_sec = self.get_clock().now().seconds_nanoseconds()[0]
        if self.start_time is None:
            self.start_time = now_sec


        t = now_sec - self.start_time
        self.time_stamps.append(t)
        self.linear_velocities.append(msg.linear.x)
        self.angular_velocities.append(msg.angular.z)


    def global_path_callback(self, msg: Path):
        # Store the full planned path one time (or overwrite on update)
        pts = []
        for pose_stamped in msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            pts.append((x, y))
        self.planned_path = pts
        self.get_logger().info(f"Received global path with {len(pts)} waypoints.")


    def target_waypoint_callback(self, msg: Point):
        # store last active waypoint the controller is chasing
        self.current_target = (msg.x, msg.y)


    # ------------------
    # Plotting
    # ------------------


    def visualize(self):
        if not self.trajectory and not self.planned_path:
            return


        # === Rotation helper ===
        def rotate_points(points, angle_deg):
            """Rotate list of (x,y) points by angle (degrees) around origin."""
            if not points:
                return []
            angle = np.deg2rad(angle_deg)
            R = np.array([
                [np.cos(angle), -np.sin(angle)],
                [np.sin(angle),  np.cos(angle)]
            ])
            pts = np.dot(R, np.array(points).T)
            return list(zip(pts[0], pts[1]))


        # === Choose rotation angle ===
        rot_angle = 90  # <-- rotate entire plot by 90 degrees


        # === Apply rotation ===
        rotated_traj = rotate_points(self.trajectory, rot_angle)
        rotated_path = rotate_points(self.planned_path, rot_angle)
        rotated_target = None
        if self.current_target is not None:
            rotated_target = rotate_points([self.current_target], rot_angle)[0]


        # === Create figure ===
        fig, axs = plt.subplots(3, 1, figsize=(8, 12), constrained_layout=True)


        # --- Plot planned path ---
        if rotated_path:
            px, py = zip(*rotated_path)
            axs[0].plot(px, py, '--o', color='blue', label='Planned Path (Rotated 90°)')


        # --- Plot actual trajectory ---
        if rotated_traj:
            tx, ty = zip(*rotated_traj)
            axs[0].plot(tx, ty, '-', color='red', linewidth=2, label='Driven Trajectory (Rotated 90°)')


        # --- Plot current target waypoint ---
        if rotated_target is not None:
            axs[0].scatter(
                rotated_target[0],
                rotated_target[1],
                color='green',
                s=80,
                marker='x',
                label='Current Target Waypoint',
            )


        axs[0].set_title("Robot Trajectory vs Planned Path (Rotated 90°)")
        axs[0].set_xlabel("X [m]")
        axs[0].set_ylabel("Y [m]")
        axs[0].legend()
        axs[0].grid(True)
        axs[0].set_aspect('equal', adjustable='datalim')


        # --- Linear velocity ---
        if self.linear_velocities:
            axs[1].plot(self.time_stamps, self.linear_velocities, color='blue')
        axs[1].set_title("Linear Velocity Over Time")
        axs[1].set_xlabel("Time [s]")
        axs[1].set_ylabel("v [m/s]")
        axs[1].grid(True)


        # --- Angular velocity ---
        if self.angular_velocities:
            axs[2].plot(self.time_stamps, self.angular_velocities, color='orange')
        axs[2].set_title("Angular Velocity Over Time")
        axs[2].set_xlabel("Time [s]")
        axs[2].set_ylabel("ω [rad/s]")
        axs[2].grid(True)


        # Save the rotated plot
        plt.savefig("task6.png")
        plt.close(fig)
        self.get_logger().info("Saved rotated trajectory plot as trajectory_debug_rotated.png")



def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryVisualizer()


    # MultiThreadedExecutor lets callbacks and timer run "simultaneously"
    executor = MultiThreadedExecutor()
    executor.add_node(node)


    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("TrajectoryVisualizer shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


