import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import numpy as np
import matplotlib.pyplot as plt
from rclpy.executors import MultiThreadedExecutor

class TrajectoryVisualizer(Node):
    def __init__(self):
        super().__init__('trajectory_visualizer')

        # Trajectory storage
        self.trajectory = []

        # Store velocity data
        self.linear_velocities = []
        self.angular_velocities = []
        self.time_stamps = []

        # Planned path from Dijkstra (optional)
        self.planned_path = []

        # Subscriptions
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        qos_path = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        self.create_subscription(Point, '/path', self.path_callback, qos_path)


        # Plot update timer
        self.plot_interval = 5.0  # seconds
        self.create_timer(self.plot_interval, self.visualize)

        # Start time for velocity plots
        self.start_time = None

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        self.trajectory.append((pos.x, pos.y))

    def cmd_vel_callback(self, msg):
        if self.start_time is None:
            self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.linear_velocities.append(msg.linear.x)
        self.angular_velocities.append(msg.angular.z)
        t = self.get_clock().now().seconds_nanoseconds()[0] - self.start_time
        self.time_stamps.append(t)

    def path_callback(self, msg):
        # Append the received waypoint
        self.planned_path.append((msg.x, msg.y))

    def visualize(self):
        if not self.trajectory and not self.planned_path:
            return

        fig, axs = plt.subplots(3, 1, figsize=(8, 12), constrained_layout=True)

        # --- Plot trajectory ---
        if self.trajectory:
            x_vals, y_vals = zip(*self.trajectory)
            axs[0].plot(x_vals, y_vals, color='red', label='Trajectory')

        # --- Plot planned path ---
        if self.planned_path:
            px, py = zip(*self.planned_path)
            axs[0].plot(px, py, color='blue', linestyle='--', marker='o', label='Planned Path')

        axs[0].set_title("Robot Trajectory and Planned Path")
        axs[0].set_xlabel("X (meters)")
        axs[0].set_ylabel("Y (meters)")
        axs[0].legend()
        axs[0].grid(True)

        # --- Linear velocity ---
        if self.linear_velocities:
            axs[1].plot(self.time_stamps, self.linear_velocities, color='blue', label='Linear Velocity (m/s)')
            axs[1].set_title("Linear Velocity Over Time")
            axs[1].set_xlabel("Time (s)")
            axs[1].set_ylabel("Linear Velocity (m/s)")
            axs[1].legend()
            axs[1].grid(True)

        # --- Angular velocity ---
        if self.angular_velocities:
            axs[2].plot(self.time_stamps, self.angular_velocities, color='orange', label='Angular Velocity (rad/s)')
            axs[2].set_title("Angular Velocity Over Time")
            axs[2].set_xlabel("Time (s)")
            axs[2].set_ylabel("Angular Velocity (rad/s)")
            axs[2].legend()
            axs[2].grid(True)

        # Save or display the plot
        plt.savefig("4_trajectory_and_path.png")
        plt.clf()
        self.get_logger().info("Saved trajectory and path visualization.")

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryVisualizer()
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
