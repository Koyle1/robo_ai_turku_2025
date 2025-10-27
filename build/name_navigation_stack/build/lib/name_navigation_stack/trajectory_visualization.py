import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import matplotlib.pyplot as plt
from rclpy.executors import MultiThreadedExecutor
import os
from collections import deque
import time

class TrajectoryVisualizer(Node):
    def __init__(self):
        super().__init__('trajectory_visualizer')

        # Trajectory storage
        self.trajectory = []

        # Parameters for map
        self.map_resolution = 0.1  # Assuming resolution is 0.1 m per cell
        self.map_size = (100, 100)  # Assuming map size is 100x100

        # For storing velocity data
        self.linear_velocities = []  # Store all linear velocities
        self.angular_velocities = []  # Store all angular velocities
        self.time_stamps = []  # Store all timestamps


        # Subscription to odometry and map file path
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(String, '/grid_map_file_path', self.map_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # To store the loaded map
        self.grid_map = None

        # For plot update control
        self.counter = 0
        self.save_interval = 10  # Save visualization every 10 seconds

        # Timer to periodically save the visualization image
        self.create_timer(1.0, self.timer_save_progress)  # Check every second

    def odom_callback(self, msg):
        # Record the trajectory (only position)
        pos = msg.pose.pose.position
        self.trajectory.append((pos.x, pos.y))

    def cmd_vel_callback(self, msg):
        # Record the linear and angular velocities
        self.linear_velocities.append(msg.linear.x)
        self.angular_velocities.append(msg.angular.z)
        self.time_stamps.append(self.get_clock().now().seconds_nanoseconds()[0]) 

    def map_callback(self, msg):
        # Load the grid map when the file path is received
        file_path = msg.data
        if os.path.exists(file_path):
            self.get_logger().info(f"Loading map from: {file_path}")
            # Load the map as a 2D numpy array (assuming it's a CSV)
            self.grid_map = np.loadtxt(file_path, delimiter=',', dtype=int)
            self.get_logger().info("Map loaded successfully.")

    def timer_save_progress(self):
    # Call the visualization function at regular intervals (every second)
        self.visualize_trajectory_and_velocities()


    def visualize_trajectory_and_velocities(self):
        if self.grid_map is not None and len(self.trajectory) > 0:
            # Create a figure with three vertical subplots
            fig, axs = plt.subplots(3, 1, figsize=(8, 12), constrained_layout=True)

            # --- Map + Trajectory ---
            axs[0].imshow(self.grid_map, cmap='gray', origin='lower', interpolation='none')
            axs[0].set_title("Map with Robot Trajectory")
            axs[0].set_xlabel("Map X (cells)")
            axs[0].set_ylabel("Map Y (cells)")

            # Map origin and robot start position (in meters)
            map_origin = (50, 50)  # map origin in grid cells
            start_pose = (-2, 0.5)  # robot's starting position in meters
            map_resolution = 0.1  # map resolution (meters per cell)

            # Convert trajectory (in meters) to grid cells
            x_vals, y_vals = zip(*self.trajectory)
            x_pixels = [(x / map_resolution) + map_origin[0] for x in x_vals]
            y_pixels = [(y / map_resolution) + map_origin[1] for y in y_vals]

            # Plot trajectory on the map in grid cells
            axs[0].plot(x_pixels, y_pixels, color='red', label="Trajectory")
            axs[0].legend()

            # --- Linear velocity ---
            if self.linear_velocities:
                time = np.array(self.time_stamps) - self.time_stamps[0]  # normalize time to 0
                linear_velocity = np.array(self.linear_velocities)
                axs[1].plot(time, linear_velocity, label="Linear Velocity (m/s)", color='blue')
                axs[1].set_title("Linear Velocity Over Time")
                axs[1].set_xlabel("Time (seconds)")
                axs[1].set_ylabel("Linear Velocity (m/s)")
                axs[1].legend()
                axs[1].grid(True)

            # --- Angular velocity ---
            if self.angular_velocities:
                time = np.array(self.time_stamps) - self.time_stamps[0]  # normalize time to 0
                angular_velocity = np.array(self.angular_velocities)
                axs[2].plot(time, angular_velocity, label="Angular Velocity (rad/s)", color='orange')
                axs[2].set_title("Angular Velocity Over Time")
                axs[2].set_xlabel("Time (seconds)")
                axs[2].set_ylabel("Angular Velocity (rad/s)")
                axs[2].legend()
                axs[2].grid(True)

            # Save the visualization image periodically
            fname = f"trajectory_visualization_{self.get_clock().now().nanoseconds}.png"
            plt.savefig(fname)
            plt.clf()
            self.get_logger().info(f"Saved visualization snapshot to {fname}")


    def timer_save_progress(self):
        self.counter += 1
        if self.counter % self.save_interval == 0:
            self.visualize_trajectory_and_velocities()

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
