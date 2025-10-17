#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from math import sin, cos, atan2

from pfilter import ParticleFilter, squared_error

from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from scipy.signal import convolve2d
import matplotlib.pyplot as plt
import time


def quat_to_yaw(q):
    """Convert quaternion to yaw (Euler angle)"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return atan2(siny_cosp, cosy_cosp)


class LidarParticleFilter(Node):
    def __init__(self):
        super().__init__('lidar_pf_with_map')

        # QoS for subscriptions
        self.qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=10,
        )

        # Particle filter parameters
        self.declare_parameters('', [
            ("weights_sigma", 1.2),
            ("num_particles", 200),
            ("measurement_noise", 0.05),
            ("resample_proportion", 0.01),
        ])

        self.weights_sigma = self.get_parameter('weights_sigma').value
        self.num_particles = self.get_parameter('num_particles').value
        self.measurement_noise = self.get_parameter('measurement_noise').value
        self.resample_proportion = self.get_parameter('resample_proportion').value

        self.get_logger().info(
            f"PF Params: sigma={self.weights_sigma}, num={self.num_particles}, noise={self.measurement_noise}"
        )

        # Known obstacle positions (for PF)
        self.real_obstacle_position = np.array([2.0, 1.0])
        self.second_obstacle_position = np.array([2.0, -1.0])

        # Create particle filter
        self.prior_fn = lambda n: np.random.uniform(-5, 5, (n, 2))
        self.pf = ParticleFilter(
            prior_fn=self.prior_fn,
            observe_fn=self.calc_hypothesis,
            dynamics_fn=self.velocity,
            n_particles=self.num_particles,
            noise_fn=self.add_noise,
            weight_fn=self.calc_weights,
            resample_proportion=self.resample_proportion
        )

        # Odometry delta tracking
        self._have_first_odom = False
        self._last_odom_xy = np.zeros(2, dtype=float)
        self.particle_odom = np.zeros(2, dtype=float)

        # History buffers
        self.odom_hist = []
        self.pf_hist = []
        self.pf_yaw_est = None
        self.odom_yaw_hist = []
        self.pf_yaw_hist = []

        # Map parameters
        self.map_size = 100
        self.grid_resolution = 0.1  # meters per cell
        self.map_center = np.array([self.map_size // 2, self.map_size // 2], dtype=int)
        # Initialize map: -1 unknown, 0 free, 1 occupied
        self.occupancy_map = -1 * np.ones((self.map_size, self.map_size), dtype=int)

        # Create convolution kernels for cubes and cylinders
        self.kernel_cube = np.array([[1, 1, 1],
                                     [1, 1, 1],
                                     [1, 1, 1]], dtype=float)
        self.kernel_cylinder = np.array([[0, 1, 0],
                                         [1, 1, 1],
                                         [0, 1, 0]], dtype=float)

        # Subscribers
        self.create_subscription(Odometry, "/odom", self.odometry_cb, qos_profile=self.qos)
        self.create_subscription(LaserScan, "/scan", self.scan_cb, qos_profile=self.qos)

        # Timer for periodic update
        self.create_timer(0.2, self.update_filter)

        # Real-time plotting setup
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(7, 6))
        (self.odom_line,) = self.ax.plot([], [], label="Odometry path")
        (self.pf_line,) = self.ax.plot([], [], label="PF path")
        # Quivers for headings
        self.heading_len = 0.5
        self.true_heading = self.ax.quiver(0, 0, 0, 0, angles='xy', scale_units='xy', scale=1, color='blue', label="True yaw")
        self.pf_heading = self.ax.quiver(0, 0, 0, 0, angles='xy', scale_units='xy', scale=1, color='red', label="PF yaw")
        self.ax.legend()
        self.ax.set_xlabel("X [m]")
        self.ax.set_ylabel("Y [m]")
        self.ax.set_title("Localization + Mapping")
        self.ax.grid(True)

        self.odometry = Odometry()
        self.scan = LaserScan()

    def odometry_cb(self, msg):
        self.odometry = msg

    def scan_cb(self, msg):
        self.scan = msg

    def calc_hypothesis(self, x):
        obs1 = self.real_obstacle_position - x
        obs2 = self.second_obstacle_position - x
        return np.hstack((obs1, obs2))

    def velocity(self, x):
        return x + self.particle_odom

    def add_noise(self, x):
        return x + np.random.normal(0, self.measurement_noise, x.shape)

    def calc_weights(self, hypotheses, observations):
        return squared_error(hypotheses, observations, sigma=self.weights_sigma)

    def world_to_map(self, pos):
        """Convert world XY to map indices."""
        # pos is [x, y]
        i = int(round(pos[0] / self.grid_resolution)) + self.map_center[0]
        j = int(round(pos[1] / self.grid_resolution)) + self.map_center[1]
        return i, j

    def bresenham(self, x0, y0, x1, y1):
        """Generate cells along line from (x0, y0) to (x1, y1)."""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        return points

    def detect_shapes(self):
        """Apply convolutional kernels on the occupied map to detect shapes."""
        # Create binary map of occupied cells
        occ = (self.occupancy_map == 1).astype(float)

        conv_c = convolve2d(occ, self.kernel_cube, mode='same', boundary='fill', fillvalue=0)
        conv_cyl = convolve2d(occ, self.kernel_cylinder, mode='same', boundary='fill', fillvalue=0)

        # thresholds
        cube_thresh = np.sum(self.kernel_cube) * 0.8
        cyl_thresh = np.sum(self.kernel_cylinder) * 0.8

        cube_hits = np.argwhere(conv_c >= cube_thresh)
        cyl_hits = np.argwhere(conv_cyl >= cyl_thresh)

        return cube_hits, cyl_hits

    def update_map(self, robot_pos, obs_rel):
        """Update occupancy map with ray from robot to observed obstacle."""
        # Convert positions
        robot_map = self.world_to_map(robot_pos)
        obs_map = self.world_to_map(robot_pos + obs_rel)

        # Raycast
        path = self.bresenham(robot_map[0], robot_map[1], obs_map[0], obs_map[1])
        # Mark path as free (0), except final cell
        for (ix, iy) in path[:-1]:
            if 0 <= ix < self.map_size and 0 <= iy < self.map_size:
                if self.occupancy_map[ix, iy] == -1:
                    self.occupancy_map[ix, iy] = 0
        # Mark obstacle
        (ox, oy) = path[-1]
        if 0 <= ox < self.map_size and 0 <= oy < self.map_size:
            self.occupancy_map[ox, oy] = 1

    def update_filter(self):
        # 1. Compute odom delta
        robot_xy = np.array([self.odometry.pose.pose.position.x,
                             self.odometry.pose.pose.position.y], dtype=float)

        if not self._have_first_odom:
            self._last_odom_xy = robot_xy.copy()
            self._have_first_odom = True
            self.particle_odom = np.zeros(2, dtype=float)
        else:
            self.particle_odom = robot_xy - self._last_odom_xy
            self._last_odom_xy = robot_xy.copy()

        # 2. Build observation from LiDAR
        obs_rel = None
        try:
            ranges = np.asarray(self.scan.ranges, dtype=float)
            valid = np.isfinite(ranges)
            if valid.any():
                r = ranges.copy()
                r[~valid] = np.inf
                i = int(np.argmin(r))
                if np.isfinite(r[i]):
                    ang = self.scan.angle_min + i * self.scan.angle_increment
                    rx = r[i] * cos(ang)
                    ry = r[i] * sin(ang)
                    yaw = quat_to_yaw(self.odometry.pose.pose.orientation)
                    cy, sy = cos(yaw), sin(yaw)
                    wx = cy * rx - sy * ry
                    wy = sy * rx + cy * ry
                    obs_rel = np.array([wx, wy])
        except Exception:
            pass

        if obs_rel is None:
            obs_rel = self.real_obstacle_position - robot_xy + np.random.normal(0, 0.05, 2)

        # 3. Update map
        self.update_map(robot_xy, obs_rel)

        # 4. Particle filter update
        self.pf.update(observed=obs_rel)

        est = np.array(self.pf.mean_state).reshape(-1)
        est_xy = est[:2] if est.size >= 2 else np.array([np.nan, np.nan])

        # 5. Yaw from odometry (true yaw)
        yaw_true = quat_to_yaw(self.odometry.pose.pose.orientation)

        # 6. Histories
        self.odom_hist.append(robot_xy.tolist())
        self.pf_hist.append(est_xy.tolist())
        self.odom_yaw_hist.append(yaw_true)
        if self.pf_yaw_est is None:
            self.pf_yaw_est = yaw_true
        self.pf_yaw_hist.append(self.pf_yaw_est)

        # 7. Estimate PF yaw by motion (if movement)
        if len(self.pf_hist) >= 2:
            dx = self.pf_hist[-1][0] - self.pf_hist[-2][0]
            dy = self.pf_hist[-1][1] - self.pf_hist[-2][1]
            if abs(dx) + abs(dy) > 1e-6:
                self.pf_yaw_est = atan2(dy, dx)

        # 8. Real-time plotting
        if len(self.odom_hist) >= 2:
            od_np = np.array(self.odom_hist)
            pf_np = np.array(self.pf_hist)

            self.odom_line.set_data(od_np[:, 0], od_np[:, 1])
            self.pf_line.set_data(pf_np[:, 0], pf_np[:, 1])

            # update heading arrows
            x_o, y_o = od_np[-1]
            x_p, y_p = pf_np[-1]
            yaw_t = yaw_true
            yaw_p = self.pf_yaw_est

            u_o = self.heading_len * cos(yaw_t)
            v_o = self.heading_len * sin(yaw_t)
            u_p = self.heading_len * cos(yaw_p)
            v_p = self.heading_len * sin(yaw_p)

            self.true_heading.set_offsets([x_o, y_o])
            self.true_heading.set_UVC(u_o, v_o)
            self.pf_heading.set_offsets([x_p, y_p])
            self.pf_heading.set_UVC(u_p, v_p)

            self.ax.relim()
            self.ax.autoscale_view()
            try:
                self.ax.set_aspect('equal', adjustable='datalim')
            except Exception:
                pass
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()

        # 9. Optionally, detect shapes periodically or on demand
        if len(self.odom_hist) % 50 == 0:  # every 50 updates
            cube_hits, cyl_hits = self.detect_shapes()
            self.get_logger().info(f"Cube hits: {cube_hits.shape}, Cylinder hits: {cyl_hits.shape}")

    def plot_final_map(self, save_path="map.png"):
        plt.figure(figsize=(8, 8))
        cmap = plt.cm.get_cmap('jet', 4)  # 4 levels: unknown, free, occupied, detections
        plt.imshow(self.occupancy_map.T, origin='lower', cmap=cmap)
        plt.colorbar(ticks=[-1, 0, 1, 2], label="Map cell value")
        plt.title("Occupancy Map")
        plt.xlabel("X grid")
        plt.ylabel("Y grid")
        plt.tight_layout()
        plt.savefig(save_path)
        self.get_logger().info(f"Saved map to {save_path}")


def main(args=None):
    rclpy.init(args=args)
    node = LidarParticleFilter()
    node.pf.init_filter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
    finally:
        node.plot_final_map()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
