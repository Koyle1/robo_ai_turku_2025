#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, qos_profile_sensor_data

import numpy as np
from math import sin, cos, atan2
from pfilter import ParticleFilter, squared_error

from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2 as pc2

import matplotlib.pyplot as plt

# === Line & Corner Detection Constants ===
CORNER_STEP1 = 3
CORNER_STEP2 = 8
CORNER_TOL_DEG = 12.0
CORNER_NMS_R = 6

GROUP_SIZE = 8
RMS_THRESH = 0.05
SLOPE_TOL = 0.10
INTERCEPT_TOL = 0.10


def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return atan2(siny_cosp, cosy_cosp)


class LidarParticleFilter(Node):
    def __init__(self):
        super().__init__('lidar_position_pf_rclpy')

        self.qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=10,
        )

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

        self.real_obstacle_position = np.array([2.0, 1])
        self.second_obstacle_position = np.array([2.0, -1])

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

        self.particle_odom = np.array([0.0, 0.0])
        self._have_first_odom = False
        self._last_odom_xy = np.zeros(2)

        self.odom_hist = []
        self.pf_hist = []
        self.pf_yaw_est = None

        self.odometry = Odometry()
        self.scan = LaserScan()

        self.create_subscription(Odometry, "/odom", self.odometry_cb, qos_profile=self.qos)
        self.create_subscription(LaserScan, "/scan", self.scan_cb, qos_profile=qos_profile_sensor_data)

        self.pub_lines = self.create_publisher(PointCloud2, '/line_points', 10)
        self.pub_corners = self.create_publisher(PointCloud2, '/corner_points', 10)

        self.create_timer(0.2, self.update_filter)

        # === Plot Setup ===
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(7, 6))
        self.odom_line, = self.ax.plot([], [], label="Odometry")
        self.pf_line, = self.ax.plot([], [], label="PF Estimate")
        self.ax.scatter(*self.real_obstacle_position, color='red', marker='x', label="Obstacle 1")
        self.ax.scatter(*self.second_obstacle_position, color='green', marker='x', label="Obstacle 2")
        self.ax.legend()
        self.ax.set_title("Real-Time Localization")
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")
        self.ax.grid(True)
        self.ax.axis('equal')
        self.fig.tight_layout()

        self.get_logger().info("Particle filter + line/corner detector initialized")

    def odometry_cb(self, msg):
        self.odometry = msg

    def scan_cb(self, msg: LaserScan):
        self.scan = msg
        self.process_lines_and_corners(msg)

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

    def _two_scan_vectors_to_world(self):
        ranges = np.asarray(self.scan.ranges, dtype=float)
        if ranges.size < 2 or not np.any(np.isfinite(ranges)):
            return None

        valid_idx = np.where(np.isfinite(ranges))[0]
        if len(valid_idx) < 2:
            return None

        closest_indices = valid_idx[np.argsort(ranges[valid_idx])[:2]]

        q = self.odometry.pose.pose.orientation
        yaw = quat_to_yaw(q)
        cy, sy = cos(yaw), sin(yaw)

        vectors = []
        for i in closest_indices:
            ang = self.scan.angle_min + i * self.scan.angle_increment
            r = ranges[i]
            rx = r * cos(ang)
            ry = r * sin(ang)

            wx = cy * rx - sy * ry
            wy = sy * rx + cy * ry
            vectors.extend([wx, wy])
        return np.array(vectors, dtype=float)

    def update_filter(self):
        robot_pos = np.array([self.odometry.pose.pose.position.x,
                              self.odometry.pose.pose.position.y], dtype=float)

        if not self._have_first_odom:
            self._last_odom_xy = robot_pos.copy()
            self._have_first_odom = True
            self.particle_odom = np.array([0.0, 0.0])
        else:
            self.particle_odom = robot_pos - self._last_odom_xy
            self._last_odom_xy = robot_pos.copy()

        obs_vec_world = self._two_scan_vectors_to_world()
        if obs_vec_world is None:
            obs1 = self.real_obstacle_position - robot_pos + np.random.normal(0, 0.05, 2)
            obs2 = self.second_obstacle_position - robot_pos + np.random.normal(0, 0.05, 2)
            obs_vec_world = np.hstack((obs1, obs2))

        self.pf.update(observed=obs_vec_world)

        est = np.array(self.pf.mean_state).reshape(-1)
        est_xy = est[:2] if est.size >= 2 else np.array([np.nan, np.nan])

        self.odom_hist.append(robot_pos.tolist())
        self.pf_hist.append(est_xy.tolist())

        if len(self.pf_hist) >= 2:
            dx = self.pf_hist[-1][0] - self.pf_hist[-2][0]
            dy = self.pf_hist[-1][1] - self.pf_hist[-2][1]
            if abs(dx) + abs(dy) > 1e-6:
                self.pf_yaw_est = atan2(dy, dx)

        self.get_logger().info(
            f"PF estimate: x={est_xy[0]:.2f}, y={est_xy[1]:.2f}, yaw={self.pf_yaw_est:.2f}"
            if self.pf_yaw_est else
            f"PF estimate: x={est_xy[0]:.2f}, y={est_xy[1]:.2f}"
        )

        self.plot_realtime()

    def process_lines_and_corners(self, msg: LaserScan):
        ranges = np.asarray(msg.ranges, dtype=float)
        n = ranges.size
        angles = msg.angle_min + np.arange(n) * msg.angle_increment
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        idx = np.arange(n)

        # Line Detection
        windows = []
        for i in range(0, n - GROUP_SIZE + 1):
            sl = slice(i, i + GROUP_SIZE)
            xi = x[sl]; yi = y[sl]

            m1, b1 = np.polyfit(xi, yi, deg=1)
            resid1 = np.mean((yi - (m1 * xi + b1)) ** 2)

            m2, b2 = np.polyfit(yi, xi, deg=1)
            resid2 = np.mean((xi - (m2 * yi + b2)) ** 2)

            if resid1 <= resid2:
                rms = float(np.sqrt(resid1))
                if rms <= RMS_THRESH:
                    windows.append((i, i + GROUP_SIZE - 1, 'y=ax+b', float(m1), float(b1), rms))
            else:
                rms = float(np.sqrt(resid2))
                if rms <= RMS_THRESH:
                    windows.append((i, i + GROUP_SIZE - 1, 'x=ay+b', float(m2), float(b2), rms))

        line_idx_vals = np.array([], dtype=int)
        if windows:
            merged = []
            s, e, t, m, b, _ = windows[0]
            cur_pts = list(range(idx[s], idx[e] + 1))
            for (s2, e2, t2, m2, b2, _) in windows[1:]:
                if (t2 == t) and (abs(m2 - m) <= SLOPE_TOL) and (abs(b2 - b) <= INTERCEPT_TOL):
                    cur_pts.extend(range(idx[s2], idx[e2] + 1))
                    m = 0.5 * (m + m2)
                    b = 0.5 * (b + b2)
                else:
                    merged.extend(cur_pts)
                    s, e, t, m, b = s2, e2, t2, m2, b2
                    cur_pts = list(range(idx[s], idx[e] + 1))
            merged.extend(cur_pts)
            line_idx_vals = np.unique(np.array(merged, dtype=int))

        # Corner Detection
        tol = np.deg2rad(CORNER_TOL_DEG)
        candidates = []
        for i in range(CORNER_STEP1, n - CORNER_STEP2):
            ax, ay = x[i - CORNER_STEP1], y[i - CORNER_STEP1]
            bx, by = x[i], y[i]
            cx, cy = x[i + CORNER_STEP2], y[i + CORNER_STEP2]

            ba = np.array([ax - bx, ay - by])
            bc = np.array([cx - bx, cy - by])
            n1 = np.linalg.norm(ba)
            n2 = np.linalg.norm(bc)
            if n1 == 0.0 or n2 == 0.0:
                continue

            cosine_angle = float(np.clip(np.dot(ba, bc) / (n1 * n2), -1.0, 1.0))
            angle = np.arccos(cosine_angle)
            err = abs(angle - (np.pi / 2.0))
            if err <= tol:
                candidates.append((idx[i], err, i))

        corner_idx_vals = np.array([], dtype=int)
        if candidates:
            candidates.sort(key=lambda t: t[1])
            taken = np.zeros(n, dtype=bool)
            keep = []
            for orig_idx, err, loc in candidates:
                if taken[loc]:
                    continue
                keep.append(orig_idx)
                lo = max(0, loc - CORNER_NMS_R)
                hi = min(n - 1, loc + CORNER_NMS_R)
                taken[lo:hi + 1] = True
            corner_idx_vals = np.array(keep, dtype=int)

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id or 'base_scan'

        line_pts = np.stack([x[line_idx_vals], y[line_idx_vals]], axis=1) if line_idx_vals.size else np.zeros((0, 2), np.float32)
        corner_pts = np.stack([x[corner_idx_vals], y[corner_idx_vals]], axis=1) if corner_idx_vals.size else np.zeros((0, 2), np.float32)

        self.pub_lines.publish(pc2.create_cloud_xyz32(header, [(float(px), float(py), 0.0) for px, py in line_pts]))
        self.pub_corners.publish(pc2.create_cloud_xyz32(header, [(float(px), float(py), 0.0) for px, py in corner_pts]))

    def plot_realtime(self):
        if len(self.odom_hist) < 2 or len(self.pf_hist) < 2:
            return

        odom_np = np.array(self.odom_hist)
        pf_np = np.array(self.pf_hist)

        self.odom_line.set_data(odom_np[:, 0], odom_np[:, 1])
        self.pf_line.set_data(pf_np[:, 0], pf_np[:, 1])

        all_x = np.concatenate((odom_np[:, 0], pf_np[:, 0]))
        all_y = np.concatenate((odom_np[:, 1], pf_np[:, 1]))

        self.ax.set_xlim(np.min(all_x) - 0.5, np.max(all_x) + 0.5)
        self.ax.set_ylim(np.min(all_y) - 0.5, np.max(all_y) + 0.5)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    node = LidarParticleFilter()
    node.pf.init_filter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down on keyboard interrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
