#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
from math import sin, cos, atan2, sqrt
from scipy.ndimage import label, center_of_mass
from scipy.spatial.distance import euclidean

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from skimage.feature import canny
from skimage.transform import probabilistic_hough_line

import matplotlib
# matplotlib.use("Agg")
import matplotlib.pyplot as plt


def quat_to_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return atan2(siny, cosy)


def fit_circle(xs, ys):
    x = xs[:, np.newaxis]
    y = ys[:, np.newaxis]
    A = np.hstack((2 * x, 2 * y, np.ones_like(x)))
    b = x * x + y * y
    sol, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    xc, yc, c = sol.flatten()
    r = sqrt(c + xc*xc + yc*yc)
    d2 = (xs - xc)**2 + (ys - yc)**2
    mse = np.mean((np.sqrt(d2) - r)**2)
    return xc, yc, r, mse


def fit_line(xs, ys):
    A = np.vstack([xs, np.ones_like(xs)]).T
    m, b = np.linalg.lstsq(A, ys, rcond=None)[0]
    y_fit = m * xs + b
    mse = np.mean((ys - y_fit)**2)
    return m, b, mse


class ShapeMapper(Node):
    def __init__(self):
        super().__init__('shape_mapper')

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=10,
        )

        self.map_size = 100
        self.grid_resolution = 0.1
        self.map_center = np.array([self.map_size // 2, self.map_size // 2], dtype=int)

        self.occupancy_map = -1 * np.ones((self.map_size, self.map_size), dtype=int)

        self.create_subscription(Odometry, "/odom", self.odom_cb, qos)
        self.create_subscription(LaserScan, "/scan", self.scan_cb, qos)

        self.odometry = Odometry()
        self.scan = LaserScan()
        self.have_odom = False

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        self.img = self.ax.imshow((self.occupancy_map + 1).T,
                                  origin='lower',
                                  cmap=plt.cm.get_cmap('tab10', 5),
                                  vmin=0, vmax=4)
        self.ax.set_title("Live Map")
        self.fig.colorbar(self.img, ax=self.ax, ticks=[0, 1, 2, 3, 4])
        self.fig.canvas.draw()

        self.create_timer(0.2, self.timer_callback)

    def odom_cb(self, msg):
        self.odometry = msg
        self.have_odom = True

    def scan_cb(self, msg):
        self.scan = msg

    def world_to_map(self, pos):
        i = int(round(pos[0] / self.grid_resolution)) + self.map_center[0]
        j = int(round(pos[1] / self.grid_resolution)) + self.map_center[1]
        return i, j

    def bresenham(self, x0, y0, x1, y1):
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

    def update_map_from_scan(self, robot_xy, obs_rel):
        rm = self.world_to_map(robot_xy)
        om = self.world_to_map(robot_xy + obs_rel)
        path = self.bresenham(rm[0], rm[1], om[0], om[1])

        for (ix, iy) in path[:-1]:
            if 0 <= ix < self.map_size and 0 <= iy < self.map_size:
                if self.occupancy_map[ix, iy] == -1:
                    self.occupancy_map[ix, iy] = 0

        ox, oy = path[-1]
        if 0 <= ox < self.map_size and 0 <= oy < self.map_size:
            self.occupancy_map[ox, oy] = 1

    def merge_blobs(self, labeled, num):
        blobs = []
        for i in range(1, num + 1):
            coords = np.argwhere(labeled == i)
            if coords.size == 0:
                continue
            blobs.append(coords)

        def bbox(bl):
            xs = bl[:, 0]
            ys = bl[:, 1]
            return (xs.min(), xs.max(), ys.min(), ys.max())

        merged = []
        used = [False] * len(blobs)
        for i, b1 in enumerate(blobs):
            if used[i]:
                continue
            used[i] = True
            (x0, x1, y0, y1) = bbox(b1)
            group = [b1]
            for j, b2 in enumerate(blobs):
                if used[j]:
                    continue
                (u0, u1, v0, v1) = bbox(b2)
                margin = 2
                if not (u0 > x1 + margin or u1 < x0 - margin or v0 > y1 + margin or v1 < y0 - margin):
                    group.append(b2)
                    used[j] = True
            merged_coords = np.vstack(group)
            merged.append(merged_coords)
        return merged

    def detect_and_classify(self):
        occ = (self.occupancy_map == 1).astype(np.uint8)
        if np.sum(occ) == 0:
            return

        edges = canny(occ, sigma=1.0)
        lines = probabilistic_hough_line(edges, threshold=10, line_length=5, line_gap=3)

        labeled, n = label(occ)
        merged_blobs = self.merge_blobs(labeled, n)
        self.get_logger().info(f"Merged into {len(merged_blobs)} objects")

        mask = (self.occupancy_map == 2) | (self.occupancy_map == 3)
        self.occupancy_map[mask] = 1

        MIN_BLOB_SIZE_FOR_CLASS = 20
        LINE_MSE_THRESHOLD = 2.0
        CIRC_MSE_THRESHOLD = 2.0
        RATIO_THRESHOLD_CYL = 3.0
        RATIO_THRESHOLD_CUBE = 0.8
        RADIUS_MIN = 2.0
        RADIUS_MAX = 20.0

        for blob in merged_blobs:
            size = blob.shape[0]
            if size < MIN_BLOB_SIZE_FOR_CLASS:
                continue

            xs = blob[:, 0].astype(float)
            ys = blob[:, 1].astype(float)

            m, b, mse_line = fit_line(xs, ys)
            xc, yc, r, mse_circ = fit_circle(xs, ys)

            dists = np.sqrt((xs - xc)**2 + (ys - yc)**2)
            std_dist = np.std(dists)

            ratio = mse_line / (mse_circ + 1e-8)

            self.get_logger().info(
                f"Blob size={size} line_mse={mse_line:.3f}, circ_mse={mse_circ:.3f}, "
                + f"ratio={ratio:.3f}, r={r:.3f}, std_dist={std_dist:.3f}"
            )

            shape_label = 1

            if (mse_circ < CIRC_MSE_THRESHOLD
                and ratio > RATIO_THRESHOLD_CYL
                and RADIUS_MIN < r < RADIUS_MAX
                and std_dist / (r + 1e-8) < 0.2):
                shape_label = 3
            elif (mse_line < LINE_MSE_THRESHOLD
                  and ratio < RATIO_THRESHOLD_CUBE):
                shape_label = 2

            for (xi, yi) in blob:
                self.occupancy_map[xi, yi] = shape_label

    def timer_callback(self):
        if not self.have_odom or len(self.scan.ranges) == 0:
            return

        rx = self.odometry.pose.pose.position.x
        ry = self.odometry.pose.pose.position.y
        robot_xy = np.array([rx, ry], dtype=float)

        try:
            ranges = np.asarray(self.scan.ranges, dtype=float)
            valid = np.isfinite(ranges)
            if valid.any():
                rr = ranges.copy()
                rr[~valid] = np.inf
                i = int(np.argmin(rr))
                if np.isfinite(rr[i]):
                    angle = self.scan.angle_min + i * self.scan.angle_increment
                    lx = rr[i] * cos(angle)
                    ly = rr[i] * sin(angle)
                    q = self.odometry.pose.pose.orientation
                    yaw = quat_to_yaw(q)
                    cy, sy = cos(yaw), sin(yaw)
                    wx = cy * lx - sy * ly
                    wy = sy * lx + cy * ly
                    obs_rel = np.array([wx, wy])

                    self.update_map_from_scan(robot_xy, obs_rel)
        except Exception as e:
            self.get_logger().warn(f"Scan error: {e}")

        if np.random.rand() < 0.2:
            self.detect_and_classify()

        disp = (self.occupancy_map + 1).T
        self.img.set_data(disp)
        self.img.set_cmap(plt.cm.get_cmap('tab10', 5))
        self.img.set_clim(0, 4)
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def report_objects(self):
        self.detect_and_classify()
        for label_val, name in [(2, "Cube"), (3, "Cylinder")]:
            binary = (self.occupancy_map == label_val).astype(int)
            labeled, n = label(binary)
            centers = center_of_mass(binary, labeled, range(1, n + 1))
            for idx, (cy, cx) in enumerate(centers):
                wx = (cx - self.map_center[1]) * self.grid_resolution
                wy = (cy - self.map_center[0]) * self.grid_resolution
                size = np.sum(labeled == (idx + 1))
                self.get_logger().info(
                    f"{name} #{idx+1}: Position=({wx:.2f}, {wy:.2f}), Size={size}"
                )

    def save_and_shutdown(self):
        np.save("occupancy_map.npy", self.occupancy_map)
        plt.ioff()
        plt.savefig("final_map.png")
        self.get_logger().info("Saved final map and occupancy_map.npy")
        self.report_objects()


def main(args=None):
    rclpy.init(args=args)
    node = ShapeMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().inf_
