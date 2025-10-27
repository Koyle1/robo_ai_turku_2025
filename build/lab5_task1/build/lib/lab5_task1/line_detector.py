#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2 as pc2

# corners
CORNER_STEP1 = 3
CORNER_STEP2 = 8
CORNER_TOL_DEG = 12.0     # accept ~90° ± 12°
CORNER_NMS_R  = 6         # keep only the best per neighborhood

# lines
GROUP_SIZE     = 8        
RMS_THRESH     = 0.05    
SLOPE_TOL      = 0.10
INTERCEPT_TOL  = 0.10

class LineCornerDetector(Node):
    def __init__(self):
        super().__init__('line_corner_detector_min')
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.pub_lines = self.create_publisher(PointCloud2, '/line_points', 10)
        self.pub_corners = self.create_publisher(PointCloud2, '/corner_points', 10)

    def scan_cb(self, msg: LaserScan):
        # Polar → Cartesian (needed for polyfit and angle math)
        ranges = np.asarray(msg.ranges, dtype=float)
        n = ranges.size
        angles = msg.angle_min + np.arange(n) * msg.angle_increment
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        idx = np.arange(n)

        # slice the data into groups and there check if the groub is a straight wall
        windows = []
        for i in range(0, n - GROUP_SIZE + 1):
            sl = slice(i, i + GROUP_SIZE)
            xi = x[sl]; yi = y[sl]

            # Fit model A: y = m*x + b
            m1, b1 = np.polyfit(xi, yi, deg=1)
            resid1 = np.mean((yi - (m1 * xi + b1)) ** 2)

            # Fit model B: x = m*y + b 
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

        # concat the values into one straigh lines
        if windows:
            merged = []
            s, e, t, m, b, _ = windows[0]
            cur_pts = list(range(idx[s], idx[e] + 1))
            for (s2, e2, t2, m2, b2, _) in windows[1:]:
                # check if they are close together
                if (t2 == t) and (abs(m2 - m) <= SLOPE_TOL) and (abs(b2 - b) <= INTERCEPT_TOL):
                    cur_pts.extend(range(idx[s2], idx[e2] + 1))
                    # average the values to get a smooth value
                    m = 0.5 * (m + m2); b = 0.5 * (b + b2)
                else:
                    merged.extend(cur_pts)
                    s, e, t, m, b = s2, e2, t2, m2, b2
                    cur_pts = list(range(idx[s], idx[e] + 1))
            merged.extend(cur_pts)
            line_idx_vals = np.unique(np.array(merged, dtype=int))

        # -------------------------CORNERS------------------------------------
        tol = np.deg2rad(CORNER_TOL_DEG)
        candidates = [] 
        # selct three points and check the angle
        for i in range(CORNER_STEP1, n - CORNER_STEP2):
            # a = i - step1, b = i, c = i + step2
            ax, ay = x[i - CORNER_STEP1], y[i - CORNER_STEP1]
            bx, by = x[i],                 y[i]
            cx, cy = x[i + CORNER_STEP2], y[i + CORNER_STEP2]

            ba = np.array([ax - bx, ay - by])
            bc = np.array([cx - bx, cy - by])
            n1 = np.linalg.norm(ba); n2 = np.linalg.norm(bc)
            if n1 == 0.0 or n2 == 0.0:
                continue

            cosine_angle = float(np.clip(np.dot(ba, bc) / (n1 * n2), -1.0, 1.0))
            angle = np.arccos(cosine_angle)
            err = abs(angle - (np.pi / 2.0))
            if err <= tol:
                candidates.append((idx[i], err, i))

        # keep best angle for each corner
        corner_idx_vals = np.array([], dtype=int)
        if candidates:
            candidates.sort(key=lambda t: t[1])    # best (closest to 90°) first
            taken = np.zeros(n, dtype=bool)
            keep = []
            for orig_idx, err, loc in candidates:
                if taken[loc]:
                    continue
                keep.append(orig_idx)
                lo = max(0, loc - CORNER_NMS_R)
                hi = min(n - 1, loc + CORNER_NMS_R)
                taken[lo:hi+1] = True
            corner_idx_vals = np.array(keep, dtype=int)

        # ---------- publish as XYZ clouds (z=0) in same frame as /scan ----------
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id or 'base_scan'

        line_pts = np.stack([x[line_idx_vals], y[line_idx_vals]], axis=1) if line_idx_vals.size else np.zeros((0,2), np.float32)
        corner_pts = np.stack([x[corner_idx_vals], y[corner_idx_vals]], axis=1) if corner_idx_vals.size else np.zeros((0,2), np.float32)

        self.pub_lines.publish(pc2.create_cloud_xyz32(header, [(float(px), float(py), 0.0) for px, py in line_pts]))
        self.pub_corners.publish(pc2.create_cloud_xyz32(header, [(float(px), float(py), 0.0) for px, py in corner_pts]))

def main(args=None):
    rclpy.init(args=args)
    node = LineCornerDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

