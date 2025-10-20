#!/usr/bin/env python
import rclpy
from rclpy.node import Node
import sys
import time
from pfilter import (
    ParticleFilter,
    gaussian_noise,
    cauchy_noise,
    t_noise,
    squared_error,
    independent_sample,
)
import numpy as np
from scipy.stats import norm, gamma, uniform

from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from rclpy.clock import Clock
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import matplotlib.pyplot as plt
from math import sin, cos, atan2


def quat_to_yaw(q):
    """Extract yaw from a geometry_msgs/Quaternion"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return atan2(siny_cosp, cosy_cosp)


class CornerDetector:
    """Helper class to detect corner features from LiDAR scans"""
    
    def __init__(self, distance_threshold=0.15, angle_threshold=np.deg2rad(60)):
        self.distance_threshold = distance_threshold
        self.angle_threshold = angle_threshold
    
    def detect_corner(self, scan_msg):
        """
        Detect a corner in the LiDAR scan by finding two perpendicular walls.
        Returns the corner position in robot frame (x, y) or None if not detected.
        """
        ranges_np = np.array(scan_msg.ranges, dtype=float)
        n = len(ranges_np)
        angles = scan_msg.angle_min + np.arange(n) * scan_msg.angle_increment
        
        # Filter valid points (finite and within reasonable range)
        valid_mask = np.isfinite(ranges_np) & (ranges_np > 0.1) & (ranges_np < 10.0)
        valid_ranges = ranges_np[valid_mask]
        valid_angles = angles[valid_mask]
        
        if len(valid_ranges) < 10:
            return None
        
        # Convert to Cartesian coordinates
        x = valid_ranges * np.cos(valid_angles)
        y = valid_ranges * np.sin(valid_angles)
        points = np.column_stack([x, y])
        
        # Simple corner detection: find two line segments
        corners = self._find_corner_from_lines(points, valid_angles, valid_ranges)
        
        return corners
    
    def _find_corner_from_lines(self, points, angles, ranges):
        """
        Simplified corner detection using angle-based clustering.
        Assumes corner is formed by two walls roughly perpendicular.
        """
        if len(points) < 10:
            return None
        
        # Split scan into left and right halves
        n = len(angles)
        mid_idx = n // 2
        
        # Find closest point on each side (potential corner vicinity)
        left_indices = np.where(angles < 0)[0]
        right_indices = np.where(angles >= 0)[0]
        
        if len(left_indices) == 0 or len(right_indices) == 0:
            return None
        
        # Get minimum distance points from each side
        left_min_idx = left_indices[np.argmin(ranges[left_indices])]
        right_min_idx = right_indices[np.argmin(ranges[right_indices])]
        
        # Check if these points could form a corner
        p_left = points[left_min_idx]
        p_right = points[right_min_idx]
        
        # Distance between the two points
        dist = np.linalg.norm(p_left - p_right)
        
        # If points are close, estimate corner at their midpoint
        if dist < 0.5:  # Threshold for corner detection
            corner = (p_left + p_right) / 2.0
            return corner
        
        return None


class CornerParticleFilter(Node):
    '''
    ROS Node that estimates absolute position of robot using odometry and 
    corner detection from LiDAR, given a known corner position in the world.
    '''
    def __init__(self):
        # Init node
        super().__init__('corner_position_pf')
        
        # Define QoS profile
        self.qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=10,
        )
        
        # Particle filter params
        self.declare_parameters(
            namespace='',
            parameters=[
                ("weights_sigma", 1.2),
                ("num_particles", 200),
                ("measurement_noise", 0.05),
                ("resample_proportion", 0.01),
                ("corner_x", 2.0),  # Known corner position in world frame
                ("corner_y", 2.0),
            ]
        )
        
        self.weights_sigma = self.get_parameter('weights_sigma').value
        self.num_particles = self.get_parameter('num_particles').value
        self.measurement_noise = self.get_parameter('measurement_noise').value
        self.resample_proportion = self.get_parameter('resample_proportion').value
        self.corner_world = np.array([
            self.get_parameter('corner_x').value,
            self.get_parameter('corner_y').value
        ])
        
        self.get_logger().info(
            'PF params - sigma: %.2f, particles: %d, noise: %.3f, resample: %.3f' % 
            (self.weights_sigma, self.num_particles, 
             self.measurement_noise, self.resample_proportion)
        )
        self.get_logger().info('Known corner position: [%.2f, %.2f]' % 
                              (self.corner_world[0], self.corner_world[1]))
        
        # Create corner detector
        self.corner_detector = CornerDetector()
        
        # Create filter - prior covers expected area
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
        
        # Create subscribers
        self.get_logger().info("Creating subscribers")
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odometry_cb, qos_profile=self.qos)
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_cb, qos_profile=self.qos)
        
        self.odometry = Odometry()
        self.scan = LaserScan()
        self.corner_detected = None
        
        # Internal state for odom delta
        self._have_first_odom = False
        self._last_odom_xy = np.zeros(2, dtype=float)
        self.particle_odom = np.array([0.0, 0.0], dtype=float)
        
        # History for plotting
        self.odom_hist = []
        self.pf_hist = []
        self.pf_yaw_est = None
        self.odom_yaw_hist = []
        self.pf_yaw_hist = []
        
        # Live plots
        self.live_plot = True
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(8, 7))
        (self.odom_line,) = self.ax.plot([], [], 'b-', label="Odometry", alpha=0.6)
        (self.pf_line,) = self.ax.plot([], [], 'r-', linewidth=2, label="Particle Filter")
        
        # Plot the known corner
        self.ax.plot(self.corner_world[0], self.corner_world[1], 
                    'gs', markersize=15, label="Known Corner", markeredgewidth=2)
        
        self.heading_len = 0.3
        self.true_heading = self.ax.quiver(0, 0, 0, 0, angles='xy', 
                                          scale_units='xy', scale=1, 
                                          color='blue', label="Odom heading")
        self.pf_heading = self.ax.quiver(0, 0, 0, 0, angles='xy', 
                                        scale_units='xy', scale=1, 
                                        color='red', label="PF heading")
        
        self.ax.legend()
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")
        self.ax.set_title("Corner-Based Localization: PF vs. Odometry")
        self.ax.grid(True, alpha=0.3)
        self.fig.show()
        self.fig.canvas.flush_events()
        
        self.get_logger().info("Particle filter initialized")
    
    def odometry_cb(self, odom):
        self.odometry = odom
    
    def scan_cb(self, scan):
        self.scan = scan
        # Detect corner in robot frame
        self.corner_detected = self.corner_detector.detect_corner(scan)
    
    def calc_hypothesis(self, x):
        '''
        Given (Nx2) matrix of robot positions,
        create N arrays of observations (what each particle would see).
        
        Observation is the vector from particle position to corner in WORLD coordinates.
        '''
        y = self.corner_world - x  # world vector from particle to corner
        return y
    
    def velocity(self, x):
        '''
        Use odometry to update robot position.
        Apply odom translation delta (world frame) to every particle.
        '''
        xp = x + self.particle_odom
        return xp
    
    def add_noise(self, x):
        '''
        Add process noise to particle positions.
        '''
        xp = x + np.random.normal(0, self.measurement_noise, x.shape)
        return xp
    
    def calc_weights(self, hypotheses, observations):
        '''
        Calculate particle weights based on observation error.
        '''
        w = squared_error(hypotheses, observations, sigma=self.weights_sigma)
        return w
    
    def update_filter(self):
        '''
        Update particle filter with odometry and corner observations.
        '''
        # --- 1) Compute odometry delta (WORLD frame) ---
        robot_pos = np.array([
            self.odometry.pose.pose.position.x,
            self.odometry.pose.pose.position.y
        ], dtype=float)
        
        if not self._have_first_odom:
            self._last_odom_xy = robot_pos.copy()
            self._have_first_odom = True
            self.particle_odom = np.array([0.0, 0.0], dtype=float)
        else:
            self.particle_odom = robot_pos - self._last_odom_xy
            self._last_odom_xy = robot_pos.copy()
        
        # --- 2) Build observation from corner detection ---
        corner_observation = None
        
        if self.corner_detected is not None:
            # Convert corner from robot frame to world frame
            q = self.odometry.pose.pose.orientation
            yaw = quat_to_yaw(q)
            cy, sy = np.cos(yaw), np.sin(yaw)
            
            # Corner position in robot frame
            cx_r, cy_r = self.corner_detected[0], self.corner_detected[1]
            
            # Transform to world frame and get observation vector
            # (vector from robot to corner in world frame)
            wx = cy * cx_r - sy * cy_r
            wy = sy * cx_r + cy * cy_r
            corner_observation = np.array([wx, wy], dtype=float)
            
            self.get_logger().info('Corner detected at [%.2f, %.2f] in robot frame' % 
                                  (cx_r, cy_r))
        else:
            # Fallback: use known corner with added noise
            corner_observation = self.corner_world - robot_pos + \
                               np.random.normal(0, 0.1, 2)
        
        # --- 3) PF update with observation ---
        self.pf.update(observed=corner_observation)
        
        # PF estimate (mean of particles)
        est = np.array(self.pf.mean_state).reshape(-1)
        est_xy = est[:2] if est.size >= 2 else np.array([np.nan, np.nan])
        
        self.get_logger().info('Robot position (PF): [%.2f, %.2f]' % 
                              (est_xy[0], est_xy[1]))
        
        # --- True orientation from odom ---
        q = self.odometry.pose.pose.orientation
        yaw_true = quat_to_yaw(q)
        
        # --- Append positions for plotting ---
        self.odom_hist.append(robot_pos.tolist())
        self.pf_hist.append(est_xy.tolist())
        
        # --- Estimated orientation from PF displacement ---
        if len(self.pf_hist) >= 2:
            dx = self.pf_hist[-1][0] - self.pf_hist[-2][0]
            dy = self.pf_hist[-1][1] - self.pf_hist[-2][1]
            if abs(dx) + abs(dy) > 1e-6:
                self.pf_yaw_est = atan2(dy, dx)
        
        self.odom_yaw_hist.append(yaw_true)
        self.pf_yaw_hist.append(self.pf_yaw_est if self.pf_yaw_est is not None else yaw_true)
        
        # --- Live plot update ---
        if self.live_plot and len(self.odom_hist) >= 2:
            odom_np = np.array(self.odom_hist)
            pf_np = np.array(self.pf_hist)
            
            # Update trajectories
            self.odom_line.set_data(odom_np[:, 0], odom_np[:, 1])
            self.pf_line.set_data(pf_np[:, 0], pf_np[:, 1])
            
            # Update heading arrows
            x_o, y_o = odom_np[-1, 0], odom_np[-1, 1]
            x_p, y_p = pf_np[-1, 0], pf_np[-1, 1]
            
            u_o = self.heading_len * np.cos(yaw_true)
            v_o = self.heading_len * np.sin(yaw_true)
            
            yaw_pf = self.pf_yaw_est if self.pf_yaw_est is not None else yaw_true
            u_p = self.heading_len * np.cos(yaw_pf)
            v_p = self.heading_len * np.sin(yaw_pf)
            
            self.true_heading.set_offsets([x_o, y_o])
            self.true_heading.set_UVC(u_o, v_o)
            self.pf_heading.set_offsets([x_p, y_p])
            self.pf_heading.set_UVC(u_p, v_p)
            
            # Autoscale and redraw
            self.ax.relim()
            self.ax.autoscale_view()
            try:
                self.ax.set_aspect('equal', adjustable='datalim')
            except Exception:
                pass
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    filter_node = CornerParticleFilter()
    
    # Init filter
    filter_node.pf.init_filter()
    
    time.sleep(1)
    
    filter_node.get_logger().info("Starting particle filter...")
    
    try:
        # Create timer for filter updates (5 Hz)
        filter_timer = filter_node.create_timer(0.2, filter_node.update_filter)
        rclpy.spin(filter_node)
    except KeyboardInterrupt:
        filter_node.get_logger().error('Keyboard Interrupt detected!')
    except Exception as e:
        filter_node.get_logger().error("Particle filter failed: %r" % (e,))
    finally:
        filter_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()