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


from std_msgs.msg       import Float64
from sensor_msgs.msg    import LaserScan
from geometry_msgs.msg  import PoseStamped
from geometry_msgs.msg  import PoseWithCovarianceStamped
from geometry_msgs.msg  import PoseArray
from sensor_msgs.msg    import Range
from nav_msgs.msg       import Odometry
from rclpy.clock        import Clock
from rclpy.duration     import Duration
from pfilter            import ParticleFilter, squared_error




from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy




import matplotlib.pyplot as plt
from math import sin, cos, atan2




def quat_to_yaw(q):
    """Extract yaw from a geometry_msgs/Quaternion"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return atan2(siny_cosp, cosy_cosp)




class LidarParticleFilter(Node) :
    '''
        ROS Node that estimates relative position of two robots using odometry and lidar detections
        given a known map or set of objects in known positions
    '''


    def __init__(self) :


        # Init node
        super().__init__('lidar_position_pf_rclpy')


        # Define QoS profile for odom and UWB subscribers
        self.qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=10,
        )


        # Particle filter params
        # Parameters for ros2 launch command
        self.declare_parameters(
            namespace = '',
            parameters=[
                ("weights_sigma", 1.2),
                ("num_particles", 200),
                ("measurement_noise", 0.05),
                ("resample_proportion", 0.01),
                ("max_pos_delay", 0.2)
            ]
        )
        
        self.weights_sigma = self.get_parameter('weights_sigma').value
        self.num_particles = self.get_parameter('num_particles').value
        self.measurement_noise = self.get_parameter('measurement_noise').value
        self.resample_proportion = self.get_parameter('resample_proportion').value


        self.get_logger().info('weights_sigma: %f, num_particles: %d, measurement_noise: %f, resample_proportion: %f'  % 
                            (self.weights_sigma,
                             self.num_particles,
                             self.measurement_noise,
                             self.resample_proportion))




        # Create filter
        self.prior_fn = lambda n: np.random.uniform(-5,5,(n,2))
        self.pf = ParticleFilter(
            prior_fn =              self.prior_fn, 
            observe_fn =            self.calc_hypothesis,  
            dynamics_fn =           self.velocity,  # implement motion model from odom delta
            n_particles =           self.num_particles, 
            noise_fn =              self.add_noise,
            weight_fn =             self.calc_weights,
            resample_proportion =   self.resample_proportion
        )

        self.get_logger().info("Creating subscribers")
        self.odom_sub = self.create_subscription(Odometry, "/odom",  self.odometry_cb, qos_profile=self.qos)
        self.scan_sub = self.create_subscription(LaserScan, "/scan",  self.scan_cb, qos_profile=self.qos)


        self.odometry = Odometry()
        self.scan = LaserScan()


        # Real obstacle position <- NOTE! This depends on your simulator
        self.real_obstacle_position = np.array([1.5, 0]) # TODO! Remember to change this!!!


        # Internal state for odom delta
        self._have_first_odom = False
        self._last_odom_xy = np.zeros(2, dtype=float)


        # --- NEW: Trajectory buffers for plotting + orientation estimate ---
        self.odom_hist = []     # list of [x, y]
        self.pf_hist = []       # list of [x, y]
        self.pf_yaw_est = None  # simple orientation estimate from PF displacement

        self.odom_yaw_hist = []
        self.pf_yaw_hist   = []
        self.pf_yaw_est    = None  # keep last good estimate



        # --- Live plot setup (minimal) ---
        self.live_plot = True
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(7,6))
        (self.odom_line,) = self.ax.plot([], [], label="Odometry (sim)")
        (self.pf_line,)   = self.ax.plot([], [], label="Particle Filter (estimate)")
        self.heading_len = 0.3  # arrow length in meters
        # quiver with 1 arrow each; start at origin, will update every tick

        self.true_heading = self.ax.quiver(0, 0, 0, 0, angles='xy', scale_units='xy', scale=1, label="Odom heading")
        self.pf_heading   = self.ax.quiver(0, 0, 0, 0, angles='xy', scale_units='xy', scale=1, label="PF heading")

        # refresh legend to include quivers (quiver doesn't always add itself automatically)
        handles, labels = self.ax.get_legend_handles_labels()
        self.ax.legend(handles, labels)
        self.ax.set_xlabel("x [m]"); self.ax.set_ylabel("y [m]")
        self.ax.set_title("Task 4: Localization — PF vs. Odometry (Live)")
        self.ax.grid(True, alpha=0.3)
        self.fig.show()
        self.fig.canvas.flush_events()



        self.get_logger().info("Particle filter initialized")


    def odometry_cb(self, odom):
        self.odometry = odom


    def scan_cb(self, scan):
        self.scan = scan


    def calc_hypothesis(self, x) :
        '''
            Given (Nx2) matrix of positions,
            create N arrays of observations
            (observations are what the particles WOULD see from their position given in 'x')


            Task-4 model: observation is the vector from particle->landmark in WORLD coordinates.
            Here, the landmark is the known obstacle position (set from the sim).
        '''        
        y = self.real_obstacle_position - x  # world vector from particle to obstacle
        return y


    def velocity(self, x) :
        '''
            Use odometry to update robot position.
            We apply the same odom translation delta (world frame) to every particle.
            The delta is computed in update_filter() and stored in self.particle_odom.
        '''
        xp = x + self.particle_odom
        return xp


    def add_noise(self, x) :
        '''
            Add noise to the estimations (process noise).
        '''
        xp = x + np.random.normal(0, self.measurement_noise, x.shape)
        return xp


    def calc_weights(self, hypotheses, observations) :
        '''
            Calculate particle weights based on error
            (how close each particle's predicted landmark vector is to the observed landmark vector).
        '''
        w = squared_error(hypotheses, observations, sigma=self.weights_sigma)
        return w


    def update_filter(self) :
        '''
            Upadate particle filter
        '''


        print("Robot pose {}".format(self.odometry.pose.pose.position))


        # --- 1) compute odometry delta -> self.particle_odom (WORLD frame) ---
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


        # --- 2) build observation from LiDAR (vector robot->obstacle in WORLD) ---
        relative_obstacle_pos = None
        try:
            ranges = np.asarray(self.scan.ranges, dtype=float)
            if ranges.size > 0:
                r = ranges.copy()
                r[~np.isfinite(r)] = np.inf
                i = int(np.argmin(r))
                if np.isfinite(r[i]):
                    ang = self.scan.angle_min + i * self.scan.angle_increment
                    rx = r[i] * np.cos(ang)
                    ry = r[i] * np.sin(ang)
                    q = self.odometry.pose.pose.orientation
                    yaw = quat_to_yaw(q)
                    cy, sy = np.cos(yaw), np.sin(yaw)
                    wx = cy * rx - sy * ry
                    wy = sy * rx + cy * ry
                    relative_obstacle_pos = np.array([wx, wy], dtype=float)
        except Exception:
            pass


        if relative_obstacle_pos is None:
            relative_obstacle_pos = self.real_obstacle_position - robot_pos + np.random.normal(0, 0.05, 2)


        # --- 3) PF update with the observation ---
        self.pf.update(observed=relative_obstacle_pos)


        # PF estimate (mean of particles)
        est = np.array(self.pf.mean_state).reshape(-1)
        est_xy = est[:2] if est.size >= 2 else np.array([np.nan, np.nan])


        print("Robot position is {}".format(self.pf.mean_state))


        # --- True orientation from odom (yaw) ---
        q = self.odometry.pose.pose.orientation
        yaw_true = quat_to_yaw(q)


        # --- Append positions once (for plotting/history) ---
        self.odom_hist.append(robot_pos.tolist())
        self.pf_hist.append(est_xy.tolist())


        # --- Estimated orientation from PF displacement (sticky if slow) ---
        if len(self.pf_hist) >= 2:
            dx = self.pf_hist[-1][0] - self.pf_hist[-2][0]
            dy = self.pf_hist[-1][1] - self.pf_hist[-2][1]
            if abs(dx) + abs(dy) > 1e-6:
                self.pf_yaw_est = atan2(dy, dx)  # radians


        # Record yaw histories (use true yaw if PF yaw not yet defined)
        self.odom_yaw_hist.append(yaw_true)
        self.pf_yaw_hist.append(self.pf_yaw_est if self.pf_yaw_est is not None else yaw_true)


        # --- Live plot refresh (paths + heading arrows) ---
        if self.live_plot and len(self.odom_hist) >= 2:
            odom_np = np.array(self.odom_hist)
            pf_np   = np.array(self.pf_hist)


            # Update line data
            self.odom_line.set_data(odom_np[:, 0], odom_np[:, 1])
            self.pf_line.set_data(pf_np[:, 0],   pf_np[:, 1])


            # Update heading arrows at last points
            x_o, y_o = odom_np[-1, 0], odom_np[-1, 1]
            x_p, y_p = pf_np[-1,   0], pf_np[-1,   1]


            # True heading components
            u_o = self.heading_len * np.cos(yaw_true)
            v_o = self.heading_len * np.sin(yaw_true)


            # PF heading components (fallback to true on first frames)
            yaw_pf = self.pf_yaw_est if self.pf_yaw_est is not None else yaw_true
            u_p = self.heading_len * np.cos(yaw_pf)
            v_p = self.heading_len * np.sin(yaw_pf)


            # Move quiver bases to current positions and set vectors
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
    filter = LidarParticleFilter()
    
    # Init filter
    filter.pf.init_filter()


    time.sleep(1)
    rclpy_check_rate = 10
    
    rclpy_check_rate = filter.create_rate(10, filter.get_clock())

    filter.get_logger().info("Starting particle filter...")
    try:
        try:
            while rclpy.ok() :
                # This now runs at 5 Hz, change accordingly to your setup
                filter_timer = filter.create_timer(0.2, filter.update_filter)
                rclpy.spin(filter)             
                pass
            
        except KeyboardInterrupt :
            filter.get_logger().error('Keyboard Interrupt detected! Trying to stop filter node!')
    except Exception as e:
        filter.destroy_node()
        filter.get_logger().info("UWB particle filter failed %r."%(e,))
    finally:
        # --- NEW: save PF vs. odometry plot on shutdown ---
        try:
            filter.plot_trajectories()
        except Exception as _:
            pass
        rclpy.shutdown()
        filter.destroy_node()   

if __name__ == "__main__":
    main()


