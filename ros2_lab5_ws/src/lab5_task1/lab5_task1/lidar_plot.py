import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt


class LidarPlotter(Node):
    def __init__(self):
        super().__init__('lidar_plotter')

        qos_profile = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1) 

        # Subscriptions
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )
        self.sub_feature = self.create_subscription(
            LaserScan, 
            '/feature_scan', 
            self.feature_callback, 
            qos_profile
        )
        self.sub_front = self.create_subscription(
            LaserScan,
            '/front_scan', 
            self.front_callback,
            qos_profile
        )


        self.scan_data = None
        self.feature_data = None
        self.front_data = None

        plt.ion()
        self.fig = plt.figure(figsize=(12, 6))
        self.ax_polar = self.fig.add_subplot(1, 2, 1, projection='polar')
        self.ax_cart  = self.fig.add_subplot(1, 2, 2)
        self.fig.show()


    def polar_to_cartesian_coordinate(self, ranges, angle_min, angle_max, angle_increment):
        """Convert LaserScan polar data to Cartesian coordinates."""
        ranges_np = np.asarray(ranges, dtype=float)

        # remove/ignore invalid values but preserve length for angles
        invalid = ~np.isfinite(ranges_np)
        ranges_np[invalid] = np.nan  # NaNs are ignored by matplotlib

        n = len(ranges_np)
        # build angles to EXACTLY match the ranges length
        angles = angle_min + np.arange(n) * angle_increment

        # Cartesian
        x_cart = ranges_np * np.cos(angles)
        y_cart = ranges_np * np.sin(angles)

        
        return angles, ranges_np, x_cart, y_cart

    def scan_callback(self, msg: LaserScan):
        if self.scan_data is None:
            self.get_logger().info('Received first /scan')
        self.scan_data = msg
        self.update_plot()

    def feature_callback(self, msg: LaserScan):
        if self.feature_data is None:
            self.get_logger().info('Received first /feature_scan')
        self.feature_data = msg
        self.update_plot()

    def front_callback(self, msg: LaserScan):
        if self.front_data is None:
            self.get_logger().info('Received first /front_scan')
        self.front_data = msg
        self.update_plot()



    def update_plot(self):
        if self.scan_data is None and self.feature_data is None and self.front_data is None:
            return

        # Clear existing content on SAME axes
        self.ax_polar.clear()
        self.ax_cart.clear()


        
        def plot_one(scan_msg, label, style='-'):
            if scan_msg is None:
                return
            rr = np.asarray(scan_msg.ranges, dtype=float).reshape(-1)
            n = rr.size
            angles = (scan_msg.angle_min + np.arange(n) * scan_msg.angle_increment).reshape(-1)

            # Clean invalids; Matplotlib ignores NaN
            rr[~np.isfinite(rr)] = np.nan
            rr[(rr < scan_msg.range_min) | (rr > scan_msg.range_max)] = np.nan
            finite = np.isfinite(rr)
            if finite.sum() == 0:
                self.get_logger().warn(f'{label}: received but 0 finite points')
                return

            # Polar
            self.ax_polar.plot(angles[finite], rr[finite], style, linewidth=1.0, label=label)

            # Cartesian
            x = rr * np.cos(angles)
            y = rr * np.sin(angles)
            finite_xy = np.isfinite(x) & np.isfinite(y)
            self.ax_cart.plot(x[finite_xy], y[finite_xy], '.', markersize=2, label=label)

            # One-time debug
            self.get_logger().info(f'{label}: plotted {finite_xy.sum()} points', throttle_duration_sec=2.0)


        # Plot available topics (colors/styles are up to you / RViz will differ)
        plot_one(self.scan_data,    '/scan',         '-')
        plot_one(self.feature_data, '/feature_scan', '-.')
        plot_one(self.front_data,   '/front_scan',   ':')

        self.ax_polar.set_title('Polar')
        self.ax_cart.set_title('Cartesian')
        self.ax_cart.axis('equal')
        self.ax_polar.legend(loc='upper right', fontsize=8)
        self.ax_cart.legend(loc='best', fontsize=8)

        self.fig.tight_layout()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()



def main(args=None):
    rclpy.init(args=args)
    node = LidarPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
