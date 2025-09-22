#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class TFListenerRevolutions(Node):
    def __init__(self):
        super().__init__('tf_listener_revolutions')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.prev_angle = None
        self.rev_count = 0.0

        self.sample_timer = self.create_timer(0.1, self._sample_once)  # 10 Hz
        self.print_timer  = self.create_timer(2.0, self._print_stats)  # every 2 s

    def _sample_once(self):
        try:
            tr = self.tf_buffer.lookup_transform('map', 'robot', rclpy.time.Time())
            x = tr.transform.translation.x
            y = tr.transform.translation.y

            angle = math.atan2(y, x)  # [-pi, pi]
            if self.prev_angle is not None:
                dtheta = angle - self.prev_angle
                # unwrap to [-pi, pi]
                if dtheta >  math.pi: dtheta -= 2.0 * math.pi
                if dtheta < -math.pi: dtheta += 2.0 * math.pi
                # accumulate revolutions
                self.rev_count += dtheta / (2.0 * math.pi)

            self.prev_angle = angle

        except TransformException:
            pass

    def _print_stats(self):
        self.get_logger().info(f"[map->robot] revolutions ≈ {self.rev_count:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = TFListenerRevolutions()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()