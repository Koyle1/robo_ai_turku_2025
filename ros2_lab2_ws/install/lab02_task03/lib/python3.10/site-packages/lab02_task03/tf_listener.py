#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class TFListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.prev_pos = None     
        self.prev_time = None 

        self.timer = self.create_timer(2.0, self._tick)

    def _tick(self):
        now = self.get_clock().now()
        try:
            t = self.tf_buffer.lookup_transform('map', 'scanner', rclpy.time.Time())

            x = t.transform.translation.x
            y = t.transform.translation.y
            z = t.transform.translation.z
            dist = math.sqrt(x*x + y*y + z*z)

            speed = 0.0
            if self.prev_pos is not None and self.prev_time is not None:
                dt = (now - self.prev_time).nanoseconds * 1e-9
                if dt > 0.0:
                    dx = x - self.prev_pos[0]
                    dy = y - self.prev_pos[1]
                    dz = z - self.prev_pos[2]
                    speed = math.sqrt(dx*dx + dy*dy + dz*dz) / dt

            self.get_logger().info(
                f"[map -> scanner] "
                f"distance = {dist:.3f} m | speed = {speed:.3f} m/s"
            )

            self.prev_pos = (x, y, z)
            self.prev_time = now

        except TransformException as ex:
            # okay while publishers start up
            self.get_logger().warn(f"TF not available (map->scanner: {ex}")

def main(args=None):
    rclpy.init(args=args)
    node = TFListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()