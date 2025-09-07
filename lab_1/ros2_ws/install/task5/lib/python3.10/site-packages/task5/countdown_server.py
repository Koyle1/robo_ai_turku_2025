import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from custom_interface.action import Countdown
import time

class CountdownServer(Node):
    def __init__(self):
        super().__init__('countdown_server')
        self.action_server = ActionServer(
            self,
            Countdown,
            'countdown_action',
            execute_callback=self.execute_callback
        )
        self.get_logger().info('Countdown action server started on /countdown_action')

    def execute_callback(self, goal_handle):
        start = int(goal_handle.request.start_from)
        self.get_logger().info(f'Executing countdown from {start}')

        feedback = Countdown.Feedback()
        # Count down to zero, publishing feedback each step
        for current in range(start, -1, -1):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().warn('Countdown goal canceled')
                return Countdown.Result(result_text='Canceled')

            feedback.current = current
            goal_handle.publish_feedback(feedback)
            time.sleep(1.0)  # 1 Hz

        goal_handle.succeed()
        result = Countdown.Result()
        result.result_text = f'Finished at 0 from start {start}'
        return result

def main(args=None):
    rclpy.init(args=args)
    node = CountdownServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()