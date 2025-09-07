import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from custom_interface.action import Countdown

class CountdownCancelClient(Node):
    def __init__(self):
        super().__init__('countdown_cancel_client')
        self.cli = ActionClient(self, Countdown, 'countdown_action')

        # parameters
        self.declare_parameter('start_from', 10)
        self.declare_parameter('cancel_after', 3.0)   # seconds
        self.start_from = int(self.get_parameter('start_from').value)
        self.cancel_after = float(self.get_parameter('cancel_after').value)

        self.goal_handle = None
        self.get_logger().info('Countdown cancel client started')
        self.send_goal()

    def send_goal(self):
        self.cli.wait_for_server()
        goal = Countdown.Goal()
        goal.start_from = self.start_from
        self.get_logger().info(f'Sending goal: start_from={self.start_from}')
        future = self.cli.send_goal_async(goal, feedback_callback=self.feedback_cb)
        future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        # Arm a timer to cancel later
        self.create_timer(self.cancel_after, self.cancel_goal)

    def feedback_cb(self, feedback_msg):
        self.get_logger().info(f'Feedback current={feedback_msg.feedback.current}')

    def cancel_goal(self):
        if self.goal_handle is None:
            return
        self.get_logger().warn('Canceling goal now…')
        cfut = self.goal_handle.cancel_goal_async()
        cfut.add_done_callback(self.cancel_done)

    def cancel_done(self, future):
        self.get_logger().info(f'Cancel response: {future.result()}')

def main(args=None):
    rclpy.init(args=args)
    node = CountdownCancelClient()
    rclpy.spin(node)
    rclpy.try_shutdown()

if __name__ == '__main__':
    main()