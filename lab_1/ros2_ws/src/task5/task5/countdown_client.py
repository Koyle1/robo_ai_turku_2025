import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from custom_interface.action import Countdown

class CountdownClient(Node):
    def __init__(self):
        super().__init__('countdown_client')
        self.cli = ActionClient(self, Countdown, 'countdown_action')

        # parameter for the goal (default 5)
        self.declare_parameter('start_from', 5)
        self.start_from = int(self.get_parameter('start_from').value)

        self.get_logger().info('Countdown action client started')
        self.send_goal()

    def send_goal(self):
        self.cli.wait_for_server()
        goal = Countdown.Goal()
        goal.start_from = self.start_from
        self.get_logger().info(f'Sending goal: start_from={self.start_from}')
        send_goal_future = self.cli.send_goal_async(
            goal,
            feedback_callback=self.feedback_cb
        )
        send_goal_future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(f'Feedback current={fb.current}')

    def result_cb(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: '{result.result_text}'")

def main(args=None):
    rclpy.init(args=args)
    node = CountdownClient()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()