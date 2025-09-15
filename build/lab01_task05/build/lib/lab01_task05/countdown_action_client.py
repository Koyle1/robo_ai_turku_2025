#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from custom_interface.action import Countdown


class CountdownActionClient(Node):
    def __init__(self):
        super().__init__('countdown_action_client')
        
        # Declare parameters
        self.declare_parameter('start_from', 10)  # Number to count down from
        
        self._action_client = ActionClient(self, Countdown, 'countdown')

    def send_goal(self):
        # Get parameter value
        start_from = self.get_parameter('start_from').get_parameter_value().integer_value
        
        goal_msg = Countdown.Goal()
        goal_msg.start_from = start_from
        
        self.get_logger().info(f'Waiting for countdown action server...')
        self._action_client.wait_for_server()
        
        self.get_logger().info(f'Sending countdown goal: start from {start_from}')
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by server')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted by server, countdown starting...')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Final result: {result.result_text}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Countdown feedback: {feedback.current}')


def main(args=None):
    rclpy.init(args=args)
    
    action_client = CountdownActionClient()
    action_client.send_goal()
    
    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        action_client.get_logger().info('Countdown client shutting down...')
    finally:
        action_client.destroy_node()


if __name__ == '__main__':
    main()