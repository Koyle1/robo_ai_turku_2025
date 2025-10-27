#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import time

from custom_interface.action import Countdown


class CountdownActionServer(Node):
    def __init__(self):
        super().__init__('countdown_action_server')
        
        # Declare parameters
        self.declare_parameter('countdown_interval', 1.0)  # Time between counts (seconds)
        
        self._has_active_goal = False
        
        self._action_server = ActionServer(
            self,
            Countdown,
            'countdown',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        self.get_logger().info('Countdown Action Server has been started')
        self.get_logger().info('Waiting for countdown goals...')

    def goal_callback(self, goal_request):
        if self._has_active_goal:
            self.get_logger().warn('Rejecting goal because another countdown is active')
            return GoalResponse.REJECT
        self.get_logger().info(f'Accepting goal request: countdown from {goal_request.start_from}')
        self._has_active_goal = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request - accepting')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        try:
            self.get_logger().info(f'Starting countdown from {goal_handle.request.start_from}')
            
            # Get parameters
            countdown_interval = self.get_parameter('countdown_interval').get_parameter_value().double_value
            
            # Create feedback message
            feedback_msg = Countdown.Feedback()
            
            # Get the starting number
            current_count = goal_handle.request.start_from
            
            # Countdown loop
            while current_count >= 0:
                # Check if cancellation was requested
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result = Countdown.Result()
                    result.result_text = f"Countdown cancelled by client at {current_count}"
                    self.get_logger().info(f'Countdown cancelled by client at {current_count}')
                    return result
                
                # Update and publish feedback
                feedback_msg.current = current_count
                goal_handle.publish_feedback(feedback_msg)
                
                self.get_logger().info(f'Current count: {current_count}')
                
                # Check for cancellation more frequently during the wait
                sleep_steps = 10  # Check cancellation 10 times per interval
                step_duration = countdown_interval / sleep_steps
                
                for _ in range(sleep_steps):
                    if goal_handle.is_cancel_requested:
                        break
                    time.sleep(step_duration)
                
                # If we were cancelled during the sleep, handle it in the next loop iteration
                if goal_handle.is_cancel_requested:
                    continue
                    
                current_count -= 1
            
            # Countdown completed successfully
            goal_handle.succeed()
            
            result = Countdown.Result()
            result.result_text = "Countdown completed successfully! Reached zero."
            
            self.get_logger().info('Countdown completed successfully! Reached zero.')
            return result
        finally:
            self._has_active_goal = False


def main(args=None):
    rclpy.init(args=args)
    
    countdown_server = CountdownActionServer()
    
    # Use MultiThreadedExecutor for concurrent callback processing
    executor = MultiThreadedExecutor()
    executor.add_node(countdown_server)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        countdown_server.get_logger().info('Countdown server shutting down...')
    finally:
        countdown_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()