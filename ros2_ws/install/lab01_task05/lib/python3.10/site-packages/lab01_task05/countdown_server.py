import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
import time

from custom_interface.action import Countdown


class CountdownActionServer(Node):
    def __init__(self):
        super().__init__('countdown_action_server')
        
        self._action_server = ActionServer(
            self,
            Countdown,
            'countdown',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        self.get_logger().info('Countdown Action Server has been started')

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info(f'Received goal request with start_from: {goal_request.start_from}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Executing countdown from {goal_handle.request.start_from}')
        
        # Create feedback message
        feedback_msg = Countdown.Feedback()
        
        # Get the starting number
        current_count = goal_handle.request.start_from
        
        while current_count >= 0:
            # Check if cancellation was requested
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = Countdown.Result()
                result.result_text = f"Countdown cancelled at {current_count}"
                self.get_logger().info(f'Countdown cancelled at {current_count}')
                return result
            
            # Update feedback
            feedback_msg.current = current_count
            goal_handle.publish_feedback(feedback_msg)
            
            self.get_logger().info(f'Current count: {current_count}')
            
            # Wait for 1 second
            time.sleep(1.0)
            
            current_count -= 1
        
        # Countdown completed successfully
        goal_handle.succeed()
        
        result = Countdown.Result()
        result.result_text = "Countdown completed successfully!"
        
        self.get_logger().info('Countdown completed successfully!')
        return result


def main(args=None):
    rclpy.init(args=args)
    
    countdown_server = CountdownActionServer()
    
    try:
        rclpy.spin(countdown_server)
    except KeyboardInterrupt:
        pass
    finally:
        countdown_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()