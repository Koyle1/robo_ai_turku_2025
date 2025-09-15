import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.client import Client
import time
import threading

from custom_interface.action import Countdown
from custom_interface.srv import CancelRequest


class CountdownActionServer(Node):
    def __init__(self):
        super().__init__('countdown_action_server')
        
        # Declare parameters
        self.declare_parameter('request_cancel_after', 8)  # Request cancellation after 8 seconds
        self.declare_parameter('countdown_id', 1)
        
        self._action_server = ActionServer(
            self,
            Countdown,
            'countdown',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        # Service client to request cancellation
        self._cancel_client = self.create_client(CancelRequest, 'request_cancel')
        
        self._current_goal_handle = None
        self._cancel_requested_externally = False
        self._cancel_lock = threading.Lock()  # Thread safety
        
        self.get_logger().info('Countdown Action Server has been started')

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info(f'Received goal request with start_from: {goal_request.start_from}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def request_cancellation_after_delay(self, delay_seconds, countdown_id):
        """Request cancellation from the cancel service after a delay"""
        def delayed_cancel():
            time.sleep(delay_seconds)
            if self._current_goal_handle and not self._current_goal_handle.is_cancel_requested:
                self.get_logger().info(f'Requesting cancellation after {delay_seconds} seconds')
                self._request_cancel_service(countdown_id)
        
        cancel_thread = threading.Thread(target=delayed_cancel)
        cancel_thread.daemon = True
        cancel_thread.start()

    def _request_cancel_service(self, countdown_id):
        """Send a cancellation request to the cancel service"""
        if not self._cancel_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Cancel service not available')
            return
            
        request = CancelRequest.Request()
        request.countdown_id = countdown_id
        request.reason = "Server requested cancellation"
        
        future = self._cancel_client.call_async(request)
        future.add_done_callback(self._cancel_service_response)

    def _cancel_service_response(self, future):
        """Handle response from cancel service"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Cancel service responded: {response.message}')
                self.get_logger().info(f'Will cancel after {response.cancel_delay} seconds')
                
                # Set flag to cancel current goal after delay
                def cancel_after_delay():
                    time.sleep(response.cancel_delay)
                    with self._cancel_lock:
                        self._cancel_requested_externally = True
                    self.get_logger().info('External cancellation triggered - flag set to True')
                    
                    # Also try to cancel the goal handle directly
                    if self._current_goal_handle:
                        self.get_logger().info('Attempting to cancel goal handle directly')
                
                cancel_thread = threading.Thread(target=cancel_after_delay)
                cancel_thread.daemon = True
                cancel_thread.start()
            else:
                self.get_logger().warn(f'Cancel service rejected request: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Cancel service call failed: {e}')

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Executing countdown from {goal_handle.request.start_from}')
        
        # Store current goal handle and reset cancellation flag
        self._current_goal_handle = goal_handle
        with self._cancel_lock:
            self._cancel_requested_externally = False
        
        # Get parameters
        request_cancel_after = self.get_parameter('request_cancel_after').get_parameter_value().integer_value
        countdown_id = self.get_parameter('countdown_id').get_parameter_value().integer_value
        
        # Start the delayed cancellation request
        self.request_cancellation_after_delay(request_cancel_after, countdown_id)
        
        # Create feedback message
        feedback_msg = Countdown.Feedback()
        
        # Get the starting number
        current_count = goal_handle.request.start_from
        
        while current_count >= 0:
            # Check cancellation status with thread safety
            with self._cancel_lock:
                external_cancel = self._cancel_requested_externally
            
            # Check if cancellation was requested (either by client or externally)
            if goal_handle.is_cancel_requested or external_cancel:
                # For client-requested cancellation, call canceled()
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                # For external cancellation, call abort() instead
                else:
                    goal_handle.abort()
                
                result = Countdown.Result()
                cancel_source = "externally" if external_cancel else "by client"
                result.result_text = f"Countdown cancelled {cancel_source} at {current_count}"
                self.get_logger().info(f'Countdown cancelled {cancel_source} at {current_count}')
                self._current_goal_handle = None  # Clear the handle
                return result
            
            # Update feedback
            feedback_msg.current = current_count
            goal_handle.publish_feedback(feedback_msg)
            
            self.get_logger().info(f'Current count: {current_count}')
            
            # Wait for 1 second, but check cancellation more frequently
            for _ in range(10):  # Check 10 times per second
                time.sleep(0.1)
                with self._cancel_lock:
                    external_cancel = self._cancel_requested_externally
                if goal_handle.is_cancel_requested or external_cancel:
                    break
            
            current_count -= 1
        
        # Countdown completed successfully
        goal_handle.succeed()
        
        result = Countdown.Result()
        result.result_text = "Countdown completed successfully!"
        
        self.get_logger().info('Countdown completed successfully!')
        self._current_goal_handle = None  # Clear the handle
        return result


def main(args=None):
    rclpy.init(args=args)
    
    countdown_server = CountdownActionServer()
    
    # Use MultiThreadedExecutor to allow concurrent callback processing
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(countdown_server)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        countdown_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()