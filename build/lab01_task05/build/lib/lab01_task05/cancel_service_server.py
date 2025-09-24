#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import random
import time

from custom_interface.srv import CancelRequest
from custom_interface.action import Countdown


class CancelServiceServer(Node):
    def __init__(self):
        super().__init__('cancel_service_server')
        
        # Declare parameters
        self.declare_parameter('min_cancel_delay', 1.0)  # Minimum delay before cancellation
        self.declare_parameter('max_cancel_delay', 3.0)  # Maximum delay before cancellation
        self.declare_parameter('cancel_probability', 0.8)  # Probability of accepting cancel request
        
        # Create service server
        self._service = self.create_service(
            CancelRequest, 
            'request_cancel', 
            self.handle_cancel_request
        )
        
        # Create action client to actually cancel the countdown
        self._action_client = ActionClient(self, Countdown, 'countdown')
        
        self.get_logger().info('Cancel Service Server has been started')
        self.get_logger().info('Waiting for cancellation requests...')

    def handle_cancel_request(self, request, response):
        """Handle incoming cancellation requests"""
        self.get_logger().info(f'Received cancel request for countdown_id: {request.countdown_id}')
        self.get_logger().info(f'Cancel reason: {request.reason}')
        
        # Get parameters
        min_delay = self.get_parameter('min_cancel_delay').get_parameter_value().double_value
        max_delay = self.get_parameter('max_cancel_delay').get_parameter_value().double_value
        cancel_prob = self.get_parameter('cancel_probability').get_parameter_value().double_value
        
        # Simulate processing time
        time.sleep(0.5)
        
        if random.random() <= cancel_prob:
            response.success = True
            response.cancel_delay = random.uniform(min_delay, max_delay)
            response.message = f"Cancellation accepted for countdown {request.countdown_id}. Will process in {response.cancel_delay:.2f} seconds."
            
            self.get_logger().info(f'Cancel request ACCEPTED. Delay: {response.cancel_delay:.2f} seconds')
            
            # Note: In this architecture, the actual cancellation will be handled by the cancel client
            # This service just approves/rejects the request
            
        else:
            # Reject the request
            response.success = False
            response.cancel_delay = 0.0
            response.message = f"Cancellation rejected for countdown {request.countdown_id}. System busy."
            
            self.get_logger().info('Cancel request REJECTED')
        
        return response


def main(args=None):
    rclpy.init(args=args)
    
    cancel_service = CancelServiceServer()
    
    try:
        rclpy.spin(cancel_service)
    except KeyboardInterrupt:
        cancel_service.get_logger().info('Cancel service server shutting down...')
    finally:
        cancel_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()