#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cognitive_framework.srv import TestService  # type: ignore

class TestServiceServer(Node):
    def __init__(self):
        super().__init__('test_service_server')
        
        self.service = self.create_service(
            TestService,
            '/cognitive/test_service',
            self.handle_test_service
        )
        
        self.get_logger().info('Test service server started at /cognitive/test_service')
    
    def handle_test_service(self, request, response):
        self.get_logger().info(f'Received request: {request.message}')
        response.reply = f'Echo: {request.message}'
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    server = TestServiceServer()
    
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        server.get_logger().info('Test service server shutting down...')
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 