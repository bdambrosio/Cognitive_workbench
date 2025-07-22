#!/usr/bin/env python3

import sys
import os

# Add the installed package to Python path
install_path = "/home/bruce/Downloads/AllTheWorldAPlay/ros/install/cognitive_framework/lib/python3.12/site-packages"
sys.path.insert(0, install_path)

try:
    from cognitive_framework.srv import MemoryRequest
    print("✅ MemoryRequest service import successful!")
    
    # Test creating a service
    import rclpy
    from rclpy.node import Node
    
    rclpy.init()
    node = Node('test_memory_node')
    
    # Test service creation
    def test_handler(request, response):
        response.entries_json = '[{"id": "test", "content": "test content"}]'
        response.success = True
        response.error_message = ""
        return response
    
    service = node.create_service(
        MemoryRequest,
        '/test_memory_service',
        test_handler
    )
    
    print("✅ Memory service creation successful!")
    print("✅ Memory service pattern is working!")
    
    node.destroy_node()
    rclpy.shutdown()
    
except Exception as e:
    print(f"❌ Error: {e}")
    import traceback
    traceback.print_exc() 