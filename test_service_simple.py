#!/usr/bin/env python3

import sys
import os

# Add the installed package to Python path
install_path = "/home/bruce/Downloads/AllTheWorldAPlay/ros/install/cognitive_framework/lib/python3.12/site-packages"
sys.path.insert(0, install_path)

try:
    from cognitive_framework.srv import TestService
    print("✅ Service import successful!")
    
    # Test creating a service
    import rclpy
    from rclpy.node import Node
    
    rclpy.init()
    node = Node('test_node')
    
    service = node.create_service(
        TestService,
        '/test_service',
        lambda req, resp: resp
    )
    
    print("✅ Service creation successful!")
    print("✅ ROS2 service pattern is working!")
    
    node.destroy_node()
    rclpy.shutdown()
    
except Exception as e:
    print(f"❌ Error: {e}")
    import traceback
    traceback.print_exc() 