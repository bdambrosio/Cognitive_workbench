#!/usr/bin/env python3
"""
Setup script to help IDE understand ROS2 environment
Run this script to verify that all imports work correctly
"""

import sys
import os

def setup_ros2_paths():
    """Add ROS2 paths to Python path"""
    paths_to_add = [
        "./ros_venv/lib/python3.12/site-packages",
        "/opt/ros/jazzy/lib/python3.12/site-packages",
        "./install/cognitive_framework/lib/python3.12/site-packages"
    ]
    
    for path in paths_to_add:
        if os.path.exists(path) and path not in sys.path:
            sys.path.insert(0, path)
            print(f"Added to Python path: {path}")

def test_imports():
    """Test all the imports that the IDE might complain about"""
    print("Testing ROS2 imports...")
    
    # Test basic ROS2 imports
    try:
        import rclpy
        print("✅ rclpy imported successfully")
    except ImportError as e:
        print(f"❌ Failed to import rclpy: {e}")
    
    try:
        import rclpy.node
        print("✅ rclpy.node imported successfully")
    except ImportError as e:
        print(f"❌ Failed to import rclpy.node: {e}")
    
    try:
        import std_msgs.msg
        print("✅ std_msgs.msg imported successfully")
    except ImportError as e:
        print(f"❌ Failed to import std_msgs.msg: {e}")
    
    try:
        import geometry_msgs.msg
        print("✅ geometry_msgs.msg imported successfully")
    except ImportError as e:
        print(f"❌ Failed to import geometry_msgs.msg: {e}")
    
    # Test launch imports
    try:
        from launch import LaunchDescription
        print("✅ launch.LaunchDescription imported successfully")
    except ImportError as e:
        print(f"❌ Failed to import launch.LaunchDescription: {e}")
    
    try:
        from launch_ros.actions import Node
        print("✅ launch_ros.actions.Node imported successfully")
    except ImportError as e:
        print(f"❌ Failed to import launch_ros.actions.Node: {e}")
    
    # Test our package imports
    try:
        from cognitive_framework.srv import MemoryRequest
        print("✅ cognitive_framework.srv.MemoryRequest imported successfully")
    except ImportError as e:
        print(f"❌ Failed to import MemoryRequest: {e}")
    
    try:
        from cognitive_framework.Messages import SystemMessage, UserMessage, AssistantMessage
        print("✅ cognitive_framework.Messages imported successfully")
    except ImportError as e:
        print(f"❌ Failed to import Messages: {e}")

if __name__ == "__main__":
    setup_ros2_paths()
    test_imports()
    print("\nSetup complete! If all imports succeeded, the IDE should work better.")
    print("If some imports failed, you may need to:")
    print("1. Run 'source setup_environment.sh' in your terminal")
    print("2. Restart your IDE")
    print("3. Reload the Python extension") 