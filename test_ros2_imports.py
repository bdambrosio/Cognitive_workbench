#!/usr/bin/env python3
"""
Test script to verify ROS2 imports work correctly
"""

import sys
import os

# Add ROS2 paths to Python path
ros2_path = "/opt/ros/jazzy/lib/python3.12/site-packages"
if ros2_path not in sys.path:
    sys.path.insert(0, ros2_path)

venv_path = "./ros_venv/lib/python3.12/site-packages"
if venv_path not in sys.path:
    sys.path.insert(0, venv_path)

print("Testing ROS2 imports...")
print(f"Python path: {sys.path[:3]}...")

try:
    import rclpy
    print("✅ rclpy imported successfully")
except ImportError as e:
    print(f"❌ Failed to import rclpy: {e}")

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

try:
    from cognitive_framework.srv import MemoryRequest
    print("✅ cognitive_framework.srv.MemoryRequest imported successfully")
except ImportError as e:
    print(f"❌ Failed to import MemoryRequest: {e}")

print("\nAll import tests completed!") 