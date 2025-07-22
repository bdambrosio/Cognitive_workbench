#!/usr/bin/env python3

"""
Test script to verify text input routing between Action Display and Sense Node.

This script monitors the /cognitive/text_input and /cognitive/sense_data topics
to ensure that text typed in the Action Display reaches the cognitive system.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import sys

class TextRoutingTester(Node):
    def __init__(self):
        super().__init__('text_routing_tester')
        
        # Subscribe to text input topic (what Action Display publishes)
        self.text_input_subscriber = self.create_subscription(
            String,
            '/cognitive/text_input',
            self.text_input_callback,
            10
        )
        
        # Subscribe to sense data topic (what Sense Node publishes)
        self.sense_data_subscriber = self.create_subscription(
            String,
            '/cognitive/sense_data',
            self.sense_data_callback,
            10
        )
        
        self.get_logger().info('🧪 Text Routing Tester initialized')
        self.get_logger().info('📡 Monitoring /cognitive/text_input and /cognitive/sense_data')
        self.get_logger().info('💡 Start Action Display and type something to test the routing')
        
    def text_input_callback(self, msg):
        """Monitor text input from Action Display"""
        text_input = msg.data
        self.get_logger().info(f'📤 TEXT INPUT detected: "{text_input}"')
        
    def sense_data_callback(self, msg):
        """Monitor sense data to see if text input appears"""
        try:
            sense_data = json.loads(msg.data)
            
            # Check for console text sensor data
            console_sensor = sense_data.get('data', {}).get('console_text_sensor', {})
            if console_sensor.get('new_input', False):
                text_input = console_sensor.get('current_input', '')
                sensor_type = sense_data.get('sensor_type', 'unknown')
                
                self.get_logger().info(f'📥 SENSE DATA includes text: "{text_input}" (type: {sensor_type})')
                
                # Check if this came from external input
                text_data = sense_data.get('data', {}).get('text', {})
                if isinstance(text_data, dict):
                    console_input = text_data.get('console_input')
                    if console_input:
                        self.get_logger().info(f'✅ SUCCESS: Text routing working! Input reached sense system.')
                
        except json.JSONDecodeError:
            pass

def main(args=None):
    rclpy.init(args=args)
    
    try:
        tester = TextRoutingTester()
        
        print("\n" + "="*60)
        print("🧪 TEXT ROUTING TESTER")
        print("="*60)
        print("1. Start this tester")
        print("2. Launch 'Debug Action Display' in another terminal")
        print("3. Type something in the Action Display")
        print("4. Watch for routing messages here")
        print("5. Ctrl+C to exit")
        print("="*60 + "\n")
        
        rclpy.spin(tester)
        
    except KeyboardInterrupt:
        print("\n👋 Text routing test complete")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main() 