#!/usr/bin/env python3

"""
Single LLM Action Example

This is a clean, simple example showing ONE LLM call per text input.
Perfect for demonstration without confusion from multiple responses.

Flow:
1. Receives textSensor input
2. Makes ONE LLM call
3. Creates ONE action based on response
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from datetime import datetime

# Import our LLM client
try:
    # Try relative import first (when running as ROS2 package)
    from .llm_client import LLMClient, LLMResponse
except ImportError:
    # Fall back to absolute import (when running directly)
    from llm_client import LLMClient, LLMResponse


class SingleLLMActionExample(Node):
    """
    Simple action node that makes exactly ONE LLM call per text input.
    
    Clean demonstration flow:
    1. Text input → 2. Single LLM call → 3. Single action
    """
    
    def __init__(self):
        super().__init__('single_llm_action_example')
        
        # Subscribe to sense data
        self.sense_subscriber = self.create_subscription(
            String,
            '/cognitive/sense_data',
            self.sense_data_callback,
            qos_profile=10
        )
        
        # Publisher for actions
        self.action_publisher = self.create_publisher(
            String,
            '/cognitive/action_data',
            qos_profile=10
        )
        
        # Initialize LLM client
        self.llm_client = LLMClient(self, service_timeout=15.0)
        
        # Simple state
        self.action_counter = 0
        
        self.get_logger().info('🤖 Single LLM Action Example initialized')
        self.get_logger().info('📝 Ready for text input - ONE LLM call per input')
    
    def sense_data_callback(self, msg):
        """Process incoming sense data and check for text input."""
        try:
            sense_data = json.loads(msg.data)
            
            # Check for text input
            data = sense_data.get('data', {})
            console_text_sensor = data.get('console_text_sensor', {})
            
            if console_text_sensor.get('new_input', False):
                current_input = console_text_sensor.get('current_input', '')
                
                if current_input.strip():
                    self.get_logger().info(f'📥 Received text input: "{current_input}"')
                    
                    # Make SINGLE LLM call
                    self._process_with_single_llm_call(current_input, sense_data)
                    
        except json.JSONDecodeError as e:
            self.get_logger().error(f'❌ Failed to parse sense data: {e}')
        except Exception as e:
            self.get_logger().error(f'❌ Error processing sense data: {e}')
    
    def _process_with_single_llm_call(self, text_input: str, sense_data: dict):
        """Make exactly ONE LLM call and create ONE action."""
        self.get_logger().info(f'🧠 Making single LLM call for: "{text_input}"')
        
        # Simple, focused prompt
        system_prompt = """You are a helpful AI assistant. The user has provided input. 
Respond naturally and helpfully in a conversational way. Respond only with your direct response to the user's input."""
        
        user_prompt = f"User said: {text_input}"
        
        try:
            # Single BLOCKING call
            response = self.llm_client.generate(
                prompt=user_prompt,
                system_prompt=system_prompt,
                max_tokens=150,
                temperature=0.7,
                timeout=10.0
            )
            
            if response.success:
                self.get_logger().info(f'✅ LLM responded: "{response.text[:100]}..."')
                
                # Create single action
                action = self._create_single_action(text_input, response.text, response.processing_time)
                self._publish_action(action)
                
            else:
                self.get_logger().error(f'❌ LLM call failed: {response.error}')
                
                # Single fallback action
                fallback_action = self._create_fallback_action(text_input)
                self._publish_action(fallback_action)
                
        except Exception as e:
            self.get_logger().error(f'❌ Error in LLM call: {e}')
            fallback_action = self._create_fallback_action(text_input)
            self._publish_action(fallback_action)
    
    def _create_single_action(self, user_input: str, llm_response: str, processing_time: float) -> dict:
        """Create a single action based on LLM response."""
        action = {
            'action_id': self.action_counter,
            'timestamp': datetime.now().isoformat(),
            'action_type': 'llm_guided_action',
            'confidence': 0.85,
            'content': {
                'summary': f'Responded to user input: "{user_input}"',
                'user_input': user_input,
                'llm_response': llm_response,
                'response_type': 'conversational',
                'intent': 'user_interaction',
                'processing_time_seconds': processing_time
            },
            'metadata': {
                'node_name': self.get_name(),
                'llm_enhanced': True,
                'single_call': True
            }
        }
        
        self.action_counter += 1
        return action
    
    def _create_fallback_action(self, text_input: str) -> dict:
        """Create fallback action when LLM fails."""
        action = {
            'action_id': self.action_counter,
            'timestamp': datetime.now().isoformat(),
            'action_type': 'fallback_action',
            'confidence': 0.5,
            'content': {
                'summary': f'Fallback response to: "{text_input}"',
                'user_input': text_input,
                'fallback_reason': 'LLM unavailable',
                'response_type': 'fallback'
            },
            'metadata': {
                'node_name': self.get_name(),
                'llm_enhanced': False,
                'fallback': True
            }
        }
        
        self.action_counter += 1
        return action
    
    def _publish_action(self, action: dict):
        """Publish the action."""
        try:
            msg = String()
            msg.data = json.dumps(action, indent=2)
            self.action_publisher.publish(msg)
            
            action_type = action.get('action_type', 'unknown')
            action_id = action.get('action_id', 'unknown')
            confidence = action.get('confidence', 0)
            
            self.get_logger().info(
                f'🤖 Published {action_type} action #{action_id} (confidence: {confidence:.2f})'
            )
            
        except Exception as e:
            self.get_logger().error(f'❌ Error publishing action: {e}')


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = SingleLLMActionExample()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Single LLM Action Example shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 