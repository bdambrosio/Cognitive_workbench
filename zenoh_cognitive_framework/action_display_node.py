#!/usr/bin/env python3
"""
Zenoh Action Display Node

This node displays actions and provides text input interface.
Replaces ROS2 complexity with simple Zenoh pub/sub.
"""

import zenoh
import json
import time
import threading
import queue
from datetime import datetime
from typing import Dict, List, Any


class ZenohActionDisplayNode:
    """
    The Action Display node provides:
    - Display of incoming actions
    - Text input interface
    - Action history and statistics
    """
    
    def __init__(self):
        # Initialize Zenoh session
        config = zenoh.Config()
        self.session = zenoh.open(config)
        
        # Subscriber for actions
        self.action_subscriber = self.session.declare_subscriber(
            "cognitive/action",
            self.action_callback
        )
        
        # Publisher for text input
        self.text_input_publisher = self.session.declare_publisher("cognitive/text_input")
        
        # Publisher for memory storage
        self.memory_publisher = self.session.declare_publisher("cognitive/memory/store")
        
        # Internal state
        self.action_history = []
        self.max_history = 100
        self.action_counter = 0
        
        # Console input handling
        self.text_input_queue = queue.Queue()
        self.input_thread = threading.Thread(target=self._console_input_thread, daemon=True)
        self.input_thread.start()
        
        print('🖥️  Zenoh Action Display Node initialized')
        print('   - Subscribing to: cognitive/action')
        print('   - Publishing to: cognitive/text_input')
        print('   - Publishing to: cognitive/memory/store')
        print('   - Console input ready')
    
    def run(self):
        """Main display loop."""
        try:
            print('Action Display Node running - press Ctrl+C to stop')
            print('Type text and press Enter to send input')
            
            while True:
                # Process console input if available
                text_input = self._get_console_input()
                if text_input:
                    self._send_text_input(text_input)
                
                time.sleep(0.1)  # 10Hz update rate
                
        except KeyboardInterrupt:
            print('Action Display Node shutting down...')
        finally:
            self.shutdown()
    
    def action_callback(self, sample):
        """Handle incoming actions."""
        try:
            action_data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            self.action_counter += 1
            
            # Add to history
            self.action_history.append({
                'timestamp': datetime.now().isoformat(),
                'action_data': action_data
            })
            
            # Limit history size
            if len(self.action_history) > self.max_history:
                self.action_history = self.action_history[-self.max_history:]
            
            # Display the action
            self._display_action(action_data)
            
            # Store in memory
            self._store_action_in_memory(action_data)
            
        except Exception as e:
            print(f'❌ Error processing action: {e}')
    
    def _display_action(self, action_data: Dict[str, Any]):
        """Display an action in a formatted way."""
        try:
            action_id = action_data.get('action_id', 'unknown')
            action_type = action_data.get('action_type', 'unknown')
            timestamp = action_data.get('timestamp', 'unknown')
            
            # Format timestamp
            try:
                dt = datetime.fromisoformat(timestamp.replace('Z', '+00:00'))
                formatted_time = dt.strftime('%H:%M:%S')
            except:
                formatted_time = timestamp
            
            print(f'\n🎯 ACTION #{self.action_counter} [{formatted_time}]')
            print(f'   ID: {action_id}')
            print(f'   Type: {action_type}')
            
            # Display input text if available
            input_text = action_data.get('input_text', '')
            if input_text:
                print(f'   Input: "{input_text}"')
            
            # Display LLM response if available
            llm_response = action_data.get('llm_response', '')
            if llm_response:
                # Truncate long responses
                if len(llm_response) > 200:
                    display_response = llm_response[:200] + "..."
                else:
                    display_response = llm_response
                print(f'   Response: {display_response}')
            
            # Display confidence if available
            confidence = action_data.get('confidence', None)
            if confidence is not None:
                print(f'   Confidence: {confidence:.2f}')
            
            # Display metadata if available
            metadata = action_data.get('metadata', {})
            if metadata:
                model = metadata.get('model', 'unknown')
                processing_time = metadata.get('processing_time', 0.0)
                memory_context = metadata.get('memory_context_used', 0)
                print(f'   Model: {model}, Time: {processing_time:.3f}s, Memory: {memory_context} entries')
            
            print()  # Empty line for readability
            
        except Exception as e:
            print(f'❌ Error displaying action: {e}')
    
    def _send_text_input(self, text_input: str):
        """Send text input to the system."""
        try:
            # Publish text input
            self.text_input_publisher.put(text_input)
            print(f'📤 Sent text input: "{text_input}"')
            
            # Store in memory
            self._store_text_input_in_memory(text_input)
            
        except Exception as e:
            print(f'❌ Error sending text input: {e}')
    
    def _console_input_thread(self):
        """Background thread for console input."""
        try:
            while True:
                try:
                    # Get user input
                    user_input = input()
                    if user_input.strip():  # Only process non-empty input
                        self.text_input_queue.put(user_input.strip())
                except EOFError:
                    # Handle Ctrl+D gracefully
                    break
                except KeyboardInterrupt:
                    # Handle Ctrl+C gracefully
                    break
        except Exception as e:
            print(f'Console input thread error: {str(e)}')
    
    def _get_console_input(self):
        """Get the most recent console input if available."""
        try:
            return self.text_input_queue.get_nowait()
        except queue.Empty:
            return None
    
    def _store_action_in_memory(self, action_data: Dict[str, Any]):
        """Store action in memory."""
        try:
            memory_data = {
                'memory_type': 'short_term',
                'key': f'action_{int(time.time())}',
                'content': {
                    'type': 'action_display',
                    'action_data': action_data,
                    'timestamp': datetime.now().isoformat(),
                    'metadata': {
                        'node': 'action_display',
                        'displayed': True
                    }
                }
            }
            
            self.memory_publisher.put(json.dumps(memory_data))
            
        except Exception as e:
            print(f'❌ Error storing action in memory: {e}')
    
    def _store_text_input_in_memory(self, text_input: str):
        """Store text input in memory."""
        try:
            memory_data = {
                'memory_type': 'short_term',
                'key': f'text_input_{int(time.time())}',
                'content': {
                    'type': 'text_input',
                    'text': text_input,
                    'timestamp': datetime.now().isoformat(),
                    'metadata': {
                        'node': 'action_display',
                        'source': 'console'
                    }
                }
            }
            
            self.memory_publisher.put(json.dumps(memory_data))
            
        except Exception as e:
            print(f'❌ Error storing text input in memory: {e}')
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get display statistics."""
        return {
            'total_actions': self.action_counter,
            'history_size': len(self.action_history),
            'uptime': time.time(),
            'last_action_time': self.action_history[-1]['timestamp'] if self.action_history else None
        }
    
    def shutdown(self):
        """Clean shutdown."""
        print('🛑 Shutting down Action Display Node...')
        
        # Print final statistics
        stats = self.get_statistics()
        print(f'📊 Final Statistics:')
        print(f'   Total actions displayed: {stats["total_actions"]}')
        print(f'   History size: {stats["history_size"]}')
        
        self.session.close()


def main():
    """Main entry point for the action display node."""
    display_node = ZenohActionDisplayNode()
    display_node.run()


if __name__ == '__main__':
    main() 