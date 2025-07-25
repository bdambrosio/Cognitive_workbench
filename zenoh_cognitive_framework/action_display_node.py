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
import argparse
import re
from datetime import datetime
from typing import Dict, List, Any, Set


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
        
        # Track active characters
        self.active_characters: Set[str] = set()
        self.character_publishers: Dict[str, Any] = {}
        self.last_character_name: str = None
        
        # Subscriber for all character actions
        self.action_subscriber = self.session.declare_subscriber(
            "cognitive/*/action",
            self.action_callback
        )
        
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
        print('   - Subscribing to: cognitive/*/action (all characters)')
        print('   - Publishing to: cognitive/{character}/text_input (dynamic)')
        print('   - Publishing to: cognitive/memory/store')
        print('   - Console input ready')
        print('   - Format: "character: message" or just "message" (uses last character)')
    
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
            
            # Extract character name from topic path
            topic_path = str(sample.key_expr)
            character_name = topic_path.split('/')[1]  # cognitive/{character}/action
            
            # Track active character
            self.active_characters.add(character_name)
            
            # Handle character announcements
            if action_data.get('action_type') == 'character_announcement':
                self._handle_character_announcement(action_data, character_name)
                return
            
            # Add to history
            self.action_history.append({
                'timestamp': datetime.now().isoformat(),
                'character': character_name,
                'action_data': action_data
            })
            
            # Limit history size
            if len(self.action_history) > self.max_history:
                self.action_history = self.action_history[-self.max_history:]
            
            # Display the action
            self._display_action(action_data, character_name)
            
            # Store in memory
            self._store_action_in_memory(action_data, character_name)
            
        except Exception as e:
            print(f'❌ Error processing action: {e}')
    
    def _display_action(self, action_data: Dict[str, Any], character_name: str):
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
            
            print(f'\n🎯 [{character_name.upper()}] ACTION #{self.action_counter} [{formatted_time}]')
            print(f'   ID: {action_id}')
            print(f'   Type: {action_type}')
            
            # Display input text if available
            input_text = action_data.get('input_text', '')
            if input_text:
                print(f'   Input: "{input_text}"')
            
            # Display LLM response if available
            llm_response = action_data.get('llm_response', '')
            if llm_response:
               
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
            # Parse character name and message
            character_name, message = self._parse_character_input(text_input)
            
            if not message:
                print(f'❌ Invalid format. Use: "character: message" or just "message" (if last character exists)')
                return
            
            # If no character name provided, use the last character
            if not character_name:
                if not self.last_character_name:
                    print(f'❌ No character specified and no previous character. Use: "character: message" (e.g., "samantha: hello")')
                    return
                character_name = self.last_character_name
                print(f'💡 Using last character: {character_name}')
            
            # Find the actual character name (case-insensitive matching)
            actual_character_name = None
            for active_char in self.active_characters:
                if active_char.lower() == character_name.lower():
                    actual_character_name = active_char
                    break
            
            if not actual_character_name:
                print(f'❌ Character "{character_name}" not found. Available: {", ".join(sorted(self.active_characters))}')
                return
            
            # Store this as the last character used
            self.last_character_name = actual_character_name
            
            # Get or create publisher for this character
            if actual_character_name not in self.character_publishers:
                self.character_publishers[actual_character_name] = self.session.declare_publisher(
                    f"cognitive/{actual_character_name}/text_input"
                )
            
            # Publish text input to specific character in JSON format
            text_input_data = {
                'source': 'ui',
                'text': message
            }
            self.character_publishers[actual_character_name].put(json.dumps(text_input_data))
            print(f'📤 Sent to {actual_character_name}: "{message}" (source: ui)')
            
            # Store in memory
            self._store_text_input_in_memory(text_input, actual_character_name)
            
        except Exception as e:
            print(f'❌ Error sending text input: {e}')
    
    def _parse_character_input(self, text_input: str) -> tuple[str, str]:
        """Parse character name and message from input text."""
        text_input = text_input.strip()
        
        # Match pattern: "character: message" or "character : message"
        match = re.match(r'^([^:]+):\s*(.+)$', text_input)
        if match:
            character_name = match.group(1).strip()
            message = match.group(2).strip()
            return character_name, message
        
        # If no colon found, treat as message only (no character specified)
        if text_input:
            return "", text_input
        
        return "", ""
    
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
    
    def _store_action_in_memory(self, action_data: Dict[str, Any], character_name: str):
        """Store action in memory."""
        try:
            memory_data = {
                'memory_type': 'short_term',
                'key': f'action_{int(time.time())}',
                'content': {
                    'type': 'action_display',
                    'character': character_name,
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
    
    def _store_text_input_in_memory(self, text_input: str, character_name: str):
        """Store text input in memory."""
        try:
            memory_data = {
                'memory_type': 'short_term',
                'key': f'text_input_{int(time.time())}',
                'content': {
                    'type': 'text_input',
                    'character': character_name,
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
            'active_characters': list(self.active_characters),
            'uptime': time.time(),
            'last_action_time': self.action_history[-1]['timestamp'] if self.action_history else None
        }
    
    def _handle_character_announcement(self, action_data: Dict[str, Any], character_name: str):
        """Handle character announcement actions."""
        try:
            message = action_data.get('message', f'Character {character_name} is ready')
            print(f'\n👋 {message}')
            print(f'💡 You can now send messages to {character_name}')
            
            # Show updated character list
            self.show_available_characters()
            
        except Exception as e:
            print(f'❌ Error handling character announcement: {e}')
    
    def show_available_characters(self):
        """Display available characters."""
        if self.active_characters:
            print(f'\n👥 Active Characters: {", ".join(sorted(self.active_characters))}')
        else:
            print(f'\n👥 No active characters detected yet. Send a message to create one.')
        print(f'💡 Format: "character: message" (e.g., "samantha: hello")')
    
    def shutdown(self):
        """Clean shutdown."""
        print('🛑 Shutting down Action Display Node...')
        
        # Print final statistics
        stats = self.get_statistics()
        print(f'📊 Final Statistics:')
        print(f'   Total actions displayed: {stats["total_actions"]}')
        print(f'   History size: {stats["history_size"]}')
        print(f'   Active characters: {", ".join(stats["active_characters"])}')
        
        self.session.close()


def main():
    """Main entry point for the action display node."""
    parser = argparse.ArgumentParser(description='Zenoh Action Display Node')
    parser.add_argument('--show-characters', action='store_true', help='Show available characters and exit')
    
    args = parser.parse_args()
    
    display_node = ZenohActionDisplayNode()
    
    if args.show_characters:
        # Wait a moment for any active characters to be detected
        import time
        time.sleep(2)
        display_node.show_available_characters()
        return
    
    display_node.run()


if __name__ == '__main__':
    main() 