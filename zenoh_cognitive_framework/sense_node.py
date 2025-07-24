#!/usr/bin/env python3
"""
Zenoh Sense Node

This node simulates sensory input and publishes perception data via Zenoh.
Replaces ROS2 complexity with simple Zenoh pub/sub.
"""

import zenoh
import json
import time
import threading
import queue
import sys
import argparse
from datetime import datetime


class ZenohSenseNode:
    """
    The Sense node handles perception and sensory input processing.
    
    Publishers:
        cognitive/sense_data: Raw sensory information
    """
    
    def __init__(self, character_name="default", character_config=None):
        # Store character info
        self.character_name = character_name
        self.character_config = character_config or {}
        
        # Initialize Zenoh session
        config = zenoh.Config()
        self.session = zenoh.open(config)
        
        # Publisher for sense data (character-specific)
        self.sense_publisher = self.session.declare_publisher(f"cognitive/{character_name}/sense_data")
        
        # Subscriber for external text input (character-specific)
        self.text_input_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/text_input",
            self.external_text_callback
        )
        
        # Subscriber for visual events (character-specific)
        self.visual_event_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/sense/visual/*",
            self.visual_event_callback
        )
        
        # Internal state
        self.sequence_id = 0
        
        # Console input handling
        self.text_input_queue = queue.Queue()
        self.last_text_input = None
        self.external_input_active = False
        
        # Only start console input thread if no external input is expected
        self.input_thread = threading.Thread(target=self._console_input_thread, daemon=True)
        self.input_thread.start()
        
        print(f'Sense Node initialized for character: {character_name}')
        print('Console Text Sensor ready - will auto-disable if external input detected')
    
    def run(self):
        """Main sensing loop."""
        try:
            while True:
                # Get console input if available
                console_input = self._get_console_input()
                if not console_input:
                    time.sleep(0.1)  # 10Hz sensing
                    continue
                
                # Create structured sense data message
                sense_data = {
                    'timestamp': datetime.now().isoformat(),
                    'sequence_id': self.sequence_id,
                    'mode': 'text',
                    'content':  console_input
                }
                
                # Publish sense data
                self.sense_publisher.put(json.dumps(sense_data))
                
                # Log
                print(f'Published sense data #{self.sequence_id} [Console input: "{console_input}"]')
                
                self.sequence_id += 1
                
        except KeyboardInterrupt:
            print('Sense Node shutting down...')
        finally:
            self.session.close()
    
    def _console_input_thread(self):
        """
        Background thread that reads console input and queues it for processing.
        
        This allows the main Zenoh loop to continue while waiting for user input.
        Will automatically disable if external input (from Action Display) is detected.
        """
        try:
            while True:
                try:                   
                    # Prompt for input (will only show in terminal, not in logs)
                    user_input = input()
                    if user_input.strip():  # Only process non-empty input
                        # Double-check external input hasn't been activated while we were waiting
                        if not self.external_input_active:
                            self.text_input_queue.put(user_input.strip())
                            # Log to console instead of ROS2
                            print(f'📝 Console text input received: "{user_input.strip()}"')
                        else:
                            print('🔄 Ignoring console input - external input is now active')
                            break
                except EOFError:
                    # Handle Ctrl+D gracefully
                    break
                except KeyboardInterrupt:
                    # Handle Ctrl+C gracefully
                    break
        except Exception as e:
            print(f'Console input thread error: {str(e)}')
    
    def _get_console_input(self):
        """
        Get the most recent console input if available.
        
        Returns:
            str or None: Recent text input or None if no new input
        """
        text_input = None
        
        # Get the most recent input (drain queue to get latest)
        while not self.text_input_queue.empty():
            try:
                text_input = self.text_input_queue.get_nowait()
            except queue.Empty:
                break
        
        # Update last input if we got new input
        if text_input is not None:
            self.last_text_input = text_input
            
        return text_input
    
    def external_text_callback(self, sample):
        """Handle text input from external sources (like action display)"""
        try:
            text_input = sample.payload.to_bytes().decode('utf-8').strip()
            if text_input:
                # Mark external input as active to prevent console input conflicts
                if not self.external_input_active:
                    self.external_input_active = True
                    print('🔄 External input detected - console input disabled to prevent conflicts')
                
                # Add to the queue (only from external source now)
                self.text_input_queue.put(text_input)
                print(f'📨 Received external text input: "{text_input}"')
        except Exception as e:
            print(f'Error processing external text input: {e}')
    
    def visual_event_callback(self, sample):
        """Handle visual events from the map node"""
        try:
            # Parse the visual event data
            visual_data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            event_type = str(sample.key_expr).split('/')[-1]
            
            print(f'👁️ Visual event received: {event_type} - {visual_data}')
            
            # TODO: Integrate visual event into sensory processing
            # This is where you'll handle the visual event integration
            
        except Exception as e:
            print(f'Error processing visual event: {e}')


def main():
    """Main entry point for the sense node."""
    parser = argparse.ArgumentParser(description='Zenoh Sense Node')
    parser.add_argument('-c', '--character-name', default='default', help='Character name for topic paths')
    parser.add_argument('-config', default='{}', help='Character configuration as JSON string')
    
    args = parser.parse_args()
    
    # Parse character config
    try:
        character_config = json.loads(args.config)
    except json.JSONDecodeError as e:
        print(f"Error parsing character config: {e}")
        return
    
    sense_node = ZenohSenseNode(args.character_name, character_config)
    sense_node.run()


if __name__ == '__main__':
    main() 