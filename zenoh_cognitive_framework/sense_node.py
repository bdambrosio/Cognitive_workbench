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
from datetime import datetime


class ZenohSenseNode:
    """
    The Sense node handles perception and sensory input processing.
    
    Publishers:
        cognitive/sense_data: Raw sensory information
    """
    
    def __init__(self):
        # Initialize Zenoh session
        config = zenoh.Config()
        self.session = zenoh.open(config)
        
        # Publisher for sense data
        self.sense_publisher = self.session.declare_publisher("cognitive/sense_data")
        
        # Subscriber for external text input
        self.text_input_subscriber = self.session.declare_subscriber(
            "cognitive/text_input",
            self.external_text_callback
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
        
        print('Sense Node initialized - starting perception loop')
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


def main():
    """Main entry point for the sense node."""
    sense_node = ZenohSenseNode()
    sense_node.run()


if __name__ == '__main__':
    main() 