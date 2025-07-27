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
import logging
import signal
from datetime import datetime

# Configure logging with unbuffered output
# Console handler with WARNING level (less verbose)
console_handler = logging.StreamHandler(sys.stdout)
console_handler.setLevel(logging.WARNING)

# File handler with INFO level (full logging)
file_handler = logging.FileHandler('logs/sense_node.log', mode='w')
file_handler.setLevel(logging.INFO)

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
    handlers=[console_handler, file_handler],
    force=True
)
logger = logging.getLogger('sense_node')


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
        
        # Shutdown flag (must be set before starting threads)
        self.shutdown_requested = False
        
        # Register signal handlers for graceful shutdown
        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)
        
        # Only start console input thread if no external input is expected
        self.input_thread = threading.Thread(target=self._console_input_thread, daemon=True)
        self.input_thread.start()
        
        logger.info(f'👁️ Sense Node initialized for character: {character_name}')
        logger.info('Console Text Sensor ready - will auto-disable if external input detected')
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully."""
        logger.info(f'Received signal {signum}, initiating shutdown...')
        self.shutdown_requested = True
    
    def run(self):
        """Main sensing loop."""
        try:
            logger.info('Sense Node running - press Ctrl+C to stop')
            
            while not self.shutdown_requested:
                # Get console input if available
                console_input = self._get_console_input()
                if not console_input:
                    time.sleep(0.1)  # 10Hz sensing
                    continue
                
                # Create structured sense data message
                # For external input, content should be JSON with source and text
                # For console input, content is just the text string
                if hasattr(self, 'last_external_source') and self.last_external_source:
                    # External input - preserve source information
                    content = json.dumps({
                        'source': self.last_external_source,
                        'text': console_input
                    })
                    self.last_external_source = None  # Reset after use
                else:
                    # Console input - just the text
                    content = console_input
                
                sense_data = {
                    'timestamp': datetime.now().isoformat(),
                    'sequence_id': self.sequence_id,
                    'mode': 'text',
                    'content': content
                }
                
                # Publish sense data
                self.sense_publisher.put(json.dumps(sense_data))
                
                # Log
                logger.info(f'📤 Published sense data: {self.sequence_id}')
                
                self.sequence_id += 1
                
        except KeyboardInterrupt:
            logger.info('Sense Node shutting down...')
        finally:
            self.shutdown()
    
    def _console_input_thread(self):
        """
        Background thread that reads console input and queues it for processing.
        
        This allows the main Zenoh loop to continue while waiting for user input.
        Will automatically disable if external input (from Action Display) is detected.
        """
        try:
            while not self.shutdown_requested:
                try:                   
                    # Prompt for input (will only show in terminal, not in logs)
                    user_input = input()
                    if user_input.strip():  # Only process non-empty input
                        # Double-check external input hasn't been activated while we were waiting
                        if not self.external_input_active:
                            self.text_input_queue.put(user_input.strip())
                            # Log to console instead of ROS2
                            logger.info(f'📝 Console text input received: "{user_input.strip()}"')
                        else:
                            logger.info('🔄 Ignoring console input - external input is now active')
                            break
                except EOFError:
                    # Handle Ctrl+D gracefully
                    break
                except KeyboardInterrupt:
                    # Handle Ctrl+C gracefully
                    break
        except Exception as e:
            logger.error(f'Console input thread error: {str(e)}')
    
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
            # Parse JSON format with source and text fields
            payload = sample.payload.to_bytes().decode('utf-8').strip()
            text_input_data = json.loads(payload)
            
            source = text_input_data.get('source', 'unknown')
            text = text_input_data.get('text', '').strip()
            
            if text:
                # Mark external input as active to prevent console input conflicts
                if not self.external_input_active:
                    self.external_input_active = True
                    logger.info('🔄 External input detected - console input disabled to prevent conflicts')
                
                # Store the source for use in the main loop
                self.last_external_source = source
                
                # Add to the queue (only from external source now)
                self.text_input_queue.put(text)
                logger.info(f'📨 Received external text input from {source}: "{text}"')
        except Exception as e:
            logger.error(f'Error processing external text input: {e}')
    
    def visual_event_callback(self, sample):
        """Handle visual events from the map node"""
        try:
            # Parse the visual event data
            visual_data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            event_type = str(sample.key_expr).split('/')[-1]
            
            logger.info(f'👁️ Visual event received: {event_type} - {visual_data}')
            
            # TODO: Integrate visual event into sensory processing
            # This is where you'll handle the visual event integration
            
        except Exception as e:
            logger.error(f'Error processing visual event: {e}')
    
    def shutdown(self):
        """Cleanup and shutdown."""
        try:
            logger.info('Sense Node shutdown initiated...')
            self.session.close()
            logger.info('Sense Node shutdown complete')
        except Exception as e:
            logger.error(f'Error during shutdown: {e}')


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
        logger.error(f"Error parsing character config: {e}")
        return
    
    sense_node = ZenohSenseNode(args.character_name, character_config)
    try:
        sense_node.run()
    finally:
        sense_node.shutdown()


if __name__ == '__main__':
    main() 