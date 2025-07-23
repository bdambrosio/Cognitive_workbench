#!/usr/bin/env python3
"""
Zenoh Memory Node

This node provides memory storage and retrieval using Zenoh's built-in storage.
Replaces ROS2 service complexity with direct Zenoh storage operations.
"""

import zenoh
import json
import time
import logging
import sys
import signal
from datetime import datetime
from typing import Dict, List, Any

# Configure logging with unbuffered output
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
    handlers=[
        logging.StreamHandler(sys.stdout),  # Console output
        logging.FileHandler('logs/memory_node.log', mode='w')  # File output
    ],
    force=True
)
logger = logging.getLogger('memory_node')


class ZenohMemoryNode:
    """
    The Memory node provides persistent storage and retrieval of cognitive data.
    
    Features:
    - Short-term memory (recent events)
    - Working memory (current context)
    - Long-term memory (persistent knowledge)
    - Automatic cleanup and management
    """
    
    def __init__(self):
        # Initialize Zenoh session
        config = zenoh.Config()
        self.session = zenoh.open(config)
        self.short_term_memory = []
        self.working_memory = []
        self.long_term_memory = []
        
        # Subscriber for incoming data to store
        self.data_subscriber = self.session.declare_subscriber(
            "cognitive/sense_data",
            self.store_data_callback
        )
        
        # Queryable storage for memory retrieval
        self.short_term_storage = self.session.declare_queryable(
            "cognitive/memory/short_term/*",
            self.handle_short_term_query
        )
        
        # Memory management
        self.max_short_term_entries = 1000
        self.max_working_entries = 100
        self.cleanup_interval = 60  # seconds
        
        # Shutdown flag
        self.shutdown_requested = False
        
        # Start cleanup thread
        import threading
        #self.cleanup_thread = threading.Thread(target=self._cleanup_loop, daemon=True)
        #self.cleanup_thread.start()
        
        # Register signal handlers for graceful shutdown
        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)
        
        logger.info('Memory Node initialized')
        logger.info('Storage available at:')
        logger.info('  - cognitive/memory/short_term/*')
        logger.info('  - cognitive/memory/working/*')
        logger.info('  - cognitive/memory/long_term/*')
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully."""
        logger.info(f'Received signal {signum}, initiating shutdown...')
        self.shutdown_requested = True
    
    def run(self):
        """Main memory node loop."""
        try:
            logger.info('Memory Node running - press Ctrl+C to stop')
            while not self.shutdown_requested:
                time.sleep(1)
        except KeyboardInterrupt:
            logger.info('Memory Node shutting down...')
        finally:
            self.session.close()
    
    def store_data_callback(self, sample):
        """Handle incoming data to store in memory."""
        try:
            data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            self.short_term_memory.append(data)                
        except Exception as e:
            logger.error(f'Error storing data: {e}')
    
    
    def handle_short_term_query(self, query):
        """Handle queries for short-term memory."""
        try:
            # Parse query parameters
            selector = str(query.selector)  # Convert Selector to string
            limit = 10  # default limit
            
            # Extract limit from query if specified
            if 'limit=' in selector:
                try:
                    limit = int(selector.split('limit=')[1].split('&')[0])
                except:
                    pass
            
            # Get recent entries
            entries = self.short_term_memory[-limit:]
            
            # Send response
            response = {
                'success': True,
                'entries': entries,
                'count': len(entries)
            }
            
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
            logger.info(f'📚 Short-term query: returned {len(entries)} entries')
            
        except Exception as e:
            logger.error(f'Error handling short-term query: {e}')
            query.reply(query.key_expr, json.dumps({'success': False, 'error': str(e), 'entries': []}).encode('utf-8'))
    

    def shutdown(self):
        """Cleanup and shutdown."""
        try:
            logger.info('Memory Node shutdown initiated...')
            self.session.close()
            logger.info('Memory Node shutdown complete')
        except Exception as e:
            logger.error(f'Error during shutdown: {e}')


def main():
    """Main entry point for the memory node."""
    memory_node = ZenohMemoryNode()
    try:
        memory_node.run()
    finally:
        memory_node.shutdown()


if __name__ == '__main__':
    main() 