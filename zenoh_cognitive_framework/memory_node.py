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
import argparse
from datetime import datetime
from typing import Dict, List, Any
from entity_model import EntityModel

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
    
    def __init__(self, character_name="default", character_config=None):
        # Store character info
        self.character_name = character_name
        self.character_config = character_config or {}
        
        # Initialize Zenoh session
        config = zenoh.Config()
        self.session = zenoh.open(config)
        self.short_term_memory = []
        self.chat_memory = []
        self.long_term_memory = []
        
        # Entity registry for tracking character interactions
        self.entity_models: Dict[str, EntityModel] = {}
        
        # Subscriber for incoming data to store (character-specific)
        self.data_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/sense_data",
            self._sense_data_callback
        )
        
        self.action_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/action",
            self._action_callback
        )
        
        # Subscriber for visual events (character-specific)
        self.visual_event_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/sense/visual/*",
            self._visual_event_callback
        )
        
        # Queryable storage for memory retrieval (character-specific)
        self.short_term_storage = self.session.declare_queryable(
            f"cognitive/{character_name}/memory/short_term/*",
            self.handle_short_term_query
        )
        
        # Queryable storage for memory retrieval (character-specific)
        self.chat_storage = self.session.declare_queryable(
            f"cognitive/{character_name}/memory/chat/*",
            self.handle_chat_query
        )
        
        # Queryable for entity data (character-specific)
        self.entity_storage = self.session.declare_queryable(
            f"cognitive/{character_name}/memory/entity/*",
            self.handle_entity_query
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
        
        logger.info(f'Memory Node initialized for character: {character_name}')
        logger.info('Storage available at:')
        logger.info(f'  - cognitive/{character_name}/memory/short_term/*')
        logger.info(f'  - cognitive/{character_name}/memory/chat/*')
        logger.info(f'  - cognitive/{character_name}/memory/long_term/*')
        logger.info(f'  - cognitive/{character_name}/memory/entity/*')
    
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
    
    def _sense_data_callback(self, sample):
        """Handle incoming data to store in memory."""
        try:
            data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            self.short_term_memory.append(data)
            
            # Track conversation entries for entity models
            if data.get('mode') == 'text' and 'content' in data:
                # Parse the content which should be JSON with source and text
                try:
                    content_data = json.loads(data['content'])
                    source = content_data.get('source', 'unknown')
                    text = content_data.get('text', '')
                    
                    if source and text:
                        # Add conversation entry for the source entity
                        self.add_conversation_entry(source, 'received', text, source)
                except (json.JSONDecodeError, KeyError):
                    # Fallback for old format or non-JSON content
                    pass
                    
        except Exception as e:
            logger.error(f'Error storing data: {e}')
    
    def _action_callback(self, sample):
        """Handle incoming data to store in memory."""
        try:
            data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            
            # Handle different action types
            action_type = data.get('action_type', 'unknown')
            
            if action_type == 'character_announcement':
                # Store announcement in short-term memory
                self.short_term_memory.append(data)
                logger.info(f'📢 Stored character announcement: {data.get("character_name", "unknown")}')
            elif action_type == 'cognitive_response':
                # Store chat interaction in chat memory
                if 'input_text' in data and 'llm_response' in data:
                    self.chat_memory.append([data['input_text'], data['llm_response']])
                    logger.info(f'💬 Stored chat interaction: {data["action_id"]}')
                    
                    # Add the character's response to entity conversation history
                    source = data.get('source', 'unknown')
                    llm_response = data['llm_response']
                    if source and llm_response:
                        # Add conversation entry: this character sent a response to the source
                        self.add_conversation_entry(source, 'sent', llm_response, self.character_name)
                        logger.info(f'💬 Added response to entity {source}: {self.character_name} -> "{llm_response[:50]}..."')
                else:
                    logger.warning(f'Missing input_text or llm_response in cognitive_response action: {data.get("action_id", "unknown")}')
            else:
                # Store other action types in short-term memory
                self.short_term_memory.append(data)
                logger.info(f'📝 Stored action: {action_type} - {data.get("action_id", "unknown")}')
                
        except Exception as e:
            logger.error(f'Error storing data: {e}')
    
    def _visual_event_callback(self, sample):
        """Handle visual events to update entity models."""
        try:
            data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            
            # Extract agent name from visual event
            agent_name = data.get('agent_name')
            if agent_name:
                # Update visual detection for the detected agent
                self.update_visual_detection(agent_name)
                
        except Exception as e:
            logger.error(f'Error processing visual event: {e}')

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
    
    def handle_chat_query(self, query):
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
            entries = self.chat_memory[-limit:]
            
            # Send response
            response = {
                'success': True,
                'entries': entries,
                'count': len(entries)
            }
            
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
            logger.info(f'📚 Chat query: returned {len(entries)} entries')
            
        except Exception as e:
            logger.error(f'Error handling chat query: {e}')
            query.reply(query.key_expr, json.dumps({'success': False, 'error': str(e), 'entries': []}).encode('utf-8'))
    
    def handle_entity_query(self, query):
        """Handle queries for entity data."""
        try:
            # Extract entity name from query key
            key_parts = str(query.key_expr).split('/')
            entity_name = key_parts[-1] if len(key_parts) > 0 else None
            
            if not entity_name:
                raise ValueError("No entity name provided")
            
            # Parse query parameters for limit
            selector = str(query.selector)
            limit = 20  # default limit
            
            # Extract limit from query if specified
            if 'limit=' in selector:
                try:
                    limit = int(selector.split('limit=')[1].split('&')[0])
                except:
                    pass
            
            # Check if entity exists
            if entity_name not in self.entity_models:
                response = {
                    'success': False,
                    'error': f"Entity '{entity_name}' not found"
                }
            else:
                entity = self.entity_models[entity_name]
                entity_data = entity.get_entity_data(limit)
                
                response = {
                    'success': True,
                    'entity_data': entity_data
                }
            
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
            logger.info(f'👥 Entity query for {entity_name}: returned data')
            
        except Exception as e:
            logger.error(f'Error handling entity query: {e}')
            error_response = {
                'success': False,
                'error': str(e)
            }
            query.reply(query.key_expr, json.dumps(error_response).encode('utf-8'))
    
    def get_or_create_entity(self, entity_name: str) -> EntityModel:
        """
        Get an existing entity or create a new one.
        
        Args:
            entity_name: Name of the entity
            
        Returns:
            EntityModel instance
        """
        if entity_name not in self.entity_models:
            self.entity_models[entity_name] = EntityModel(entity_name)
            logger.info(f'👥 Created new entity: {entity_name}')
        
        return self.entity_models[entity_name]
    
    def add_conversation_entry(self, entity_name: str, direction: str, text: str, source: str) -> None:
        """
        Add a conversation entry for an entity.
        
        Args:
            entity_name: Name of the entity
            direction: 'sent' or 'received'
            text: The message text
            source: 'ui' or character name
        """
        entity = self.get_or_create_entity(entity_name)
        entity.add_conversation_entry(direction, text, source)
        logger.info(f'💬 Added conversation entry for {entity_name}: {direction} "{text[:50]}..."')
    
    def update_visual_detection(self, entity_name: str) -> None:
        """
        Update visual detection for an entity.
        
        Args:
            entity_name: Name of the entity
        """
        entity = self.get_or_create_entity(entity_name)
        entity.update_visual_detection()
        logger.info(f'👁️ Updated visual detection for {entity_name}')

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
    parser = argparse.ArgumentParser(description='Zenoh Memory Node')
    parser.add_argument('-c', '--character-name', default='default', help='Character name for topic paths')
    parser.add_argument('-config', default='{}', help='Character configuration as JSON string')
    
    args = parser.parse_args()
    
    # Parse character config
    try:
        character_config = json.loads(args.config)
    except json.JSONDecodeError as e:
        print(f"Error parsing character config: {e}")
        return
    
    memory_node = ZenohMemoryNode(args.character_name, character_config)
    try:
        memory_node.run()
    finally:
        memory_node.shutdown()


if __name__ == '__main__':
    main() 