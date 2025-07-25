#!/usr/bin/env python3
"""
Zenoh Single LLM Action Example

This node demonstrates a simple cognitive loop:
1. Receive sense data
2. Make LLM call
3. Publish action
4. Store in memory

Replaces ROS2 complexity with simple Zenoh pub/sub.
"""

import zenoh
import json
import time
import threading
import logging
import sys
import signal
import argparse
from datetime import datetime
from typing import Dict, List, Any

# Configure logging with unbuffered output
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
    handlers=[
        logging.StreamHandler(sys.stdout),  # Console output
        logging.FileHandler('logs/action_node.log', mode='w')  # File output
    ],
    force=True
)
logger = logging.getLogger('action_node')

# Import LLM client
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__))))

try:
    from llm_client import ZenohLLMClient
    LLM_CLIENT_AVAILABLE = True
except ImportError as e:
    print(f"⚠️  LLM Client not available: {e}")
    LLM_CLIENT_AVAILABLE = False


class ZenohActionNode:
    """
    A simple cognitive node that:
    - Receives sense data
    - Makes LLM calls
    - Publishes actions
    - Stores results in memory
    """
    
    def __init__(self, character_name="default", character_config=None):
        # Store character info
        self.character_name = character_name
        self.character_config = character_config or {}
        
        # Initialize Zenoh session
        config = zenoh.Config()
        self.session = zenoh.open(config)
        
        # Subscriber for sense data (character-specific)
        self.sense_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/sense_data",
            self.sense_data_callback
        )
        
        # Publisher for actions (character-specific)
        self.action_publisher = self.session.declare_publisher(f"cognitive/{character_name}/action")
        
        # Publisher for memory storage (character-specific)
        self.memory_publisher = self.session.declare_publisher(f"cognitive/{character_name}/memory/store")
        
        # Publisher for text input to other characters (character-specific)
        self.text_input_publisher = self.session.declare_publisher(f"cognitive/{character_name}/text_input")
        
        # LLM client
        self.llm_client = None
        if LLM_CLIENT_AVAILABLE:
            self.llm_client = ZenohLLMClient(service_timeout=30.0)
        
        # Internal state
        self.action_counter = 0
        self.last_sense_data = None
        
        # Shutdown flag
        self.shutdown_requested = False
        
        # Register signal handlers for graceful shutdown
        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)
        
        logger.info(f'🧠 Zenoh Action Node initialized for character: {character_name}')
        logger.info(f'   - Subscribing to: cognitive/{character_name}/sense_data')
        logger.info(f'   - Publishing to: cognitive/{character_name}/action')
        logger.info(f'   - Publishing to: cognitive/{character_name}/memory/store')
        logger.info(f'   - Publishing to: cognitive/{character_name}/text_input')
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully."""
        logger.info(f'Received signal {signum}, initiating shutdown...')
        self.shutdown_requested = True
    
    def run(self):
        """Main node loop."""
        try:
            logger.info('Action Node running - press Ctrl+C to stop')
            
            # Announce character presence
            self._announce_character()
            
            while not self.shutdown_requested:
                time.sleep(1)
        except KeyboardInterrupt:
            logger.info('Action Node shutting down...')
        finally:
            self.shutdown()
    
    def _announce_character(self):
        """Announce character presence to the action display node."""
        try:
            # Create announcement action
            announcement_data = {
                'type': 'announcement',
                'action_id': f'announcement_{self.character_name}',
                'timestamp': datetime.now().isoformat(),
                'action_type': 'character_announcement',
                'character_name': self.character_name,
                'character_config': self.character_config,
                'message': f'Character {self.character_name} is ready for interaction'
            }
            
            # Publish announcement
            self.action_publisher.put(json.dumps(announcement_data))
            logger.info(f'📢 Announced character presence: {self.character_name}')
            
        except Exception as e:
            logger.error(f'Error announcing character: {e}')
    
    def send_text_input(self, target_character: str, message: str):
        """Send text input to another character."""
        try:
            # Create text input data in JSON format
            text_input_data = {
                'source': self.character_name,
                'text': message
            }
            
            # Publish to the target character's text_input topic
            target_publisher = self.session.declare_publisher(f"cognitive/{target_character}/text_input")
            target_publisher.put(json.dumps(text_input_data))
            
            logger.info(f'📤 Sent text input to {target_character}: "{message}" (source: {self.character_name})')
            
        except Exception as e:
            logger.error(f'Error sending text input to {target_character}: {e}')
    
    def sense_data_callback(self, sample):
        """Handle incoming sense data."""
        # Check if shutdown has been requested
        if self.shutdown_requested:
            return
            
        try:
            sense_data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            self.last_sense_data = sense_data
            
            # Extract text input from sense data
            content = sense_data['content']
            
            # Parse content - it could be JSON (external input) or plain text (console input)
            try:
                # Try to parse as JSON first (external input format)
                content_data = json.loads(content)
                text_input = content_data.get('text', '')
                source = content_data.get('source', 'unknown')
            except (json.JSONDecodeError, TypeError):
                # Fallback to plain text (console input format)
                text_input = content
                source = 'console'
            
            # Process if we have text input
            if text_input and text_input.strip():
                logger.info(f'📥 Received text input: "{text_input}" (source: {source})')
                
                # Process in background thread to avoid blocking
                thread = threading.Thread(
                    target=self._process_with_single_llm_call,
                    args=(text_input, source),
                    daemon=True
                )
                thread.start()
                
        except Exception as e:
            logger.error(f'Error processing sense data: {e}')
    
    def _process_with_single_llm_call(self, text_input: str, source: str):
        """Process text input with a single LLM call."""
        try:
            logger.info(f'🧠 Making single LLM call for: "{text_input}" (from {source})')
            
            # Get recent memory entries for context
            recent_memories = self._get_recent_chat_memories(3)
            
            # Get entity context if source is not console
            entity_context = None
            if source != 'console':
                entity_context = self.get_entity_context(source, 10)
            
            # Simple, focused prompt
            system_prompt = """You are a helpful AI assistant. 
            Analyze the input and provide a clear, actionable response.
            Focus on being helpful and direct."""
            if self.character_config.get('character', None):
                system_prompt = self.character_config['character']
            if self.character_config.get('drives', None):
                system_prompt += f"\n\nYour drives are: {self.character_config['drives']}"
            
            # Build user prompt with context
            user_prompt = ''
            if entity_context:
                for i, memory in enumerate(entity_context['conversation_history']):  # Use last 2 memories
                    user_prompt += f"{memory['source']}: {memory['text']}\n"
                    
            # Make LLM call
            if self.llm_client:
                response = self.llm_client.generate(
                    messages=[system_prompt, user_prompt],
                    max_tokens=200,
                    temperature=0.7
                )

                if response.success:
                    logger.info(f'🤖 LLM Response: {response.text}')
                    
                    # Create action
                    action_data = {
                        'type': 'action',
                        'action_id': f'action_{self.action_counter}',
                        'timestamp': datetime.now().isoformat(),
                        'action_type': 'cognitive_response',
                        'input_text': text_input,
                        'llm_response': response.text,
                        'confidence': 0.8,
                        'source': source
                     }
                    
                    # Publish action (this will be picked up by action_display_node)
                    self.action_publisher.put(json.dumps(action_data))
                    logger.info(f'📤 Published action: {action_data["action_id"]}')
                    
                    # Store in memory
                    #self._store_in_memory(text_input, response.text, action_data)
                    
                    self.action_counter += 1
                else:
                    logger.error(f'LLM call failed: {response.error}')
            else:
                logger.error('LLM client not available')
                
        except Exception as e:
            logger.error(f'Error in LLM processing: {e}')
    
    def _get_recent_chat_memories(self, num_entries: int) -> List[Dict[str, Any]]:
        """Get recent memory entries using Zenoh queries."""
        try:
            # Query short-term memory
            entries = []
            for reply in self.session.get(f"cognitive/{self.character_name}/memory/chat/*"):
                try:
                    if reply.ok:
                        content = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                        entries.append(content)
                except Exception as e:
                    logger.error(f'Error getting recent memories: {e}')
                    continue
            
            m1 = entries[0]
            m2 = m1['entries']

            logger.info(f'📚 Retrieved {len(entries)} recent memory entries')
            return m2
            
        except Exception as e:
            logger.error(f'Error getting recent memories: {e}')
            return []
    
    def _store_in_memory(self, input_text: str, response_text: str, action_data: Dict[str, Any]):
        """Store the interaction in memory."""
        try:
            memory_data = {
                'memory_type': 'short_term',
                'key': f'interaction_{int(time.time())}',
                'content': {
                    'type': 'llm_interaction',
                    'input_text': input_text,
                    'response_text': response_text,
                    'action_id': action_data['action_id'],
                    'timestamp': datetime.now().isoformat(),
                    'metadata': {
                        'node': 'action_node',
                        'processing_time': 0.0
                    }
                }
            }
            
            self.memory_publisher.put(json.dumps(memory_data))
            logger.info(f'💾 Stored interaction in memory')
            
        except Exception as e:
            logger.error(f'Error storing in memory: {e}')
    
    def get_entity_context(self, entity_name: str, limit: int = 20) -> Dict[str, Any]:
        """
        Query entity data from memory node for context.
        
        Args:
            entity_name: Name of the entity to query
            limit: Number of recent conversation entries to include (default 20)
            
        Returns:
            Dictionary with entity data or None if query failed
        """
        try:
            # Query entity data from memory node - try without query parameters first
            for reply in self.session.get(f"cognitive/{self.character_name}/memory/entity/{entity_name}"):
                try:
                    if reply.ok:
                        data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                        if data['success']:
                            logger.info(f'👥 Retrieved entity context for {entity_name}')
                            return data['entity_data']
                        else:
                            logger.warning(f'Entity query failed for {entity_name}: {data.get("error", "Unknown error")}')
                            return None
                except Exception as e:
                    logger.error(f'Error parsing entity query response for {entity_name}: {e}')
                    return None
            
            logger.warning(f'No response received for entity query: {entity_name}')
            return None
            
        except Exception as e:
            logger.error(f'Error querying entity context for {entity_name}: {e}')
            return None
    
    def shutdown(self):
        """Clean shutdown."""
        logger.info('Shutting down Action Node...')
        
        if self.llm_client:
            self.llm_client.cleanup()
        
        self.session.close()
        logger.info('Action Node shutdown complete')


def main():
    """Main entry point for the action node."""
    parser = argparse.ArgumentParser(description='Zenoh Action Node')
    parser.add_argument('-c', '--character-name', default='default', help='Character name for topic paths')
    parser.add_argument('-config', default='{}', help='Character configuration as JSON string')
    
    args = parser.parse_args()
    
    # Parse character config
    try:
        character_config = json.loads(args.config)
    except json.JSONDecodeError as e:
        print(f"Error parsing character config: {e}")
        return
    
    action_node = ZenohActionNode(args.character_name, character_config)
    try:
        action_node.run()
    finally:
        action_node.shutdown()


if __name__ == '__main__':
    main() 