#!/usr/bin/env python3
"""
Zenoh Situation Node

This node maintains situational awareness by monitoring sense data and actions.
Replaces ROS2 complexity with simple Zenoh pub/sub.
"""

import traceback
import zenoh
import json
import time
import logging
import sys
import signal
import argparse
import os
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Any

from utils import hash_utils

# Configure logging with unbuffered output
# Console handler with WARNING level (less verbose)
console_handler = logging.StreamHandler(sys.stdout)
console_handler.setLevel(logging.WARNING)

# File handler with INFO level (full logging)
file_handler = logging.FileHandler('logs/situation_node.log', mode='w')
file_handler.setLevel(logging.INFO)

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
    handlers=[console_handler, file_handler],
    force=True
)
logger = logging.getLogger('situation_node')


class ZenohSituationNode:
    """
    The Situation node maintains situational awareness by monitoring:
    - Sense data (perceptions, inputs)
    - Actions (responses, behaviors)
    - Map data (location, environment)
    
    Features:
    - Real-time situation tracking
    - Map integration
    - Context awareness
    - Situation history
    """
    
    def __init__(self, character_name="default", character_config=None):
        # Store character info (canonicalized)
        self.character_name = character_name.capitalize()
        self.character_config = character_config or {}
        
        # Initialize Zenoh session
        config = zenoh.Config()
        self.session = zenoh.open(config)
        
        # Situation state
        self.situation = {
            'look': None,  # Will contain map look contents
            'adjacent_to': {
                'resources': [],
                'characters': []
            }
        }
        
        # Persistence setup
        self.situation_file = Path(f"data/situation/{character_name}_situation.json")
        self.situation_file.parent.mkdir(parents=True, exist_ok=True)
        
        # Load existing situation data
        self.load_situation()
        
        # Subscriber for sense data (character-specific)
        self.sense_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/sense_data",
            self.sense_data_callback
        )
        
        # Subscriber for actions (character-specific)
        self.action_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/action",
            self.action_callback
        )
        
        # Publisher for situation updates (character-specific)
        self.situation_publisher = self.session.declare_publisher(f"cognitive/{character_name}/situation")
        
        # Queryable for situation data (character-specific)
        self.situation_storage = self.session.declare_queryable(
            f"cognitive/{character_name}/situation/*",
            self.handle_situation_query
        )
        
        # Shutdown flag
        self.shutdown_requested = False
        
        # Register signal handlers for graceful shutdown
        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)
        
        logger.info(f'🧭 Situation Node initialized for character: {character_name}')
        logger.info(f'   - Subscribing to: cognitive/{character_name}/sense_data')
        logger.info(f'   - Subscribing to: cognitive/{character_name}/action')
        logger.info(f'   - Publishing to: cognitive/{character_name}/situation')
        logger.info(f'   - Queryable at: cognitive/{character_name}/situation/*')
        logger.info(f'   - Storage available at: {self.situation_file}')
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully."""
        logger.info(f'Received signal {signum}, initiating shutdown...')
        self.shutdown_requested = True
    
    def run(self):
        """Main situation node loop."""
        try:
            logger.info('Situation Node running - press Ctrl+C to stop')
            
            while not self.shutdown_requested:
                time.sleep(1)
        except KeyboardInterrupt:
            logger.info('Situation Node shutting down...')
        finally:
            self.session.close()
    
    def sense_data_callback(self, sample):
        """Handle incoming sense data to update situation."""
        try:
            data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            logger.info(f'📥 Received sense data: {data.get("mode", "unknown")}')
            
            # Update situation based on sense data
            self._update_situation_from_sense_data(data)
            
            # Publish updated situation
            self._publish_situation()
            
        except Exception as e:
            logger.error(f'Error processing sense data: {e}')
    
    def action_callback(self, sample):
        """Handle incoming actions to update situation."""
        try:
            data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            logger.info(f'📥 Received action: {data.get("action_type", "unknown")}')
            
            # Update situation based on action
            self._update_situation_from_action(data)
            
            # Publish updated situation
            self._publish_situation()
            
        except Exception as e:
            logger.error(f'Error processing action: {e}')
    
    def _update_situation_from_sense_data(self, sense_data: Dict[str, Any]):
        """Update situation based on incoming sense data."""
        try:
            # TODO: Add map lookup query here
            #self._update_map_data()
            pass
            
        except Exception as e:
            logger.error(f'Error updating situation from sense data: {e}')
    
    def _update_situation_from_action(self, action_data: Dict[str, Any]):
        """Update situation based on incoming action."""
        try:
            if 'announcement' in action_data.get('type') or 'move' in action_data.get('type'):
                self._update_map_data()
            
        except Exception as e:
            logger.error(f'Error updating situation from action: {e}')
    
    def _update_map_data(self):
        """Update map data through lookup query."""
        try:
            # Query map node for agent look data with timeout
            for reply in self.session.get(f"cognitive/map/agent/{self.character_name}/look", timeout=10.0):
                try:
                    if reply.ok:
                        map_look_data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                        if not map_look_data['success']:
                            logger.warning(f'Map query failed for {self.character_name}: {map_look_data["error"]}')
                            return
                        location = map_look_data['location']
                        visible_characters = map_look_data['characters']
                        views = hash_utils.findall('view', map_look_data['look_result'])
                        
                        # Extract adjacent resources and characters (within 1 unit)
                        adjacent_resources = []
                        adjacent_characters = []
                        
                        # Check if resources and characters are in the response
                        if 'resources' in map_look_data:
                            for resource in map_look_data['resources']:
                                if isinstance(resource, dict) and 'distance' in resource:
                                    if resource['distance'] <= 1:
                                        adjacent_resources.append(resource.get('id', resource))
                                else:
                                    # If it's just a string, assume it's adjacent
                                    adjacent_resources.append(resource)
                        
                        if 'characters' in map_look_data:
                            for character in map_look_data['characters']:
                                if isinstance(character, dict) and 'distance' in character:
                                    if character['distance'] <= 1:
                                        adjacent_characters.append(character.get('name', character))
                                else:
                                    # If it's just a string, assume it's adjacent
                                    adjacent_characters.append(character)
                        
                        if views:
                            #self.situation['delta'] = self.delta(self.situation['look'], views)
                            self.situation['look'] = views
                            self.situation['location'] = location
                            self.situation['visible_characters'] = visible_characters
                            self.situation['adjacent_to']['resources'] = adjacent_resources
                            self.situation['adjacent_to']['characters'] = adjacent_characters
                            logger.info(f'🗺️ Updated map look data for {self.character_name}')
                            logger.debug(f'   Adjacent resources: {adjacent_resources}')
                            logger.debug(f'   Adjacent characters: {adjacent_characters}')
                            # Save and publish updated situation
                            self.save_situation()
                            self._publish_situation()
                        else:
                            logger.debug(f'No map look data available for {self.character_name}')
                    else:
                        logger.warning(f'Map query failed for {self.character_name}')
                except Exception as e:
                    logger.error(f'Error parsing map look response for {self.character_name}: {e}')
                    logger.error(traceback.format_exc())
                    continue
            
        except Exception as e:
            if "timeout" in str(e).lower():
                logger.debug(f'Map query timeout for {self.character_name} (map node may be busy)')
            else:
                logger.error(f'Error updating map data: {e}')
    
    def delta(self, old_data, new_data):
        """Calculate the delta between two data sets."""
        delta = {}
        for key, value in new_data.items():
            new_value_set = set(value.split(','))
            if key in old_data:
                old_value_set = set(old_data[key].split(','))
                delta[key] = new_value_set - old_value_set
            else:
                delta[key] = new_value_set
        return delta
    
    def _publish_situation(self):
        """Publish current situation."""
        try:
            # Publish situation
            self.situation_publisher.put(json.dumps(self.situation))
            logger.debug(f'📤 Published situation update for {self.character_name}')
            
        except Exception as e:
            logger.error(f'Error publishing situation: {e}')
    
    def handle_situation_query(self, query):
        """Handle queries for situation data."""
        try:
            # Parse query parameters
            selector = str(query.selector)
            
            # For now, just return the current situation
            response = {
                'success': True,
                'situation': self.situation
            }
            
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
            logger.info(f'🧭 Situation query: returned current situation')
            
        except Exception as e:
            logger.error(f'Error handling situation query: {e}')
            error_response = {
                'success': False,
                'error': str(e)
            }
            query.reply(query.key_expr, json.dumps(error_response).encode('utf-8'))
    
    def load_situation(self):
        """Load situation data from file."""
        try:
            if self.situation_file.exists():
                with open(self.situation_file, 'r') as f:
                    loaded_situation = json.load(f)
                    self.situation.update(loaded_situation)
                    logger.info(f'📂 Loaded situation data for {self.character_name}')
            else:
                logger.info(f'📂 No existing situation file for {self.character_name}, starting fresh')
        except Exception as e:
            logger.error(f'Error loading situation data: {e}')
    
    def save_situation(self):
        """Save situation data to file."""
        try:
            with open(self.situation_file, 'w') as f:
                json.dump(self.situation, f, indent=2)
            logger.debug(f'💾 Saved situation data for {self.character_name}')
        except Exception as e:
            logger.error(f'Error saving situation data: {e}')
    
    def shutdown(self):
        """Cleanup and shutdown."""
        try:
            logger.info('Situation Node shutdown initiated...')
            self.save_situation()
            
            # Close Zenoh session more carefully
            try:
                # Wait longer for cleanup to avoid Zenoh panics
                time.sleep(2.0)
                self.session.close()
                logger.info('Zenoh session closed')
            except Exception as e:
                logger.error(f'Error closing Zenoh session: {e}')
            
            logger.info('Situation Node shutdown complete')
        except Exception as e:
            logger.error(f'Error during shutdown: {e}')


def main():
    """Main entry point for the situation node."""
    parser = argparse.ArgumentParser(description='Zenoh Situation Node')
    parser.add_argument('-c', '--character-name', default='default', help='Character name for topic paths')
    parser.add_argument('-config', default='{}', help='Character configuration as JSON string')
    
    args = parser.parse_args()
    
    # Parse character config
    try:
        character_config = json.loads(args.config)
    except json.JSONDecodeError as e:
        print(f"Error parsing character config: {e}")
        return
    
    situation_node = ZenohSituationNode(args.character_name, character_config)
    try:
        situation_node.run()
    finally:
        situation_node.shutdown()


if __name__ == '__main__':
    main() 