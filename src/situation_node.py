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
import urllib.parse


from utils import hash_utils, zenoh_utils

# Configure logging with unbuffered output
# Console handler with WARNING level (less verbose)
console_handler = logging.StreamHandler(sys.stdout)
console_handler.setLevel(logging.WARNING)
if os.getenv('CWB_DEBUG', '') in ('1', 'true', 'yes', 'on'):
    console_handler.setLevel(logging.INFO)

# File handler with INFO level (full logging)
file_handler = logging.FileHandler('logs/situation_node.log', mode='w')
file_handler.setLevel(logging.WARNING)
if os.getenv('CWB_DEBUG', '') in ('1', 'true', 'yes', 'on'):
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
        
        # Debug mode flag
        self.debug = str(os.getenv('CWB_DEBUG', '')).lower() in ('1', 'true', 'yes', 'on')
        if self.debug:
            logger.info(f'🔧 Debug mode enabled for {self.character_name}')
        
        # Initialize Zenoh session
        config = zenoh.Config()
        self.session = zenoh.open(config)
        
        # Situation state
        self.situation = {
            'location': None,
            'views': [],
            'visible_characters': [],
            'adjacent_to': {
                'resources': [],
                'characters': []
            },
            'resources': [],
            'characters': []
        }
        
        # Thresholds for condition evaluation
        self.near_threshold = 2.0  # Distance threshold for "near" condition
        self.at_location_threshold = 1.0  # Distance threshold for "at_location" condition
        
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

        # Subscriber for explicit situation update requests (step_look, etc.)
        # Executive publishes to this topic after actions that change visibility/adjacency
        self.request_update_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/situation/request_update",
            self.map_update_callback
        )
                
        # Publisher for situation updates (character-specific)
        self.situation_publisher = self.session.declare_publisher(f"cognitive/{character_name}/situation/update")
        
        # Queryable for situation data (character-specific)
        self.situation_storage = self.session.declare_queryable(
            f"cognitive/{character_name}/situation/current_situation",
            self.handle_situation_query
        )
        
        # Queryables for condition evaluation (character-specific)
        self.proximity_queryable = self.session.declare_queryable(
            f"cognitive/{character_name}/situation/proximity",
            self.handle_proximity_query
        )
        
        self.visibility_queryable = self.session.declare_queryable(
            f"cognitive/{character_name}/situation/visibility",
            self.handle_visibility_query
        )
        
        self.location_queryable = self.session.declare_queryable(
            f"cognitive/{character_name}/situation/location",
            self.handle_location_query
        )
        
        # Subscriber for save commands (global)
        self.save_subscriber = self.session.declare_subscriber(
            "cognitive/save_all",
            self.save_callback
        )
        
        # Subscriber for shutdown commands (global)
        self.shutdown_subscriber = self.session.declare_subscriber(
            "cognitive/shutdown/situation",
            self.shutdown_callback
        )
        
        # Shutdown flags
        self.shutdown_requested = False
        self._shutting_down = False
        self.update_map_retries = 0
        self.map_types = {}
        for reply in self.session.get("cognitive/map/types", timeout=2.0 if not self.debug else 600.0):
            if reply.ok:
                data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                if data.get('success'):
                    self.map_types = data
                    break
        # Register signal handlers for graceful shutdown
        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)
        
        logger.info(f'🧭 Situation Node initialized for character: {character_name}')
        logger.info(f'   - Subscribing to: cognitive/{character_name}/sense_data')
        logger.info(f'   - Subscribing to: cognitive/{character_name}/action')
        logger.info(f'   - Subscribing to: cognitive/{character_name}/situation/request_update')
        logger.info(f'   - Publishing to: cognitive/{character_name}/situation/update')
        logger.info(f'   - Queryable at: cognitive/{character_name}/situation/current_situation')
        logger.info(f'   - Proximity queryable at: cognitive/{character_name}/situation/proximity')
        logger.info(f'   - Visibility queryable at: cognitive/{character_name}/situation/visibility')
        logger.info(f'   - Location queryable at: cognitive/{character_name}/situation/location')
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
            logger.info(f'📥 Received action: {data.get("type", "unknown")}')
            
            if data.get('type') == 'move' or data.get('type') == 'announcement':
                # Update situation based on action
                self._update_situation_from_action(data)
            
        except Exception as e:
            logger.error(f'Error processing action: {e}')
    
    def map_update_callback(self, sample):
        """Handle incoming actions to update situation."""
        try:
            self.update_map_retries = 0
            self._update_map_data()
            
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
                self.update_map_retries = 0
                self._update_map_data()
            
        except Exception as e:
            logger.error(f'Error updating situation from action: {e}')
    
    def _update_map_data(self):
        """Update map data through lookup query."""
        try:
            # Query map node for agent look data with timeout
            logger.info(f'Updating map data for {self.character_name}')
            for reply in self.session.get(f"cognitive/map/agent/{self.character_name}/look", timeout=4.0 if not self.debug else 600.0):
                try:
                    if reply.ok:
                        map_look_data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                    if reply.ok and map_look_data['success']:
                        for character in map_look_data['characters']:
                            if type(character) == dict:
                                logger.error(f'Character is a dict: {character}')
                        self.situation['location'] = map_look_data['location']
                        self.situation['visible_characters'] = map_look_data['characters']
                        view_strings = hash_utils.findall('view', map_look_data['look_result'])
                        self.situation['views'] = [self.parse_view_string(view_string) for view_string in view_strings]
                        self.situation['adjacent_to']['resources'] = []
                        self.situation['adjacent_to']['characters'] = []
                        self.situation['adjacent_to']['paths'] = []
                        self.situation['resources'] = []
                        self.situation['paths'] = []
                        self.situation['characters'] = map_look_data['characters']
                        self.situation['paths'] = map_look_data['paths']

                        # Check if resources and characters are in the response
                        for view in self.situation['views']:
                            for resource in view['resources']:
                                if isinstance(resource, dict) and 'distance' in resource:
                                    if resource['distance'] <= 1:
                                        self.situation['adjacent_to']['resources'].append(resource['name'])
                                    if resource['name'] not in self.situation['resources']:
                                        self.situation['resources'].append(resource['name'])
                        
                            for path in view['paths']:
                                if isinstance(path, dict) and 'distance' in path:
                                    if path['distance'] <= 1:
                                        self.situation['adjacent_to']['paths'].append(path['name'])
                                    if path['name'] not in self.situation['paths']:
                                        self.situation['paths'].append(path['name'])
                        
                            for character in view['characters']:
                                if isinstance(character, dict) and 'distance' in character:
                                    if character['distance'] <= 1 and (character not in self.situation['adjacent_to']['characters']):
                                        self.situation['adjacent_to']['characters'].append(character['name'])
                                    if character['name'] not in self.situation['characters']:
                                        self.situation['characters'].append(character['name'])
                        self.situation['adjacent_to']['characters'] = list(set(self.situation['adjacent_to']['characters']))
                        self.situation['adjacent_to']['resources'] = list(set(self.situation['adjacent_to']['resources']))
                        logger.info(f'🗺️ Updated map look data for {self.character_name}')
                        logger.debug(f'   Adjacent resources: {self.situation["adjacent_to"]["resources"]}')
                        logger.debug(f'   Adjacent characters: {self.situation["adjacent_to"]["characters"]}')
                        # Save and publish updated situation
                        self.save_situation()
                        self._publish_situation()
                        logger.info(f'Map look data updated for {self.character_name}')
                    else:
                        reply_str = str(reply)
                        decoded_error = zenoh_utils.decode_zenoh_error_payload(reply_str)
                        if 'timeout' in decoded_error or 'not found' in decoded_error:
                            self.update_map_retries += 1
                            if self.update_map_retries < 3:
                                self._update_map_data()
                            else:
                                logger.error(f'Map query timeout for {self.character_name} (map node may be busy)')
                except Exception as e:
                    logger.error(f'Error parsing map look response for {self.character_name}: {e}')
                    traceback.print_exc()
                    continue
            
        except Exception as e:
            if "timeout" in str(e).lower():
                logger.debug(f'Map query timeout for {self.character_name} (map node may be busy)')
            else:
                logger.error(f'Error updating map data: {e}')
    
    def delta(self, old_data, new_data):
        """Calculate the delta between two data sets. tbd"""
        delta = {}
        
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
            response = {'success': True,'situation': self.situation}
            
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
            logger.info(f'🧭 Situation query: returned current situation')
            
        except Exception as e:
            logger.error(f'Error handling situation query: {e}')
            error_response = {'success': False, 'error': str(e) }
            query.reply(query.key_expr, json.dumps(error_response).encode('utf-8'))
    
    def handle_proximity_query(self, query):
        """Handle proximity queries for condition evaluation.
        attempts to find a binding that satisfies the condition"""
        try:
            # Parse query parameters
            selector = str(query.selector)
            target = None
            response = None
            # Extract target from query
            if 'target=' in selector:
                try:
                    target = urllib.parse.unquote(selector.split('target=')[1].split('&')[0])
                except:
                    pass
            if not target:
                response = { 'success': False,'value': False, 'binding': None}
            else:
                if 'negated=' in selector:
                    negated = urllib.parse.unquote(selector.split('negated=')[1].split('&')[0])
                    negated = negated.lower() == 'true'
                else:
                    negated = False
                # Check if target is in current situation and distance < near_threshold
                target_canonical = target.capitalize()
                response = None
                for view in self.situation['views']:
                    if view.get('terrain', '').capitalize() == target_canonical:
                        response = {'success': True, 'value': not negated, 'binding': view['direction']}
                        break
                    # Check resources in this direction
                    if not response:
                        for resource in view.get('resources', []):
                            if resource['name'].capitalize() == target_canonical:
                                if ((not negated and resource.get('distance', float('inf')) <= self.near_threshold)
                                    or (negated and resource.get('distance', float('inf')) > self.near_threshold)):
                                    response = {'success': True, 'value': True, 'binding': resource['name']}
                                else:
                                    response = {'success': True, 'value': False, 'binding': resource['name']}
                                break
                    # Check paths in this direction
                    if not response:
                        for path in view.get('paths', []):
                            if path.get('name', '').capitalize() == target_canonical:
                                if ((not negated and path.get('distance', float('inf')) <= self.near_threshold)
                                    or (negated and path.get('distance', float('inf')) > self.near_threshold)):
                                    response = {'success': True, 'value': True, 'binding': path['name']}
                                else:
                                    response = {'success': True, 'value': False, 'binding': path['name']}
                                break
                    # Check characters if not found in resources
                    if not response:
                        for character in view.get('characters', []):
                            if ('name' in character and character['name'].capitalize() == target_canonical):
                                if ((not negated and character.get('distance', float('inf')) <= self.near_threshold) 
                                    or (negated and character.get('distance', float('inf')) > self.near_threshold)):
                                    response = {'success': True, 'value': True, 'binding': character['name']}
                                else:
                                    response = {'success': True, 'value': False, 'binding': character['name']}
                                break

                if not response:
                    response = {'success': False, 'value': negated, 'binding': target_canonical}
            
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
            logger.info(f'🧭 Proximity query for {target}: {response["value"]}')
            
        except Exception as e:
            logger.error(f'Error handling proximity query: {e}')
            traceback.print_exc()
            error_response = {'success': False,'value': False,'binding': None}
            query.reply(query.key_expr, json.dumps(error_response).encode('utf-8'))
    
    def handle_visibility_query(self, query):
        """Handle visibility queries for condition evaluation."""
        try:
            # Parse query parameters
            selector = str(query.selector)
            target = None
            
            # Extract target from query
            if 'target=' in selector:
                try:
                    target = urllib.parse.unquote(selector.split('target=')[1].split('&')[0])
                except:
                    pass
            
            if not target:
                response = {'success': False, 'value': False, 'binding': None}
            else:
                if 'negated=' in selector:
                    negated = urllib.parse.unquote(selector.split('negated=')[1].split('&')[0])
                    negated = negated.lower() == 'true'
                else:
                    negated = False
                # Check if target is in current situation and distance < near_threshold
                target_canonical = target.capitalize()
                response = None

                # Check if target is in visible terrains
                for view in self.situation['views']:
                    if view.get('terrain', '').capitalize() == target_canonical:
                        response = {'success': True, 'value': not negated, 'binding': view['direction']}
                        break
                # Check resources
                for resource in self.situation.get('resources', []):
                    # situation resources are strings, not dicts
                    if resource.capitalize() == target_canonical:
                        response = {'success': True, 'value': not negated, 'binding': resource}
                        break
                # Paths currently only appear per-view; no top-level list to scan here
                for character in self.situation.get('characters', []):
                    # situation characters are strings, not dicts
                        if character == target_canonical:
                            response = {'success': True, 'value': not negated, 'binding': character}
                        break
                
                if not response:
                    response = {'success': True, 'value': negated, 'binding': target_canonical}
            
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
            logger.info(f'🧭 Visibility query for {target}: {response["value"]}')
            
        except Exception as e:
            logger.error(f'Error handling visibility query: {e}')
            traceback.print_exc()
            error_response = {'success': False,'value': False,'binding': None}
            query.reply(query.key_expr, json.dumps(error_response).encode('utf-8'))
    
    def handle_location_query(self, query):
        """Handle location queries for condition evaluation."""
        try:
            # Parse query parameters
            selector = str(query.selector)
            target = None
            
            # Extract target from query
            if 'target=' in selector:
                try:
                    import urllib.parse
                    target = urllib.parse.unquote(selector.split('target=')[1].split('&')[0])
                except:
                    pass
            
            if not target:
                response = { 'success': False,'value': False, 'binding': None}
            else:
                # Check if target is in current situation and distance < at_location_threshold
                if 'negated=' in selector:
                    negated = urllib.parse.unquote(selector.split('negated=')[1].split('&')[0])
                    negated = negated.lower() == 'true'
                else:
                    negated = False
                # Check if target is in current situation and distance < near_threshold
                target_canonical = target.capitalize()
                response = None
                # Check terrains
                for view in self.situation['views']:
                    if view.get('terrain', '').capitalize() == target_canonical:
                        response = {'success': True, 'value': not negated, 'binding': view['direction']}
                        break
                    # Check resources in this direction
                    if not response:
                        for resource in view.get('resources', []):
                            if resource['name'].capitalize() == target_canonical:
                                if ((not negated and resource.get('distance', float('inf')) <= self.at_location_threshold) 
                                     or (negated and resource.get('distance', float('inf')) > self.at_location_threshold)):
                                    response = {'success': True, 'value': True, 'binding': resource['name']}
                                else:
                                    response = {'success': True, 'value': False, 'binding': resource['name']}
                                break
                    # Check paths in this direction
                    if not response:
                        for path in view.get('paths', []):
                            if path.get('name', '').capitalize() == target_canonical:
                                if ((not negated and path.get('distance', float('inf')) <= self.at_location_threshold)
                                     or (negated and path.get('distance', float('inf')) > self.at_location_threshold)):
                                    response = {'success': True, 'value': True, 'binding': path['name']}
                                else:
                                    response = {'success': True, 'value': False, 'binding': path['name']}
                                break
                
                    # Check characters if not found in resources
                    if not response:
                        for character in view.get('characters', []):
                            if target_canonical == 'person' or ('name' in character and character['name'].capitalize() == target_canonical):
                                if ((not negated and character.get('distance', float('inf')) <= self.at_location_threshold) 
                                     or (negated and character.get('distance', float('inf')) > self.at_location_threshold)):
                                    response = {'success': True, 'value': True, 'binding': character['name']}
                                else:
                                    response = {'success': True, 'value': False, 'binding': character['name']}
                                break
                
                if not response:
                    response = {'success': False, 'value': False, 'binding': target_canonical}
            
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
            logger.info(f'🧭 Proximity query for {target}: {response["value"]}')
            
        except Exception as e:
            logger.error(f'Error handling location query: {e}')
            traceback.print_exc()
            error_response = {'success': False, 'value': False, 'binding': None}
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
    
    def save_callback(self, sample):
        """Handle save command from UI."""
        try:
            logger.info(f'💾 {self.character_name} Situation Node received save command')
            self.save_situation()
        except Exception as e:
            logger.error(f'Error in save callback: {e}')
    
    def shutdown_callback(self, sample):
        """Handle shutdown command from UI."""
        try:
            logger.warning(f'🔌 {self.character_name} Situation Node received shutdown command')
            self.shutdown_requested = True
        except Exception as e:
            logger.error(f'Error in shutdown callback: {e}')
    
    def shutdown(self):
        """Cleanup and shutdown."""
        try:
            if self._shutting_down:
                return
            self._shutting_down = True
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
    
    def parse_view_string(self, view_string: str) -> dict:
        """
        Parse a view string into a structured dictionary.
        
        Example input: "Southwest: visibility 40, terrain Forest, slope Downhill ; resources: Fallen_Branch22 distance 9; characters: Joe distance 1;"
        
        Returns:
            dict: {'direction': str, 'visibility': int, 'terrain': str, 'slope': str, 'resources': [{'name': str, 'distance': int}], 'characters': [{'name': str, 'distance': int}]}
        """
        result = {
            'direction': '',
            'visibility': 0,
            'terrain': '',
            'property': '',
            'slope': '',
            'resources': [],
            'characters': [],
            'paths': []
        }
        
        # Split direction from the rest
        parts = view_string.split(':', 1)
        if len(parts) < 2:
            return result
            
        result['direction'] = parts[0].strip()
        remaining = parts[1].strip()
        
        # Split by semicolons to get main section and optional sections
        sections = remaining.split(';')
        
        # Parse main section (visibility, terrain, slope)
        main_section = sections[0].strip()
        main_parts = [part.strip() for part in main_section.split(',')]
        
        for part in main_parts:
            if part.startswith('visibility'):
                if part.split()[1].isdigit():
                    result['visibility'] = int(part.split()[1])
                else:
                    result['visibility'] = 0
            elif part.startswith('terrain'):
                result['terrain'] = part.split()[1]
            elif part.startswith('property'):
                result['property'] = part.split()[1]
            elif part.startswith('slope'):
                result['slope'] = part.split()[1]
        
        # Parse optional sections (resources, characters, paths)
        for section in sections[1:]:
            section = section.strip()
            if not section:
                continue
                
            if section.startswith('resources:'):
                # Parse resources: "resources: Name1 distance X, Name2 distance Y"
                resources_part = section[10:].strip()  # Remove "resources: "
                if resources_part:
                    resource_items = [item.strip() for item in resources_part.split(',')]
                    for item in resource_items:
                        item_parts = item.split()
                        if len(item_parts) >= 3 and item_parts[-2] == 'distance':
                            resource_name = ' '.join(item_parts[:-2])
                            distance = int(item_parts[-1])
                            result['resources'].append({'name': resource_name, 'distance': distance})
            
            elif section.startswith('characters:'):
                # Parse characters: "characters: Name1 distance X, Name2 distance Y"
                characters_part = section[11:].strip()  # Remove "characters: "
                if characters_part:
                    character_items = [item.strip() for item in characters_part.split(',')]
                    for item in character_items:
                        item_parts = item.split()
                        if len(item_parts) >= 3 and item_parts[-2] == 'distance':
                            character_name = ' '.join(item_parts[:-2])
                            distance = int(item_parts[-1])
                            result['characters'].append({'name': character_name, 'distance': distance})
            
            elif section.startswith('roads:') or section.startswith('paths:'):
                # Parse infrastructure: "roads: PathXY distance X, PathUV distance Y"
                infra_part = section.split(':', 1)[1].strip()
                if infra_part:
                    infra_items = [item.strip() for item in infra_part.split(',')]
                    for item in infra_items:
                        item_parts = item.split()
                        if len(item_parts) >= 3 and item_parts[-2] == 'distance':
                            path_name = ' '.join(item_parts[:-2])
                            distance = int(item_parts[-1])
                            result['paths'].append({'name': path_name, 'distance': distance})
        
        return result


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