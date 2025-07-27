#!/usr/bin/env python3

import sys
import os
import argparse
import logging
import json
import importlib.util
import signal
import time
import threading
from pathlib import Path
from datetime import datetime
from typing import Dict, Any, Optional

# Add current directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import zenoh
from map import WorldMap, Agent, hash_direction_info, extract_direction_info

# Configure logging
# Console handler with WARNING level (less verbose)
console_handler = logging.StreamHandler()
console_handler.setLevel(logging.WARNING)

# File handler with INFO level (full logging)
file_handler = logging.FileHandler('map_node.log', mode='w')
file_handler.setLevel(logging.INFO)

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[console_handler, file_handler],
    force=True
)
logger = logging.getLogger(__name__)

class MapNode:
    def __init__(self, map_file: str, world_name: str = None):
        self.map_file = map_file
        self.world_name = world_name or map_file.replace('.py', '')
        self.world_map = None
        self.session = None
        self.shutdown_requested = False
        
        # Agent registry: character_name -> Agent instance
        self.agent_registry = {}
        
        # Agent visibility tracking: agent_name -> set of visible agent names
        self.agent_visibility = {}
        
        # Persistence setup
        self.world_file = Path(f"data/world/{self.world_name}_world.json")
        self.world_file.parent.mkdir(parents=True, exist_ok=True)
        
        # Persistence timer
        self.last_save_time = time.time()
        self.save_interval = 120  # 2 minutes
        self.persistence_thread = None
        
        # Turn management
        self.turn_state = {
            'turn_number': 0,
            'active_characters': [],
            'completed_characters': [],
            'turn_start_time': None,
            'timeout_seconds': 30
        }
        self.turn_publisher = None
        self.turn_complete_subscriber = None
        
        # Load the map module
        self.load_map_module()
        
        # Initialize Zenoh session
        self.init_zenoh()
        
        # Start persistence thread
        self.start_persistence_thread()
        
    def load_map_module(self):
        """Load the map module from the maps subdirectory"""
        try:
            maps_dir = os.path.join(os.path.dirname(__file__), 'maps')
            map_path = os.path.join(maps_dir, self.map_file)
            
            if not os.path.exists(map_path):
                raise FileNotFoundError(f"Map file not found: {map_path}")
            
            logger.info(f"Loading map module from: {map_path}")
            
            # Load the module dynamically
            spec = importlib.util.spec_from_file_location("map_module", map_path)
            map_module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(map_module)
            
            # Create WorldMap instance
            logger.info("Creating WorldMap instance...")
            self.world_map = WorldMap(map_module)
            logger.info(f"WorldMap created successfully: {self.world_map.width}x{self.world_map.height}")
            
            # Load existing world data if available
            self.load_world_data()
            
        except Exception as e:
            logger.error(f"Failed to load map module: {e}")
            raise
    
    def init_zenoh(self):
        """Initialize Zenoh session and set up queryables"""
        try:
            # Create Zenoh configuration
            conf = zenoh.Config()
            conf.insert_json5("listen/endpoints", '["tcp/localhost:7447"]')
            conf.insert_json5("connect/endpoints", '["tcp/localhost:7447"]')
            
            # Create Zenoh session
            logger.info("Creating Zenoh session...")
            self.session = zenoh.open(conf)
            
            # Set up queryables for map services
            self.setup_queryables()
            
            # Set up turn management
            self.setup_turn_management()
            
            logger.info("Zenoh session initialized successfully")
            
        except Exception as e:
            logger.error(f"Failed to initialize Zenoh: {e}")
            raise
    
    def setup_queryables(self):
        """Set up Zenoh queryables for map services"""
        
        # Map summary queryable
        self.map_summary_queryable = self.session.declare_queryable(
            "cognitive/map/summary",
            self.handle_map_summary
        )
        
        # Resource list queryable
        self.resources_queryable = self.session.declare_queryable(
            "cognitive/map/resources",
            self.handle_resources_query
        )
        
        # Resource by name queryable
        self.resource_by_name_queryable = self.session.declare_queryable(
            "cognitive/map/resource/*",
            self.handle_resource_by_name
        )
        
        # Terrain queryable
        self.terrain_queryable = self.session.declare_queryable(
            "cognitive/map/terrain",
            self.handle_terrain_query
        )
        
        # Random location by resource queryable
        self.random_location_resource_queryable = self.session.declare_queryable(
            "cognitive/map/random_location/resource/*",
            self.handle_random_location_by_resource
        )
        
        # Random location by terrain queryable
        self.random_location_terrain_queryable = self.session.declare_queryable(
            "cognitive/map/random_location/terrain/*",
            self.handle_random_location_by_terrain
        )
        
        # Agent registration queryable
        self.agent_register_queryable = self.session.declare_queryable(
            "cognitive/map/agent/register/*",
            self.handle_agent_register
        )
        
        # Agent look queryable
        self.agent_look_queryable = self.session.declare_queryable(
            "cognitive/map/agent/*/look",
            self.handle_agent_look
        )
        
        # Agent move queryable
        self.agent_move_queryable = self.session.declare_queryable(
            "cognitive/map/agent/*/move",
            self.handle_agent_move
        )
        
        # Subscriber for character announcements
        self.character_announcement_subscriber = self.session.declare_subscriber(
            "cognitive/*/action",
            self.handle_character_announcement
        )
        
        logger.info("Map queryables and subscribers set up successfully")
    
    def setup_turn_management(self):
        """Set up turn management system"""
        try:
            # Publisher for turn "GO" signals
            self.turn_publisher = self.session.declare_publisher("cognitive/map/turn/go")
            
            # Publisher for step complete signals (for FastAPI UI)
            self.step_complete_publisher = self.session.declare_publisher("cognitive/map/step_complete")
            
            # Subscriber for turn completion signals
            self.turn_complete_subscriber = self.session.declare_subscriber(
                "cognitive/map/turn/complete/*",
                self.handle_turn_complete
            )
            
            # Queryable for turn status
            self.turn_status_queryable = self.session.declare_queryable(
                "cognitive/map/turn/status",
                self.handle_turn_status_query
            )
            
            # Subscribers for manual turn control
            self.turn_step_subscriber = self.session.declare_subscriber(
                "cognitive/map/turn/step",
                self.handle_turn_step
            )
            
            self.turn_run_subscriber = self.session.declare_subscriber(
                "cognitive/map/turn/run",
                self.handle_turn_run
            )
            
            self.turn_stop_subscriber = self.session.declare_subscriber(
                "cognitive/map/turn/stop",
                self.handle_turn_stop
            )
            
            # Turn control state
            self.turn_control_mode = "step"  # "step" or "run"
            self.auto_progression_enabled = False
            
            logger.info("Turn management system set up successfully")
            logger.info("Manual turn control enabled - starting in STEP mode")
            
        except Exception as e:
            logger.error(f"Failed to set up turn management: {e}")
            raise
    
    def get_active_characters(self):
        """Get list of active characters from agent registry"""
        return list(self.agent_registry.keys())
    
    def start_new_turn(self):
        """Start a new turn for all active characters"""
        import random
        
        # Get active characters and randomize order
        active_characters = self.get_active_characters()
        logger.info(f"Starting new turn with active characters: {active_characters}")
        if not active_characters:
            logger.debug("No active characters for turn")
            return
        
        random.shuffle(active_characters)
        
        # Update turn state
        self.turn_state['turn_number'] += 1
        self.turn_state['active_characters'] = active_characters
        self.turn_state['completed_characters'] = []
        self.turn_state['turn_start_time'] = time.time()
        
        # Publish "GO" signal
        go_signal = {
            'turn_number': self.turn_state['turn_number'],
            'active_characters': active_characters,
            'timestamp': time.time()
        }
        self.turn_publisher.put(json.dumps(go_signal).encode('utf-8'))
        
        logger.info(f"🚦 Turn {self.turn_state['turn_number']} started for: {', '.join(active_characters)}")
    
    def handle_turn_complete(self, sample):
        """Handle turn completion signals from characters"""
        try:
            # Extract character name from topic
            topic_parts = str(sample.key_expr).split('/')
            character_name = topic_parts[-1] if len(topic_parts) > 0 else None
            
            if not character_name:
                logger.warning("Turn complete signal without character name")
                return
            
            # Add to completed list
            if character_name not in self.turn_state['completed_characters']:
                self.turn_state['completed_characters'].append(character_name)
                logger.info(f"✅ {character_name} completed turn {self.turn_state['turn_number']}")
                logger.info(f"Turn progress: {len(self.turn_state['completed_characters'])}/{len(self.turn_state['active_characters'])} characters completed")
            
            # Check if all characters have completed
            if len(self.turn_state['completed_characters']) >= len(self.turn_state['active_characters']):
                logger.info(f"🎯 All characters completed turn {self.turn_state['turn_number']}")
                
                # In Step mode, never auto-progress - always wait for manual step
                if self.turn_control_mode == "step":
                    logger.info("Step mode - waiting for manual step command")
                    # Publish step complete message for FastAPI UI
                    step_complete_data = {
                        'turn_number': self.turn_state['turn_number'],
                        'active_characters': self.turn_state['active_characters'],
                        'completed_characters': self.turn_state['completed_characters'],
                        'timestamp': time.time()
                    }
                    self.step_complete_publisher.put(json.dumps(step_complete_data).encode('utf-8'))
                    logger.info("📢 Published step_complete message for FastAPI UI")
                    # Reset turn state so next step command starts a new turn
                    self.turn_state['active_characters'] = []
                    self.turn_state['completed_characters'] = []
                    self.turn_state['turn_start_time'] = None
                # In Run mode, auto-progress to next turn
                elif self.auto_progression_enabled:
                    logger.info("Run mode - auto-progressing to next turn")
                    threading.Timer(1.0, self.start_new_turn).start()
                else:
                    logger.info("Run mode but auto-progression disabled - waiting for manual step")
            
        except Exception as e:
            logger.error(f"Error handling turn complete: {e}")
    
    def handle_turn_status_query(self, query):
        """Handle turn status queries"""
        try:
            response = {
                'success': True,
                'turn_state': self.turn_state,
                'turn_control': {
                    'mode': self.turn_control_mode,
                    'auto_progression': self.auto_progression_enabled
                }
            }
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
        except Exception as e:
            logger.error(f"Error handling turn status query: {e}")
            error_response = {
                'success': False,
                'error': str(e)
            }
            query.reply(query.key_expr, json.dumps(error_response).encode('utf-8'))
    
    def check_turn_timeout(self):
        """Check for turn timeouts and advance turn if needed"""
        if not self.turn_state['turn_start_time']:
            return
        
        elapsed = time.time() - self.turn_state['turn_start_time']
        if elapsed > self.turn_state['timeout_seconds']:
            # In step mode, don't timeout - just log the elapsed time
            if not self.auto_progression_enabled:
                logger.info(f"⏰ Turn {self.turn_state['turn_number']} elapsed time: {elapsed:.1f}s (no timeout in step mode)")
                return
            
            # Timeout occurred - remove unresponsive characters (only in run mode)
            completed = set(self.turn_state['completed_characters'])
            active = set(self.turn_state['active_characters'])
            unresponsive = active - completed
            
            if unresponsive:
                logger.warning(f"⏰ Turn timeout - removing unresponsive characters: {', '.join(unresponsive)}")
                # Remove unresponsive characters from agent registry
                for char in unresponsive:
                    if char in self.agent_registry:
                        del self.agent_registry[char]
                        logger.info(f"Removed unresponsive character: {char}")
            
            # Start next turn only if auto-progression is enabled
            if self.auto_progression_enabled:
                self.start_new_turn()
            else:
                logger.info("Turn timed out but auto-progression disabled - waiting for manual step")
    
    def handle_turn_step(self, sample):
        """Handle manual step turn command"""
        try:
            logger.info("🎯 Manual Step Turn command received")
            self.turn_control_mode = "step"
            self.auto_progression_enabled = False
            
            # If no turn is currently active, start one
            if not self.turn_state['active_characters']:
                logger.info("Starting new turn for step command")
                self.start_new_turn()
            else:
                logger.info("Turn already in progress - will complete normally")
                
        except Exception as e:
            logger.error(f"Error handling turn step command: {e}")
    
    def handle_turn_run(self, sample):
        """Handle manual run turns command"""
        try:
            logger.info("🏃 Manual Run Turns command received")
            self.turn_control_mode = "run"
            self.auto_progression_enabled = True
            
            # If no turn is currently active, start one
            if not self.turn_state['active_characters']:
                self.start_new_turn()
            else:
                logger.info("Turn already in progress - will auto-progress after completion")
                
        except Exception as e:
            logger.error(f"Error handling turn run command: {e}")
    
    def handle_turn_stop(self, sample):
        """Handle manual stop turns command"""
        try:
            logger.info("⏹️ Manual Stop Turns command received")
            self.turn_control_mode = "step"
            self.auto_progression_enabled = False
            
            logger.info("Auto-progression disabled - waiting for manual step")
                
        except Exception as e:
            logger.error(f"Error handling turn stop command: {e}")
    
    def handle_map_summary(self, query):
        """Handle map summary queries"""
        try:
            summary = self.world_map.get_map_summary()
            response = {
                'success': True,
                'map_file': self.map_file,
                'summary': summary
            }
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
        except Exception as e:
            logger.error(f"Error handling map summary query: {e}")
            error_response = {
                'success': False,
                'error': str(e)
            }
            query.reply(query.key_expr, json.dumps(error_response).encode('utf-8'))
    
    def handle_resources_query(self, query):
        """Handle resources list queries"""
        try:
            resources = self.world_map.get_resource_list()
            response = {
                'success': True,
                'resources': resources
            }
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
        except Exception as e:
            logger.error(f"Error handling resources query: {e}")
            error_response = {
                'success': False,
                'error': str(e)
            }
            query.reply(query.key_expr, json.dumps(error_response).encode('utf-8'))
    
    def handle_resource_by_name(self, query):
        """Handle resource by name queries"""
        try:
            # Extract resource name from query key
            key_parts = str(query.key_expr).split('/')
            resource_name = key_parts[-1] if len(key_parts) > 0 else None
            
            if not resource_name:
                raise ValueError("No resource name provided")
            
            resource = self.world_map.get_resource_by_name(resource_name)
            if resource:
                response = {
                    'success': True,
                    'resource': resource
                }
            else:
                response = {
                    'success': False,
                    'error': f"Resource '{resource_name}' not found"
                }
            
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
        except Exception as e:
            logger.error(f"Error handling resource by name query: {e}")
            error_response = {
                'success': False,
                'error': str(e)
            }
            query.reply(query.key_expr, json.dumps(error_response).encode('utf-8'))
    
    def handle_terrain_query(self, query):
        """Handle terrain queries"""
        try:
            # Get terrain information
            terrain_info = {
                'width': self.world_map.width,
                'height': self.world_map.height,
                'terrain_types': [t.name for t in self.world_map.terrain_types]
            }
            
            response = {
                'success': True,
                'terrain': terrain_info
            }
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
        except Exception as e:
            logger.error(f"Error handling terrain query: {e}")
            error_response = {
                'success': False,
                'error': str(e)
            }
            query.reply(query.key_expr, json.dumps(error_response).encode('utf-8'))
    
    def handle_random_location_by_resource(self, query):
        """Handle random location by resource queries"""
        try:
            # Extract resource name from query key
            key_parts = str(query.key_expr).split('/')
            resource_name = key_parts[-1] if len(key_parts) > 0 else None
            
            if not resource_name:
                raise ValueError("No resource name provided")
            
            location = self.world_map.random_location_by_resource(resource_name)
            if location:
                response = {
                    'success': True,
                    'resource': resource_name,
                    'location': location
                }
            else:
                response = {
                    'success': False,
                    'error': f"No location found for resource '{resource_name}'"
                }
            
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
        except Exception as e:
            logger.error(f"Error handling random location query: {e}")
            error_response = {
                'success': False,
                'error': str(e)
            }
            query.reply(query.key_expr, json.dumps(error_response).encode('utf-8'))
    
    def handle_random_location_by_terrain(self, query):
        """Handle random location by terrain queries"""
        try:
            # Extract terrain name from query key
            key_parts = str(query.key_expr).split('/')
            terrain_name = key_parts[-1] if len(key_parts) > 0 else None
            
            if not terrain_name:
                raise ValueError("No terrain name provided")
            
            location = self.world_map.random_location_by_terrain(terrain_name)
            if location:
                response = {
                    'success': True,
                    'terrain': terrain_name,
                    'location': location
                }
            else:
                response = {
                    'success': False,
                    'error': f"No location found for terrain '{terrain_name}'"
                }
            
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
        except Exception as e:
            logger.error(f"Error handling random terrain location query: {e}")
            error_response = {
                'success': False,
                'error': str(e)
            }
            query.reply(query.key_expr, json.dumps(error_response).encode('utf-8'))
    
    def handle_agent_register(self, query):
        """Handle agent registration"""
        try:
            # Extract character name from query key
            key_parts = str(query.key_expr).split('/')
            character_name = key_parts[-1] if len(key_parts) > 0 else None
            
            if not character_name:
                raise ValueError("No character name provided")
            
            # Check if agent already exists (case-insensitive)
            canonical_character_name = character_name.capitalize()
            if canonical_character_name in self.agent_registry:
                response = {
                    'success': False,
                    'error': f"Agent for character '{character_name}' already registered"
                }
            else:
                # Find a random valid location for the agent
                location = self.world_map.random_location_by_terrain("Clearing")
                if not location:
                    # Fallback to any valid location
                    location = (25, 25)  # Center of map
                
                # Create agent instance
                agent = Agent(location[0], location[1], self.world_map, canonical_character_name)
                
                # Register agent with world map
                self.world_map.register_agent(agent)
                
                # Store in our registry
                self.agent_registry[canonical_character_name] = agent
                
                # Initialize visibility tracking for this agent
                self.agent_visibility[canonical_character_name] = set()
                
                response = {
                    'success': True,
                    'character_name': canonical_character_name,
                    'location': location,
                    'message': f"Agent registered at {location}"
                }
                
                logger.info(f"Agent registered for character '{canonical_character_name}' at {location}")
            
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
            
        except Exception as e:
            logger.error(f"Error handling agent registration: {e}")
            error_response = {
                'success': False,
                'error': str(e)
            }
            query.reply(query.key_expr, json.dumps(error_response).encode('utf-8'))
    
    def handle_agent_look(self, query):
        """Handle agent look command"""
        try:
            # Extract character name from query key
            key_parts = str(query.key_expr).split('/')
            character_name = key_parts[-2] if len(key_parts) > 1 else None
            
            if not character_name:
                raise ValueError("No character name provided")
            
            # Get agent from registry (case-insensitive)
            canonical_character_name = character_name.capitalize()
            if canonical_character_name not in self.agent_registry:
                response = {
                    'success': False,
                    'error': f"Agent for character '{character_name}' not found"
                }
            else:
                agent = self.agent_registry[canonical_character_name]
                
                # Call agent's look method
                look_result = agent.look()
                view = {}
                for dir in ['Current','North', 'Northeast', 'East', 'Southeast', 
                        'South', 'Southwest', 'West', 'Northwest']:
                    dir_obs = extract_direction_info(self.world_map, look_result, dir)
                    view[dir] = dir_obs

                view_text, resources, characters, paths, percept_summary = hash_direction_info(view, world=self.world_map)
                
                response = {
                    'success': True,
                    'character_name': canonical_character_name,
                    'look_result': view_text,
                    'location': [agent.x, agent.y],
                    'characters': list(set(characters)),
                }
            
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
            
        except Exception as e:
            logger.error(f"Error handling agent look: {e}")
            error_response = {
                'success': False,
                'error': str(e)
            }
            query.reply(query.key_expr, json.dumps(error_response).encode('utf-8'))
    
    def handle_agent_move(self, query):
        """Handle agent move command"""
        try:
            # Extract character name from query key
            key_parts = str(query.key_expr).split('/')
            character_name = key_parts[-2] if len(key_parts) > 1 else None
            
            if not character_name:
                raise ValueError("No character name provided")
            
            # Parse direction from query payload
            direction = 'current'  # Default direction
            try:
                if query.payload:
                    payload = query.payload.to_bytes().decode('utf-8')
                    data = json.loads(payload) if payload else {}
                    direction = data.get('direction', 'current')
            except Exception as e:
                logger.warning(f"Could not parse move payload, using default direction: {e}")
            
            # Get agent from registry (case-insensitive)
            canonical_character_name = character_name.capitalize()
            if canonical_character_name not in self.agent_registry:
                response = {
                    'success': False,
                    'error': f"Agent for character '{character_name}' not found"
                }
            else:
                agent = self.agent_registry[canonical_character_name]
                
                # Call agent's move method
                move_result = agent.move(direction)
                
                # Check for visibility changes after movement
                self.check_visibility_changes(canonical_character_name)
                
                response = {
                    'success': True,
                    'character_name': canonical_character_name,
                    'direction': direction,
                    'move_result': move_result
                }
            
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
            
        except Exception as e:
            logger.error(f"Error handling agent move: {e}")
            error_response = {
                'success': False,
                'error': str(e)
            }
            query.reply(query.key_expr, json.dumps(error_response).encode('utf-8'))
    
    def check_visibility_changes(self, moved_agent_name):
        """Check if the moved agent is now visible to other agents"""
        try:
            if moved_agent_name not in self.agent_registry:
                return
            
            moved_agent = self.agent_registry[moved_agent_name]
            
            # Get current visibility from moved agent's perspective
            # This will reveal all agents that can see the moved agent
            current_visibility = set()
            
            # Check each other agent to see if they can see the moved agent
            for other_agent_name, other_agent in self.agent_registry.items():
                if other_agent_name == moved_agent_name:
                    continue
                
                # Check if other agent can see the moved agent
                # We'll use a simple distance-based check for now
                if self.can_agent_see_agent(other_agent, moved_agent):
                    current_visibility.add(other_agent_name)
            
            # Check for new detections
            previous_visibility = self.agent_visibility.get(moved_agent_name, set())
            new_detections = current_visibility - previous_visibility
            
            # Publish events for new detections
            for detecting_agent_name in new_detections:
                self.publish_agent_detected_event(detecting_agent_name, moved_agent_name)
            
            # Update visibility tracking
            self.agent_visibility[moved_agent_name] = current_visibility
            
            if new_detections:
                logger.info(f"Agent '{moved_agent_name}' is now visible to: {new_detections}")
                
        except Exception as e:
            logger.error(f"Error checking visibility changes for {moved_agent_name}: {e}")
    
    def can_agent_see_agent(self, observer_agent, target_agent):
        """Check if observer_agent can see target_agent"""
        try:
            # Get positions
            observer_pos = (observer_agent.x, observer_agent.y)
            target_pos = (target_agent.x, target_agent.y)
            
            # Calculate distance
            distance = self.calculate_distance(observer_pos, target_pos)
            
            # Simple visibility: within 10 units and line of sight
            if distance <= 10:
                # Check line of sight (simplified - just check if path is clear)
                return self.has_line_of_sight(observer_pos, target_pos)
            
            return False
            
        except Exception as e:
            logger.error(f"Error checking agent visibility: {e}")
            return False
    
    def calculate_distance(self, pos1, pos2):
        """Calculate Manhattan distance between two positions"""
        return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])
    
    def has_line_of_sight(self, pos1, pos2):
        """Check if there's a clear line of sight between two positions"""
        try:
            # Simple line of sight check - just make sure it's not blocked by water/mountains
            x1, y1 = pos1
            x2, y2 = pos2
            
            # Check points along the path
            steps = max(abs(x2 - x1), abs(y2 - y1))
            if steps == 0:
                return True
            
            for i in range(1, steps):
                t = i / steps
                x = int(x1 + t * (x2 - x1))
                y = int(y1 + t * (y2 - y1))
                
                # Check if this point is passable terrain
                if 0 <= x < self.world_map.width and 0 <= y < self.world_map.height:
                    patch = self.world_map.patches[x][y]
                    if patch.terrain_type and patch.terrain_type.name in ['Water', 'Mountain']:
                        return False
            
            return True
            
        except Exception as e:
            logger.error(f"Error checking line of sight: {e}")
            return True  # Default to visible if error
    
    def publish_agent_detected_event(self, observer_agent_name, detected_agent_name):
        """Publish agent detection event"""
        try:
            detected_agent = self.agent_registry[detected_agent_name]
            observer_agent = self.agent_registry[observer_agent_name]
            
            # Calculate distance
            distance = self.calculate_distance(
                (observer_agent.x, observer_agent.y),
                (detected_agent.x, detected_agent.y)
            )
            
            event_data = {
                'agent_name': detected_agent_name,
                'position': (detected_agent.x, detected_agent.y),
                'distance': distance,
                'timestamp': datetime.now().isoformat()
            }
            
            topic = f"cognitive/{observer_agent_name}/sense/visual/agent_detected"
            self.session.put(topic, json.dumps(event_data).encode('utf-8'))
            
            logger.info(f"Published visual event: {observer_agent_name} detected {detected_agent_name} at distance {distance}")
            
        except Exception as e:
            logger.error(f"Error publishing agent detected event: {e}")
    
    def handle_character_announcement(self, sample):
        """Handle character announcement actions to create agents"""
        try:
            data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            
            # Check if this is a character announcement
            if data.get('type') == 'announcement':
                character_name = data.get('character_name')
                
                if character_name:
                    # Check if agent already exists (case-insensitive)
                    canonical_character_name = character_name.capitalize()
                    if canonical_character_name not in self.agent_registry:
                        # Create agent at default spawn point (20,20)
                        agent = Agent(20, 20, self.world_map, canonical_character_name)
                        
                        # Register agent with world map
                        self.world_map.register_agent(agent)
                        
                        # Store in our registry
                        self.agent_registry[canonical_character_name] = agent
                        
                        # Initialize visibility tracking for this agent
                        self.agent_visibility[canonical_character_name] = set()
                        
                        logger.info(f"Agent created for character '{canonical_character_name}' at (20,20)")
                        
                        # Start first turn if this is the first character and auto-progression is enabled
                        if len(self.agent_registry) == 1:
                            logger.info("First character added - turn management ready")
                            if self.auto_progression_enabled:
                                logger.info("Auto-progression enabled - starting first turn")
                                threading.Timer(2.0, self.start_new_turn).start()
                            else:
                                logger.info("Manual control mode - waiting for step command")
                
        except Exception as e:
            logger.error(f"Error handling character announcement: {e}")
    
    def run(self):
        """Run the map node"""
        logger.info(f"Map node started with map file: {self.map_file}")
        
        try:
            # Keep the node running
            while not self.shutdown_requested:
                # Check for turn timeouts
                self.check_turn_timeout()
                time.sleep(1)
                
        except KeyboardInterrupt:
            logger.info("Received keyboard interrupt")
        except Exception as e:
            logger.error(f"Error in map node main loop: {e}")
        finally:
            self.shutdown()
    
    def load_world_data(self):
        """Load world data from file."""
        try:
            if self.world_file.exists():
                with open(self.world_file, 'r') as f:
                    world_data = json.load(f)
                    
                    # Restore agent positions
                    if 'agents' in world_data:
                        for agent_data in world_data['agents']:
                            character_name = agent_data['character_name']
                            x, y = agent_data['position']
                            
                            # Create agent at saved position
                            agent = Agent(x, y, self.world_map, character_name)
                            self.world_map.register_agent(agent)
                            self.agent_registry[character_name] = agent
                            self.agent_visibility[character_name] = set()
                            
                            logger.info(f"📂 Restored agent {character_name} at position ({x}, {y})")
                    
                    # Restore world map state if available
                    if 'world_map' in world_data:
                        # TODO: Restore world map modifications (resources, terrain changes, etc.)
                        logger.info("📂 World map state restored")
                    
                    logger.info(f"📂 Loaded world data for '{self.world_name}'")
                    
                    # Start turn management if agents were restored
                    if len(self.agent_registry) > 0:
                        logger.info(f"📂 {len(self.agent_registry)} agents restored - starting turn management")
                        # Give characters time to initialize before starting turns
                        threading.Timer(5.0, self.start_new_turn).start()
                        
                        # Trigger character announcements for restored agents
                        # This ensures the map node knows about them for turn management
                        for character_name in self.agent_registry.keys():
                            logger.info(f"📂 Triggering announcement for restored character: {character_name}")
            else:
                logger.info(f"📂 No existing world data for '{self.world_name}', starting fresh")
        except Exception as e:
            logger.error(f"Error loading world data: {e}")
    
    def save_world_data(self):
        """Save world data to file."""
        try:
            world_data = {
                'world_name': self.world_name,
                'map_file': self.map_file,
                'timestamp': datetime.now().isoformat(),
                'agents': []
            }
            
            # Save agent positions
            for character_name, agent in self.agent_registry.items():
                agent_data = {
                    'character_name': character_name,
                    'position': [agent.x, agent.y]
                }
                world_data['agents'].append(agent_data)
            
            # Save world map state
            # TODO: Add world map modifications (resources, terrain changes, etc.)
            world_data['world_map'] = {
                'width': self.world_map.width,
                'height': self.world_map.height
                # Add more world state as needed
            }
            
            with open(self.world_file, 'w') as f:
                json.dump(world_data, f, indent=2)
            
            logger.debug(f"💾 Saved world data for '{self.world_name}'")
        except Exception as e:
            logger.error(f"Error saving world data: {e}")
    
    def start_persistence_thread(self):
        """Start background thread for periodic persistence."""
        self.persistence_thread = threading.Thread(target=self._persistence_loop, daemon=True)
        self.persistence_thread.start()
        logger.info(f"🔄 Started persistence thread for '{self.world_name}'")
    
    def _persistence_loop(self):
        """Background thread for periodic persistence."""
        while not self.shutdown_requested:
            try:
                current_time = time.time()
                if current_time - self.last_save_time >= self.save_interval:
                    self.save_world_data()
                    self.last_save_time = current_time
                time.sleep(10)  # Check every 10 seconds
            except Exception as e:
                logger.error(f"Error in persistence loop: {e}")
                time.sleep(30)  # Wait longer on error
    
    def shutdown(self):
        """Shutdown the map node"""
        logger.info("Shutting down map node...")
        self.shutdown_requested = True
        
        # Save world data on shutdown BEFORE closing session
        try:
            self.save_world_data()
            logger.info("✅ World data saved successfully")
        except Exception as e:
            logger.error(f"Error saving world data: {e}")
        
        # Clean up agents
        for character_name, agent in self.agent_registry.items():
            try:
                self.world_map.unregister_agent(agent)
                logger.info(f"Unregistered agent for character: {character_name}")
            except Exception as e:
                logger.error(f"Error unregistering agent for {character_name}: {e}")
        
        # Clear visibility tracking
        self.agent_visibility.clear()
        
        # Close Zenoh session more carefully
        if self.session:
            try:
                # Wait longer for any pending operations to complete
                time.sleep(2.0)
                self.session.close()
                logger.info("Zenoh session closed")
            except Exception as e:
                logger.error(f"Error closing Zenoh session: {e}")
        
        logger.info("Map node shutdown complete")

def signal_handler(signum, frame):
    """Handle shutdown signals"""
    logger.info(f"Received signal {signum}")
    if hasattr(signal_handler, 'map_node'):
        signal_handler.map_node.shutdown()

def main():
    parser = argparse.ArgumentParser(description='Shared Map Node')
    parser.add_argument('-m', '--map-file', required=True, 
                       help='Map file name (e.g., forest.py)')
    parser.add_argument('-w', '--world-name', 
                       help='World name (defaults to map file name without .py)')
    
    args = parser.parse_args()
    
    try:
        # Set up signal handlers
        signal.signal(signal.SIGTERM, signal_handler)
        signal.signal(signal.SIGINT, signal_handler)
        
        # Create and run map node
        map_node = MapNode(args.map_file, args.world_name)
        signal_handler.map_node = map_node  # Store reference for signal handler
        
        map_node.run()
        
    except Exception as e:
        logger.error(f"Failed to start map node: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main() 