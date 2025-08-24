#!/usr/bin/env python3

import asyncio
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
from datetime import datetime, timedelta
from typing import Dict, Any, Optional
from utils.zenoh_utils import datetime_handler

# Add current directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import zenoh
from map import WorldMap, Agent, hash_direction_info, extract_direction_info

# Configure logging
# Console handler with WARNING level (less verbose)
console_handler = logging.StreamHandler(sys.stdout)
console_handler.setLevel(logging.WARNING)

# Raise console verbosity when CWB_DEBUG is set
_debug_env = str(os.getenv('CWB_DEBUG', '')).lower() in ('1', 'true', 'yes', 'on')
if _debug_env:
    console_handler.setLevel(logging.INFO)

# File handler with INFO level (full logging)
file_handler = logging.FileHandler('logs/map_node.log', mode='w')
file_handler.setLevel(logging.INFO)

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[console_handler, file_handler],
    force=True
)
logger = logging.getLogger('map_node')
if _debug_env:
    logger.info('🔧 Debug mode enabled for MapNode (console INFO)')

# LLM client import
try:
    from llm_client import ZenohLLMClient
    LLM_CLIENT_AVAILABLE = True
except ImportError as e:
    print(f"⚠️  LLM Client not available: {e}")
    LLM_CLIENT_AVAILABLE = False


class MapNode:
    def __init__(self, map_file: str, world_name: str = None, setting: str = None):
        self.map_file = map_file
        self.world_name = world_name or map_file.replace('.py', '')
        self.world_map = None
        self.session = None
        self.shutdown_requested = False
        self._shutting_down = False
        self.setting = setting
        # Agent registry: character_name -> Agent instance
        self.agent_registry = {}
        
        # Agent visibility tracking: agent_name -> set of visible agent names
        self.agent_visibility = {}
        
        # Conversation lock management
        self.conversation_locks = {}  # character_name -> set of locked_with_character_names
        self.lock_request_counts = {}  # (requester, target) -> count of failed attempts
        self.lock_timeout_threshold = 3  # Number of failed attempts before timeout
        
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
        # Debug mode: allow launcher to disable turn timeouts via env var
        self.debug_disable_timeout = str(os.getenv('CWB_DEBUG', '')).lower() in ('1', 'true', 'yes', 'on')
        self.turn_publisher = None
        self.turn_complete_subscriber = None
        
        # Time advancement system
        self.turn_delay_minutes = 0  # Default delay between turns in run mode
        self.time_proposals = []  # List of (character_name, proposed_minutes) tuples
        self.time_proposal_subscriber = None
        self.time_delay_setting_subscriber = None
        self.time_advanced_publisher = None
        
        # Load the map module
        self.load_map_module()
        
        # Initialize Zenoh session
        self.init_zenoh()
        
        # Start persistence thread
        self.start_persistence_thread()
        self.llm_client = None
        if LLM_CLIENT_AVAILABLE:
            self.llm_client = ZenohLLMClient(service_timeout=60.0)
        
        self.initialize_setting()
        
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
    
    def initialize_setting(self):
        """Initialize the setting"""
        if self.setting:
            prompt = f"""Given the following setting, create a datetime in isoformat consistent with the setting, analyzing both the era, the season, and the implied time of day.
            
#Setting: \n{self.setting}
##

Response ONLY with the datetime in isoformat.
Do not include any other text, reasoning, introductory, expository, or markdown.
            """
            response = self.llm_client.generate([prompt], max_tokens=100)
            try:
                self.world_map.datetime = datetime.fromisoformat(response.text.strip())
                logger.info(f"Setting datetime to: {self.world_map.datetime}")
            except Exception as e:
                logger.error(f"Failed to set datetime: {e}")
                logger.error(f"Response: {response.text}")
                # Use a reasonable default simulation time instead of real-world time
                default_time = datetime(2024, 6, 15, 12, 0, 0)  # Midday, summer
                self.world_map.datetime = default_time
                logger.info(f"Setting datetime to default simulation time: {self.world_map.datetime}")
            return self.world_map.datetime
    
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
        
        # Resource removal queryable
        self.resource_remove_queryable = self.session.declare_queryable(
            "cognitive/map/resource/remove/*",
            self.handle_resource_remove
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
        
        # Map types queryable
        self.map_types_queryable = self.session.declare_queryable(
            "cognitive/map/types",
            self.handle_map_types
        )
        
        # Simulation time queryable
        self.simulation_time_queryable = self.session.declare_queryable(
            "cognitive/map/simulation_time",
            self.handle_simulation_time
        )
        
        # Conversation lock queryables
        self.conversation_lock_acquire_queryable = self.session.declare_queryable(
            "cognitive/map/conversation/lock/acquire",
            self.handle_conversation_lock_acquire
        )
        
        self.conversation_lock_release_queryable = self.session.declare_queryable(
            "cognitive/map/conversation/lock/release",
            self.handle_conversation_lock_release
        )
        
        self.conversation_lock_status_queryable = self.session.declare_queryable(
            "cognitive/map/conversation/lock/status/*",
            self.handle_conversation_lock_status
        )
        
        # Subscriber for character announcements
        self.character_announcement_subscriber = self.session.declare_subscriber(
            "cognitive/*/action",
            self.handle_character_announcement
        )
        
        # Subscriber for save commands (global)
        self.save_subscriber = self.session.declare_subscriber(
            "cognitive/save_all",
            self.save_callback
        )
        
        # Subscriber for shutdown commands (global)
        self.shutdown_subscriber = self.session.declare_subscriber(
            "cognitive/shutdown/shared",
            self.shutdown_callback
        )
        
        # Subscriber for character shutdown events (to cleanup locks)
        self.character_shutdown_subscriber = self.session.declare_subscriber(
            "cognitive/*/shutdown",
            self.handle_character_shutdown
        )
        
        logger.info("Map queryables and subscribers set up successfully")
    
    def setup_turn_management(self):
        """Set up turn management system"""
        try:
            # Publisher for turn "GO" signals
            self.turn_publisher = self.session.declare_publisher("cognitive/map/turn/go")
            
            # Publisher for step complete signals (for FastAPI UI)
            self.step_complete_publisher = self.session.declare_publisher("cognitive/map/step_complete")
            
            # Publisher for turn control status updates (for FastAPI UI)
            self.turn_control_publisher = self.session.declare_publisher("cognitive/map/turn_status")
            
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
            
            # Time advancement system
            self.time_proposal_subscriber = self.session.declare_subscriber(
                "cognitive/map/time_proposal",
                self.handle_time_proposal
            )
            
            self.time_delay_setting_subscriber = self.session.declare_subscriber(
                "cognitive/map/time_delay_setting",
                self.handle_time_delay_setting
            )
            
            self.time_advanced_publisher = self.session.declare_publisher(
                "cognitive/map/time_advanced"
            )
            
            # Turn control state
            self.turn_control_mode = "step"  # "step" or "run"
            self.auto_progression_enabled = False
            if self.debug_disable_timeout:
                logger.info("Debug mode detected (CWB_DEBUG). Turn timeout will be disabled.")
            
            logger.info("Turn management system set up successfully")
            logger.info("Manual turn control enabled - starting in STEP mode")
            logger.info("Time advancement system initialized")
            
            # Publish initial turn control status
            self._publish_turn_control_status()
            
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

    def start_new_turn_if_auto(self):
        """Start a new turn only if auto progression is still enabled."""
        try:
            if not self.auto_progression_enabled:
                logger.info("Auto start skipped: auto-progression disabled")
                return
            logger.info("Auto start invoked: starting next turn (auto-progression still enabled)")
            self.start_new_turn()
        except Exception as e:
            logger.error(f"Error in start_new_turn_if_auto: {e}")
    
    def delayed_turn_start_threaded(self, delay_minutes):
        """Start a new turn after the specified delay, checking for changes every 10 seconds."""
        try:
            total_delay_seconds = delay_minutes * 60
            elapsed_seconds = 0
            
            while elapsed_seconds < total_delay_seconds:
                # Sleep in 10-second increments for responsiveness
                time.sleep(10)
                elapsed_seconds += 10
                
                # Check if delay setting changed
                if self.turn_delay_minutes != delay_minutes:
                    logger.info(f"⏰ Turn delay changed from {delay_minutes} to {self.turn_delay_minutes} minutes - recalculating")
                    # Recalculate remaining time with new delay
                    remaining_seconds = max(0, (self.turn_delay_minutes * 60) - elapsed_seconds)
                    if remaining_seconds > 0:
                        logger.info(f"⏰ Continuing with {remaining_seconds} seconds remaining")
                        # Continue with remaining time
                        while elapsed_seconds < (delay_minutes * 60) + remaining_seconds:
                            time.sleep(10)
                            elapsed_seconds += 10
                    break
            
            # Only start turn if auto-progression is still enabled
            if self.auto_progression_enabled:
                logger.info("⏰ Delay completed - starting next turn")
                self.start_new_turn_if_auto()
            else:
                logger.info("⏰ Delay completed but auto-progression disabled - not starting turn")
                
        except Exception as e:
            logger.error(f"Error in delayed turn start: {e}")
    
    def handle_turn_complete(self, sample):
        """Handle turn completion signals from characters"""
        try:
            # Extract character name from topic
            topic_parts = str(sample.key_expr).split('/')
            character_name = topic_parts[-1] if len(topic_parts) > 0 else None
            
            if not character_name:
                logger.error("Turn complete signal without character name")
                return
            
            # Add to completed list
            if character_name not in self.turn_state['completed_characters']:
                self.turn_state['completed_characters'].append(character_name)
                logger.info(f"✅ {character_name} completed turn {self.turn_state['turn_number']}")
                logger.info(f"Turn progress: {len(self.turn_state['completed_characters'])}/{len(self.turn_state['active_characters'])} characters completed")
            
            # Check if all characters have completed
            if len(self.turn_state['completed_characters']) >= len(self.turn_state['active_characters']):
                logger.info(f"🎯 All characters completed turn {self.turn_state['turn_number']}")
                
                # Advance simulation time using hybrid logic
                self.advance_simulation_time()
                
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
                    if self.turn_delay_minutes > 0:
                        logger.info(f"Scheduling auto start of next turn with {self.turn_delay_minutes} minute delay")
                        # Use threading with periodic checks for responsiveness
                        threading.Timer(1.0, self.delayed_turn_start_threaded, args=[self.turn_delay_minutes]).start()
                    else:
                        logger.info("No delay - starting next turn immediately")
                        self.start_new_turn_if_auto()
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
        
        # In debug mode, disable timeout behavior entirely
        if self.debug_disable_timeout:
            try:
                elapsed = time.time() - self.turn_state['turn_start_time']
                #logger.info(f"⏰ Turn {self.turn_state['turn_number']} elapsed time: {elapsed:.1f}s (debug mode - timeout disabled)")
            except Exception:
                pass
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
                logger.info("Timeout path: auto-progression enabled - starting next turn")
                self.start_new_turn_if_auto()
            else:
                logger.info("Turn timed out but auto-progression disabled - waiting for manual step")
    
    def handle_turn_step(self, sample):
        """Handle manual step turn command"""
        try:
            logger.info("🎯 Manual Step Turn command received")
            self.turn_control_mode = "step"
            self.auto_progression_enabled = False
            
            # Publish turn control status update
            self._publish_turn_control_status()
            
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
            
            # Publish turn control status update
            self._publish_turn_control_status()
            
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
            
            # Publish turn control status update
            self._publish_turn_control_status()
            
            logger.info("Auto-progression disabled - waiting for manual step")
                
        except Exception as e:
            logger.error(f"Error handling turn stop command: {e}")
    
    def _publish_turn_control_status(self):
        """Publish current turn control status for UI updates"""
        try:
            turn_control_data = {
                'mode': self.turn_control_mode,
                'auto_progression': self.auto_progression_enabled,
                'timestamp': time.time()
            }
            self.turn_control_publisher.put(json.dumps(turn_control_data).encode())
            logger.info(f"📤 Published turn control status: mode={self.turn_control_mode}, auto_progression={self.auto_progression_enabled}")
        except Exception as e:
            logger.error(f"Error publishing turn control status: {e}")
    
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
                # Create JSON-safe version of resource
                json_safe_resource = resource.copy()
                # Convert ResourceType object to string
                if 'type' in json_safe_resource:
                    json_safe_resource['type'] = str(json_safe_resource['type'])
                
                response = {
                    'success': True,
                    'resource': json_safe_resource
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
    
    def handle_resource_remove(self, query):
        """Handle resource removal queries"""
        try:
            # Extract resource name from query key
            key_parts = str(query.key_expr).split('/')
            resource_name = key_parts[-1] if len(key_parts) > 0 else None
            
            if not resource_name:
                raise ValueError("No resource name provided")
            
            # Get resource to remove by name
            resource = self.world_map.get_resource_by_name(resource_name)
            if not resource:
                response = {
                    'success': False,
                    'error': f"Resource '{resource_name}' not found"
                }
            else:
                # Remove the resource using the resource ID
                resource_id = resource['name']  # The 'name' field contains the resource_id
                success = self.world_map.remove_resource(resource_id)
                
                if success:
                    response = {
                        'success': True,
                        'message': f"Resource '{resource_name}' removed successfully"
                    }
                    logger.info(f"Resource '{resource_name}' removed from map")
                else:
                    response = {
                        'success': False,
                        'error': f"Failed to remove resource '{resource_name}'"
                    }
            
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
        except Exception as e:
            logger.error(f"Error handling resource removal query: {e}")
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
            logger.info(f'Handling agent look command for {query.key_expr}')
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

                view_text, resources, characters, paths, percept_summary = hash_direction_info(view, distance_threshold=16, world=self.world_map)
                
                response = {
                    'success': True,
                    'character_name': canonical_character_name,
                    'look_result': view_text,
                    'location': [agent.x, agent.y],
                    'characters': list(set(characters)),
                }
            
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
            logger.info(f'Agent look command handled for {query.key_expr}')
            
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
                logger.error(f"Could not parse move payload, using default direction: {e}")
            
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
    
    def handle_map_types(self, query):
        """Handle map types query - returns available terrain, infrastructure, property, and resource types"""
        try:
            # Get map types from the world map - extract enum member names
            terrain_types = []
            if self.world_map.terrain_types:
                if hasattr(self.world_map.terrain_types, '__members__'):
                    terrain_types = list(self.world_map.terrain_types.__members__.keys())
                else:
                    terrain_types = [t.name for t in self.world_map.terrain_types]
            
            infrastructure_types = []
            if self.world_map.infrastructure_types:
                if hasattr(self.world_map.infrastructure_types, '__members__'):
                    infrastructure_types = list(self.world_map.infrastructure_types.__members__.keys())
                else:
                    infrastructure_types = [t.name for t in self.world_map.infrastructure_types]
            
            property_types = []
            if self.world_map.property_types:
                if hasattr(self.world_map.property_types, '__members__'):
                    property_types = list(self.world_map.property_types.__members__.keys())
                else:
                    property_types = [t.name for t in self.world_map.property_types]
            
            resource_types = []
            if self.world_map.resource_types:
                if hasattr(self.world_map.resource_types, '__members__'):
                    resource_types = list(self.world_map.resource_types.__members__.keys())
                else:
                    resource_types = [t.name for t in self.world_map.resource_types]
            
            response = {
                'success': True,
                'terrain_types': terrain_types,
                'infrastructure_types': infrastructure_types,
                'property_types': property_types,
                'resource_types': resource_types
            }
            
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
            logger.info(f'🗺️ Map types query: returned {len(terrain_types)} terrain, {len(infrastructure_types)} infrastructure, {len(property_types)} property, {len(resource_types)} resource types')
            
        except Exception as e:
            logger.error(f'Error handling map types query: {e}')
            error_response = {
                'success': False,
                'error': str(e),
                'terrain_types': [],
                'infrastructure_types': [],
                'property_types': [],
                'resource_types': []
            }
            query.reply(query.key_expr, json.dumps(error_response).encode('utf-8'))
    
    def calculate_time_info(self, dt):
        """Calculate time_info structure from a datetime object"""
        # Simple period calculation based on hour
        hour = dt.hour
        if 6 <= hour < 8:
            period = "dawn"
        elif 8 <= hour < 12:
            period = "morning"
        elif 12 <= hour < 17:
            period = "afternoon"
        elif 17 <= hour < 19:
            period = "dusk"
        elif 19 <= hour < 22:
            period = "evening"
        else:
            period = "night"
        
        # Simple season calculation based on month
        month = dt.month
        if month in [12, 1, 2]:
            season = "winter"
        elif month in [3, 4, 5]:
            season = "spring"
        elif month in [6, 7, 8]:
            season = "summer"
        else:
            season = "autumn"
        
        return {
            'datetime': dt.isoformat(),
            'period': period,
            'season': season,
            'hour': hour,
            'month': month
        }

    def handle_simulation_time(self, query):
        """Handle simulation time query - returns current datetime, season, period, and weather"""
        try:
            # For now, return stub data with constants
            # TODO: Implement actual time progression and weather simulation
            if type(self.world_map.datetime) != datetime:
                # Use a reasonable default simulation time instead of real-world time
                default_time = datetime(2024, 6, 15, 12, 0, 0)  # Midday, summer
                self.world_map.datetime = default_time
            
            time_info = self.calculate_time_info(self.world_map.datetime)
            
            # Stub weather data
            weather = "sunny"  # TODO: Implement weather simulation
            
            response = {
                'success': True,
                'time_info': time_info,
                'weather': weather
            }
            
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
            logger.info(f'⏰ Simulation time query: returned {time_info["period"]} {time_info["season"]} day, {weather}')
            
        except Exception as e:
            logger.error(f'Error handling simulation time query: {e}')
            error_response = {
                'success': False,
                'error': str(e),
                'time_info': {},
                'weather': 'unknown'
            }
            query.reply(query.key_expr, json.dumps(error_response).encode('utf-8'))
    
    def handle_time_proposal(self, sample):
        """Handle time advancement proposals from characters"""
        try:
            data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            character_name = data.get('character_name')
            proposed_minutes = data.get('proposed_minutes')
            
            if not character_name or proposed_minutes is None:
                logger.error(f"Invalid time proposal: missing character_name or proposed_minutes")
                return
            
            # Validate proposal
            if proposed_minutes < 0:
                logger.error(f"Character {character_name} proposed negative time: {proposed_minutes} minutes")
                return
            
            # Cap at 24 hours (1440 minutes)
            if proposed_minutes > 1440:
                logger.error(f"Character {character_name} proposed time > 24 hours ({proposed_minutes} min), capping at 1440")
                proposed_minutes = 1440
            
            # Store proposal (replace any existing proposal from this character)
            self.time_proposals = [(name, minutes) for name, minutes in self.time_proposals if name != character_name]
            self.time_proposals.append((character_name, proposed_minutes))
            
            logger.info(f"⏰ Time proposal from {character_name}: {proposed_minutes} minutes")
            
        except Exception as e:
            logger.error(f"Error handling time proposal: {e}")
    
    def handle_time_delay_setting(self, sample):
        """Handle time delay setting changes from UI"""
        try:
            data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            delay_minutes = data.get('delay_minutes')
            
            if delay_minutes is None:
                logger.error("Invalid time delay setting: missing delay_minutes")
                return
            
            # Validate range (0 to 30 minutes)
            if delay_minutes < 0:
                delay_minutes = 0
            elif delay_minutes > 30:
                delay_minutes = 30
            
            self.turn_delay_minutes = delay_minutes
            logger.info(f"⏰ Turn delay setting updated to {delay_minutes} minutes")
            
        except Exception as e:
            logger.error(f"Error handling time delay setting: {e}")
    
    def advance_simulation_time(self):
        """Advance simulation time using time proposals only"""
        try:
            # Determine time advancement using time proposals only
            advance_minutes = 10  # Default fallback if no proposals
            
            if self.time_proposals:
                # Get minimum proposal
                proposal_minutes = [minutes for _, minutes in self.time_proposals]
                advance_minutes = min(proposal_minutes)
                
                logger.info(f"⏰ Time proposals: {self.time_proposals}, advancing: {advance_minutes}min")
            else:
                logger.info(f"⏰ No time proposals, using default: {advance_minutes}min")
            
            # Advance the simulation time
            if advance_minutes > 0:
                old_datetime = self.world_map.datetime
                new_datetime = old_datetime + timedelta(minutes=advance_minutes)
                self.world_map.datetime = new_datetime
                
                # Calculate time_info for old and new times
                old_time_info = self.calculate_time_info(old_datetime)
                new_time_info = self.calculate_time_info(new_datetime)
                
                # Publish time advancement notification
                weather = "sunny"  # TODO: Implement weather simulation
                advancement_data = {
                    'old_time_info': old_time_info,
                    'new_time_info': new_time_info,
                    'weather': weather,
                    'advance_minutes': advance_minutes
                }
                
                self.time_advanced_publisher.put(json.dumps(advancement_data).encode('utf-8'))
                
                logger.info(f"⏰ Time advanced by {advance_minutes} minutes: {old_time_info['datetime']} → {new_time_info['datetime']}")
                
                # Log period/season changes
                if old_time_info['period'] != new_time_info['period']:
                    logger.info(f"🌅 Period changed: {old_time_info['period']} → {new_time_info['period']}")
                if old_time_info['season'] != new_time_info['season']:
                    logger.info(f"🍂 Season changed: {old_time_info['season']} → {new_time_info['season']}")
            
            # Clear time proposals for next turn
            self.time_proposals.clear()
            
        except Exception as e:
            logger.error(f"Error advancing simulation time: {e}")
    
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
                        # Determine spawn location: default (20,20) or scenario-provided resource instance
                        spawn_x, spawn_y = 20, 20
                        character_config = data.get('character_config', {})
                        if isinstance(character_config, dict):
                            resource_location_name = character_config.get('location')
                            if resource_location_name:
                                resource = self.world_map.get_resource_by_name(resource_location_name)
                                if resource and isinstance(resource, dict) and 'location' in resource:
                                    try:
                                        rx, ry = resource['location']
                                        spawn_x, spawn_y = int(rx), int(ry)
                                    except Exception:
                                        logger.error(f"Invalid resource location for '{resource_location_name}' - defaulting to (20,20)")
                                else:
                                    logger.error(f"Resource instance '{resource_location_name}' not found for '{canonical_character_name}' - defaulting to (20,20)")

                        # Create agent at resolved spawn point
                        agent = Agent(spawn_x, spawn_y, self.world_map, canonical_character_name)
                        
                        # Register agent with world map
                        self.world_map.register_agent(agent)
                        
                        # Store in our registry
                        self.agent_registry[canonical_character_name] = agent
                        
                        # Initialize visibility tracking for this agent
                        self.agent_visibility[canonical_character_name] = set()
                        
                        logger.info(f"Agent created for character '{canonical_character_name}' at ({agent.x},{agent.y})")
                        
                        # Start first turn if this is the first character and auto-progression is enabled
                        if len(self.agent_registry) == 1:
                            logger.info("First character added - turn management ready")
                            if self.auto_progression_enabled:
                                logger.info("Auto-progression enabled - starting first turn")
                                logger.info("Scheduling auto start of first turn in 2.0s")
                                threading.Timer(2.0, self.start_new_turn_if_auto).start()
                            else:
                                logger.info("Manual control mode - waiting for step command")
                
        except Exception as e:
            logger.error(f"Error handling character announcement: {e}")
    
    def handle_conversation_lock_acquire(self, query):
        """Handle conversation lock acquisition request."""
        try:
            # Parse the query payload
            if not query.payload:
                response = {
                    'success': False,
                    'error': 'No payload provided'
                }
                query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
                return
            
            data = json.loads(query.payload.to_bytes().decode('utf-8'))
            requester = data.get('requester')
            target = data.get('target')
            
            if not requester or not target:
                response = {
                    'success': False,
                    'error': 'Missing requester or target'
                }
                query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
                return
            
            # Attempt to acquire the lock
            lock_acquired = self.acquire_conversation_lock(requester, target)
            
            response = {
                'success': lock_acquired,
                'requester': requester,
                'target': target,
                'lock_acquired': lock_acquired
            }
            
            if not lock_acquired:
                response['error'] = 'Lock unavailable'
            
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
            logger.info(f"Conversation lock acquire request: {requester} -> {target}, result: {lock_acquired}")
            
        except Exception as e:
            logger.error(f"Error handling conversation lock acquire request: {e}")
            error_response = {
                'success': False,
                'error': str(e)
            }
            query.reply(query.key_expr, json.dumps(error_response).encode('utf-8'))
    
    def handle_conversation_lock_release(self, query):
        """Handle conversation lock release request."""
        try:
            # Parse the query payload
            if not query.payload:
                response = {
                    'success': False,
                    'error': 'No payload provided'
                }
                query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
                return
            
            data = json.loads(query.payload.to_bytes().decode('utf-8'))
            character1 = data.get('character1')
            character2 = data.get('character2')
            
            if not character1 or not character2:
                response = {
                    'success': False,
                    'error': 'Missing character1 or character2'
                }
                query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
                return
            
            # Release the locks
            self.release_conversation_locks(character1, character2)
            
            response = {
                'success': True,
                'character1': character1,
                'character2': character2,
                'message': 'Locks released successfully'
            }
            
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
            logger.info(f"Conversation lock release request: {character1} <-> {character2}")
            
        except Exception as e:
            logger.error(f"Error handling conversation lock release request: {e}")
            error_response = {
                'success': False,
                'error': str(e)
            }
            query.reply(query.key_expr, json.dumps(error_response).encode('utf-8'))
    
    def handle_conversation_lock_status(self, query):
        """Handle conversation lock status query."""
        try:
            # Extract character name from query key
            key_parts = str(query.key_expr).split('/')
            character_name = key_parts[-1] if len(key_parts) > 0 else None
            
            if not character_name:
                response = {
                    'success': False,
                    'error': 'No character name provided'
                }
                query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
                return
            
            # Get the lock status
            locked_with = self.get_conversation_lock_status(character_name)
            
            response = {
                'success': True,
                'character': character_name,
                'locked_with': locked_with,
                'is_locked': locked_with is not None
            }
            
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
            logger.info(f"Conversation lock status query for {character_name}: locked with {locked_with}")
            
        except Exception as e:
            logger.error(f"Error handling conversation lock status query: {e}")
            error_response = {
                'success': False,
                'error': str(e)
            }
            query.reply(query.key_expr, json.dumps(error_response).encode('utf-8'))
    
    def handle_character_shutdown(self, sample):
        """Handle character shutdown events to cleanup conversation locks."""
        try:
            # Extract character name from the topic
            topic = str(sample.key_expr)
            character_name = topic.split('/')[1] if len(topic.split('/')) > 1 else None
            
            if not character_name:
                logger.error("Character shutdown event missing character name")
                return
            
            # Clean up any conversation locks for this character
            if character_name in self.conversation_locks:
                locked_with_set = self.conversation_locks[character_name]
                logger.info(f"🧹 Cleaning up conversation locks for shutting down character {character_name} (locked with {locked_with_set})")
                
                # Release locks for all partners
                for partner in list(locked_with_set):
                    self.release_conversation_locks(character_name, partner)
            
            # Also check if this character is locked with any other character
            for char, locked_with_set in list(self.conversation_locks.items()):
                if character_name in locked_with_set:
                    logger.info(f"🧹 Cleaning up conversation locks for shutting down character {character_name} (locked by {char})")
                    self.release_conversation_locks(char, character_name)
            
            # Clean up any request counts involving this character
            keys_to_remove = []
            for key in self.lock_request_counts:
                if character_name in key:
                    keys_to_remove.append(key)
            
            for key in keys_to_remove:
                del self.lock_request_counts[key]
            
            logger.info(f"🧹 Cleanup complete for shutting down character {character_name}")
            
        except Exception as e:
            logger.error(f"Error handling character shutdown for cleanup: {e}")
    
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
                    
                    # Restore simulation time if available
                    if 'simulation_time' in world_data:
                        try:
                            self.world_map.datetime = datetime.fromisoformat(world_data['simulation_time'])
                            logger.info(f"📂 Simulation time restored to: {self.world_map.datetime}")
                        except Exception as e:
                            logger.error(f"Failed to restore simulation time: {e}, using current time")
                            # Use a reasonable default simulation time instead of real-world time
                            default_time = datetime(2024, 6, 15, 12, 0, 0)  # Midday, summer
                            self.world_map.datetime = default_time
                    else:
                        logger.info("📂 No simulation time in saved data, using current time")
                        # Use a reasonable default simulation time instead of real-world time
                        default_time = datetime(2024, 6, 15, 12, 0, 0)  # Midday, summer
                        self.world_map.datetime = default_time
                    
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
                'simulation_time': self.world_map.datetime.isoformat(),
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
    
    def save_callback(self, sample):
        """Handle save command from UI."""
        try:
            logger.info(f'💾 Map Node received save command')
            self.save_world_data()
        except Exception as e:
            logger.error(f'Error in save callback: {e}')
    
    def shutdown_callback(self, sample):
        """Handle shutdown command."""
        try:
            logger.info("Received shutdown command")
            self.shutdown_requested = True
        except Exception as e:
            logger.error(f"Error handling shutdown command: {e}")

    def acquire_conversation_lock(self, requester: str, target: str) -> bool:
        """
        Attempt to acquire conversation locks for both characters.
        
        Args:
            requester: Name of character requesting the lock
            target: Name of character to talk to
            
        Returns:
            bool: True if locks acquired, False if not available
        """
        try:
            # Canonicalize character names
            requester_canon = requester.capitalize()
            target_canon = target.capitalize()
            
            # Check if this specific pair is already locked
            requester_locked_with = self.conversation_locks.get(requester_canon, set())
            target_locked_with = self.conversation_locks.get(target_canon, set())
            
            if (target_canon in requester_locked_with or 
                requester_canon in target_locked_with):
                
                # Increment failed attempt count
                lock_key = (requester_canon, target_canon)
                self.lock_request_counts[lock_key] = self.lock_request_counts.get(lock_key, 0) + 1
                
                # Check if we should timeout existing locks
                if self.lock_request_counts[lock_key] >= self.lock_timeout_threshold:
                    logger.info(f"🔓 Conversation lock timeout triggered for {requester_canon} -> {target_canon} after {self.lock_timeout_threshold} attempts")
                    self._timeout_conversation_locks(requester_canon, target_canon)
                    # Clear the request count since we're releasing locks
                    del self.lock_request_counts[lock_key]
                    # Now try to acquire locks again
                    return self._acquire_locks_internal(requester_canon, target_canon)
                
                logger.info(f"🔒 Conversation lock unavailable for {requester_canon} -> {target_canon} (attempt {self.lock_request_counts[lock_key]})")
                return False
            
            # Both characters are free, acquire locks
            return self._acquire_locks_internal(requester_canon, target_canon)
            
        except Exception as e:
            logger.error(f"Error acquiring conversation lock for {requester} -> {target}: {e}")
            return False
    
    def _acquire_locks_internal(self, requester: str, target: str) -> bool:
        """Internal method to actually acquire the locks."""
        try:
            # Initialize sets if they don't exist
            if requester not in self.conversation_locks:
                self.conversation_locks[requester] = set()
            if target not in self.conversation_locks:
                self.conversation_locks[target] = set()
            
            # Lock both characters to each other
            self.conversation_locks[requester].add(target)
            self.conversation_locks[target].add(requester)
            
            logger.info(f"🔒 Conversation lock acquired: {requester} <-> {target}")
            return True
            
        except Exception as e:
            logger.error(f"Error in internal lock acquisition for {requester} -> {target}: {e}")
            return False
    
    def release_conversation_locks(self, character1: str, character2: str):
        """
        Release conversation locks for both characters.
        
        Args:
            character1: First character in conversation
            character2: Second character in conversation
        """
        try:
            # Canonicalize character names
            char1_canon = character1.capitalize()
            char2_canon = character2.capitalize()
            
            # Release locks for both characters
            if char1_canon in self.conversation_locks:
                self.conversation_locks[char1_canon].discard(char2_canon)
                if not self.conversation_locks[char1_canon]:
                    del self.conversation_locks[char1_canon]
            if char2_canon in self.conversation_locks:
                self.conversation_locks[char2_canon].discard(char1_canon)
                if not self.conversation_locks[char2_canon]:
                    del self.conversation_locks[char2_canon]
            
            # Clear any request counts for this pair
            lock_keys = [(char1_canon, char2_canon), (char2_canon, char1_canon)]
            for key in lock_keys:
                if key in self.lock_request_counts:
                    del self.lock_request_counts[key]
            
            logger.info(f"🔓 Conversation locks released: {char1_canon} <-> {char2_canon}")
            
        except Exception as e:
            logger.error(f"Error releasing conversation locks for {character1} <-> {character2}: {e}")
    
    def _timeout_conversation_locks(self, requester: str, target: str):
        """
        Timeout and release existing conversation locks.
        
        Args:
            requester: Character requesting the lock (triggering timeout)
            target: Character they want to talk to
        """
        try:
            # Find which characters are currently locked
            locked_characters = []
            
            # Check if requester is locked
            if requester in self.conversation_locks:
                locked_characters.extend(list(self.conversation_locks[requester]))
            
            # Check if target is locked
            if target in self.conversation_locks:
                locked_characters.extend(list(self.conversation_locks[target]))
            
            # Release all locks for the involved characters
            if locked_characters:
                logger.info(f"⏰ Timing out conversation locks for: {', '.join(locked_characters)}")
                for char in locked_characters:
                    if char in self.conversation_locks:
                        self.conversation_locks[char].discard(requester)
                        self.conversation_locks[char].discard(target)
                        if not self.conversation_locks[char]:
                            del self.conversation_locks[char]
            
        except Exception as e:
            logger.error(f"Error timing out conversation locks: {e}")
    
    def get_conversation_lock_status(self, character: str) -> Optional[list]:
        """
        Get the conversation lock status for a character.
        
        Args:
            character: Name of the character
            
        Returns:
            Optional[list]: List of character names they're locked with, or None if not locked
        """
        try:
            canonical_name = character.capitalize()
            return list(self.conversation_locks.get(canonical_name, []))
        except Exception as e:
            logger.error(f"Error getting conversation lock status for {character}: {e}")
            return None

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
        if self._shutting_down:
            return
        self._shutting_down = True
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
                # Known issue: closing Zenoh session here triggers a PanicException in some versions.
                # Skip explicit close and rely on process exit to tear down resources.
                # self.session.close()
                logger.info("Skipping explicit Zenoh session close to avoid panic; relying on process exit")
            except Exception as e:
                logger.error(f"Error during Zenoh session cleanup: {e}")
        
        logger.info("Map node shutdown complete")

def signal_handler(signum, frame):
    """Handle shutdown signals"""
    logger.info(f"Received signal {signum}")
    if hasattr(signal_handler, 'map_node'):
        # Request shutdown; allow main loop to exit and perform cleanup once
        signal_handler.map_node.shutdown_requested = True

def main():
    parser = argparse.ArgumentParser(description='Shared Map Node')
    parser.add_argument('-m', '--map-file', required=True, 
                       help='Map file name (e.g., forest.py)')
    parser.add_argument('-w', '--world-name', 
                       help='World name (defaults to map file name without .py)')
    parser.add_argument('-s', '--setting', 
                       help='Setting text string (defaults to None)')
    
    args = parser.parse_args()
    
    try:
        # Set up signal handlers
        signal.signal(signal.SIGTERM, signal_handler)
        signal.signal(signal.SIGINT, signal_handler)
        
        # Create and run map node
        map_node = MapNode(args.map_file, args.world_name, setting=args.setting)
        signal_handler.map_node = map_node  # Store reference for signal handler
        
        map_node.run()
        
    except Exception as e:
        logger.error(f"Failed to start map node: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main() 