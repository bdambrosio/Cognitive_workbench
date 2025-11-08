from __future__ import annotations
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
import queue
import hashlib
from pathlib import Path
from datetime import datetime, timedelta
import traceback
from typing import Dict, Any, Optional
from templates import WORLD_STATE_UPDATE_TEMPLATE
from utils.zenoh_utils import datetime_handler

# Add current directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import zenoh
from world_map import WorldMap
from map_agent import MapAgent
from infospace_map import InfospaceMap

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
from llm_client import ZenohLLMClient

# Infospace resource manager
from infospace_resource_manager import InfospaceResourceManager


class MapNode:
    def __init__(self, map_file: str, world_name: str = None, setting: str = None, max_turns: int = None, server_name: str = 'openai', model_name: str = 'gpt-4.1'):
        self.map_file = map_file
        self.world_name = world_name or map_file.replace('.py', '')
        self.world_map = None
        self.session = None
        self.shutdown_requested = False
        self._shutting_down = False
        self.setting = setting
        self.server_name = server_name
        self.model_name = model_name
        # Agent registry: character_name -> Agent instance
        self.agent_registry = {}
        
        # Agent visibility tracking: agent_name -> set of visible agent names
        self.agent_visibility = {}
        
        # Conversation lock management
        self.conversation_locks = {}  # character_name -> set of locked_with_character_names
        self.lock_request_counts = {}  # (requester, target) -> count of failed attempts
        self.lock_timeout_threshold = 3  # Number of failed attempts before timeout
        
        # Infospace resource manager (initialized after world_map loads)
        self.resource_manager = None
        
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
            'timeout_seconds': 60
        }
        # Debug mode: allow launcher to disable turn timeouts via env var
        self.debug_disable_timeout = str(os.getenv('CWB_DEBUG', '')).lower() in ('1', 'true', 'yes', 'on')
        
        # Character initialization tracking
        self.expected_character_count = 0  # Set by launcher ready signal
        self.system_ready = False  # True when launcher signals ready
        self.turn_publisher = None
        self.turn_complete_subscriber = None
        
        # Time advancement system
        self.turn_delay_minutes = 0  # Default delay between turns in run mode
        self.time_proposals = []  # List of (character_name, proposed_minutes) tuples
        self.time_proposal_subscriber = None
        self.time_delay_setting_subscriber = None
        self.time_advanced_publisher = None
        
        # World state tracking and updates
        self.world_state = ''
        self.world_state_update_queue = queue.Queue(maxsize=10)
        self.world_state_worker = None
        
        # Optional scenario cap on turns
        try:
            self.max_turns = int(max_turns) if max_turns is not None else None
        except Exception:
            self.max_turns = None
        
        # Load the map module
        self.load_map_module()
        
        # Initialize Zenoh session
        self.init_zenoh()
        
        # Start persistence thread
        self.start_persistence_thread()
        self.llm_client = ZenohLLMClient(server_name=self.server_name, model_name=self.model_name, service_timeout=60.0)
        
        # Start world state worker thread
        self.start_world_state_worker()
        
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
            
            # Determine map class from module attribute (defaults to WorldMap for backward compatibility)
            MapClass = getattr(map_module, 'map_class', WorldMap)
            logger.info(f"Creating {MapClass.__name__} instance...")
            self.world_map = MapClass(map_module)
            logger.info(f"{MapClass.__name__} created successfully: {self.world_map.width}x{self.world_map.height}")
            
            # Initialize resource manager after world_map is ready
            self.resource_manager = InfospaceResourceManager(
                self.world_map, self.session, self.world_name, self.agent_registry
            )
            logger.info("Infospace resource manager initialized")
            
            # Load existing world data if available
            self.load_world_data()
            
            # Initialize system collections (must be after load_world_data)
            self.resource_manager.initialize_system_collections()
            
            # Create distinguished note_null system resource
            self.create_note_null()
            
        except Exception as e:
            logger.error(traceback.format_exc())
            logger.error(f"Failed to load map module: {e}")
            raise
    
    def create_note_null(self):
        """
        Create distinguished note_null system resource.
        
        This is a singleton Note instance representing "no result" / empty value.
        Created at map initialization and visible to all agents.
        """
        try:
            # Check if Note resource type exists (only for infospace maps)
            if not hasattr(self.world_map.scenario_module.resource_types, 'Note'):
                logger.info("Skipping note_null creation (not an infospace map)")
                return
            
            # Generate unique ID
            note_id = f"Note_null"
            
            # Create Note properties
            properties = {
                'content': None,
                'format': 'json',
                'created_at': datetime.now().isoformat(),
                'created_by': 'system',
                'source_skill': 'system',
                'source_value': '',
                'is_system': True
            }
            
            # Place at origin (0, 0)
            x, y = 0, 0
            
            # Register in map's resource registry
            resource_data = {
                'type': self.world_map.scenario_module.resource_types.Note,
                'location': (x, y),
                'properties': properties
            }
            
            self.world_map.resource_registry[note_id] = resource_data
            self.world_map.patches[x][y].resources[note_id] = resource_data
            
            logger.info(f"Created system resource: {note_id} at ({x}, {y})")
            
        except Exception as e:
            logger.error(f"Failed to create note_null: {e}")
            # Non-fatal - continue initialization
    
    def initialize_setting(self):
        """Initialize the setting"""
        if self.setting:
            prompt = f"""Given the following setting, create a datetime in isoformat consistent with the setting, analyzing both the era, the season, and the implied time of day.
            
#Setting: \n{self.setting}
##

Response with the datetime in isoformat.
Do not include any other text, reasoning, introductory, expository, or markdown.
end your response with: 
</end>
            """
            response = self.llm_client.generate([prompt], max_tokens=50, stops=['</end>'])
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
        
        # Resource rules by type queryable
        self.resource_rules_queryable = self.session.declare_queryable(
            "cognitive/map/resource_rules/*",
            self.handle_resource_rules_by_name
        )
        
        # Resource removal queryable
        self.resource_remove_queryable = self.session.declare_queryable(
            "cognitive/map/resource/remove/*",
            self.handle_resource_remove
        )

        # Resource place queryable
        self.resource_place_queryable = self.session.declare_queryable(
            "cognitive/map/resource/place/*",
            self.handle_resource_place
        )
        
        # Note creation queryable (dynamic resource creation)
        self.note_create_queryable = self.session.declare_queryable(
            "cognitive/map/note/create",
            self.handle_create_note
        )
        
        # Collection creation queryable (dynamic resource creation)
        self.collection_create_queryable = self.session.declare_queryable(
            "cognitive/map/collection/create",
            self.handle_create_collection
        )
        
        # Collection add queryable (mutate existing Collection)
        self.collection_add_queryable = self.session.declare_queryable(
            "cognitive/map/collection/add",
            self.handle_collection_add
        )
        
        # Collection persist queryable (mark Collection as persistent)
        self.collection_persist_queryable = self.session.declare_queryable(
            "cognitive/map/collection/persist",
            self.handle_collection_persist
        )
        
        # Resource viewer queryable (get Note/Collection content by ID)
        self.resource_viewer_queryable = self.session.declare_queryable(
            "cognitive/map/resource/view/*",
            self.handle_resource_viewer
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
        
        # Agent move-to-resource queryable
        self.agent_move_to_queryable = self.session.declare_queryable(
            "cognitive/map/agent/*/move_to",
            self.handle_agent_move_to
        )
        
        # Agent use_path queryable
        self.agent_use_path_queryable = self.session.declare_queryable(
            "cognitive/map/agent/*/use_path",
            self.handle_agent_use_path
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
        
        # Subscriber for launcher ready signal (to know expected character count)
        self.launcher_ready_subscriber = self.session.declare_subscriber(
            "cognitive/launcher/ready",
            self.handle_launcher_ready
        )
        
        # World state queryable
        self.world_state_queryable = self.session.declare_queryable(
            "cognitive/map/world_state",
            self.handle_world_state_query
        )
        
        # World state update subscriber
        self.world_state_update_subscriber = self.session.declare_subscriber(
            "cognitive/map/world_state/update",
            self.handle_world_state_update
        )
        
        # Infospace vector store subscribers (for semantic search/memory)
        self.index_request_subscriber = self.session.declare_subscriber(
            "map/*/index_request/*",
            self.handle_index_request
        )
        
        self.search_request_subscriber = self.session.declare_subscriber(
            "map/*/search_request/*",
            self.handle_search_request
        )
        
        # Infospace tool execution (placeholder for now, tools handled by map)
        self.tool_request_subscriber = self.session.declare_subscriber(
            "map/*/tool_request/*",
            self.handle_tool_request
        )
        
        # Infospace scan request (find resources by name/interface)
        self.scan_request_subscriber = self.session.declare_subscriber(
            "map/*/scan_request/*",
            self.handle_scan_request
        )
        
        # Infospace move request (navigate to resource)
        self.move_request_subscriber = self.session.declare_subscriber(
            "map/*/move_request/*",
            self.handle_move_request
        )
        
        logger.info("Map queryables and subscribers set up successfully")
    
    def setup_turn_management(self):
        """Set up turn management system"""
        try:
            # === ZENOH PUBLICATION ===
            # NAME: turn_go
            # TOPIC: cognitive/map/turn/go
            # DESCRIPTION: Signal to all characters to start their turn
            # PAYLOAD: {"turn_number": int, "active_characters": list[str], "timestamp": float}
            # TRIGGERS: (internal turn management - characters begin OODA loop)
            # ========================
            self.turn_publisher = self.session.declare_publisher("cognitive/map/turn/go")
            
            # === ZENOH PUBLICATION ===
            # NAME: turn_state_update
            # TOPIC: cognitive/map/turn_state_update
            # DESCRIPTION: Comprehensive turn state and UI button states
            # PAYLOAD: {"type": str, "turn": dict, "buttons": dict, "timestamp": float}
            # TRIGGERS: (UI updates)
            # ========================
            self.turn_state_update_publisher = self.session.declare_publisher("cognitive/map/turn_state_update")
            
            # Subscriber for turn completion signals
            self.turn_complete_subscriber = self.session.declare_subscriber(
                "cognitive/map/turn/complete/*",
                self.handle_turn_complete
            )
            
            # Subscriber for heartbeat signals (resets timeout)
            self.turn_heartbeat_subscriber = self.session.declare_subscriber(
                "cognitive/map/turn/heartbeat/*",
                self.handle_turn_heartbeat
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
            
            # === ZENOH PUBLICATION ===
            # NAME: time_advanced
            # TOPIC: cognitive/map/time_advanced
            # DESCRIPTION: Simulation time advanced (hybrid mode - max proposal or fixed increment)
            # PAYLOAD: {"old_time_info": dict, "new_time_info": dict, "minutes_advanced": int}
            # TRIGGERS: MorningBriefing (daily@morning), WeeklyPlanning (weekly@morning), EndOfDayReview (daily@night)
            # ========================
            self.time_advanced_publisher = self.session.declare_publisher(
                "cognitive/map/time_advanced"
            )
            
            # === ZENOH PUBLICATION ===
            # NAME: scenario_complete
            # TOPIC: cognitive/map/scenario_complete
            # DESCRIPTION: Scenario reached completion condition (max_turns reached)
            # PAYLOAD: {"reason": str, "max_turns": int, "turn_number": int}
            # TRIGGERS: PrepareStatusReport, ArchiveCompletedWork
            # ========================
            self.scenario_complete_publisher = self.session.declare_publisher(
                "cognitive/map/scenario_complete"
            )
            
            # Turn control state
            self.turn_control_mode = "step"  # "step" or "run"
            self.auto_progression_enabled = False
            if self.debug_disable_timeout:
                logger.info("Debug mode detected (CWB_DEBUG). Turn timeout will be disabled.")
            
            logger.info("Turn management system set up successfully")
            logger.info("Manual turn control enabled - starting in STEP mode")
            logger.info("Time advancement system initialized")
            
            # Publish initial turn state
            self._publish_turn_state_update()
            
        except Exception as e:
            logger.error(f"Failed to set up turn management: {e}")
            raise
    
    def get_active_characters(self):
        """Get list of active characters from agent registry"""
        return list(self.agent_registry.keys())
    
    def _compute_ui_button_states(self) -> dict:
        """
        Single source of truth for UI button states.
        
        This centralizes all button state logic in one place, making it:
        - Testable (can verify logic without UI)
        - Debuggable (one place to check state)
        - Consistent (no race conditions from distributed state)
        
        Returns:
            dict: Button states with enabled flags and human-readable reasons
        """
        turn_active = len(self.turn_state['active_characters']) > 0
        turn_complete = (len(self.turn_state['completed_characters']) >= 
                        len(self.turn_state['active_characters']) and 
                        len(self.turn_state['active_characters']) > 0)
        
        # Check if all expected characters have initialized
        all_characters_ready = (self.expected_character_count > 0 and 
                               len(self.agent_registry) >= self.expected_character_count)
        
        button_states = {
            'step': {
                'enabled': (self.turn_control_mode == 'step' and 
                           not turn_active and 
                           all_characters_ready),
                'reason': self._get_step_button_reason(turn_active, all_characters_ready)
            },
            'run': {
                'enabled': (self.turn_control_mode == 'step' and 
                           not turn_active and 
                           all_characters_ready),
                'reason': self._get_run_button_reason(turn_active, all_characters_ready)
            },
            'stop': {
                'enabled': self.turn_control_mode == 'run',
                'reason': 'In run mode' if self.turn_control_mode == 'run' else 'Not running'
            }
        }
        
        return button_states
    
    def _get_step_button_reason(self, turn_active: bool, all_characters_ready: bool) -> str:
        """Get human-readable reason for step button state"""
        if self.turn_control_mode == 'run':
            return 'Disabled while in run mode'
        elif not all_characters_ready:
            announced = len(self.agent_registry)
            expected = self.expected_character_count
            return f'Waiting for characters to initialize ({announced}/{expected} ready)'
        elif turn_active:
            return f'Turn {self.turn_state["turn_number"]} in progress ({len(self.turn_state["completed_characters"])}/{len(self.turn_state["active_characters"])} complete)'
        else:
            return 'Ready - click to advance turn'
    
    def _get_run_button_reason(self, turn_active: bool, all_characters_ready: bool) -> str:
        """Get human-readable reason for run button state"""
        if self.turn_control_mode == 'run':
            return 'Already in run mode'
        elif not all_characters_ready:
            announced = len(self.agent_registry)
            expected = self.expected_character_count
            return f'Waiting for characters to initialize ({announced}/{expected} ready)'
        elif turn_active:
            return 'Turn in progress - wait for completion'
        else:
            return 'Ready - click to run continuously'
    
    def start_new_turn(self):
        """Start a new turn for all active characters"""
        import random
        
        # Enforce max_turns if configured
        try:
            if self.max_turns is not None and self.turn_state['turn_number'] >= int(self.max_turns):
                logger.info(f"🛑 Reached max_turns={self.max_turns}; not starting a new turn")
                try:
                    notice = {"reason": "max_turns", "max_turns": self.max_turns, "turn_number": self.turn_state['turn_number']}
                    self.scenario_complete_publisher.put(json.dumps(notice).encode('utf-8'))
                except Exception:
                    pass
                # Freeze auto-progression
                self.auto_progression_enabled = False
                return
        except Exception:
            pass

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
    
    def handle_turn_heartbeat(self, sample):
        """Handle heartbeat signals from characters (resets timeout)"""
        try:
            # Extract character name from topic
            topic_parts = str(sample.key_expr).split('/')
            character_name = topic_parts[-1] if len(topic_parts) > 0 else None
            
            if not character_name:
                return
            
            # Reset turn start time if character is active and turn is in progress
            if (self.turn_state['turn_start_time'] and 
                character_name in self.turn_state['active_characters'] and
                character_name not in self.turn_state['completed_characters']):
                self.turn_state['turn_start_time'] = time.time()
                logger.debug(f"💓 Heartbeat from {character_name} - timeout reset")
            
        except Exception as e:
            logger.error(f"Error handling turn heartbeat: {e}")
    
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
                    # Reset turn state so next step command starts a new turn
                    self.turn_state['active_characters'] = []
                    self.turn_state['completed_characters'] = []
                    self.turn_state['turn_start_time'] = None
                    
                    # Publish unified turn state update (replaces old step_complete message)
                    self._publish_turn_state_update()
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
                logger.error(f"⏰ Turn timeout - removing unresponsive characters: {', '.join(unresponsive)}")
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
            
            # If no turn is currently active, start one
            if not self.turn_state['active_characters']:
                logger.info("Starting new turn for step command")
                self.start_new_turn()
                # Publish state update AFTER starting turn so button reflects active state
                self._publish_turn_state_update()
            else:
                logger.info("Turn already in progress - will complete normally")
                # Publish state update to confirm mode
                self._publish_turn_state_update()
                
        except Exception as e:
            logger.error(f"Error handling turn step command: {e}")
    
    def handle_turn_run(self, sample):
        """Handle manual run turns command"""
        try:
            logger.info("🏃 Manual Run Turns command received")
            self.turn_control_mode = "run"
            self.auto_progression_enabled = True
            
            # Publish state update first (mode change to 'run' disables step/run buttons)
            self._publish_turn_state_update()
            
            # If no turn is currently active, start one
            if not self.turn_state['active_characters']:
                self.start_new_turn()
                # Publish again after turn starts to update progress
                self._publish_turn_state_update()
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
            
            # Publish unified turn state update (includes mode change)
            self._publish_turn_state_update()
            
            logger.info("Auto-progression disabled - waiting for manual step")
                
        except Exception as e:
            logger.error(f"Error handling turn stop command: {e}")
    
    # REMOVED: _publish_turn_control_status() - replaced by _publish_turn_state_update()
    
    def _publish_turn_state_update(self):
        """
        Publish comprehensive turn state update for UI.
        
        This is the NEW unified message that replaces multiple separate messages:
        - Includes complete turn information
        - Includes computed button states (from _compute_ui_button_states)
        - Includes progress information
        - Single message = no race conditions
        """
        try:
            # Get computed button states
            button_states = self._compute_ui_button_states()
            
            # Build comprehensive state update
            state_update = {
                'type': 'turn_state_update',
                'turn': {
                    'number': self.turn_state['turn_number'],
                    'mode': self.turn_control_mode,
                    'auto_progression': self.auto_progression_enabled,
                    'active_count': len(self.turn_state['active_characters']),
                    'completed_count': len(self.turn_state['completed_characters']),
                    'active_characters': self.turn_state['active_characters'].copy(),
                    'completed_characters': self.turn_state['completed_characters'].copy(),
                    'is_active': len(self.turn_state['active_characters']) > 0
                },
                'buttons': {
                    'step': {
                        'enabled': button_states['step']['enabled'],
                        'label': '🎯 Step Turn',
                        'tooltip': button_states['step']['reason']
                    },
                    'run': {
                        'enabled': button_states['run']['enabled'],
                        'label': '🏃 Run',
                        'tooltip': button_states['run']['reason']
                    },
                    'stop': {
                        'enabled': button_states['stop']['enabled'],
                        'label': '⏹️ Stop',
                        'tooltip': button_states['stop']['reason']
                    }
                },
                'timestamp': time.time()
            }
            
            # Publish unified message
            self.turn_state_update_publisher.put(json.dumps(state_update).encode())
            logger.info(f"📤 Published turn_state_update: turn={state_update['turn']['number']}, "
                       f"mode={state_update['turn']['mode']}, "
                       f"step_enabled={state_update['buttons']['step']['enabled']}, "
                       f"progress={state_update['turn']['completed_count']}/{state_update['turn']['active_count']}")
            
        except Exception as e:
            logger.error(f"Error publishing turn state update: {e}")
    
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
            
            # Convert ResourceType enums to strings for JSON serialization
            json_safe_resources = []
            for resource in resources:
                resource_copy = resource.copy()
                if 'type' in resource_copy:
                    resource_copy['type'] = str(resource_copy['type'])
                json_safe_resources.append(resource_copy)
            
            response = {
                'success': True,
                'resources': json_safe_resources
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
            
            # Try direct lookup first (checks resource_registry and patches)
            resource = self.world_map.get_resource_by_name(resource_name)
            
            # If not found, try resource_registry directly (for Note/Collection IDs)
            if not resource and resource_name in self.world_map.resource_registry:
                resource = self.world_map.resource_registry[resource_name]
            
            # If not found and doesn't look like an ID, try named_collections
            if not resource and not resource_name.startswith(('Note_', 'Collection_')):
                if resource_name in self.resource_manager.named_collections:
                    collection_id = self.resource_manager.named_collections[resource_name]
                    resource = self.world_map.resource_registry.get(collection_id)
                    logger.info(f"📚 Resolved collection name '{resource_name}' → {collection_id}")
            
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
    
    def handle_resource_rules_by_name(self, query):
        """Return the resource_rules entry for a resource type (JSON-safe).

        Topic: cognitive/map/resource_rules/<resource_type_or_instance>
        If an instance name is provided, the resource is looked up to determine its type.
        """
        try:
            # Extract name from query key
            key_parts = str(query.key_expr).split('/')
            raw_name = key_parts[-1] if len(key_parts) > 0 else None

            if not raw_name:
                raise ValueError("No resource name provided")

            # Try to resolve as resource instance first to get exact type
            type_name = raw_name
            resource = self.world_map.get_resource_by_name(raw_name)
            if resource:
                # Extract type from resource instance
                rtype = resource.get('type')
                if rtype:
                    type_name = getattr(rtype, 'name', str(rtype))
            # If not found as instance, assume raw_name is the type name itself

            rules = getattr(self.world_map, '_resource_rules', None) or {}
            allocations = rules.get('allocations') or []

            found = None
            for allocation in allocations:
                rt = allocation.get('resource_type')
                rt_name = ''
                try:
                    rt_name = rt.name  # Enum or ResourceType
                except Exception:
                    rt_name = getattr(rt, 'name', str(rt))

                if isinstance(rt_name, str) and rt_name.lower() == type_name.lower():
                    # Build JSON-safe object
                    result = {
                        'resource_type': rt_name,
                    }
                    if 'description' in allocation:
                        result['description'] = allocation.get('description')
                    if 'count' in allocation:
                        result['count'] = allocation.get('count')
                    if 'requires_property' in allocation:
                        result['requires_property'] = allocation.get('requires_property')
                    if 'has_npc' in allocation:
                        result['has_npc'] = allocation.get('has_npc')
                    if 'use' in allocation:
                        # Pass through use effects (list of {need, effect})
                        result['use'] = allocation.get('use')

                    # terrain_weights: convert enum keys to names
                    tw = allocation.get('terrain_weights') or {}
                    if isinstance(tw, dict) and tw:
                        tw_out = {}
                        for k, v in tw.items():
                            key_name = getattr(k, 'name', str(k))
                            tw_out[str(key_name)] = v
                        result['terrain_weights'] = tw_out

                    # Optional names list for this type, if defined
                    names_map = rules.get('names') or {}
                    try:
                        if isinstance(names_map, dict) and rt_name in names_map:
                            result['names'] = names_map[rt_name]
                    except Exception:
                        pass
                    
                    # For Skill type, include all skill instances with metadata
                    if rt_name.lower() == 'skill':
                        skill_instances = []
                        for resource_id, resource_data in self.world_map.resource_registry.items():
                            res_type = resource_data.get('type')
                            res_type_name = getattr(res_type, 'name', str(res_type))
                            
                            if res_type_name.lower() == 'skill':
                                props = resource_data.get('properties', {})
                                instance_info = {
                                    'name': resource_data.get('name', resource_id),
                                    'description': resource_data.get('description', ''),
                                    'tool_name': props.get('tool_name', ''),
                                    'tool_type': props.get('tool_type', 'prompt_augmentation')
                                }
                                skill_instances.append(instance_info)
                        
                        result['skill_instances'] = skill_instances
                    
                    # For all resource types, include tool metadata fields if present
                    tool_metadata_fields = [
                        'tool_name', 'tool_type', 'tool_path', 'tool_md_content',
                        'execution_mode', 'value_type', 'result_type', 
                        'context_injection', 'entry_point', 'input_format', 'output_format'
                    ]
                    for field in tool_metadata_fields:
                        if field in allocation:
                            result[field] = allocation[field]

                    found = result
                    break

            if found is None:
                response = {
                    'success': False,
                    'error': f"Resource type '{raw_name}' not found"
                }
            else:
                response = {
                    'success': True,
                    'resource_rules': found
                }

            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
        except Exception as e:
            logger.error(f"Error handling resource rules query: {e}")
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
            
            # Get resource instance by name (instance-only)
            resource = self.world_map.get_resource_by_name(resource_name)
            if not resource:
                response = {
                    'success': False,
                    'error': f"Resource '{resource_name}' not found"
                }
            else:
                # Determine type and consult rules
                try:
                    rtype_obj = resource.get('type')
                    rtype_name = getattr(rtype_obj, 'name', str(rtype_obj))
                except Exception:
                    rtype_name = None

                rules = getattr(self.world_map, '_resource_rules', None) or {}
                allocations = rules.get('allocations') or []

                remove_allowed = None  # None = unknown type
                for allocation in allocations:
                    rt = allocation.get('resource_type')
                    rt_name = ''
                    try:
                        rt_name = rt.name
                    except Exception:
                        rt_name = getattr(rt, 'name', str(rt))
                    if rtype_name and isinstance(rt_name, str) and rt_name.lower() == rtype_name.lower():
                        # Default to True if key missing
                        remove_allowed = bool(allocation.get('remove_on_take', True))
                        break

                if remove_allowed is False:
                    # Policy: do not remove from map, but treat as success
                    response = {
                        'success': True,
                        'removed': False,
                        'message': f"Removal skipped by policy for '{resource_name}'"
                    }
                elif remove_allowed is None and rtype_name:
                    # Unknown type: skip removal but succeed
                    response = {
                        'success': True,
                        'removed': False,
                        'message': f"Removal skipped: unknown resource type '{rtype_name}'"
                    }
                else:
                    # Proceed with actual removal (either rule allows or no rule exists)
                    resource_id = resource['name']  # The 'name' field contains the resource_id
                    success = self.world_map.remove_resource(resource_id)
                    if success:
                        response = {
                            'success': True,
                            'removed': True,
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

    def handle_resource_place(self, query):
        """Handle resource placement queries. Places resource at agent's current location.

        Topic: cognitive/map/resource/place/<resource_name>
        Payload (optional): {"character_name": "Name"} to identify agent; if omitted, fails.
        """
        try:
            key_parts = str(query.key_expr).split('/')
            resource_name = key_parts[-1] if len(key_parts) > 0 else None
            if not resource_name:
                raise ValueError("No resource name provided")

            # Parse payload for character name
            character_name = None
            if query.payload:
                try:
                    payload = query.payload.to_bytes().decode('utf-8')
                    data = json.loads(payload) if payload else {}
                    character_name = data.get('character_name')
                except Exception:
                    pass
            if not character_name:
                response = {'success': False, 'error': 'Missing character_name in payload'}
                query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
                return

            canonical_character_name = character_name.capitalize()
            if canonical_character_name not in self.agent_registry:
                response = {'success': False, 'error': f"Agent for character '{character_name}' not found"}
                query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
                return

            agent = self.agent_registry[canonical_character_name]
            x, y = agent.x, agent.y

            # Resolve resource by name
            resource = self.world_map.get_resource_by_name(resource_name)
            if not resource:
                response = {'success': False, 'error': f"Resource '{resource_name}' not found"}
                query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
                return

            resource_id = resource['name']
            success = self.world_map.place_resource(resource_id, x, y)

            if success:
                response = {'success': True, 'message': f"Resource '{resource_name}' placed at ({x},{y})"}
            else:
                response = {'success': False, 'error': f"Failed to place resource '{resource_name}'"}

            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
        except Exception as e:
            logger.error(f"Error handling resource placement query: {e}")
            error_response = {'success': False, 'error': str(e)}
            query.reply(query.key_expr, json.dumps(error_response).encode('utf-8'))
    
    def handle_create_note(self, query):
        """
        Handle Note resource creation from skill execution.
        
        Creates a dynamic Note resource instance and places it in the infospace.
        This is called when a skill completes and returns a result that needs to be
        bound to a plan variable.
        
        Topic: cognitive/map/note/create
        Payload: {
            "character_name": str,  # Agent creating the note
            "content": str or dict,  # Skill result content
            "format": "text|json",  # Content format
            "source_skill": str,  # Name of skill that created this
            "source_value": str,  # Input value to the skill
            "properties": dict,  # Optional extra metadata (kind, parent_id, order, span, section, source, entity, edge)
        }
        
        Returns: {
            "success": bool,
            "info_id": str,  # Unique ID of created Note instance
            "location": [x, y]  # Location where placed
        }
        """
        try:
            # Parse payload
            if not query.payload:
                raise ValueError("No payload provided")
            
            payload_bytes = query.payload.to_bytes()
            payload_str = payload_bytes.decode('utf-8')
            payload = json.loads(payload_str)
            
            character_name = payload.get('character_name')
            content = payload.get('content')
            format_type = payload.get('format', 'text')
            source_skill = payload.get('source_skill', '')
            source_value = payload.get('source_value', '')
            extra_props = payload.get('properties', {}) or {}
            
            # Call resource manager to create note
            success, note_id, error_msg, location = self.resource_manager.create_note(
                character_name, content, format_type, source_skill, source_value, extra_props
            )
            
            if success:
                response = {
                    'success': True,
                    'info_id': note_id,
                    'location': list(location)
                }
            else:
                response = {
                    'success': False,
                    'error': error_msg
                }
            
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
            
        except Exception as e:
            logger.error(f"Error creating Note instance: {e}")
            error_response = {
                'success': False,
                'error': str(e)
            }
            query.reply(query.key_expr, json.dumps(error_response).encode('utf-8'))
    
    def handle_create_collection(self, query):
        """
        Handle Collection resource creation from skill execution or explicit create.
        
        Creates a dynamic Collection resource instance and places it in the infospace.
        Collections are lists/arrays that can be indexed and searched.
        
        Topic: cognitive/map/collection/create
        Payload: {
            "character_name": str,  # Agent creating the collection
            "content": list or dict,  # Collection items/content
            "format": "list|dict",  # Content format
            "source_skill": str,  # Name of skill/operation that created this (optional)
            "source_value": str,  # Input value to the operation (optional)
            "collection_name": str,  # Optional stable name for referencing (optional)
            "properties": dict,  # Optional extra metadata (kind, doc_meta, chunking, indexes)
        }
        
        Returns: {
            "success": bool,
            "info_id": str,  # Unique ID of created Collection instance
            "location": [x, y]  # Location where placed
        }
        """
        try:
            # Parse payload
            if not query.payload:
                raise ValueError("No payload provided")
            
            payload_bytes = query.payload.to_bytes()
            payload_str = payload_bytes.decode('utf-8')
            payload = json.loads(payload_str)
            
            character_name = payload.get('character_name')
            content = payload.get('content')
            format_type = payload.get('format', 'list')
            source_skill = payload.get('source_skill', '')
            source_value = payload.get('source_value', '')
            collection_name = payload.get('collection_name', '')
            extra_props = payload.get('properties', {}) or {}
            
            # Call resource manager to create collection
            success, collection_id, error_msg, location = self.resource_manager.create_collection(
                character_name, content, format_type, source_skill, source_value, collection_name, extra_props
            )
            
            if success:
                response = {
                    'success': True,
                    'info_id': collection_id,
                    'location': list(location)
                }
            else:
                response = {
                    'success': False,
                    'error': error_msg
                }
            
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
            
        except Exception as e:
            logger.error(f"Error creating Collection instance: {e}")
            error_response = {
                'success': False,
                'error': str(e)
            }
            query.reply(query.key_expr, json.dumps(error_response).encode('utf-8'))
    
    def handle_collection_add(self, query):
        """
        Handle adding a Note to an existing Collection (controlled mutation).
        
        Topic: cognitive/map/collection/add
        Payload: {
            "collection_id": str,  # Collection to modify
            "note_id": str,  # Note to add
            "agent_name": str  # Agent performing the add
        }
        
        Returns: {
            "success": bool,
            "collection_id": str,
            "item_count": int  # New count after add
        }
        """
        try:
            # Parse payload
            if not query.payload:
                raise ValueError("No payload provided")
            
            payload_bytes = query.payload.to_bytes()
            payload_str = payload_bytes.decode('utf-8')
            payload = json.loads(payload_str)
            
            collection_id = payload.get('collection_id')
            note_id = payload.get('note_id')
            agent_name = payload.get('agent_name')
            operation = payload.get('operation', 'add')  # 'add' or 'update'
            content = payload.get('content')  # For update operation
            
            # Call resource manager
            success, item_count, error_msg = self.resource_manager.add_to_collection(
                collection_id, note_id, agent_name, operation, content
            )
            
            if success:
                response = {
                    'success': True,
                    'collection_id': collection_id,
                    'item_count': item_count
                }
            else:
                response = {
                    'success': False,
                    'error': error_msg
                }
            
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
            
        except Exception as e:
            logger.error(f"Error adding to Collection: {e}")
            error_response = {
                'success': False,
                'error': str(e)
            }
            query.reply(query.key_expr, json.dumps(error_response).encode('utf-8'))
    
    def handle_collection_persist(self, query):
        """
        Handle marking a Note or Collection as persistent.
        
        Topic: cognitive/map/collection/persist
        Payload: {
            "resource_id": str,  # Note or Collection to mark as persistent
            "character_name": str  # Agent performing the persist
        }
        
        Returns: {
            "success": bool,
            "resource_id": str
        }
        """
        try:
            # Parse payload
            if not query.payload:
                raise ValueError("No payload provided")
            
            payload_bytes = query.payload.to_bytes()
            payload_str = payload_bytes.decode('utf-8')
            payload = json.loads(payload_str)
            
            resource_id = payload.get('resource_id') or payload.get('collection_id')
            character_name = payload.get('character_name')
            
            # Call resource manager
            success, error_msg = self.resource_manager.mark_persistent(resource_id, character_name)
            
            if success:
                response = {
                    'success': True,
                    'resource_id': resource_id
                }
            else:
                response = {
                    'success': False,
                    'error': error_msg
                }
            
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
            
        except Exception as e:
            logger.error(f"Error persisting Collection: {e}")
            error_response = {
                'success': False,
                'error': str(e)
            }
            query.reply(query.key_expr, json.dumps(error_response).encode('utf-8'))
    
    def handle_resource_viewer(self, query):
        """
        Handle resource content viewing for UI.
        
        Topic: cognitive/map/resource/view/<resource_id>
        
        Returns: {
            "success": bool,
            "type": "Note" | "Collection",
            "content": any,
            "metadata": {
                "name": str,
                "creator": str,
                "created_at": str,
                "description": str
            }
        }
        """
        try:
            # Extract resource_id from query key
            key_expr = str(query.key_expr)
            resource_id = key_expr.split('/')[-1]
            
            # Fetch resource from registry
            resource = self.world_map.resource_registry.get(resource_id)
            if not resource:
                raise ValueError(f"Resource {resource_id} not found")
            
            # Get resource type
            resource_type = resource.get('type')
            type_name = getattr(resource_type, 'name', str(resource_type))
            
            # Get content from properties
            props = resource.get('properties', {})
            content = props.get('content')
            
            # Build metadata
            metadata = {
                'name': resource.get('name', resource_id),
                'creator': props.get('created_by', props.get('creator', 'unknown')),
                'created_at': props.get('created_at', ''),
                'description': resource.get('description', ''),
                'format': props.get('format', 'unknown'),
                'source_skill': props.get('source_skill', '')
            }
            
            # For Collections, also include item count
            if type_name == 'Collection':
                if isinstance(content, list):
                    metadata['item_count'] = len(content)
                metadata['persistent'] = props.get('persistent', False)
            
            response = {
                'success': True,
                'type': type_name,
                'content': content,
                'metadata': metadata
            }
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
            
        except Exception as e:
            logger.error(f"Error fetching resource for viewer: {e}")
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
                    # Fallback to center of map using actual map dimensions
                    location = (self.world_map.width // 2, self.world_map.height // 2)
                
                # Create agent instance
                agent = MapAgent(location[0], location[1], self.world_map, canonical_character_name)
                
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
                agent: MapAgent = self.agent_registry[canonical_character_name]
                
                # Call agent's look method
                look_result = agent.look()
                view = {}
                for dir in ['Current','North', 'Northeast', 'East', 'Southeast', 
                        'South', 'Southwest', 'West', 'Northwest']:
                    dir_obs = self.world_map.extract_direction_info(look_result, dir)
                    view[dir] = dir_obs

                view_text, resources, characters, paths, percept_summary = self.world_map.hash_direction_info(view, distance_threshold=16)
                
                response = {
                    'success': True,
                    'character_name': canonical_character_name,
                    'look_result': view_text,
                    'location': [agent.x, agent.y],
                    'characters': list(set(characters)),
                    'paths': list(set(paths)),
                }
            
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
            logger.info(f'Agent look command handled for {query.key_expr}')
            
        except Exception as e:
            traceback.print_exc()
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
                agent: MapAgent = self.agent_registry[canonical_character_name]
                
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


    def handle_agent_move_to(self, query):
        """Handle agent move-to-resource command (instance-only)."""
        try:
            # Extract character name from query key
            key_parts = str(query.key_expr).split('/')
            character_name = key_parts[-2] if len(key_parts) > 1 else None
            if not character_name:
                raise ValueError("No character name provided")

            canonical_character_name = character_name.capitalize()
            if canonical_character_name not in self.agent_registry:
                response = {
                    'success': False,
                    'error': f"Agent for character '{character_name}' not found"
                }
                query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
                return

            # Parse payload for resource instance name
            resource_name = None
            if query.payload:
                try:
                    payload = query.payload.to_bytes().decode('utf-8')
                    data = json.loads(payload) if payload else {}
                    resource_name = data.get('resource_name')
                except Exception:
                    resource_name = None
            if not resource_name:
                response = {'success': False, 'error': 'Missing resource_name in payload'}
                query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
                return

            # Resolve instance by name and move
            resource = self.world_map.get_resource_by_name(resource_name)
            if not resource:
                response = {
                    'success': False,
                    'error': f"Resource '{resource_name}' not found"
                }
                query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
                return

            resource_id = resource.get('name')
            agent: MapAgent = self.agent_registry[canonical_character_name]
            ok = agent.move_to_resource(resource_id)

            if ok:
                response = {
                    'success': True,
                    'location': [agent.x, agent.y]
                }
            else:
                response = {
                    'success': False,
                    'error': 'Failed to move to resource'
                }

            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
        except Exception as e:
            logger.error(f"Error handling agent move_to: {e}")
            error_response = {
                'success': False,
                'error': str(e)
            }
            query.reply(query.key_expr, json.dumps(error_response).encode('utf-8'))


    def handle_agent_use_path(self, query):
        """Handle agent use_path command"""
        try:
            # Extract character name from query key
            key_parts = str(query.key_expr).split('/')
            character_name = key_parts[3] if len(key_parts) > 3 else None
            if not character_name:
                raise ValueError("No character name provided")

            canonical_character_name = character_name.capitalize()
            if canonical_character_name not in self.agent_registry:
                response = {
                    'success': False,
                    'error': f"Agent for character '{character_name}' not found"
                }
            else:
                agent = self.agent_registry[canonical_character_name]
                # Parse path_id from payload
                path_id = None
                if query.payload:
                    payload = query.payload.to_bytes().decode('utf-8')
                    try:
                        data = json.loads(payload)
                        path_id = data.get('path_id')
                    except Exception:
                        path_id = None
                if not path_id:
                    response = {'success': False, 'error': 'Missing path_id'}
                else:
                    ok = agent.use_path(path_id)
                    response = {
                        'success': bool(ok),
                        'location': [agent.x, agent.y]
                    }

            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
        except Exception as e:
            logger.error(f"Error handling agent use_path: {e}")
            error_response = {
                'success': False,
                'error': str(e)
            }
            query.reply(query.key_expr, json.dumps(error_response).encode('utf-8'))
    
    def handle_map_types(self, query):
        """Handle map types query - returns available terrain, infrastructure, property, resource types, and skill instances"""
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
            
            # Get skill instances with name, description, and type_name
            skills = []
            for resource_id, resource_data in self.world_map.resource_registry.items():
                res_type = resource_data.get('type')
                res_type_name = getattr(res_type, 'name', str(res_type))
                
                if res_type_name.lower() == 'skill':
                    props = resource_data.get('properties', {})
                    skill_info = {
                        'name': resource_data.get('name', resource_id),
                        'description': resource_data.get('description', ''),
                        'type_name': props.get('tool_name', '')
                    }
                    skills.append(skill_info)
            
            response = {
                'success': True,
                'terrain_types': terrain_types,
                'infrastructure_types': infrastructure_types,
                'property_types': property_types,
                'resource_types': resource_types,
                'character_names': list(self.agent_registry.keys()),
                'skills': skills
            }
            
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
            logger.info(f'🗺️ Map types query: returned {len(terrain_types)} terrain, {len(infrastructure_types)} infrastructure, {len(property_types)} property, {len(resource_types)} resource types, {len(self.agent_registry)} characters, {len(skills)} skills')
            
        except Exception as e:
            logger.error(f'Error handling map types query: {e}')
            error_response = {
                'success': False,
                'error': str(e),
                'terrain_types': [],
                'infrastructure_types': [],
                'property_types': [],
                'resource_types': [],
                'character_names': [],
                'skills': []
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
                
                logger.debug(f"⏰ Time advanced by {advance_minutes} minutes: {old_time_info['datetime']} → {new_time_info['datetime']}")
                
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
                
                # Use the map's own visibility logic instead of hardcoded terrain checks
                if not self.world_map.is_visible(x1, y1, x, y, observer_height=5):
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
            
            # === ZENOH PUBLICATION (ad-hoc) ===
            # NAME: agent_detected
            # TOPIC: cognitive/{character}/sense/visual/agent_detected
            # DESCRIPTION: Character detected another character within line of sight
            # PAYLOAD: {"agent_name": str, "position": tuple, "distance": float, "timestamp": str}
            # TRIGGERS: (social activities - greet, approach, converse)
            # ========================
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
                        # Determine spawn location: default (map center) or scenario-provided resource instance
                        spawn_x, spawn_y = self.world_map.width // 2, self.world_map.height // 2
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
                                        logger.error(f"Invalid resource location for '{resource_location_name}' - defaulting to map center")
                                else:
                                    logger.error(f"Resource instance '{resource_location_name}' not found for '{canonical_character_name}' - defaulting to map center")

                        # Create agent at resolved spawn point
                        agent = MapAgent(spawn_x, spawn_y, self.world_map, canonical_character_name)
                        
                        # Register agent with world map
                        self.world_map.register_agent(agent)
                        
                        # Store in our registry
                        self.agent_registry[canonical_character_name] = agent
                        
                        # Initialize visibility tracking for this agent
                        self.agent_visibility[canonical_character_name] = set()
                        
                        logger.info(f"Agent created for character '{canonical_character_name}' at ({agent.x},{agent.y})")
                        
                        # Publish updated button states (character count changed)
                        self._publish_turn_state_update()
                        
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
    
    def handle_launcher_ready(self, sample):
        """Handle launcher ready signal to learn expected character count."""
        try:
            data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            
            if data.get('status') == 'ready':
                self.expected_character_count = data.get('character_count', 0)
                self.system_ready = True
                
                logger.info(f"🚀 Launcher ready signal: expecting {self.expected_character_count} characters")
                
                # Don't auto-start turns even after restore - wait for user to click Step/Run
                # This matches the behavior of fresh initialization
                
                # Publish updated button states now that we know the expected count
                self._publish_turn_state_update()
                
        except Exception as e:
            logger.error(f"Error handling launcher ready signal: {e}")
    
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
    
    def handle_world_state_query(self, query):
        """Handle world state query"""
        response = {'world_state': self.world_state}
        query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
    
    def handle_world_state_update(self, sample):
        """Handle world state update request by queuing it"""
        update_text = sample.payload.to_bytes().decode('utf-8')
        if self.world_state_update_queue.full():
            logger.warning("World state update queue is full, dropping oldest update")
            self.world_state_update_queue.get()
        self.world_state_update_queue.put(update_text)
        logger.info("World state update queued")
    
    def start_world_state_worker(self):
        """Start the world state worker thread"""
        self.world_state_worker = threading.Thread(target=self._world_state_worker, daemon=True)
        self.world_state_worker.start()
        logger.info("World state worker thread started")
    
    def _world_state_worker(self):
        """Worker thread that processes world state updates serially"""
        while not self.shutdown_requested:
            if self.world_state_update_queue.empty():
                time.sleep(1.0)
                continue
            
            update_json = self.world_state_update_queue.get()
            
            if not self.llm_client:
                logger.warning("LLM client not available for world state update")
                continue
            
            # Parse the JSON to extract fields
            update_data = json.loads(update_json)
            action = update_data.get('action', {})
            update_text = update_data.get('update_text', '')
            
            logger.info(f"Processing world state update: {update_text[:50]}...")
            
            bindings = {
                'current_world_state': self.world_state,
                'action': json.dumps(action, indent=2) if action else '',
                'update_text': update_text,
                'simulation_time': self.world_map.datetime.isoformat() if self.world_map else '',
                'setting': self.setting or ''
            }
            
            response = self.llm_client.generate(
                messages=[WORLD_STATE_UPDATE_TEMPLATE],
                bindings=bindings,
                max_tokens=240,
                temperature=0.7
            )
            if response and hasattr(response, 'text'):
                self.world_state = response.text
                logger.info(f"World state updated: {self.world_state[:100]}...")
    
    def run(self):
        """Run the map node"""
        logger.info(f"Map node started with map file: {self.map_file}")
        
        try:
            # Keep the node running
            while not self.shutdown_requested:
                # Check for turn timeouts (skip entirely in debug mode)
                if not self.debug_disable_timeout:
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
                            agent = MapAgent(x, y, self.world_map, character_name)
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
                    
                    # Restore world state if available
                    if 'world_state' in world_data:
                        self.world_state = world_data['world_state']
                        logger.info(f"📂 World state restored: {self.world_state[:50]}...")
                    else:
                        logger.info("📂 No world state in saved data")
                    
                    # Restore Note and Collection instances (via resource manager)
                    self.resource_manager.load_resources(world_data)
                    
                    logger.info(f"📂 Loaded world data for '{self.world_name}'")
                    
                    # Start turn management if agents were restored
                    if len(self.agent_registry) > 0:
                        logger.info(f"📂 {len(self.agent_registry)} agents restored - will start turns after launcher ready signal")
                        # Don't start turns immediately - wait for launcher_ready signal
                        # This ensures all character nodes have time to fully initialize
                        
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
                'world_state': self.world_state,
                'agents': []
            }
            
            # Save agent positions
            for character_name, agent in self.agent_registry.items():
                agent_data = {
                    'character_name': character_name,
                    'position': [agent.x, agent.y]
                }
                world_data['agents'].append(agent_data)
            
            # Save Note and Collection instances (via resource manager)
            resource_data = self.resource_manager.save_resources()
            world_data['note_instances'] = resource_data['note_instances']
            world_data['note_counter'] = resource_data['note_counter']
            world_data['collection_instances'] = resource_data['collection_instances']
            world_data['collection_counter'] = resource_data['collection_counter']
            
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
            
            # If they're already locked together, grant the lock (same conversation)
            if (target_canon in requester_locked_with and 
                requester_canon in target_locked_with):
                return True
            
            # If either is locked with someone else, deny
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
    
    def handle_index_request(self, sample):
        """
        Handle request to create/update searchable vector store.
        
        Store is created using Collection ID as the key. The Collection becomes indexed.
        
        Message format:
        {
            'agent_name': str,
            'collection_id': str,  # Collection_N - also used as store name
            'index_type': 'semantic' | 'keyword' | 'hybrid',
            'fields': {field_name: 'embed'|'keyword'|'store'}
        }
        """
        request = json.loads(sample.payload.to_bytes().decode('utf-8'))
        agent_name = request.get('agent_name')
        collection_id = request.get('collection_id')
        index_type = request.get('index_type', 'semantic')
        fields = request.get('fields', {})
        
        # Call resource manager
        success, indexed_count, error_msg = self.resource_manager.index_collection(
            agent_name, collection_id, index_type, fields
        )
        
        if success:
            response = {
                'status': 'success',
                'collection_id': collection_id,
                'indexed_count': indexed_count
            }
        else:
            response = {
                'status': 'error',
                'reason': error_msg
            }
        
        # Publish response
        response_topic = f"map/{self.world_name}/index_response/{agent_name}"
        time.sleep(0.05)
        self.session.put(response_topic, json.dumps(response))
    
    def handle_search_request(self, sample):
        """
        Handle semantic search query on indexed Collection.
        
        Store is looked up using Collection ID as the key.
        
        Message format:
        {
            'agent_name': str,
            'collection_id': str,  # Collection_N - also the store name
            'query': str,
            'mode': 'semantic' | 'keyword' | 'hybrid',
            'limit': int,
            'threshold': float (optional, default 0.0),
            'return_mode': 'chunks' | 'notes' (optional, default 'chunks')
        }
        """
        request = json.loads(sample.payload.to_bytes().decode('utf-8'))
        agent_name = request.get('agent_name')
        collection_id = request.get('collection_id')
        query = request.get('query')
        mode = request.get('mode', 'semantic')
        limit = request.get('limit', 5)
        threshold = request.get('threshold', 0.0)
        return_mode = request.get('return_mode', 'chunks')
        
        # Call resource manager
        success, results, error_msg = self.resource_manager.search_collection(
            agent_name, collection_id, query, mode, limit, threshold, return_mode
        )
        
        if success:
            response = {
                'status': 'success',
                'results': results,
                'count': len(results)
            }
        else:
            response = {
                'status': 'error',
                'reason': error_msg
            }
        
        # Publish response
        response_topic = f"map/{self.world_name}/search_response/{agent_name}"
        time.sleep(0.05)
        self.session.put(response_topic, json.dumps(response))
    
    def handle_tool_request(self, sample):
        """
        Handle tool execution request (placeholder for Phase 1).
        
        Message format:
        {
            'agent_name': str,
            'tool_name': str,
            'input_value': str,
            'reason': str
        }
        """
        request = json.loads(sample.payload.to_bytes().decode('utf-8'))
        agent_name = request.get('agent_name')
        tool_name = request.get('tool_name')
        
        logger.info(f"Tool request from {agent_name}: {tool_name}")
        
        # Phase 1: Return placeholder response
        # TODO: Actually execute tools from /maps/skills/ directory
        response = {
            'status': 'success',
            'result': f"[Tool {tool_name} executed - placeholder result]"
        }
        
        response_topic = f"map/{self.world_name}/tool_response/{agent_name}"
        time.sleep(0.05)
        self.session.put(response_topic, json.dumps(response))
    
    def handle_scan_request(self, sample):
        """
        Handle scan request to find resources by name or interface.
        
        Message format:
        {
            'agent_name': str,
            'target': str (resource name or interface type),
            'scan_type': str (tool, information, etc.)
        }
        """
        request = json.loads(sample.payload.to_bytes().decode('utf-8'))
        agent_name = request.get('agent_name')
        target = request.get('target')
        
        logger.info(f"Scan request from {agent_name}: {target}")
        
        # Get all resources and search for matches
        resources = self.world_map.get_resource_list()
        matching_resources = []
        
        for resource_info in resources:
            resource_name = resource_info.get('name', '')
            if resource_name and target.lower() in resource_name.lower():
                resource_type = resource_info.get('type', '')
                # Convert enum to string for JSON serialization
                if hasattr(resource_type, 'name'):
                    resource_type = resource_type.name
                matching_resources.append({
                    'name': resource_name,
                    'type': str(resource_type),
                    'location': resource_info.get('location', (0, 0)),
                    'id': resource_info.get('id', '')
                })
        
        # Return first match or error
        if matching_resources:
            result = matching_resources[0]
            response = {
                'status': 'success',
                'result': result
            }
            logger.info(f"Scan found: {result['name']}")
        else:
            response = {
                'status': 'error',
                'reason': f"No resource matching '{target}' found"
            }
            logger.info(f"Scan found no matches for '{target}'")
        
        response_topic = f"map/{self.world_name}/scan_response/{agent_name}"
        time.sleep(0.05)
        self.session.put(response_topic, json.dumps(response))
    
    def handle_move_request(self, sample):
        """
        Handle move request to navigate to resource.
        
        Message format:
        {
            'agent_name': str,
            'target': str (resource name) or dict (resource info)
        }
        """
        request = json.loads(sample.payload.to_bytes().decode('utf-8'))
        agent_name = request.get('agent_name')
        target = request.get('target')
        
        logger.info(f"Move request from {agent_name}: {target}")
        
        # Get agent
        agent = self.agent_registry.get(agent_name)
        if not agent:
            response = {
                'status': 'error',
                'reason': f"Agent '{agent_name}' not registered"
            }
        else:
            # If target is dict with location, use it
            if isinstance(target, dict) and 'location' in target:
                target_x, target_y = target['location']
            else:
                # Find resource by name
                resources = self.world_map.get_resource_list()
                target_resource = None
                
                for resource_info in resources:
                    resource_name = resource_info.get('name', '')
                    if resource_name and target.lower() in resource_name.lower():
                        target_resource = resource_info
                        break
                
                if not target_resource:
                    response = {
                        'status': 'error',
                        'reason': f"Resource '{target}' not found"
                    }
                    response_topic = f"map/{self.world_name}/move_response/{agent_name}"
                    time.sleep(0.05)
                    self.session.put(response_topic, json.dumps(response))
                    return
                
                target_x, target_y = target_resource.get('location', (0, 0))
            
            # Move agent to target location
            agent.x = target_x
            agent.y = target_y
            
            response = {
                'status': 'success',
                'location': (target_x, target_y)
            }
            logger.info(f"Moved {agent_name} to ({target_x}, {target_y})")
        
        response_topic = f"map/{self.world_name}/move_response/{agent_name}"
        time.sleep(0.05)
        self.session.put(response_topic, json.dumps(response))




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
    parser.add_argument('--max-turns', type=int, default=None,
                       help='Maximum number of turns before stopping (optional)')
    parser.add_argument('--server', default='openai',
                       help='LLM server name (default: openai)')
    parser.add_argument('--model', default='gpt-4.1',
                       help='LLM model name (default: gpt-4.1)')
    
    args = parser.parse_args()
    
    try:
        # Set up signal handlers
        signal.signal(signal.SIGTERM, signal_handler)
        signal.signal(signal.SIGINT, signal_handler)
        
        # Create and run map node
        map_node = MapNode(args.map_file, args.world_name, setting=args.setting, max_turns=args.max_turns, server_name=args.server, model_name=args.model)
        signal_handler.map_node = map_node  # Store reference for signal handler
        
        map_node.run()
        
    except Exception as e:
        logger.error(f"Failed to start map node: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main() 