#!/usr/bin/env python3
"""
Zenoh Executive Node

This node implements the OODA loop for character decision-making and action execution.

"""

import math
import random
import traceback
import zenoh
from zenoh import QueryTarget, ConsolidationMode
import json
import time
import threading
import logging
import sys
import signal
import argparse
from datetime import datetime
from typing import Dict, List, Any, Union, Optional
from Messages import SystemMessage, UserMessage
from activity import ActivityManager, derive_drive_lexicon
import utils.hash_utils as hash_utils
from utils.llm_api import LLM
from utils.zenoh_utils import datetime_handler
import plan as plan_module
from dataclasses import dataclass, asdict
import os
from templates import DRIVE_ASSESSMENT_TEMPLATE, GOAL_TEMPLATE, PLAN_TEMPLATE, PHYSIOLOGICAL_STATES, PLAN_VERBS, REWRITE_TEMPLATE
from plan import generate_plan_with_context
from utils.format_utils import format_map_types, format_views_compact
from utils.state_utils import tick_state, apply_restore, initialize_character_states
from utils.format_utils import format_views_compact
from utils.condition_utils import deref_plan_target

# Configure logging with unbuffered output
# Console handler with WARNING level (less verbose)
console_handler = logging.StreamHandler(sys.stdout)
console_handler.setLevel(logging.INFO)

# File handler with INFO level (full logging)
file_handler = logging.FileHandler('logs/executive_node.log', mode='w')
file_handler.setLevel(logging.INFO)
if os.getenv('CWB_DEBUG', '') in ('1', 'true', 'yes', 'on'):
    file_handler.setLevel(logging.INFO)

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
    handlers=[console_handler, file_handler],
    force=True
)
logger = logging.getLogger('executive_node')
logger.setLevel(logging.INFO)

# Import LLM client
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__))))

from llm_client import ZenohLLMClient

@dataclass
class ActionRecord:
    """Record of an action and its result."""
    action: Dict[str, Any]
    result: Optional[str]
    timestamp: datetime
    # Optional telemetry fields (kept lightweight; None when not applicable)
    step_id: Optional[int] = None
    plan_id: Optional[str] = None
    requested_target: Optional[str] = None
    resolved_target: Optional[str] = None
    resolution_status: Optional[str] = None     # resolved | too_far | not_visible | not_in_inventory | ambiguous | not_found | passthrough
    preconditions: Optional[Dict[str, Any]] = None  # e.g., {"visible": bool, "near": bool, "distance": float}
    outcome_status: Optional[str] = None         # success | failure | skipped | no_op | not_implemented
    failure_code: Optional[str] = None           # e.g., target_too_far | target_not_visible | inventory_missing | while_max_iterations | error
    started_at: Optional[datetime] = None
    ended_at: Optional[datetime] = None
    duration_ms: Optional[int] = None
    notes: Optional[str] = None
    # Phase 1 telemetry
    hunger_after: Optional[float] = None
    fatigue_after: Optional[float] = None
    thirst_after: Optional[float] = None
    proposed_minutes: Optional[int] = None
    simulation_time: Optional[str] = None
    # Optional traceability/telemetry extensions
    bindings_after: Optional[Dict[str, Any]] = None
    binding_evidence: Optional[Dict[str, Any]] = None
    feature_snapshot: Optional[Dict[str, Any]] = None

class ZenohExecutiveNode:
    """
    Executive node that implements the OODA loop:
    - Observe: Collect current situation and sense data
    - Orient: Assess current state and goals
    - Decide: Choose next action
    - Act: Execute the chosen action
    """
    
    def __init__(self, character_name="default", character_config=None):
        # Store character info (canonicalized)
        self.character_name = character_name.capitalize()
        self.character_config = character_config or {}
        self.drives = self.character_config.get('drives', [])
        self.drives_str = '\n'.join(self.drives)   
        self.drive_lex = derive_drive_lexicon(self.drives_str)
        
        # Debug mode flag - must be set early as it's used throughout initialization
        self.debug = str(os.getenv('CWB_DEBUG', '')).lower() in ('1', 'true', 'yes', 'on')
        if self.debug:
            console_handler.setLevel(logging.INFO)
            logger.info(f'🔧 Debug mode enabled for {self.character_name}')
        
        # Manual control flags
        self.manual = bool(self.character_config.get('manual', False))
        self.manual_response = bool(self.character_config.get('manual_response', False))
        
        # Initialize Zenoh session
        config = zenoh.Config()
        self.session = zenoh.open(config)
        
        # Subscriber for sense data (character-specific)
        self.sense_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/sense_data",
            self.sense_data_callback
        )
        
        # Subscriber for situation data (character-specific)
        self.situation_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/situation/update",
            self.situation_callback
        )
        
        # Publisher for actions (character-specific)
        self.action_publisher = self.session.declare_publisher(f"cognitive/{character_name}/action")

        # Publisher for map update requests (character-specific)
        self.map_update_request_publisher = self.session.declare_publisher(f"cognitive/{character_name}/situation/request_update")
        # Publisher for plan logs (analytics)
        self.plan_log_publisher = self.session.declare_publisher(
            f"cognitive/{character_name}/planning/plan_log"
        )
        
        # Publisher for memory storage (character-specific)
        self.memory_publisher = self.session.declare_publisher(f"cognitive/{character_name}/memory/store")
        
        
        # Publisher for goals (character-specific)
        self.goal_publisher = self.session.declare_publisher(f"cognitive/{character_name}/goal")
        
        # Publisher for decided actions (character-specific) 
        self.decided_action_publisher = self.session.declare_publisher(f"cognitive/{character_name}/decided_action")
        
        # Publisher for current plans (character-specific)
        self.current_plan_publisher = self.session.declare_publisher(f"cognitive/{character_name}/current_plan")
        
        # Publisher for current activities (character-specific)
        self.current_activity_publisher = self.session.declare_publisher(f"cognitive/{character_name}/current_activity")
        # Publisher for current internal state (character-specific)
        self.current_state_publisher = self.session.declare_publisher(f"cognitive/{character_name}/current_state")
        
        # Backward compatibility properties
        @property
        def last_action(self):
            """Get the last action for backward compatibility."""
            return self.action_history[-1].action if self.action_history else None
        
        @property
        def last_action_result(self):
            """Get the last action result for backward compatibility."""
            return self.action_history[-1].result if self.action_history else None
        
        # LLM client
        llm_config = self.character_config.get('llm_config', {})
        server_name = llm_config.get('server_name', 'openai')
        model_name = llm_config.get('model_name', 'gpt-4.1')
        
        self.llm_client = ZenohLLMClient(server_name=server_name, model_name=model_name, service_timeout=200.0 if not self.debug else 300.0)
        logger.info(f'🤖 LLM client initialized (server={server_name}, model={model_name})')
        self.llm = LLM(server_name=server_name, model_name=model_name)
        
        # Detect infospace and initialize infospace executor if needed
        self.is_infospace = self.character_config.get('is_infospace', False)
        self.map_name = self.character_config.get('map_name', 'infolab' if self.is_infospace else 'default')
        self.infospace_executor = None
        self.available_tools = {}
        
        # Initialize unified planner
        from unified_planner import UnifiedPlanner
        world_type = 'infospace' if self.is_infospace else 'physical'
        self.planner = UnifiedPlanner(
            llm_client=self.llm_client,
            world_type=world_type,
            map_name=self.map_name,
            logger_instance=logger
        )
        self.available_tools = self.planner.available_tools
        
        if self.is_infospace:
            from infospace_executor import InfospaceExecutor
            
            self.infospace_executor = InfospaceExecutor(
                character_name,
                self.session,
                self.map_name,
                self.llm_client,
                self.available_tools
            )
            logger.info(f'🧩 Infospace executor initialized for {character_name}')
        
        # Internal state
        self.action_counter = 0
        self.last_sense_data = None
        self.last_situation_data = None
        
        # OODA loop state
        self.current_goal = None
        self.goal_source = None  # Track goal origin: 'ui', 'autonomous', or None
        self.awaiting_user_input = False  # Pause autonomous behavior after UI goal completion
        self.plan_just_generated = False  # Skip execution on same turn as plan generation
        self.observations = None
        self.interrupt_pending = False
        self.text_input_queue = []
        self.action_history = []  # List of ActionRecord instances
        
        # Plan execution state
        self.current_activity = None
        self.current_step = None
        self.current_plan = None
        self.plan_state = None
        self.plan_bindings_cache = {}
        # Snapshot of percepts at plan start (normalized, optional)
        self.percepts_at_plan: Optional[List[Dict[str, Any]]] = None
        self.plan_bindings = {}  # Store scan action results: {var_name: scan_result}
        self.plan_summary_completed = False  # Track if current plan has been summarized
        self.plan_summary = None
        # Control-flow telemetry and plan/step identifiers
        self.control_flow_events: List[Dict[str, Any]] = []
        self.current_plan_id: Optional[str] = None
        self.plan_counter: int = 0
        self.step_counter: int = 0
        self.current_plan_prompt_template:str = ''
        self.current_plan_prompt_bindings:dict = {}
        self.current_plan_prompt: str = ''  # Set during LLM planning or manual plan load
        self.plan_log: List[Dict[str, Any]] = [] # log of plans and actions
        # Track simulation time at plan boundaries
        self.current_plan_start_sim_iso: Optional[str] = None
        # Local physiology/state. 0=good, 100=worst.
        self.self_state: Dict[str, Dict[str, float]] = initialize_character_states()
        # Local cache of inventory item ids (exact strings)
        self.inventory_cache: List[str] = []
        self.last_state_update_time: Optional[datetime] = None
        # Ontology (guard for manual characters / missing files)
        try:
            if self.character_config.get('ontology', True):
                self.ontology = json.load(open(f'../scenarios/{self.character_name}-activity-ontology.json'))
            else:
                self.ontology = {}
        except Exception as e:
            self.ontology = {}
            if not self.manual:
                logger.warning(f"No activity ontology for {self.character_name}: {e}")
        self.map_types = None # wait to initialize untill all characters have registered

        # Publish initial state snapshot so UI has data before first time advance
        try:
            state_payload = {
                'character': self.character_name,
                'timestamp': datetime.now().isoformat(),
                'state': self.self_state
            }
            self.current_state_publisher.put(json.dumps(state_payload))
        except Exception:
            pass
        # Turn management
        self.turn_subscriber = self.session.declare_subscriber(
            "cognitive/map/turn/go",
            self.turn_callback
        )
        self.turn_complete_publisher = self.session.declare_publisher(
            f"cognitive/map/turn/complete/{character_name}"
        )
        self.time_proposal_publisher = self.session.declare_publisher(
            "cognitive/map/time_proposal"
        )
        self.world_state_update_publisher = self.session.declare_publisher(
            "cognitive/map/world_state/update"
        )
        self.perception_action_result_publisher = self.session.declare_publisher(
            f"cognitive/{character_name}/perception/action_result"
        )
        self.waiting_for_turn = True
        self.current_turn_number = 0
        
        # Time advancement management
        self.time_advanced_subscriber = self.session.declare_subscriber(
            "cognitive/map/time_advanced",
            self.time_advanced_callback
        )
        self.current_time = None  # Will be set when we receive time updates
        
        # Subscriber for shutdown commands (global)
        self.shutdown_subscriber = self.session.declare_subscriber(
            "cognitive/shutdown/executive",
            self.shutdown_callback
        )
        
        # Subscriber for end dialog queries (character-specific)
        self.end_dialog_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/dialog_end",
            self._dialog_end_callback
        )
        # Shutdown flags
        self.shutdown_requested = False
        self._shutting_down = False

        self.inspections = {} # cache of inspections
        self.uses = {} # cache of uses
        self.activities = {} # dictionary of all available activities for initializing activity manager
        try:
            if self.character_config.get('activities', True):
                self.activities = json.load(open(f'../scenarios/{self.character_name}-activities.json'))
        except Exception as e:
            if not self.manual:
                logger.error(f'Error loading activities for {self.character_name}: {e}')

        logger.info(f'🧠 Zenoh Executive Node initialized for character: {character_name}')
        logger.info(f'   - Subscribing to: cognitive/{character_name}/sense_data')
        logger.info(f'   - Subscribing to: cognitive/{character_name}/situation/update')
        logger.info(f'   - Subscribing to: cognitive/map/turn/go')
        logger.info(f'   - Subscribing to: cognitive/map/time_advanced')
        logger.info(f'   - Subscribing to: cognitive/{character_name}/end_dialog')
        logger.info(f'   - Publishing to: cognitive/{character_name}/action')
        logger.info(f'   - Publishing to: cognitive/{character_name}/situation/request_update')
        logger.info(f'   - Publishing to: cognitive/{character_name}/memory/store')
        logger.info(f'   - Publishing to: cognitive/{character_name}/goal')
        logger.info(f'   - Publishing to: cognitive/{character_name}/decided_action')
        logger.info(f'   - Publishing to: cognitive/map/turn/complete/{character_name}')
        logger.info(f'   - Publishing to: cognitive/map/time_proposal')

        self.activity_manager = None
        if self.activities:
            self.activity_manager = ActivityManager(self, self.activities, self.llm_client, self.map_types)

        # Register signal handlers for graceful shutdown
        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)

    def _snapshot_physiology(self, action_record: ActionRecord) -> None:
        """Capture current physiology into the given action_record.
        Errors are contained here to avoid try proliferation in callers.
        """
        try:
            action_record.hunger_after = float(self.self_state.get('hunger', {}).get('value', 0.0))
            action_record.fatigue_after = float(self.self_state.get('fatigue', {}).get('value', 0.0))
            action_record.thirst_after = float(self.self_state.get('thirst', {}).get('value', 0.0))
        except Exception as e:
            logger.error(f'Error capturing physiology: {e}')
            traceback.print_exc()
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully."""
        logger.warning(f'Received signal {signum}, initiating shutdown...')
        self.shutdown_requested = True
    
    def run(self):
        """Main OODA loop."""
        try:
            logger.info('Executive Node running - press Ctrl+C to stop')
            
            # Announce character presence
            self._announce_character()
            time.sleep(0.1)
            self.map_update_request_publisher.put(json.dumps({'type': 'announcement look'}))
            time.sleep(1.0)
            
            # Start OODA loop
            while not self.shutdown_requested:
                if self.waiting_for_turn:
                    time.sleep(0.2)
                else:
                    try:
                        # Priority order: Physiological overrides → Text input → Normal OODA
                        goal, key = self.check_physiological_overrides()
                        if key:
                            # Send canned response to any queued text (no turn cost)
                            if self.text_input_queue:
                                self._send_canned_physiological_response(key)
                            
                            if goal: # new goal! plan for it.
                                self.current_goal = goal
                                self._publish_goal(self.current_goal)
                                logger.info(f'🎯 {self.character_name} oriented to goal: {self.current_goal.to_string()}')
                                action, status = self.plan_and_initiate_execution(goal)
                        elif self.text_input_queue:
                            self._process_text_input()
                        self._run_ooda_loop()
                    except Exception as e:
                        traceback.print_exc()
                        logger.error(f'Error in OODA loop: {e}')
                    self._complete_turn()
                time.sleep(0.2)
                
        except KeyboardInterrupt:
            logger.info('Executive Node shutting down...')
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
                'character_name': self.character_name,
                'character_config': self.character_config,
           }
            
            # Publish announcement
            self.action_publisher.put(json.dumps(announcement_data))
            logger.info(f'📢 Announced character presence: {self.character_name}')
            
        except Exception as e:
            logger.error(f'Error announcing character: {e}')
    
    def _send_canned_physiological_response(self, goal_type: str):
        """Send automatic response when physiological need interrupts conversation."""
        if not self.text_input_queue:
            return
        
        # Pop one text from queue
        sense_data = self.text_input_queue.pop(0)
        content = sense_data['content']
        try:
            content_data = json.loads(content)
            source = content_data.get('source', 'someone')
        except (json.JSONDecodeError, TypeError):
            source = 'someone'
        
        # Simple template responses
        canned_responses = {
            'eat': f"Sorry {source}, I really need to eat!",
            'drink': f"Excuse me {source}, I'm very thirsty!",
            'rest': f"Sorry {source}, I need to rest!"
        }
        response = canned_responses.get(goal_type, f"Sorry {source}, I'm busy right now!")
        
        self.send_text_input(source, response)
        logger.info(f'🏃 {self.character_name} auto-responded to {source} (physiological priority: {goal_type})')

    def send_text_input(self, target_character: str, message: str):
        """Send text input to another character."""
        try:
            # Create sense data format
            sense_data = {
                'timestamp': datetime.now().isoformat(),
                'sequence_id': 0,
                'mode': 'text',
                'content': json.dumps({
                    'source': self.character_name,
                    'text': message
                })
            }
            
            # Publish directly to target character's sense_data topic
            target_publisher = self.session.declare_publisher(f"cognitive/{target_character}/sense_data")
            target_publisher.put(json.dumps(sense_data))
            
            logger.info(f'📤 {self.character_name} Sent text input to {target_character}: "{message}" (source: {self.character_name})')
            
        except Exception as e:
            logger.error(f'Error sending text input to {target_character}: {e}')

    def _publish_goal(self, goal):
        """Publish current goal to the goal topic for UI display."""
        if not goal:
            return
        try:
            goal_data = {
                'goal': goal.to_string(),
                'timestamp': datetime.now().isoformat(),
                'character': self.character_name
            }
            
            self.goal_publisher.put(json.dumps(goal_data))
            logger.info(f'🎯 Published goal for {self.character_name}: {goal.to_string()}')
            
        except Exception as e:
            logger.error(f'Error publishing goal: {e}')

    def _publish_decided_action(self, action):
        """Publish decided action to the decided_action topic for UI display."""
        try:
            # Prefer displaying a bound variable value if target references a plan variable
            raw_target = action.get('target', '')
            display_target = raw_target
            try:
                if isinstance(raw_target, str) and raw_target.startswith('$'):
                    var_name = raw_target[1:]
                    if var_name in self.plan_bindings:
                        display_target = self.plan_bindings[var_name]
            except Exception:
                pass

            decided_action_data = {
                'decided_action': f"{action['type']}: {display_target} - {action.get('value', '')}",
                'type': action['type'],
                'target': display_target,
                'requested_target': raw_target,
                'value': action.get('value', ''),
                'timestamp': datetime.now().isoformat(),
                'character': self.character_name
            }
            
            self.decided_action_publisher.put(json.dumps(decided_action_data))
            
        except Exception as e:
            logger.error(f'Error publishing decided action: {e}')

    def _publish_current_plan(self):
        """Publish current plan to the current_plan topic for UI display."""
        try:
            current_plan_data = {
                'current_plan': json.dumps(self.current_plan, indent=2) if self.current_plan else '',
                'plan_data': self.current_plan,
                'timestamp': datetime.now().isoformat(),
                'character': self.character_name
            }
            
            self.current_plan_publisher.put(json.dumps(current_plan_data))
            logger.info(f'📋 Published current plan for {self.character_name}')
            
            # Log plan_bindings if they exist
            if self.plan_bindings:
                logger.info(f'🔗 {self.character_name} current plan_bindings: {self.plan_bindings}')
            
        except Exception as e:
            logger.error(f'Error publishing current plan: {e}')

    def _publish_current_activity(self):
        """Publish current activity to the current_activity topic for UI display."""
        try:
            # Get current activity from activity manager if available
            current_activity = None
            current_step = None
            activity_state = None
            
            if self.activity_manager:
                current_activity = self.activity_manager.current_activity
                activity_state = self.activity_manager.current_activity_state
                if current_activity and activity_state:
                    current_step = self.activity_manager.activity_step()
            
                current_activity_data = {
                    'current_activity': json.dumps(current_activity, indent=2) if current_activity else '',
                    'activity_data': current_activity,
                    'current_step': current_step,
                    'activity_state': activity_state,
                    'timestamp': datetime.now().isoformat(),
                    'character': self.character_name
                }
                
                self.current_activity_publisher.put(json.dumps(current_activity_data, default=datetime_handler))
                logger.info(f'🎯 Published current activity for {self.character_name}')
            
        except Exception as e:
            logger.error(f'Error publishing current activity: {e}')

    def publish_time_proposal(self, proposed_minutes):
        """Publish time advancement proposal to map_node."""
        try:
            proposal_data = {
                'character_name': self.character_name,
                'proposed_minutes': proposed_minutes,
                'timestamp': datetime.now().isoformat()
            }
            
            self.time_proposal_publisher.put(json.dumps(proposal_data))
            logger.info(f'⏰ {self.character_name} proposed time advance: {proposed_minutes} minutes')
            
        except Exception as e:
            logger.error(f'Error publishing time proposal: {e}')

    def _ooda_housekeeping(self):
        try:
            if self.current_time is not None:
                # Initialize last update on first tick
                if self.last_state_update_time is None:
                    self.last_state_update_time = self.current_time
                else:
                    dt_minutes = max(0.0, (self.current_time - self.last_state_update_time).total_seconds() / 60.0)
                    if dt_minutes > 0 and self.manual is False:
                        tick_state(self.self_state, dt_minutes)
                        self.last_state_update_time = self.current_time
                        # Log cap condition
                        hunger_val = float(self.self_state.get('hunger', {}).get('value', 0.0))
                        if hunger_val >= 95.0:
                            logger.error(f'🍽️ {self.character_name} hunger reached 95 (worst)')
                        fatigue_val = float(self.self_state.get('fatigue', {}).get('value', 0.0))
                        if fatigue_val >= 95.0:
                            logger.error(f'🍽️ {self.character_name} fatigue reached 95 (worst)')
                        thirst_val = float(self.self_state.get('thirst', {}).get('value', 0.0))
                        state_payload = {
                            'character': self.character_name,
                            'timestamp': datetime.now().isoformat(),
                            'state': self.self_state
                        }
                        self.current_state_publisher.put(json.dumps(state_payload))
                        state_payload = {
                            'character': self.character_name,
                            'timestamp': datetime.now().isoformat(),
                            'state': self.self_state
                        }
                        self.current_state_publisher.put(json.dumps(state_payload))
        except Exception:
            # Non-fatal; continue OODA
            pass

    def check_physiological_overrides(self):
        physiological_drives = {"thirst": """Maintain hydration — avoid thirst and keep the body supplied with fluids.""", 
            "fatigue":"""Maintain alertness — avoid fatigue and keep the body rested.""", 
            "hunger":"""Maintain energy — avoid hunger and keep the body supplied with nutrients."""}
        if self.self_state:
            worst_state_value = 0
            worst_state_key = None
            for key, item in self.self_state.items():
                if item['value'] > 75:
                    logger.warning(f'{self.character_name} is in danger of death')
                    if item['value'] > worst_state_value:
                        worst_state_value = item['value']
                        worst_state_key = key
            if worst_state_key and self.current_goal and worst_state_key not in self.current_goal.name:
                logger.warning(f'{self.character_name} is in danger of death, overriding current goal with physiological override: {physiological_drives[worst_state_key]}')
                self.current_goal = plan_module.Goal(physiological_drives[worst_state_key], actors=[self.character_name], termination=f'{worst_state_key} below 50')
                if self.activity_manager:
                    self.activity_manager._complete_activity()
                self.current_activity = None
                self.current_step = None
                self._plan_completed(reason='physiological override')
                return self.current_goal, worst_state_key
            return None, worst_state_key
        return None, None


    def _process_text_input(self):
        """Process one queued text input."""
        sense_data = self.text_input_queue.pop(0)
        content = sense_data['content']
        try:
            content_data = json.loads(content)
            text_input = content_data.get('text', '')
            source = content_data.get('source', 'unknown')
        except (json.JSONDecodeError, TypeError):
            text_input = content
            source = 'console'
        
        if text_input and text_input.strip():
            clean_input = text_input.strip().strip('"').strip("'")
            
            # Handle special commands from User BEFORE processing as dialog
            if source == 'User':
                if clean_input.startswith('goal:'):
                    logger.info(f'📥 {self.character_name} Received goal command from User: "{clean_input}"')
                    self.parse_and_set_goal(clean_input)
                    return  # Don't process as speech
                elif clean_input.startswith('plan:'):
                    logger.info(f'📥 {self.character_name} Received plan command from User: "{clean_input}"')
                    self.parse_and_set_plan(clean_input)
                    return  # Don't process as speech
                elif clean_input.startswith('edit:'):
                    logger.info(f'📥 {self.character_name} Received edit command from User: "{clean_input}"')
                    self.parse_and_edit_plan(clean_input)
                    return  # Don't process as speech
                elif clean_input.startswith('test:'):
                    logger.info(f'📥 {self.character_name} Received test command from User: "{clean_input}"')
                    if self.is_infospace and self.infospace_executor:
                        if 'primitives' in clean_input.lower():
                            self.infospace_executor.test_primitives()
                        else:
                            logger.warning(f'Unknown test command: {clean_input}')
                    else:
                        logger.warning(f'Test command only available in infospace mode')
                    return  # Don't process as speech
            
            # Normal dialog processing
            logger.info(f'📥 {self.character_name} Processing text input: "{text_input}" (source: {source})')
            self.generate_speech(text_input, source, mode='respond')
            self.plan_just_generated = True  # Skip action execution this turn
        
    def _run_ooda_loop(self):
        """Execute the OODA loop: Observe, Orient, Decide, Act."""
        try:
            # Clear plan generation flag from previous turn
            plan_was_just_generated = self.plan_just_generated
            self.plan_just_generated = False
            
            # Tick local physiology/state using global simulation time if available
            self._ooda_housekeeping()
            # Request fresh situation update for UI
            self.map_update_request_publisher.put(json.dumps({'type': 'ooda observe'}))
            time.sleep(0.1)
            # Observe: Collect current situation and sense data
            observations = self._observe()
            # Manual characters skip orient/plan entirely
            if self.manual:
                if self.current_plan and not plan_was_just_generated:
                    action, action_succeeded = self._plan_step(self.current_plan)
                    if action is not None and not action_succeeded:
                        # Action failed, plan index already advanced, will try next step next turn
                        logger.info(f'Manual action failed for {self.character_name}')
                        return
                return
            
            # If awaiting user input but turn advanced, user is signaling to resume autonomous behavior
            if self.awaiting_user_input:
                logger.info(f'▶️ {self.character_name} resuming autonomous behavior (user clicked step/run)')
                self.awaiting_user_input = False
                # Clear any lingering UI goal/plan state to ensure fresh autonomous selection
                self.current_goal = None
                self.current_plan = None
                self.goal_source = None
            
            # Check for active activity and get current step
            previous_step = self.current_step
            previous_goal = self.current_goal
            previous_plan = self.current_plan

            if self.current_plan and not plan_was_just_generated:
                action, status = self._plan_step(self.current_plan)
                if action is not None:
                    return
                # Check if plan completion set awaiting_user_input flag
                if self.awaiting_user_input:
                    logger.info(f'⏸️ {self.character_name} awaiting user input after plan completion')
                    return
            
            # If plan was just generated, skip autonomous behavior this turn
            if plan_was_just_generated and self.current_plan:
                logger.info(f'📋 {self.character_name} plan ready for execution (next turn)')
                return
                
            # Physiological overrides are now checked at top level (line ~358) before entering OODA loop
            
            if self.activity_manager and self.activity_manager.has_active_activity():
                should_continue, reason = self.activity_manager.continue_activity()
                if should_continue:
                    self.current_step = self.activity_manager.activity_step()  # Get current step without advancing
                    logger.info(f'🎯 {self.character_name} continuing activity: {self.current_activity["name"]} step: {self.current_step["name"]}')
                    self.current_goal = plan_module.Goal(self.current_step['name'], self.current_step['actors'], self.current_step['description'], self.current_step['termination'])
                    logger.info(f'🎯 {self.character_name} oriented to goal: {self.current_goal.to_string()}')
                    action, status = self.plan_and_initiate_execution(self.current_goal, activity_manager=self.activity_manager)
                    return
                else:
                    # Abandon current activity - this will cause next block to select new activity
                    self.activity_manager._complete_activity()
                    self.current_step = None
                    self.current_activity = None
                    self.current_goal = None
                    self.current_plan = None


            # No active activity or current one was abandoned - select new one
            if self.activity_manager and not self.activity_manager.has_active_activity():
                self.current_step, self.current_activity = self.activity_manager.select_activity()
                if self.current_activity:
                    self.current_goal = None
                self._publish_current_activity()
                if self.current_step and self.current_activity:
                    logger.info(f'🎯 {self.character_name} starting new activity: {self.current_activity["name"]}')
                else:
                    logger.warning(f'🚫 {self.character_name} no activity selected')

            if self.current_step:
                self.current_goal = self._orient(observations, step_rewrite=True)
                self.goal_source = 'autonomous'
                self._publish_goal(self.current_goal)
                logger.info(f'🎯 {self.character_name} oriented to goal: {self.current_goal.to_string()}')
                action, status = self.plan_and_initiate_execution(self.current_goal, activity_manager=self.activity_manager)
                return

            self.current_goal = self._orient(observations, step_rewrite=False)
            if self.current_goal:
                self.goal_source = 'autonomous'
                self._publish_goal(self.current_goal)
                logger.info(f'🎯 {self.character_name} oriented to goal: {self.current_goal.to_string()}')
                action, status = self.plan_and_initiate_execution(self.current_goal)
                return

        except Exception as e:
            logger.error(f'Error in OODA loop: {e}')
            logger.error(traceback.format_exc())
        return


    def plan_and_initiate_execution(self, goal, activity_manager=None):
        """Plan and execute a goal."""
        # Plan: Return existing plan or create single-action plan
        plan = self._plan(self.current_goal)
        self.current_plan = plan
        if activity_manager:
            activity_manager.current_plan = plan
        if not plan:
            logger.error(f'🚫 {self.character_name} no plan found for goal: {self.current_goal.to_string()}')
            return None, False

        # Plan Step: Execute current step of plan
        action, action_succeeded = self._plan_step(self.current_plan)
        if not action_succeeded:
            # Action failed (e.g., conversation lock unavailable)
            logger.warning(f'Action {action} failed for {self.character_name}')
        return action, action_succeeded

    def format_situation(self):
        """Format the situation data for the LLM."""
        formatted_situation = ''
        if self.last_situation_data and self.last_situation_data.get('location'):
            formatted_situation += f"\n#You are at location: {self.last_situation_data['location']}, in terrain: {self.last_situation_data['views'][0]['terrain']}, on property type: {self.last_situation_data['views'][0]['property']}\n"
        if self.last_situation_data and self.last_situation_data.get('visible_characters'):
            formatted_situation += f"\n#You can see {len(self.last_situation_data['visible_characters'])} people: {', and '.join(self.last_situation_data['visible_characters'])}\n"
        if self.last_situation_data and self.last_situation_data.get('look'):
            formatted_situation += f"\n#You can see the following:\n\t{'\n\t'.join(self.last_situation_data['look'])}\n"
        
        # Add adjacent information
        if self.last_situation_data and self.last_situation_data.get('adjacent_to'):
            adjacent = self.last_situation_data['adjacent_to']
            if adjacent.get('resources'):
                formatted_situation += f"\n#You are adjacent to these resources (available to take, inspect, or use): {', '.join(adjacent['resources'])}\n"
            if adjacent.get('characters'):
                formatted_situation += f"\n#You are adjacent to these characters (available to interact with): {', '.join(adjacent['characters'])}\n"

        if self.last_situation_data and self.last_situation_data.get('characters'):
            for character_name in self.last_situation_data['characters']:
                #entity_context = self.get_entity_context(character_name, 10)
                #if entity_context:
                formatted_situation += f"\n#You can see {character_name}"#, with whom you have had the following conversation history:\n"
                    #for memory in entity_context['conversation_history']: 
                        #formatted_situation += f"\n\t{memory['source']}: {memory['text']}"
                    #formatted_situation += '\n'

        if self.last_situation_data and self.last_situation_data.get('views'):
            compact_views = format_views_compact(self.last_situation_data['views'])
            if compact_views:
                formatted_situation += f"\n#You can see the following:\n" + compact_views

        try:
            for reply in self.session.get("cognitive/map/world_state", target=QueryTarget.BEST_MATCHING, consolidation=ConsolidationMode.NONE, timeout=4.0 if not self.debug else 300.0):
                if reply.ok:
                    data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                    world_state = data.get('world_state', '')
                    if world_state:
                        formatted_situation += f"\n#World state: \n{world_state}\n"
                    break
        except Exception as e:
            logger.error(f'Error querying world state in format_situation: {e}')

        if self.inspections:
            formatted_situation += f"\n#You have inspected the following:\n"
            for target, inspection in self.inspections.items():
                if target:
                    formatted_situation += f"\n\t{target}: {inspection}"
            formatted_situation += '\n'

        if self.uses:
            formatted_situation += f"\n#You have used the following:\n"
            for target, use in self.uses.items():
                if target:
                    formatted_situation += f"\n\t{target}: {use}"
            formatted_situation += '\n'

        if self.plan_summary:
            formatted_situation += f"\n#The summary of your most recent plan before the current one was:\n{self.plan_summary}\n"

        return formatted_situation

    def _update_system_prompt(self):
        """Update the system prompt with the current situation."""
        system_prompt = self.observations['static']
        if self.activity_manager and self.activity_manager.has_active_activity():
            system_prompt += f"\n#Your current hi-level activity is:\n\t{self.activity_manager.current_activity.get('name')}\n"
        if self.current_goal:
            system_prompt += f"\n#Your current goal is:\n\t{self.current_goal.to_string()}\n"
        if self.current_plan:
            system_prompt += f"\n#Your current plan is:\n\t{json.dumps(self.current_plan, indent=2)}\n"
        if self.action_history:
            last_action = self.action_history[-1]
            system_prompt += f"\n#Your last action was:\n\t{last_action.action.get('type')}: {last_action.action.get('target')} - {last_action.result}\n"
        return system_prompt

    def _observe(self):
        """Observe: Collect current situation and sense data. Stub."""
        system_prompt = ''
        user_prompt = ''
        try:
            if self.character_config.get('character', None):
                system_prompt = self.character_config['character']
            if self.character_config.get('drives', None):
                system_prompt += f"\n#Your drives are:\n\t{'\n\t'.join(self.character_config['drives'])}\n"
            if not self.map_types:
                try:
                    for reply in self.session.get("cognitive/map/types", target=QueryTarget.BEST_MATCHING, consolidation=ConsolidationMode.NONE, timeout=2.0 if not self.debug else 300.0):
                        if reply.ok:
                            data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                            if data.get('success'):
                                self.map_types = data
                                break
                except Exception as e:
                    logger.error(f'Error querying map types in _build_user_prompt: {e}')
            if self.map_types:
                system_prompt += f"\n#The following are the types primary types in the scenario:\n{format_map_types(self.map_types)}"
            # Build user prompt with context
            user_prompt += self.format_situation()
            # Include current self state (e.g., hunger)
            try:
                if hasattr(self, 'self_state') and isinstance(self.self_state, dict) and self.self_state:
                    user_prompt += self.format_current_physiological_state()
            except Exception:
                pass
            entity_context = None
            entity_context = self.get_entity_context(self.character_name, 10)
            if entity_context:
                user_prompt += f'\n#Your most recent thoughts include:'
                for i, memory in enumerate(entity_context['conversation_history']):  # Use last 2 memories
                    user_prompt += f"\n\t{memory['source']}: {memory['text']}"
                user_prompt += '\n'
            # get inventory and cache exact ids
            inventory = []
            try:
                for reply in self.session.get(f"cognitive/{self.character_name}/memory/inventory", target=QueryTarget.BEST_MATCHING, consolidation=ConsolidationMode.NONE, timeout=2.0 if not self.debug else 300.0):
                    if reply.ok:
                        data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                        if data.get('success'):
                            value = data.get('value', [])
                            if isinstance(value, list):
                                inventory.extend(value)
                            elif value:
                                inventory.append(value)
            except Exception as e:
                logger.error(f'Error querying inventory in _build_user_prompt: {e}')
            # update local cache
            try:
                self.inventory_cache = [str(v) for v in inventory]
            except Exception:
                self.inventory_cache = []
            if inventory:
                user_prompt += f'\n#Your inventory includes:'
                for item in inventory:
                    user_prompt += f"\n\t{item}"
            user_prompt += '\n'
            if self.action_history:
                last_action = self.action_history[-1]
                user_prompt += f'\n#Your last action was:'
                user_prompt += f"\n\t{last_action.action.get('type')}: {last_action.action.get('target')} - {last_action.result}\n"

            self.observations = {'static': system_prompt, 'dynamic': user_prompt}
            return self.observations
        except Exception as e:
            logger.error(f'Error in _observe: {e}')  
            traceback.print_exc()
            self.observations = {'static': system_prompt, 'dynamic': user_prompt}
            return self.observations

    def format_current_physiological_state(self):
        """Format the current physiological state for the LLM."""
        def_str = '\nPHYSIOLOGICAL STATE DEFINITIONS:\n'+'+\n'.join(str(need) for need in PHYSIOLOGICAL_STATES)
        return def_str+'\n'+'CURRENT_STATE:\n'+'\n'.join([f'{key.capitalize()}: {item["value"]:.1f}' for key, item in self.self_state.items()])

    def _orient(self, observations: Dict[str, Any], step_rewrite: bool = False):
        """Orient: Assess current state and drives"""

        # Create a new goal based on current situation
        other_characters_str = ''
        for other_character_name, other_character_desc in self.character_config.get('characters', {}).items():
            if other_character_name != self.character_name:
                other_characters_str += f"\n\t{other_character_name}: {other_character_desc}"
        character_names = list(self.character_config.get("characters", {}).keys())

        self.map_update_request_publisher.put(json.dumps({'type': 'goal_look'}))
        map_types_str = format_map_types(self.map_types)
        if self.ontology:
            ontology_nouns_str = f"#ABSTRACT NOUNS:\n{'\n'.join(self.ontology['nouns'])}\n"
            ontology_verbs_str = f"#ABSTRACT VERBS:\n{'\n'.join(self.ontology['verbs'])}\n"
        else:
            ontology_nouns_str = ''
            ontology_verbs_str = ''
        character_drives = self.drives
        character_drives_str = '\n'.join(character_drives)   

        drives = character_drives.copy()
        physiological_drives = ["""Maintain hydration — avoid thirst and keep the body supplied with fluids.""", 
        """Maintain alertness — avoid fatigue and keep the body rested.""", 
        """Maintain energy — avoid hunger and keep the body supplied with nutrients."""]
        drives.extend(physiological_drives)
        character_drives_str = '\n'.join(self.drives)   

        # Make LLM call
        try:
            # Use shorter timeout during shutdown
            timeout = 100.0 if self.shutdown_requested else None
            response = self.llm_client.generate(
                messages=[GOAL_TEMPLATE if not step_rewrite else REWRITE_TEMPLATE],
                bindings={
                    "physiological_states": self.format_current_physiological_state(),
                    "character_drives": character_drives_str,
                    "primitive_nouns": map_types_str,
                    "primitive_verbs": PLAN_VERBS,
                    "abstract_nouns": ontology_nouns_str,
                    "abstract_verbs": ontology_verbs_str,
                    "character_names": '\n'.join(character_names),
                    "other_characters": other_characters_str,
                    "static_information": self.observations['static'],
                    "current_information": self.observations['dynamic'],
                    "current_percepts": self.percepts_at_plan,
                    "activity_name": self.current_activity.get('name', '') if self.current_activity else '',
                    "activity_steps": self.current_activity.get('steps', '') if self.current_activity else '',
                    "step_to_rewrite": self.current_step if self.current_step else '',
                },
                max_tokens=400,
                temperature=0.5,
                stops=['</end>'],
                timeout=timeout
            )

            if response.success:
                logger.info(f'🤖 {self.character_name} New Goal: {response.text.strip()}')
                goals = []
                forms = hash_utils.findall_forms(response.text)
                if len(forms) == 0:
                    logger.error(f'No goal found in LLM response: {response.text}')
                    self.current_goal = plan_module.Goal('sleep', actors=[self.character_name])
                    self._publish_goal(self.current_goal)
                    return self.current_goal
                for goal_hash in forms:
                    goal = plan_module.validate_and_create_goal(self.character_name, goal_hash)
                    if goal:
                        logger.info(f'{self.character_name} generated goal: {goal.to_string()}')
                        self.current_goal = goal
                        self.current_plan = None  # Clear plan so _plan creates new one for this goal
                        self.plan_bindings = {}  # Clear scan variables for new plan
                        logger.info(f'🔄 {self.character_name} cleared plan_bindings for new plan')
                        self.plan_bindings_cache = {}
                        self._publish_goal(goal)
                        return self.current_goal
                    else:
                        logger.error(f'Warning: Invalid goal generation response for {goal_hash}')
            else:
                logger.error(f'LLM call failed: {response.error}')
                self.current_plan = None
                self.plan_bindings = {}  # Clear scan variables for new plan
                self.current_goal = plan_module.Goal('sleep', actors=[self.character_name])
                self._publish_goal(self.current_goal)

            if not self.current_goal:
                logger.error(f'No goal generated for {self.character_name}')
        except Exception as e:
            logger.error(f'Error in _orient: {e}')
            traceback.print_exc()
            self.current_goal = plan_module.Goal('sleep', actors=[self.character_name])
            self._publish_goal(self.current_goal)
        return self.current_goal


    def _plan(self, goal: plan_module.Goal):
        """Plan: Return existing plan or create single-action plan from goal."""
        # If we already have a plan, return it
        if self.current_plan is not None:
            logger.info(f'🎯 {self.character_name} continuing existing plan')
            return self.current_plan
        if not self.current_goal or self.current_goal.name == 'sleep':
            single_action = None
        else:
            system_prompt = self._update_system_prompt()
            user_prompt = self.observations['dynamic']
            goal_prompt = f"\n\nYour current goal is: {goal.to_string()}"
            target = goal.actors[1] if len(goal.actors) > 1 else None
            if target:
                entity_context = self.get_entity_context(target, 10)
                if entity_context and len(entity_context['conversation_history']) > 0:
                    goal_prompt += f'your recent dialog with {target} has been:\n'
                    for i, memory in enumerate(entity_context['conversation_history']):  
                        if isinstance(memory, dict) and 'source' in memory and 'text' in memory: # Use last 2 memories
                            goal_prompt += f"\t{memory['source']}: {memory['text']}\n"
                        else:
                            logger.error(f'Invalid memory format in conversation history: {memory}')
                            goal_prompt += f"\t{memory}\n"

            if self.action_history and (self.action_history[-1].action['type'].lower() == 'say' or self.action_history[-1].action['type'].lower() == 'response'):
                directive = f"""\nrespond only with the JSON plan, no other text.\nend your response with </end>"""
            else:
                directive = f"""\nrespond only with the JSON plan, no other text.\nend your response with </end>"""

            self.current_plan_prompt = system_prompt + user_prompt + goal_prompt
            # Capture percepts at plan start (before LLM call)
            try:
                self.percepts_at_plan = self._capture_percepts_at_plan()
            except Exception:
                self.percepts_at_plan = []
            
            # Clear plan state for infospace
            if self.is_infospace and self.infospace_executor:
                self.infospace_executor.clear_plan_state()
            
            # Use unified planner
            if not self.shutdown_requested:
                goal_text = goal.name + (': ' + goal.description if goal.description != goal.name else '')
                
                # Build context
                if self.is_infospace:
                    context = {
                        'variables': self.infospace_executor.plan_bindings if self.infospace_executor else {},
                        'situation': user_prompt
                    }
                else:
                    context = {
                        'character': self,
                        'current_activity': self.current_activity,
                        'situation': user_prompt,
                        'percepts': None,
                        'server_name': os.getenv('CWB_LLM_SERVER', 'vllm'),
                        'model_name': os.getenv('CWB_LLM_MODEL', 'llama3.3-70B-instruct')
                    }
                
                plan_candidate = self.planner.generate_plan(goal=goal_text, context=context)
            else:
                plan_candidate = None

            if plan_candidate and isinstance(plan_candidate, dict):
                logger.debug(f'🤖 {self.character_name} New Plan candidate: {json.dumps(plan_candidate)}')
                valid = False
                try:
                    # Validate with appropriate validator
                    if self.is_infospace:
                        import infospace_planner
                        valid = infospace_planner.verify_plan(plan_candidate)
                    else:
                        valid = plan_module.verify_plan(plan_candidate)
                    
                    if not valid:
                        logger.error(f'Invalid plan JSON from planner')
                        plan_candidate = None
                    else:
                        logger.debug(f'📋 {self.character_name} generated new plan: {json.dumps(plan_candidate, indent=2)}')
                except Exception as e:
                    logger.error(f'Invalid plan structure from planner: {e}')
                    plan_candidate = None
                if not plan_candidate or not plan_candidate.get('plan') or len(plan_candidate['plan']) == 0:
                    logger.error('No action, target, or value found in planner output')
                    single_action = {'type': 'sleep', 'target': 'self', 'value': '', 'reason': ''}
                else:
                    self.current_plan = plan_candidate
                    self.plan_bindings = {}  # Clear scan variables for new plan
                    logger.info(f'🔄 {self.character_name} cleared plan_bindings for new plan')
                    logger.info(f'📋 {self.character_name} assigned LLM-generated plan with {len(plan_candidate["plan"])} steps')
                    self.plan_bindings_cache = {}
                    self.plan_summary_completed = False  # Reset for new plan
                    # Initialize plan identifiers and control-flow events
                    self.plan_counter += 1
                    self.current_plan_id = f"p_{self.plan_counter}"
                    self.control_flow_events = []
                    self.step_counter = 0
                    # Capture simulation time at plan start
                    try:
                        for time_reply in self.session.get("cognitive/map/simulation_time", target=QueryTarget.BEST_MATCHING, consolidation=ConsolidationMode.NONE, timeout=2.0 if not self.debug else 300.0):
                            if time_reply.ok:
                                time_data = json.loads(time_reply.ok.payload.to_bytes().decode('utf-8'))
                                if time_data.get('success'):
                                    self.current_plan_start_sim_iso = time_data.get('time_info', {}).get('datetime')
                                    break
                    except Exception:
                        logger.error(traceback.format_exc())
                        self.current_plan_start_sim_iso = None
                    self._publish_current_plan()
                    self.plan_state = {'step_stack': plan_module.Stack()}
                    return self.current_plan
            else:
                logger.error('Planner call failed or returned no plan')
                single_action = {'type': 'sleep', 'target': 'self', 'value': '', 'reason': ''}

        # Create single-action plan
        if single_action:
            self.current_plan = {'plan': [{'type': single_action['type'], 'target': single_action['target'], 'value': single_action['value'], 'reason': single_action.get('reason', '')}]}   
            self.plan_bindings = {}  # Clear scan variables for new plan
            logger.info(f'🔄 {self.character_name} cleared plan_bindings for single-action plan')
            logger.info(f'📋 {self.character_name} created single-action plan: {single_action["type"]}')
            # Clear goal_source for fallback plans - they shouldn't trigger UI goal completion
            if single_action['type'] == 'sleep' and self.goal_source == 'ui':
                logger.warning(f'⚠️ {self.character_name} plan generation failed for UI goal, clearing goal_source to allow retry')
                self.goal_source = None
        self.plan_summary_completed = False  # Reset for new plan
        # Initialize plan identifiers and control-flow events
        self.plan_counter += 1
        self.current_plan_id = f"p_{self.plan_counter}"
        self.control_flow_events = []
        self.step_counter = 0
        # Capture percepts at plan start
        try:
            self.percepts_at_plan = self._capture_percepts_at_plan()
        except Exception:
            logger.error(traceback.format_exc())
            self.percepts_at_plan = None
        # Capture simulation time at plan start (single-action plans)
        try:
            for time_reply in self.session.get("cognitive/map/simulation_time", target=QueryTarget.BEST_MATCHING, consolidation=ConsolidationMode.NONE, timeout=2.0 if not self.debug else 300.0):
                if time_reply.ok:
                    time_data = json.loads(time_reply.ok.payload.to_bytes().decode('utf-8'))
                    if time_data.get('success'):
                        self.current_plan_start_sim_iso = time_data.get('time_info', {}).get('datetime')
                        break
        except Exception:
            logger.error(traceback.format_exc())
            self.current_plan_start_sim_iso = None
        self._publish_current_plan()
        # Initialize plan state for new plan
        self.plan_state = {'step_stack': plan_module.Stack()}
        self.plan_bindings_cache = {}      
        return self.current_plan

    def _replan(self, goal, reason):
        """RePlan: Update an existing plan based on new input."""
        logger.warning(f'🤖 {self.character_name} Replanning.')
        if not self.current_plan:
            return self._plan(goal)
        if self.current_plan and self.plan_state:
            remaining_plan = json.dumps(plan_module.remaining_plan(self.current_plan, self.plan_state['step_stack']), indent=2)
        else:
            remaining_plan = 'None'
        system_prompt = """Your task is to consider replacing your current goal and your remaining current plan, in light of the following recent conversation.
"""     
        system_prompt += self._update_system_prompt()
        user_prompt = self.observations['dynamic']
        goal_prompt = f"\n\n#Your current goal is:\n{goal.to_string() if goal else 'None'}"
        target = goal.actors[1] if goal and len(goal.actors) > 1 else None
        if target:
            entity_context = self.get_entity_context(target, 10)
            if entity_context and len(entity_context['conversation_history']) > 0:
                goal_prompt += f'your recent dialog with {target} has been:\n'
                for i, memory in enumerate(entity_context['conversation_history']):  # Use last 2 memories
                    goal_prompt += f"\t{memory['source']}: {memory['text']}\n"

                goal_prompt += "Don't repeat yourself.\n"

        directive = f"""\n#Your current goal is:
{goal.to_string()}

#Your current remaining plan is:
{remaining_plan}

#The new input causing you to reconsider your goal and plan is:
{reason}

Consider whether to abandon your current goal and replace it with a new goal.
Is this conversation a good reason to abandon your current goal? 
I.E. does it present a new issue / opportunity / obligation demanding the character's attention?
If so, is your current goal important enought to be merged into the new goal?
Nothing in this or other instructions limits your use of deception or surprise.
                  
Respond with either the word 'current' or the word 'new' followed by a newline.
if you choose a new goal, specify it using the following hash-formatted text, where each tag is preceded by a # and followed by a single space, followed by its content.
Each goal should begin with a #goal tag, and should end with ## on a separate line as shown below:
be careful to insert line breaks only where shown, separating a value from the next tag:

#goal terse (5-8) words) name for this goal
#description concise (8-14) words) further details of this goal
#otherCharacterName name of the other actor involved in this goal, or None if no other actor is involved
#termination terse (5-6 words) statement of condition that would mark achievement or partial achievement of this goal. This should be a specific observable condition that can be checked for.
##

Respond ONLY with the above keyword 'current' or the keyword 'new' followed by a newline and the above hash-formatted text.
end your response with </end>
"""

        # Make LLM call
        try:
            response = self.llm_client.generate(
                messages=[system_prompt, user_prompt, PLAN_TEMPLATE, directive],
                max_tokens=300,
                temperature=0.7,
                stops=['</end>']
            )

            if response.success:
                logger.info(f'🤖 {self.character_name} New Goal: {response.text.strip()}')
                goals = []
                if response.text.strip().lower() == 'current':
                    return self.current_goal
                forms = hash_utils.findall_forms(response.text)
                for goal_hash in forms:
                    goal = plan_module.validate_and_create_goal(self.character_name, goal_hash)
                    if goal:
                        logger.info(f'{self.character_name} generated goal: {goal.to_string()}')
                        self.current_goal = goal
                        self.current_plan = None  # Clear plan so _plan creates new one for this goal
                        self.plan_bindings = {}  # Clear scan variables for new plan
                        logger.info(f'🔄 {self.character_name} cleared plan_bindings for replan')
                        self._publish_goal(goal)
                        return self.current_goal
                    else:
                        logger.error(f'Warning: Invalid goal generation response for {goal_hash}')
            else:
                logger.error(f'LLM call failed: {response.error}')
                self.current_goal = plan_module.Goal('sleep', actors=[self.character_name])
                self.current_plan = None  # Clear plan so _plan creates new one for this goal
                logger.info(f'🔄 {self.character_name} cleared plan_bindings for sleep goal (replan failed)')
                self._publish_goal(self.current_goal)
        except Exception as e:
            logger.error(f'Error in LLM processing: {e}')
            logger.error(traceback.format_exc())
            self.current_goal = plan_module.Goal('sleep', actors=[self.character_name])
            self.current_plan = None  # Clear plan so _plan creates new one for this goal
            logger.info(f'🔄 {self.character_name} cleared plan_bindings for sleep goal (LLM unavailable)')
            self._publish_goal(self.current_goal)
        return self.current_goal
        
        
    def _plan_completed(self, reason='plan completed'):
        """Handle successful plan completion."""
        # Existing telemetry and cleanup
        if reason == 'plan completed':
            self._summarize_plan_execution()
        completed_plan = self.current_plan
        self.current_plan = None
        self.plan_bindings = {}
        self.plan_bindings_cache = {}
        self.action_history = []
        self.plan_state = None
        if reason == 'physiological override' or reason == 'manual goal override' or reason == 'manual plan override':
            return
        
        # Handle activity advancement if this plan is for the activity - assume a single plan per activity step
        if self.activity_manager and self.activity_manager.has_active_activity() and self.activity_manager.current_plan == completed_plan:
            # Advance to next activity step
            next_step, activity = self.activity_manager.step_completion('success')
            self._publish_current_activity()
            self.current_goal = None
            if next_step:
                logger.info(f'🎯 Activity step completed, advancing to: {next_step["name"]}')
            else:
                # Activity completed - clear goal
                self._publish_goal(self.current_goal)
                self.current_activity = None
                logger.info(f'✅ Activity completed for {self.character_name}')
        else:
            # No activity - clear goal (existing behavior)
            self.current_goal = None
            self._publish_goal(self.current_goal)
        
        # If goal was from UI, pause autonomous behavior to await next user input
        if self.goal_source == 'ui':
            self.awaiting_user_input = True
            logger.info(f'⏸️ {self.character_name} UI goal completed, awaiting user input')
        self.goal_source = None

        self._publish_current_plan()
        

    def _plan_step(self, plan):
        """Execute current step of plan and return next action using frame-based stack.
        Invariant ?"""
        # Extract plan steps from dict format
        if isinstance(plan, dict) and 'plan' in plan:
            plan_steps = plan['plan']
        else:
            plan_steps = plan
            
        if not plan_steps or len(plan_steps) == 0:
            self.current_plan = None
            self._publish_current_plan()
            logger.error(f'🚫 {self.character_name} no plan steps found')
            return None, False # how did we get here?
        
        step_stack = self.plan_state['step_stack']

        # Guardrails: abort if any state dimension exceeds threshold (Phase 1: 95)
        try:
            if hasattr(self, 'self_state') and isinstance(self.self_state, dict):
                for key, data in self.self_state.items():
                    val = float(data.get('value', 0.0))
                    if val >= 95.0:
                        logger.error(f'🚫 {self.character_name} aborting plan due to state threshold: {key}={val:.1f}')
                        # Finalize current plan and publish plan log
                        self._plan_completed()
                        # Publish stop to map turn controller and UI reflects via turn_status
                        self.session.put("cognitive/map/turn/stop", json.dumps({"character": self.character_name, "reason": "state_threshold", "metric": key, "value": val}).encode())
                        # Clear plan/activity state
                        self.current_plan = None
                        self._publish_current_plan()
                        # Terminate current activity if present
                        if self.activity_manager and self.activity_manager.has_active_activity():
                            self.activity_manager.step_completion('failure', 'state_threshold')
                            self._publish_current_activity()
                        return None, False
        except Exception:
            pass
        
        # Initialize stack if empty with main plan frame
        if step_stack.is_empty():
            main_frame = {
                'plan': plan_steps,
                'idx': 0,
                'type': 'main'
            }
            step_stack.push(main_frame)
        
        # Execute next step
        try:
            action = self._execute_next_step(step_stack)
            action_succeeded = False
            if action:
                self.current_action = action
                self._publish_decided_action(action)
                action_succeeded = self._act(action)
                if not action_succeeded:
                    logger.info(f'Action failed for {self.character_name}: {action}')
            return action, action_succeeded
        except Exception as e:
            logger.error(f'Error in plan execution: {e}')
            traceback.print_exc()
            # Clear plan state on error
            self.plan_state = {'step_stack': plan_module.Stack()}
            self._plan_completed()
            return None, False
    
    def _execute_next_step(self, step_stack):
        """Execute next step using frame-based stack."""
        if step_stack.is_empty():
            logger.error('Step stack is empty')
            return None

        def _cond_outcome(cond):
            if self.is_infospace:
                outcome = self.infospace_executor._evaluate_condition(cond)
                return outcome, None
            else:
                result = plan_module._evaluate_condition(self, cond, self.observations)
                return result['value'], result['binding'] if result['binding'] else None

        current = step_stack.peek()
        plan = current['plan']
        idx = current['idx']

        # ---- Completed current frame? ----
        if idx >= len(plan):
            logger.info(f'🤖 {self.character_name} completed current frame {current["type"]}')
            if current['type'] == 'while':
                # Finished one body iteration
                current['iteration_count'] += 1

                # Hard stop guard
                if current['iteration_count'] >= current['max_iterations']:
                    step_stack.pop()
                    if step_stack.is_empty():
                        self._plan_completed()
                        return None
                    parent = step_stack.peek()
                    parent['idx'] = current['return_to']
                    return self._execute_next_step(step_stack)

                # Re-evaluate loop condition (pre-test for next iter)
                outcome, resolved = _cond_outcome(current['condition'])

                if outcome:
                    current['idx'] = 0  # next iteration
                    return self._execute_next_step(step_stack)
                else:
                    # Exit loop
                    step_stack.pop()
                    if step_stack.is_empty():
                        self._plan_completed()
                        return None
                    parent = step_stack.peek()
                    parent['idx'] = current['return_to']
                    return self._execute_next_step(step_stack)

            # Generic frame finished (main/if_then/if_else)
            step_stack.pop()
            if step_stack.is_empty():
                self._plan_completed()
                return None
            parent = step_stack.peek()
            parent['idx'] = current.get('return_to', parent['idx'])
            return self._execute_next_step(step_stack)

        # ---- Still within current frame ----
        step = plan[idx]
        stype = step.get('type')
        
        logger.debug(f'🔄 {self.character_name} executing plan step {idx + 1}/{len(plan)}: {stype} action')

        # Primitive actions (spec-compliant + infospace primitives)
        executable_primitives = {
            # Physical world primitives
            'move', 'say', 'think', 'take', 'place', 'inspect', 'use', 'scan',
            # Infospace primitives - Phase 1 & 2
            'apply', 'display', 'createNote', 'createCollection', 'persist', 'load', 'index', 'organize', 'search',
            'extract', 'filter', 'merge', 'transform',
            'aggregate', 'sort', 'group_by', 'compare', 'map', 'flatten', 'add', 'expand'
        }
        if stype in executable_primitives:
            current['idx'] = idx + 1
            return step
        
        # Check if step type is a tool (direct tool invocation)
        if self.is_infospace and stype in self.available_tools:
            # Convert tool invocation to apply format for execution
            tool_step = {
                'type': 'apply',
                'target': stype,
                'value': step.get('value'),
                'args': step.get('args', {}),
                'out': step.get('out'),
                'reason': step.get('reason', '')
            }
            current['idx'] = idx + 1
            return tool_step

        elif stype == 'while':
            body = step.get('body', [])
            if not isinstance(body, list):
                # Malformed; skip this step
                logger.error('While body must be a list; skipping.')
                current['idx'] = idx + 1
                return self._execute_next_step(step_stack)

            cond = step.get('condition')
            outcome, resolved = _cond_outcome(cond)

            if outcome:
                while_frame = {
                    'plan': body,
                    'idx': 0,
                    'type': 'while',
                    'condition': cond,
                    'return_to': idx + 1,
                    'iteration_count': 0,
                    'max_iterations': 15
                }
                step_stack.push(while_frame)
                return self._execute_next_step(step_stack)
            else:
                current['idx'] = idx + 1
                return self._execute_next_step(step_stack)

        elif stype == 'if':
            cond = step.get('condition')
            then_body = step.get('then')
            else_body = step.get('else')

            if not isinstance(then_body, list) and else_body is None:
                # Malformed if (no then and no else): skip
                logger.error('If step missing valid then/else; skipping.')
                current['idx'] = idx + 1
                return self._execute_next_step(step_stack)

            outcome, resolved = _cond_outcome(cond)
            branch = 'then' if outcome else ('else' if else_body is not None else None)

            if branch:
                step_stack.push({
                    'plan': step[branch],
                    'idx': 0,
                    'type': 'if_then' if branch == 'then' else 'if_else',
                    'return_to': idx + 1
                })
            else:
                current['idx'] = idx + 1
            return self._execute_next_step(step_stack)

        elif stype == 'wait':
            # Evaluate wait condition; if true, advance; else publish turn complete and re-evaluate next turn.
            cond = step.get('condition')
            outcome, resolved = _cond_outcome(cond if isinstance(cond, dict) else {'type': str(cond), 'target': ''})
            wait_state = current.setdefault('wait_state', {'turns': 0})
            if outcome:
                logger.info(f"⏳ wait condition satisfied; advancing")
                current['idx'] = idx + 1
                return self._execute_next_step(step_stack)
            else:
                wait_state['turns'] += 1
                logger.info(f"⏳ wait turn {wait_state['turns']}/15; publishing turn complete")
                # Publish turn completion and do not advance index
                try:
                    turn_data = {
                        'character': self.character_name,
                        'timestamp': datetime.now().isoformat(),
                        'type': 'turn_complete'
                    }
                    self.turn_complete_publisher.put(json.dumps(turn_data))
                except Exception:
                    pass
                # Hard cap to avoid indefinite wait
                if wait_state['turns'] >= 15:
                    logger.warning("⏳ wait exceeded 15 turns; advancing with warning")
                    current['idx'] = idx + 1
                return self._execute_next_step(step_stack)

        elif stype == 'sleep':
            # Evaluate wait condition; if true, advance; else publish turn complete and re-evaluate next turn.
            logger.info(f"slept for 1 minute; advancing")
            current['idx'] = idx + 1
            return self._execute_next_step(step_stack)

        else:
            # Unknown or non-executable type: skip
            logger.error(f'Unknown or non-executable step type: {stype}')
            current['idx'] = idx + 1
            return self._execute_next_step(step_stack)
    

    def _act(self, action: Dict[str, Any]) -> bool:
        """Act: Execute the chosen action. Returns True if action succeeded, False if it failed."""

        # Publish time advancement proposal for testing TBD - condition on type
        ACTION_TIME_COSTS = {
                            "move": 1,
                            "scan": .5,
                            "inspect": 0.75,
                            "think": 0.75,
                            "take": 5,
                            "use": 5,
                            "say": 1,
                            "wait": 5,  # use explicit duration
                            }

        logger.info (f'🎯 {self.character_name} performing action: {str(action)}')
        if action['type'] in ACTION_TIME_COSTS:
            proposed_minutes = ACTION_TIME_COSTS[action['type']]
        else:
            proposed_minutes = random.randint(2, 5)
        self.publish_time_proposal(proposed_minutes)

        action = self.current_action if self.current_action else action
        
        # Route to infospace executor if in infospace mode
        if self.is_infospace:
            action_type = action.get('type', '').lower()
            # Infospace primitives that route to infospace executor
            infospace_primitives = {
                # Core primitives
                'apply', 'map', 'transform', 'move',
                'createnote', 'createcollection', 'persist', 'load',
                'index', 'organize', 'search',
                # Data operations
                'flatten', 'add', 'expand',
                # Communication
                'say', 'display', 'think'
            }
            
            # Route infospace-specific actions to infospace executor
            if action_type in infospace_primitives:
                logger.info(f'🧩 Routing {action_type} to infospace executor')
                result = self.infospace_executor.execute_action(action)
                
                # Create action record for tracking
                now_ts = datetime.now()
                self.step_counter += 1
                action_record = ActionRecord(
                    action=action,
                    result=result.get('value', '') if result.get('status') == 'success' else result.get('reason', 'failed'),
                    timestamp=now_ts,
                    step_id=self.step_counter,
                    plan_id=self.current_plan_id,
                    requested_target=action.get('target', ''),
                    started_at=now_ts,
                    ended_at=datetime.now()
                )
                action_record.proposed_minutes = proposed_minutes
                action_record.outcome_status = result.get('status', 'unknown')
                self._snapshot_physiology(action_record)
                self.action_history.append(action_record)
                
                # Publish action for UI display
                result_value = str(result.get('value', '')) if result.get('value') else ''
                display_value = result_value if self.is_infospace else result_value[:200]
                
                action_data = {
                    'type': action_type,
                    'action_id': self.action_counter,
                    'timestamp': now_ts.isoformat(),
                    'target': action.get('target', ''),
                    'out': action.get('out', ''),
                    'status': result.get('status', 'unknown'),
                    'value': display_value
                }
                
                # Add 'text' field for say/display/think actions (memory_node expects this)
                if action_type in ('say', 'display', 'think'):
                    action_data['text'] = display_value
                    action_data['source'] = self.character_name
                
                self.action_publisher.put(json.dumps(action_data))
                self.action_counter += 1
                
                # Return success/failure based on result
                return result.get('status') == 'success'
            
            # say and think work in both spaces, fall through to existing implementation
        
        # Continue with existing physical world execution
        # ...
        
        # Create action record with basic telemetry
        now_ts = datetime.now()
        self.step_counter += 1
        action_record = ActionRecord(
            action=action,
            result=None,  # Will be set below
            timestamp=now_ts,
            step_id=self.step_counter,
            plan_id=self.current_plan_id,
            requested_target=action.get('target', ''),
            started_at=now_ts
        )
        # Record proposed minutes for this step (used in plan metrics)
        try:
            action_record.proposed_minutes = int(proposed_minutes)
        except Exception:
            action_record.proposed_minutes = None
        self.action_history.append(action_record)
        
        if action['type'].lower() == "sleep":
            action_data = {'type': 'sleep','action_id': self.action_counter,'timestamp': datetime.now().isoformat(),'target': self.character_name}
            self.action_publisher.put(json.dumps(action_data))
            action_record.result = 'slept'
            self._snapshot_physiology(action_record)
            action_record.outcome_status = 'success'
            action_record.failure_code = None
            try:
                for time_reply in self.session.get("cognitive/map/simulation_time", target=QueryTarget.BEST_MATCHING, consolidation=ConsolidationMode.NONE, timeout=2.0 if not self.debug else 300.0):
                    if time_reply.ok:
                        time_data = json.loads(time_reply.ok.payload.to_bytes().decode('utf-8'))
                        if time_data.get('success'):
                            action_record.simulation_time = time_data.get('time_info', {}).get('datetime')
                            break
            except Exception:
                logger.error(traceback.format_exc())
            time.sleep(1)  # Sleep for 1 second
            return True
        if action['type'].lower() == 'think':
            thought = self.think_about(action)
            self._snapshot_physiology(action_record)
            action_record.outcome_status = 'success'
            action_record.failure_code = None
            return True
            
        if action['type'].lower() == "move":
            action_type = action['type'].lower()
            if action_type == "move":
                move_direction = None
                # Dereference variable targets (e.g., $var -> bound value)
                raw_target = deref_plan_target(self.plan_bindings, action.get('target', ''))
                if raw_target and isinstance(raw_target, str) and raw_target.startswith('$'):
                    logger.error(f'Plan aborted: unbound variable {raw_target}')
                    action_record.outcome_status = 'failure'
                    action_record.failure_code = 'unbound_variable'
                    self._plan_completed(reason='unbound_variable')
                    return False
                move_target = raw_target.strip().lower()
                # Telemetry: store resolved target string if available
                try:
                    action_record.resolved_target = raw_target
                except Exception:
                    pass
                # Step 1: Test for compass points first (case insensitive)
                cardinal_directions = ['north', 'northeast', 'southeast', 'south', 'southwest', 'northwest', 'east', 'west']
                if move_target in cardinal_directions:
                    move_direction = move_target
                else:
                    cond_action = action.copy()
                    cond_action['type'] = 'can_see'
                    cond_action['target'] = move_target.capitalize()
                    binding = plan_module._evaluate_condition(self, cond_action, self.observations)
                    outcome = binding['value']
                    resolved = binding['binding']
                    if outcome:
                        move_direction = self._find_target_direction(resolved)
                    else:
                        move_direction = None
                # Publish move action for UI display
                action_data = {
                    'type': 'move',
                    'action_id': self.action_counter,
                    'timestamp': datetime.now().isoformat(),
                    'target': raw_target,
                    'status': 'success' if move_direction else 'failed',
                    'error': 'target not visible/resolvable' if not move_direction else None
                }
                self.action_publisher.put(json.dumps(action_data))
                self.action_counter += 1
                
                if move_direction:
                    self.move(move_direction)
                    # Telemetry snapshot after action
                    self._snapshot_physiology(action_record)
                    action_record.outcome_status = 'success'
                    action_record.failure_code = None
                else:
                    action_record.outcome_status = 'failure'
                    action_record.failure_code = 'target_not_visible or 0 distance'
                # Request situation/map update for UI immediately after move
                try:
                    self.map_update_request_publisher.put(json.dumps({'type': 'step_look'}))
                except Exception:
                    pass
                return True
            else:
                logger.error(f'❌ Cannot move toward "{move_target}" - target not resolved or visible, choosing random direction')
                self.move(random.choice(cardinal_directions))
                self.map_update_request_publisher.put(json.dumps({'type': 'step_look'}))
                self._snapshot_physiology(action_record)
                action_record.outcome_status = 'success'
                action_record.failure_code = None
                return True
        elif action['type'].lower() == "say":
            # Check if we can acquire conversation lock
            # Capture the originally requested target for UI/reporting
            requested_target = action.get('target', '')
            raw_target = deref_plan_target(self.plan_bindings, requested_target)
            resolved_target = deref_plan_target(self.plan_bindings, raw_target)
            
            if resolved_target and isinstance(resolved_target, str) and resolved_target.startswith('$'):
                logger.error(f'Plan aborted: unbound variable {resolved_target}')
                action_record.outcome_status = 'failure'
                action_record.failure_code = 'unbound_variable'
                self._plan_completed(reason='unbound_variable')
                return False

            # Telemetry: store resolved target
            try:
                action_record.resolved_target = resolved_target if resolved_target else ''
            except Exception:
                pass
            if resolved_target and self._acquire_conversation_lock(resolved_target):
                self.generate_speech(action['value'], resolved_target, mode='say')
                self._snapshot_physiology(action_record)
                action_record.outcome_status = 'success'
                action_record.failure_code = None
            else:
                # Lock acquisition failed, publish failure action
                action_data = {
                    'type': 'say',
                    'action_id': self.action_counter,
                    'timestamp': datetime.now().isoformat(),
                    'target': resolved_target if resolved_target else '',                    
                    'requested_target': requested_target,
                    'resolved_target': resolved_target if resolved_target else '',
                    'source': self.character_name,
                    'text': action.get('value', ''),
                    'status': 'failed',
                    'error': 'conversation lock unavailable'
                }
                self.action_publisher.put(json.dumps(action_data))
                logger.error('📤 Published action: say (failed - lock unavailable)')
                # Telemetry failure tagging
                action_record.outcome_status = 'failure'
                action_record.failure_code = 'lock:conversation_unavailable'
                try:
                    action_record.result = 'failed - conversation lock unavailable'
                    self._snapshot_physiology(action_record)
                except Exception:
                    pass
                self.action_counter += 1
                return False
        elif action['type'].lower() == "take":
            # If resolution failed, publish a failure action with requested_target
            cond_action = action.copy()
            cond_action['target'] = deref_plan_target(self.plan_bindings, cond_action.get('target', ''))
            
            if cond_action['target'] and isinstance(cond_action['target'], str) and cond_action['target'].startswith('$'):
                logger.error(f'Plan aborted: unbound variable {cond_action["target"]}')
                action_record.outcome_status = 'failure'
                action_record.failure_code = 'unbound_variable'
                self._plan_completed(reason='unbound_variable')
                return False
            
            cond_action['type'] = 'near'
            cond_action['target'] = cond_action['target'].strip().capitalize()
            binding = plan_module._evaluate_condition(self, cond_action, self.observations)
            outcome = binding['value']
            resolved = binding['binding']   
            # Telemetry: store resolved target
            try:
                action_record.resolved_target = resolved if resolved else ''
            except Exception:
                pass
            # Disallow infrastructure for take
            try:
                if isinstance(resolved, str) and resolved.startswith('Path'):
                    action_data = {
                        'type': 'take',
                        'action_id': self.action_counter,
                        'timestamp': datetime.now().isoformat(),
                        'target': '',
                        'requested_target': action.get('target', ''),
                        'resolved_target': resolved,
                        'status': 'failed',
                        'error': 'illegal for infrastructure'
                    }
                    self.action_publisher.put(json.dumps(action_data))
                    action_record.outcome_status = 'failure'
                    action_record.failure_code = 'illegal:infrastructure'
                    if self.action_history:
                        self.action_history[-1].result = 'failed - illegal on infrastructure'
                    self.action_counter += 1
                    return False
            except Exception:
                pass
            if not outcome:
                action_data = {
                    'type': 'take',
                    'action_id': self.action_counter,
                    'timestamp': datetime.now().isoformat(),
                    'target': '',
                    'requested_target': action['target'],
                    'resolved_target': '',
                    'status': 'failed',
                    'error': 'target not nearby/visible'
                }
                self.action_publisher.put(json.dumps(action_data))
                logger.error('📤 Published action: take (failed)')
                # Telemetry failure tagging
                action_record.outcome_status = 'failure'
                action_record.failure_code = 'precondition:near'
                # Record a concise failure result for summaries
                if self.action_history:
                    self.action_history[-1].result = 'failed - target not nearby/visible'
                try:
                    self._snapshot_physiology(action_record)
                except Exception:
                    pass
                self.action_counter += 1
                return False
            ok = self.take(action, resolved)
            # Set final outcome string for summaries
            try:
                if self.action_history:
                    self.action_history[-1].result = f'took {resolved}' if ok else 'take failed'
            except Exception:
                pass
            try:
                self._snapshot_physiology(action_record)
            except Exception:
                pass
            if ok:
                action_record.outcome_status = 'success'
                action_record.failure_code = None
            else:
                action_record.outcome_status = 'failure'
                action_record.failure_code = 'execution:failed'
            self.action_counter += 1
        elif action['type'].lower() == "place":
            # Precondition: must have item in inventory (has_item)
            cond_action = action.copy()
            # Dereference variable targets
            try:
                rt = cond_action.get('target', '')
                if isinstance(rt, str) and rt.startswith('$'):
                    vn = rt[1:]
                    if vn in self.plan_bindings:
                        cond_action['target'] = self.plan_bindings[vn]
            except Exception:
                pass
            # Disallow infrastructure for place
            try:
                tname = cond_action.get('target', '')
                if isinstance(tname, str) and tname.strip().capitalize().startswith('Path'):
                    action_data = {
                        'type': 'place',
                        'action_id': self.action_counter,
                        'timestamp': datetime.now().isoformat(),
                        'target': '',
                        'requested_target': action.get('target', ''),
                        'resolved_target': '',
                        'status': 'failed',
                        'error': 'illegal for infrastructure'
                    }
                    self.action_publisher.put(json.dumps(action_data))
                    action_record.outcome_status = 'failure'
                    action_record.failure_code = 'illegal:infrastructure'
                    if self.action_history:
                        self.action_history[-1].result = 'failed - illegal on infrastructure'
                    self.action_counter += 1
                    return False
            except Exception:
                pass
            cond_action['type'] = 'has_item'
            cond_action['target'] = cond_action['target'].strip().capitalize()
            binding = plan_module._evaluate_condition(self, cond_action, self.observations)
            outcome = binding['value']
            resolved = binding['binding']
            # Telemetry: store resolved target
            try:
                action_record.resolved_target = resolved if resolved else ''
            except Exception:
                pass
            action_data = {
                'type': 'place',
                'action_id': self.action_counter,
                'timestamp': datetime.now().isoformat(),
                'target': resolved if resolved else '',
                'requested_target': action.get('target', ''),
                'resolved_target': resolved if resolved else ''
            }
            if not outcome or not resolved:
                action_data['status'] = 'failed'
                action_data['error'] = 'item not in inventory'
                self.action_publisher.put(json.dumps(action_data))
                # Telemetry failure tagging
                action_record.outcome_status = 'failure'
                action_record.failure_code = 'precondition:has_item'
                # Record a concise failure result for summaries
                if self.action_history:
                    self.action_history[-1].result = 'failed - item not in inventory'
                try:
                    self._snapshot_physiology(action_record)
                except Exception:
                    pass
                self.action_counter += 1
                return False
            # Perform place on map
            ok = self.place(action, resolved)
            # Telemetry snapshot after action
            if ok:
                try:
                    action_record.hunger_after = float(self.self_state.get('hunger', {}).get('value', 0.0))
                    action_record.fatigue_after = float(self.self_state.get('fatigue', {}).get('value', 0.0))
                    action_record.thirst_after = float(self.self_state.get('thirst', {}).get('value', 0.0))
                except Exception:
                    action_record.thirst_after = None
            if ok:
                action_record.outcome_status = 'success'
                action_record.failure_code = None
            else:
                action_record.outcome_status = 'failure'
                action_record.failure_code = 'execution:failed'
            self.action_counter += 1
            return ok
        elif action['type'].lower() == "inspect":
            # Publish inspect action including requested vs resolved target
            cond_action = action.copy()
            cond_action['target'] = deref_plan_target(self.plan_bindings, cond_action.get('target', ''))
            
            if cond_action['target'] and isinstance(cond_action['target'], str) and cond_action['target'].startswith('$'):
                logger.error(f'Plan aborted: unbound variable {cond_action["target"]}')
                action_record.outcome_status = 'failure'
                action_record.failure_code = 'unbound_variable'
                self._plan_completed(reason='unbound_variable')
                return False
            
            cond_action['type'] = 'near'
            cond_action['target'] = cond_action['target'].strip().capitalize()
            binding = plan_module._evaluate_condition(self, cond_action, self.observations)
            outcome = binding['value']
            resolved = binding['binding']
            # Telemetry: store resolved target
            try:
                action_record.resolved_target = resolved if resolved else ''
            except Exception:
                pass

            action_data = {
                'type': 'inspect',
                'action_id': self.action_counter,
                'timestamp': datetime.now().isoformat(),
                'target': resolved if resolved else '',
                'requested_target': action['target'],
                'resolved_target': resolved if resolved else ''
            }
            if not resolved or not outcome:
                action_data['status'] = 'failed'
                action_data['error'] = 'target not nearby/visible'
                self.action_publisher.put(json.dumps(action_data))
                # Telemetry failure tagging
                action_record.outcome_status = 'failure'
                action_record.failure_code = 'precondition:near'
                # Record a concise failure result for summaries
                if self.action_history:
                    self.action_history[-1].result = 'failed - target not nearby/visible'
                try:
                    self._snapshot_physiology(action_record)
                except Exception:
                    pass
            else:
                # Perform inspect which may populate last_action_result
                self.inspect(action, resolved)
                # Telemetry snapshot after action
                try:
                    action_record.hunger_after = float(self.self_state.get('hunger', {}).get('value', 0.0))
                    action_record.fatigue_after = float(self.self_state.get('fatigue', {}).get('value', 0.0))
                    action_record.thirst_after = float(self.self_state.get('thirst', {}).get('value', 0.0))
                except Exception:
                    action_record.thirst_after = None
                # Publish result if available so UI can display it
                try:
                    result_text = action_record.result if isinstance(action_record.result, str) else ''
                    if result_text and result_text.strip():
                        result_action = {
                            'type': 'inspect',
                            'action_id': f"inspect_{int(time.time())}",
                            'timestamp': datetime.now().isoformat(),
                            'target': resolved,
                            'llm_response': result_text
                        }
                        self.action_publisher.put(json.dumps(result_action))
                except Exception:
                    pass
                action_record.outcome_status = 'success'
                action_record.failure_code = None
        elif action['type'].lower() == "use":
            # Create action - include requested vs resolved
            cond_action = action.copy()
            # Dereference variable targets
            try:
                rt = cond_action.get('target', '')
                if isinstance(rt, str) and rt.startswith('$'):
                    vn = rt[1:]
                    if vn in self.plan_bindings:
                        cond_action['target'] = self.plan_bindings[vn]
            except Exception:
                pass
            
            if cond_action.get('target', '') and isinstance(cond_action['target'], str) and cond_action['target'].startswith('$'):
                logger.error(f'Plan aborted: unbound variable {cond_action["target"]}')
                action_record.outcome_status = 'failure'
                action_record.failure_code = 'unbound_variable'
                self._plan_completed(reason='unbound_variable')
                return False
            
            cond_action['type'] = 'near'
            cond_action['target'] = cond_action['target'].strip().capitalize()
            binding = plan_module._evaluate_condition(self, cond_action, self.observations)
            outcome = binding['value']
            resolved = binding['binding']
            # Telemetry: store resolved target
            try:
                action_record.resolved_target = resolved if resolved else ''
            except Exception:
                pass
            if not outcome:
                cond_action['type'] = 'has_item'
                binding = plan_module._evaluate_condition(self, cond_action, self.observations)
                outcome = binding['value']
                resolved = binding['binding']

            action_data = {
                'type': 'use',
                'action_id': self.action_counter,
                'timestamp': datetime.now().isoformat(),
                'target': resolved if resolved else '',
                'requested_target': action['target'],
                'resolved_target': resolved if resolved else ''
            }
            if not outcome or not resolved:
                action_data['status'] = 'failed'
                action_data['error'] = 'target not nearby/visible'
                # Telemetry failure tagging (choose specific reason if known)
                action_record.outcome_status = 'failure'
                action_record.failure_code = 'precondition:near' if binding and not binding.get('value') else 'precondition:has_item'
                # Record a concise failure result for summaries
                if self.action_history:
                    self.action_history[-1].result = (
                        'failed - target not nearby/visible'
                        if (binding and not binding.get('value')) else 'failed - item not in inventory'
                    )
                try:
                    self._snapshot_physiology(action_record)
                except Exception:
                    pass
                return False
            else:
                # Infrastructure: invoke map use_path to traverse
                if isinstance(resolved, str) and resolved.startswith('Path'):
                    try:
                        payload = json.dumps({'path_id': resolved}).encode('utf-8')
                    except Exception:
                        payload = None
                    try:
                        for reply in self.session.get(f"cognitive/map/agent/{self.character_name}/use_path", target=QueryTarget.BEST_MATCHING, consolidation=ConsolidationMode.NONE, payload=payload, timeout=5.0 if not self.debug else 300.0):
                            if reply.ok:
                                data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                                ok = data.get('success', False)
                                if ok:
                                    self.action_history[-1].result = f'traversed {resolved}'
                                    action_data['status'] = 'success'
                                else:
                                    self.action_history[-1].result = 'use path failed'
                                    action_data['status'] = 'failed'
                                break
                    except Exception as e:
                        logger.error(f'Error querying use_path: {e}')
                    self.action_publisher.put(json.dumps(action_data))
                    if action_data.get('status') == 'success':
                        action_record.outcome_status = 'success'
                        action_record.failure_code = None
                    else:
                        action_record.outcome_status = 'failure'
                        action_record.failure_code = 'execution:failed'
                    try:
                        self._snapshot_physiology(action_record)
                    except Exception:
                        pass
                    self.action_counter += 1
                    return True
                # Resource/other use path
                self.use(action, resolved, action_data)
                action_data['result'] = self.action_history[-1].result
                action_record.outcome_status = 'success'
                action_record.failure_code = None
                # Apply rules-driven physiological effects based on resource type
                try:
                    if isinstance(resolved, str) and resolved:
                        # Query resource rules using instance name (handler strips numeric suffix)
                        rules_response = None
                        try:
                            for reply in self.session.get(f"cognitive/map/resource_rules/{resolved}", target=QueryTarget.BEST_MATCHING, consolidation=ConsolidationMode.NONE, timeout=3.0 if not self.debug else 300.0):
                                if reply.ok:
                                    rules_response = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                                    break
                        except Exception as e:
                            logger.error(f'Error querying resource_rules: {e}')
                        if rules_response and rules_response.get('success'):
                            rr = rules_response.get('resource_rules') or {}
                            use_effects = rr.get('use') or []
                            if isinstance(use_effects, list) and use_effects:
                                # Build case-insensitive map of current needs
                                try:
                                    need_key_map = {str(k).lower(): k for k in (self.self_state or {}).keys()}
                                except Exception:
                                    need_key_map = {}
                                for eff in use_effects:
                                    try:
                                        need_name = str(eff.get('need', '')).lower()
                                        delta = float(eff.get('effect', 0.0))
                                        if not need_name:
                                            continue
                                        if need_name not in need_key_map:
                                            logger.error(f"Unknown need in use effects: '{eff.get('need')}'")
                                            continue
                                        key = need_key_map[need_name]
                                        # Read current value
                                        cur = 0.0
                                        try:
                                            cur = float((self.self_state.get(key) or {}).get('value', 0.0))
                                        except Exception:
                                            cur = 0.0
                                        new_val = cur + delta
                                        # Clamp to 0..100
                                        if new_val < 0.0:
                                            new_val = 0.0
                                        elif new_val > 100.0:
                                            new_val = 100.0
                                        # Write back
                                        try:
                                            if key not in self.self_state or not isinstance(self.self_state[key], dict):
                                                self.self_state[key] = {'value': new_val}
                                            else:
                                                self.self_state[key]['value'] = new_val
                                        except Exception:
                                            pass
                                    except Exception:
                                        # Skip malformed effect entries
                                        pass
                except Exception:
                    # Rules lookup/apply errors are non-fatal for the action
                    pass
                # Apply simple hunger/thirst restore heuristics
                try:
                    state_payload = {
                        'character': self.character_name,
                        'timestamp': datetime.now().isoformat(),
                        'state': self.self_state
                    }
                    self.current_state_publisher.put(json.dumps(state_payload))
                except Exception:
                    pass
                # Telemetry snapshot after action (post-use may alter hunger)
                try:
                    action_record.hunger_after = float(self.self_state.get('hunger', {}).get('value', 0.0))
                    action_record.fatigue_after = float(self.self_state.get('fatigue', {}).get('value', 0.0))
                    action_record.thirst_after = float(self.self_state.get('thirst', {}).get('value', 0.0))
                except Exception:
                    action_record.thirst_after = None
            self.action_publisher.put(json.dumps(action_data))
            self.action_counter += 1
            return True
        elif action['type'].lower() == "scan":
            scan_result, distance = self._execute_scan_action(action)
            action_record.result = scan_result
            # Record bindings_after and simple binding evidence
            try:
                var_name = action.get('out', '')
                if var_name and scan_result:
                    action_record.bindings_after = {var_name: scan_result}
                    # If distance is available in situation, include minimal evidence
                    try:
                        # Prefer the cached scan distance if available; fallback to recompute
                        best_dist = distance
                        if best_dist is not None:
                            action_record.binding_evidence = {"criterion":"nearest","distance": best_dist}
                    except Exception:
                        pass
            except Exception:
                pass
            if scan_result:
                action_record.outcome_status = 'success'
                action_record.failure_code = None
            else:
                action_record.outcome_status = 'failure'
                action_record.failure_code = 'execution:failed'
            # Telemetry snapshot after action

            action_record.hunger_after = float(self.self_state.get('hunger', {}).get('value', 0.0))
            action_record.fatigue_after = float(self.self_state.get('fatigue', {}).get('value', 0.0))
            action_record.thirst_after = float(self.self_state.get('thirst', {}).get('value', 0.0))

            
            # Publish scan action result to FastAPI
            action_data = {
                'type': 'scan',
                'action_id': self.action_counter,
                'timestamp': datetime.now().isoformat(),
                'target': action.get('target', ''),
                'out': action.get('out', ''),
                'result': scan_result,
                'status': 'success' if scan_result else 'failed',
                'variable_bound': action.get('out', '') if scan_result else '',
                'bound_value': scan_result if scan_result else ''
            }
            self.action_publisher.put(json.dumps(action_data))
            logger.info(f'Action: scan {action.get("target", "")}: {"success" if scan_result else "failed"}')
            
            # Publish to perception_node
            if scan_result:
                self.perception_action_result_publisher.put(json.dumps({
                    'action': action,
                    'update_text': f'scanned and found {scan_result}',
                    'prediction': action.get('prediction', '')
                }))
            
            self.action_counter += 1
            
            return True if scan_result else False
        else:
            logger.error(f'❌ Unknown action type: {action.get("type", "unknown")}')
            try:
                action_record.result = 'failed - unknown action'
                self._snapshot_physiology(action_record)
            except Exception:
                pass
            action_record.outcome_status = 'failure'
            action_record.failure_code = 'error:unknown_action'
            return False

        # Request situation/map update for UI after non-move actions that may affect visibility/adjacency
        try:
            if action['type'].lower() in ["take", "inspect", "use", "save", "create"]:
                self.map_update_request_publisher.put(json.dumps({'type': 'step_look'}))
        except Exception:
            pass

        # Finalize timing and outcome inference
        action_record.ended_at = datetime.now()
        try:
            action_record.duration_ms = int((action_record.ended_at - action_record.started_at).total_seconds() * 1000)
        except Exception:
            pass

        # Attach a minimal feature snapshot when available (low-risk fields only)
        try:
            snap: Dict[str, Any] = {}
            # If resolved_target/near/visibility can be inferred cheaply, include them
            if action.get('type','').lower() in ['move','take','use','inspect']:
                # Best-effort distance from last_situation_data
                rt = None
                try:
                    rt = (action_record.resolved_target or '').lower()
                except Exception:
                    rt = ''
                if rt:
                    best_dist = None
                    for view in (self.last_situation_data or {}).get('views', []):
                        for key in ('characters','resources'):
                            for ent in view.get(key, []) or []:
                                name = ent.get('name','')
                                if name and name.lower() == rt:
                                    d = ent.get('distance')
                                    if d is not None:
                                        dd = float(d)
                                        if best_dist is None or dd < best_dist:
                                            best_dist = dd
                    if best_dist is not None:
                        snap['distance_to_target'] = best_dist
            if snap:
                action_record.feature_snapshot = snap
        except Exception:
            pass

        logger.info(f'📤 Published action: {action["type"]}')
        self.action_counter += 1
        return True

    def _handle_interrupt(self):
        """Handle interrupt from sense data or situation updates."""
        logger.warning('🔄 Handling interrupt')
        # TODO: Implement interrupt handling logic
        pass
    
    def _complete_turn(self):
        """Complete the current turn and prepare for the next one."""
        logger.info(f'🔄 {self.character_name} completing turn')
        
        # Publish turn completion
        turn_data = {
            'character': self.character_name,
            'timestamp': datetime.now().isoformat(),
            'type': 'turn_complete'
        }
        self.turn_complete_publisher.put(json.dumps(turn_data))
        
        # Wait for next turn
        self.waiting_for_turn = True
    
    def _summarize_plan_execution(self):
        """Summarize the completed plan execution for memory storage."""
        if self.plan_summary_completed:
            logger.debug(f'📝 Plan summary already completed for {self.character_name}')
            return
            
        if not self.action_history:
            logger.debug(f'📝 No actions to summarize for {self.character_name}')
            return
            
        logger.info(f'📝 Creating plan execution summary for {self.character_name}')
        
        # Convert action history to text structure
        actions_text = []
        for record in self.action_history:
            action_type = record.action.get('type', 'unknown')
            target = record.action.get('target', 'unknown')
            result = record.result if record.result else 'no result recorded'
            timestamp = record.timestamp.strftime('%H:%M:%S')
            actions_text.append(f"{timestamp} - {action_type}: {target} -> {result}")
        
        actions_summary = '\n'.join(actions_text)
        
        # Prepare context for LLM summary
        goal_text = self.current_goal.to_string() if self.current_goal else "No specific goal"
        plan_text = json.dumps(self.current_plan, indent=2) if self.current_plan else "No plan available"
        
        # Build compact structured telemetry for selected fields (bindings/evidence/features)
        try:
            telemetry_actions = []
            for ar in self.action_history:
                sel = {
                    'step_id': ar.step_id,
                    'type': (ar.action or {}).get('type', '')
                }
                if getattr(ar, 'bindings_after', None):
                    sel['bindings_after'] = ar.bindings_after
                if getattr(ar, 'binding_evidence', None):
                    sel['binding_evidence'] = ar.binding_evidence
                if getattr(ar, 'feature_snapshot', None):
                    sel['feature_snapshot'] = ar.feature_snapshot
                if len(sel.keys()) > 2:
                    telemetry_actions.append(sel)
        except Exception:
            telemetry_actions = []

        percepts_json = json.dumps(self.percepts_at_plan, indent=2) if self.percepts_at_plan else "[]"
        telemetry_json = json.dumps({"actions": telemetry_actions}, indent=2)

        summary_prompt = """
        #Goal: 
        {{$goal_text}}
        
        #Plan (JSON):
        {{$plan_text}}
        
        #Actions Taken (text):
        {{$actions_summary}}
        
        #Percepts at plan start (JSON):
        {{$percepts_json}}
        
        #Structured telemetry (selected fields; JSON):
        {{$telemetry_json}}
        
        Please provide JSON object with:
        1. 
        2. a JSON formatted assessment of the plan's success or failure in meeting the goal, in the following format:
            {
                "Summary": str "a concise paragraph summarizing this plan execution, including the goal, actions taken, and observed results",
                "How": str "concise (8-10 words) explanation how this plan intended to achieve the goal",
                "outcome": str "concise (20-28 words) explanation of the outcome - did it achieve the goal? If not, where did it fail and why?",
                "plan_score": int (0-100) "did the plan execute as expected?"
                "goal_score": int (0-100) "how well the goal was met as measured by goal termination condition"
            }

        Do not include any other introductory, explanatory, discursive, or formatting text in your response.

        """
        response = self.llm_client.generate([summary_prompt], 
                                            bindings={"goal_text": goal_text, 
                                                    "plan_text": plan_text, 
                                                    "actions_summary": actions_summary, 
                                                    "percepts_json": percepts_json, 
                                                    "telemetry_json": telemetry_json}, 
        max_tokens=500, is_json=True)
        summary = response.text if isinstance(response.text, dict) else None
        self.plan_summary = summary
        if not summary:
            logger.error(f'❌ No summary found in response: {response}')
            summary = {}
        logger.info(f'📝 Plan post-mortem prepared for {self.character_name}\n{self.plan_summary}\n')
        
        # Mark as completed to prevent redundant calls
        self.plan_summary_completed = True
        # Phase 1 metrics aggregation
        try:
            # Hunger series
            hunger_values = [ar.hunger_after for ar in self.action_history if ar.hunger_after is not None]
            hunger_start = hunger_values[0] if hunger_values else float(self.self_state.get('hunger', {}).get('value', 0.0))
            hunger_end = hunger_values[-1] if hunger_values else hunger_start
            hunger_min = min(hunger_values) if hunger_values else hunger_start
            hunger_max = max(hunger_values) if hunger_values else hunger_start
            hunger_delta = hunger_end - hunger_start

            # Fatigue series
            fatigue_values = [ar.fatigue_after for ar in self.action_history if getattr(ar, 'fatigue_after', None) is not None]
            fatigue_start = fatigue_values[0] if fatigue_values else float(self.self_state.get('fatigue', {}).get('value', 0.0))
            fatigue_end = fatigue_values[-1] if fatigue_values else fatigue_start
            fatigue_min = min(fatigue_values) if fatigue_values else fatigue_start
            fatigue_max = max(fatigue_values) if fatigue_values else fatigue_start
            fatigue_delta = fatigue_end - fatigue_start

            # Thirst series
            thirst_values = [ar.thirst_after for ar in self.action_history if getattr(ar, 'thirst_after', None) is not None]
            thirst_start = thirst_values[0] if thirst_values else float(self.self_state.get('thirst', {}).get('value', 0.0))
            thirst_end = thirst_values[-1] if thirst_values else thirst_start
            thirst_min = min(thirst_values) if thirst_values else thirst_start
            thirst_max = max(thirst_values) if thirst_values else thirst_start
            thirst_delta = thirst_end - thirst_start
            # Steps and outcomes
            steps_total = len(self.action_history)
            moves = sum(1 for ar in self.action_history if ar.action.get('type','').lower()=='move')
            takes = sum(1 for ar in self.action_history if ar.action.get('type','').lower()=='take')
            uses = sum(1 for ar in self.action_history if ar.action.get('type','').lower()=='use')
            inspects = sum(1 for ar in self.action_history if ar.action.get('type','').lower()=='inspect')
            failures = sum(1 for ar in self.action_history if getattr(ar, 'outcome_status', None) == 'failure')
            # Time (proposed)
            minutes_advanced = sum((ar.proposed_minutes or 0) for ar in self.action_history)
            # Time (actual via simulation time, if available)
            actual_minutes = None
            try:
                plan_start = self.current_plan_start_sim_iso
                plan_end = None
                try:
                    for time_reply in self.session.get("cognitive/map/simulation_time", target=QueryTarget.BEST_MATCHING, consolidation=ConsolidationMode.NONE, timeout=2.0 if not self.debug else 300.0):
                        if time_reply.ok:
                            time_data = json.loads(time_reply.ok.payload.to_bytes().decode('utf-8'))
                            if time_data.get('success'):
                                plan_end = time_data.get('time_info', {}).get('datetime')
                                break
                except Exception as e:
                    logger.error(f'Error querying simulation_time in plan digest: {e}')
                if plan_start and plan_end:
                    from datetime import datetime as _dt
                    start_dt = _dt.fromisoformat(plan_start)
                    end_dt = _dt.fromisoformat(plan_end)
                    delta = end_dt - start_dt
                    actual_minutes = int(delta.total_seconds() // 60)
            except Exception:
                actual_minutes = None
            # Inventory deltas (best-effort; requires exact ids in logs later if needed)
            items_taken = [ar.action.get('target','') for ar in self.action_history if ar.action.get('type','').lower()=='take']
            items_used = [ar.action.get('target','') for ar in self.action_history if ar.action.get('type','').lower()=='use']

            metrics = {
                'time': {
                    'minutes_advanced': minutes_advanced,
                    'actual_minutes': actual_minutes
                },
                'steps': {
                    'total': steps_total,
                    'moves': moves,
                    'takes': takes,
                    'uses': uses,
                    'inspects': inspects,
                    'failures': failures
                },
                'hunger': {
                    'start': hunger_start,
                    'end': hunger_end,
                    'min': hunger_min,
                    'max': hunger_max,
                    'delta': hunger_delta
                },
                'fatigue': {
                    'start': fatigue_start,
                    'end': fatigue_end,
                    'min': fatigue_min,
                    'max': fatigue_max,
                    'delta': fatigue_delta
                },
                'thirst': {
                    'start': thirst_start,
                    'end': thirst_end,
                    'min': thirst_min,
                    'max': thirst_max,
                    'delta': thirst_delta
                },
                'inventory': {
                    'taken': items_taken,
                    'used': items_used
                }
            }
        except Exception:
            metrics = {}

        # Assess drive fulfillment (stubbed LLM scaffolding)
        try:
            drive_fulfillment = self._assess_drive_fulfillment()
            if isinstance(drive_fulfillment, dict):
                metrics['drive_fulfillment'] = drive_fulfillment
        except Exception:
            pass

        # Add goal satisfaction score to metrics for reflection scoring
        if 'goal_score' in summary and summary['goal_score'] is not None:
            metrics['goal_satisfaction'] = summary['goal_score']/100.0
        else:
            metrics['goal_satisfaction'] = 0.0

        # Add goal satisfaction score to metrics for reflection scoring
        if 'plan_score' in summary:
            metrics['plan_score'] = summary['plan_score']/100.0
        else:
            metrics['plan_score'] = 0.0

        entry = {
            'activity': self.current_activity['name'] if self.current_activity else 'None',
            'goal': self.current_goal.to_string() if self.current_goal else '',
            'prompt': self.current_plan_prompt,
            'plan': self.current_plan,
            'summary': self.plan_summary,
            'actions': [asdict(ar) for ar in self.action_history],
            'percepts_at_plan': self.percepts_at_plan,
            'metrics': metrics
        }
        self.plan_log.append(entry)
        # Publish and persist JSONL
        try:
            self.plan_log_publisher.put(json.dumps(entry))
        except Exception:
            pass
        try:
            os.makedirs('data', exist_ok=True)
            with open(f'data/{self.character_name}-plans.jsonl', 'a') as f:
                f.write(json.dumps(entry, default=datetime_handler) + '\n')
        except Exception as e:
            logger.error(f'Error writing plan log to file: {e}')
        #self.review_planning()
        self.action_history = []
    
    def review_planning(self):
        """Review the planning process and update the plan prompt and bindings."""
        if len(self.plan_log) < 3:
            return
        logger.info(f'📝 Reviewing planning for {self.character_name}')
        system_prompt = f"""Review the following planning information for one or more planning efforts and recommend improvements to the format or content of the PLAN_TEMPLATE.
"""
        
        user_prompt = f"""
#Plan syntax specification:
{PLAN_TEMPLATE}

"""
        for n, item in enumerate(self.plan_log):
            # Build a compact metrics summary per plan (Phase 1)
            m = item.get('metrics', {})
            steps = m.get('steps', {})
            hunger = m.get('hunger', {})
            fatigue = m.get('fatigue', {})
            thirst = m.get('thirst', {})
            time_m = m.get('time', {})
            metrics_summary = f"steps: {steps.get('total', 0)}, failures: {steps.get('failures', 0)}, minutes: {time_m.get('minutes_advanced', 0)}, hunger Δ: {hunger.get('delta', 0)}, fatigue Δ: {fatigue.get('delta', 0)}, thirst Δ: {thirst.get('delta', 0)}"
            user_prompt += f"""

#### Planning effort {n+1}

# Goal the plan was created for: 
{item.get('goal','')}

#Planning prompt:
{item.get('prompt','')}

#Resulting Plan:
{json.dumps(item.get('plan',{}), indent=2)}

#Plan metrics (Phase 1):
{metrics_summary}

#Plan post-mortem summary:
{item.get('summary','')}

"""
        user_prompt += f"""
Respond with an analysis of how the plan syntax and or the planning prompt could be improved to better achieve the goal. For example:
- If the plan syntax is ambiguous or unclear explain what is unclear and how to improve it.
- If an action alternative is missing or could be modified to be more appropriate, explain what is missing and how to improve it. Provide specific instances where possible.
- If the planning prompt contins inadequate information about the character's current situation for planning for the goal, suggest additional information that would improve the planning process.
- If the planning prompt instructions could be improved or re-arranged describe how to improve it.

Identify any other issues with the planning process and recommend improvements.

Provide your response as a list of items with full text descriptions, no other text.
End your response with </end>
"""
        logger.info(f'📝 Planning review for {self.character_name}:\n{system_prompt}\n{user_prompt}')
        response = self.llm.ask(bindings={}, prompt=[SystemMessage(content=system_prompt), UserMessage(content=user_prompt)], 
                                       max_tokens=800, stops=['</end>'], is_json=False)
        logger.info(f'📝 Planning review for {self.character_name}:\n {response}')
        self.plan_log = []
        

    def sense_data_callback(self, sample):
        """Handle incoming sense data."""
        # Check if shutdown has been requested
        if self.shutdown_requested:
            return
            
        try:
            sense_data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            self.last_sense_data = sense_data
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
                self.text_input_queue.append(sense_data)
                logger.info(f'📥 {self.character_name} Queued text input: "{text_input}" (source: {source}, queue size: {len(self.text_input_queue)})')
                if len(self.text_input_queue) > 3:
                    logger.warning(f'⚠️ Text input queue size {len(self.text_input_queue)} > 3, dropping oldest')
                    self.text_input_queue.pop(0)
                
        except Exception as e:
            logger.error(f'Error processing sense data: {e}')
    
    def situation_callback(self, sample):
        """Handle incoming situation data."""
        try:
            situation_data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            logger.info(f'📊 {self.character_name} Received situation update')
            
            # Store situation data for potential use in LLM processing
            self.last_situation_data = situation_data
            
        except Exception as e:
            logger.error(f'Error processing situation data: {e}')
    
    def turn_callback(self, sample):
        """Handle turn "GO" signals."""
        try:
            data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            turn_number = data.get('turn_number', 0)
            logger.info(f'🚦 Turn {turn_number} received by {self.character_name}')
            active_characters = data.get('active_characters', [])
            
            if self.character_name in active_characters:
                if not self.waiting_for_turn:
                    logger.error(f'⚠️ Turn {turn_number} arrived but {self.character_name} still processing turn {self.current_turn_number}!')
                    return
                self.current_turn_number = turn_number
                self.waiting_for_turn = False
                logger.info(f'🚦 Turn {turn_number} started for {self.character_name}')
                
                # Set interrupt flag to start OODA loop
                self.interrupt_pending = True
            else:
                logger.warning(f'Turn {turn_number} started but {self.character_name} not included')
            
        except Exception as e:
            logger.error(f'Error handling turn signal: {e}')
            traceback.print_exc()
    
    def shutdown_callback(self, sample):
        """Handle shutdown command from UI."""
        try:
            logger.warning(f'🔌 {self.character_name} Executive Node received shutdown command')
            self.shutdown_requested = True
        except Exception as e:
            logger.error(f'Error in shutdown callback: {e}')
    
    def time_advanced_callback(self, sample):
        """Handle time advancement notifications from map_node."""
        try:
            data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            new_time_info = data.get('new_time_info', {})
            
            if new_time_info and 'datetime' in new_time_info:
                # Store the current simulation time
                self.current_time = datetime.fromisoformat(new_time_info['datetime'])
                logger.info(f'⏰ {self.character_name} received time update: {self.current_time}')
            else:
                logger.warning(f'Received time_advanced but no valid time_info in data')
                
        except Exception as e:
            logger.error(f'Error in time_advanced callback: {e}')
    
    def _dialog_end_callback(self, sample):
        """Handle end dialog query from other characters."""
        try:
            # Extract the other character name from the JSON payload
            data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            other_name = data.get('other_name', 'unknown')
            if other_name == 'unknown':
                logger.error(f'Error in end dialog callback: no other character name found in payload')
                return
                
            logger.info(f'💬 {self.character_name} received end dialog from {other_name}')
            
            # Release conversation lock with this character
            self._release_conversation_lock(other_name)
            
            entity_context = self.get_entity_context(other_name, 10)
            # Build user prompt with context
            dialog_history = '' 
            if entity_context and isinstance(entity_context, dict):
                conversation_history = entity_context.get('conversation_history', [])
                if isinstance(conversation_history, list):
                    dialog_history += f"Your recent conversation with {other_name} has been:\n"
                    for i, memory in enumerate(conversation_history):
                        if isinstance(memory, dict) and 'source' in memory and 'text' in memory:
                            dialog_history += f"\t{memory['source']}: {memory['text']}\n"

            reason = f'\nDialog end detected with {other_name}, dialog_history:\n{dialog_history}\n'
            # In manual mode, do not replan due to dialog
            if not self.manual and self.current_plan:
                new_goal = self._replan(self.current_goal, reason)
                if new_goal:
                    self._plan(new_goal)
            return
        except Exception as e:
            logger.error(f'Error in end dialog callback: {e}')
    
    def publish_dialog_end(self, source: str):
        """Publish dialog end notification to another character."""
        try:
            key = f"cognitive/{source}/dialog_end"
            payload = json.dumps({'other_name': self.character_name})
            self.session.put(key, payload)
            
            # Release conversation lock with this character
            self._release_conversation_lock(source)
            key = f"cognitive/{source}/memory/close_dialog"
            payload = json.dumps({'entity_name': self.character_name})
            self.session.put(key, payload)
            
        except Exception as e:
            logger.error(f'Error publishing dialog end to {source}: {e}')
 

    def parse_and_set_goal(self, goal_text):
        """Parse goal input from UI and set current goal."""
        try:
            parsed_goal = goal_text.strip().strip('"').strip("'")[6:]
            
            # Immediately clear existing plan/activity to interrupt execution
            self.current_plan = None
            self.current_activity = None
            self.plan_state = None
            self.plan_bindings = {}
            self.plan_bindings_cache = {}
            self.goal_source = 'ui'
            self.awaiting_user_input = False
            logger.info(f'🛑 {self.character_name} interrupting existing plan for new goal')
            
            if not self.observations:
                self._observe()
            self.current_goal = plan_module.Goal(parsed_goal, [self.character_name], description='', termination='')
            
            # In infospace mode, skip goal rewriting and plan immediately
            if self.is_infospace:
                self._publish_goal(self.current_goal)
                logger.info(f'🧩 {self.character_name} infospace planning for goal: {parsed_goal}')
                self._plan(self.current_goal)
                if self.current_plan:
                    self._publish_current_plan()
                    self.plan_just_generated = True
                    logger.info(f'📋 {self.character_name} generated plan with {len(self.current_plan["plan"])} steps')
                return
            
            # In manual mode, publish goal and auto-generate plan
            if self.manual:
                self._publish_goal(self.current_goal)
                
                # Auto-generate plan from goal using LLM
                logger.info(f'🤖 {self.character_name} auto-generating plan for manual goal: {parsed_goal}')
                self._plan(self.current_goal)
                
                # Publish the generated plan to UI
                if self.current_plan:
                    self._publish_current_plan()
                    self.plan_just_generated = True
                    logger.info(f'📋 {self.character_name} auto-generated plan with {len(self.current_plan["plan"])} steps')
                else:
                    logger.warning(f'⚠️ {self.character_name} failed to auto-generate plan for goal: {parsed_goal}')
            
            return
        except Exception as e:
            logger.error(f"Goal parsing failed for {self.character_name}: {e}")
            traceback.print_exc()
            return

    def parse_and_set_plan(self, plan_text):
        """Parse JSON plan input from UI and set current plan."""
        try:
            # Parse JSON format
            parsed_plan = plan_module.parse_plan_json(plan_text)
            
            # Validate with appropriate validator
            if self.is_infospace:
                import infospace_planner
                valid = infospace_planner.verify_plan(parsed_plan)
            else:
                valid = plan_module.verify_plan(parsed_plan)
            
            if not valid:
                logger.error(f"Invalid plan for {self.character_name}")
                return
            
            # Immediately clear existing plan/activity to interrupt execution
            self.current_plan = None
            self.current_activity = None
            self.plan_state = None
            self.plan_bindings = {}
            self.plan_bindings_cache = {}
            self.goal_source = 'ui'
            self.awaiting_user_input = False
            logger.info(f'🛑 {self.character_name} interrupting existing plan for new plan')
            
            # Set new plan
            self.current_plan = parsed_plan
            self.current_plan_prompt = "Manual plan via UI"
            logger.info(f'📋 {self.character_name} assigned UI plan with {len(parsed_plan["plan"])} steps')
            self.plan_summary_completed = False
            self._publish_current_plan()
            self.plan_just_generated = True
            self.plan_state = {
                'step_stack': plan_module.Stack()
            }
            
            logger.info(f"📋 {self.character_name} received new plan with {len(parsed_plan['plan'])} steps")
        except Exception as e:
            logger.error(f"Plan parsing failed for {self.character_name}: {e}")
            # Plan assignment failed - character continues with existing behavior

    def parse_and_edit_plan(self, edit_text):
        """Edit existing plan using natural language instruction."""
        try:
            if not self.current_plan:
                logger.warning(f'⚠️ {self.character_name} no current plan to edit')
                return
            
            instruction = edit_text.strip().strip('"').strip("'")[5:].strip()
            if not instruction:
                logger.warning(f'⚠️ {self.character_name} empty edit instruction')
                return
            
            logger.info(f'✏️ {self.character_name} editing plan with instruction: {instruction}')
            
            current_plan_json = json.dumps(self.current_plan, indent=2)
            
            # Build prompt with planning language spec
            if self.is_infospace:
                import infospace_planner
                plan_template = infospace_planner.INFOSPACE_PLAN_TEMPLATE
                
                # Format available tools
                tools_text = ""
                if self.available_tools:
                    tools_text = "\n\nAVAILABLE TOOLS:\n"
                    for tool_name, tool_info in self.available_tools.items():
                        tools_text += f"- {tool_name}: {tool_info.get('description', 'No description')}\n"
                
                prompt = f"""You are editing an infospace plan.

{plan_template}
{tools_text}

INSTRUCTION: {instruction}

CURRENT PLAN:
{current_plan_json}

Return ONLY the edited plan as valid JSON, with no explanation or commentary.

EDITED PLAN:"""
            else:
                prompt = f"""Edit the following plan according to the instruction provided.
Return ONLY valid JSON in the same format, with no explanation or commentary.
Preserve all required fields and use only valid action types.

INSTRUCTION: {instruction}

CURRENT PLAN:
{current_plan_json}

EDITED PLAN (JSON only):"""
            
            response = self.llm_client.generate([prompt], temperature=0.3, is_json=True)
            if not response or not response.success:
                logger.error(f'❌ {self.character_name} LLM failed to edit plan')
                return
            
            # Handle response - may already be parsed dict or JSON string
            if isinstance(response.text, dict):
                edited_plan = response.text
            else:
                edited_plan = json.loads(response.text)
            
            # Validate with appropriate validator
            if self.is_infospace:
                import infospace_planner
                valid = infospace_planner.verify_plan(edited_plan)
            else:
                valid = plan_module.verify_plan(edited_plan)
            
            if not valid:
                logger.error(f'❌ {self.character_name} edited plan validation failed')
                return
            
            # Update plan
            self.current_plan = edited_plan
            self.plan_just_generated = True
            self._publish_current_plan()
            logger.info(f'✅ {self.character_name} plan edited successfully, {len(edited_plan["plan"])} steps')
        except Exception as e:
            logger.error(f"Plan editing failed for {self.character_name}: {e}")
            traceback.print_exc()

    def generate_speech(self, text_input: str, source: str, mode: str = 'say'):
        """In say mode, this is start of conversation. In respond mode, this is a response in an ongoing dialog"""
        if self.character_name == 'User' and mode == 'respond':
            self.publish_dialog_end(source)
            return
        # In manual mode with manual_response disabled, do not auto-respond
        if self.manual and self.manual_response:
            return ''
        text_to_send = 'failure to generate response'
        try:
            if mode == 'respond':
                logger.info(f'Responding to: "{text_input}" from {source}')
            else:
                logger.info(f'Saying intent: "{text_input}" to {source}')
            
            entity_context = self.get_entity_context(source, 10)
            logger.info(f' Retrieved entity context for {source}: {entity_context}')
            
            # Get RAG context for semantic retrieval
            rag_context = self.get_rag_context(text_input, entity_name=source, k=5)
            #logger.info(f' Retrieved RAG context for {source}: {rag_context}')
            #rag_entries = rag_context.get('retrieved_entries', [])
            rag_entries = []
            logger.info(f'🔍 Retrieved {len(rag_entries)} RAG entries for query: "{text_input[:50]}..."')
            
            # Build user prompt with context
            dialog_history = '' 
            if entity_context and isinstance(entity_context, dict):
                conversation = entity_context.get('conversation_history', [])
                if isinstance(conversation, list):
                    dialog_history += f"Your recent conversation with {source} has been:\n"
                    for i, memory in enumerate(conversation):
                        if isinstance(memory, dict) and 'source' in memory and 'text' in memory:
                            dialog_history += f"\t{memory['source']}: {memory['text']}\n"

            if entity_context:
                discourse_tom_models = self.get_entity_discourse_tom_models(source)
                logger.info(f' Retrieved discourse/tom models for {source}: {discourse_tom_models}')
                if discourse_tom_models:
                    dialog_history += f"\nBeliefs about {source}:\n\n{discourse_tom_models.get('tom_model', '')}\n"
                    dialog_history += f"\nDiscourse state with {source}:\n\n{discourse_tom_models.get('discourse_state', '')}\n"
            
            # Add RAG retrieved entries with context
            #if rag_entries:
            #    logger.info(f'RAG: Adding {len(rag_entries[:5])} retrieved conversation snippets')
            #    dialog_history += f"\n\nRelevant past conversations:\n"
            #    for entry in rag_entries[:5]:
            #        expanded = self.expand_rag_context(entry, window_before=3)
            #        dialog_history += f"{expanded}\n---\n"

            # Check if dialog should naturally end
            if mode == 'respond':
                # Ensure observations are available for context
                if not self.observations:
                    self._observe()
                are_we_done = self.check_natural_dialog_end(source, text_input)
                logger.info(f'🤖 {self.character_name} Dialog end check: {are_we_done}')
                if are_we_done:
                    reason = f'Dialog end detected with {source}, dialog_history:\n{dialog_history}'
                    self.publish_dialog_end(source)
                    if not self.manual and self.current_goal:
                        new_goal = self._replan(self.current_goal, reason)
                        if new_goal:
                            self._plan(new_goal)
                    action_data = {'type': 'dialog_end','action_id': f'action_{self.action_counter}','timestamp': datetime.now().isoformat(),'input': text_input,'text': 'Done','source': source}
                    self.action_publisher.put(json.dumps(action_data))
                    logger.info(f'📤 Published action: {action_data["action_id"]}')
                    return False
            
            # Simple, focused prompt
            if mode == 'say':
                system_prompt = f"""Your task is to say the following text:

    "{text_input}" 

updated to reflect the current context described below. Keep the length of the text at most 10 words longer than the original text above.

you are:
"""
            else:
                system_prompt = f"""Your task is to respond to:

    "{text_input}"

    in the current context described below. You are in a conversation, so you should respond in a conversational manner, saying only enough for the conversation to continue.

you are:
"""
            if not self.observations:
                self._observe()
                
            system_prompt += self._update_system_prompt()
            
            # Build user prompt with context
            user_prompt = '' 
            user_prompt = self.observations['dynamic']
            if dialog_history:
                user_prompt += f"{dialog_history}\n"

            if mode == 'say':
                user_prompt +=  f"""\nAgain, you are rewriting a text to be spoken within a conversation:\n\n"""
            else:
                user_prompt += """\nAgain, you are responding to the following:\n\n"""
                
            user_prompt += f"""{text_input}
            
Output ONLY the updated text to be spoken within a conversation, Speak in a conversational manner in your own voice. 
Do NOT output any reasoning or thinking.
Use the following hash-formatted text using the text tag to report your response, where the text tag is preceded by a # and followed by a single space, followed by its content.
Follow your response with a ## marker. Do not insert and '\n' in your response.

#text <your response here>
##

Do not include any reasoning, introductory, explanatory, discursive, or formatting text in your response, only the text of the response.
End your response with:
</end>
"""
                    
            # Make LLM call
            if self.llm_client and not self.shutdown_requested:
                # Use shorter timeout during shutdown
                timeout = 20.0 if self.shutdown_requested else None
                response = self.llm_client.generate(
                    messages=[system_prompt, user_prompt],
                    max_tokens=200,
                    temperature=0.7,
                    timeout=timeout,
                    stops=['</end>']
                )

                if response.success:
                    if '#text' in response.text:
                        text_to_send = response.text[response.text.index('#text')+5:].strip()
                    else:
                        text_to_send = response.text.strip()
                    
                    if not text_to_send or not text_to_send.strip():
                        logger.error(f'LLM returned empty or malformed response. Raw response: {response.text[:200]}')
                        text_to_send = response.text if response.text and len(response.text.strip()) > 0 else 'failure to generate response'
                    elif not text_to_send.lower().startswith('done'):
                        self.send_text_input(source, text_to_send)
                        logger.info(f'Responding to: "{text_input}" from {source}: {text_to_send}')

                else:
                    logger.error(f'LLM call failed: {response.error}')
            else:
                logger.error('LLM client not available, skipping LLM call')
                
        except Exception as e:
            logger.error(f'Error in LLM processing: {e}')
            logger.error(traceback.format_exc())
            response = None
                    # Create action
        action_data = {
                        'type': 'say' if mode == 'say' else 'response',
                        'action_id': f'action_{self.action_counter}',
                        'timestamp': datetime.now().isoformat(),
                        'input': text_input,
                        'text': text_to_send,
                        'source': self.character_name if mode == 'say' else source,
                        'target': source if mode == 'say' else None
        }

        if self.action_history:
            try:
                last_type = (self.action_history[-1].action.get('type', '') or '').lower()
            except Exception:
                last_type = ''
            if last_type == 'say':
                self.action_history[-1].result = text_to_send
            else:
                # Non-say last action: suppress recording dialog text into action result
                pass
        # Publish action (this will be picked up by memory_node and action_display_node)
        self.action_publisher.put(json.dumps(action_data))
        logger.info(f'📤 Published action: {action_data["action_id"]}')
        self.action_counter += 1
        return text_to_send
            
    def _execute_scan_action(self, action: Dict[str, Any]) -> str:
        """Execute scan action and return the found target name."""
        target = action.get('target', '')
        var_name = action.get('out', '')
        if target.lower() == 'person':
            target = 'Person'
        # Resolve noun target to one or more concrete map_types (if indices exist)
        candidates = []
        try:
            if self.ontology:
                noun_mappings = self.ontology.get('noun_mappings', {})
            else:
                noun_mappings = {}
            mapped = noun_mappings.get(target)
            if isinstance(mapped, list) and mapped:
                candidates = [c for c in mapped if isinstance(c, str) and c]
        except Exception:
            pass
        if not candidates:
            candidates = [target]

        # If multiple candidate types, choose globally nearest match across them
        found_target = ''
        best_dist = float('inf')
        chosen_dist = None
        for cand in candidates:
            name, dist = self._find_target_in_situation(cand)
            if not name:
                continue
            # Normalize missing distance (terrain) to mid-priority
            nearest = float(dist) if dist is not None else 1e8
            if nearest < best_dist:
                best_dist = nearest
                found_target = name
                chosen_dist = nearest
        
        # Special-case: if target is Person, bind to nearest other character (exclude self)
        try:
            if str(target).strip().lower() == 'person':
                best_name = ''
                best_d = float('inf')
                for view in (self.last_situation_data or {}).get('views', []):
                    for character in view.get('characters', []) or []:
                        nm = character.get('name', '')
                        if not nm or nm == self.character_name:
                            continue
                        d = character.get('distance')
                        if d is None:
                            continue
                        dd = float(d)
                        if best_name == '' or dd < best_d or (dd == best_d and nm < best_name):
                            best_name = nm
                            best_d = dd
                if best_name:
                    found_target = best_name
                    chosen_dist = best_d
        except Exception:
            pass
        
        # Store result in plan_bindings if var_name provided
        if var_name and found_target:
            self.plan_bindings[var_name] = found_target
            logger.info(f'🔗 {self.character_name} bound variable: {var_name} = {found_target}')
        elif var_name and not found_target:
            logger.warning(f'⚠️ {self.character_name} scan failed to find target "{target}" for variable {var_name}')
        elif not var_name:
            logger.error(f'🔍 {self.character_name} scan found target "{target}" but no variable to bind')
        
        # Cache chosen scan distance for evidence use
        if found_target:
            return found_target, chosen_dist
        else:
            return '', sys.maxsize

    def _find_target_in_situation(self, target: str) -> tuple[str, float | None]:
        """Search last_situation_data for target, return (nearest_name, distance).

        distance is None for terrain-only matches (no distance available).
        """
        if not self.last_situation_data:
            return '', None
            
        target_lower = target.lower()
        
        best_name = ''
        best_dist = float('inf')
        
        for view in self.last_situation_data.get('views', []):
            # Characters
            if 'characters' in view:
                for character in view['characters']:
                    char_name = character.get('name', '')
                    if (char_name and (
                        char_name.lower() == target_lower or
                        target_lower == 'person' or
                        char_name.lower().startswith(target_lower))):
                        dist = float(character.get('distance', 1e9))
                        if dist < best_dist or (dist == best_dist and char_name < best_name):
                            best_name = char_name
                            best_dist = dist
            # Resources
            if 'resources' in view:
                for resource in view['resources']:
                    resource_name = resource.get('name', '')
                    if (resource_name and (
                        resource_name.lower() == target_lower or
                        resource_name.lower().startswith(target_lower))):
                        dist = float(resource.get('distance', 1e9))
                        if dist < best_dist or (dist == best_dist and resource_name < best_name):
                            best_name = resource_name
                            best_dist = dist
            # Paths (infrastructure)
            if 'paths' in view:
                for path in view['paths']:
                    path_name = path.get('name', '')
                    if (path_name and (
                        path_name.lower() == target_lower or
                        path_name.lower().startswith(target_lower))):
                        dist = float(path.get('distance', 1e9))
                        if dist < best_dist or (dist == best_dist and path_name < best_name):
                            best_name = path_name
                            best_dist = dist
            # Terrain exact match (no distance available; treat as mid priority)
            if 'terrain' in view:
                terrain = view['terrain']
                if isinstance(terrain, str) and terrain.lower() == target_lower:
                    # Prefer any finite distance match over terrain
                    if best_name == '':
                        best_name = terrain
                        best_dist = 1e8
        
        # If best_name is terrain (no distance), best_dist may be 1e8 sentinel. Normalize to None.
        if best_name and best_dist == 1e8:
            return best_name, None
        return (best_name, best_dist) if best_name else ('', None)
            
    def _find_target_direction(self, target: str) -> str:
        """Find which direction a target (character or resource) is visible in."""
        try:
            # Query situation node for current situation data
            for reply in self.session.get(f"cognitive/{self.character_name}/situation/current_situation", target=QueryTarget.BEST_MATCHING, consolidation=ConsolidationMode.NONE, timeout=5.0 if not self.debug else 300.0):
                if reply.ok:
                    situation_data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                    if not situation_data.get('success'):
                        logger.error(f'Failed to get situation data: {situation_data.get("error", "Unknown error")}')
                        return None
                    
                    situation = situation_data.get('situation', {})
                    views = situation.get('views', {})
                    
                    if not views:
                        logger.error(f'No views available for target search')
                        return None
                    
                    # Search through each direction for the target
                    directions = ['North', 'Northeast', 'East', 'Southeast', 'South', 'Southwest', 'West', 'Northwest']
                    target_lower = target.lower()
                    
                    for view in views:
                        direction = view.get('direction', {})
                        
                        # Check resources in this direction
                        if view['terrain'].lower() == target_lower:
                            return direction
                        
                        if 'resources' in view:
                            for resource in view['resources']:
                                if resource.get('name', '').lower() == target_lower:
                                    logger.info(f'🎯 Found target "{target}" as resource in direction: {direction}')
                                    return direction
                        
                        # Check paths (infrastructure) in this direction
                        if 'paths' in view:
                            for path in view['paths']:
                                if path.get('name', '').lower() == target_lower:
                                    logger.info(f'🎯 Found target "{target}" as path in direction: {direction}')
                                    return direction
                        
                        # Check characters in this direction
                        if 'characters' in view:
                            for character in view['characters']:
                                character_name = character.get('name', '') 
                                if character_name.lower() == target_lower:
                                    logger.info(f'🎯 Found target "{target}" as character in direction: {direction}')
                                    return direction
                    
                    logger.warning(f'❌ Target "{target}" not found in any visible direction')
                    return None
                
                break  # Only process first reply
            
            logger.error(f'❌ No response from situation query for target "{target}"')
            return None
            
        except Exception as e:
            logger.error(f'Error finding direction for target "{target}": {e}')
            return None

    def _capture_percepts_at_plan(self) -> List[Dict[str, Any]]:
        """Capture a compact, normalized snapshot of percepts at plan start.

        Limits size and fields to keep logs small. Returns [] on failure.
        """
        snapshot: List[Dict[str, Any]] = []
        try:
            situation = None
            try:
                for reply in self.session.get(f"cognitive/{self.character_name}/situation/current_situation", target=QueryTarget.BEST_MATCHING, consolidation=ConsolidationMode.NONE, timeout=3.0 if not self.debug else 300.0):
                    if reply.ok:
                        situation_data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                        if situation_data.get('success'):
                            situation = situation_data.get('situation', {})
                            break
            except Exception as e:
                logger.error(f'Error querying current_situation in format_situation: {e}')
            views = situation.get('views', []) if isinstance(situation, dict) else []
            # Cap number of views
            for view in views[:9]:
                entry: Dict[str, Any] = {
                    'direction': view.get('direction'),
                    'terrain': view.get('terrain'),
                    'visibility': view.get('visibility'),
                }
                resources_out = []
                for res in (view.get('resources') or [])[:5]:
                    resources_out.append({'name': res.get('name'), 'distance': res.get('distance')})
                chars_out = []
                for ch in (view.get('characters') or [])[:5]:
                    chars_out.append({'name': ch.get('name'), 'distance': ch.get('distance')})
                if resources_out:
                    entry['resources'] = resources_out
                if chars_out:
                    entry['characters'] = chars_out
                paths_out = []
                for ch in (view.get('paths') or [])[:5]:
                    chars_out.append({'name': ch.get('name'), 'distance': ch.get('distance')})
                if paths_out:
                    entry['paths'] = paths_out
                snapshot.append(entry)
        except Exception:
            return []
        return snapshot

    def move(self, move_direction: str):
        """Move the character in the specified direction."""
        logger.info(f'Moving {move_direction}')
        
        try:
            # Query map node to move the agent
            move_data = {'direction': move_direction}
            try:
                for reply in self.session.get(f"cognitive/map/agent/{self.character_name}/move", target=QueryTarget.BEST_MATCHING, consolidation=ConsolidationMode.NONE, payload=json.dumps(move_data).encode('utf-8'), timeout=5.0 if not self.debug else 300.0):
                    try:
                        if reply.ok:
                            move_result = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                            if move_result.get('success'):
                                logger.info(f'✅ Move successful: {move_direction}')
                            else:
                                logger.error(f'❌ Move failed: {move_result.get("error", "Unknown error")}')
                        else:
                            logger.error(f'❌ Move query failed for {self.character_name}')
                    except Exception as e:
                        logger.error(f'Error parsing move response: {e}')
                        break
            except Exception as e:
                logger.error(f'Error querying move: {e}')
            
            # Update the most recent action record with the result
            if self.action_history:
                self.action_history[-1].result = f'moved in direction {move_direction}'
            
        except Exception as e:
            # Update the most recent action record with the error result
            if self.action_history:
                self.action_history[-1].result = f'move failed {e}\n'
            logger.error(f'Error in move operation: {e}')
        # Note: Action publication handled by _act() method, not here
    
    def take(self, action: dict, target: str):
        """Take a resource and add it to inventory."""
        # Update the most recent action record with the result
        if self.action_history:
            self.action_history[-1].result = f'taking {target}'

        def _do_take(target: str):
            # First validate that the target exists and is a resource
            logger.info(f'📦 Taking {target} for {self.character_name}')
            if self.action_history:
                self.action_history[-1].result = f'taking {target}'
            # Remove the resource from the map
            removed = False
            last_error = ''
            try:
                for reply in self.session.get(f"cognitive/map/resource/remove/{target}", target=QueryTarget.BEST_MATCHING, consolidation=ConsolidationMode.NONE, timeout=2.0 if not self.debug else 300.0):
                    if reply.ok:
                        data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                        if data.get('success'):
                            logger.info(f'🗑️ Removed {target} from map')
                            removed = True
                        else:
                            last_error = data.get('error', 'Unknown error')
                            logger.error(f'⚠️ Failed to remove {target} from map: {last_error}')
                    else:
                        last_error = 'No response from map_node'
                        logger.error(f'⚠️ Failed to remove {target} from map - no response')
                    break
            except Exception as e:
                logger.error(f'Error querying resource/remove: {e}')

            return removed

        try:
            result = _do_take(target)
            # Note: Action publication handled by _act() method, not here
            
            # Publish to perception_node
            if result:
                self.perception_action_result_publisher.put(json.dumps({
                    'action': action,
                    'update_text': f'took {target}',
                    'prediction': action.get('prediction', '')
                }))
            
            return result

        except Exception as e:
            logger.error(f'Error in take operation for {target}: {e}')
            if self.action_history:
                self.action_history[-1].result = f'take failed'
            return False

    def place(self, action: dict, target: str):
        """Place a resource from inventory onto the current map square."""
        # Update the most recent action record with the result
        if self.action_history:
            self.action_history[-1].result = f'placing {target}'

        def _do_place(target: str):
            logger.info(f'📦 Placing {target} for {self.character_name}')
            if self.action_history:
                self.action_history[-1].result = f'placing {target}'

            placed = False
            last_error = ''
            try:
                payload = json.dumps({'character_name': self.character_name}).encode('utf-8')
            except Exception:
                payload = None
            try:
                for reply in self.session.get(f"cognitive/map/resource/place/{target}", target=QueryTarget.BEST_MATCHING, consolidation=ConsolidationMode.NONE, payload=payload, timeout=2.0 if not self.debug else 300.0):
                    if reply.ok:
                        data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                        if data.get('success'):
                            logger.info(f'🧺 Placed {target} onto map')
                            placed = True
                        else:
                            last_error = data.get('error', 'Unknown error')
                            logger.error(f'⚠️ Failed to place {target} on map: {last_error}')
                    else:
                        last_error = 'No response from map_node'
                        logger.error(f'⚠️ Failed to place {target} on map - no response')
                    break
            except Exception as e:
                logger.error(f'Error querying resource/place: {e}')

            return placed

        try:
            result = _do_place(target)
            # Note: Action publication handled by _act() method, not here
            
            # Publish to perception_node
            if result:
                self.perception_action_result_publisher.put(json.dumps({
                    'action': action,
                    'update_text': f'placed {target}',
                    'prediction': action.get('prediction', '')
                }))
            
            return result

        except Exception as e:
            logger.error(f'Error in place operation for {target}: {e}')
            if self.action_history:
                self.action_history[-1].result = f'place failed'
            return False
    
    def inspect(self, action: dict, target: str):
        """Learn about a resource."""
        # Note: Action publication handled by _act() method, not here
        try:
            system_prompt = f'Task: You are {self.character_name} and you are inspecting {target} to gather information about {action['reason']}.'\
                + '\n' + self._update_system_prompt()
            user_prompt = self.observations['dynamic']
            # Query for resource details
            resource_data = None
            try:
                for reply in self.session.get(
                    f"cognitive/map/resource/{target}", 
                    target=QueryTarget.BEST_MATCHING,
                    consolidation=ConsolidationMode.NONE,
                    timeout=2.0 if not self.debug else 300.0
                ):
                    if reply.ok:
                        data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                        if data.get('success'):
                            resource_data = data.get('resource')
                            break
            except Exception as e:
                logger.error(f'Error querying resource details: {e}')

            # Use the description in the directive
            resource_description = resource_data.get('description', '') if resource_data else ''
            directive = f"""You are inspecting: {target} to gather information about {action['reason']}.
#Resource Description: 
{resource_description}
##

Respond with a short text description of the resource consistent with its name and description, situated in the situation.
The description should particularly highlight information relevant to your drives, goal, and current plan.
It should specifically provide information relevant to {action['reason']}.
The description should be 20 words max.
Do not include any other introductory, explanatory, discursive, or formatting text in your response.
End your response with: 
</end>
"""
            if self.llm_client and not self.shutdown_requested:
                # Use shorter timeout during shutdown
                timeout = 5.0 if self.shutdown_requested else None
                response = self.llm_client.generate(
                    messages=[system_prompt, user_prompt, directive],
                    max_tokens=50,
                    temperature=0.7,
                    timeout=timeout,
                    stops=['</end>']
                )
                if response.success:
                    logger.info(f'🤖 {self.character_name} Inspected {target}:\n\t {response.text}')
                    if self.action_history:
                        self.action_history[-1].result = response.text
                    self.inspections[target] = response.text
                    self.perception_action_result_publisher.put(json.dumps({
                        'action': action,
                        'update_text': response.text,
                        'prediction': action.get('prediction', '')
                    }))
                    return True
                else:
                    logger.error(f'LLM call failed: {response.error}')
            else:
                logger.error('LLM client not available')

            logger.info(f'📦 Inspecting {target} for {self.character_name}')
            if self.action_history:
                self.action_history[-1].result = f'inspected {target}'
            return True

        except Exception as e:
            logger.error(f'Error in inspect operation for {target}: {e}')
            if self.action_history:
                self.action_history[-1].result = f'inspect failed'

    def use(self, action: dict, target: str, action_data: dict):
        """Use a resource or skill."""
        # Note: Action publication handled by _act() method, not here
        try:
            # Query resource rules to determine if this is a skill
            rules_response = None
            resource_type = None
            skill_config = None
            
            try:
                for reply in self.session.get(f"cognitive/map/resource_rules/{target}", target=QueryTarget.BEST_MATCHING, consolidation=ConsolidationMode.NONE, timeout=3.0 if not self.debug else 300.0):
                    if reply.ok:
                        rules_response = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                        break
            except Exception as e:
                logger.error(f'Error querying resource_rules: {e}')
            
            if rules_response and rules_response.get('success'):
                resource_rules = rules_response.get('resource_rules', {})
                resource_type = resource_rules.get('resource_type', '').lower()
                skill_config = resource_rules
            
            # Branch: Skill execution or physical resource use
            if resource_type == 'skill':
                # SKILL EXECUTION PATH
                logger.info(f'🧠 Executing skill: {target}')
                
                from skill_executor import SkillExecutor
                executor = SkillExecutor(self)
                
                # Extract value parameter from action
                value = action.get('value')
                reason = action.get('reason', '')
                
                # Execute skill
                result = executor.execute_skill(
                    skill_name=target,
                    value=value,
                    reason=reason,
                    skill_config=skill_config
                )
                
                # Format result as outcome text
                if result['success']:
                    result_text = result.get('result', '')
                    if isinstance(result_text, dict):
                        # For structured results, format nicely
                        result_text = json.dumps(result_text, indent=2)
                    elif not isinstance(result_text, str):
                        result_text = str(result_text)
                    
                    outcome_text = f"Skill {target} completed successfully.\nResult: {result_text}"
                    logger.info(f'✓ {self.character_name} used skill {target}:\n{result_text[:200]}...')
                else:
                    outcome_text = f"Skill {target} failed: {result.get('error', 'Unknown error')}"
                    logger.error(f'✗ Skill {target} failed: {result.get("error")}')
                
                # Store result (same flow as physical resources)
                if self.action_history:
                    self.action_history[-1].result = outcome_text
                self.uses[target] = outcome_text
                
                # Publish (same flow as physical resources)
                self.world_state_update_publisher.put(json.dumps({
                    'action': action,
                    'update_text': outcome_text
                }))
                self.perception_action_result_publisher.put(json.dumps({
                    'action': action,
                    'update_text': outcome_text,
                    'prediction': action.get('prediction', '')
                }))
                action_data['response'] = outcome_text
                
                # If skill succeeded and 'out' field present, create Note or Collection instance
                if result['success'] and action.get('out'):
                    var_name = action.get('out')
                    
                    # Determine content type and format
                    skill_result = result.get('result', '')
                    
                    # Detect if result is a Collection (list) or Note (single value)
                    if isinstance(skill_result, list):
                        # Collection type
                        content = skill_result
                        format_type = 'list'
                        resource_kind = 'Collection'
                        create_topic = "cognitive/map/collection/create"
                    elif isinstance(skill_result, dict):
                        # Note with JSON content
                        content = skill_result
                        format_type = 'json'
                        resource_kind = 'Note'
                        create_topic = "cognitive/map/note/create"
                    else:
                        # Note with text content
                        content = str(skill_result)
                        format_type = 'text'
                        resource_kind = 'Note'
                        create_topic = "cognitive/map/note/create"
                    
                    # Create Note or Collection instance via map_node
                    try:
                        for reply in self.session.get(
                            create_topic,
                            target=QueryTarget.BEST_MATCHING,
                            consolidation=ConsolidationMode.NONE,
                            timeout=5.0 if not self.debug else 300.0,
                            payload=json.dumps({
                                'character_name': self.character_name,
                                'content': content,
                                'format': format_type,
                                'source_skill': target,
                                'source_value': value or ''
                            }).encode('utf-8')
                        ):
                            if reply.ok:
                                info_response = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                                if info_response.get('success'):
                                    info_id = info_response.get('info_id')
                                    # Bind variable to Note or Collection resource ID
                                    self.plan_bindings[var_name] = info_id
                                    logger.info(f'🔗 {self.character_name} bound variable: {var_name} = {info_id} ({resource_kind})')
                                else:
                                    logger.error(f'Failed to create {resource_kind} instance: {info_response.get("error")}')
                            break
                    except Exception as e:
                        logger.error(f'Error creating {resource_kind} instance for variable {var_name}: {e}')
                
                return result['success']
                
            else:
                # PHYSICAL RESOURCE PATH (original LLM-based assessment)
                system_prompt = self._update_system_prompt()
                user_prompt = self.observations['dynamic']

                directive = f"""You are attempting to use {target}.\n
to achieve: {action['reason']}.\n\n 
respond with a short assessment of the outcome of your attempt to use {target} to achieve {action['reason']}.
Your assessment may conclude that the use of {target} was successful, or that it was not successful, or that it was not possible to use {target} to achieve {action['reason']}.
The assessment should include consequences for yourself and the target resource, and be concise and to the point.
The assessment should be 20 words max.
Do not include any other introductory, explanatory, discursive, or formatting text in your response.
End your response with: 
</end>
"""
                if self.llm_client and not self.shutdown_requested:
                    # Use shorter timeout during shutdown
                    timeout = 5.0 if self.shutdown_requested else None
                    response = self.llm_client.generate(
                        messages=[system_prompt, user_prompt, directive],
                        max_tokens=50,
                        temperature=0.7,
                        timeout=timeout,
                        stops=['</end>']
                    )
                    if response.success:
                        logger.info(f'🤖 {self.character_name} Used {target}:\n\t {response.text}')
                        if self.action_history:
                            self.action_history[-1].result = response.text
                        self.uses[target] = response.text
                        self.world_state_update_publisher.put(json.dumps({'action': action, 'update_text': response.text}))
                        self.perception_action_result_publisher.put(json.dumps({
                            'action': action,
                            'update_text': response.text,
                            'prediction': action.get('prediction', '')
                        }))
                        action_data['response'] = response.text
                        return True
                    else:
                        logger.error(f'LLM call failed: {response.error}')
                else:
                    logger.error('LLM client not available')

                logger.info(f'📦 Using {target} for {self.character_name}')
                if self.action_history:
                    self.action_history[-1].result = f'{response.text}'
                return True

        except Exception as e:
            logger.error(f'Error in use operation for {target}: {e}')
            if self.action_history:
                self.action_history[-1].result = f'use failed'
            return False
        
    def think_about(self, action: dict):
        """Think about a value."""
        logger.info(f'Thinking about: {action}')
        thought = ''
        try:
            system_prompt = self._update_system_prompt()
            user_prompt = self.observations['dynamic']

            directive = f"""You are thinking about: {action['value']}.\n\n Derive new information, insights, goals, or conclusions based on your memories, drives, and the current situation.
    This new information should be a short statement (10 words max) not explicit in the information provided that will guide your future thoughts and actions.
    Respond with the new information in the following hash-formatted syntax using the thought tag, where the thought tag is preceded by a # and followed by a single space, followed by its content.
    Follow your response with a ## marker. Do not insert and '\n' in your response.

    #thought <new information>
    ##

    Do not include any other introductory, explanatory, discursive, or formatting text in your response.
    End your response with: 
    </end>
    """

            # Make LLM call
            if self.llm_client and not self.shutdown_requested:
                # Use shorter timeout during shutdown
                timeout = 5.0 if self.shutdown_requested else None
                response = self.llm_client.generate(
                    messages=[system_prompt, user_prompt, directive],
                    max_tokens=50,
                    temperature=0.7,
                    timeout=timeout,
                    stops=['</end>']
                )

                if response.success:
                    logger.info(f'🤖 {self.character_name} New Thought: {response.text}')
                    thought = hash_utils.find('thought', response.text)
                    if not thought:
                        logger.error(f'No thought found in LLM response: {response.text}')
                else:
                    logger.error(f'LLM call failed: {response.error}')
            else:
                logger.error('LLM client not available')
                
        except Exception as e:
            logger.error(f'Error in LLM processing: {e}')
            logger.error(traceback.format_exc())
            # Create action
        action_data = {
                        'type': 'think',
                        'action_id': f'action_{self.action_counter}',
                        'timestamp': datetime.now().isoformat(),
                        'input': action['value'],
                        'text': response.text.strip(),
                        'thought': thought,
                        'character': self.character_name,
                        'source': self.character_name
            }
        if self.action_history:
            self.action_history[-1].result = thought
        # Publish action (this will be picked up by memory_node and action_display_node)
        self.action_publisher.put(json.dumps(action_data))
        logger.info(f'📤 Published action: {action_data["action_id"]}')
        self.action_counter += 1
            
        return thought
    
    def _get_recent_chat_memories(self, num_entries: int) -> List[Dict[str, Any]]:
        """Get recent memory entries using Zenoh queries."""
        try:
            # Query short-term memory
            entries = []
            for reply in self.session.get(f"cognitive/{self.character_name}/memory/chat/*", target=QueryTarget.BEST_MATCHING, consolidation=ConsolidationMode.NONE, timeout=3.0 if not self.debug else 300.0):
                try:
                    if reply.ok:
                        content = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                        entries.append(content)
                except Exception as e:
                    logger.error(f'Error getting recent memories: {e}')
                    continue
            
            m1 = entries[0] if len(entries) > 0 else None
            m2 = m1['entries'] if m1 else []

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
    

    def _assess_drive_fulfillment(self) -> Dict[str, Any]:
        """Assess drive fulfillment using an LLM. Returns structured scores per drive.

        Output shape:
          { "drives": [{"name": str, "score": float, "rationale": str}]}
        """
        assessment = []
        try:
            goal_text = self.current_goal.to_string() if self.current_goal else ''
            plan_text = json.dumps(self.current_plan, indent=2) if self.current_plan else '{}'
            # Physiological snapshot
            state = {}
            for k, v in (self.self_state or {}).items():
                state[k] = float(v.get('value', 0.0))
            state_text = json.dumps(state, indent=2)
            # Inventory (use cache)
            inventory_text = list(self.inventory_cache or [])
            # Conversations (recent)
            chats_text = self._get_recent_chat_memories(10)
            # Knowledge: thoughts, inspects, uses
            thoughts = []
            for ar in (self.action_history or []):
                if (ar.action or {}).get('type', '').lower() == 'think' and ar.result:
                    thoughts.append(str(ar.result))
            thoughts_text = '\n'.join(thoughts)
            inspects_map = dict(self.inspections or {})
            uses_map = dict(self.uses or {})
            # Recent actions compact
            actions_text = ''
            for ar in (self.action_history or [])[-20:]:
                actions_text += f"""{ar.action.get('type', '')}, {ar.action .get('target', '')}: result: {ar.result or ''}\n"""

        except Exception as e:
            logger.error(f'Error collecting drive assessment context: {e}')
            traceback.print_exc()
            return []

        for drive in self.drives:
            if not self.llm_client or self.shutdown_requested:
                continue
            response = self.llm_client.generate(
                messages=[DRIVE_ASSESSMENT_TEMPLATE],
                bindings={
                    "drive": drive,
                    "goal": goal_text,
                    "plan": plan_text,
                    "state": state_text,
                    "inventory": inventory_text,
                    "chats": chats_text,
                    "thoughts": thoughts_text,
                    "inspects_map": inspects_map,
                    "uses_map": uses_map,
                    "actions": actions_text
                },
                max_tokens=600,
                temperature=0.3,
                timeout=200.0,
                is_json=True,
                stops=['</end>']
            )

            if response.success:
                if isinstance(response.text, dict):
                    result = response.text.copy()
                    result['name'] = drive
                    assessment.append(result)
                else:
                    try:
                        result = json.loads(response.text.strip())
                        result['name'] = drive
                        assessment.append(result)
                    except Exception as e:
                        logger.error(f'Error parsing drive fulfillment response: {e}')
                        return { }
            else:
                logger.error(f'Error assessing drive fulfillment: {response.error}')
                return { }
        return { 'drives': assessment }

 
    def get_entity_context(self, entity_name: str, limit: int = 20, scope='all') -> Dict[str, Any]:
        """
        Query entity data from memory node for context.
        
        Args:
            entity_name: Name of the entity to query
            limit: Number of recent conversation entries to include (default 20)
            
        Returns:
            Dictionary with entity data or None if query failed
        """
        # safeguard - don't allow variables in query
        entity_name = entity_name.replace('$', '')
        try:
            key_expr = f"cognitive/{self.character_name}/memory/entity/{entity_name}?query=dialog&limit={limit}&scope={scope}"
            for reply in self.session.get(key_expr, target=QueryTarget.BEST_MATCHING, consolidation=ConsolidationMode.NONE, timeout=10.0 if not self.debug else 20.0):
                if reply.ok:
                    data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                    if data and data.get('success'):
                        logger.info(f'👥 Retrieved entity context for {entity_name}')
                        return data.get('entity_data')
            logger.warning(f'Entity query failed or no response for {entity_name}')
            for handler in logger.handlers:
                handler.flush()
            return None
        except Exception as e:
            logger.error(f'Error querying entity context for {entity_name}: {e}')
            for handler in logger.handlers:
                handler.flush()
            return None
    
    def get_rag_context(self, query_text: str, entity_name: str = None, k: int = 5) -> Dict[str, Any]:
        """
        Query memory_node for RAG semantic search results.
        
        Args:
            query_text: The text to search for semantically
            entity_name: Optional entity to filter results (default: None for all entities)
            k: Number of results to retrieve (default: 5)
            
        Returns:
            Dictionary with retrieved_entries list and count, always returns even if empty
        """
        import urllib.parse
        import time
        
        try:
            # Build query parameters
            params = [f"query={urllib.parse.quote(query_text)}", f"k={k}"]
            if entity_name:
                params.append(f"entity={urllib.parse.quote(entity_name.capitalize())}")
            
            key_expr = f"cognitive/{self.character_name}/memory/rag/search?{'&'.join(params)}"
            start_ts = time.time()
            logger.info(f'RAG: Sending query with key_expr={key_expr}')
            for handler in logger.handlers: handler.flush()

            for reply in self.session.get(key_expr, 
                                        target=QueryTarget.BEST_MATCHING,          # ← don’t wait for all
                                        consolidation=ConsolidationMode.NONE,
                                        timeout=5.0 if not self.debug else 30.0):
                logger.info(f'RAG: Received a reply, checking if ok...')
                for handler in logger.handlers: handler.flush()
                if reply.ok:
                    data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                    if data and isinstance(data, dict) and data.get('success') is True:
                        latency_ms = int((time.time() - start_ts) * 1000)
                        entries = data.get('retrieved_entries', [])
                        count = data.get('count', len(entries) if isinstance(entries, list) else 0)
                        return {
                            'success': True,
                            'retrieved_entries': entries if isinstance(entries, list) else [],
                            'count': int(count) if isinstance(count, int) else 0,
                            'query': data.get('query', query_text),
                            'error': '',
                            'latency_ms': latency_ms
                        }
                else:
                    latency_ms = int((time.time() - start_ts) * 1000)
                    logger.error(f'RAG query failed {reply.err.payload.to_bytes().decode('utf-8')}')
                    for handler in logger.handlers: handler.flush()
                    return {'success': False, 'retrieved_entries': [], 'count': 0, 'query': query_text, 'error': 'timeout_or_no_reply', 'latency_ms': latency_ms}

            latency_ms = int((time.time() - start_ts) * 1000)
            logger.warning(f'RAG query had no reply or failed for "{query_text[:50]}..."')
            return {'success': False, 'retrieved_entries': [], 'count': 0, 'query': query_text, 'error': 'timeout_or_no_reply', 'latency_ms': latency_ms}
            
        except Exception as e:
            logger.error(f'Error querying RAG context for "{query_text[:50]}...": {e}')
            for handler in logger.handlers:
                handler.flush()
            # Provide a consistent failure schema
            latency_ms = int((time.time() - start_ts) * 1000)
            return {'success': False, 'retrieved_entries': [], 'count': 0, 'query': query_text, 'error': str(e), 'latency_ms': latency_ms}
    
    def expand_rag_context(self, rag_entry: Dict[str, Any], window_before: int = 3) -> str:
        """
        Expand a RAG retrieved entry with preceding context from the dialog.
        
        Args:
            rag_entry: Dictionary with doc_id, entity, text from RAG retrieval
            window_before: Number of preceding turns to include (default: 3)
            
        Returns:
            String with context window or just the retrieved text if verification fails
        """
        try:
            logger.info(f' Expanding RAG context for {rag_entry}')
            for handler in logger.handlers: handler.flush()
            doc_id = rag_entry.get('doc_id', '')
            if not doc_id:
                logger.warning(f'RAG expand: missing doc_id in RAG entry, returning text only')
                for handler in logger.handlers: handler.flush()
                return rag_entry.get('text', '')
            
            parts = doc_id.split(':')
            if len(parts) < 4:
                logger.warning(f'RAG expand: doc_id malformed ({doc_id}), returning text only')
                for handler in logger.handlers: handler.flush()
                return rag_entry.get('text', '')
            
            try:
                entity_name = parts[1]
                dialog_idx = int(parts[2])
                entry_idx = int(parts[3])
            except (ValueError, IndexError) as e:
                logger.warning(f'RAG expand: failed to parse doc_id {doc_id}: {e}')
                for handler in logger.handlers:  handler.flush()
                return rag_entry.get('text', '')
            
            entity_context = self.get_entity_context(entity_name)
            if not entity_context:
                logger.warning(f'RAG expand: no entity_context retrieved for {entity_name}')
                for handler in logger.handlers: handler.flush()
                return rag_entry.get('text', '')
            
            # get_entity_data now includes 'dialogs' for RAG context expansion
            dialogs = entity_context.get('dialogs')
            if not dialogs:
                logger.warning(f'RAG expand: no dialogs field in entity_context for {entity_name}')
                for handler in logger.handlers:  handler.flush()
                return rag_entry.get('text', '')
            
            if not isinstance(dialogs, list):
                logger.warning(f'RAG expand: dialogs not a list for {entity_name}, type={type(dialogs)}')
                for handler in logger.handlers: handler.flush()
                return rag_entry.get('text', '')
            
            if dialog_idx >= len(dialogs):
                logger.warning(f'RAG expand: dialog_idx {dialog_idx} >= len(dialogs) {len(dialogs)} for {entity_name} - data mismatch')
                for handler in logger.handlers: handler.flush()
                return rag_entry.get('text', '')
            
            dialog = dialogs[dialog_idx]
            if not isinstance(dialog, list):
                logger.warning(f'RAG expand: dialog {dialog_idx} not a list for {entity_name}, type={type(dialog)}')
                for handler in logger.handlers: handler.flush()
                return rag_entry.get('text', '')
            
            if entry_idx >= len(dialog):
                logger.warning(f'RAG expand: entry_idx {entry_idx} >= len(dialog) {len(dialog)} for {entity_name} - data mismatch')
                for handler in logger.handlers: handler.flush()
                return rag_entry.get('text', '')
            
            # Verify the entry matches
            entry = dialog[entry_idx]
            if isinstance(entry, dict):
                entry_text = f"{entry.get('source', '')}: {entry.get('text', '')}"
            else:
                entry_text = str(entry)
            
            rag_text = rag_entry.get('text', '')
            if rag_text and rag_text not in entry_text:
                logger.warning(f'RAG expand: entry mismatch for {entity_name} dialog {dialog_idx} entry {entry_idx}, RAG text not found in dialog entry')
                for handler in logger.handlers: handler.flush()
                return rag_text
            
            # Build context window
            start_idx = max(0, entry_idx - window_before)
            context_lines = []
            for i in range(start_idx, entry_idx + 1):
                if i < len(dialog):
                    if isinstance(dialog[i], dict):
                        context_lines.append(f"{dialog[i].get('source', '')}: {dialog[i].get('text', '')}")
                    else:
                        context_lines.append(str(dialog[i]))
            
            logger.info(f' Returning expanded RAG context: {context_lines}')
            for handler in logger.handlers: handler.flush()
            return '\n'.join(context_lines) if context_lines else rag_entry.get('text', '')
            
        except Exception as e:
            logger.error(f'Error expanding RAG context for doc_id {rag_entry.get("doc_id", "")}: {e}')
            for handler in logger.handlers:
                handler.flush()
            return rag_entry.get('text', '')
    
    def get_entity_discourse_tom_models(self, entity_name: str, limit: int = 20, scope='all') -> Dict[str, Any]:
        """
        Query entity data from memory node for discourse analysis and tom_model.
        Args:
            entity_name: Name of the entity to query
            limit: Number of recent conversation entries to include (default 20)
        Returns:
            Dictionary with entity data or None if query failed
        """
        # safeguard - don't allow variables in query
        entity_name = entity_name.replace('$', '')
        logger.info(f' Getting entity discourse/tom for {entity_name}')
        for handler in logger.handlers: handler.flush()
        try:
            key_expr = f"cognitive/{self.character_name}/memory/entity/{entity_name}?query=relation"
            for reply in self.session.get(key_expr, target=QueryTarget.BEST_MATCHING, consolidation=ConsolidationMode.NONE, timeout=3.0 if not self.debug else 300.0):
                if reply.ok:
                    data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                    if data and data.get('success'):
                        logger.info(f'👥 Retrieved entity discourse/tom for {entity_name}')
                        return data
            logger.warning(f'Entity discourse/tom query failed or no response for {entity_name}')
            for handler in logger.handlers:  handler.flush()
            return None
        except Exception as e:
            logger.error(f'Error querying entity discourse/tom for {entity_name}: {e}')
            for handler in logger.handlers: handler.flush()
            return None
   
    def check_natural_dialog_end(self, entity_name: str, input_text: str) -> bool:
        """
        Check if dialog should naturally end after the given input via memory node.
        
        Args:
            entity_name: Name of the entity
            input_text: The text input that would end the dialog
            
        Returns:
            bool: True if dialog should end, False if it should continue
        """
        try:
            import urllib.parse
            logger.info(f' Checking natural dialog end for {entity_name}: {input_text}')
            for handler in logger.handlers: handler.flush()
            encoded_text = urllib.parse.quote(input_text)
            # Use observations if available, otherwise use empty context
            context = self.observations['static'] if self.observations else ""
            query_url = f"cognitive/{self.character_name}/memory/entity/{entity_name}?query=natural_dialog_end&input_text={encoded_text}&context={context}"
            
            for reply in self.session.get(query_url, target=QueryTarget.BEST_MATCHING, consolidation=ConsolidationMode.NONE, timeout=20.0 if not self.debug else 300.0):
                if reply.ok:
                    data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                    if data and data.get('success'):
                        logger.info(f'🤔 Natural dialog end check for {entity_name}: {data.get("should_end", True)}')
                        for handler in logger.handlers: handler.flush()
                        return data.get("should_end", True)
                else:
                    logger.error(f'Natural dialog end query failed for {entity_name}!')
                    for handler in logger.handlers: handler.flush()
                    return True
            
            logger.warning(f'Natural dialog end query failed or no response for {entity_name}, assuming should end')
            for handler in logger.handlers:  handler.flush()
            return True
            
        except Exception as e:
            logger.error(f'Error querying natural dialog end for {entity_name}: {e}')
            logger.error(traceback.format_exc())
            for handler in logger.handlers:  handler.flush()
            return True
    
    def _acquire_conversation_lock(self, target_character: str) -> bool:
        """
        Attempt to acquire conversation lock with target character.
        
        Args:
            target_character: Name of character to talk to
            
        Returns:
            bool: True if lock acquired, False if not available
        """
        try:
            # Query map node to acquire conversation lock
            lock_request = {
                'requester': self.character_name,
                'target': target_character
            }
            
            for reply in self.session.get("cognitive/map/conversation/lock/acquire", 
                                        target=QueryTarget.BEST_MATCHING,
                                        consolidation=ConsolidationMode.NONE,
                                        payload=json.dumps(lock_request).encode('utf-8'),
                                        timeout=5.0 if not self.debug else 60.0):
                if reply.ok:
                    data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                    if data.get('success'):
                        lock_acquired = data.get('lock_acquired', False)
                        if lock_acquired:
                            logger.info(f'🔒 Conversation lock acquired with {target_character}')
                        else:
                            logger.info(f'🔒 Conversation lock unavailable with {target_character}')
                        return lock_acquired
                    else:
                        logger.error(f'Failed to acquire conversation lock: {data.get("error", "Unknown error")}')
                        return False
            
            logger.error(f'No response received for conversation lock request with {target_character}')
            return False
            
        except Exception as e:
            logger.error(f'Error acquiring conversation lock with {target_character}: {e}')
            return False
    
    def _release_conversation_lock(self, target_character: str):
        """
        Release conversation lock with target character.
        
        Args:
            target_character: Name of character to release lock with
        """
        try:
            # Query map node to release conversation lock
            lock_release = {
                'character1': self.character_name,
                'character2': target_character
            }
            
            for reply in self.session.get("cognitive/map/conversation/lock/release", 
                                        target=QueryTarget.BEST_MATCHING,
                                        consolidation=ConsolidationMode.NONE,
                                        payload=json.dumps(lock_release).encode('utf-8'),
                                        timeout=5.0 if not self.debug else 60.0):
                if reply.ok:
                    data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                    if data.get('success'):
                        logger.info(f'🔓 Conversation lock released with {target_character}')
                    else:
                        logger.error(f'Failed to release conversation lock: {data.get("error", "Unknown error")}')
                break
            
        except Exception as e:
            logger.error(f'Error releasing conversation lock with {target_character}: {e}')
    
    def check_conversation_lock_availability(self, target_character: str) -> bool:
        """
        Check if conversation lock is available with target character.
        
        Args:
            target_character: Name of character to check
            
        Returns:
            bool: True if lock is available, False if not
        """
        try:
            # Query map node to check conversation lock status for current character
            for reply in self.session.get(f"cognitive/map/conversation/lock/status/{self.character_name}", 
                                        target=QueryTarget.BEST_MATCHING,
                                        consolidation=ConsolidationMode.NONE,
                                        timeout=5.0 if not self.debug else 60.0):
                if reply.ok:
                    data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                    if data.get('success'):
                        is_locked = data.get('is_locked', False)
                        if not is_locked:
                            logger.info(f'🔓 Conversation lock available with {target_character}')
                            return True
                        else:
                            locked_with = data.get('locked_with', [])
                            # Check if we're already locked with the target character
                            if target_character in locked_with:
                                logger.info(f'🔒 Already locked with {target_character}')
                                return False
                            else:
                                logger.info(f'🔓 Conversation lock available with {target_character}')
                                return True
                    else:
                        logger.error(f'Failed to check conversation lock status: {data.get("error", "Unknown error")}')
                        return False
            
            logger.error(f'No response received for conversation lock status check')
            return False
            
        except Exception as e:
            logger.error(f'Error checking conversation lock availability with {target_character}: {e}')
            return False
    
    def shutdown(self):
        """Clean shutdown."""
        try:
            if getattr(self, '_shutting_down', False):
                return
            self._shutting_down = True
            
            logger.info(f'Executive Node shutdown initiated for {self.character_name}...')
            
            # Publish shutdown event for cleanup
            try:
                shutdown_data = {
                    'character_name': self.character_name,
                    'timestamp': datetime.now().isoformat(),
                    'type': 'shutdown'
                }
                self.session.put(f"cognitive/{self.character_name}/shutdown", 
                               json.dumps(shutdown_data).encode('utf-8'))
                logger.info(f'Published shutdown event for {self.character_name}')
            except Exception as e:
                logger.error(f'Error publishing shutdown event: {e}')
            
            # Close Zenoh session
            try:
                self.session.close()
                logger.info('Zenoh session closed')
            except Exception as e:
                logger.error(f'Error closing Zenoh session: {e}')
            
            logger.info(f'Executive Node shutdown complete for {self.character_name}')
            
        except Exception as e:
            logger.error(f'Error during shutdown: {e}')

def main():
    """Main entry point for the executive node."""
    parser = argparse.ArgumentParser(description='Zenoh Executive Node')
    parser.add_argument('-c', '--character-name', default='default', help='Character name for topic paths')
    parser.add_argument('-config', default='{}', help='Character configuration as JSON string')
    
    args = parser.parse_args()
    
    # Parse character config
    try:
        character_config = json.loads(args.config)
    except json.JSONDecodeError as e:
        print(f"Error parsing character config: {e}")
        return
    
    executive_node = ZenohExecutiveNode(args.character_name, character_config)
    try:
        executive_node.run()
    finally:
        executive_node.shutdown()


if __name__ == '__main__':
    main() 