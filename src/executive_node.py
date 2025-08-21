#!/usr/bin/env python3
"""
Zenoh Executive Node

This node implements the OODA loop for character decision-making and action execution.

"""

import math
import random
import traceback
import zenoh
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
from activity import ActivityManager
import utils.hash_utils as hash_utils
from utils.zenoh_utils import datetime_handler
import plan as plan_module
from dataclasses import dataclass
import os
from templates import PLAN_TEMPLATE

# Configure logging with unbuffered output
# Console handler with WARNING level (less verbose)
console_handler = logging.StreamHandler(sys.stdout)
console_handler.setLevel(logging.WARNING)

# File handler with INFO level (full logging)
file_handler = logging.FileHandler('logs/executive_node.log', mode='w')
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
logger = logging.getLogger('executive_node')

# Import LLM client
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__))))

try:
    from llm_client import ZenohLLMClient
    LLM_CLIENT_AVAILABLE = True
except ImportError as e:
    print(f"⚠️  LLM Client not available: {e}")
    LLM_CLIENT_AVAILABLE = False



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
        
        # Publisher for memory storage (character-specific)
        self.memory_publisher = self.session.declare_publisher(f"cognitive/{character_name}/memory/store")
        
        # Publisher for text input to other characters (character-specific)
        self.text_input_publisher = self.session.declare_publisher(f"cognitive/{character_name}/text_input")
        
        # Publisher for goals (character-specific)
        self.goal_publisher = self.session.declare_publisher(f"cognitive/{character_name}/goal")
        
        # Publisher for decided actions (character-specific) 
        self.decided_action_publisher = self.session.declare_publisher(f"cognitive/{character_name}/decided_action")
        
        # Publisher for current plans (character-specific)
        self.current_plan_publisher = self.session.declare_publisher(f"cognitive/{character_name}/current_plan")
        
        # Publisher for current activities (character-specific)
        self.current_activity_publisher = self.session.declare_publisher(f"cognitive/{character_name}/current_activity")
        
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
        self.llm_client = None
        if LLM_CLIENT_AVAILABLE:
            self.llm_client = ZenohLLMClient(service_timeout=30.0 if not self.debug else 600.0)
        
        # Internal state
        self.action_counter = 0
        self.last_sense_data = None
        self.last_situation_data = None
        
        # OODA loop state
        self.current_goal = None
        self.observations = None
        self.interrupt_pending = False
        self.text_input_pending = False
        self.action_history = []  # List of ActionRecord instances
        
        # Plan execution state
        self.current_plan = None
        self.plan_state = None
        self.plan_bindings_cache = {}
        self.plan_summary_completed = False  # Track if current plan has been summarized
        self.plan_summary = None
        # Control-flow telemetry and plan/step identifiers
        self.control_flow_events: List[Dict[str, Any]] = []
        self.current_plan_id: Optional[str] = None
        self.plan_counter: int = 0
        self.step_counter: int = 0
        self.current_plan_prompt_template:str = ''
        self.current_plan_prompt_bindings:dict = {}
        self.plan_log: List[Dict[str, Any]] = [] # log of plans and actions
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

        # Get map types
        self.map_types = {}
        for reply in self.session.get("cognitive/map/types", timeout=2.0 if not self.debug else 600.0):
            if reply.ok:
                data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                if data.get('success'):
                    self.map_types = data
                    break
        self.inspections = {} # cache of inspections
        self.uses = {} # cache of uses
        self.activities = {} # dictionary of all available activities for initializing activity manager

        try:
            self.activities = json.load(open(f'../scenarios/{self.character_name}-activities.json'))
            if 'activities' in self.activities: # happens sometimes, not sure why
                self.activities = self.activities['activities']
        except Exception as e:
            if not self.manual:
                logger.error(f'Error loading activities for {self.character_name}: {e}')
            self.activities = {}
        self.activity_manager: ActivityManager = ActivityManager(self, self.activities, self.llm_client, self.map_types)

        # Register signal handlers for graceful shutdown
        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)
        
        logger.info(f'🧠 Zenoh Executive Node initialized for character: {character_name}')
        logger.info(f'   - Subscribing to: cognitive/{character_name}/sense_data')
        logger.info(f'   - Subscribing to: cognitive/{character_name}/situation/update')
        logger.info(f'   - Subscribing to: cognitive/map/turn/go')
        logger.info(f'   - Subscribing to: cognitive/map/time_advanced')
        logger.info(f'   - Subscribing to: cognitive/{character_name}/end_dialog')
        logger.info(f'   - Publishing to: cognitive/{character_name}/action')
        logger.info(f'   - Publishing to: cognitive/{character_name}/situation/request_update')
        logger.info(f'   - Publishing to: cognitive/{character_name}/memory/store')
        logger.info(f'   - Publishing to: cognitive/{character_name}/text_input')
        logger.info(f'   - Publishing to: cognitive/{character_name}/goal')
        logger.info(f'   - Publishing to: cognitive/{character_name}/decided_action')
        logger.info(f'   - Publishing to: cognitive/map/turn/complete/{character_name}')
        logger.info(f'   - Publishing to: cognitive/map/time_proposal')
    
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
                    # Wait for turn signal
                    if self.text_input_pending and self.last_sense_data:
                        content = self.last_sense_data['content']
                        try:
                            content_data = json.loads(content)
                            text_input = content_data.get('text', '')
                            source = content_data.get('source', 'unknown')
                            #if source == 'User': # other text inputs must be handled by OODA to observe dialog turn taking
                            self.generate_speech(text_input, source, mode='respond')
                            self.text_input_pending = False
                            self.last_sense_data = None
                        except (json.JSONDecodeError, TypeError):
                            logger.error(f'Error parsing text input: {content}')
                            self.text_input_pending = False
                            self.last_sense_data = None
                            continue
                    else:
                        time.sleep(0.1)
                else:
                    self._run_ooda_loop()
                    # Complete turn after OODA loop
                    self._complete_turn()
                time.sleep(1.0)  # Small delay to prevent busy waiting
                
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
            decided_action_data = {
                'decided_action': f"{action['type']}: {action.get('target', '')} - {action.get('value', '')}",
                'type': action['type'],
                'target': action.get('target', ''), 
                'value': action.get('value', ''),
                'timestamp': datetime.now().isoformat(),
                'character': self.character_name
            }
            
            self.decided_action_publisher.put(json.dumps(decided_action_data))
            logger.info(f'📋 Published decided action for {self.character_name}: {decided_action_data["decided_action"]}')
            
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
            
        except Exception as e:
            logger.error(f'Error publishing current plan: {e}')

    def _publish_current_activity(self):
        """Publish current activity to the current_activity topic for UI display."""
        try:
            # Get current activity from activity manager if available
            current_activity = None
            current_step = None
            activity_state = None
            
            if hasattr(self, 'activity_manager') and self.activity_manager:
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

    def _run_ooda_loop(self):
        """Execute the OODA loop: Observe, Orient, Decide, Act."""
        try:
            # Observe: Collect current situation and sense data
            observations = self._observe()

            # Manual characters skip orient/plan entirely
            if self.manual:
                if self.current_plan:
                    action = self._plan_step(self.current_plan)
                    if action is not None:
                        action_succeeded = self._act(action)
                        if not action_succeeded:
                            # Action failed, don't advance plan state but complete turn
                            logger.info(f'Manual action failed for {self.character_name}, will retry same step next turn')
                            return
                return
            # Check for active activity and get current step
            if hasattr(self, 'activity_manager') and self.activity_manager.has_active_activity():
                should_continue, reason = self.activity_manager.continue_activity()
                if should_continue:
                    self.current_step = self.activity_manager.activity_step()  # Get current step without advancing
                else:
                    # Abandon current activity
                    self.activity_manager.step_completion('failure', reason)
                    self.current_step = None
                    self.current_activity = None
                    self.current_goal = None
                    self.current_plan = None
                    self._publish_current_activity()
                    logger.info(f'🚫 {self.character_name} abandoned activity: {reason}')
                    # Fall through to select new activity
            
            # No active activity or current one was abandoned - select new one
            if hasattr(self, 'activity_manager') and not self.activity_manager.has_active_activity():
                self.current_step, self.current_activity = self.activity_manager.select_activity()
                self.current_goal = None
                self._publish_current_activity()
                if self.current_step and self.current_activity:
                    logger.info(f'🎯 {self.character_name} starting new activity: {self.current_activity["name"]}')
                else:
                    logger.warning(f'🚫 {self.character_name} no activity selected')

            # Convert step to goal or use existing goal/orient
            new_goal = None
            if self.current_step:
                # Only create new goal if we don't have the right one already
                if not self.current_goal or self.current_goal.name != self.current_step['name']:
                    new_goal = plan_module.Goal(self.current_step['name'], self.current_step['actors'], self.current_step['description'], self.current_step['termination'])

            self.current_goal = self._orient(observations, new_goal)
            self._publish_goal(self.current_goal)

            logger.info(f'🎯 {self.character_name} oriented to goal: {self.current_goal.to_string()}')

            # Plan: Return existing plan or create single-action plan
            plan = self._plan(self.current_goal)
            if not plan:
                logger.warning(f'🚫 {self.character_name} no plan found for goal: {self.current_goal.to_string()}')
                return

            # Plan Step: Execute current step of plan
            action = self._plan_step(plan)
            if action:
                logger.info(f'🎯 {self.character_name} planned action: {action.get("type")}: {action.get("target")} - {action.get("value")}')
            else:
                logger.warning(f'🚫 {self.character_name} no action found for plan, pbly completed')
                return

            # Act: Execute the chosen action (if we have one)
            if action is not None:
                action_succeeded = self._act(action)
                if not action_succeeded:
                    # Action failed (e.g., conversation lock unavailable), don't advance plan state
                    # but still complete the turn so launcher can continue
                    logger.info(f'Action failed for {self.character_name}, will retry same step next turn')
                    return
            
        except Exception as e:
            logger.error(f'Error in OODA loop: {e}')
            logger.error(traceback.format_exc())
        return

    def format_situation(self):
        """Format the situation data for the LLM."""
        formatted_situation = ''
        if self.last_situation_data and self.last_situation_data.get('location'):
            formatted_situation += f"\n#You are at location: {self.last_situation_data['location']}\n"
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
                entity_context = self.get_entity_context(character_name, 10)
                if entity_context:
                    formatted_situation += f"\n#You can see {character_name}, with whom you have had the following conversation history:\n"
                    for memory in entity_context['conversation_history']: 
                        formatted_situation += f"\n\t{memory['source']}: {memory['text']}"
                    formatted_situation += '\n'

        if self.last_situation_data and self.last_situation_data.get('views'):
            formatted_situation += f"\n#You can see the following:\n"+json.dumps(self.last_situation_data['views'], indent=2)

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
        if hasattr(self, 'activity_manager') and self.activity_manager.has_active_activity():
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
                for reply in self.session.get("cognitive/map/types", timeout=2.0 if not self.debug else 600.0):
                    if reply.ok:
                        data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                        if data.get('success'):
                            self.map_types = data
                            break
            if self.map_types:
                system_prompt += f'\n#Available map types:'
                if self.map_types.get('terrain_types'):
                    system_prompt += f"\n\tTerrain: {', '.join(self.map_types['terrain_types'])}"
                if self.map_types.get('infrastructure_types'):
                    system_prompt += f"\n\tInfrastructure: {', '.join(self.map_types['infrastructure_types'])}"
                if self.map_types.get('property_types'):
                    system_prompt += f"\n\tProperties: {', '.join(self.map_types['property_types'])}"
                if self.map_types.get('resource_types'):
                    system_prompt += f"\n\tResources: {', '.join(self.map_types['resource_types'])}"
                system_prompt += '\n'

                
            # Build user prompt with context
            user_prompt += self.format_situation()
            entity_context = None
            entity_context = self.get_entity_context(self.character_name, 10)
            if entity_context:
                user_prompt += f'\n#Your most recent thoughts include:'
                for i, memory in enumerate(entity_context['conversation_history']):  # Use last 2 memories
                    user_prompt += f"\n\t{memory['source']}: {memory['text']}"
                user_prompt += '\n'
            # get inventory
            inventory = []
            for reply in self.session.get(f"cognitive/{self.character_name}/memory/inventory", timeout=2.0 if not self.debug else 600.0):
                if reply.ok:
                    data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                    if data.get('success'):
                        value = data.get('value', [])
                        if isinstance(value, list):
                            inventory.extend(value)
                        elif value:
                            inventory.append(value)
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

    def _orient(self, observations: Dict[str, Any], new_goal: plan_module.Goal = None):
        """Orient: Assess current state and drives"""
        """{'static': system_prompt, 'dynamic': user_prompt}"""
        # If we have a real goal (not placeholder), return it
        if self.current_goal:
            return self.current_goal

        if new_goal and new_goal.name != 'GOAL_NEEDED':
            self.current_goal = new_goal
            return self.current_goal
        
        # We have placeholder goal or no goal - create a real goal
        if new_goal and new_goal.name == 'GOAL_NEEDED':
            logger.info(f'🤔 {self.character_name} received placeholder goal, creating situated goal')

        # Create a new goal based on current situation
        self.map_update_request_publisher.put(json.dumps({'type': 'goal_look'}))
        system_prompt = observations['static']
        user_prompt = observations['dynamic']
        directive = """What would you like to do next? 
Consider:
1. What is the central issue / opportunity / obligation demanding the character's attention?
2. Given the following available information about the character, the situation, and the surroundings, how can the character best satify their drives?
3. Identify any other actors involved in the goal, and their relationships to the character.
4. Each goal should be a candiate for the center of activity for the near future.
5. Goals must be distinct from one another.
6. Goals must be consistent with the character's drives and emotional stance.
7. Goals must be consistent with the available map types.

Nothing in this or other instructions limits your use of deception or surprise.
                  
Respond using the following hash-formatted text, where each tag is preceded by a # and followed by a single space, followed by its content.
Each goal should begin with a #goal tag, and should end with ## on a separate line as shown below:
be careful to insert line breaks only where shown, separating a value from the next tag:

#goal terse (5-8) words) name for this goal
#description concise (8-14) words) further details of this goal
#otherCharacterName name of the other actor involved in this goal, or None if no other actor is involved
#termination terse (5-6 words) statement of condition that would mark achievement or partial achievement of this goal. This should be a specific observable condition that can be checked for.
##

Respond ONLY with the above hash-formatted text.
end your response with </end>
"""
        # Make LLM call
        if self.llm_client and not self.shutdown_requested:
            # Use shorter timeout during shutdown
            timeout = 5.0 if self.shutdown_requested else None
            response = self.llm_client.generate(
                messages=[system_prompt, user_prompt, directive],
                max_tokens=400,
                temperature=0.5,
                stops=['</end>'],
                timeout=timeout
            )

            if response.success:
                logger.info(f'🤖 {self.character_name} New Goal: {response.text.strip()}')
                goals = []
                forms = hash_utils.findall_forms(response.text)
                for goal_hash in forms:
                    goal = plan_module.validate_and_create_goal(self.character_name, goal_hash)
                    if goal:
                        logger.info(f'{self.character_name} generated goal: {goal.to_string()}')
                        self.current_goal = goal
                        self.current_plan = None  # Clear plan so _plan creates new one for this goal
                        self._publish_goal(goal)
                        return self.current_goal
                    else:
                        logger.error(f'Warning: Invalid goal generation response for {goal_hash}')
            else:
                logger.error(f'LLM call failed: {response.error}')
                self.current_plan = None
                self.current_goal = plan_module.Goal('sleep', actors=[self.character_name])
                self._publish_goal(self.current_goal)
        else:   
            logger.error('LLM client not available')
            self.current_plan = None
            self.current_goal = plan_module.Goal('sleep', actors=[self.character_name])
            self._publish_goal(self.current_goal)
        return self.current_goal


    def _plan(self, goal):
        """Plan: Return existing plan or create single-action plan from goal."""
        # If we already have a plan, return it
        if self.current_plan is not None:
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
                    for i, memory in enumerate(entity_context['conversation_history']):  # Use last 2 memories
                        goal_prompt += f"\t{memory['source']}: {memory['text']}\n"

            if self.action_history and (self.action_history[-1].action['type'].lower() == 'say' or self.action_history[-1].action['type'].lower() == 'response'):
                directive = f"""\nrespond only with the JSON plan, no other text.\nend your response with </end>"""
            else:
                directive = f"""\nrespond only with the JSON plan, no other text.\nend your response with </end>"""

            self.current_plan_prompt = system_prompt + user_prompt + goal_prompt + PLAN_TEMPLATE + directive
            # Make LLM call
            #self.current_plan_prompt= self.llm_client.substitute_bindings(self.current_plan_prompt_template, self.current_plan_prompt_bindings)
            if self.llm_client and not self.shutdown_requested:
                response = self.llm_client.generate(
                    messages=[system_prompt, user_prompt, goal_prompt, PLAN_TEMPLATE, directive],
                    max_tokens=1500,
                    temperature=0.7,
                    stops=['</end>'],
                    is_json=True
                )

                if response.success:
                    logger.debug(f'🤖 {self.character_name} New Action: {response.text}')
                    plan_candidate = None
                    valid = False
                    try:
                        plan_candidate = response.text #json.loads(response.text.strip())
                        valid = plan_module.verify_plan(plan_candidate)
                        if not valid:
                            logger.error(f'Invalid plan JSON in LLM response: {response.text}')
                            plan_candidate = None
                    except Exception as e:
                        logger.error(f'Invalid plan JSON in LLM response: {e}')
                        plan_candidate = None
                    if not plan_candidate or not plan_candidate.get('plan') or len(plan_candidate['plan']) == 0:
                        logger.error(f'No action, target, or value found in LLM response: {response.text}')
                        single_action = {'type': 'sleep', 'target': 'self', 'value': '', 'reason': ''}
                    else:
                        self.current_plan = plan_candidate
                        self.plan_bindings_cache = {}
                        self.plan_summary_completed = False  # Reset for new plan
                        # Initialize plan identifiers and control-flow events
                        self.plan_counter += 1
                        self.current_plan_id = f"p_{self.plan_counter}"
                        self.control_flow_events = []
                        self.step_counter = 0
                        self._publish_current_plan()
                        self.plan_state = {'step_stack': plan_module.Stack()}
                        return self.current_plan
                else:
                    logger.error(f'LLM call failed: {response.error}')
                    single_action = {'type': 'sleep', 'target': 'self', 'value': '', 'reason': ''}
            else:   
                logger.error('LLM client not available')
                single_action = {'type': 'sleep', 'target': 'self', 'value': '', 'reason': ''}
        
        # Create single-action plan
        if single_action:
            self.current_plan = {'plan': [{'type': single_action['type'], 'target': single_action['target'], 'value': single_action['value'], 'reason': single_action.get('reason', '')}]}   
        self.plan_summary_completed = False  # Reset for new plan
        # Initialize plan identifiers and control-flow events
        self.plan_counter += 1
        self.current_plan_id = f"p_{self.plan_counter}"
        self.control_flow_events = []
        self.step_counter = 0
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
        if self.llm_client and not self.shutdown_requested:
            response = self.llm_client.generate(
                messages=[system_prompt, user_prompt, PLAN_TEMPLATE, directive],
                max_tokens=200,
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
                        self._publish_goal(goal)
                        return self.current_goal
                    else:
                        logger.error(f'Warning: Invalid goal generation response for {goal_hash}')
            else:
                logger.error(f'LLM call failed: {response.error}')
                self.current_goal = plan_module.Goal('sleep', actors=[self.character_name])
                self.current_plan = None  # Clear plan so _plan creates new one for this goal
                self._publish_goal(self.current_goal)
        else:   
            logger.error('LLM client not available')
            self.current_goal = plan_module.Goal('sleep', actors=[self.character_name])
            self.current_plan = None  # Clear plan so _plan creates new one for this goal
            self._publish_goal(self.current_goal)
        return self.current_goal
        
    def _plan_completed(self):
        """Handle successful plan completion."""
        # Existing telemetry and cleanup
        self._summarize_plan_execution()
        self.current_plan = None
        self.plan_bindings_cache = {}
        self.action_history = []
        self.plan_state = None
        
        # Handle activity advancement
        if hasattr(self, 'activity_manager') and self.activity_manager.has_active_activity():
            # Advance to next activity step
            next_step, activity = self.activity_manager.step_completion('success')
            self._publish_current_activity()
            if next_step:
                # Continue with next step - set new goal to trigger planning
                self.current_goal = plan_module.Goal(
                    next_step['name'], 
                    next_step['actors'], 
                    next_step['description'], 
                    next_step['termination']
                )
                self._publish_goal(self.current_goal)   
                logger.info(f'🎯 Activity step completed, advancing to: {next_step["name"]}')
            else:
                # Activity completed - clear goal
                self.current_goal = None
                self._publish_goal(self.current_goal)
                logger.info(f'✅ Activity completed for {self.character_name}')
        else:
            # No activity - clear goal (existing behavior)
            self.current_goal = None
            self._publish_goal(self.current_goal)

        
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
            return None # how did we get here?
        
        step_stack = self.plan_state['step_stack']
        
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
            if action:
                self.current_action = action
                self._publish_decided_action(action)
            return action
        except Exception as e:
            logger.error(f'Error in plan execution: {e}')
            traceback.print_exc()
            # Clear plan state on error
            self.plan_state = {
                'step_stack': plan_module.Stack()
            }
            self._plan_completed()
            return None
    
    def _execute_next_step(self, step_stack):
        """Execute next step using frame-based stack."""
        if step_stack.is_empty():
            logger.error('Step stack is empty')
            return None

        def _cond_outcome(cond):
            result = plan_module._evaluate_condition(self, cond)
            return result['value'], result['binding'] if result['binding'] else None

        current = step_stack.peek()
        plan = current['plan']
        idx = current['idx']

        # ---- Completed current frame? ----
        if idx >= len(plan):
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

        # Primitive actions (spec-compliant)
        if stype in ('move', 'say', 'think', 'take', 'inspect', 'use'):
            current['idx'] = idx + 1
            return step

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
                    'max_iterations': 5
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

        else:
            # Unknown or non-executable type: skip
            logger.error(f'Unknown or non-executable step type: {stype}')
            current['idx'] = idx + 1
            return self._execute_next_step(step_stack)
    

    def _act(self, action: Dict[str, Any]) -> bool:
        """Act: Execute the chosen action. Returns True if action succeeded, False if it failed."""

        # Publish time advancement proposal for testing TBD - condition on type
        proposed_minutes = random.randint(2, 5)
        self.publish_time_proposal(proposed_minutes)

        action = self.current_action if self.current_action else action
        
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
        self.action_history.append(action_record)
        
        if action['type'].lower() == "sleep":
            action_data = {'type': 'sleep','action_id': self.action_counter,'timestamp': datetime.now().isoformat(),'target': self.character_name}
            self.action_publisher.put(json.dumps(action_data))
            action_record.result = 'slept'
            time.sleep(1)  # Sleep for 1 second
            return True
        if action['type'].lower() == 'think':
            thought = self.think_about(action)
            return True
            
        if action['type'].lower() == "move":
            action_type = action['type'].lower()
            if action_type == "move":
                move_direction = None
                move_target = action['target'].strip().lower()
                # Step 1: Test for compass points first (case insensitive)
                cardinal_directions = ['north', 'northeast', 'southeast', 'south', 'southwest', 'northwest', 'east', 'west']
                if move_target in cardinal_directions:
                    move_direction = move_target
                else:
                    cond_action = action.copy()
                    cond_action['type'] = 'can_see'
                    cond_action['target'] = move_target.capitalize()
                    binding = plan_module._evaluate_condition(self, cond_action)
                    outcome = binding['value']
                    resolved = binding['binding']
                    if outcome:
                        move_direction = self._find_target_direction(resolved)
                    else:
                        move_direction = None
                if move_direction:
                    self.move(move_direction)
                # Request situation/map update for UI immediately after move
                try:
                    self.map_update_request_publisher.put(json.dumps({'type': 'step_look'}))
                except Exception:
                    pass
                return True
            else:
                logger.error(f'❌ Cannot move toward "{move_target}" - target not resolved or visible, choosing random direction')
                self.move(random.choice(cardinal_directions))
                try:
                    self.map_update_request_publisher.put(json.dumps({'type': 'step_look'}))
                except Exception:
                    pass
                return True
        elif action['type'].lower() == "say":
            # Check if we can acquire conversation lock
            # Capture the originally requested target for UI/reporting
            requested_target = action.get('target', '')
            resolved_target = self._resolve_target(action)

            if resolved_target and self._acquire_conversation_lock(resolved_target):
                self.generate_speech(action['value'], resolved_target, mode='say')
            else:
                # Lock acquisition failed, publish failure action
                action_data = {
                    'type': 'say',
                    'action_id': self.action_counter,
                    'timestamp': datetime.now().isoformat(),
                    'target': resolved_target if resolved_target else '',                    
                    'requested_target': requested_target,
                    'resolved_target': resolved_target if resolved_target else '',
                    'text': action.get('value', ''),
                    'status': 'failed',
                    'error': 'conversation lock unavailable'
                }
                self.action_publisher.put(json.dumps(action_data))
                logger.error('📤 Published action: say (failed - lock unavailable)')
                self.action_counter += 1
                return False
        elif action['type'].lower() == "take":
            # If resolution failed, publish a failure action with requested_target
            cond_action = action.copy()
            cond_action['type'] = 'near'
            cond_action['target'] = action['target'].strip().capitalize()
            binding = plan_module._evaluate_condition(self, cond_action)
            outcome = binding['value']
            resolved = binding['binding']   
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
                self.action_counter += 1
                return False
            self.take(resolved)
            self.action_counter += 1
        elif action['type'].lower() == "inspect":
            # Publish inspect action including requested vs resolved target
            cond_action = action.copy()
            cond_action['type'] = 'near'
            cond_action['target'] = action['target'].strip().capitalize()
            binding = plan_module._evaluate_condition(self, cond_action)
            outcome = binding['value']
            resolved = binding['binding']
            if not outcome:
                cond_action['type'] = 'has_item'
                binding = plan_module._evaluate_condition(self, cond_action)
                outcome = binding['value']
                resolved = binding['binding']

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
            else:
                # Perform inspect which may populate last_action_result
                self.inspect(action, resolved)
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
        elif action['type'].lower() == "use":
            # Create action - include requested vs resolved
            cond_action = action.copy()
            cond_action['type'] = 'near'
            cond_action['target'] = action['target'].strip().capitalize()
            binding = plan_module._evaluate_condition(self, cond_action)
            outcome = binding['value']
            resolved = binding['binding']
            if not outcome:
                cond_action['type'] = 'has_item'
                binding = plan_module._evaluate_condition(self, cond_action)
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
            if not resolved:
                action_data['status'] = 'failed'
                action_data['error'] = 'target not nearby/visible'
                return False
            else:
                self.use(action, resolved, action_data)
                action_data['result'] = self.action_history[-1].result
            self.action_publisher.put(json.dumps(action_data))
            self.action_counter += 1
            return True

        # Request situation/map update for UI after non-move actions that may affect visibility/adjacency
        try:
            if action['type'].lower() in ["take", "inspect", "use"]:
                self.map_update_request_publisher.put(json.dumps({'type': 'step_look'}))
        except Exception:
            pass

        # Finalize timing and outcome inference
        action_record.ended_at = datetime.now()
        try:
            action_record.duration_ms = int((action_record.ended_at - action_record.started_at).total_seconds() * 1000)
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
        # Reset turn-specific state
        self.text_input_pending = False
        self.interrupt_pending = False
        
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
        
        # TODO: Complete the LLM prompt and call
        # This is a stub - the actual LLM integration will be implemented in the next phase
        summary_prompt = f"""
        Goal: {goal_text}
        
        Plan: {plan_text}
        
        Actions Taken:
        {actions_summary}
        
        Please provide a concise paragraph summarizing this plan execution, including the goal, actions taken, and observed results.
        Do not include any other introductory, explanatory, discursive, or formatting text in your response.
        End your text with: </end>
        """
        response = self.llm_client.generate([summary_prompt], max_tokens=200, stops=['</end>'])
        self.plan_summary = response.text
        logger.info(f'📝 Plan post-mortem prepared for {self.character_name}\n{self.plan_summary}\n')
        
        # Mark as completed to prevent redundant calls
        self.plan_summary_completed = True
        self.plan_log.append({'goal': self.current_goal.to_string(), 'prompt': self.current_plan_prompt, 'plan': self.current_plan, 'summary': self.plan_summary, 'actions': self.action_history})
        self.review_planning()
        self.action_history = []
    
    def review_planning(self):
        """Review the planning process and update the plan prompt and bindings."""
        if len(self.plan_log) < 3:
            return
        logger.info(f'📝 Reviewing planning for {self.character_name}')
        system_prompt = f"""Review the following planning information for one or more planning efforts and recommend improvements to the content or instructions in the Plan syntax or the planning prompt.
"""
        
        actions_text = []
        for record in self.action_history:
            action_type = record.action.get('type', 'unknown')
            target = record.action.get('target', 'unknown')
            result = record.result if record.result else 'no result recorded'
            timestamp = record.timestamp.strftime('%H:%M:%S')
            actions_text.append(f"{timestamp} - {action_type}: {target} -> {result}")
        
        actions_summary = '\n'.join(actions_text)

        user_prompt = f"""
#Plan syntax specification:
{PLAN_TEMPLATE}

"""
        for n, item in enumerate(self.plan_log):
            user_prompt += f"""

#### Planning effort {n+1}

# Goal the plan was created for: 
{self.plan_log[-1]['goal']}

#Planning prompt:
{self.plan_log[-1]['prompt']}

#Resulting Plan:
{json.dumps(self.plan_log[-1]['plan'], indent=2)}

#Actions taken:
{actions_summary}

#Plan post-mortem summary:
{self.plan_summary}

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

        response = self.llm_client.ask(bindings={}, prompt=[SystemMessage(content=system_prompt), UserMessage(content=user_prompt)], 
                                       max_tokens=800, stops=['</end>'], is_json=False)
        logger.info(f'📝 Planning review for {self.character_name}: {response}')
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
                self.text_input_pending = True
                logger.info(f'📥 {self.character_name} Received text input: "{text_input}" (source: {source})')
                
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
                self._replan(self.current_goal, reason)
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
            
        except Exception as e:
            logger.error(f'Error publishing dialog end to {source}: {e}')
 

    def parse_and_set_goal(self, goal_text):
        """Parse goal input from UI and set current goal."""
        try:
            parsed_goal = goal_text.strip().strip('"').strip("'")[6:]
            if not self.observations:
                self._observe()
            self.current_goal = plan_module.Goal(parsed_goal, [self.character_name], description='supplied by user', termination='')
            # In manual mode, do not auto-plan/replan; just publish goal
            if self.manual:
                self._publish_goal(self.current_goal)
            else:
                self._plan_completed()
            return
        except Exception as e:
            logger.error(f"Goal parsing failed for {self.character_name}: {e}")
            traceback.print_exc()
            return

    def parse_and_set_plan(self, plan_text):
        """Parse plan input from UI and set current plan."""
        try:
            parsed_plan = plan_module.parse_plan_text(plan_text)
            self._plan_completed()  # Clear any existing plan
            self.current_plan = parsed_plan
            self.plan_bindings_cache = {}
            self.plan_summary_completed = False  # Reset for new plan
            self._publish_current_plan()
            self.plan_state = {
                'step_stack': plan_module.Stack()
            }
            # Clear plan bindings cache for new plan
            self.plan_bindings_cache = {}
            if not plan_module.verify_plan(self.current_plan):
                logger.error(f"Invalid plan for {self.character_name}")
                return
            logger.info(f"📋 {self.character_name} received new plan with {len(parsed_plan['plan'])} steps")
        except Exception as e:
            logger.error(f"Plan parsing failed for {self.character_name}: {e}")
            # Plan assignment failed - character continues with existing behavior

    def generate_speech(self, text_input: str, source: str, mode: str = 'say'):
        """In say mode, this is start of conversation. In respond mode, this is a response in an ongoing dialog"""
        # Handle plan input from UI - strip quotes if present
        clean_input = text_input.strip().strip('"').strip("'")
        if source == 'User' and clean_input.startswith('goal:'):
            self.parse_and_set_goal(clean_input)
            return
        if source == 'User' and clean_input.startswith('plan:'):
            self.parse_and_set_plan(clean_input)
            return
        if self.character_name == 'User' and mode == 'respond':
            self.publish_dialog_end(source)
            return
        # In manual mode with manual_response disabled, do not auto-respond
        if self.manual and self.manual_response:
            return ''
        text_to_send = ''
        try:
            if mode == 'respond':
                logger.info(f'Responding to: "{text_input}" from {source}')
            else:
                logger.info(f'Saying intent: "{text_input}" to {source}')
            
            entity_context = self.get_entity_context(source, 10)
            # Build user prompt with context
            dialog_history = '' 
            if entity_context and isinstance(entity_context, dict):
                conversation = entity_context.get('conversation_history', [])
                if isinstance(conversation, list):
                    dialog_history += f"Your recent conversation with {source} has been:\n"
                    for i, memory in enumerate(conversation):
                        if isinstance(memory, dict) and 'source' in memory and 'text' in memory:
                            dialog_history += f"\t{memory['source']}: {memory['text']}\n"

            # Check if dialog should naturally end
            if mode == 'respond':
                are_we_done = self.check_natural_dialog_end(source, text_input)
                logger.info(f'🤖 {self.character_name} Dialog end check: {are_we_done}')
                if are_we_done:
                    reason = f'Dialog end detected with {source}, dialog_history:\n{dialog_history}'
                    self.publish_dialog_end(source)
                    if not self.manual:
                        self._replan(self.current_goal, reason)
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

            user_prompt +=  """Speak in a conversational manner in your own voice. 
Do not invent knowledge not contained in the data of the current situation. 
For example if a resource is listed (e.g. mushroom29), do not invent knowledge about it beyond general knowledge of mushroomsnot contained above.
Do not include any other introductory, explanatory, discursive, or formatting text in your response.
End your text with: </end>"""
                    
            # Make LLM call
            if self.llm_client and not self.shutdown_requested:
                # Use shorter timeout during shutdown
                timeout = 5.0 if self.shutdown_requested else None
                response = self.llm_client.generate(
                    messages=[system_prompt, user_prompt],
                    max_tokens=200,
                    temperature=0.7,
                    timeout=timeout,
                    stops=['</end>']
                )

                if response.success:
                    text_to_send = response.text.strip()
                    if not text_to_send.lower().startswith('done'):
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
                        'text': response.text.strip() if (response and response.success) else text_to_send,
                        'source': self.character_name if mode == 'say' else source,
                        'target': source if mode == 'say' else None
        }

        if self.action_history:
            self.action_history[-1].result = text_to_send
        # Publish action (this will be picked up by memory_node and action_display_node)
        self.action_publisher.put(json.dumps(action_data))
        logger.info(f'📤 Published action: {action_data["action_id"]}')
        self.action_counter += 1
        return text_to_send

    
    def _resolve_target(self, action: Dict[str, Any]) -> Union[str, bool]:
        """
        Resolve abstract target name to specific instance reference using plan bindings cache.
        This ONLY handles abstract -> specific resolution (e.g., "Berry" -> "Berry23").
        Note this is merely attempting to resolve the target to a specific map instance. It is NOT evaluating the target wrt the action.
        Nonetheless, for 'near', for example, we should attempt to resolve to the closest instance, right?
        too hard.
        next - only instances in view? Let's say no. That means this must be a map_node query.
        
        Args:
            action: Action dictionary containing 'type' and 'target', or condition dictionary
            
        Returns:
            Resolved target name (str) or False if resolution fails
        """
        action_type = action.get('type', 'unknown')
        raw_target = action.get('target', None)
        
        if not raw_target:
            return None
            
        # Check plan bindings cache first
        cache_key = raw_target
        # TODO: Re-enable cache
        # disabling cache untill we implement invalidation (e.g. on move or visibility change)
        #if cache_key in self.plan_bindings_cache:
            #cached_result = self.plan_bindings_cache[cache_key]
            #logger.debug(f'🎯 Cache hit: {raw_target} -> {cached_result}')
            #action_copy = action.copy()
            #action_copy['target'] = cached_result
            #return action_copy
        
        logger.debug(f'🔍 Resolving target "{raw_target}" for action type "{action_type}"')
        resolved_target = False
        
        try:
            # Context-aware resolution based on action type
            action_type = action['type'].lower()
            negated = False
            if 'hasnt' in action_type or 'cant' in action_type or 'not' in action_type:
                negated = True

            if action_type == "move":
                move_target = action['target'].strip()
                move_direction = move_target.lower()
                # Step 1: Test for compass points first (case insensitive)
                cardinal_directions = ['north', 'northeast', 'southeast', 'south', 'southwest', 'northwest', 'east', 'west']
                if move_direction in cardinal_directions:
                    return move_direction
                else:
                    resolved_target = self._resolve_visible_instance(negated, raw_target)
                    if not resolved_target:
                        return None
                    move_direction = self._find_target_direction(resolved_target)
                    return move_direction
            if action_type in ['take', 'inspect', 'use']:
                # Resource actions/conditions - resolve to specific resource instance
                resolved_target = self._resolve_near_resource_instance(negated, raw_target)
                return resolved_target
                
            if action_type in ['has_item', 'hasnt_item']:
                # Resource actions/conditions - resolve to specific resource instance
                resolved_target = self._resolve_inventory_instance(negated, raw_target)
                if resolved_target and not negated:
                    return resolved_target
                elif not resolved_target and negated:
                    return True
                return False
                
            elif action_type in ['say', 'can_see', 'cant_see']:
                # Character actions/conditions - resolve to specific character name
                resolved_target = self._resolve_visible_instance(negated, raw_target)
                if resolved_target and not negated:
                    return resolved_target
                elif not resolved_target and negated:
                    return True
                return False
                
            elif action_type in ['near', 'notnear']:
                # Proximity conditions - could be character or resource
                resolved_target = self._resolve_near_instance(negated,raw_target)
                if resolved_target and not negated:
                    return resolved_target
                elif not resolved_target and negated:
                    return True
                return False
            
            elif action_type in ['at_location', 'notat_location']:
                # Location conditions - pass through for now
                resolved_target = self._resolve_at_instance(negated,raw_target)
                if resolved_target and not negated:
                    return resolved_target
                elif not resolved_target and negated:
                    return True
                return False
            else:
                logger.error(f'❓ Unknown action type for resolution: {action_type}')
                resolved_target = raw_target  # Pass through unchanged
                return resolved_target
        except Exception as e:
            logger.error(f'❌ Error resolving target "{raw_target}" for {action_type}: {e}')
            resolved_target = False
        
        # Cache the result (even if False or pass-through)
        if resolved_target:
            self.plan_bindings_cache[cache_key] = resolved_target
        
        if resolved_target and resolved_target != raw_target:
            logger.info(f'✅ Resolved "{raw_target}" -> "{resolved_target}" ({action_type})')
        elif resolved_target == raw_target:
            logger.debug(f'✅ Validated "{raw_target}" ({action_type})')
        else:
            logger.error(f'❌ Failed to resolve "{raw_target}" ({action_type})')
            
        return action

    def _resolve_visible_instance(self, negated: bool, raw_target: str) -> Union[str, bool]:
        """Resolve abstract visible instance to specific visible instance."""
        try:
            # visible means in current situation, so can be resolved locally
            target = raw_target.capitalize()
            if self.last_situation_data and target in self.last_situation_data.get('characters', []):
                return target
            for view in self.last_situation_data.get('views', []):
                if 'characters' in view:
                    for character in view['characters']:
                        if character.get('name', '') == target or target == 'person':
                            return character.get('name', '')
                if 'resources' in view: 
                    for resource in view['resources']:
                        if ((not any(ch.isdigit() for ch in target) and resource['name'].startswith(target)) 
                            or resource['name'] == target):
                            return resource.get('name', '')
                if 'terrain' in view:
                    if view['terrain'] == target:
                        return target
            return False
        except Exception as e:
            logger.error(f'Error resolving visible instance {raw_target}: {e}')
            return False
        
    def _resolve_near_instance(self, negated: bool, raw_target: str) -> Union[str, bool]:
        """Resolve abstract visible instance to specific visible instance."""
        try:
            # visible means in current situation, so can be resolved locally
            target = raw_target.capitalize()
            # check inventory first!
            resolved_target = self._resolve_inventory_instance(negated, raw_target)
            if resolved_target:
                return resolved_target
            
            for view in self.last_situation_data.get('views', []):
                if 'characters' in view:
                    for character in view['characters']:
                        if character.get('distance', 20) <= 2 and (character.get('name', '') == target or target == 'person'):
                            return character.get('name', '')
                if 'resources' in view:
                    for resource in view['resources']:  
                        if resource.get('distance', 20) <= 2 and ((not any(ch.isdigit() for ch in target) and resource['name'].startswith(target)) 
                            or resource['name'] == target):
                            return resource.get('name', '')
                if 'terrain' in view:
                    if view['terrain'] == target:
                        return target
            
            return False
        except Exception as e:
            logger.error(f'Error resolving visible instance {raw_target}: {e}')
            return False

    def _resolve_near_resource_instance(self, negated: bool, raw_target: str) -> Union[str, bool]:
        """Resolve abstract visible instance to specific visible instance."""
        try:
            # visible means in current situation, so can be resolved locally
            target = raw_target.capitalize()
            for view in self.last_situation_data.get('views', []):
                if 'resources' in view:
                    for resource in view['resources']:  
                        if resource.get('distance', 20) <= 2 \
                            and ((not any(ch.isdigit() for ch in target) and resource['name'].startswith(target)) or resource['name'] == target):                            
                            return resource.get('name', '')
            return False
        except Exception as e:
            logger.error(f'Error resolving visible instance {raw_target}: {e}')
            return False

    def _resolve_at_instance(self, negated: bool, raw_target: str) -> Union[str, bool]:
        """Resolve abstract visible instance to specific visible instance."""
        try:
            # visible means in current situation, so can be resolved locally
            target = raw_target.capitalize()
            for view in self.last_situation_data.get('views', []):
                if 'characters' in view:
                    for character in view['characters']:
                        if character.get('distance', 20) <= 1 and (character.get('name', '') == target or target == 'person'):
                            return character.get('name', '')
                if 'resources' in view:
                    for resource in view['resources']:  
                        if resource.get('distance', 20) <= 2 and ((not any(ch.isdigit() for ch in target) and resource['name'].startswith(target)) 
                            or resource['name'] == target):                            
                            return resource.get('name', '')
                if 'terrain' in view:
                    if view['terrain'] == target:
                        return target
            return False
        except Exception as e:
            logger.error(f'Error resolving visible instance {raw_target}: {e}')
            return False

    
    def _resolve_inventory_instance(self, negated: bool, raw_target: str) -> Union[str, bool]:
        """Resolve abstract resource name to specific resource instance."""
        try:
            # First try exact match validation
            for reply in self.session.get(f"cognitive/{self.character_name}/memory/inventory?item={raw_target}", timeout=2.0 if not self.debug else 600.0):
                if reply.ok:
                    data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                    if data.get('success'):
                        logger.debug(f'✅ Exact resource instance: {raw_target}')
                        return raw_target
                break
            
            # First try exact match validation
            for reply in self.session.get(f"cognitive/map/bind/resource/{raw_target}", timeout=2.0 if not self.debug else 600.0):
                if reply.ok:
                    data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                    if data.get('success'):
                        resolved_target = data.get('resolved_target', raw_target)
                        logger.debug(f'✅ Exact resource instance: {raw_target} -> {resolved_target}')
                        return resolved_target
                break
            
            return False
            
        except Exception as e:
            logger.error(f'Error resolving resource instance {raw_target}: {e}')
            return False
            
    def _find_target_direction(self, target: str) -> str:
        """Find which direction a target (character or resource) is visible in."""
        try:
            # Query situation node for current situation data
            for reply in self.session.get(f"cognitive/{self.character_name}/situation/current_situation", timeout=5.0 if not self.debug else 600.0):
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
                    target_name = target.capitalize()
                    
                    for view in views:
                        direction = view.get('direction', {})
                        
                        # Check resources in this direction
                        if view['terrain'] == target_name:
                            return direction
                        
                        if 'resources' in view:
                            for resource in view['resources']:
                                if resource.get('name') == target_name:
                                    logger.info(f'🎯 Found target "{target}" as resource in direction: {direction}')
                                    return direction
                        
                        # Check characters in this direction
                        if 'characters' in view:
                            for character in view['characters']:
                                character_name = character.get('name', '') 
                                if character_name == target_name:
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

    def move(self, move_direction: str):
        """Move the character in the specified direction."""
        logger.info(f'Moving {move_direction}')
        
        try:
            # Query map node to move the agent
            move_data = {'direction': move_direction}
            for reply in self.session.get(f"cognitive/map/agent/{self.character_name}/move", payload=json.dumps(move_data).encode('utf-8'), timeout=5.0 if not self.debug else 600.0):
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
            
            # Update the most recent action record with the result
            if self.action_history:
                self.action_history[-1].result = f'moved in direction {move_direction}'
            
        except Exception as e:
            # Update the most recent action record with the error result
            if self.action_history:
                self.action_history[-1].result = f'move failed {e}\n'
            logger.error(f'Error in move operation: {e}')
            # Create and publish action data
        action_data = {
                'type': 'move',
                'direction': move_direction,
                'action_id': f"move_{int(time.time())}",
                'timestamp': datetime.now().isoformat()
            }
        self.action_publisher.put(json.dumps(action_data).encode('utf-8'))
    
    def take(self, target: str):
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
            for reply in self.session.get(f"cognitive/map/resource/remove/{target}", timeout=2.0 if not self.debug else 600.0   ):
                if reply.ok:
                    data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                    if data.get('success'):
                        logger.info(f'🗑️ Removed {target} from map')
                    else:
                        logger.error(f'⚠️ Failed to remove {target} from map: {data.get("error", "Unknown error")}')
                else:
                    logger.error(f'⚠️ Failed to remove {target} from map - no response')
                break

            return True

        try:
            result = _do_take(target)
            # Create and publish action data for logging/display
            action_data = {
                'type': 'take',
                'target': target,
                'action_id': f"take_{int(time.time())}",
                'timestamp': datetime.now().isoformat(),
                'character': self.character_name
            }
            self.action_publisher.put(json.dumps(action_data))
            logger.info(f'📤 Published take action: {target}')
            return True

        except Exception as e:
            logger.error(f'Error in take operation for {target}: {e}')
            if self.action_history:
                self.action_history[-1].result = f'take failed'

        # Create and publish action data for logging/display
        action_data = {
            'type': 'take',
            'target': target,
            'action_id': f"take_{int(time.time())}",
            'timestamp': datetime.now().isoformat(),
            'character': self.character_name
        }
        self.action_publisher.put(json.dumps(action_data))
        logger.info(f'📤 Published take action: {target}')
        return True
    
    def inspect(self, action: dict, target: str):
        """Learn about a resource."""
            # Create and publish action data for logging/display
        action_data = {
                'type': 'inspect',
                'target': target,
                'action_id': f"take_{int(time.time())}",
                'timestamp': datetime.now().isoformat(),
                'character': self.character_name,
        }
        try:
            system_prompt = self._update_system_prompt()
            user_prompt = self.observations['dynamic']

            directive = f"""You are inspecting: {target} to gather information about {action['reason']}.
respond with a short text description of the resource consistent with its name and situated in the situation.
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
        self.action_publisher.put(json.dumps(action_data))

    def use(self, action: dict, target: str, action_data: dict):
        """Use a resource."""
            # Create and publish action data for logging/display
        try:
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
                    action_data['response'] = response.text
                    return True
                else:
                    logger.error(f'LLM call failed: {response.error}')
            else:
                logger.error('LLM client not available')

            logger.info(f'📦 Inspecting {target} for {self.character_name}')
            if self.action_history:
                self.action_history[-1].result = f'{response.text}'
            return True

        except Exception as e:
            logger.error(f'Error in inspect operation for {target}: {e}')
            if self.action_history:
                self.action_history[-1].result = f'inspect failed'
        self.action_publisher.put(json.dumps(action_data))
        
    def think_about(self, action: dict):
        """Think about a value."""
        logger.info(f'Thinking about: {action}')
        thought = ''
        try:
            system_prompt = self._update_system_prompt()
            user_prompt = self.observations['dynamic']

            directive = f"""You are thinking about: {action['value']}.\n\n Derive new information, insights, goals, or conclusions based on your memories, drives, and the current situation.
    This new information should be a short statement (10 words max) not explicit in the information provided that will guide your future thoughts and actions.
    Respond with the new information in the following hash-formatted syntax:

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
            for reply in self.session.get(f"cognitive/{self.character_name}/memory/chat/*", timeout=3.0 if not self.debug else 600.0):
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
    
    def get_entity_context(self, entity_name: str, limit: int = 20, scope='all') -> Dict[str, Any]:
        """
        Query entity data from memory node for context.
        
        Args:
            entity_name: Name of the entity to query
            limit: Number of recent conversation entries to include (default 20)
            
        Returns:
            Dictionary with entity data or None if query failed
        """
        try:
            # Query entity data from memory node with query and limit parameters
            for reply in self.session.get(f"cognitive/{self.character_name}/memory/entity/{entity_name}?query=dialog&limit={limit}&scope={scope}", timeout=3.0 if not self.debug else 600.0):
                try:
                    if reply.ok:
                        data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                        if data['success']:
                            logger.info(f'👥 Retrieved entity context for {entity_name}')
                            return data['entity_data']
                        else:
                            logger.debug(f'Entity query failed for {entity_name}: {data.get("error", "Unknown error")}')
                            return None
                except Exception as e:
                    logger.error(f'Error parsing entity query response for {entity_name}: {e}')
                    return None
            
            logger.debug(f'No response received for entity query: {entity_name}')
            return None
            
        except Exception as e:
            logger.error(f'Error querying entity context for {entity_name}: {e}')
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
            encoded_text = urllib.parse.quote(input_text)
            query_url = f"cognitive/{self.character_name}/memory/entity/{entity_name}?query=natural_dialog_end&input_text={encoded_text}&context={self.observations['static']}"
            
            for reply in self.session.get(query_url, timeout=10.0 if not self.debug else 600.0):
                try:
                    if reply.ok:
                        data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                        if data['success']:
                            logger.info(f'🤔 Natural dialog end check for {entity_name}: {data.get("should_end", True)}')
                            return data.get("should_end", True)
                        else:
                            logger.debug(f'Natural dialog end query failed for {entity_name}: {data.get("error", "Unknown error")}')
                            return True
                except Exception as e:
                    logger.error(f'Error parsing natural dialog end response for {entity_name}: {e}')
                    return True
            
            logger.debug(f'No response received for natural dialog end query: {entity_name}')
            return True
            
        except Exception as e:
            logger.error(f'Error querying natural dialog end for {entity_name}: {e}')
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