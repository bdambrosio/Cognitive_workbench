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
from utils.zenoh_utils import datetime_handler
from dataclasses import dataclass, asdict
import os
from templates import DRIVE_ASSESSMENT_TEMPLATE, GOAL_TEMPLATE, PLAN_TEMPLATE, PLAN_VERBS, REWRITE_TEMPLATE
from utils.format_utils import format_map_types, format_views_compact
from weakref import WeakValueDictionary
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


# ============================================================================
# Plan support classes (extracted from plan.py for infospace-only usage)
# ============================================================================

class Stack:
    """Simple stack implementation for plan state."""
    def __init__(self):
        self.stack = []

    def push(self, item):
        self.stack.append(item)

    def pop(self):
        if not self.is_empty():
            return self.stack.pop()
        return None

    def peek(self):
        if not self.is_empty():
            return self.stack[-1]
        return None

    def is_empty(self):
        return len(self.stack) == 0

    def size(self):
        return len(self.stack)
    
    def get_entries(self):
        """Return stack entries as a list, oldest first"""
        return self.stack.copy()


class Goal:
    """Goal representation for executive node."""
    _id_counter = 0
    _instances = WeakValueDictionary()
    
    def __init__(self, name, actors, description='', termination=None):
        Goal._id_counter += 1
        self.id = f"g{Goal._id_counter}"
        Goal._instances[self.id] = self
        self.name = name
        self.actors = actors
        self.description = description
        self.termination = termination
        self.task_plan = []
        self.tasks = []
        self.completion_statement = ''

    def __eq__(self, other):
        if not isinstance(other, Goal):
            return False
        return self.id == other.id

    def __hash__(self):
        return hash(self.id)
    
    @classmethod
    def get_by_id(cls, id: str):
        return cls._instances.get(id)
    
    def short_string(self):
        return f'{self.name}: {self.description}. termination: {self.termination}'
    
    def to_string(self):
        return f"Goal {self.name}: {self.description}; actors: {', '.join([character_name for character_name in self.actors])}; termination: {self.termination}"


def validate_and_create_goal(character_name, goal_hash):
    """Validate a goal hash and create a goal object."""
    goal_name = hash_utils.find('goal', goal_hash)
    description = hash_utils.find('description', goal_hash)
    other_character_name = hash_utils.find('otherCharacterName', goal_hash)
    termination = hash_utils.find('termination', goal_hash)

    if other_character_name and other_character_name.strip().lower() != 'none':
        other_character_name = other_character_name.strip().capitalize()
    else:
        other_character_name = None

    if goal_name and description and termination:
        goal = Goal(
            name=goal_name, 
            actors=[character_name, other_character_name] if other_character_name else [character_name],
            description=description, 
            termination=termination.replace('##','').strip()
        )
        return goal
    else:
        logger.warning(f"Invalid goal generation response for {goal_hash}") 
        return None


def parse_plan_json(plan_text):
    """Parse JSON plan string into internal plan structure."""
    plan_text = plan_text.strip()
    
    # Remove 'plan:' prefix if present
    if plan_text.startswith('plan:'):
        plan_text = plan_text[5:].strip()
    
    try:
        parsed = json.loads(plan_text)
        
        # If it's already wrapped, return as-is
        if isinstance(parsed, dict) and 'plan' in parsed:
            return parsed
        
        # If it's an array, wrap it
        if isinstance(parsed, list):
            return {'plan': parsed}
        
        # If it's a single action dict, wrap in array then dict
        if isinstance(parsed, dict):
            return {'plan': [parsed]}
        
        return {'plan': []}
    except json.JSONDecodeError as e:
        logger.error(f"Failed to parse plan JSON: {e}")
        return {'plan': []}


def verify_plan(plan_json):
    """Basic plan verification - checks structure only."""
    if isinstance(plan_json, str):
        try:
            plan_json = json.loads(plan_json)
        except json.JSONDecodeError:
            logger.error("Invalid JSON in plan")
            return False
    
    if isinstance(plan_json, dict) and 'plan' in plan_json:
        plan_steps = plan_json['plan']
        if not isinstance(plan_steps, list):
            return False
        # Basic check - each step should have 'type'
        for step in plan_steps:
            if not isinstance(step, dict) or 'type' not in step:
                return False
        return True
    
    return False


# ============================================================================
# End of plan support classes
# ============================================================================


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
        
        # === ZENOH PUBLICATION ===
        # NAME: action
        # TOPIC: cognitive/{character}/action
        # DESCRIPTION: Character completed an action (move, communicate, search, etc)
        # PAYLOAD: {"type": str, "result": Any, "timestamp": str, "action_record": dict}
        # TRIGGERS: EndOfDayReview, AssessPerformance, ReviewErrors
        # ========================
        self.action_publisher = self.session.declare_publisher(f"cognitive/{character_name}/action")

        # === ZENOH PUBLICATION ===
        # NAME: plan_log
        # TOPIC: cognitive/{character}/planning/plan_log
        # DESCRIPTION: Plan execution log entry for analytics
        # PAYLOAD: {"plan_id": str, "step": dict, "result": Any, "timestamp": str}
        # TRIGGERS: DebugFailedPlan, BuildPlanLibrary, ReviewPlanQuality
        # ========================
        self.plan_log_publisher = self.session.declare_publisher(
            f"cognitive/{character_name}/planning/plan_log"
        )
        
        
        # === ZENOH PUBLICATION ===
        # NAME: goal
        # TOPIC: cognitive/{character}/goal
        # DESCRIPTION: New goal set (from UI or autonomous)
        # PAYLOAD: {"goal": str, "source": str, "timestamp": str}
        # TRIGGERS: WeeklyPlanning, AssessKnowledgeGaps
        # ========================
        self.goal_publisher = self.session.declare_publisher(f"cognitive/{character_name}/goal")
        
        # === ZENOH PUBLICATION ===
        # NAME: decided_action
        # TOPIC: cognitive/{character}/decided_action
        # DESCRIPTION: Character decided on next action (before execution)
        # PAYLOAD: {"action": str, "rationale": str, "timestamp": str}
        # TRIGGERS: EvaluateMethodology, AssessPerformance
        # ========================
        self.decided_action_publisher = self.session.declare_publisher(f"cognitive/{character_name}/decided_action")
        
        # === ZENOH PUBLICATION ===
        # NAME: current_plan
        # TOPIC: cognitive/{character}/current_plan
        # DESCRIPTION: Plan state changed (started, step completed, failed, completed)
        # PAYLOAD: {"status": str, "plan": dict, "step_index": int, "failure_reason": str}
        # TRIGGERS: HandlePlanFailure, DebugFailedPlan, RepairPlan, LearnFromFailure
        # ========================
        self.current_plan_publisher = self.session.declare_publisher(f"cognitive/{character_name}/current_plan")
        
        # === ZENOH PUBLICATION ===
        # NAME: current_activity
        # TOPIC: cognitive/{character}/current_activity
        # DESCRIPTION: Activity pattern changed (started, completed, switched)
        # PAYLOAD: {"activity": str, "status": str, "timestamp": str}
        # TRIGGERS: EvaluateMethodology, OptimizeWorkflow
        # ========================
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
        llm_config = self.character_config.get('llm_config', {})
        server_name = llm_config.get('server_name', 'openai')
        model_name = llm_config.get('model_name', 'gpt-4.1')
        
        self.llm_client = ZenohLLMClient(server_name=server_name, model_name=model_name, service_timeout=200.0 if not self.debug else 300.0)
        logger.info(f'🤖 LLM client initialized (server={server_name}, model={model_name})')
        
        # Create llm_generate wrapper function (unified interface for LLM calls)
        def llm_generate(messages, bindings=None, max_tokens=2000, temperature=0.7, is_json=False, stops=None):
            """Unified LLM generation interface - wrapper around llm_client.generate()."""
            return self.llm_client.generate(
                messages=messages,
                bindings=bindings,
                max_tokens=max_tokens,
                temperature=temperature,
                is_json=is_json,
                stops=stops if stops else ['</end>']
            )
        self.llm_generate = llm_generate
        
        # Initialize memory module (wraps EntityModel and discourse)
        from memory import Memory
        self.memory = Memory(
            character_name=self.character_name,
            llm_generate=self.llm_generate,
            persistence_path=f"data/memory/{self.character_name}_memory.json"
        )
        logger.info(f'🧠 Memory module initialized for {self.character_name}')
        
        # Subscriber for save_all command (forward to memory and resource_manager)
        self.save_subscriber = self.session.declare_subscriber(
            "cognitive/save_all",
            self._handle_save_command
        )
        logger.info(f'💾 Subscribed to cognitive/save_all')
        
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
            character=self,
            world_type=world_type,
            map_name=self.map_name,
            logger_instance=logger
        )
        self.available_tools = self.planner.available_tools
        
        if self.is_infospace:
            from infospace_executor import InfospaceExecutor
            from infospace_semantic_validator import InfospaceSemanticValidator
            from infospace_resource_manager import InfospaceResourceManager
            from pathlib import Path
            
            # Create resource manager for direct resource access
            self.resource_manager = InfospaceResourceManager(self.map_name, session=self.session)
            logger.info(f'📦 Resource manager initialized for {self.map_name}')
            
            # Load resources from file on startup
            self.resource_manager.load_from_file()
            logger.info(f'📂 Loaded resources from file for {self.map_name}')
            
            self.infospace_executor = InfospaceExecutor(
                character_name,
                self.session,
                self.map_name,
                self.llm_client,
                self.available_tools,
                self,
                self.resource_manager
            )
            logger.info(f'🧩 Infospace executor initialized for {character_name}')
            
            # Initialize semantic validator
            tools_dir = Path(__file__).parent / 'tools'
            
            self.semantic_validator = InfospaceSemanticValidator(character=self, tools_dir=str(tools_dir) if tools_dir.exists() else None, prefix_prompt=self.planner.template)
            logger.info(f'🔍 Semantic validator initialized for {character_name}')
        
        # Internal state
        self.action_counter = 0
        self.last_sense_data = None
        
        # OODA loop state
        self.current_goal = None
        self.goal_source = None  # Track goal origin: 'ui', 'autonomous', or None
        self.awaiting_user_input = False  # Pause autonomous behavior after UI goal completion
        self.plan_just_generated = False  # Skip execution on same turn as plan generation
        self.observations = None
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

        # Execution control (replaces turn management)
        self.execution_paused = True  # Start paused, wait for step/run command
        self.execution_mode = 'step'  # 'step' or 'run'
        
        # Subscribers for direct execution control from UI
        self.control_step_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/control/step",
            self.handle_step_command
        )
        self.control_run_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/control/run",
            self.handle_run_command
        )
        self.control_stop_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/control/stop",
            self.handle_stop_command
        )
        
        # === ZENOH PUBLICATION ===
        # NAME: execution_state_update
        # TOPIC: cognitive/{character}/execution_state
        # DESCRIPTION: Current execution state (paused/running, mode)
        # PAYLOAD: {"paused": bool, "mode": str, "timestamp": float}
        # TRIGGERS: (UI updates)
        # ========================
        self.execution_state_publisher = self.session.declare_publisher(
            f"cognitive/{character_name}/execution_state"
        )
        
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
        
        # Subscriber for manual activity selection (character-specific)
        self.activity_selection_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/activity/set",
            self._activity_selection_callback
        )
        
        # Subscriber for enabling compliance tracking (evaluation mode)
        if self.is_infospace:
            self.compliance_tracking_subscriber = self.session.declare_subscriber(
                f"cognitive/{character_name}/enable_compliance_tracking",
                self._enable_compliance_tracking_callback
            )
        
        # Queryable for activity list (character-specific)
        # Queryables for resource management (for UI and resource_browser)
        if self.is_infospace:
            self.resource_view_queryable = self.session.declare_queryable(
                f"cognitive/{character_name}/resource/view/*",
                self._handle_resource_view_query
            )
            self.resources_list_queryable = self.session.declare_queryable(
                f"cognitive/{character_name}/resources",
                self._handle_resources_list_query
            )
            self.resource_by_id_queryable = self.session.declare_queryable(
                f"cognitive/{character_name}/resource/*",
                self._handle_resource_by_id_query
            )
            self.resource_remove_queryable = self.session.declare_queryable(
                f"cognitive/{character_name}/resource/remove/*",
                self._handle_resource_remove_query
            )
        
        self.activity_list_queryable = self.session.declare_queryable(
            f"cognitive/{character_name}/activity/list",
            self._activity_list_query_handler
        )
        
        # Queryable for sync plan execution (character-specific)
        self.sync_plan_execution_queryable = self.session.declare_queryable(
            f"cognitive/{character_name}/execute_plan_sync",
            self._sync_plan_execution_handler
        )
        
        # Shutdown flags
        self.shutdown_requested = False
        self._shutting_down = False

        self.inspections = {} # cache of inspections
        self.uses = {} # cache of uses
        self.activities = {} # dictionary of all available activities for initializing activity manager
        try:
            self.activities = json.load(open(f'../scenarios/{self.character_name}-activities.json'))
            logger.info(f'📚 Loaded {len(self.activities)} activities for {self.character_name}')
        except Exception as e:
            logger.error(f'Error loading activities for {self.character_name}: {e}')

        logger.info(f'🧠 Zenoh Executive Node initialized for character: {character_name}')
        logger.info(f'   - Subscribing to: cognitive/{character_name}/sense_data')
        logger.info(f'   - Subscribing to: cognitive/{character_name}/control/step')
        logger.info(f'   - Subscribing to: cognitive/{character_name}/control/run')
        logger.info(f'   - Subscribing to: cognitive/{character_name}/control/stop')
        logger.info(f'   - Subscribing to: cognitive/map/time_advanced')
        logger.info(f'   - Subscribing to: cognitive/{character_name}/end_dialog')
        logger.info(f'   - Publishing to: cognitive/{character_name}/action')
        logger.info(f'   - Publishing to: cognitive/{character_name}/situation/request_update')
        logger.info(f'   - Subscribing to: cognitive/save_all')
        logger.info(f'   - Publishing to: cognitive/{character_name}/goal')
        logger.info(f'   - Publishing to: cognitive/{character_name}/decided_action')
        logger.info(f'   - Publishing to: cognitive/{character_name}/execution_state')
        logger.info(f'   - Publishing to: cognitive/map/time_proposal')

        self.activity_manager = None
        if self.activities:
            self.activity_manager = ActivityManager(self, self.activities, self.llm_client, self.map_types)

        # Register signal handlers for graceful shutdown
        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)

    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully."""
        logger.warning(f'Received signal {signum}, initiating shutdown...')
        self.shutdown_requested = True
    
    def _handle_save_command(self, sample):
        """Handle save_all command - save memory and resource_manager state."""
        try:
            logger.info(f'💾 {self.character_name} received save_all command')
            self.memory.save()
            if self.is_infospace and self.resource_manager:
                self.resource_manager.save_to_file()
                logger.info(f'💾 Saved resource manager state for {self.map_name}')
        except Exception as e:
            logger.error(f'Error handling save command: {e}')
    
    def run(self):
        """Main OODA loop."""
        try:
            logger.info('Executive Node running - press Ctrl+C to stop')
            
            # Announce character presence
            self._announce_character()
            time.sleep(0.1)
            time.sleep(1.0)
            
            # Start OODA loop
            while not self.shutdown_requested:
                if self.execution_paused:
                    time.sleep(0.2)
                else:
                    try:
                        # Priority order: Text input → Normal OODA
                        if self.text_input_queue:
                            # Don't process text_input if awaiting ask response
                            # Let ask handler in _execute_next_step grab the response
                            if not (self.is_infospace and self.plan_state and self.plan_state.get('awaiting_ask')):
                                self._process_text_input()
                        self._run_ooda_loop()
                        
                        # In step mode, pause after one OODA cycle
                        if self.execution_mode == 'step':
                            self.execution_paused = True
                            self._publish_execution_state()
                    except Exception as e:
                        traceback.print_exc()
                        logger.error(f'Error in OODA loop: {e}')
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
            
            # Publish initial execution state so UI knows character is ready
            self._publish_execution_state()
            
        except Exception as e:
            logger.error(f'Error announcing character: {e}')
    
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

    def _publish_action_result(self, action: Dict, result: Dict, action_type: str, timestamp: datetime):
        """Publish action result to action topic for UI display."""
        try:
            # Normalize action_type - handle both 'create-note' and 'createnote' formats
            normalized_type = action_type.replace('_', '-').lower()
            
            # Format action data for UI display
            action_data = {
                'type': normalized_type,
                'action_type': normalized_type,  # Also set action_type for UI compatibility
                'action_id': f'{normalized_type}_{int(timestamp.timestamp() * 1000)}',
                'timestamp': timestamp.isoformat(),
                'character': self.character_name,
                'status': result.get('status', 'unknown')
            }
            
            # Add action-specific fields for UI display
            # Handle both hyphenated and non-hyphenated action types
            action_types_to_handle = ['create-note', 'createnote', 'create-collection', 'createcollection', 
                                     'expand', 'flatten', 'map', 'summarize', 'query-web', 'semantic-scholar']
            if normalized_type in action_types_to_handle:
                target = action.get('target', '')
                value = action.get('value', '')
                out_var = action.get('out', '')
                
                # Prefer displaying a bound variable value if target references a plan variable
                display_target = target
                if isinstance(target, str) and target.startswith('$'):
                    var_name = target[1:]
                    if var_name in self.plan_bindings:
                        display_target = self.plan_bindings[var_name]
                
                # Show what was created/processed
                if result.get('status') == 'success':
                    result_value = result.get('value', '')
                    if result_value:
                        action_data['result'] = result_value
                        action_data['target'] = target  # Keep original target (variable name)
                        action_data['resolved_target'] = display_target  # Show resolved value if different
                        logger.debug(f'Published action result: {normalized_type} -> result={result_value}, target={target}')
                    else:
                        action_data['target'] = target
                        action_data['resolved_target'] = display_target if display_target != target else None
                        action_data['value'] = value
                        logger.debug(f'Published action result: {normalized_type} -> no result_value, target={target}')
                else:
                    action_data['target'] = target
                    action_data['resolved_target'] = display_target if display_target != target else None
                    action_data['value'] = value
                    action_data['error'] = result.get('reason', 'Failed')
            else:
                # Generic action format - serialize objects to prevent [object Object] in UI
                # For action, extract key fields or serialize if it's a complex object
                if isinstance(action, dict):
                    # Extract key fields for display
                    action_data['target'] = action.get('target', '')
                    action_data['value'] = action.get('value', '')
                    action_data['out'] = action.get('out', '')
                    # If action has other complex nested structures, serialize them
                    if len(action) > 3:  # More than just target/value/out
                        action_data['action'] = json.dumps(action)  # Serialize complex action
                    else:
                        action_data['action'] = action  # Simple dict is fine
                else:
                    action_data['action'] = json.dumps(action) if not isinstance(action, str) else action
                
                # For result, ensure value is serialized if it's a complex object
                if isinstance(result, dict):
                    result_value = result.get('value', '')
                    if result_value:
                        # Serialize if it's a complex object (not a simple string/number)
                        if isinstance(result_value, (dict, list)):
                            action_data['result'] = json.dumps(result_value)
                        else:
                            action_data['result'] = result_value
                    else:
                        # Serialize the whole result dict if it's complex
                        action_data['result'] = json.dumps(result) if len(result) > 2 else result
                else:
                    action_data['result'] = json.dumps(result) if not isinstance(result, str) else result
            
            self.action_publisher.put(json.dumps(action_data))
            self.action_counter += 1
        except Exception as e:
            logger.error(f'Error publishing action result: {e}')

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
                    return  # Don't process as speech                   return  # Don't process as speech
            
            # Normal dialog processing
            logger.info(f'📥 {self.character_name} Processing text input: "{text_input}" (source: {source})')
            self.plan_just_generated = True  # Skip action execution this turn
        
    def _run_ooda_loop(self):
        """Execute the OODA loop: Observe, Orient, Decide, Act."""
        try:
            # Clear plan generation flag from previous turn
            plan_was_just_generated = self.plan_just_generated
            self.plan_just_generated = False
            
            # Request fresh situation update for UI
            time.sleep(0.1)
            # Observe: Collect current situation and sense data
            observations = self._observe()
            # Manual characters skip orient/plan entirely
            return

        except Exception as e:
            logger.error(f'Error in OODA loop: {e}')
            logger.error(traceback.format_exc())
        finally:
            # In step mode, pause after plan execution
            if self.execution_mode == 'step':
                self.execution_paused = True
                self._publish_execution_state()


    def format_situation(self):
        """Format the situation data for the LLM."""
        formatted_situation = ''
        
        # Skip map-based situation data for infospace characters
        # (situation_node has been removed - it was only for map-based spatial awareness)
        if not self.is_infospace:
            # Map-based situation data would go here if needed in the future
            # Currently not used since situation_node is removed
            pass
        
        # world state updates disabled

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

    def _truncate_result(self, result: Any, max_len: int = 120) -> str:
        """
        Truncate action result for prompt inclusion.
        
        Args:
            result: Action result (can be string, dict, or other)
            max_len: Maximum length in characters
            
        Returns:
            Truncated string representation
        """
        if result is None:
            return ''
        result_str = str(result)
        if len(result_str) <= max_len:
            return result_str
        return result_str[:max_len] + '...'
    
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
            result_str = self._truncate_result(last_action.result)
            system_prompt += f"\n#Your last action was:\n\t{last_action.action.get('type')}: {last_action.action.get('target')} - {result_str}\n"
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
            if self.character_config.get('setting', None):
                system_prompt += f"\n#The overall setting is:\n{self.character_config['setting']}\n"
            if not self.map_types:
                try:
                    if self.is_infospace and self.resource_manager:
                        self.map_types = self.resource_manager.get_resource_types()
                    else:
                        # Fallback for non-infospace (shouldn't happen in current architecture)
                        self.map_types = {'success': False, 'resource_types': []}
                except Exception as e:
                    logger.error(f'Error getting resource types in _build_user_prompt: {e}')
                    self.map_types = {'success': False, 'resource_types': []}
            if self.map_types:
                system_prompt += f"\n#The following are the types primary types in the scenario:\n{format_map_types(self.map_types)}"
            # Build user prompt with context
            user_prompt += self.format_situation()
            entity_context = None
            entity_context = self.get_entity_context(self.character_name, 10)
            if entity_context:
                user_prompt += f'\n#Your most recent thoughts include:'
                for i, memory in enumerate(entity_context['conversation_history']):  # Use last 2 memories
                    user_prompt += f"\n\t{memory['source']}: {memory['text'][:600]}..."
                user_prompt += '\n'
            if self.action_history:
                last_action = self.action_history[-1]
                result_str = self._truncate_result(last_action.result)
                user_prompt += f'\n#Your last action was:'
                user_prompt += f"\n\t{last_action.action.get('type')}: {last_action.action.get('target')} - {result_str}\n"

            self.observations = {'static': system_prompt, 'dynamic': user_prompt}
            return self.observations
        except Exception as e:
            logger.error(f'Error in _observe: {e}')  
            traceback.print_exc()
            self.observations = {'static': system_prompt, 'dynamic': user_prompt}
            return self.observations

    def _orient(self, observations: Dict[str, Any], step_rewrite: bool = False):
        """Orient: Assess current state and drives"""

        # Create a new goal based on current situation
        other_characters_str = ''
        for other_character_name, other_character_desc in self.character_config.get('characters', {}).items():
            if other_character_name != self.character_name:
                other_characters_str += f"\n\t{other_character_name}: {other_character_desc}"
        character_names = list(self.character_config.get("characters", {}).keys())

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
        # Make LLM call
        try:
            # Use shorter timeout during shutdown
            timeout = 100.0 if self.shutdown_requested else None
            response = self.llm_generate(
                messages=[GOAL_TEMPLATE if not step_rewrite else REWRITE_TEMPLATE],
                bindings={
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
                    self.current_goal = Goal('sleep', actors=[self.character_name])
                    self._publish_goal(self.current_goal)
                    return self.current_goal
                for goal_hash in forms:
                    goal = validate_and_create_goal(self.character_name, goal_hash)
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
                self.current_goal = Goal('sleep', actors=[self.character_name])
                self._publish_goal(self.current_goal)

            if not self.current_goal:
                logger.error(f'No goal generated for {self.character_name}')
        except Exception as e:
            logger.error(f'Error in _orient: {e}')
            traceback.print_exc()
            self.current_goal = Goal('sleep', actors=[self.character_name])
            self._publish_goal(self.current_goal)
        return self.current_goal


    def _plan(self, goal: Goal):
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
            entity_context = None
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

            directive = f"""\nrespond only with the JSON plan, no other text.\nend your response with </end>"""

            self.current_plan_prompt = system_prompt + user_prompt + goal_prompt
            # Capture percepts at plan start (before LLM call)
            try:
                self.percepts_at_plan = self._capture_percepts_at_plan()
            except Exception:
                self.percepts_at_plan = []
            
            # Clear plan state for infospace
            self.infospace_executor.clear_plan_state()
            goal_text = goal.name + (': ' + goal.description if goal.description != goal.name else '')
            character_context = system_prompt  # Character description + drives
            self_entity_context = self.get_entity_context(self.character_name, 10)
            recent_context = ""
            if self_entity_context and self_entity_context.get('conversation_history'):
                recent_context += "\n# Recent thoughts:\n"
                for memory in self_entity_context['conversation_history'][:3]:  # Last 3
                    if isinstance(memory, dict):
                        recent_context += f"  {memory.get('source', '')}: {memory.get('text', '')[:200]}...\n"
            if entity_context and entity_context.get('conversation_history'):
                recent_context += f"\n# Recent dialog with {target}:\n"
                for memory in entity_context['conversation_history'][:3]:  # Last 3
                    if isinstance(memory, dict):
                        recent_context += f"  {memory.get('source', '')}: {memory.get('text', '')[:200]}...\n"
            
            # Add last action if available
            if self.action_history:
                last_action = self.action_history[-1]
                result_str = self._truncate_result(last_action.result)
                recent_context += f"\n# Last action:\n  {last_action.action.get('type')}: {last_action.action.get('target')} - {result_str}\n"
            
            # Build context for planner (always needed for infospace)
            context = {
                'variables': self.infospace_executor.plan_bindings if self.infospace_executor else {},
                'character_context': character_context,
                'recent_context': recent_context,
                'executor': self.infospace_executor  # Pass executor for incremental planner
            }
            
            # Initialize plan identifiers before plan generation (needed for incremental planner action tracking)
            self.plan_counter += 1
            self.current_plan_id = f"p_{self.plan_counter}"
            self.step_counter = 0
            self.current_plan = self.planner.generate_plan(goal=goal_text, context=context)

        return self.current_plan

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
        if reason == 'manual goal override' or reason == 'manual plan override':
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
            result_str = self._truncate_result(result)
            timestamp = record.timestamp.strftime('%H:%M:%S')
            actions_text.append(f"{timestamp} - {action_type}: {target} -> {result_str}")
        
        actions_summary = '\n'.join(actions_text)
        
        # Prepare context for LLM summary
        goal_text = self.current_goal.to_string() if self.current_goal else "No specific goal"
        plan_text = json.dumps(self.current_plan, indent=2) if self.current_plan else "No plan available"
        
        # Extract planner's own assessment if available (for incremental planner)
        planner_assessment = ""
        if self.current_plan and isinstance(self.current_plan, dict):
            reasoning = self.current_plan.get('reasoning', '')
            success = self.current_plan.get('success', None)
            if reasoning:
                planner_assessment = f"Planner's final assessment: {reasoning}"
                if success is not None:
                    planner_assessment += f" (success={success})"
        
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
        
        {{$planner_assessment}}
        
        Please provide JSON object with:
        1. 
        2. a JSON formatted assessment of the plan's success or failure in meeting the goal, in the following format:
            {
                "Summary": str "a concise paragraph summarizing this plan execution, including the goal, actions taken, and observed results",
                "How": str "concise (8-10 words) explanation how this plan intended to achieve the goal",
                "outcome": str "concise (20-28 words) explanation of the outcome - did it achieve the goal? If not, where did it fail and why?",
                "plan_score": int (0-100) "did the plan execute as expected? (lower if steps failed, even if goal was achieved)"
                "goal_score": int (0-100) "how well the goal was met as measured by goal termination condition. If planner marked goal as DONE/achieved, use 80-100. If planner marked as incomplete, use 0-60."
            }
        
        IMPORTANT: The goal_score should reflect whether the GOAL was achieved (not whether all steps succeeded). If the planner explicitly marked the goal as DONE/achieved, the goal_score should be high (80-100) even if some intermediate steps failed. The plan_score can be lower if steps failed, but goal_score reflects goal achievement.
        
        Do not include any other introductory, explanatory, discursive, or formatting text in your response.
        
        """
        bindings = {
            "goal_text": goal_text, 
            "plan_text": plan_text, 
            "actions_summary": actions_summary, 
            "percepts_json": percepts_json, 
            "telemetry_json": telemetry_json
        }
        if planner_assessment:
            bindings["planner_assessment"] = planner_assessment
        else:
            bindings["planner_assessment"] = ""
        
        response = self.llm_generate([summary_prompt], 
                                      bindings=bindings, 
                                      max_tokens=500, is_json=True)
        summary = response.text if isinstance(response.text, dict) else None
        self.plan_summary = summary
        if not summary:
            logger.error(f'❌ No summary found in response: {response}')
            summary = {}
        logger.info(f'📝 Plan post-mortem prepared for {self.character_name}\n{self.plan_summary}\n')
        
        # Mark as completed to prevent redundant calls
        self.plan_summary_completed = True
        # Metrics aggregation (limited since physiology/time data removed)
        try:
            steps_total = len(self.action_history)
            moves = sum(1 for ar in self.action_history if ar.action.get('type', '').lower() == 'move')
            takes = sum(1 for ar in self.action_history if ar.action.get('type', '').lower() == 'take')
            uses = sum(1 for ar in self.action_history if ar.action.get('type', '').lower() == 'use')
            inspects = sum(1 for ar in self.action_history if ar.action.get('type', '').lower() == 'inspect')
            failures = sum(1 for ar in self.action_history if getattr(ar, 'outcome_status', None) == 'failure')
            items_taken = [ar.action.get('target', '') for ar in self.action_history if ar.action.get('type', '').lower() == 'take']
            items_used = [ar.action.get('target', '') for ar in self.action_history if ar.action.get('type', '').lower() == 'use']

            metrics = {
                'steps': {
                    'total': steps_total,
                    'moves': moves,
                    'takes': takes,
                    'uses': uses,
                    'inspects': inspects,
                    'failures': failures
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
        
        # Add infospace compliance metrics if evaluation mode is active
        if self.is_infospace and hasattr(self.infospace_executor, '_compliance_tracker'):
            tracker = self.infospace_executor._compliance_tracker
            if tracker:
                compliance_metrics = tracker.get_metrics()
                metrics['infospace_compliance'] = compliance_metrics
                
                # Log eval statistics summary
                violation_count = len(compliance_metrics.get('type_violations', []))
                checks_count = compliance_metrics.get('compatibility_checks', 0)
                
                if violation_count > 0:
                    logger.info(f'🧪 EVAL STATS [{self.character_name}]: {violation_count} type violations, {checks_count} compatibility checks')
                    # Log violation details
                    for v in compliance_metrics.get('type_violations', []):
                        logger.info(f'  ❌ {v["operation"]} on {v["variable"]}: expected {v["expected_type"]}, got {v["actual_type"]}')
                else:
                    logger.info(f'🧪 EVAL STATS [{self.character_name}]: ✓ No violations, {checks_count} compatibility checks passed')

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
                
                # Add conversation entry to memory (default entity is "User")
                entity_name = source if source != 'console' else "User"
                self.memory.add_conversation_entry(entity_name, source, text_input)
                
        except Exception as e:
            traceback.print_exc()
            logger.error(f'Error processing sense data: {e}')
    
    def handle_step_command(self, sample):
        """Handle step command - advance one OODA cycle."""
        try:
            logger.info(f'🎯 Step command received by {self.character_name}')
            self.execution_mode = 'step'
            self.execution_paused = False
            self._publish_execution_state()
        except Exception as e:
            logger.error(f'Error handling step command: {e}')
            traceback.print_exc()
    
    def handle_run_command(self, sample):
        """Handle run command - enable continuous execution."""
        try:
            logger.info(f'🏃 Run command received by {self.character_name}')
            self.execution_mode = 'run'
            self.execution_paused = False
            self._publish_execution_state()
        except Exception as e:
            logger.error(f'Error handling run command: {e}')
            traceback.print_exc()
    
    def handle_stop_command(self, sample):
        """Handle stop command - pause execution."""
        try:
            logger.info(f'⏹️ Stop command received by {self.character_name}')
            self.execution_mode = 'step'
            self.execution_paused = True
            self._publish_execution_state()
        except Exception as e:
            logger.error(f'Error handling stop command: {e}')
            traceback.print_exc()
    
    def _publish_execution_state(self):
        """Publish current execution state for UI."""
        try:
            state_data = {
                'paused': self.execution_paused,
                'mode': self.execution_mode,
                'character': self.character_name,
                'timestamp': time.time()
            }
            self.execution_state_publisher.put(json.dumps(state_data).encode('utf-8'))
        except Exception as e:
            logger.error(f'Error publishing execution state: {e}')
    
    def shutdown_callback(self, sample):
        """Handle shutdown command from UI."""
        try:
            logger.warning(f'🔌 {self.character_name} Executive Node received shutdown command')
            self.shutdown_requested = True
        except Exception as e:
            logger.error(f'Error in shutdown callback: {e}')
    
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
    
    def _activity_selection_callback(self, sample):
        """Handle manual activity selection from UI."""
        try:
            data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            activity_name = data.get('activity_name')
            
            if not activity_name:
                logger.error(f'No activity_name in manual selection request')
                return
            
            logger.info(f'🎯 {self.character_name} received manual activity selection: {activity_name}')
            
            if not self.activity_manager:
                logger.error(f'No activity_manager available for {self.character_name}')
                return
            
            # Set the activity manually
            step, activity = self.activity_manager.set_activity_manually(activity_name)
            
            if activity:
                # Clear current plan and goal to start fresh with new activity
                self.current_plan = None
                self.current_goal = None
                self._publish_current_activity()
                logger.info(f'✅ {self.character_name} activity set to: {activity_name}')
            else:
                logger.error(f'Failed to set activity {activity_name} for {self.character_name}')
        except Exception as e:
            logger.error(f'Error in activity selection callback: {e}')
    
    def _enable_compliance_tracking_callback(self, sample):
        """Handle enabling compliance tracking for evaluation mode."""
        try:
            if self.is_infospace and hasattr(self, 'infospace_executor'):
                from infospace_compliance import ComplianceTracker
                
                # Create and attach compliance tracker to executor
                self.infospace_executor._compliance_tracker = ComplianceTracker()
                logger.info(f'🧪 {self.character_name} compliance tracking enabled')
            else:
                logger.warning(f'Cannot enable compliance tracking: not in infospace mode')
        except Exception as e:
            logger.error(f'Error enabling compliance tracking: {e}')
    
    def _activity_list_query_handler(self, query):
        """Handle query for available activities list."""
        try:
            activities = []
            if self.activity_manager:
                activities = self.activity_manager.get_available_activities()
            
            response = {
                'success': True,
                'activities': activities
            }
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
        except Exception as e:
            logger.error(f'Error handling activity list query: {e}')
            response = {
                'success': False,
                'activities': []
            }
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
    
    def _handle_resource_view_query(self, query):
        """Handle query for resource content viewing (for UI)."""
        try:
            if not self.is_infospace or not self.resource_manager:
                response = {
                    'success': False,
                    'error': 'Resource viewing only available for infospace characters'
                }
                query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
                return
            
            # Extract resource_id from query key (cognitive/{character}/resource/view/{resource_id})
            key_parts = str(query.key_expr).split('/')
            if len(key_parts) < 5:
                response = {
                    'success': False,
                    'error': 'Invalid resource view query format'
                }
                query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
                return
            
            resource_id = key_parts[-1]
            resource_content = self.resource_manager.get_resource_content(resource_id)
            
            if resource_content:
                query.reply(query.key_expr, json.dumps(resource_content).encode('utf-8'))
            else:
                response = {
                    'success': False,
                    'error': f'Resource {resource_id} not found'
                }
                query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
        except Exception as e:
            logger.error(f'Error handling resource view query: {e}')
            response = {
                'success': False,
                'error': str(e)
            }
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
    
    def _handle_resources_list_query(self, query):
        """Handle query for resources list (for resource_browser)."""
        try:
            if not self.is_infospace or not self.resource_manager:
                response = {
                    'success': False,
                    'error': 'Resource list only available for infospace characters'
                }
                query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
                return
            
            resources = self.resource_manager.get_resource_list()
            
            # Convert ResourceType enums to strings for JSON serialization
            json_safe_resources = []
            for resource in resources:
                resource_copy = resource.copy()
                if 'type' in resource_copy:
                    resource_copy['type'] = str(resource_copy['type'])
                # Convert location tuple to list if present
                if 'location' in resource_copy and isinstance(resource_copy['location'], tuple):
                    resource_copy['location'] = list(resource_copy['location'])
                # Ensure properties are JSON-serializable
                if 'properties' in resource_copy:
                    props = resource_copy['properties'].copy()
                    for key, value in props.items():
                        if isinstance(value, datetime):
                            props[key] = value.isoformat()
                    resource_copy['properties'] = props
                json_safe_resources.append(resource_copy)
            
            response = {
                'success': True,
                'resources': json_safe_resources
            }
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
        except Exception as e:
            logger.error(f'Error handling resources list query: {e}')
            response = {
                'success': False,
                'error': str(e)
            }
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
    
    def _handle_resource_by_id_query(self, query):
        """Handle query for resource by ID (for resource_browser)."""
        try:
            if not self.is_infospace or not self.resource_manager:
                response = {
                    'success': False,
                    'error': 'Resource query only available for infospace characters'
                }
                query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
                return
            
            # Extract resource_id from query key (cognitive/{character}/resource/{resource_id})
            key_parts = str(query.key_expr).split('/')
            if len(key_parts) < 4:
                response = {
                    'success': False,
                    'error': 'Invalid resource query format'
                }
                query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
                return
            
            resource_id = key_parts[-1]
            resource = self.resource_manager.get_resource(resource_id)
            
            if resource:
                # Make resource JSON-serializable
                resource_copy = resource.copy()
                if 'type' in resource_copy:
                    # Convert ResourceType enum to string
                    resource_copy['type'] = str(resource_copy['type'])
                # Ensure all nested dicts are serializable
                if 'properties' in resource_copy:
                    props = resource_copy['properties'].copy()
                    # Convert any datetime objects to ISO strings
                    for key, value in props.items():
                        if isinstance(value, datetime):
                            props[key] = value.isoformat()
                    resource_copy['properties'] = props
                # Convert location tuple to list if present
                if 'location' in resource_copy and isinstance(resource_copy['location'], tuple):
                    resource_copy['location'] = list(resource_copy['location'])
                
                response = {
                    'success': True,
                    'resource': resource_copy
                }
            else:
                response = {
                    'success': False,
                    'error': f'Resource {resource_id} not found'
                }
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
        except Exception as e:
            logger.error(f'Error handling resource by id query: {e}')
            response = {
                'success': False,
                'error': str(e)
            }
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
    
    def _handle_resource_remove_query(self, query):
        """Handle query for resource removal (for resource_browser)."""
        try:
            if not self.is_infospace or not self.resource_manager:
                response = {
                    'success': False,
                    'error': 'Resource removal only available for infospace characters'
                }
                query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
                return
            
            # Extract resource_id from query key (cognitive/{character}/resource/remove/{resource_id})
            key_parts = str(query.key_expr).split('/')
            if len(key_parts) < 5:
                response = {
                    'success': False,
                    'error': 'Invalid resource remove query format'
                }
                query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
                return
            
            resource_id = key_parts[-1]
            success, error_msg = self.resource_manager.delete_resource(resource_id)
            
            if success:
                response = {
                    'success': True,
                    'message': f'Resource {resource_id} deleted'
                }
            else:
                response = {
                    'success': False,
                    'error': error_msg or f'Failed to delete resource {resource_id}'
                }
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
        except Exception as e:
            logger.error(f'Error handling resource remove query: {e}')
            response = {
                'success': False,
                'error': str(e)
            }
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
    
    def _sync_plan_execution_handler(self, query):
        """Handle query for synchronous plan execution."""
        try:
            if not self.is_infospace or not self.infospace_executor:
                response = {
                    'success': False,
                    'error': 'Sync plan execution only available for infospace characters'
                }
                query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
                return
            
            # Parse plan from query payload
            payload = query.payload.to_bytes().decode('utf-8')
            request_data = json.loads(payload)
            plan = request_data.get('plan')
            max_steps = request_data.get('max_steps', 1000)
            
            if not plan:
                response = {
                    'success': False,
                    'error': 'Plan is required'
                }
                query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
                return
            
            # Execute plan synchronously
            result = self.infospace_executor.execute_plan_sync(plan, max_steps=max_steps)
            
            # Return result
            response = {
                'success': result.get('status') == 'success',
                'status': result.get('status'),
                'reason': result.get('reason'),
                'executed_steps': result.get('executed_steps', 0),
                'bindings': result.get('bindings', {}),
                'suspended': result.get('status') == 'suspended',
                'suspension_reason': result.get('reason') if result.get('status') == 'suspended' else None
            }
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
            
        except Exception as e:
            logger.error(f'Error handling sync plan execution query: {e}')
            import traceback
            logger.error(traceback.format_exc())
            response = {
                'success': False,
                'error': str(e)
            }
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
    
    def publish_dialog_end(self, source: str):
        """Publish dialog end notification to another character."""
        # Only User can close dialogs - prevent other characters from closing
        if self.character_name != 'User':
            logger.info(f'🔒 {self.character_name} cannot close dialogs - only User can end conversations')
            return
        
        try:
            # === ZENOH PUBLICATION (ad-hoc) ===
            # NAME: dialog_end
            # TOPIC: cognitive/{character}/dialog_end
            # DESCRIPTION: User ended conversation with character
            # PAYLOAD: {"other_name": str}
            # TRIGGERS: HandlePlanFailure (conversation interrupted), ResumePreviousActivity
            # ========================
            key = f"cognitive/{source}/dialog_end"
            payload = json.dumps({'other_name': self.character_name})
            self.session.put(key, payload)
            
            # Release conversation lock with this character
            self._release_conversation_lock(source)
            
            # === ZENOH PUBLICATION (ad-hoc) ===
            # NAME: memory_close_dialog
            # Close dialog in memory module (direct call, no Zenoh)
            self.memory.close_dialog(self.character_name)
            
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
            self.current_goal = Goal(parsed_goal, [self.character_name], description='', termination='')
            
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

    def _capture_percepts_at_plan(self) -> List[Dict[str, Any]]:
        """Capture a compact, normalized snapshot of percepts at plan start.

        Limits size and fields to keep logs small. Returns [] on failure.
        For infospace characters, returns empty list (no spatial percepts).
        """
        # Skip situation queries for infospace characters (situation_node removed)
        if self.is_infospace:
            return []
        
        snapshot: List[Dict[str, Any]] = []
        try:
            # Map-based situation queries removed (situation_node no longer exists)
            # Return empty snapshot for now
            pass
        except Exception:
            return []
        return snapshot

    
    def _get_recent_chat_memories(self, num_entries: int) -> List[Dict[str, Any]]:
        """Get recent memory entries from memory module."""
        try:
            # Get entity data for User (default entity)
            entity_data = self.memory.get_entity_data("User", limit=num_entries, scope='all')
            if entity_data:
                return entity_data.get('conversation_history', [])
            return []
        except Exception as e:
            logger.error(f'Error getting recent memories: {e}')
            return []
      

    def _assess_drive_fulfillment(self) -> Dict[str, Any]:
        """Assess drive fulfillment using an LLM. Returns structured scores per drive.

        Output shape:
          { "drives": [{"name": str, "score": float, "rationale": str}]}
        """
        assessment = []
        try:
            goal_text = self.current_goal.to_string() if self.current_goal else ''
            plan_text = json.dumps(self.current_plan, indent=2) if self.current_plan else '{}'
            # Physiological snapshot removed; keep placeholder for compatibility
            state_text = "{}"
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
                result_str = self._truncate_result(ar.result)
                actions_text += f"""{ar.action.get('type', '')}, {ar.action .get('target', '')}: result: {result_str}\n"""

        except Exception as e:
            logger.error(f'Error collecting drive assessment context: {e}')
            traceback.print_exc()
            return []

        for drive in self.drives:
            if not self.llm_client or self.shutdown_requested:
                continue
            response = self.llm_generate(
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
        Get entity data from memory module for context.
        
        Args:
            entity_name: Name of the entity to query (defaults to "User" if not provided)
            limit: Number of recent conversation entries to include (default 20)
            scope: 'current' for last dialog only, 'all' for entries from all dialogs
            
        Returns:
            Dictionary with entity data or None if not found
        """
        # safeguard - don't allow variables in query
        entity_name = entity_name.replace('$', '')
        if not entity_name:
            entity_name = "User"
        
        try:
            entity_data = self.memory.get_entity_data(entity_name, limit=limit, scope=scope)
            if entity_data:
                logger.info(f'👥 Retrieved entity context for {entity_name}')
                return entity_data
            return None
        except Exception as e:
            logger.error(f'Error getting entity context for {entity_name}: {e}')
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
        Get entity discourse state and ToM model from memory module.
        Args:
            entity_name: Name of the entity to query (defaults to "User" if not provided)
            limit: Number of recent conversation entries to include (default 20, unused but kept for compatibility)
            scope: 'current' or 'all' (unused but kept for compatibility)
        Returns:
            Dictionary with discourse_state and tom_model
        """
        # safeguard - don't allow variables in query
        entity_name = entity_name.replace('$', '')
        if not entity_name:
            entity_name = "User"
        
        logger.info(f' Getting entity discourse/tom for {entity_name}')
        try:
            result = self.memory.get_entity_discourse_tom(entity_name)
            if result.get('success'):
                logger.info(f'👥 Retrieved entity discourse/tom for {entity_name}')
            return result
        except Exception as e:
            logger.error(f'Error getting entity discourse/tom for {entity_name}: {e}')
            return {'success': False, 'discourse_state': '', 'tom_model': ''}

    
    def shutdown(self):
        """Clean shutdown."""
        try:
            if getattr(self, '_shutting_down', False):
                return
            self._shutting_down = True
            
            logger.info(f'Executive Node shutdown initiated for {self.character_name}...')
            
            # Save memory before shutdown
            try:
                self.memory.save()
                logger.info(f'💾 Saved memory state for {self.character_name}')
            except Exception as e:
                logger.error(f'Error saving memory during shutdown: {e}')
            
            # Save resource manager if infospace
            if self.is_infospace and self.resource_manager:
                try:
                    self.resource_manager.save_to_file()
                    logger.info(f'💾 Saved resource manager state for {self.map_name}')
                except Exception as e:
                    logger.error(f'Error saving resource manager during shutdown: {e}')
            
            # Publish shutdown event for cleanup
            try:
                # === ZENOH PUBLICATION (ad-hoc) ===
                # NAME: character_shutdown
                # TOPIC: cognitive/{character}/shutdown
                # DESCRIPTION: Character shutting down gracefully
                # PAYLOAD: {"character_name": str, "timestamp": str, "type": "shutdown"}
                # TRIGGERS: PrepareStatusReport, ArchiveCompletedWork, SaveMemoryState
                # ========================
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
