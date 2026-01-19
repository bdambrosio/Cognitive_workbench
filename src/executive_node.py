#!/usr/bin/env python3
"""
Zenoh Executive Node

This node implements the OODA loop for character decision-making and action execution.

"""

import math
import random
import re
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
from pathlib import Path
from typing import Dict, List, Any, Union, Optional
from Messages import SystemMessage, UserMessage

import utils.hash_utils as hash_utils
from utils.zenoh_utils import datetime_handler
from dataclasses import dataclass, asdict
import os
from templates import DRIVE_ASSESSMENT_TEMPLATE, GOAL_TEMPLATE, PLAN_TEMPLATE, PLAN_VERBS, REWRITE_TEMPLATE
from utils.format_utils import format_map_types, format_views_compact
from weakref import WeakValueDictionary
from utils.format_utils import format_views_compact
from utils.condition_utils import deref_plan_target
from transformers import AutoTokenizer
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

# SGLang imports
try:
    import sglang as sgl
    HAS_SGLANG = True
except ImportError:
    HAS_SGLANG = False
    sgl = None


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
    result: Optional[str]  # Formatted result string (for backward compatibility)
    timestamp: datetime
    result_dict: Optional[Dict[str, Any]] = None  # Full uniform result format: {status, value, resource_id, reason}
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
    
    def _load_tools(self) -> Dict[str, Dict]:
        """
        Load tools from src/tools directory and world-specific directories.
        
        Always loads general tools from src/tools/.
        Conditionally loads world-specific tools from src/world-tools/<world_name>/ only if world_config.world_name is set.
        
        Note: load_tools() scans only immediate subdirectories (not recursive), so world-tools/ 
        will not be picked up when scanning src/tools/.
        """
        from utils.tool_loader import load_tools
        from pathlib import Path
        
        tools = {}
        src_dir = Path(__file__).parent
        
        # Always load general tools from src/tools (top-level tools directory)
        tools_dir = src_dir / 'tools'
        if tools_dir.exists():
            logger.info(f"Loading tools from: {tools_dir}")
            tools.update(load_tools(str(tools_dir)))
        else:
            logger.warning(f"Tools directory not found: {tools_dir}")
        
        # Conditionally load world-specific tools from src/world-tools/<world_name>/ only if configured
        world_config = self.character_config.get('world_config', {})
        world_name = world_config.get('world_name')
        if world_name:
            world_tools_dir = src_dir / 'world-tools' / world_name
            if world_tools_dir.exists():
                logger.info(f"Loading {world_name} tools from: {world_tools_dir}")
                tools.update(load_tools(str(world_tools_dir)))
            else:
                logger.debug(f"World tools directory not found: {world_tools_dir}")
        
        return tools
    
    def __init__(self, character_name="default", character_config=None):
        # Store character info (canonicalized)
        self.character_name = character_name.capitalize()
        self.character_config = character_config or {}
        self.drives = self.character_config.get('drives', [])
        self.drives_str = '\n'.join(self.drives)   
        
        # Debug mode flag - must be set early as it's used throughout initialization
        self.debug = str(os.getenv('CWB_DEBUG', '')).lower() in ('1', 'true', 'yes', 'on')
        if self.debug:
            console_handler.setLevel(logging.INFO)
            logger.info(f'🔧 Debug mode enabled for {self.character_name}')
        
        # Manual control flags
        self.manual = bool(self.character_config.get('manual', False))
        self.manual_response = bool(self.character_config.get('manual_response', False))
        
        # Benchmark mode flag - disables conversation collection initialization and updates
        self.benchmark_mode = bool(self.character_config.get('benchmark_mode', False))
        if self.benchmark_mode:
            logger.info(f'📊 Benchmark mode enabled for {self.character_name} - conversation collections disabled')
        
        # Initialize Zenoh session
        config = zenoh.Config()
        self.session = zenoh.open(config)
        
        # Autonomous mode state
        self.previous_autonomous_goal_text = ''
        self.autonomous_task_state = ''
        
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
        # NAME: plan_result
        # TOPIC: cognitive/{character}/plan_result
        # DESCRIPTION: Published when incremental planning completes
        # PAYLOAD: {"status": str, "final_thoughts": str, "final_content": str, "bindings": dict}
        # ========================
        self.plan_result_publisher = self.session.declare_publisher(f"cognitive/{character_name}/plan_result")
        
        # === ZENOH PUBLICATION ===
        # NAME: goal_result
        # TOPIC: cognitive/{character}/goal_result
        # DESCRIPTION: Published when goal planning completes - contains complete result from _plan()
        # PAYLOAD: Complete result dict from incremental_planner.generate_plan() with plan, response, task_state, success, error_count
        # ========================
        self.goal_result_publisher = self.session.declare_publisher(f"cognitive/{character_name}/goal_result")
        
        # === ZENOH PUBLICATION ===
        # NAME: bindings_update
        # TOPIC: cognitive/{character}/bindings
        # DESCRIPTION: Published when plan bindings change
        # PAYLOAD: {"bindings": dict, "timestamp": str, "character": str}
        # TRIGGERS: (UI updates)
        # ========================
        self.bindings_publisher = self.session.declare_publisher(f"cognitive/{character_name}/bindings")
        
        
        # Backward compatibility properties
        @property
        def last_action(self):
            """Get the last action for backward compatibility."""
            return self.action_history[-1].action if self.action_history else None
        
        @property
        def last_action_result(self):
            """Get the last action result in uniform format."""
            if not self.action_history:
                return None
            last_record = self.action_history[-1]
            # Return full uniform format if available, otherwise fall back to string result
            if last_record.result_dict:
                return last_record.result_dict
            # Fallback: construct uniform format from string result (backward compatibility)
            if last_record.result:
                return {
                    'status': 'success' if last_record.outcome_status == 'success' else 'failed',
                    'value': last_record.result,
                    'resource_id': None,
                    'reason': None if last_record.outcome_status == 'success' else last_record.result
                }
            return None
        
        # LLM backend - Use SGLang.Runtime for infospace, ZenohLLMClient as fallback
        llm_config = self.character_config.get('llm_config', {})
        server_name = llm_config.get('server_name', 'openai')
        model_name = llm_config.get('model_name', 'gpt-4.1')
        sgl_model_path = llm_config.get('sgl_model_path')
        
        self.runtime = None
        self.llm_client = None
        
        # Check SGLang availability
        HAS_SGLANG = False
        try:
            from incremental_planner import HAS_SGLANG as _HAS_SGLANG
            HAS_SGLANG = _HAS_SGLANG
        except ImportError:
            pass
        
        # Initialize SGLang.Runtime if available and configured 
        if HAS_SGLANG and sgl_model_path:
            try:
                logger.info(f"🚀 Initializing SGLang Runtime with model: {sgl_model_path}")
                self.tokenizer = AutoTokenizer.from_pretrained(sgl_model_path)
                if sgl_model_path.startswith("allenai/Olmo-3"):
                    self.runtime = sgl.Runtime(
                    model_path=sgl_model_path,
                    context_length=32768,
                    cuda_graph_max_bs=4,
                    tp_size=1,
                    mem_fraction_static=0.82,
                    attention_backend="triton"
                )
                elif 'FP8' in sgl_model_path: # patch for FP8 models as of 1/9/2026
                    logger.info(f"🚀 Initializing SGLang Runtime with FP8 patch!")
                    self.runtime = sgl.Runtime(
                        model_path=sgl_model_path,
                        tokenizer_path=sgl_model_path,
                        device="cuda",
                        context_length=65536,
                        dtype="auto",
                        tp_size=1,
                        mem_fraction_static=0.9,
                        fp8_gemm_runner_backend="triton",
                        attention_backend="flashinfer"
                    )
                else:
                    self.runtime = sgl.Runtime(
                        model_path=sgl_model_path,
                        tokenizer_path=sgl_model_path,
                        device="cuda",
                        context_length=65536,
                        dtype="auto",
                        tp_size=1,
                        mem_fraction_static=0.9,
                        attention_backend="flashinfer"
                    )
                sgl.set_default_backend(self.runtime)
                logger.info(f'🤖 SGLang Runtime initialized (model={sgl_model_path})')
                
                # Start OpenAI-compatible API server on port 5000
                try:
                    from sglang_api_server import SGLangAPIServer
                    self.api_server = SGLangAPIServer(
                        runtime=self.runtime,
                        model_path=sgl_model_path,
                        port=5000
                    )
                    self.api_server.start()
                    logger.info(f'🌐 OpenAI-compatible API server started on port 5000')
                except Exception as e:
                    logger.warning(f"Failed to start API server: {e}")
                    self.api_server = None
            except Exception as e:
                logger.error(f"Failed to initialize SGLang Runtime: {e}")
                logger.info("Falling back to ZenohLLMClient")
                self.runtime = None
                self.api_server = None
        
        # Fallback to ZenohLLMClient if SGLang not available
        if not self.runtime:
            self.llm_client = ZenohLLMClient(server_name=server_name, model_name=model_name, service_timeout=200.0 if not self.debug else 300.0)
            logger.info(f'🤖 LLM client initialized (server={server_name}, model={model_name})')
        
        # Create llm_generate wrapper function (unified interface for LLM calls)
        def llm_generate(messages, bindings=None, max_tokens=2000, temperature=0.7, is_json=False, stops=None):
            """Unified LLM generation interface - uses SGLang Runtime if available, else ZenohLLMClient."""
            if self.runtime:
                # Use SGLang Runtime - apply bindings if provided
                if bindings and isinstance(messages, list):
                    processed_messages = []
                    for msg in messages:
                        if isinstance(msg, str):
                            # Apply template bindings
                            processed_msg = msg
                            for key, value in bindings.items():
                                processed_msg = processed_msg.replace(f"{{${key}}}", str(value))
                            processed_messages.append(processed_msg)
                        else:
                            processed_messages.append(msg)
                    messages = processed_messages
                
                # Use infospace_executor's _sglang_generate method via temporary executor instance
                if self.infospace_executor:
                    return self.infospace_executor._sglang_generate(messages, max_tokens, temperature, stops, is_json)
                else:
                    logger.error("SGLang Runtime available but no infospace_executor to use it")
                    return type('Response', (), {'success': False, 'error': 'No executor', 'text': ''})()
            else:
                # Fallback to ZenohLLMClient
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
        from pathlib import Path
        # Memory path relative to project root (where data/ directory exists)
        project_root = Path(__file__).parent.parent  # src/ -> project root
        memory_path = project_root / "data" / "memory" / f"{self.character_name}_memory.json"
        self.memory = Memory(
            character_name=self.character_name,
            llm_generate=self.llm_generate,
            persistence_path=str(memory_path)
        )
        logger.info(f'🧠 Memory module initialized for {self.character_name}')
        
        # Subscriber for save_all command (forward to memory and resource_manager)
        self.save_subscriber = self.session.declare_subscriber(
            "cognitive/save_all",
            self._handle_save_command
        )
        logger.info(f'💾 Subscribed to cognitive/save_all')
        
        # Infospace is always enabled now (physical world removed)
        self.map_name = self.character_config.get('map_name', 'infolab')
        self.infospace_executor = None
        
        # Load tools for infospace planning
        self.available_tools = self._load_tools()
        logger.info(f'🔧 Loaded {len(self.available_tools)} tools for {self.character_name}')
        
        # Initialize infospace components
        from infospace_executor import InfospaceExecutor
        from infospace_resource_manager import InfospaceResourceManager
        from pathlib import Path
        
        # Create resource manager for direct resource access
        world_config = self.character_config.get('world_config', {})
        self.resource_manager = InfospaceResourceManager(self.map_name, session=self.session, world_config=world_config)
        logger.info(f'📦 Resource manager initialized for {self.map_name}')
        
        # Load resources from file on startup
        self.resource_manager.load_from_file()
        logger.info(f'📂 Loaded resources from file for {self.map_name}')
        
        # Get world_name from world_config if available, otherwise use map_name (legacy fallback)
        world_config = self.character_config.get('world_config', {})
        world_name = world_config.get('world_name') or self.map_name
        
        # Initialize action_counter before creating executor (executor init may trigger init tool which publishes actions)
        self.action_counter = 0
        
        # Initialize InfospaceExecutor first (needed for WorldModel and ToolModel)
        self.infospace_executor = InfospaceExecutor(
            agent_name=character_name,
            session=self.session,
            world_name=world_name,
            llm_client=self.runtime or self.llm_client,  # Pass runtime if available, else llm_client
            available_tools=self.available_tools,
            executive_node=self,  # Pass actual executive node
            resource_manager=self.resource_manager
        )
        # Share runtime with executor
        if self.runtime:
            self.infospace_executor.runtime = self.runtime
            self.infospace_executor.tokenizer = self.tokenizer
        logger.info(f'🧩 Infospace executor initialized for {character_name}')
        
        # Create WorldModel instance (with executor)
        from world_model import WorldModel
        self.world_model = WorldModel(
            world_name=world_name,
            agent_name=character_name,
            resource_manager=self.resource_manager,
            executor=self.infospace_executor,
            available_tools=self.available_tools
        )
        logger.info(f'🌍 WorldModel initialized for {character_name} in {world_name}')
        
        # Create ToolModel instance (with executor)
        from tool_model import ToolModel
        self.tool_model = ToolModel(
            world_name=world_name,
            agent_name=character_name,
            resource_manager=self.resource_manager,
            executor=self.infospace_executor,
            available_tools=self.available_tools
        )
        self.tool_model.build_task_tool_index()
        logger.info(f'🔧 ToolModel initialized for {character_name} in {world_name}')
        
        # Attach models to executor for access by planner
        self.infospace_executor.world_model = self.world_model
        self.infospace_executor.tool_model = self.tool_model
        
        # World initialization removed - worlds are assumed to run as separate servers
        
        # Initialize conversation collections
        self._initialize_conversation_collections()
        
        # Initialize planners (reused across all goals)
 
        # IncrementalPlanner for plan generation (SGLang-based)
        self.incremental_planner = None
        try:
            from incremental_planner import IncrementalPlanner, HAS_SGLANG
            
            if HAS_SGLANG:
                # Get SGLang model path from config
                llm_config = self.character_config.get('llm_config', {})
                sgl_model_path = llm_config.get('sgl_model_path')   
                
                self.incremental_planner = IncrementalPlanner(
                    executor=self.infospace_executor,
                    available_tools=self.available_tools,
                    logger_instance=logger,
                    sgl_model_path=sgl_model_path
                )
                logger.info(f'🚀 IncrementalPlanner initialized (SGLang) for {character_name}')
            else:
                logger.warning(f'⚠️  SGLang not available - incremental planning disabled')
        except Exception as e:
            logger.warning(f'⚠️  Failed to initialize IncrementalPlanner: {e}')
            import traceback
            traceback.print_exc()
        
        # Internal state (action_counter already initialized above, before executor creation)
        self.last_sense_data = None
        
        # OODA loop state
        self.current_goal = None
        self.goal_source = None  # Track goal origin: 'ui', 'autonomous', or None
        self.awaiting_user_input = False  # Pause autonomous behavior after UI goal completion
        self.continuous_mode = False  # Continuous mode: resubmit goal on completion
        self.continuous_goal_text = None  # Original goal text for continuous resubmission
        self.last_completed_goal_text = None  # Last completed goal text (for enabling continuous mode after completion)
        self.plan_just_generated = False  # Skip execution on same turn as plan generation
        self.observations = None
        self.text_input_queue = []
        self.action_history = []  # List of ActionRecord instances
        self.last_say_text = ''
        self._last_published_final_answer = None  # Track last published final_answer to prevent duplicates
        
        # Ask primitive state
        self.awaiting_ask_response = False
        
        # Track last action outputs for plan_result
        self.last_say_text = ''
        self.last_out_resource_id = None
        
        # World environments removed - worlds run as separate servers
        
        # Plan execution state
        self.current_step = None
        self.current_plan = None
        self.plan_bindings_cache = {}
        # Snapshot of percepts at plan start (normalized, optional)
        self.percepts_at_plan: Optional[List[Dict[str, Any]]] = None
        # Note: plan_bindings now live in infospace_executor.plan_bindings (single source of truth)
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
        
        # Execution control (replaces turn management)
        self.execution_paused = True  # Start paused, wait for step/run command
        self.execution_mode = 'step'  # 'step', 'run', or 'autonomous'
        self.autonomous_mode = False  # Flag for autonomous execution mode
        self.interrupt_requested = False  # Global interrupt flag (checked once per planner step)
        
        # Subscribers for direct execution control from UI
        self.control_autonomous_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/control/autonomous",
            self.handle_autonomous_command
        )
        self.control_stop_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/control/stop",
            self.handle_stop_command
        )
        self.control_interrupt_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/control/interrupt",
            self.handle_interrupt_command
        )
        self.control_continuous_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/control/continuous",
            self.handle_continuous_toggle
        )
        self.control_clear_world_model_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/control/clear_world_model",
            self.handle_clear_world_model
        )
        self.control_clear_map_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/control/clear_map",
            self.handle_clear_map
        )
        self.control_clear_transients_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/control/clear_transients",
            self.handle_clear_transients
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
        
        # Subscriber for enabling compliance tracking (evaluation mode)
        self.compliance_tracking_subscriber = self.session.declare_subscriber(
                f"cognitive/{character_name}/enable_compliance_tracking",
                self._enable_compliance_tracking_callback
            )
        
        self.execute_saved_plan_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/execute_saved_plan",
            self._handle_execute_saved_plan
        )
        
        # Queryables for resource management (for UI and resource_browser)
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
        self.resource_create_note_queryable = self.session.declare_queryable(
            f"cognitive/{character_name}/resource/create_note",
            self._handle_resource_create_note_query
        )
        
        self.resource_clear_transient_queryable = self.session.declare_queryable(
            f"cognitive/{character_name}/resource/clear_transient",
            self._handle_resource_clear_transient_query
        )

        self.resource_reset_models_queryable = self.session.declare_queryable(
            f"cognitive/{character_name}/resource/reset_models",
            self._handle_resource_reset_models_query
        )
        
        self.llm_generate_queryable = self.session.declare_queryable(
            f"cognitive/{character_name}/llm/generate",
            self._handle_llm_generate_query
        )
        
        # Queryable for sync plan execution (character-specific)
        self.sync_plan_execution_queryable = self.session.declare_queryable(
            f"cognitive/{character_name}/execute_plan_sync",
            self._sync_plan_execution_handler
        )
        
        self.plan_bindings_queryable = self.session.declare_queryable(
            f"cognitive/{character_name}/plan_bindings",
            self._plan_bindings_query_handler
        )
        
        self.world_state_queryable = self.session.declare_queryable(
            f"cognitive/{character_name}/world_state",
            self._world_state_query_handler
        )
        
        # Queryable for explicit planner bindings clearing (character-specific)
        self.clear_planner_bindings_queryable = self.session.declare_queryable(
            f"cognitive/{character_name}/clear_planner_bindings",
            self._clear_planner_bindings_handler
        )
        
        # Queryable for planner feedback (character-specific)
        self.planner_feedback_queryable = self.session.declare_queryable(
            f"cognitive/{character_name}/planner/feedback",
            self._planner_feedback_handler
        )
        
        # Shutdown flags
        self.shutdown_requested = False
        self._shutting_down = False

        logger.info(f'🧠 Zenoh Executive Node initialized for character: {character_name}')
        logger.info(f'   - Subscribing to: cognitive/{character_name}/sense_data')
        logger.info(f'   - Subscribing to: cognitive/{character_name}/control/autonomous')
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
            if self.resource_manager:
                self.resource_manager.save_to_file()
                logger.info(f'💾 Saved resource manager state for {self.map_name}')
        except Exception as e:
            logger.error(f'Error handling save command: {e}')
    
    def _handle_execute_saved_plan(self, sample):
        """Handle execute_saved_plan command - load and execute a saved plan."""
        try:
            payload_bytes = sample.payload.to_bytes()
            data = json.loads(payload_bytes.decode('utf-8'))
            plan_name = data.get("plan_name")
            
            if not plan_name:
                logger.error(f'execute_saved_plan: missing plan_name')
                return
            
            plan_path = Path('saved_plans') / plan_name / 'plan.json'
            if not plan_path.exists():
                logger.error(f'execute_saved_plan: plan not found: {plan_path}')
                return
            
            logger.info(f'📂 {self.character_name} loading saved plan: {plan_name}')
            plan_data = json.loads(plan_path.read_text())
            
            goal_text = plan_data.get("goal", "")
            match = re.match(r'Goal (.+?):\s*;\s*actors:', goal_text)
            goal_description = match.group(1) if match else goal_text
            
            self.current_goal = Goal(goal_description, [self.character_name], description='', termination='')
            self.current_plan = plan_data.get("plan")
            
            self._publish_goal(self.current_goal)
            self._publish_current_plan()
            
            logger.info(f'▶️  {self.character_name} executing saved plan: {plan_name}')
            
            result = self.infospace_executor.execute_plan_sync(self.current_plan)
            
            if result['status'] == 'success':
                logger.info(f'✅ {self.character_name} saved plan completed: {plan_name}')
            else:
                logger.error(f'❌ {self.character_name} saved plan failed: {plan_name}')
                logger.error(f'  Step {result.get("executed_steps", 0)}: {result.get("reason", "unknown")}')
                logger.error(f'  Bindings at failure: {list(self.infospace_executor.plan_bindings_flat.keys())}')
        
        except Exception as e:
            logger.error(f'Error executing saved plan: {e}')
            traceback.print_exc()
    
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
                # Check if there's a goal in queue that should unpause execution
                # (either explicit 'goal:' prefix or User text that will be converted to goal)
                if self.execution_paused and self.text_input_queue:
                    for queued in self.text_input_queue:
                        content = queued.get('content', '')
                        try:
                            content_data = json.loads(content)
                            text = content_data.get('text', '')
                            source = content_data.get('source', 'unknown')
                        except (json.JSONDecodeError, TypeError):
                            text = content
                            source = 'console'
                        # Unpause for explicit goals OR User text (which gets converted to goal)
                        if text.strip().startswith('goal:') or source == 'User':
                            logger.info(f'🚀 Goal/User input in queue, unpausing execution')
                            self.execution_paused = False
                            self._publish_execution_state()
                            break
                
                if self.execution_paused:
                    time.sleep(0.2)
                else:
                    try:
                        # Priority order: Text input → Normal OODA
                        if self.text_input_queue:
                            self._process_text_input()
                        self._run_ooda_loop()
                        
                        # Check autonomous mode completion conditions
                        if self.execution_mode == 'autonomous':
                            self.run_autonomous_mode()
                            if not self.autonomous_mode:
                                # Autonomous mode completed or stopped, pause and enable button
                                self.execution_mode = 'step'
                                self.execution_paused = True
                                self._publish_execution_state()
                        # In step mode, pause after one OODA cycle
                        elif self.execution_mode == 'step':
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
            goal_preview = goal.to_string()[:80] + ('...' if len(goal.to_string()) > 80 else '')
            logger.info(f'🎯 Published goal for {self.character_name}: {goal_preview}')
            
        except Exception as e:
            logger.error(f'Error publishing goal: {e}')

    def _publish_action_result(self, action: Dict, result: Dict, action_type: str, timestamp: datetime):
        """Publish action result to action topic for UI display."""
        try:
            # Normalize action_type - handle both 'create-note' and 'createnote' formats
            normalized_type = action_type.replace('_', '-').lower()
            
            # Prefix with method_name if within a method protocol
            if '_inner_loop' in action:
                inner_loop = action['_inner_loop']
                method_name = inner_loop.get('method_name')
                if method_name:
                    normalized_type = f"{method_name}:{normalized_type}"
            
            # Format action data for UI display
            action_data = {
                'type': normalized_type,
                'action_type': normalized_type,  # Also set action_type for UI compatibility
                'action_id': f'{normalized_type}_{int(timestamp.timestamp() * 1000)}',
                'timestamp': timestamp.isoformat(),
                'character': self.character_name,
                'status': result.get('status', 'unknown')
            }
            
            # Add inner loop metadata if present
            if '_inner_loop' in action:
                action_data['inner_loop'] = action['_inner_loop']
            
            # Add action-specific fields for UI display
            # Handle both hyphenated and non-hyphenated action types
            action_types_to_handle = ['create-note', 'createnote', 'create-collection', 'createcollection', 
                                     'split', 'flatten', 'map', 'summarize', 'search-web', 'semantic-scholar']
            if normalized_type in action_types_to_handle:
                target = action.get('target', '')
                value = action.get('value', '')
                out_var = action.get('out', '')
                
                # Prefer displaying a bound variable value if target references a plan variable
                display_target = target
                if isinstance(target, str) and target.startswith('$'):
                    var_name = target[1:]
                    if self.infospace_executor:
                        binding = self.infospace_executor.plan_bindings_flat.get(var_name)
                        if binding:
                            display_target = binding
                
                # Always include full action JSON for UI display
                if isinstance(action, dict):
                    action_data['action'] = json.dumps(action)
                else:
                    action_data['action'] = json.dumps(action) if not isinstance(action, str) else action
                
                # Show what was created/processed (using uniform format)
                if result.get('status') == 'success':
                    result_value = result.get('value', '')
                    resource_id = result.get('resource_id')
                    if result_value:
                        action_data['result'] = result_value
                    if resource_id:
                        action_data['resource_id'] = resource_id
                    action_data['target'] = target  # Keep original target (variable name)
                    action_data['resolved_target'] = display_target  # Show resolved value if different
                    logger.debug(f'Published action result: {normalized_type} -> value={result_value}, resource_id={resource_id}, target={target}')
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
            
            serialized_action = json.dumps(action_data)
            # Truncate serialized_action for logging (keep full version for publisher)
            log_action = serialized_action
            if len(log_action) > 128:
                log_action = log_action[:125] + "..."
            logger.info(f"ACTION_LOG {normalized_type}: {log_action}")
            self.action_publisher.put(serialized_action)
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
                    if self.infospace_executor:
                        binding = self.infospace_executor.plan_bindings_flat.get(var_name)
                        if binding:
                            display_target = binding
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
        if not isinstance(self.current_plan, dict):
            self.current_plan = {}
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
            if self.infospace_executor and self.infospace_executor.plan_bindings:
                logger.info(f'🔗 {self.character_name} current plan_bindings: {self.infospace_executor.plan_bindings}')
            
            # Publish plan_result for external consumers
            self._publish_plan_result()
            
        except Exception as e:
            logger.error(f'Error publishing current plan: {e}')

    def _publish_goal_result(self, result: Dict[str, Any]):
        """Publish complete goal result from _plan() for external consumers (e.g., eval scripts)."""
        try:
            # Add metadata to the result
            goal_result = result.copy()
            goal_result['timestamp'] = datetime.now().isoformat()
            goal_result['character'] = self.character_name
            
            # Add bindings if available
            if self.infospace_executor:
                goal_result['bindings'] = self.infospace_executor.plan_bindings_flat
            
            self.goal_result_publisher.put(json.dumps(goal_result))
            logger.info(f'📤 Published goal_result for {self.character_name}')
        except Exception as e:
            logger.error(f'Error publishing goal_result: {e}')
    
    def _publish_plan_result(self):
        """Publish plan result for external consumers (e.g., MMLU eval)."""
        if not self.current_plan:
            return
        
        # Get bindings as Note IDs
        bindings = {}
        if self.infospace_executor:
            bindings = self.infospace_executor.plan_bindings_flat
        
        # Get final_thoughts from plan response
        final_thoughts = self.current_plan.get('response', '')
        
        # Get final_content: prefer last_say_text, else last_out_resource content
        final_content = getattr(self, 'last_say_text', '') or ''
        if not final_content:
            last_out_id = getattr(self, 'last_out_resource_id', None)
            if last_out_id and self.infospace_executor:
                final_content = self.infospace_executor._get_content(last_out_id) or ''
                if not isinstance(final_content, str):
                    final_content = json.dumps(final_content)
        
        # Get last_action_result in uniform format and add action field for external API consumers
        last_action_result = None
        # Safely access property - it may return None if action_history is empty
        if hasattr(self, 'action_history') and self.action_history:
            try:
                last_action_result = self.last_action_result
            except (AttributeError, IndexError):
                # Property may fail if action_history is in unexpected state
                pass
        
        if last_action_result and isinstance(last_action_result, dict):
            # Get action type from last action in history
            if self.action_history:
                last_action = self.action_history[-1].action
                if isinstance(last_action, dict):
                    last_action_result = last_action_result.copy()
                    last_action_result['action'] = last_action.get('type', 'unknown')
        
        plan_result = {
            'status': 'complete' if self.current_plan.get('success') else 'failed',
            'final_thoughts': final_thoughts,
            'final_content': final_content,
            'bindings': bindings,
            'timestamp': datetime.now().isoformat(),
            'character': self.character_name,
            'last_action_result': last_action_result  # Uniform format: {status, value, resource_id, reason, action}
        }
        
        self.plan_result_publisher.put(json.dumps(plan_result))
        logger.info(f'📤 Published plan_result for {self.character_name}')
        
        # Save response to conversation collection
        # Prefer actual 'say' action content over FINAL_ANSWER if both exist
        last_say = (getattr(self, 'last_say_text', '') or '').strip()
        final_thoughts_clean = final_thoughts.strip() if final_thoughts else ''
        
        # Determine what to save to conversation
        response_to_save = None
        if last_say and len(last_say) > 0:
            # Use actual 'say' action content if it exists
            response_to_save = last_say
            logger.debug(f'Using actual say action content for conversation: {last_say[:50]}...')
        elif final_thoughts_clean and len(final_thoughts_clean) >= 20 and final_thoughts_clean.replace(' ', '').replace('.', '').replace(',', '').replace('!', '').replace('?', '').strip():
            # Fall back to FINAL_ANSWER if no actual say action
            response_to_save = final_thoughts_clean
            logger.debug(f'Using FINAL_ANSWER for conversation (no say action found): {final_thoughts_clean[:50]}...')
        
        # Save to conversation if we have content
        if response_to_save:
            self._add_to_conversation(f"{self.character_name} responds: {response_to_save}")
        
        # Publish final_answer as a "say" action to User so it appears in action log
        # Only publish if final_thoughts is non-empty, meaningful, and different from last_say_text
        # (to avoid duplicating the actual say action that was already published)
        # Require meaningful content: at least 20 chars and not just whitespace/punctuation
        if final_thoughts_clean and len(final_thoughts_clean) >= 20 and final_thoughts_clean.replace(' ', '').replace('.', '').replace(',', '').replace('!', '').replace('?', '').strip():
            # Only publish if final_thoughts is different from the last say action
            # This prevents duplicating the actual say action content
            if final_thoughts_clean != last_say:
                # Check if we've already published this final_answer for this plan
                plan_id = getattr(self, 'current_plan_id', None)
                last_published_final = getattr(self, '_last_published_final_answer', None)
                if last_published_final != final_thoughts_clean:
                    final_answer_action = {
                        'type': 'say',
                        'action_type': 'say',
                        'action_id': f'final_answer_{int(time.time() * 1000)}',
                        'timestamp': datetime.now().isoformat(),
                        'text': final_thoughts_clean,
                        'source': self.character_name,
                        'target': 'User',
                        'is_text_only': True
                    }
                    self.action_publisher.put(json.dumps(final_answer_action))
                    self._last_published_final_answer = final_thoughts_clean
                    logger.info(f'📤 Published FINAL_ANSWER to action log: {final_thoughts_clean[:100]}...')
                else:
                    logger.debug(f'Skipping final_answer publication - already published for this plan')
            else:
                logger.debug(f'Skipping final_answer publication - same as last_say_text')
        elif final_thoughts_clean:
            # Log when we skip due to insufficient content (for debugging)
            logger.debug(f'Skipping final_answer publication - content too short or only punctuation: "{final_thoughts_clean[:50]}..."')

    def _initialize_conversation_collections(self):
        """
        Initialize conversation collections on startup.
        Creates 'conversation_history' collection if it doesn't exist and makes it persistent.
        Creates 'conversation' collection if it doesn't exist.
        """
        if self.benchmark_mode:
            logger.info('Benchmark mode: skipping conversation collection initialization')
            return
        
        if not self.infospace_executor:
            logger.warning('Infospace executor not available, skipping conversation collection initialization')
            return
        
        try:
            # Check if conversation_history exists, create if not
            load_action = {
                "type": "load",
                "target": "conversation_history",
                "out": "$conv_history"
            }
            result = self.infospace_executor.execute_action(load_action)
            
            if result.get('status') != 'success' or not result.get('resource_id'):
                # Collection doesn't exist, create it
                create_action = {
                    "type": "create-collection",
                    "name": "conversation_history",
                    "out": "$conv_history"
                }
                result = self.infospace_executor.execute_action(create_action)
                if result.get('status') == 'success':
                    # Make it persistent
                    persist_action = {
                        "type": "persist",
                        "target": "$conv_history"
                    }
                    persist_result = self.infospace_executor.execute_action(persist_action)
                    if persist_result.get('status') == 'success':
                        logger.info(f'✓ Created and persisted conversation_history collection')
                    else:
                        logger.warning(f'Failed to persist conversation_history: {persist_result.get("reason", "unknown")}')
                else:
                    logger.warning(f'Failed to create conversation_history: {result.get("reason", "unknown")}')
            else:
                logger.info(f'✓ conversation_history collection already exists')
            
            # Check if conversation exists, create if not
            load_action = {
                "type": "load",
                "target": "conversation",
                "out": "$conv"
            }
            result = self.infospace_executor.execute_action(load_action)
            
            if result.get('status') != 'success' or not result.get('resource_id'):
                # Collection doesn't exist, create it
                create_action = {
                    "type": "create-collection",
                    "name": "conversation",
                    "out": "$conv"
                }
                result = self.infospace_executor.execute_action(create_action)
                if result.get('status') == 'success':
                    logger.info(f'✓ Created conversation collection')
                else:
                    logger.warning(f'Failed to create conversation: {result.get("reason", "unknown")}')
            else:
                logger.info(f'✓ conversation collection already exists')
                
        except Exception as e:
            logger.error(f'Error initializing conversation collections: {e}')
            import traceback
            traceback.print_exc()

    def _archive_conversation(self):
        """
        Archive conversation on shutdown: summarize conversation collection and add to conversation_history.
        Only archives if conversation collection exists and is not empty.
        """
        logger.info(f'📝 Starting conversation archiving for {self.character_name}...')
        
        if not self.infospace_executor:
            logger.warning('Infospace executor not available, skipping conversation archiving')
            return
        
        try:
            # Load conversation collection
            load_action = {"type": "load", "target": "conversation", "out": "$conv"}
            result = self.infospace_executor.execute_action(load_action)
            
            if result.get('status') != 'success' or not result.get('resource_id'):
                logger.info('No conversation collection found, skipping archiving')
                return
            
            conv_collection_id = result.get('resource_id')
            
            # Check if collection is empty using size primitive
            size_action = {"type": "size", "target": "$conv", "out": "$conv_size"}
            size_result = self.infospace_executor.execute_action(size_action)
            
            if size_result.get('status') != 'success':
                logger.warning(f'Failed to get conversation size: {size_result.get("reason", "unknown")}')
                return
            
            # Get size value (should be an integer)
            conv_size = size_result.get('value', 0)
            try:
                conv_size = int(conv_size) if isinstance(conv_size, str) else conv_size
            except (ValueError, TypeError):
                logger.warning(f'Invalid conversation size value: {conv_size}')
                return
            
            if conv_size == 0:
                logger.info('Conversation collection is empty, skipping archiving')
                return
            
            logger.info(f'📝 Archiving conversation with {conv_size} items...')
            
            # Summarize conversation collection using summarize tool
            # The summarize tool creates a Note and binds it to $summary
            summarize_action = {"type": "summarize", "target":  "$conv", "out": "$summary"}
            summarize_result = self.infospace_executor.execute_action(summarize_action)
            
            if summarize_result.get('status') != 'success':
                logger.warning(f'Failed to summarize conversation: {summarize_result.get("reason", "unknown")}')
                return
            
            # The summarize tool already created a Note and bound it to $summary
            # Get the Note ID from the result (resource_id) or from plan_bindings
            summary_note_id = summarize_result.get('resource_id')
            if not summary_note_id:
                # Try to get from plan_bindings
                summary_note_id = self.infospace_executor.plan_bindings_flat.get('summary')
            if not summary_note_id:
                logger.warning('Summarize tool did not return a Note ID')
                return
            
            # Verify the summary note has content
            summary_content = self.infospace_executor._get_content(summary_note_id)
            if not summary_content or (isinstance(summary_content, str) and not summary_content.strip()):
                logger.warning('Summarization returned empty result, skipping archiving')
                return
            
            # Ensure summary note is persisted before adding to persistent collection
            persist_action = {"type": "persist", "target": "$summary"}
            persist_result = self.infospace_executor.execute_action(persist_action)
            if persist_result.get('status') != 'success':
                logger.warning(f'Failed to persist summary note: {persist_result.get("reason", "unknown")}')
                # Continue anyway - note will be saved when resource manager saves
            
            # Add the summary note (already created by summarize tool) to conversation_history
            add_action = {"type": "add", "target": "conversation_history", "value": "$summary", "out": "conversation_history"}
            add_result = self.infospace_executor.execute_action(add_action)
            
            if add_result.get('status') == 'success':
                logger.info(f'✓ Successfully archived conversation summary ({conv_size} items) to conversation_history')
            else:
                logger.warning(f'Failed to add summary to conversation_history: {add_result.get("reason", "unknown")}')
                
        except Exception as e:
            logger.error(f'Error archiving conversation: {e}')
            import traceback
            traceback.print_exc()
        finally:
            logger.info(f'📝 Conversation archiving completed for {self.character_name}')

    def _add_to_conversation(self, content: str):
        """
        Create a note with content and add it to the 'conversation' collection.
        Creates the collection if it doesn't exist.
        """
        if self.benchmark_mode:
            return  # Skip adding to conversation in benchmark mode
        
        if not self.infospace_executor:
            logger.warning('Infospace executor not available, skipping conversation note creation')
            return
        
        try:
            # Create note with content
            create_action = {
                "type": "create-note",
                "value": content,
                "out": "$conv_note"
            }
            result = self.infospace_executor.execute_action(create_action)
            if result.get('status') != 'success':
                logger.warning(f'Failed to create conversation note: {result.get("reason", "unknown")}')
                return
            
            note_id = result.get('resource_id')
            if not note_id:
                logger.warning(f'Created conversation note but no resource_id returned')
                return
            
            # Ensure conversation collection exists
            load_action = {
                "type": "load",
                "target": "conversation",
                "out": "$conv"
            }
            load_result = self.infospace_executor.execute_action(load_action)
            
            if load_result.get('status') != 'success' or not load_result.get('resource_id'):
                # Collection doesn't exist, create it
                create_collection_action = {
                    "type": "create-collection",
                    "name": "conversation",
                    "out": "$conv"
                }
                create_result = self.infospace_executor.execute_action(create_collection_action)
                if create_result.get('status') != 'success':
                    logger.warning(f'Failed to create conversation collection: {create_result.get("reason", "unknown")}')
                    return
            
            # Add note to conversation collection
            add_action = {
                "type": "add",
                "target": "conversation",
                "value": "$conv_note",
                "out": "conversation"
            }
            add_result = self.infospace_executor.execute_action(add_action)
            if add_result.get('status') == 'success':
                logger.debug(f'✓ Added note to conversation collection')
            else:
                logger.warning(f'Failed to add note to conversation: {add_result.get("reason", "unknown")}')
                
        except Exception as e:
            logger.error(f'Error adding to conversation: {e}')
            import traceback
            traceback.print_exc()

    def _create_character_note(self):
        """
        Create the character note with character_name as the note name.
        Only creates if it doesn't already exist (once per session).
        """
        if not self.infospace_executor or not self.resource_manager:
            logger.warning('Infospace executor or resource manager not available, skipping character note creation')
            return False
        
        # Check if character note already exists
        if self.character_name in self.resource_manager.named_notes:
            logger.info(f'Character note "{self.character_name}" already exists, skipping creation')
            return True
        
        # Build scenario content from character config
        character_desc = self.character_config.get('character', '').strip()
        backstory = self.character_config.get('backstory', '').strip()
        capabilities = self.character_config.get('capabilities', '').strip()
        setting = self.character_config.get('setting', '').strip()
        drives = self.character_config.get('drives', [])
        
        # Format drives
        if isinstance(drives, list):
            drives_text = '\n'.join(f"- {d}" for d in drives)
        else:
            drives_text = str(drives)
        
        # Build scenario content
        scenario_content = f"You are {self.character_name}\n\n"
        if character_desc:
            scenario_content += f"## Character Description\n{character_desc}\n\n"
        if backstory:
            scenario_content += f"## Backstory\n{backstory}\n\n"
        if capabilities:
            scenario_content += f"## Capabilities\n{capabilities}\n\n"
        if setting:
            scenario_content += f"## Setting & World Rules\n{setting}\n\n"
        if drives_text:
            scenario_content += f"## Drives\n{drives_text}\n\n"
        scenario_content += "Respond to User, who just said:\n\n"
        
        # Create note with character_name as the note name
        create_action = {
            "type": "create-note",
            "value": scenario_content,
            "name": self.character_name,
            "out": "$character_note"
        }
        result = self.infospace_executor.execute_action(create_action)
        if result.get('status') == 'success':
            logger.info(f'✓ Created character note: "{self.character_name}"')
            return True
        else:
            logger.warning(f'Failed to create character note: {result.get("reason", "unknown")}')
            return False

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
            goal = clean_input.strip().strip('"').strip("'")
            
            # If awaiting ask response, the ask primitive is polling text_input_queue directly
            # So we should not process text inputs here - they'll be consumed by the polling loop
            
            # Check for direct JSON action execution (before other special commands)
            if source == 'User' and clean_input.strip().startswith('{'):
                try:
                    action_dict = json.loads(clean_input)
                    if isinstance(action_dict, dict) and 'type' in action_dict:
                        # Valid action JSON - execute directly
                        logger.info(f'🔧 Direct action execution: {action_dict.get("type")}')
                        result = self.infospace_executor.execute_action(action_dict)
                        timestamp = datetime.now()
                        self._publish_action_result(action_dict, result, action_dict.get('type'), timestamp)
                        return  # Don't process as normal input
                except json.JSONDecodeError:
                    # Not valid JSON, fall through to normal processing
                    pass
                except Exception as e:
                    # Execution error - log and publish error result
                    logger.error(f'❌ Direct action execution failed: {e}')
                    error_result = {"status": "failed", "reason": str(e)}
                    timestamp = datetime.now()
                    action_type = action_dict.get('type', 'unknown') if 'action_dict' in locals() else 'unknown'
                    self._publish_action_result(action_dict if 'action_dict' in locals() else {}, error_result, action_type, timestamp)
                    return
            
            # Handle special commands from User BEFORE processing as dialog
            if source == 'User':
                if clean_input.startswith('goal:'):
                    # Explicit goal command - process as before
                    goal_preview = clean_input[:80] + ('...' if len(clean_input) > 80 else '')
                    logger.info(f'📥 {self.character_name} Received goal: "{goal_preview}"')
                    self.parse_and_set_goal("", clean_input[5:].strip())
                    return  # Don't process as speech
                else:
                    # Regular user input - ensure character note exists and load it
                    # Create character note if it doesn't exist (only once per session)
                    self._create_character_note()
                    
                    # Create note with user input and add to conversation collection
                    self._add_to_conversation(f"User says: {clean_input}")
                    
                    # Prepend load instruction to user input using character_name
                    template = f"""{clean_input}"""
                    
                    # Convert to goal format
                    logger.info(f'📥 {self.character_name} Received goal: "{goal}"')
                    self.parse_and_set_goal(template, goal)
                    # Unpause execution so plan executes immediately (same as explicit goal: prefix)
                    self.execution_paused = False
                    self._publish_execution_state()
                    return  # Don't process as speech
            
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
        """Format the situation data for the LLM (infospace only - no spatial data)."""
        formatted_situation = ''
        
        # Only include plan summary if available (legacy inspections/uses removed - not used in infospace)
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
        # Build system prompt fresh from character_config (not cached observations)
        # This ensures ScienceWorld-populated setting is always current
        system_prompt = ''
        if self.character_config.get('character', None):
            system_prompt = self.character_config['character']
        if self.character_config.get('character', None):
            system_prompt += f"\n#Your capabilities are:\n\t{self.character_config['capabilities']}\n"
        if self.character_config.get('drives', None):
            system_prompt += f"\n#Your drives are:\n\t{'\n\t'.join(self.character_config['drives'])}\n"
        if self.character_config.get('setting', None):
            system_prompt += f"\n#The world you are operating in is:\n{self.character_config['setting']}\n"
        
        # Add dynamic updates
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
        """Observe: Collect current situation and sense data (infospace only)."""
        try:
            # Use _update_system_prompt() to get fresh system prompt (includes ScienceWorld-populated setting)
            system_prompt = self._update_system_prompt()
            
            # Build dynamic user prompt from situation, entity context, and action history
            user_prompt = self.format_situation()
            
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
            # Fallback: build minimal observations
            system_prompt = self._update_system_prompt()
            user_prompt = self.format_situation()
            self.observations = {'static': system_prompt, 'dynamic': user_prompt}
            return self.observations

    def _orient(self):
        """Orient: Assess current state and drives"""

        prompt = f"""
        You are a strict goal generator.
        Given what you know about the world, your situation and your character, assess your current needs and pressing priorities.
        Given these, generate 2-3 goal statements that you might work on next.
        Finally, prioritize the goals by their importance, urgency, and feasibility, and return the highest priority goal.
        Your response should be a single goal statement, with clear desired outcome and a clear termination condition, followed by </end>.
        Respond with no other text, no markdown, no code fences, no reasoning, no explanation, no commentary.
        """
        response = self.llm_generate(
                messages=[prompt],
                max_tokens=200,
                temperature=0.3,
                stops=['</end>'],
                )
        self.current_goal = Goal(response.text.strip(), [self.character_name], description='', termination='')
        return self.current_goal


    def _plan(self, template, goal: Goal):
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
            # Capture percepts at plan start (always empty for infospace)
            self.percepts_at_plan = []
            
            # Note: plan_bindings are NOT cleared here to preserve bindings from execute_plan_sync
            # Use clear_planner_bindings API to explicitly clear if needed
            goal_text = goal.name
            if goal.description != goal.name and goal.description != '':
                goal_text += ':' + goal.description
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
                'variables': self.infospace_executor.plan_bindings_flat if self.infospace_executor else {},
                'character_context': character_context,
                'recent_context': recent_context,
                'executor': self.infospace_executor  # Pass executor for incremental planner
            }
            
            # Initialize plan identifiers before plan generation (needed for incremental planner action tracking)
            self.plan_counter += 1
            self.current_plan_id = f"p_{self.plan_counter}"
            self.step_counter = 0
            
            result = self.incremental_planner.generate_plan(
                template=template,
                goal=goal_text, 
                context=context, 
                max_steps=16
            )
            self.current_plan = result.get('plan', {})

        return result
    

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
                input_preview = text_input[:60] + ('...' if len(text_input) > 60 else '')
                logger.info(f'📥 {self.character_name} Queued: "{input_preview}" (source: {source})')
                if len(self.text_input_queue) > 3:
                    logger.warning(f'⚠️ Text input queue size {len(self.text_input_queue)} > 3, dropping oldest')
                    self.text_input_queue.pop(0)
                
                # Add conversation entry to memory (default entity is "User")
                entity_name = source if source != 'console' else "User"
                self.memory.add_conversation_entry(entity_name, source, text_input)
                
                # Auto-unpause for goal commands so they execute immediately
                if text_input.strip().startswith('goal:') and self.execution_paused:
                    logger.info(f'🚀 Goal received, unpausing execution')
                    self.execution_paused = False
                    self._publish_execution_state()
                
        except Exception as e:
            traceback.print_exc()
            logger.error(f'Error processing sense data: {e}')
    
    def handle_autonomous_command(self, sample):
        """Handle autonomous command - start autonomous execution mode."""
        try:
            if not hasattr(self, 'character_name'):
                logger.warning('🤖 Autonomous command received before initialization complete')
                return
            logger.info(f'🤖 Autonomous command received by {self.character_name}')
            self.autonomous_mode = True
            self.execution_mode = 'autonomous'
            self.execution_paused = False
            self._publish_execution_state()
        except Exception as e:
            logger.error(f'Error handling autonomous command: {e}')
            traceback.print_exc()
            # Ensure button is enabled (unshaded) on error
            self.autonomous_mode = False
            self.execution_mode = 'step'
            self.execution_paused = True
            self._publish_execution_state()
    
    def run_autonomous_mode(self):
        """Check if autonomous mode should continue or complete."""
        # Autonomous mode completes when:
        # 1. Flag is False (stopped by user)
        # 2. No current goal and no current plan (no work to do)
        if not self.autonomous_mode:
            return  # Already stopped, exit
        #generate a new goal
        prompt = f"""
        You are a strict goal generator.
        You are generating a goal for autonomous mode operation.
        Your previous goal, if any, was: \n{self.previous_autonomous_goal_text}\n
        The resulting autonomous task state, if any, was: \n{self.autonomous_task_state}\n
        Given what you know about the world, your situation and your character, assess your current needs and pressing priorities.
        Given these, generate 2-3 goal statements that you might work on next.
        Finally, prioritize the goals by their importance, urgency, and feasibility, and return the highest priority goal.
        Report to the user the highest priority goal, with clear desired outcome, followed by </end>.
        Do NOT repeat the previous goal unless you are in an immediately life-threatening situation and it is the only way to save yourself.
        Respond with no other text, no markdown, no code fences, no reasoning, no explanation, no commentary.
        """

        planner_response = self.incremental_planner.generate_plan(template='',goal=prompt, context=None, max_steps=16)
        if not planner_response or not planner_response.get('success', False):
            return False
        goal_text = planner_response.get('response', '')
        self.previous_autonomous_goal_text = goal_text
        self.current_goal = Goal(goal_text, [self.character_name], description='', termination='')
        self._publish_goal(self.current_goal)
        planner_response = self.incremental_planner.generate_plan(template='',goal=goal_text, context=None, max_steps=16)
        if not planner_response or not planner_response.get('success', False):
            return False
        self.autonomous_task_state = planner_response.get('task_state', '')
        self.execution_paused = False
        self._publish_execution_state()
        return True
    
    def handle_continuous_toggle(self, sample):
        """Handle continuous mode toggle - can be enabled/disabled at any time."""
        try:
            payload_bytes = sample.payload.to_bytes()
            data = json.loads(payload_bytes.decode('utf-8'))
            enable = data.get('enable', None)
            
            # If enable is None, toggle current state
            if enable is None:
                enable = not self.continuous_mode
            
            if enable:
                # Enable continuous mode: store current goal text for resubmission
                # Use current goal if available, otherwise use last completed goal
                goal_text = None
                if self.current_goal:
                    goal_text = self.current_goal.to_string()
                elif self.last_completed_goal_text:
                    goal_text = self.last_completed_goal_text
                    logger.info(f'🔄 {self.character_name} using last completed goal for continuous mode')
                
                if goal_text:
                    self.continuous_mode = True
                    self.continuous_goal_text = goal_text
                    logger.info(f'🔄 {self.character_name} continuous mode enabled, goal: {self.continuous_goal_text[:80]}...')
                    # If we're using last completed goal and execution is paused, resubmit immediately
                    if not self.current_goal and self.execution_paused:
                        logger.info(f'🔄 {self.character_name} resubmitting last completed goal immediately')
                        goal_text_formatted = f"goal: {self.continuous_goal_text}"
                        self.parse_and_set_goal("", goal_text_formatted)
                else:
                    logger.warning(f'⚠️ {self.character_name} continuous toggle ON but no current or last completed goal')
            else:
                # Disable continuous mode
                self.continuous_mode = False
                self.continuous_goal_text = None
                logger.info(f'🔄 {self.character_name} continuous mode disabled')
            
            self._publish_execution_state()
        except Exception as e:
            logger.error(f'Error handling continuous toggle: {e}')
            traceback.print_exc()
    
    def handle_clear_world_model(self, sample):
        """Handle clear world model command - resets persistent world_model.json to empty and saves."""
        try:
            if not hasattr(self, 'world_model') or not self.world_model:
                logger.error("WorldModel not available, cannot clear world model")
                return

            from world_model import empty_world_model_raw
            self.world_model.world_model = empty_world_model_raw()
            # Keep planner-facing cache consistent in-memory
            if hasattr(self.world_model, "_derive_beliefs"):
                self.world_model._beliefs_cache = self.world_model._derive_beliefs(self.world_model.world_model)
            self.world_model.save()
            logger.info("🗑️ Cleared world_model (reset to empty and saved)")
        except Exception as e:
            logger.error(f'Error clearing world model: {e}')
            traceback.print_exc()
    
    def handle_clear_map(self, sample):
        """Handle clear map command - deletes the persistent map Collection, all its Notes, and the SpatialMap file."""
        try:
            if not self.resource_manager:
                logger.error("Resource manager not available, cannot clear map")
                return
            
            agent_name = getattr(self, 'character_name', 'unknown')
            map_name = f"{agent_name}-minecraft_map"
            
            # === Clear Collection-based map ===
            
            # Resolve named Collection
            collection_id = self.resource_manager.named_collections.get(map_name)
            if not collection_id:
                logger.info(f"Map '{map_name}' not found, nothing to clear")
            else:
                # Get Collection content (list of Note IDs)
                collection_resource = self.resource_manager.get_resource(collection_id)
                if not collection_resource:
                    logger.warning(f"Map Collection {collection_id} resolved but resource not found")
                else:
                    note_ids = collection_resource.get('properties', {}).get('content', [])
                    if not isinstance(note_ids, list):
                        note_ids = []
                    
                    # Delete all Notes in the Collection
                    deleted_notes = 0
                    for note_id in note_ids:
                        if isinstance(note_id, str) and note_id.startswith('Note_'):
                            success, _ = self.resource_manager.delete_resource(note_id)
                            if success:
                                deleted_notes += 1
                    
                    # Delete the Collection itself
                    success, error_msg = self.resource_manager.delete_resource(collection_id)
                    if success:
                        logger.info(f"🗑️ Cleared map '{map_name}' ({collection_id}) - deleted {deleted_notes} Notes")
                    else:
                        logger.error(f"Failed to clear map Collection '{map_name}': {error_msg}")
            
            # === Clear SpatialMap file ===
            
            # Import SpatialMap and clear it
            try:
                from pathlib import Path
                
                # Determine base directory
                if hasattr(self.resource_manager, 'base_dir'):
                    base_dir = self.resource_manager.base_dir
                else:
                    # Fallback to scenarios/<world_name>/resources/
                    current_dir = Path(__file__).parent
                    project_root = current_dir.parent
                    world_name = getattr(self, 'world_name', 'minecraft')
                    base_dir = project_root / "scenarios" / world_name / "resources"
                
                # Delete spatial map file directly
                spatial_map_file = base_dir / f"{agent_name}_spatial_map.json"
                if spatial_map_file.exists():
                    spatial_map_file.unlink()
                    logger.info(f"🗑️ Deleted spatial map file: {spatial_map_file}")
                else:
                    logger.info(f"Spatial map file not found: {spatial_map_file}")
                
                # Also clear from mc-map-update cache if it exists
                try:
                    import sys
                    mc_map_update = sys.modules.get('src.world-tools.minecraft.mc-map-update.tool')
                    if mc_map_update and hasattr(mc_map_update, '_spatial_map_cache'):
                        world_name = getattr(self, 'world_name', 'minecraft')
                        cache_key = f"{agent_name}:{world_name}"
                        if cache_key in mc_map_update._spatial_map_cache:
                            mc_map_update._spatial_map_cache[cache_key].clear()
                            del mc_map_update._spatial_map_cache[cache_key]
                            logger.info(f"🗑️ Cleared spatial map cache for {cache_key}")
                except Exception as e:
                    logger.debug(f"Could not clear spatial map cache: {e}")
                    
            except Exception as e:
                logger.warning(f"Failed to clear spatial map file: {e}")
                
        except Exception as e:
            logger.error(f'Error clearing map: {e}')
            traceback.print_exc()
    
    def handle_clear_transients(self, sample):
        """Handle clear transients command - clears all non-persistent resources via query."""
        try:
            # Use existing clear_transient query mechanism
            # Create a mock query object to call the handler
            import zenoh
            from zenoh import Query
            
            # We'll use the queryable directly instead
            # Since we can't easily create a Query object, we'll duplicate the logic
            if not self.resource_manager:
                logger.error("Resource manager not available, cannot clear transients")
                return
            
            PRESERVED_COLLECTIONS = {'conversation', 'conversation_history'}
            
            deleted_notes = 0
            deleted_collections = 0
            to_delete = []
            for resource_id, resource_data in self.resource_manager.resource_registry.items():
                props = resource_data.get('properties', {})
                
                # Skip persistent resources
                if props.get('persistent', False):
                    continue
                
                if resource_id.startswith('Note_') and resource_id != 'Note_null':
                    to_delete.append(resource_id)
                elif resource_id.startswith('Collection_'):
                    collection_name = props.get('collection_name')
                    is_preserved = False
                    if collection_name and collection_name in PRESERVED_COLLECTIONS:
                        is_preserved = True
                    for preserved_name in PRESERVED_COLLECTIONS:
                        if self.resource_manager.named_collections.get(preserved_name) == resource_id:
                            is_preserved = True
                            break
                    if is_preserved:
                        continue
                    to_delete.append(resource_id)
            
            for resource_id in to_delete:
                success, _ = self.resource_manager.delete_resource(resource_id)
                if success:
                    if resource_id.startswith('Note_'):
                        deleted_notes += 1
                    else:
                        deleted_collections += 1
            
            # Also clear planner bindings
            bindings_cleared = 0
            if self.infospace_executor:
                bindings_cleared = len(self.infospace_executor.plan_bindings_flat)
                self.infospace_executor.clear_plan_state()
            
            logger.info(f"🗑️ Cleared transients - {deleted_notes} Notes, {deleted_collections} Collections, {bindings_cleared} bindings")
        except Exception as e:
            logger.error(f'Error clearing transients: {e}')
            traceback.print_exc()
    
    def handle_stop_command(self, sample):
        """Handle stop command - stop autonomous mode and pause execution."""
        try:
            logger.info(f'⏹️ Stop command received by {self.character_name}')
            self.autonomous_mode = False
            self.execution_mode = 'step'
            self.execution_paused = True
            # Note: continuous_mode is NOT cleared here - user must toggle it off explicitly
            self.awaiting_user_input = False  # Clear awaiting flag
            self._publish_execution_state()
        except Exception as e:
            logger.error(f'Error handling stop command: {e}')
            traceback.print_exc()

    def handle_interrupt_command(self, sample):
        """Handle interrupt command - request planner interrupt and pause execution."""
        try:
            logger.warning(f'🛑 Interrupt command received by {self.character_name}')
            self.interrupt_requested = True
            if self.infospace_executor:
                self.infospace_executor.interrupt_requested = True
            # Pause execution immediately; planner will notice interrupt at next step boundary
            self.execution_mode = 'step'
            self.execution_paused = True
            # Note: continuous_mode is NOT cleared here - user must toggle it off explicitly
            self.awaiting_user_input = False
            self._publish_execution_state()
        except Exception as e:
            logger.error(f'Error handling interrupt command: {e}')
            traceback.print_exc()
    
    def _publish_execution_state(self):
        """Publish current execution state for UI."""
        try:
            state_data = {
                'paused': self.execution_paused,
                'mode': self.execution_mode,
                'character': self.character_name,
                'continuous_mode': self.continuous_mode,
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
    
    def _enable_compliance_tracking_callback(self, sample):
        """Handle enabling compliance tracking for evaluation mode."""
        try:
            if hasattr(self, 'infospace_executor'):
                from infospace_compliance import ComplianceTracker
                
                # Create and attach compliance tracker to executor
                self.infospace_executor._compliance_tracker = ComplianceTracker()
                logger.info(f'🧪 {self.character_name} compliance tracking enabled')
            else:
                logger.warning(f'Cannot enable compliance tracking: not in infospace mode')
        except Exception as e:
            logger.error(f'Error enabling compliance tracking: {e}')
    
    
    def _handle_resource_view_query(self, query):
        """Handle query for resource content viewing (for UI)."""
        try:
            if not self.resource_manager:
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
            if not self.resource_manager:
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
            if not self.resource_manager:
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
            if not self.resource_manager:
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
    
    def _handle_resource_clear_transient_query(self, query):
        """Handle query to clear all Notes and Collections except Note_null, persistent resources, and conversation collections.
        
        Optional JSON payload: {"global": true} to also reset world_model and tool_model to empty.
        """
        if not self.resource_manager:
            response = {'success': False, 'error': 'Resource manager not available'}
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
            return
        
        # Parse optional payload
        global_clear = False
        if query.payload:
            try:
                payload = json.loads(query.payload.to_bytes().decode('utf-8'))
                global_clear = payload.get('global', False)
            except (json.JSONDecodeError, UnicodeDecodeError, AttributeError):
                pass  # Default to False if payload parsing fails
        
        # Collections to preserve (by name) - conversation collections should persist across benchmark runs
        PRESERVED_COLLECTIONS = {'conversation', 'conversation_history'}
        
        deleted_notes = 0
        deleted_collections = 0
        to_delete = []
        for resource_id, resource_data in self.resource_manager.resource_registry.items():
            props = resource_data.get('properties', {})
            
            # Skip persistent resources
            if props.get('persistent', False):
                continue
            
            if resource_id.startswith('Note_') and resource_id != 'Note_null':
                to_delete.append(resource_id)
            elif resource_id.startswith('Collection_'):
                # Skip preserved named Collections (conversation and conversation_history)
                # These are special system Collections that should never be deleted by clear_transients
                # Check both by collection_name property and by named_collections registry
                collection_name = props.get('collection_name')
                is_preserved = False
                if collection_name and collection_name in PRESERVED_COLLECTIONS:
                    is_preserved = True
                # Also check if this resource_id is registered as a preserved collection name
                # This provides double protection even if collection_name property is missing
                for preserved_name in PRESERVED_COLLECTIONS:
                    if self.resource_manager.named_collections.get(preserved_name) == resource_id:
                        is_preserved = True
                        break
                if is_preserved:
                    logger.debug(f"Preserving special Collection: {collection_name or resource_id}")
                    continue
                to_delete.append(resource_id)
        
        for resource_id in to_delete:
            success, _ = self.resource_manager.delete_resource(resource_id)
            if success:
                if resource_id.startswith('Note_'):
                    deleted_notes += 1
                else:
                    deleted_collections += 1
        
        # Also clear planner bindings
        bindings_cleared = 0
        if self.infospace_executor:
            bindings_cleared = len(self.infospace_executor.plan_bindings_flat)
            self.infospace_executor.clear_plan_state()
        
        # If global=True, reset world_model and tool_model to empty
        world_model_cleared = False
        tool_model_cleared = False
        if global_clear:
            if hasattr(self, 'world_model') and self.world_model:
                from world_model import empty_world_model_raw
                self.world_model.world_model = empty_world_model_raw()
                self.world_model.save()
                world_model_cleared = True
                logger.info("🌍 Cleared world_model (reset to empty)")
            
            if hasattr(self, 'tool_model') and self.tool_model:
                from tool_model import empty_tool_model
                self.tool_model.tool_model = empty_tool_model()
                self.tool_model.save()
                tool_model_cleared = True
                logger.info("🔧 Cleared tool_model (reset to empty)")
        
        response = {
            'success': True,
            'deleted_notes': deleted_notes,
            'deleted_collections': deleted_collections,
            'bindings_cleared': bindings_cleared,
            'world_model_cleared': world_model_cleared,
            'tool_model_cleared': tool_model_cleared
        }
        query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
        logger.info(f"Cleared {deleted_notes} Notes, {deleted_collections} Collections, {bindings_cleared} planner bindings" + 
                   (f", world_model, tool_model" if global_clear else ""))
    
    def _handle_resource_reset_models_query(self, query):
        """Handle query to explicitly reset world_model and tool_model to empty."""
        world_model_cleared = False
        tool_model_cleared = False
        exec_state_cleared = False
        resources_file_cleared = False
        
        if hasattr(self, 'world_model') and self.world_model:
            from world_model import empty_world_model_raw
            self.world_model.world_model = empty_world_model_raw()
            self.world_model.save()
            world_model_cleared = True
            logger.info("🌍 Explicitly cleared world_model (reset to empty)")
        
        if hasattr(self, 'tool_model') and self.tool_model:
            from tool_model import empty_tool_model
            self.tool_model.tool_model = empty_tool_model()
            self.tool_model.save()
            tool_model_cleared = True
            logger.info("🔧 Explicitly cleared tool_model (reset to empty)")

        # Also clear executive-local "last action" state (benchmark hygiene)
        if hasattr(self, 'action_history'):
            self.action_history = []
        if hasattr(self, 'last_say_text'):
            self.last_say_text = ''
        if hasattr(self, 'last_out_resource_id'):
            self.last_out_resource_id = None
        exec_state_cleared = True
        
        # Clear resources.json by resetting registry to minimal state (Note_null only) and saving
        # This ensures no cross-benchmark carryover from persistent resources
        if hasattr(self, 'resource_manager') and self.resource_manager:
            # Keep only Note_null (system resource)
            note_null_id = None
            for resource_id in list(self.resource_manager.resource_registry.keys()):
                if resource_id == 'Note_null':
                    note_null_id = resource_id
                else:
                    # Delete all other resources (including persistent ones for benchmark hygiene)
                    self.resource_manager.delete_resource(resource_id)
            
            # Reset counters (Note_null counts as 1 if it exists)
            if note_null_id:
                self.resource_manager.note_counter = 1
            else:
                self.resource_manager.note_counter = 0
                # Recreate Note_null if missing
                self.resource_manager._create_note_null()
                self.resource_manager.note_counter = 1
            
            self.resource_manager.collection_counter = 0
            
            # Save minimal state to resources.json
            if self.resource_manager.save_to_file():
                resources_file_cleared = True
                logger.info(f"📦 Cleared resources.json (reset to minimal state: Note_null only, saved to {self.resource_manager.resources_file})")
            
        response = {
            'success': True,
            'world_model_cleared': world_model_cleared,
            'tool_model_cleared': tool_model_cleared,
            'exec_state_cleared': exec_state_cleared,
            'resources_file_cleared': resources_file_cleared
        }
        query.reply(query.key_expr, json.dumps(response).encode('utf-8'))

    def _handle_resource_create_note_query(self, query):
        """Handle query to create a Note from external caller."""
        if not self.resource_manager:
            response = {'success': False, 'error': 'Resource manager not available'}
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
            return
        
        payload_bytes = query.payload.to_bytes() if query.payload else b'{}'
        params = json.loads(payload_bytes.decode('utf-8')) if payload_bytes else {}
        
        content = params.get('content', '')
        format_type = params.get('format', 'text')
        note_name = params.get('name', '')
        
        success, note_id, error_msg, _ = self.resource_manager.create_note(
            character_name=self.character_name,
            content=content,
            format_type=format_type,
            source_skill='external',
            source_value='',
            note_name=note_name,
            extra_props={}
        )
        
        if success:
            response = {'success': True, 'note_id': note_id}
        else:
            response = {'success': False, 'error': error_msg or 'Failed to create Note'}
        query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
    
    def _handle_llm_generate_query(self, query):
        """Handle LLM generation query from external caller."""
        payload_bytes = query.payload.to_bytes() if query.payload else b'{}'
        params = json.loads(payload_bytes.decode('utf-8')) if payload_bytes else {}
        
        messages = params.get('messages', [])
        max_tokens = params.get('max_tokens', 2000)
        temperature = params.get('temperature', 0.7)
        is_json = params.get('is_json', False)
        stops = params.get('stops')
        
        if not messages:
            response = {'success': False, 'error': 'messages required'}
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
            return
        
        result = self.llm_generate(messages, max_tokens=max_tokens, temperature=temperature, is_json=is_json, stops=stops)
        
        if result.success:
            response = {'success': True, 'text': result.text}
        else:
            response = {'success': False, 'error': getattr(result, 'error', 'Generation failed')}
        query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
    
    def _plan_bindings_query_handler(self, query):
        """Handle query for current plan bindings."""
        try:
            bindings_data = {}
            # Get bindings from infospace_executor (where incremental planner stores them)
            bindings = {}
            if self.infospace_executor and hasattr(self.infospace_executor, 'plan_bindings_flat'):
                bindings = self.infospace_executor.plan_bindings_flat
            
            for var_name, value in bindings.items():
                if isinstance(value, str):
                    if value.startswith('Note_') or value.startswith('Collection_'):
                        bindings_data[var_name] = {'type': 'resource_id', 'value': value}
                    else:
                        bindings_data[var_name] = {'type': 'string', 'value': value[:100] if len(value) > 100 else value}
                elif isinstance(value, dict):
                    bindings_data[var_name] = {'type': 'dict', 'keys': list(value.keys())}
                elif isinstance(value, list):
                    bindings_data[var_name] = {'type': 'list', 'length': len(value)}
                else:
                    bindings_data[var_name] = {'type': type(value).__name__, 'value': str(value)[:100]}
            
            response = {'success': True, 'bindings': bindings_data}
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
        except Exception as e:
            logger.error(f'Error in plan_bindings query handler: {e}')
            response = {'success': False, 'error': str(e)}
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
    
    def _world_state_query_handler(self, query):
        """Handle query for current world state."""
        try:
            world_state = {}
            if self.infospace_executor and hasattr(self.infospace_executor, 'world_state'):
                world_state = self.infospace_executor.world_state
            
            response = {'success': True, 'world_state': world_state}
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
        except Exception as e:
            logger.error(f'Error in world_state query handler: {e}')
            response = {'success': False, 'error': str(e)}
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
    
    def _clear_planner_bindings_handler(self, query):
        """
        Handle query for explicit clearing of planner bindings.
        
        Returns:
            success, bindings_cleared (count)
        """
        try:
            if not self.infospace_executor:
                response = {
                    'success': False,
                    'error': 'Clear bindings only available for infospace characters'
                }
                query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
                return
            
            # Get count before clearing
            bindings_count = len(self.infospace_executor.plan_bindings_flat)
            
            # Clear plan bindings
            self.infospace_executor.clear_plan_state()
            
            logger.info(f'🔄 {self.character_name} cleared {bindings_count} planner bindings via API')
            
            # Return success
            response = {
                'success': True,
                'bindings_cleared': bindings_count
            }
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
            
        except Exception as e:
            logger.error(f'Error in clear_planner_bindings handler: {e}')
            traceback.print_exc()
            response = {'success': False, 'error': str(e)}
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
    
    def _planner_feedback_handler(self, query):
        """
        Handle query for planner feedback.
        
        Accepts:
            {"outcome": true/false}
        
        Returns:
            {"success": true/false, "error": "..." if failed}
        """
        try:
            if not self.incremental_planner:
                response = {
                    'success': False,
                    'error': 'IncrementalPlanner not available'
                }
                query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
                return
            
            payload_bytes = query.payload.to_bytes() if query.payload else b'{}'
            params = json.loads(payload_bytes.decode('utf-8')) if payload_bytes else {}
            
            outcome = params.get('outcome')
            if outcome is None:
                response = {'success': False, 'error': 'outcome field required'}
                query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
                return
            
            # Convert to bool if needed (handle string "true"/"false")
            if isinstance(outcome, str):
                outcome = outcome.lower() in ('true', '1', 'yes', 'on')
            outcome = bool(outcome)
            
            # Call feedback method
            result = self.incremental_planner._feedback(outcome)
            
            logger.info(f'📝 {self.character_name} received planner feedback: outcome={outcome}')
            
            query.reply(query.key_expr, json.dumps(result).encode('utf-8'))
            
        except Exception as e:
            logger.error(f'Error in planner_feedback handler: {e}')
            traceback.print_exc()
            response = {'success': False, 'error': str(e)}
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
    
    def _sync_plan_execution_handler(self, query):
        """
        Handle query for synchronous plan execution.
        
        Accepts saved_plan format:
            {"plan": {"plan": [...], "response": "..."}, "goal": "optional"}
        Or simple format:
            {"plan": [...], "max_steps": 1000}
        
        Returns:
            success, status, reason, executed_steps, bindings, last_result
        """
        try:
            if not self.infospace_executor:
                response = {
                    'success': False,
                    'error': 'Sync plan execution only available for infospace characters'
                }
                query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
                return
            
            # Parse plan from query payload
            payload = query.payload.to_bytes().decode('utf-8')
            request_data = json.loads(payload)
            plan_data = request_data.get('plan')
            max_steps = request_data.get('max_steps', 1000)
            
            if not plan_data:
                response = {
                    'success': False,
                    'error': 'Plan is required'
                }
                query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
                return
            
            # Support saved_plan format: {"plan": {"plan": [...]}} or simple: {"plan": [...]}
            if isinstance(plan_data, dict) and 'plan' in plan_data:
                plan_steps = plan_data['plan']
            elif isinstance(plan_data, list):
                plan_steps = plan_data
            else:
                response = {
                    'success': False,
                    'error': 'Plan must be a list of actions or {"plan": [...]}'
                }
                query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
                return
            
            # Validate each action has 'type'
            for i, action in enumerate(plan_steps):
                if not isinstance(action, dict) or 'type' not in action:
                    response = {
                        'success': False,
                        'error': f'Action {i} missing "type" field'
                    }
                    query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
                    return
            
            # Execute plan synchronously
            result = self.infospace_executor.execute_plan_sync(plan_data, max_steps=max_steps)
            
            # Get last action result in uniform format (already provided by execute_plan_sync)
            last_action_result = result.get('last_action_result')
            
            # Add action field to last_action_result for external API consumers
            if last_action_result and isinstance(last_action_result, dict):
                # Get action type from last action in history
                if self.action_history:
                    last_action = self.action_history[-1].action
                    if isinstance(last_action, dict):
                        last_action_result = last_action_result.copy()
                        last_action_result['action'] = last_action.get('type', 'unknown')
            
            # Return result
            response = {
                'success': result.get('status') == 'success',
                'status': result.get('status'),
                'reason': result.get('reason'),
                'executed_steps': result.get('executed_steps', 0),
                'bindings': result.get('bindings', {}),
                'last_action_result': last_action_result,  # Uniform format: {status, value, resource_id, reason, action}
                'suspended': result.get('status') == 'suspended',
                'suspension_reason': result.get('reason') if result.get('status') == 'suspended' else None
            }
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
            
        except json.JSONDecodeError as e:
            logger.error(f'Invalid JSON in plan execution request: {e}')
            response = {
                'success': False,
                'error': f'Invalid JSON: {str(e)}'
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
    

    def parse_and_set_goal(self, template, goal_text):
        """Parse goal input from UI and set current goal."""
        try:
            parsed_goal = goal_text.strip().strip('"').strip("'").replace('goal:', '').strip()
            
            # Immediately clear existing plan to interrupt execution
            self.current_plan = None
            # Note: plan_bindings are NOT cleared here - they persist across plans unless explicitly cleared
            self.plan_bindings_cache = {}
            self.goal_source = 'ui'
            self.awaiting_user_input = False
            logger.info(f'🛑 {self.character_name} interrupting existing plan for new goal')
            
            if not self.observations:
                self._observe()
            self.current_goal = Goal(parsed_goal, [self.character_name], description='', termination='')
            
            # If continuous mode is active, update goal text for resubmission
            if self.continuous_mode:
                self.continuous_goal_text = self.current_goal.to_string()
                self.last_completed_goal_text = self.current_goal.to_string()  # Also update last completed
                logger.info(f'🔄 {self.character_name} continuous mode goal updated: {self.continuous_goal_text[:80]}...')
            
            # Skip goal rewriting and plan immediately (infospace mode)
            self._publish_goal(self.current_goal)
            logger.info(f'🧩 {self.character_name} infospace planning for goal: {parsed_goal}')
            result = self._plan(template, self.current_goal)
            # Publish complete goal result for external consumers (e.g., eval scripts)
            self._publish_goal_result(result)
            if result.get('success', False):
                self.current_plan = result.get('plan', {})
                self._publish_current_plan()
                self.plan_just_generated = True
                if result.get('response', None):
                    self.infospace_executor.execute_action({"type": "say", "target": "user", "value": result.get('response')})
            else:
                logger.error(f"Error in _plan: {result.get('error', 'Unknown error')}")
                traceback.print_exc()
                return
            if self.current_plan:
                self._publish_current_plan()
                self.plan_just_generated = True
            return
            
        except Exception as e:
            logger.error(f"Goal parsing failed for {self.character_name}: {e}")
            traceback.print_exc()
            return

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
            # Legacy inspections/uses caches removed (not used in infospace)
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
    
    def shutdown(self):
        """Clean shutdown."""
        try:
            if getattr(self, '_shutting_down', False):
                return
            self._shutting_down = True
            
            logger.info(f'Executive Node shutdown initiated for {self.character_name}...')
            
            # Archive conversation if it exists and is not empty (do this first before saving state)
            try:
                self._archive_conversation()
            except Exception as e:
                logger.error(f'Error archiving conversation during shutdown: {e}')
                import traceback
                traceback.print_exc()
            
            # Save memory before shutdown
            try:
                self.memory.save()
                logger.info(f'💾 Saved memory state for {self.character_name}')
            except Exception as e:
                logger.error(f'Error saving memory during shutdown: {e}')
            
            # Save resource manager
            if self.resource_manager:
                try:
                    self.resource_manager.save_to_file()
                    logger.info(f'💾 Saved resource manager state for {self.map_name}')
                except Exception as e:
                    logger.error(f'Error saving resource manager during shutdown: {e}')
            
            # Stop API server if running
            if hasattr(self, 'api_server') and self.api_server:
                try:
                    self.api_server.stop()
                    logger.info('🌐 API server stopped')
                except Exception as e:
                    logger.warning(f'Error stopping API server: {e}')
            
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
    """Main entry point."""
    parser = argparse.ArgumentParser(description='Zenoh Executive Node')
    parser.add_argument('-c', '--character', type=str, default='default', help='Character name')
    parser.add_argument('-config', type=str, default='{}', help='Character configuration as JSON string')
    args = parser.parse_args()
    
    try:
        character_config = json.loads(args.config) if args.config else {}
    except json.JSONDecodeError as e:
        logger.error(f'Failed to parse character config JSON: {e}')
        sys.exit(1)
    
    try:
        executive_node = ZenohExecutiveNode(character_name=args.character, character_config=character_config)
        executive_node.run()
    except KeyboardInterrupt:
        logger.info('Executive Node interrupted by user')
    except Exception as e:
        logger.error(f'Executive Node error: {e}')
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
