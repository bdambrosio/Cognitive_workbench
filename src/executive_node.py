#!/usr/bin/env python3
"""
Zenoh Executive Node

This node implements the OODA loop for character decision-making and action execution.

"""

import math
import random
import re
import traceback
import json
import time
import threading
import logging
import sys
import signal
import argparse
import os
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Any, Union, Optional
from dataclasses import dataclass, asdict
from weakref import WeakValueDictionary

# Ensure src/ is on sys.path before local imports (needed for multiprocessing spawn)
_src_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _src_dir)
# Fallback: spawn child may not inherit PYTHONPATH (e.g. under debugger)
for _p in os.environ.get("PYTHONPATH", "").split(os.pathsep):
    if _p and _p not in sys.path:
        sys.path.insert(0, _p)

import zenoh
from zenoh import QueryTarget, ConsolidationMode
from Messages import SystemMessage, UserMessage
from utils.zenoh_utils import datetime_handler
from templates import DRIVE_ASSESSMENT_TEMPLATE, GOAL_TEMPLATE, PLAN_TEMPLATE, PLAN_VERBS, REWRITE_TEMPLATE
from utils.format_utils import format_map_types, format_views_compact
from utils.condition_utils import deref_plan_target
import utils.hash_utils as hash_utils
from transformers import AutoTokenizer
import requests
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
    
    def __init__(self, character_name="default", character_config=None, runtime=None, tokenizer=None):
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
        
        # LLM backend - Use SGLang.Runtime for infospace, vLLM or OpenRouter as alternatives, ZenohLLMClient as fallback
        llm_config = self.character_config.get('llm_config', {})
        server_name = llm_config.get('server_name', 'openai')
        model_name = llm_config.get('model_name', 'gpt-4.1')
        sgl_model_path = llm_config.get('sgl_model_path')
        vllm_model_path = llm_config.get('vllm_model_path')
        vllm_url = llm_config.get('vllm_url', 'http://localhost:5000/v1/chat/completions')
        openrouter_model_path = llm_config.get('openrouter_model_path')
        anthropic_model_path = llm_config.get('anthropic_model_path')
        
        self.runtime = None
        self.llm_client = None
        self.vllm_model = None
        self.vllm_url = None
        self.openrouter_model = None
        self.openrouter_api_key = None
        self.anthropic_model = None
        self.anthropic_api_key = None
        
        # Check SGLang availability
        HAS_SGLANG = False
        try:
            from incremental_planner import HAS_SGLANG as _HAS_SGLANG
            HAS_SGLANG = _HAS_SGLANG
        except ImportError:
            pass
        
        # Initialize Anthropic API if configured
        if anthropic_model_path:
            api_key = os.getenv('CLAUDE_API_KEY')
            if not api_key:
                logger.error("❌ Anthropic model configured but CLAUDE_API_KEY environment variable not set")
                raise ValueError("CLAUDE_API_KEY environment variable required for Anthropic API")
            self.anthropic_model = anthropic_model_path
            self.anthropic_api_key = api_key
            logger.info(f"✅ Anthropic API configured with model: {anthropic_model_path}")
        # Initialize OpenRouter if configured
        elif openrouter_model_path:
            api_key = os.getenv('OPENROUTER_API_KEY')
            if not api_key:
                logger.error("❌ OpenRouter model configured but OPENROUTER_API_KEY environment variable not set")
                raise ValueError("OPENROUTER_API_KEY environment variable required for OpenRouter")
            self.openrouter_model = openrouter_model_path
            self.openrouter_api_key = api_key
            logger.info(f"✅ OpenRouter configured with model: {openrouter_model_path}")
        
        # Initialize vLLM model resolution if configured (only if OpenRouter not configured)
        elif vllm_model_path:
            try:
                import requests
                logger.info(f"🔍 Querying vLLM server for available models...")
                response = requests.get('http://localhost:5000/v1/models', timeout=10)
                response.raise_for_status()  # Fail fast
                data = response.json()
                available_models = []
                if data.get('data') and len(data['data']) > 0:
                    available_models = [model['id'] for model in data['data']]
                    logger.info(f"📋 Available vLLM models: {available_models}")
                    
                    # Check if configured model is in available models
                    if vllm_model_path in available_models:
                        self.vllm_model = vllm_model_path
                        logger.info(f"✅ Using configured vLLM model: {vllm_model_path}")
                    else:
                        # Use first available model with warning
                        self.vllm_model = available_models[0]
                        logger.warning(f"⚠️  Configured vLLM model '{vllm_model_path}' not found in available models. Using first available: {self.vllm_model}")
                else:
                    raise ValueError("No models found in vLLM response")
                self.vllm_url = vllm_url
            except Exception as e:
                logger.error(f"❌ Failed to query vLLM server: {e}")
                raise  # Fail fast
        
        # Initialize SGLang.Runtime - accept externally-provided runtime (shared across agents)
        # or create a new one if running standalone
        if runtime is not None:
            # Runtime provided by launcher (shared across agent threads)
            self.runtime = runtime
            self.tokenizer = tokenizer if tokenizer is not None else (AutoTokenizer.from_pretrained(sgl_model_path) if sgl_model_path else None)
            self.api_server = None  # API server managed by launcher
            logger.info(f'🤖 Using shared SGLang Runtime for {self.character_name}')
        elif HAS_SGLANG and sgl_model_path:
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
        
        # Subscriber for save_all command (save resource_manager state)
        self.save_subscriber = self.session.declare_subscriber(
            "cognitive/save_all",
            self._handle_save_command
        )
        logger.info(f'💾 Subscribed to cognitive/save_all')

        # Subscriber for close_dialog (UI "End conversation" button)
        self.close_dialog_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/memory/close_dialog",
            self._handle_close_dialog
        )
        logger.info(f'🔚 Subscribed to cognitive/{character_name}/memory/close_dialog')
        
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
        self.resource_manager = InfospaceResourceManager(self.map_name, session=self.session, world_config=world_config, agent_name=self.character_name)
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
        # Share runtime, vLLM, and OpenRouter config with executor
        if self.runtime:
            self.infospace_executor.runtime = self.runtime
            self.infospace_executor.tokenizer = self.tokenizer
        if self.vllm_model and self.vllm_url:
            self.infospace_executor.vllm_model = self.vllm_model
            self.infospace_executor.vllm_url = self.vllm_url
        if self.openrouter_model and self.openrouter_api_key:
            self.infospace_executor.openrouter_model = self.openrouter_model
            self.infospace_executor.openrouter_api_key = self.openrouter_api_key
        if self.anthropic_model and self.anthropic_api_key:
            self.infospace_executor.anthropic_model = self.anthropic_model
            self.infospace_executor.anthropic_api_key = self.anthropic_api_key
        logger.info(f'🧩 Infospace executor initialized for {character_name}')
        
        # Build available_tools for models: loaded tools + infospace primitives (load, persist, etc.)
        from infospace_executor import INFOSPACE_PRIMITIVES
        available_for_models = dict(self.available_tools)
        for p in INFOSPACE_PRIMITIVES:
            if p not in available_for_models:
                available_for_models[p] = {"name": p, "description": "Infospace primitive", "type": "primitive"}
        # Create WorldModel instance (with executor)
        from world_model import WorldModel
        self.world_model = WorldModel(
            world_name=world_name,
            agent_name=character_name,
            resource_manager=self.resource_manager,
            executor=self.infospace_executor,
            available_tools=available_for_models
        )
        logger.info(f'🌍 WorldModel initialized for {character_name} in {world_name}')
        
        # Create ToolModel instance (with executor)
        from tool_model import ToolModel
        self.tool_model = ToolModel(
            world_name=world_name,
            agent_name=character_name,
            resource_manager=self.resource_manager,
            executor=self.infospace_executor,
            available_tools=available_for_models
        )
        self.tool_model.build_task_tool_index()
        logger.info(f'🔧 ToolModel initialized for {character_name} in {world_name}')
        
        # Attach models to executor for access by planner
        self.infospace_executor.world_model = self.world_model
        self.infospace_executor.tool_model = self.tool_model
        
        # World initialization removed - worlds are assumed to run as separate servers
        
        # Initialize conversation collections
        self._initialize_conversation_collections()
        
        # Load living context (situation note) — must follow executor + resource manager init
        self.situation_context = ""
        self._load_situation_note()
        from conversation_store import ConversationStore
        self.conversation_store = ConversationStore(
            resource_manager=self.resource_manager,
            character_name=self.character_name,
            logger=logger
        )
        self.conversation_store.initialize()
        
        # Initialize task manager
        from task_manager import TaskManager
        self.task_manager = TaskManager(
            resource_manager=self.resource_manager,
            executor=self.infospace_executor,
            character_name=self.character_name
        )
        if not self.benchmark_mode:
            self.task_manager.initialize()

        # Initialize task scheduler (auto-proceed timer)
        from task_scheduler import TaskScheduler
        sched_cfg = self.character_config.get('task_scheduler', {})
        self.task_scheduler = TaskScheduler(
            character_name=self.character_name,
            interval=sched_cfg.get('interval', 15.0),
            enabled=sched_cfg.get('enabled', False),
        )
        self.task_scheduler.start(
            check_fn=self._scheduler_eligible_tasks,
            proceed_fn=self._scheduler_proceed_task,
        )

        # Initialize planners (reused across all goals)
 
        # IncrementalPlanner for plan generation (SGLang or vLLM-based)
        self.incremental_planner = None
        try:
            from incremental_planner import IncrementalPlanner, HAS_SGLANG
            
            # Get LLM config
            llm_config = self.character_config.get('llm_config', {})
            sgl_model_path = llm_config.get('sgl_model_path')
            vllm_model_path = llm_config.get('vllm_model_path')
            vllm_url = llm_config.get('vllm_url', 'http://localhost:5000/v1/chat/completions')
            openrouter_model_path = llm_config.get('openrouter_model_path')
            anthropic_model_path = llm_config.get('anthropic_model_path')
            
            if HAS_SGLANG and sgl_model_path:
                # Use SGLang
                self.incremental_planner = IncrementalPlanner(
                    executor=self.infospace_executor,
                    available_tools=self.available_tools,
                    logger_instance=logger,
                    sgl_model_path=sgl_model_path
                )
                logger.info(f'🚀 IncrementalPlanner initialized (SGLang) for {character_name}')
            elif vllm_model_path and self.vllm_model:
                # Use vLLM
                self.incremental_planner = IncrementalPlanner(
                    executor=self.infospace_executor,
                    available_tools=self.available_tools,
                    logger_instance=logger,
                    vllm_model_path=vllm_model_path,
                    vllm_url=vllm_url,
                    vllm_model=self.vllm_model
                )
                logger.info(f'🚀 IncrementalPlanner initialized (vLLM) for {character_name}')
            elif anthropic_model_path and self.anthropic_model:
                # Use Anthropic API
                self.incremental_planner = IncrementalPlanner(
                    executor=self.infospace_executor,
                    available_tools=self.available_tools,
                    logger_instance=logger,
                    anthropic_model_path=anthropic_model_path
                )
                logger.info(f'🚀 IncrementalPlanner initialized (Anthropic API) for {character_name}')
            elif openrouter_model_path and self.openrouter_model:
                # Use OpenRouter
                self.incremental_planner = IncrementalPlanner(
                    executor=self.infospace_executor,
                    available_tools=self.available_tools,
                    logger_instance=logger,
                    openrouter_model_path=openrouter_model_path
                )
                logger.info(f'🚀 IncrementalPlanner initialized (OpenRouter) for {character_name}')
            else:
                logger.warning(f'⚠️  Neither SGLang, vLLM, Anthropic, nor OpenRouter configured - incremental planning disabled')
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
        self._last_agent_message = {}  # {source: message} for dedup of agent-to-agent messages
        self._agent_conversation_turns = 0  # Count of agent-to-agent exchanges
        self._dialog_purposes = {}  # {agent_name: original_purpose} for topic anchoring
        self._dialog_cooldowns = {}  # {agent_name: timestamp} suppress re-open after close
        
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
        self.control_clear_persistents_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/control/clear_persistents",
            self.handle_clear_persistents
        )
        self.control_scheduler_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/control/task_scheduler",
            self._handle_scheduler_control
        )
        self.control_task_schedule_mode_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/control/task_schedule_mode",
            self._handle_task_schedule_mode
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
        
        self.tasks_queryable = self.session.declare_queryable(
            f"cognitive/{character_name}/tasks",
            self._tasks_query_handler
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



        # Register signal handlers for graceful shutdown (only works in main thread)
        import threading as _threading
        if _threading.current_thread() is _threading.main_thread():
            signal.signal(signal.SIGTERM, self._signal_handler)
            signal.signal(signal.SIGINT, self._signal_handler)

    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully."""
        logger.warning(f'Received signal {signum}, initiating shutdown...')
        self.shutdown_requested = True
    
    def _handle_save_command(self, sample):
        """Handle save_all command - save resource_manager state."""
        try:
            logger.info(f'💾 {self.character_name} received save_all command')
            if self.resource_manager:
                self.resource_manager.save_to_file()
                logger.info(f'💾 Saved resource manager state for {self.map_name}')
        except Exception as e:
            logger.error(f'Error handling save command: {e}')

    def _handle_close_dialog(self, sample):
        """Handle close_dialog from UI (End conversation button)."""
        try:
            payload_bytes = sample.payload.to_bytes()
            data = json.loads(payload_bytes.decode('utf-8'))
            entity_name = data.get('entity_name', 'User')
            self.conversation_store.close_dialog(entity_name)
            self._dialog_cooldowns[entity_name] = time.time()
            self._dialog_purposes.pop(entity_name, None)
            # End-dialog should immediately terminate any in-progress ask/wait.
            self.interrupt_requested = True
            if self.infospace_executor:
                self.infospace_executor.interrupt_requested = True
            self.awaiting_ask_response = False
            logger.info(f'🔚 {self.character_name} closed dialog with {entity_name} (from UI)')
            self._publish_execution_state()
        except Exception as e:
            logger.warning(f'Error handling close_dialog: {e}')
    
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
                        # Unpause for explicit goals, User text, or messages from other agents
                        if text.strip().startswith('goal:') or source == 'User' or source not in ('unknown', 'console'):
                            logger.info(f'🚀 Goal/User input in queue, unpausing execution')
                            self.execution_paused = False
                            self._publish_execution_state()
                            break
                
                if self.execution_paused:
                    time.sleep(0.2)
                else:
                    try:
                        # Priority order: Text input → Normal OODA
                        # Skip if ask primitive is polling text_input_queue directly
                        if self.text_input_queue and not self.awaiting_ask_response:
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
                                     'split', 'flatten', 'map', 'synthesize', 'search-web', 'semantic-scholar']
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
                        # Truncate result for UI display (same as logging truncation)
                        if isinstance(result_value, (dict, list)):
                            result_str = json.dumps(result_value)
                        else:
                            result_str = str(result_value)
                        if len(result_str) > 128:
                            result_str = result_str[:125] + "..."
                        action_data['result'] = result_str
                    if resource_id:
                        action_data['resource_id'] = resource_id
                    action_data['target'] = target  # Keep original target (variable name)
                    action_data['resolved_target'] = display_target  # Show resolved value if different
                    logger.debug(f'Published action result: {normalized_type} -> value={result_value}, resource_id={resource_id}, target={target}')
                else:
                    action_data['target'] = target
                    action_data['resolved_target'] = display_target if display_target != target else None
                    # Truncate value for UI display (same as logging truncation)
                    value_str = str(value) if value else ''
                    if len(value_str) > 128:
                        value_str = value_str[:125] + "..."
                    action_data['value'] = value_str
                    action_data['error'] = result.get('reason', 'Failed')
            else:
                # Generic action format - serialize objects to prevent [object Object] in UI
                # For action, extract key fields or serialize if it's a complex object
                if isinstance(action, dict):
                    # Extract key fields for display
                    action_data['target'] = action.get('target', '')
                    # Truncate value for UI display (same as logging truncation)
                    value = action.get('value', '')
                    value_str = str(value) if value else ''
                    if len(value_str) > 128:
                        value_str = value_str[:125] + "..."
                    action_data['value'] = value_str
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
                            result_str = json.dumps(result_value)
                        else:
                            result_str = str(result_value)
                        # Truncate result for UI display (same as logging truncation)
                        if len(result_str) > 128:
                            result_str = result_str[:125] + "..."
                        action_data['result'] = result_str
                    else:
                        # Serialize the whole result dict if it's complex
                        result_str = json.dumps(result) if len(result) > 2 else str(result)
                        # Truncate result for UI display (same as logging truncation)
                        if len(result_str) > 128:
                            result_str = result_str[:125] + "..."
                        action_data['result'] = result_str
                else:
                    result_str = json.dumps(result) if not isinstance(result, str) else str(result)
                    # Truncate result for UI display (same as logging truncation)
                    if len(result_str) > 128:
                        result_str = result_str[:125] + "..."
                    action_data['result'] = result_str
            
            # Truncate action payload for UI display (mc-map-update can embed huge observations)
            try:
                action_field = action_data.get("action")
                if action_field is not None:
                    if isinstance(action_field, (dict, list)):
                        action_str = json.dumps(action_field)
                    else:
                        action_str = str(action_field)
                    if len(action_str) > 128:
                        action_str = action_str[:125] + "..."
                    action_data["action"] = action_str
            except Exception:
                pass

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
            'quality_status': self.current_plan.get('quality_status', 'unknown'),
            'verification_answer': self.current_plan.get('verification_answer', ''),
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
        
        interrupted_final = final_thoughts_clean == "Interrupted by user."

        # Save fallback response to conversation only when no explicit say action occurred.
        if (not last_say) and (not interrupted_final) and final_thoughts_clean and len(final_thoughts_clean) >= 20 and final_thoughts_clean.replace(' ', '').replace('.', '').replace(',', '').replace('!', '').replace('?', '').strip():
            # Fall back to FINAL_ANSWER if no actual say action
            logger.debug(f'Using FINAL_ANSWER for conversation (no say action found): {final_thoughts_clean[:50]}...')
            self.conversation_store.record_outgoing("User", final_thoughts_clean, act_type="response")
        
        # Publish final_answer as a "say" action to User so it appears in action log
        # Only publish if final_thoughts is non-empty, meaningful, and different from last_say_text
        # (to avoid duplicating the actual say action that was already published)
        # Require meaningful content: at least 20 chars and not just whitespace/punctuation
        if (not interrupted_final) and final_thoughts_clean and len(final_thoughts_clean) >= 20 and final_thoughts_clean.replace(' ', '').replace('.', '').replace(',', '').replace('!', '').replace('?', '').strip():
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

        # Update living context (situation note) after non-trivial goal completion
        if not interrupted_final and self.current_goal and plan_result.get('status') in ('complete', 'failed'):
            try:
                self._update_situation_note(self.current_goal.name, plan_result)
            except Exception as e:
                logger.warning(f'Error in post-goal situation note update: {e}')

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

    def _load_situation_note(self):
        """Load the living context (_situation) note at startup."""
        if self.benchmark_mode:
            logger.info('Benchmark mode: skipping situation note load')
            return
        if not self.infospace_executor:
            logger.warning('Infospace executor not available, skipping situation note load')
            return
        try:
            result = self.infospace_executor.execute_action({"type": "load", "target": "_situation", "out": "$_situation"})
            if result.get('status') == 'success' and result.get('resource_id'):
                content = self.infospace_executor._get_content(result['resource_id'])
                if content and isinstance(content, str) and content.strip():
                    self.situation_context = content.strip()
                    logger.info(f'✓ Loaded situation note ({len(self.situation_context)} chars)')
                else:
                    logger.info('Situation note exists but is empty')
            else:
                logger.info('No situation note found (first run or not yet created)')
        except Exception as e:
            logger.warning(f'Error loading situation note: {e}')

    def _build_situation_context(self) -> str:
        """Build situation context from task state + persisted situation note."""
        parts = []
        if hasattr(self, 'task_manager'):
            task_summary = self.task_manager.situation_summary()
            if task_summary:
                parts.append(task_summary)
        if self.situation_context:
            parts.append(self.situation_context)
        return "\n\n".join(parts)

    def _update_situation_note(self, goal_text: str, plan_result: dict):
        """Update the living context (_situation) note after goal completion."""
        if self.benchmark_mode or not self.infospace_executor:
            return
        try:
            current = self.situation_context or "(No prior situation note — this is the first entry.)"
            status = plan_result.get('status', 'unknown')
            summary = (plan_result.get('final_thoughts') or '')[:500]

            char_desc = (self.character_config.get('character') or self.character_name)[:200]
            prompt_system = (
                f"You maintain a concise living-context note for {self.character_name}. "
                f"Character: {char_desc}\n"
                "The note tracks: active projects and their states, what is blocked, recent decisions, "
                "and what to do next. Keep it under 600 words. Drop anything no longer relevant. "
                "Output ONLY the updated note text, no preamble."
            )
            prompt_user = (
                f"## CURRENT SITUATION NOTE\n{current}\n\n"
                f"## JUST COMPLETED\nGoal: {goal_text}\nOutcome: {status}\nSummary: {summary}\n\n"
                "Write the updated situation note."
            )

            response = self.infospace_executor.llm_generate(
                [prompt_system, prompt_user], max_tokens=1200, temperature=0.3
            )
            if not response.success or not response.text or not response.text.strip():
                logger.warning(f'Situation note update LLM call failed: {getattr(response, "error", "empty")}')
                return

            new_content = response.text.strip()

            # Backup current note to _situation_prev before overwriting
            self._write_named_note('_situation_prev', self.situation_context or '')

            # Write updated note
            self._write_named_note('_situation', new_content)
            self.situation_context = new_content
            logger.info(f'✓ Updated situation note ({len(new_content)} chars)')
        except Exception as e:
            logger.warning(f'Error updating situation note: {e}')

    def _consolidate_situation_note(self):
        """Final situation note consolidation at shutdown."""
        if self.benchmark_mode or not self.infospace_executor:
            return
        if self.plan_counter == 0 and not self.situation_context:
            logger.info('No goals executed and no situation note — skipping consolidation')
            return
        try:
            current = self._build_situation_context() or "(empty)"
            char_desc = (self.character_config.get('character') or self.character_name)[:200]
            prompt_system = (
                f"You maintain a concise living-context note for {self.character_name}. "
                f"Character: {char_desc}\n"
                "A session is ending. Consolidate the note for next startup. "
                "Keep it under 600 words. Drop stale items. Output ONLY the note text."
            )
            prompt_user = (
                f"## CURRENT SITUATION NOTE\n{current}\n\n"
                f"Session summary: {self.plan_counter} goal(s) attempted this session.\n"
                "Write the consolidated situation note for next session."
            )
            response = self.infospace_executor.llm_generate(
                [prompt_system, prompt_user], max_tokens=1200, temperature=0.3
            )
            if response.success and response.text and response.text.strip():
                new_content = response.text.strip()
                self._write_named_note('_situation_prev', self.situation_context or '')
                self._write_named_note('_situation', new_content)
                self.situation_context = new_content
                logger.info(f'✓ Consolidated situation note at shutdown ({len(new_content)} chars)')
            else:
                logger.warning(f'Situation consolidation LLM call failed, keeping existing note')
        except Exception as e:
            logger.warning(f'Error consolidating situation note at shutdown: {e}')

    def _write_named_note(self, name: str, content: str):
        """Create or overwrite a named Note via the resource manager."""
        # Try to load existing note by name
        result = self.infospace_executor.execute_action({"type": "load", "target": name, "out": f"$_{name}_tmp"})
        if result.get('status') == 'success' and result.get('resource_id'):
            note_id = result['resource_id']
            success, err = self.resource_manager.update_note_content(note_id, content)
            if not success:
                logger.warning(f'Failed to update note {name}: {err}')
        else:
            # Create new named note
            self.infospace_executor.execute_action({"type": "create-note", "value": content, "name": name, "out": f"$_{name}_tmp"})
            # Persist it
            self.infospace_executor.execute_action({"type": "persist", "target": f"$_{name}_tmp"})

    # ------------------------------------------------------------------
    # Task lifecycle handlers
    # ------------------------------------------------------------------

    def _character_desc_short(self) -> str:
        """Return a concise character description for task prompts."""
        desc = (self.character_config.get('character') or '')[:600]
        caps = (self.character_config.get('capabilities') or '')[:600]
        parts = []
        if desc:
            parts.append(desc.strip())
        if caps:
            parts.append(f"Capabilities: {caps.strip()}")
        return "\n".join(parts) if parts else self.character_name

    def _handle_task_create(self, description: str):
        """User entered 'task: <description>'. Create task and submit planning goal."""
        from task_manager import TASK_PLANNING
        if not description:
            self._say_to_user("Please provide a task description after 'task:'.")
            return
        task_id, task = self.task_manager.create_task(description)
        self.task_manager.set_status(task_id, TASK_PLANNING)
        self.task_manager.set_active_task_goal(task_id, "plan")
        goal_text = self.task_manager.planning_goal_text(task, self._character_desc_short())
        logger.info(f'📋 {self.character_name} Planning task {task_id}: {description[:80]}')
        self.parse_and_set_goal("", goal_text)

    def _handle_task_proceed(self, task_id: str = None):
        """User said 'proceed' (or 'proceed task_X'). Execute the requested ready task."""
        from task_manager import TASK_BLOCKED, TASK_STEP_READY, TASK_EXECUTING
        active = self.task_manager.get_active_tasks()
        ready = [t for t in active if t.get("status") == TASK_STEP_READY]
        if not ready:
            self._say_to_user("No tasks are ready for the next step. Use 'tasks' to see current task states.")
            return
        if task_id:
            task = next((t for t in ready if t.get("task_id") == task_id), None)
            if not task:
                self._say_to_user(f"Task '{task_id}' is not ready. Use 'tasks' to inspect status.")
                return
        else:
            task = ready[0]
        task_id = task["task_id"]
        step_num = task.get("current_step", 1)
        missing_inputs = self._missing_step_inputs(task)
        if missing_inputs:
            reason = f"Missing required input artifacts for step {step_num}: {', '.join(missing_inputs)}"
            self.task_manager.update_task(task_id, blockers=[reason])
            self.task_manager.set_status(task_id, TASK_BLOCKED)
            self._say_to_user(
                f"Task \"{task['name']}\" is blocked.\n"
                f"{reason}\n"
                f"Provide or create these artifacts, then say 'unblock {task_id}'."
            )
            logger.info(f'⛔ {self.character_name} Blocked task {task_id}: {reason}')
            return
        if self.infospace_executor:
            self.infospace_executor.clear_plan_state()
            plan = task.get("abstract_plan", [])
            step = plan[step_num - 1] if 0 < step_num <= len(plan) else {}
            for art_name in step.get("input_artifacts", []):
                if not isinstance(art_name, str) or not art_name.strip():
                    continue
                name = art_name.strip()
                resolved = None
                if hasattr(self.resource_manager, "_resolve_resource_id"):
                    resolved = self.resource_manager._resolve_resource_id(name)
                if resolved:
                    self.infospace_executor._bind_variable(name, resolved)
                    logger.debug(f"Step {step_num} pre-seeded ${name} → {resolved}")
        pre_resource_ids = set(self.resource_manager.resource_registry.keys()) if self.resource_manager else set()
        self.task_manager.set_status(task_id, TASK_EXECUTING)
        self.task_manager.set_active_task_goal(task_id, "execute", step_num, pre_resource_ids=list(pre_resource_ids))
        goal_text = self.task_manager.step_execution_goal_text(task, self._character_desc_short())
        logger.info(f'▶️ {self.character_name} Executing phase {step_num} of task {task_id}')
        self.parse_and_set_goal("", goal_text)

    def _missing_step_inputs(self, task: Dict) -> List[str]:
        """Return unresolved declared inputs for the task's current step."""
        if not self.resource_manager:
            return []
        plan = task.get("abstract_plan", [])
        idx = task.get("current_step", 1) - 1
        if idx < 0 or idx >= len(plan):
            return []
        step = plan[idx] if isinstance(plan[idx], dict) else {}
        inputs = step.get("input_artifacts", [])
        if not isinstance(inputs, list):
            inputs = [inputs] if inputs else []
        outputs = step.get("output_artifacts", [])
        if not isinstance(outputs, list):
            outputs = [outputs] if outputs else []
        output_set = {str(o).strip().lstrip("$") for o in outputs if o}
        missing = []
        for item in inputs:
            if not isinstance(item, str):
                item = str(item)
            name = item.strip()
            if not name:
                continue
            if name.lstrip("$") in output_set:
                continue
            resolved_id = None
            if hasattr(self.resource_manager, "_resolve_resource_id"):
                resolved_id = self.resource_manager._resolve_resource_id(name)
            if resolved_id or self.resource_manager.get_resource(name):
                continue
            missing.append(name)
        return missing

    # ── Task scheduler callbacks ────────────────────────────────────

    def _scheduler_eligible_tasks(self):
        """Return tasks the scheduler may auto-proceed (called from scheduler thread)."""
        from task_manager import TASK_STEP_READY
        from datetime import date, datetime as _dt
        if self.execution_mode == 'autonomous':
            return []  # don't interfere with autonomous goal generation
        active = self.task_manager.get_active_tasks()
        eligible = []
        for t in active:
            if t.get("status") != TASK_STEP_READY:
                continue
            if self._missing_step_inputs(t):
                continue
            mode = t.get("schedule_mode", "manual")
            if mode in ("auto", "recurring"):
                eligible.append(t)
            elif mode == "daily":
                run_at = t.get("run_at", "")          # "HH:MM"
                last_run = t.get("last_run_date", "")  # "YYYY-MM-DD"
                today = date.today().isoformat()
                if not run_at or last_run == today:
                    continue
                now = _dt.now().strftime("%H:%M")
                if now >= run_at:
                    eligible.append(t)
        return eligible

    def _scheduler_proceed_task(self, task_id):
        """Enqueue a synthetic 'proceed' command (called from scheduler thread)."""
        from datetime import date
        task = self.task_manager.get_task(task_id)
        if task and task.get("schedule_mode") == "daily":
            self.task_manager.update_task(task_id, last_run_date=date.today().isoformat())
        command = json.dumps({"text": f"proceed {task_id}", "source": "scheduler"})
        self.text_input_queue.append({"content": command})
        self.execution_paused = False
        self._publish_execution_state()

    def _handle_scheduler_control(self, sample):
        """Zenoh callback for scheduler enable/disable/interval changes."""
        try:
            data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            if 'enable' in data:
                self.task_scheduler.set_enabled(bool(data['enable']))
            if 'interval' in data:
                self.task_scheduler.set_interval(float(data['interval']))
            self._publish_execution_state()
        except Exception as e:
            logger.error(f'Error in scheduler control handler: {e}')

    def _handle_task_schedule_mode(self, sample):
        """Zenoh callback for per-task schedule mode changes."""
        try:
            data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            task_id = data.get("task_id")
            mode = data.get("schedule_mode")
            if not task_id or mode not in ("manual", "auto", "recurring", "daily"):
                logger.warning(f"Invalid task_schedule_mode payload: {data}")
                return
            updates = {"schedule_mode": mode}
            if mode == "daily" and data.get("run_at"):
                updates["run_at"] = data["run_at"]
            self.task_manager.update_task(task_id, **updates)
            logger.info(f"Task {task_id} schedule_mode set to '{mode}'" +
                        (f" run_at={updates.get('run_at')}" if "run_at" in updates else ""))
        except Exception as e:
            logger.error(f"Error in task_schedule_mode handler: {e}")

    def _handle_task_reuse(self, task_id: str = None):
        """User said 'reuse task_X'. Reset task to step 1, restoring initial plan."""
        if not task_id:
            self._say_to_user("Please specify which task to reuse, e.g. 'reuse task_3'.")
            return
        task = self.task_manager.get_task(task_id)
        if not task:
            self._say_to_user(f"Task '{task_id}' not found.")
            return
        reset = self.task_manager.reset_for_reuse(task_id)
        if not reset:
            self._say_to_user(f"Failed to reset task '{task_id}' for reuse.")
            return
        self._say_to_user(f"Task \"{task['name']}\" reset to phase 1. Say 'proceed {task_id}' to start.")
        logger.info(f'🔁 {self.character_name} Reused task {task_id}')

    def _handle_task_terminate(self, task_id: str = None):
        """User said 'terminate' or 'terminate task_X'. Abandon the specified or most recent task."""
        from task_manager import TASK_ABANDONED, TERMINAL_STATUSES
        if task_id:
            task = self.task_manager.get_task(task_id)
            if not task:
                self._say_to_user(f"Task '{task_id}' not found.")
                return
            if task.get("status") in TERMINAL_STATUSES:
                deleted = self.task_manager.delete_task(task_id)
                if not deleted:
                    self._say_to_user(f"Task '{task_id}' could not be removed.")
                    return
                if self.task_manager.active_task_goal and self.task_manager.active_task_goal.get("task_id") == task_id:
                    self.task_manager.clear_active_task_goal()
                self._say_to_user(f"Task '{task['name']}' has been removed.")
                logger.info(f'🗑️ {self.character_name} Removed task {task_id}')
                return
            self.task_manager.set_status(task_id, TASK_ABANDONED)
            self.task_manager.clear_active_task_goal()
            self._say_to_user(f"Task '{task['name']}' has been abandoned.")
            logger.info(f'🛑 {self.character_name} Abandoned task {task_id}')
            return

        active = self.task_manager.get_active_tasks()
        if not active:
            self._say_to_user("No active tasks to terminate.")
            return
        else:
            task = active[-1]
            task_id = task["task_id"]
        self.task_manager.set_status(task_id, TASK_ABANDONED)
        self.task_manager.clear_active_task_goal()
        self._say_to_user(f"Task '{task['name']}' has been abandoned.")
        logger.info(f'🛑 {self.character_name} Abandoned task {task_id}')

    def _handle_task_unblock(self, task_id: str = None):
        """User said 'unblock' or 'unblock task_X'. Clear blockers and set back to step_ready."""
        from task_manager import TASK_BLOCKED, TASK_STEP_READY
        active = self.task_manager.get_active_tasks()
        blocked = [t for t in active if t.get("status") == TASK_BLOCKED]
        if not blocked:
            self._say_to_user("No blocked tasks to unblock.")
            return
        if task_id:
            task = next((t for t in blocked if t.get("task_id") == task_id), None)
            if not task:
                self._say_to_user(f"Task '{task_id}' is not blocked.")
                return
        else:
            task = blocked[0]
            task_id = task["task_id"]
        self.task_manager.update_task(task_id, blockers=[])
        self.task_manager.set_status(task_id, TASK_STEP_READY)
        step = task.get("current_step", 1)
        self._say_to_user(f"Task \"{task['name']}\" unblocked. Step {step} is ready. Say 'proceed' to retry.")
        logger.info(f'🔓 {self.character_name} Unblocked task {task_id} at step {step}')

    def _handle_task_list(self):
        """User said 'tasks'. Report all active task states."""
        tasks = self.task_manager.get_active_tasks()
        if not tasks:
            self._say_to_user("No active tasks.")
            return
        lines = []
        for t in tasks:
            plan = t.get("abstract_plan", [])
            total = len(plan)
            completed = sum(1 for s in plan if s.get("status") == "completed")
            lines.append(f"• [{t['status']}] {t['name']} — step {t.get('current_step', '?')}/{total} ({completed} done)")
            if t.get("blockers"):
                lines.append(f"  Blocked: {'; '.join(t['blockers'])}")
        self._say_to_user("Active tasks:\n" + "\n".join(lines))

    def _handle_task_goal_completed(self, result: Dict[str, Any]):
        """Called after a task-related goal completes. Drives the task state machine."""
        from task_manager import (TASK_STEP_READY, TASK_REVIEWING, TASK_BLOCKED,
                                  TASK_COMPLETED, TASK_EXECUTING)
        atg = self.task_manager.active_task_goal
        if not atg:
            return
        task_id = atg["task_id"]
        goal_type = atg["goal_type"]
        self.task_manager.clear_active_task_goal()

        task = self.task_manager.get_task(task_id)
        if not task:
            logger.warning(f"Task {task_id} not found after goal completion")
            return

        if goal_type == "execute" and self.resource_manager:
            pre_ids = set(atg.get("pre_resource_ids", []) or [])
            now_ids = set(self.resource_manager.resource_registry.keys())
            new_ids = [rid for rid in (now_ids - pre_ids) if rid != "Note_null"]
            if new_ids:
                existing = task.get("task_created_resources", [])
                merged = list(dict.fromkeys(existing + new_ids))
                self.task_manager.update_task(task_id, task_created_resources=merged)
                task = self.task_manager.get_task(task_id) or task

        if goal_type == "plan":
            self._handle_plan_goal_completed(task_id, task, result)
        elif goal_type == "execute":
            self._handle_execute_goal_completed(task_id, task, result)
        elif goal_type == "review":
            self._handle_review_goal_completed(task_id, task, result)

    def _handle_plan_goal_completed(self, task_id: str, task: Dict, result: Dict):
        """Planning goal finished — parse the draft plan and present to user."""
        from task_manager import TASK_STEP_READY, TASK_ABANDONED
        plan_steps = self._read_and_delete_draft_note("_task_plan_draft")
        if plan_steps and isinstance(plan_steps, list):
            self.task_manager.set_abstract_plan(task_id, plan_steps)
            task = self.task_manager.get_task(task_id)
            plan = task.get("abstract_plan", [])
            plan_summary = "\n".join(f"  {s['step']}. {s['description']}" for s in plan)
            self._say_to_user(
                f"Task accepted: \"{task['name']}\"\n"
                f"Plan ({len(plan)} steps):\n{plan_summary}\n\n"
                f"Say 'proceed' to start phase 1, or 'terminate' to abandon."
            )
        else:
            response = (result.get("response") or "")[:500]
            self.task_manager.set_status(task_id, TASK_ABANDONED)
            self._say_to_user(f"Task could not be planned: {response}")

    def _handle_execute_goal_completed(self, task_id: str, task: Dict, result: Dict):
        """Step execution finished — auto-submit review goal (via subplanner, no vision eval)."""
        from task_manager import TASK_REVIEWING
        self.task_manager.set_status(task_id, TASK_REVIEWING)
        step_num = task.get("current_step", 1)
        self.task_manager.set_active_task_goal(task_id, "review", step_num)
        goal_text = self.task_manager.review_goal_text(task, result, self._character_desc_short())
        logger.info(f'🔍 {self.character_name} Reviewing step {step_num} of task {task_id}')
        review_result = self.infospace_executor.call_subplanner(goal=goal_text, max_steps=8)
        self._handle_task_goal_completed(review_result if isinstance(review_result, dict) else {"success": False, "response": str(review_result)})

    def _handle_review_goal_completed(self, task_id: str, task: Dict, result: Dict):
        """Review goal finished — parse review note, update task, report to user."""
        from task_manager import TASK_STEP_READY, TASK_BLOCKED, TASK_COMPLETED
        review = self._read_and_delete_draft_note("_task_review")
        outcome = ""
        output_artifacts = []
        if review and isinstance(review, dict):
            outcome = review.get("step_outcome", "")
            output_artifacts = review.get("output_artifacts", [])
            if review.get("blocked"):
                self.task_manager.update_task(task_id, blockers=[review.get("block_reason", "unknown")])
                self.task_manager.set_status(task_id, TASK_BLOCKED)
                self._say_to_user(
                    f"Task \"{task['name']}\" is blocked: {review.get('block_reason', 'unknown')}\n"
                    f"Step outcome: {outcome}"
                )
                self.task_scheduler.notify_task_terminal(task_id)
                return
            revised = review.get("revised_remaining_steps")
            if revised and isinstance(revised, list):
                self.task_manager.complete_current_step(task_id, outcome, output_artifacts)
                task = self.task_manager.get_task(task_id)
                if task and task.get("status") != TASK_COMPLETED:
                    self.task_manager.update_remaining_plan(task_id, revised)
            else:
                self.task_manager.complete_current_step(task_id, outcome, output_artifacts)
        else:
            response = (result.get("response") or "step completed")[:300]
            self.task_manager.complete_current_step(task_id, response, [])

        task = self.task_manager.get_task(task_id)
        if not task:
            return
        if task.get("status") == TASK_COMPLETED:
            self._cleanup_completed_task_resources(task_id)
            self._say_to_user(
                f"Task \"{task['name']}\" is complete!\n"
                f"Last step outcome: {outcome or 'done'}"
            )
            if task.get("schedule_mode") in ("recurring", "daily"):
                self.task_manager.reset_for_reuse(task_id)
                mode_label = "Recurring" if task.get("schedule_mode") == "recurring" else "Daily"
                self._say_to_user(f"{mode_label} task \"{task['name']}\" reset to phase 1.")
                self.task_scheduler.notify_step_completed(task_id)
            else:
                self.task_scheduler.notify_task_terminal(task_id)
        else:
            plan = task.get("abstract_plan", [])
            idx = task.get("current_step", 1) - 1
            next_desc = plan[idx]["description"] if 0 <= idx < len(plan) else "unknown"
            arts_str = ", ".join(output_artifacts) if output_artifacts else "none recorded"
            self._say_to_user(
                f"Step {task.get('current_step', 1) - 1} complete: {outcome or 'done'}\n"
                f"Output artifacts: {arts_str}\n"
                f"Next step ({task.get('current_step', '?')} of {len(plan)}): {next_desc}\n\n"
                f"Say 'proceed' for next step, or 'terminate' to abandon."
            )
            self.task_scheduler.notify_step_completed(task_id)

    def _cleanup_completed_task_resources(self, task_id: str):
        """
        Conservative cleanup for completed tasks.
        Deletes only task-created, non-persistent resources that are not explicitly kept.
        """
        task = self.task_manager.get_task(task_id)
        if not task or not self.resource_manager:
            return

        created_ids = set(task.get("task_created_resources", []))
        if not created_ids:
            return

        keep_ids = set(task.get("artifacts", []))
        for step in task.get("abstract_plan", []):
            step_outputs = step.get("output_artifacts", [])
            for art in step_outputs or []:
                if isinstance(art, str):
                    keep_ids.add(art)
                    keep_ids.add(art.strip())
                    if hasattr(self.resource_manager, "_resolve_resource_id"):
                        resolved = self.resource_manager._resolve_resource_id(art)
                        if resolved:
                            keep_ids.add(resolved)
        preserved_collections = {"conversation", "conversation_history", "_tasks"}
        preserved_notes = {"_situation", "_situation_prev"}
        deleted = 0
        for resource_id in created_ids:
            if resource_id in keep_ids:
                continue
            resource = self.resource_manager.get_resource(resource_id)
            if not resource:
                continue
            props = resource.get("properties", {})
            if props.get("persistent", False):
                continue
            note_name = props.get("note_name", "")
            collection_name = props.get("collection_name", "")
            if note_name in preserved_notes:
                continue
            if collection_name in preserved_collections:
                continue
            if note_name.startswith("_task_"):
                continue
            if note_name in {"_task_plan_draft", "_task_review"}:
                continue
            if resource_id == "Note_null":
                continue
            success, _ = self._delete_resource_and_unbind(resource_id)
            if success:
                deleted += 1
        if deleted:
            logger.info(f"🧹 Cleaned {deleted} non-kept task resources for {task_id}")

    def _read_and_delete_draft_note(self, note_name: str) -> Any:
        """Load a named Note, parse its JSON content, delete it, return parsed data."""
        result = self.infospace_executor.execute_action({"type": "load", "target": note_name, "out": "$_draft_tmp"})
        if result.get("status") != "success" or not result.get("resource_id"):
            return None
        note_id = result["resource_id"]
        content = self.infospace_executor._get_content(note_id)
        # Clean up the draft note
        try:
            self._delete_resource_and_unbind(note_id)
        except Exception:
            pass
        if not content:
            return None
        try:
            return json.loads(content) if isinstance(content, str) else content
        except (json.JSONDecodeError, TypeError):
            return content

    def _say_to_user(self, text: str):
        """Send a message to user via the say action."""
        if self.infospace_executor:
            self.infospace_executor.execute_action({"type": "say", "target": "user", "value": text})

    def _delete_resource_and_unbind(self, resource_id: str):
        """Delete a resource and clear any binding variables targeting it."""
        if not self.resource_manager:
            return False, "Resource manager not available"
        if self.infospace_executor and hasattr(self.infospace_executor, "delete_resource_and_unbind"):
            return self.infospace_executor.delete_resource_and_unbind(resource_id)
        success, error_msg = self.resource_manager.delete_resource(resource_id)
        if success and self.infospace_executor and hasattr(self.infospace_executor, "_remove_bindings_for_resource"):
            self.infospace_executor._remove_bindings_for_resource(resource_id)
        return success, error_msg

    def _archive_conversation(self):
        """
        Archive conversation on shutdown: synthesize conversation collection and add to conversation_history.
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
            
            # Summarize conversation collection using synthesize tool
            summarize_action = {"type": "synthesize", "target":  "$conv", "focus": "concise conversation summary", "out": "$summary"}
            summarize_result = self.infospace_executor.execute_action(summarize_action)
            
            if summarize_result.get('status') != 'success':
                logger.warning(f'Failed to synthesize conversation: {summarize_result.get("reason", "unknown")}')
                return
            
            # The synthesize tool already created a Note and bound it to $summary
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
            
            # Add the summary note (already created by synthesize tool) to conversation_history
            add_action = {"type": "add", "target": "conversation_history", "value": "$summary", "out": "$conversation_history"}
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
        close_flag = False
        try:
            content_data = json.loads(content)
            text_input = content_data.get('text', '')
            source = content_data.get('source', 'unknown')
            close_flag = content_data.get('close', False)
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
            
            # Handle scheduler-issued proceed commands
            if source == 'scheduler' and clean_input.strip().lower().startswith('proceed'):
                parts = clean_input.strip().split()
                task_id = parts[1] if len(parts) > 1 and parts[1].startswith('task_') else None
                self._handle_task_proceed(task_id=task_id)
                return

            # Handle special commands from User BEFORE processing as dialog
            if source == 'User':
                if clean_input.startswith('goal:'):
                    # Explicit goal command - process as before
                    goal_preview = clean_input[:80] + ('...' if len(clean_input) > 80 else '')
                    logger.info(f'📥 {self.character_name} Received goal: "{goal_preview}"')
                    self.parse_and_set_goal("", clean_input[5:].strip())
                    return  # Don't process as speech
                elif clean_input.startswith('task:'):
                    self._handle_task_create(clean_input[5:].strip())
                    return
                elif clean_input.strip().lower().startswith('proceed') or clean_input.strip().lower() == 'next step':
                    parts = clean_input.strip().split()
                    task_id = parts[1] if len(parts) > 1 and parts[1].startswith('task_') else None
                    self._handle_task_proceed(task_id=task_id)
                    return
                elif clean_input.strip().lower().startswith('terminate'):
                    parts = clean_input.strip().split()
                    task_id = parts[1] if len(parts) > 1 and parts[1].startswith('task_') else None
                    self._handle_task_terminate(task_id=task_id)
                    return
                elif clean_input.strip().lower().startswith('reuse'):
                    parts = clean_input.strip().split()
                    task_id = parts[1] if len(parts) > 1 and parts[1].startswith('task_') else None
                    self._handle_task_reuse(task_id=task_id)
                    return
                elif clean_input.strip().lower().startswith('unblock'):
                    parts = clean_input.strip().split()
                    task_id = parts[1] if len(parts) > 1 and parts[1].startswith('task_') else None
                    self._handle_task_unblock(task_id=task_id)
                    return
                elif clean_input.strip().lower() in ('tasks', 'task list', 'list tasks'):
                    self._handle_task_list()
                    return
                else:
                    # Regular user input
                    self._create_character_note()
                    # End-of-conversation keywords
                    end_phrases = ('goodbye', 'bye', 'end conversation', "we're done", "that's all", 'done talking')
                    if clean_input.strip().lower() in end_phrases:
                        self.conversation_store.close_dialog("User")
                        logger.info(f'📥 {self.character_name} User ended conversation')
                        return

                    goal_lower = clean_input.lower()
                    dialog_keywords = ('ask', 'tell', 'discuss', 'talk')
                    other_agents = [n for n in self.character_config.get('characters', {}).keys() if n != self.character_name]
                    mentioned = [a for a in other_agents if a in clean_input]
                    if mentioned and any(kw in goal_lower for kw in dialog_keywords):
                        other_agent = mentioned[0]
                        self._dialog_purposes[other_agent] = clean_input[:500]
                        goal_text = f"Initiate dialog with {other_agent}"
                        context = f"\n## CONTEXT ##\nUser instruction: {clean_input[:500]}"
                        logger.info(f'📥 {self.character_name} Received dialog goal (wrapped): "{goal_text}"')
                        self.parse_and_set_goal("", f"{goal_text}{context}")
                    else:
                        # User talking to character — treat as conversation (initiate or continue)
                        in_conversation = self.conversation_store.is_dialog_active("User") and self.conversation_store.get_turn_count("User") > 0
                        # Side note: when initiating (no prior dialog), envision has less context; fallback handles it
                        envision = self._envision_conversation_turn("User", clean_input, "")
                        goal_text = "Continue dialog with User" if in_conversation else "Respond to User"
                        context = (
                            f"\n## CONTEXT ##\n"
                            f"Their move: {envision['turn_intent']}\n"
                            f"Your move: {envision['my_move']}\n"
                            f"Message from User: {clean_input[:500]}"
                        )
                        logger.info(f'📥 {self.character_name} Received: "{goal_text}"')
                        self.parse_and_set_goal("", f"{goal_text}{context}")
                    self.execution_paused = False
                    self._publish_execution_state()
                    return  # Don't process as speech
            
            # Agent-to-agent message processing — treat like User input (create goal)
            if source and source not in ('unknown', 'console'):
                # Dedup: skip if identical to last message from this source
                if self._last_agent_message.get(source) == clean_input:
                    logger.info(f'📥 {self.character_name} Skipping duplicate message from {source}')
                    return
                self._last_agent_message[source] = clean_input
                self._create_character_note()
                goal_preview = clean_input[:80] + ('...' if len(clean_input) > 80 else '')
                logger.info(f'📥 {self.character_name} Received message from {source}: "{goal_preview}"')

                # Close flag: other agent signaled end of conversation — record but don't reply
                if close_flag:
                    logger.info(f'📥 {self.character_name} Dialog closed by {source} (close flag)')
                    self.conversation_store.close_dialog(source)
                    self._dialog_cooldowns[source] = time.time()
                    self._dialog_purposes.pop(source, None)
                    return

                # Cooldown: suppress re-open if we just closed dialog with this source
                cooldown_until = self._dialog_cooldowns.get(source, 0)
                if time.time() - cooldown_until < 10:
                    logger.info(f'📥 {self.character_name} Suppressing re-open from {source} (cooldown)')
                    return

                # Envision the conversational moment
                purpose = self._dialog_purposes.get(source, '')
                envision = self._envision_conversation_turn(source, clean_input, purpose)
                goal_text = f"Continue dialog with {source}"
                context = (
                    f"\n## CONTEXT ##\n"
                    f"Their move: {envision['turn_intent']}\n"
                    f"Your move: {envision['my_move']}\n"
                    f"Message from {source}: {clean_input[:500]}\n"
                    f"Do NOT say anything to User. This is a direct conversation with {source} only."
                )
                if purpose:
                    context += f"\nOriginal purpose: {purpose}"
                # Check turn count and natural dialog end
                turn_count = 0
                should_close = False
                try:
                    turn_count = self.conversation_store.get_turn_count(source)
                except Exception:
                    pass
                if turn_count >= 8:
                    should_close = True
                    logger.info(f'📥 {self.character_name} Enforcing closure: turn_count={turn_count}')
                if should_close:
                    context += "\n\nThis conversation should end now. Send a BRIEF closing statement. Your say action MUST include close: true."
                elif turn_count >= 6:
                    context += "\n\nConsider wrapping up — conversation is approaching the 8-turn limit."
                self.parse_and_set_goal("", f"{goal_text}{context}")
                self.execution_paused = False
                self._publish_execution_state()
                return

            # Normal dialog processing
            logger.info(f'📥 {self.character_name} Processing text input: "{text_input}" (source: {source})')
            self.plan_just_generated = True  # Skip action execution this turn
        
    def _envision_conversation_turn(self, source: str, message: str, purpose: str = '') -> dict:
        """Lightweight LLM call to characterize the conversational moment."""
        # Gather recent dialog turns for context
        recent_turns = ""
        entity_data = self.conversation_store.get_entity_context(source, limit=6, scope='current')
        if entity_data and 'conversation_history' in entity_data:
            for entry in entity_data['conversation_history'][-6:]:
                if isinstance(entry, dict) and 'source' in entry and 'text' in entry:
                    text_preview = str(entry['text'])[:200]
                    recent_turns += f"{entry['source']}: {text_preview}\n"

        # Include character, setting, and capabilities so envision knows what the agent can do
        char_desc = (self.character_config.get('character') or '')[:500]
        setting = (self.character_config.get('setting') or '')[:600]
        capabilities = (self.character_config.get('capabilities') or '')[:500]
        context_lines = []
        if char_desc:
            context_lines.append(f"CHARACTER: {char_desc}")
        if setting:
            context_lines.append(f"SETTING/WORLD: {setting}")
        if capabilities:
            context_lines.append(f"CAPABILITIES: {capabilities}")
        context_block = "\n".join(context_lines) + "\n\n" if context_lines else ""

        purpose_line = f"CONVERSATION PURPOSE: {purpose}\n" if purpose else ""
        prompt = (
            f"{context_block}"
            f"{purpose_line}"
            f"RECENT DIALOG:\n{recent_turns}\n"
            f"INCOMING MESSAGE from {source}:\n{message[:500]}\n\n"
            f"You are {self.character_name}. Given this conversational moment, provide:\n"
            f"1. TURN_INTENT: What conversational move is {source} making? (1 sentence)\n"
            f"2. MY_MOVE: Name the dialogue act you should perform next. \n"
            f"  - Describe it as an action (e.g., 'comply and report the target artifact contents'), not as the response itself.\n"
            f"  - Do not generate response content here.\n"
            f"  - If the conversation's purpose has been achieved, indicate you should say so and wrap up. (1 sentence)\n"
        )

        try:
            result = self.llm_generate(messages=[prompt], max_tokens=128, temperature=0.3, stops=["\n\n"])
            if result.success and result.text:
                text = result.text.strip()
                turn_intent = ""
                my_move = ""
                for line in text.split('\n'):
                    line = line.strip()
                    if line.upper().startswith('TURN_INTENT:') or line.startswith('1.'):
                        turn_intent = line.split(':', 1)[-1].strip() if ':' in line else line[2:].strip()
                    elif line.upper().startswith('MY_MOVE:') or line.startswith('2.'):
                        my_move = line.split(':', 1)[-1].strip() if ':' in line else line[2:].strip()
                if turn_intent and my_move:
                    logger.info(f'🎭 Envision: intent="{turn_intent[:60]}" move="{my_move[:60]}"')
                    return {'turn_intent': turn_intent, 'my_move': my_move}
        except Exception as e:
            logger.warning(f'Envision failed: {e}')

        # Fallback: generic framing
        return {'turn_intent': f'{source} is communicating', 'my_move': f'Engage with {source} — contribute your own perspective'}

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
            
            # Resolve declared output_artifacts for the current task step (used as eval target hint)
            step_output_artifacts = []
            active_goal = self.task_manager.active_task_goal if self.task_manager else None
            if active_goal and active_goal.get("goal_type") == "execute":
                task_for_context = self.task_manager.get_task(active_goal["task_id"])
                if task_for_context:
                    plan_steps = task_for_context.get("abstract_plan", [])
                    step_idx = task_for_context.get("current_step", 1) - 1
                    if 0 <= step_idx < len(plan_steps):
                        oa = plan_steps[step_idx].get("output_artifacts", [])
                        step_output_artifacts = oa if isinstance(oa, list) else ([oa] if oa else [])

            # Build scoped vision goal for step execution (avoids _generate_vision using the full task description)
            vision_goal = goal_text
            if active_goal and active_goal.get("goal_type") == "execute" and task_for_context:
                step_idx = task_for_context.get("current_step", 1) - 1
                plan_steps = task_for_context.get("abstract_plan", [])
                if 0 <= step_idx < len(plan_steps):
                    step_desc = plan_steps[step_idx].get("description", "")
                    step_outs = ", ".join(step_output_artifacts) or "none declared"
                    vision_goal = f"Step: {step_desc}\nDeclared outputs: {step_outs}"

            # Build context for planner (always needed for infospace)
            context = {
                'variables': self.infospace_executor.plan_bindings_flat if self.infospace_executor else {},
                'character_context': character_context,
                'situation_context': self._build_situation_context(),
                'recent_context': recent_context,
                'executor': self.infospace_executor,  # Pass executor for incremental planner
                'output_artifacts': step_output_artifacts,
                'vision_goal': vision_goal,
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
                
                # Add conversation entry to note/collection conversation store
                self.conversation_store.record_incoming(source, text_input, close=bool(content_data.get('close', False)) if 'content_data' in locals() else False)
                
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
                            success, _ = self._delete_resource_and_unbind(note_id)
                            if success:
                                deleted_notes += 1
                    
                    # Delete the Collection itself
                    success, error_msg = self._delete_resource_and_unbind(collection_id)
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
            
            PRESERVED_COLLECTIONS = {'conversation', 'conversation_history', '_tasks'}
            PRESERVED_NOTES = {'_situation', '_situation_prev'}
            PRESERVED_NOTE_PREFIXES = ('_task_',)
            
            deleted_notes = 0
            deleted_collections = 0
            to_delete = []
            for resource_id, resource_data in self.resource_manager.resource_registry.items():
                props = resource_data.get('properties', {})
                
                # Skip persistent resources
                if props.get('persistent', False):
                    continue
                
                if resource_id.startswith('Note_') and resource_id != 'Note_null':
                    note_name = props.get('note_name')
                    if note_name and (note_name in PRESERVED_NOTES or note_name.startswith(PRESERVED_NOTE_PREFIXES)):
                        continue
                    if any(self.resource_manager.named_notes.get(pn) == resource_id for pn in PRESERVED_NOTES):
                        continue
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
                success, _ = self._delete_resource_and_unbind(resource_id)
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
    
    def handle_clear_persistents(self, sample):
        """Handle clear persistents command - clears all persistent resources."""
        try:
            if not self.resource_manager:
                logger.error("Resource manager not available, cannot clear persistents")
                return
            
            PRESERVED_COLLECTIONS = {'conversation', 'conversation_history', '_tasks'}
            PRESERVED_NOTES = {'_situation', '_situation_prev'}
            PRESERVED_NOTE_PREFIXES = ('_task_',)
            
            deleted_notes = 0
            deleted_collections = 0
            to_delete = []
            for resource_id, resource_data in self.resource_manager.resource_registry.items():
                props = resource_data.get('properties', {})
                
                # Only target persistent resources
                if not props.get('persistent', False):
                    continue
                
                if resource_id.startswith('Note_') and resource_id != 'Note_null':
                    note_name = props.get('note_name')
                    if note_name and (note_name in PRESERVED_NOTES or note_name.startswith(PRESERVED_NOTE_PREFIXES)):
                        continue
                    if any(self.resource_manager.named_notes.get(pn) == resource_id for pn in PRESERVED_NOTES):
                        continue
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
                success, _ = self._delete_resource_and_unbind(resource_id)
                if success:
                    if resource_id.startswith('Note_'):
                        deleted_notes += 1
                    else:
                        deleted_collections += 1
            
            # Clear bindings that reference deleted resources
            bindings_cleared = 0
            if self.infospace_executor:
                deleted_set = set(to_delete)
                for scope in self.infospace_executor.plan_bindings:
                    to_remove = [k for k, v in scope.items() if v in deleted_set]
                    for k in to_remove:
                        del scope[k]
                        bindings_cleared += 1
            
            logger.info(f"🗑️ Cleared persistents - {deleted_notes} Notes, {deleted_collections} Collections, {bindings_cleared} bindings")
        except Exception as e:
            logger.error(f'Error clearing persistents: {e}')
            traceback.print_exc()
    
    def handle_stop_command(self, sample):
        """Handle stop command - stop autonomous mode and pause execution."""
        try:
            logger.info(f'⏹️ Stop command received by {self.character_name}')
            # Treat stop as an immediate interrupt signal so blocking waits can exit.
            self.interrupt_requested = True
            if self.infospace_executor:
                self.infospace_executor.interrupt_requested = True
            self.autonomous_mode = False
            self.execution_mode = 'step'
            self.execution_paused = True
            # Note: continuous_mode is NOT cleared here - user must toggle it off explicitly
            self.awaiting_user_input = False  # Clear awaiting flag
            self.awaiting_ask_response = False
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
            active_dialog = False
            try:
                active_dialog = self.conversation_store.has_active_dialogs()
            except Exception:
                pass
            state_data = {
                'paused': self.execution_paused,
                'mode': self.execution_mode,
                'character': self.character_name,
                'continuous_mode': self.continuous_mode,
                'active_dialog': active_dialog,
                'task_scheduler': self.task_scheduler.get_status() if hasattr(self, 'task_scheduler') else None,
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
            success, error_msg = self._delete_resource_and_unbind(resource_id)
            
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
        
        # Collections and Notes to preserve (by name) - should persist across benchmark runs
        PRESERVED_COLLECTIONS = {'conversation', 'conversation_history', '_tasks'}
        PRESERVED_NOTES = {'_situation', '_situation_prev'}
        PRESERVED_NOTE_PREFIXES = ('_task_',)
        
        deleted_notes = 0
        deleted_collections = 0
        to_delete = []
        for resource_id, resource_data in self.resource_manager.resource_registry.items():
            props = resource_data.get('properties', {})
            
            # Skip persistent resources
            if props.get('persistent', False):
                continue
            
            if resource_id.startswith('Note_') and resource_id != 'Note_null':
                note_name = props.get('note_name')
                if note_name and (note_name in PRESERVED_NOTES or note_name.startswith(PRESERVED_NOTE_PREFIXES)):
                    logger.debug(f"Preserving special Note: {note_name}")
                    continue
                if any(self.resource_manager.named_notes.get(pn) == resource_id for pn in PRESERVED_NOTES):
                    continue
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
            success, _ = self._delete_resource_and_unbind(resource_id)
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
                    self._delete_resource_and_unbind(resource_id)
            
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
    
    def _tasks_query_handler(self, query):
        """Handle query for tasks (active + terminal)."""
        try:
            tasks = []
            if hasattr(self, 'task_manager') and self.task_manager:
                tasks = self.task_manager.get_all_tasks()
            response = {'success': True, 'tasks': tasks}
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
        except Exception as e:
            logger.error(f'Error in tasks query handler: {e}')
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
        handler_start = time.time()
        logger.info(f'📥 _sync_plan_execution_handler called (query key: {query.key_expr})')
        try:
            if not self.infospace_executor:
                response = {
                    'success': False,
                    'error': 'Sync plan execution only available for infospace characters'
                }
                query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
                logger.info(f'📤 Replied with error: no executor')
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
            
            # Single action optimization: use fast path (like _process_text_input)
            if len(plan_steps) == 1:
                action = plan_steps[0]
                action_type = action.get('type', 'unknown')
                logger.info(f'🔧 Single-action fast path: {action_type}')
                try:
                    # Execute directly (fast path)
                    exec_start = time.time()
                    result = self.infospace_executor.execute_action(action)
                    exec_elapsed = time.time() - exec_start
                    logger.info(f'✅ Action {action_type} executed in {exec_elapsed:.3f}s, status={result.get("status")}')
                    
                    timestamp = datetime.now()
                    
                    # Publish action result for UI consistency (like _process_text_input does)
                    pub_start = time.time()
                    self._publish_action_result(action, result, action_type, timestamp)
                    pub_elapsed = time.time() - pub_start
                    logger.debug(f'Published action result in {pub_elapsed:.3f}s')
                    
                    # Reply with uniform_result format
                    response = {
                        'success': result.get('status') == 'success',
                        'status': result.get('status'),
                        'reason': result.get('reason'),
                        'executed_steps': 1,
                        'bindings': {},  # Single actions don't modify bindings
                        'last_action_result': result,  # Full uniform_result format
                        'suspended': False,
                        'suspension_reason': None
                    }
                    reply_start = time.time()
                    query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
                    reply_elapsed = time.time() - reply_start
                    handler_elapsed = time.time() - handler_start
                    logger.info(f'📤 Replied to query in {reply_elapsed:.3f}s (total handler time: {handler_elapsed:.3f}s)')
                    return
                except Exception as e:
                    logger.error(f'❌ Single-action execution failed: {e}')
                    import traceback
                    logger.error(traceback.format_exc())
                    error_result = self.infospace_executor._create_uniform_return('failed', reason=str(e))
                    timestamp = datetime.now()
                    self._publish_action_result(action, error_result, action.get('type', 'unknown'), timestamp)
                    response = {
                        'success': False,
                        'status': 'failed',
                        'error': str(e),
                        'reason': str(e),
                        'executed_steps': 0,
                        'bindings': {},
                        'last_action_result': error_result,
                        'suspended': False,
                        'suspension_reason': None
                    }
                    query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
                    return
            
            # Multi-action plan: execute synchronously via full plan execution
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
            logger.error(f'❌ Invalid JSON in plan execution request: {e}')
            response = {
                'success': False,
                'error': f'Invalid JSON: {str(e)}'
            }
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
            logger.info(f'📤 Replied with JSON decode error')
        except Exception as e:
            handler_elapsed = time.time() - handler_start if 'handler_start' in locals() else 0
            logger.error(f'❌ Error handling sync plan execution query after {handler_elapsed:.3f}s: {e}')
            import traceback
            logger.error(traceback.format_exc())
            response = {
                'success': False,
                'error': str(e),
                'executed_steps': 0,
                'bindings': {},
                'last_action_result': None
            }
            try:
                query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
                logger.info(f'📤 Replied with exception error')
            except Exception as reply_error:
                logger.error(f'❌❌ Failed to send error reply: {reply_error}')
    

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
            # New user submit starts a fresh planning turn; consume any stale interrupt from prior End/Stop.
            self.interrupt_requested = False
            if self.infospace_executor:
                self.infospace_executor.interrupt_requested = False
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

            # Drive task state machine if this goal was task-related
            if self.task_manager.active_task_goal:
                self._handle_task_goal_completed(result)
                return

            if result.get('success', False):
                self.current_plan = result.get('plan', {})
                self._publish_current_plan()
                self.plan_just_generated = True
                if result.get('response', None):
                    self.infospace_executor.execute_action({"type": "say", "target": "user", "value": result.get('response')})
            else:
                if result.get('response') == "Interrupted by user.":
                    logger.info(f"Planning interrupted for {self.character_name}; clearing current plan for clean terminal ask turn.")
                    self.current_plan = None
                elif result.get('response'):
                    logger.warning(
                        f"Plan returned non-success with response (quality_status={result.get('quality_status', 'unknown')}, "
                        f"verification_answer={result.get('verification_answer', '')}); not treating as internal error."
                    )
                else:
                    logger.error(f"Error in _plan: {result.get('error', 'Unknown error')}")
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
            entity_data = self.conversation_store.get_entity_context("User", limit=num_entries, scope='all')
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
        Get entity data from conversation store for context.
        
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
            entity_data = self.conversation_store.get_entity_context(entity_name, limit=limit, scope=scope)
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

            # Stop task scheduler
            if hasattr(self, 'task_scheduler'):
                self.task_scheduler.stop()

            # Consolidate situation note while LLM is still available
            try:
                self._consolidate_situation_note()
            except Exception as e:
                logger.error(f'Error consolidating situation note during shutdown: {e}')
            
            # Archive conversation if it exists and is not empty (do this first before saving state)
            try:
                self._archive_conversation()
            except Exception as e:
                logger.error(f'Error archiving conversation during shutdown: {e}')
                import traceback
                traceback.print_exc()
            
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
