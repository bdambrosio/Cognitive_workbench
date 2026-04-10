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
import queue
import logging
import sys
import signal
import argparse
import os
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Any, Union, Optional, Tuple
from dataclasses import dataclass, asdict
from weakref import WeakValueDictionary
from turn_metrics import TurnMetrics

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
_cli_mode = os.getenv('CWB_CLI_MODE', '') == '1'

# File handler with INFO level (full logging)
file_handler = logging.FileHandler('logs/executive_node.log', mode='w')
file_handler.setLevel(logging.INFO)

_handlers = [file_handler]
if not _cli_mode:
    # Console handler only when not in CLI mode (CLI owns the terminal)
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(logging.WARNING)
    _handlers.insert(0, console_handler)

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
    handlers=_handlers,
    force=True
)
logger = logging.getLogger('executive_node')
logger.setLevel(logging.INFO)

SCHEDULED_GOAL_NOTE_PREFIX = "_scheduled_goal_"
SCHEDULED_GOALS_COLLECTION = "_scheduled_goals"


def _is_goal_cmd(s):
    """Legacy helper — only referenced by _DEAD_process_text_input_original."""
    return s and s.strip().lower().startswith('goal:')


# ── OODA pipeline data structures ──────────────────────────────────────

@dataclass
class EventPacket:
    """Structured event produced by Observe stage."""
    event_type: str          # 'user_text', 'sensor_event', 'timer'
    classification: str      # 'chat', 'alert', 'trigger', 'trigger_task', 'inform', 'timer', 'ask_reply', 'agent_message'
    content: str
    source: str
    raw_sense_data: dict
    goal_id: Optional[str] = None
    goal_name: Optional[str] = None
    task_template: Optional[str] = None
    close_flag: bool = False
    is_reaction: bool = False


@dataclass
class OrientedEvent:
    """Event enriched with character evaluation from Orient stage."""
    event: EventPacket
    assessment: Optional[Dict[str, Any]]


@dataclass
class Action:
    """Routing decision produced by Decide stage."""
    type: str               # 'dispatch_goal', 'proceed_goal', 'reuse_goal', 'chat_response',
                            # 'alert_response', 'ask_reply', 'agent_message',
                            # 'proactive_remark', 'trigger_existing_goal',
                            # 'no_action'
    payload: dict
    assessment: Optional[Dict[str, Any]] = None


# Import LLM client
from llm_client import ZenohLLMClient
import character_evaluator
from output_sizing import compute_output_guidance, format_guidance_for_planner

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

    def _register_tool(self, tool_dir_path: str) -> bool:
        """Hot-register a single tool from its directory into all subsystems."""
        from utils.tool_loader import load_tools
        from pathlib import Path
        parent = str(Path(tool_dir_path).parent)
        new_tools = load_tools(parent)
        tool_name = Path(tool_dir_path).name
        if tool_name not in new_tools:
            logger.warning(f"_register_tool: '{tool_name}' not found in {parent}")
            return False
        meta = new_tools[tool_name]
        self.available_tools[tool_name] = meta
        if self.infospace_executor:
            self.infospace_executor.available_tools[tool_name] = meta
        if self.incremental_planner:
            from incremental_planner import build_tool_catalog, tool_catalog_text
            self.incremental_planner.available_tools[tool_name] = meta
            self.incremental_planner.tools = build_tool_catalog(self.incremental_planner.available_tools)
            self.incremental_planner.tools_catalog_text = tool_catalog_text(self.incremental_planner.tools)
        logger.info(f"_register_tool: '{tool_name}' registered (type={meta.get('type')})")
        return True

    def __init__(self, character_name="default", character_config=None, runtime=None, tokenizer=None):
        # Store character info (canonicalized)
        self.character_name = character_name.capitalize()
        self.character_config = character_config or {}
        self.drives = self.character_config.get('drives', [])
        self.drives_str = '\n'.join(self.drives)   
        
        # Debug mode flag - must be set early as it's used throughout initialization
        self.debug = str(os.getenv('CWB_DEBUG', '')).lower() in ('1', 'true', 'yes', 'on')
        if self.debug:
            logger.info(f'🔧 Debug mode enabled for {self.character_name}')
        
        # Shutdown flag — must be early since Zenoh callbacks can fire before run()
        self.shutdown_requested = False

        # Manual control flags
        self.manual = bool(self.character_config.get('manual', False))
        self.manual_response = bool(self.character_config.get('manual_response', False))
        
        # Benchmark mode flag - disables conversation collection initialization and updates
        self.benchmark_mode = bool(self.character_config.get('benchmark_mode', False))
        if self.benchmark_mode:
            logger.info(f'📊 Benchmark mode enabled for {self.character_name} - conversation collections disabled')
        
        # Initialize Zenoh session (localhost only)
        from utils.zenoh_utils import make_localhost_config
        self.session = zenoh.open(make_localhost_config())
        
        # LLM toggle state (Primary/Alt, switched between goals)
        self.llm_mode = 'primary'  # 'primary' or 'alt'
        self.llm_switch_pending = None  # Set to 'primary'/'alt' by UI, applied at next goal start

        # Sensor priority queues — must be initialized before the sense_data
        # subscriber is declared, because sensor callbacks can fire immediately.
        self._sensor_alert_queue: list = []    # disposition='alert' — high priority
        self._sensor_trigger_queue: list = []  # disposition='trigger:X' — goal dispatch
        self._sensor_trigger_task_queue: list = []  # disposition='trigger-task:X' — task dispatch
        self._sensor_inform_queue: list = []   # disposition='inform' — rolling context (last 10)

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
        # PAYLOAD: Complete result dict from incremental_planner.generate_plan() with plan, response, success, error_count
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
        
        # LLM backend - Use shared/local SGLang when available, direct provider APIs otherwise,
        # and fall back to ZenohLLMClient only when no direct backend is configured.
        llm_config = self.character_config.get('llm_config', {})
        server_name = llm_config.get('server_name', 'openai')
        model_name = llm_config.get('model_name', 'gpt-4.1')
        sgl_model_path = llm_config.get('sgl_model_path')
        vllm_model_path = llm_config.get('vllm_model_path')
        vllm_url = llm_config.get('vllm_url', 'http://localhost:5000')
        # Normalize: strip trailing path so we have a clean base for /v1/models etc.
        vllm_base = vllm_url.split('/v1')[0] if '/v1' in vllm_url else vllm_url.rstrip('/')
        openai_model_path = llm_config.get('openai_model_path')
        openrouter_model_path = llm_config.get('openrouter_model_path')
        openrouter_provider = llm_config.get('openrouter_provider')
        anthropic_model_path = llm_config.get('anthropic_model_path')
        # extra_request_params: arbitrary key-value pairs merged into LLM API payloads
        # Subsumes the old 'reasoning' config key. Supports any backend-specific params
        # e.g. {"reasoning": {"effort": "low"}, "extra_body": {"chat_template_kwargs": {"enable_thinking": false}}}
        self.extra_request_params = llm_config.get('extra_request_params', {})
        # Backwards compatibility: if old 'reasoning' key exists, fold it in
        if not self.extra_request_params and llm_config.get('reasoning'):
            self.extra_request_params = {"reasoning": llm_config['reasoning']}

        self.runtime = None
        self.llm_client = None
        self.vllm_model = None
        self.vllm_url = None
        self.openai_model = None
        self.openai_api_key = None
        self.openrouter_model = None
        self.openrouter_api_key = None
        self.openrouter_provider = None
        self.anthropic_model = None
        self.anthropic_api_key = None
        self.alt_openai_model = None
        self.alt_openai_api_key = None
        self.alt_openrouter_model = None
        self.alt_openrouter_api_key = None
        self.alt_openrouter_provider = None
        self.alt_vllm_model = None
        self.alt_vllm_url = None
        self.alt_anthropic_model = None
        self.alt_anthropic_api_key = None
        
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
        # Initialize OpenAI API if configured
        elif openai_model_path:
            api_key = os.getenv('OPENAI_API_KEY')
            if not api_key:
                logger.error("❌ OpenAI model configured but OPENAI_API_KEY environment variable not set")
                raise ValueError("OPENAI_API_KEY environment variable required for OpenAI API")
            self.openai_model = openai_model_path
            self.openai_api_key = api_key
            logger.info(f"✅ OpenAI API configured with model: {openai_model_path}")
        # Initialize OpenRouter if configured
        elif openrouter_model_path:
            api_key = os.getenv('OPENROUTER_API_KEY')
            if not api_key:
                logger.error("❌ OpenRouter model configured but OPENROUTER_API_KEY environment variable not set")
                raise ValueError("OPENROUTER_API_KEY environment variable required for OpenRouter")
            self.openrouter_model = openrouter_model_path
            self.openrouter_api_key = api_key
            self.openrouter_provider = openrouter_provider
            logger.info(f"✅ OpenRouter configured with model: {openrouter_model_path}")
        
        # Initialize vLLM model resolution if configured (only if OpenRouter not configured)
        elif vllm_model_path:
            try:
                import requests
                logger.info(f"🔍 Querying vLLM server at {vllm_base} for available models...")
                response = requests.get(f'{vllm_base}/v1/models', timeout=10)
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
                self.vllm_url = f'{vllm_base}/v1/chat/completions'
            except Exception as e:
                logger.error(f"❌ Failed to query vLLM server: {e}")
                raise  # Fail fast
        
        # Initialize alt LLM backend if alt_llm_config present (global alternate, toggled via UI)
        alt_llm_config = self.character_config.get('alt_llm_config', {})
        if alt_llm_config:
            alt_anthropic = alt_llm_config.get('anthropic_model_path')
            alt_openai = alt_llm_config.get('openai_model_path')
            alt_openrouter = alt_llm_config.get('openrouter_model_path')
            alt_openrouter_prov = alt_llm_config.get('openrouter_provider')
            alt_vllm = alt_llm_config.get('vllm_model_path')
            alt_vllm_url_raw = alt_llm_config.get('vllm_url', 'http://localhost:5000')
            alt_vllm_base = alt_vllm_url_raw.split('/v1')[0] if '/v1' in alt_vllm_url_raw else alt_vllm_url_raw.rstrip('/')
            if alt_anthropic:
                api_key = os.getenv('CLAUDE_API_KEY')
                if api_key:
                    self.alt_anthropic_model = alt_anthropic
                    self.alt_anthropic_api_key = api_key
                    logger.info(f"✅ Alt LLM (Anthropic) configured: {alt_anthropic}")
                else:
                    logger.warning("⚠️ alt_llm_config has anthropic_model_path but CLAUDE_API_KEY not set - alt LLM disabled")
            elif alt_openai:
                api_key = os.getenv('OPENAI_API_KEY')
                if api_key:
                    self.alt_openai_model = alt_openai
                    self.alt_openai_api_key = api_key
                    logger.info(f"✅ Alt LLM (OpenAI) configured: {alt_openai}")
                else:
                    logger.warning("⚠️ alt_llm_config has openai_model_path but OPENAI_API_KEY not set - alt LLM disabled")
            elif alt_openrouter:
                api_key = os.getenv('OPENROUTER_API_KEY')
                if api_key:
                    self.alt_openrouter_model = alt_openrouter
                    self.alt_openrouter_api_key = api_key
                    self.alt_openrouter_provider = alt_openrouter_prov
                    logger.info(f"✅ Alt LLM (OpenRouter) configured: {alt_openrouter}")
                else:
                    logger.warning("⚠️ alt_llm_config has openrouter_model_path but OPENROUTER_API_KEY not set - alt LLM disabled")
            elif alt_vllm:
                try:
                    import requests
                    response = requests.get(f'{alt_vllm_base}/v1/models', timeout=10)
                    response.raise_for_status()
                    data = response.json()
                    if data.get('data') and len(data['data']) > 0:
                        available = [m['id'] for m in data['data']]
                        self.alt_vllm_model = alt_vllm if alt_vllm in available else available[0]
                        self.alt_vllm_url = f'{alt_vllm_base}/v1/chat/completions'
                        logger.info(f"✅ Alt LLM (vLLM) configured: {self.alt_vllm_model}")
                    else:
                        logger.warning("⚠️ alt_llm_config vLLM: no models found - alt LLM disabled")
                except Exception as e:
                    logger.warning(f"⚠️ alt_llm_config vLLM init failed: {e} - alt LLM disabled")
        
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
                elif 'NVFP4' in sgl_model_path: # patch for FP8 models as of 1/9/2026
                    logger.info(f"🚀 Initializing SGLang Runtime with NVFP4 patch!")
                    self.runtime = sgl.Runtime(
                        model_path=sgl_model_path,
                        tokenizer_path=sgl_model_path,
                        device="cuda",
                        context_length=65536,
                        dtype="auto",
                        tp_size=1,
                        mem_fraction_static=0.9,
                        quantization="modelopt_fp4",
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
                        fp8_gemm_backend="triton",
                        attention_backend="triton"
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
        has_direct_llm_backend = any((
            self.anthropic_model and self.anthropic_api_key,
            self.openai_model and self.openai_api_key,
            self.openrouter_model and self.openrouter_api_key,
            self.vllm_model and self.vllm_url,
        ))
        if not self.runtime and not has_direct_llm_backend:
            self.llm_client = ZenohLLMClient(server_name=server_name, model_name=model_name, service_timeout=200.0 if not self.debug else 300.0)
            logger.info(f'🤖 LLM client initialized (server={server_name}, model={model_name})')
        
        # Create llm_generate wrapper function (unified interface for LLM calls)
        def llm_generate(messages, bindings=None, max_tokens=2000, temperature=0.7, is_json=False, stops=None):
            """Unified LLM generation interface - uses executor-backed providers when available, else ZenohLLMClient."""
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
                    _t0 = time.monotonic()
                    try:
                        return self.infospace_executor._sglang_generate(messages, max_tokens, temperature, stops, is_json)
                    finally:
                        self.infospace_executor.turn_metrics.record_llm("sglang", time.monotonic() - _t0)
                else:
                    logger.error("SGLang Runtime available but no infospace_executor to use it")
                    return type('Response', (), {'success': False, 'error': 'No executor', 'text': ''})()
            elif self.infospace_executor and any((
                self.infospace_executor.anthropic_model and self.infospace_executor.anthropic_api_key,
                self.infospace_executor.openai_model and self.infospace_executor.openai_api_key,
                self.infospace_executor.openrouter_model and self.infospace_executor.openrouter_api_key,
                self.infospace_executor.vllm_model and self.infospace_executor.vllm_url,
            )):
                return self.infospace_executor.llm_generate(
                    messages=messages,
                    bindings=bindings,
                    max_tokens=max_tokens,
                    temperature=temperature,
                    is_json=is_json,
                    stops=stops,
                )
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
        self.infospace_executor.extra_request_params = self.extra_request_params
        if self.openai_model and self.openai_api_key:
            self.infospace_executor.openai_model = self.openai_model
            self.infospace_executor.openai_api_key = self.openai_api_key
        if self.openrouter_model and self.openrouter_api_key:
            self.infospace_executor.openrouter_model = self.openrouter_model
            self.infospace_executor.openrouter_api_key = self.openrouter_api_key
            self.infospace_executor.openrouter_provider = self.openrouter_provider
        if self.anthropic_model and self.anthropic_api_key:
            self.infospace_executor.anthropic_model = self.anthropic_model
            self.infospace_executor.anthropic_api_key = self.anthropic_api_key
        if self.alt_openai_model and self.alt_openai_api_key:
            self.infospace_executor.alt_openai_model = self.alt_openai_model
            self.infospace_executor.alt_openai_api_key = self.alt_openai_api_key
        if self.alt_openrouter_model and self.alt_openrouter_api_key:
            self.infospace_executor.alt_openrouter_model = self.alt_openrouter_model
            self.infospace_executor.alt_openrouter_api_key = self.alt_openrouter_api_key
            self.infospace_executor.alt_openrouter_provider = self.alt_openrouter_provider
        if self.alt_vllm_model and self.alt_vllm_url:
            self.infospace_executor.alt_vllm_model = self.alt_vllm_model
            self.infospace_executor.alt_vllm_url = self.alt_vllm_url
        if self.alt_anthropic_model and self.alt_anthropic_api_key:
            self.infospace_executor.alt_anthropic_model = self.alt_anthropic_model
            self.infospace_executor.alt_anthropic_api_key = self.alt_anthropic_api_key
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
        
        # ToolModel disabled — tool contracts in WorldModel serve this role
        self.tool_model = None

        # Attach models to executor for access by planner
        self.infospace_executor.world_model = self.world_model
        self.infospace_executor.tool_model = None
        
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
        self.conversation_store._archive_callback = self._archive_dialog
        self._purge_untagged_history_notes()

        from user_concern_model import UserConcernModel
        self.user_concern_model = UserConcernModel(
            resource_manager=self.resource_manager,
            character_name=self.character_name,
            llm_generate=llm_generate,
            infospace_executor=self.infospace_executor,
        )
        self.user_concern_model.load()

        from ooda_living_state import OodaLivingState
        self._ooda_living_state = OodaLivingState()
        self._ooda_living_state.load(self.infospace_executor)

        from derived_concern_model import DerivedConcernModel
        self._derived_concern_model = DerivedConcernModel(
            resource_manager=self.resource_manager,
            character_name=self.character_name,
            llm_generate=llm_generate,
            infospace_executor=self.infospace_executor,
        )
        self._derived_concern_model.set_seed_concerns(
            self.character_config.get('seed_concerns', []))
        self._derived_concern_model.load()

        from concern_triage import ConcernTriage
        self._concern_triage = ConcernTriage()

        # ── Cognitive Memory Graph ──────────────────────────────────────
        from cognitive_graph import CognitiveGraph
        self._cognitive_graph = CognitiveGraph(
            embedder=self.resource_manager._generate_embedding if self.resource_manager else None,
        )
        self._graph_node_by_key: Dict[str, str] = {}  # "goal:<id>" / "concern_change:<id>" etc.
        # Option B stash for tight OODA chain (same tick)
        self._last_event_node: Optional[str] = None
        self._last_assessment_node: Optional[str] = None
        self._last_decision_node: Optional[str] = None
        # Load persisted graph if available
        try:
            base = self.resource_manager.base_dir if self.resource_manager else None
            if base:
                self._cognitive_graph_path = str(Path(base) / "cognitive_graph")
                self._cognitive_graph.load(self._cognitive_graph_path)
            else:
                self._cognitive_graph_path = None
        except Exception as e:
            logger.warning(f"Failed to load cognitive graph: {e}")
            self._cognitive_graph_path = None

        self._last_graph_maintenance: float = 0.0
        self._graph_maintenance_interval: float = 300.0  # 5 minutes

        # ── Entity Index (NER-backed entity tracking) ─────────────────
        from entity_index import EntityIndex
        self._entity_index = EntityIndex(cognitive_graph=self._cognitive_graph)
        # Seed common aliases
        self._entity_index.add_aliases({
            "the user": "user", "bruce": "user", "bdambrosio": "user",
            self.character_name.lower(): self.character_name.lower(),
        })
        # Load persisted entity index if available
        try:
            if self._cognitive_graph_path:
                ei_path = str(Path(self._cognitive_graph_path).parent / "entity_index.json")
                import json as _json
                with open(ei_path) as f:
                    self._entity_index.load_dict(_json.load(f))
                logger.info(f"✓ Loaded entity index ({len(self._entity_index.get_all_entities())} entities)")
        except FileNotFoundError:
            pass
        except Exception as e:
            logger.debug(f"Entity index load skipped: {e}")
        # Expose entity index to executor for search_resources augmentation
        if self.infospace_executor:
            self.infospace_executor._entity_index = self._entity_index

        self._scheduled_goal_counter = 0
        self._active_scheduled_goal_id = None
        self._initialize_scheduled_goals()

        # Initialize task scheduler (auto-proceed timer)
        from goal_scheduler import GoalScheduler
        sched_cfg = self.character_config.get('goal_scheduler', self.character_config.get('task_scheduler', {}))
        interval_min = sched_cfg.get('interval', 15)  # minutes (consistent with UI)
        autonomy_cfg = self.character_config.get('autonomy', {})
        self.goal_scheduler = GoalScheduler(
            character_name=self.character_name,
            interval=float(interval_min) * 60.0,  # convert to seconds
            enabled=sched_cfg.get('enabled', False),
            budget_minutes=float(autonomy_cfg.get('budget_minutes_per_hour', 15)),
            budget_window_minutes=float(autonomy_cfg.get('budget_window_minutes', 60)),
        )
        self.goal_scheduler.start(
            check_fn=self._scheduler_eligible_goals,
            proceed_fn=self._scheduler_proceed_goal,
        )
        self._scheduler_events = []
        self._scheduler_event_limit = 40
        self._scheduler_event_keys = set()
        self._scheduler_event_lock = threading.Lock()
        self._scheduler_started_goals = set()

        # Initialize planners (reused across all goals)
 
        # IncrementalPlanner for plan generation (SGLang or vLLM-based)
        self.incremental_planner = None
        try:
            from incremental_planner import IncrementalPlanner, HAS_SGLANG
            
            # Get LLM config
            llm_config = self.character_config.get('llm_config', {})
            sgl_model_path = llm_config.get('sgl_model_path')
            vllm_model_path = llm_config.get('vllm_model_path')
            openai_model_path = llm_config.get('openai_model_path')
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
                # Use vLLM — self.vllm_url already normalized to full endpoint
                self.incremental_planner = IncrementalPlanner(
                    executor=self.infospace_executor,
                    available_tools=self.available_tools,
                    logger_instance=logger,
                    vllm_model_path=vllm_model_path,
                    vllm_url=self.vllm_url,
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
            elif openai_model_path and self.openai_model:
                # Use OpenAI API
                self.incremental_planner = IncrementalPlanner(
                    executor=self.infospace_executor,
                    available_tools=self.available_tools,
                    logger_instance=logger,
                    openai_model_path=openai_model_path
                )
                logger.info(f'🚀 IncrementalPlanner initialized (OpenAI API) for {character_name}')
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
                logger.warning(f'⚠️  Neither SGLang, vLLM, Anthropic, OpenAI, nor OpenRouter configured - incremental planning disabled')
        except Exception as e:
            logger.warning(f'⚠️  Failed to initialize IncrementalPlanner: {e}')
            import traceback
            traceback.print_exc()
        
        # Internal state (action_counter already initialized above, before executor creation)
        self.last_sense_data = None
        
        # OODA loop state
        self.current_goal = None
        self.goal_source = None  # Track goal origin: 'ui' or None
        self.awaiting_user_input = False
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
        self._ask_response_queue: queue.Queue = queue.Queue()

        # Task-in-progress (milestone loop) state
        self.active_task_wip: Optional[str] = None       # Note name e.g. "_task_wip_1"
        self.active_task_wip_waiting: bool = False        # True while a milestone goal runs
        self._task_wip_counter: int = 0
        self._resync_task_wip_counter()

        # Operational task execution state
        self._operational_task_note: Optional[str] = None  # Note name of task with running goal
        self._operational_goal_waiting: bool = False        # True while operational goal runs

        # Goal worker thread state
        self._goal_thread: Optional[threading.Thread] = None
        self._goal_thread_result: Optional[Dict[str, Any]] = None
        self._goal_done_event = threading.Event()

        # Sensor configuration summary (enriched by launcher before agent start)
        self.sensor_configs: list = self.character_config.get('_sensor_configs', [])
        
        # Last character evaluator assessment (for orientation-to-chat integration)
        self._last_character_eval: Optional[Dict[str, Any]] = None
        self._character_concern_activations: Dict[str, float] = {}

        # Periodic orientation timer (30s wall-clock)
        self._orient_timer_interval: float = 30.0
        self._last_orient_timer: float = time.monotonic()
        self._non_timer_event_since_last_orient: bool = True  # True initially to force first eval

        # Tier 2 Decide: proactive remark suppression
        self._last_proactive_remark_at: float = 0.0
        self._proactive_remark_cooldown: float = 180.0  # seconds

        # OODA event feed ring buffer for UI
        self._ooda_event_feed: List[Dict[str, Any]] = []
        self._OODA_FEED_MAX = 30

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
        self.execution_mode = 'step'  # 'step' or 'run'
        self.interrupt_requested = False  # Global interrupt flag (checked once per planner step)
        
        # Subscribers for direct execution control from UI
        self.control_llm_toggle_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/control/llm_toggle",
            self.handle_llm_toggle
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
            f"cognitive/{character_name}/control/goal_scheduler",
            self._handle_scheduler_control
        )
        self.control_goal_schedule_mode_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/control/goal_schedule_mode",
            self._handle_goal_schedule_mode
        )
        self.control_goal_rename_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/control/goal_rename",
            self._handle_goal_rename
        )
        self.control_goal_execution_mode_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/control/goal_execution_mode",
            self._handle_goal_execution_mode
        )
        self.control_goal_text_update_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/control/goal_text_update",
            self._handle_goal_text_update
        )
        self.control_goal_cache_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/control/goal_cache",
            self._handle_goal_cache
        )
        self.control_goal_interrupt_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/control/goal_interrupt",
            self._handle_goal_interrupt
        )
        self.control_goal_remove_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/control/goal_remove",
            self._handle_goal_remove
        )

        self.control_task_wip_delete_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/control/task_wip_delete",
            self._handle_task_wip_delete
        )

        self.control_task_wip_interrupt_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/control/task_wip_interrupt",
            self._handle_task_wip_interrupt
        )

        # Task management control (for task manager UI)
        self.control_task_approve_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/control/task_approve",
            self._handle_task_approve
        )
        self.control_task_abandon_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/control/task_abandon",
            self._handle_task_abandon
        )
        self.control_task_edit_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/control/task_edit",
            self._handle_task_edit
        )
        self.control_task_cooldown_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/control/task_cooldown",
            self._handle_task_cooldown
        )
        self.control_task_run_now_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/control/task_run_now",
            self._handle_task_run_now
        )
        self.control_concern_manage_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/control/concern_manage",
            self._handle_concern_manage
        )

        # ── Command registry ─────────────────────────────────────────────
        # The single manifest of all imperative operations in the system.
        # Every /command maps to a handler that accepts a plain dict.
        self._command_registry = self._build_command_registry()

        # Unified command channel — all clients (CLI, web UI, scheduler)
        # send structured commands here instead of raw text.
        self.command_subscriber = self.session.declare_subscriber(
            f"cognitive/{character_name}/command",
            self._handle_command_message
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
        
        self.resource_update_queryable = self.session.declare_queryable(
            f"cognitive/{character_name}/resource/update/*",
            self._handle_resource_update_query
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
        
        self.scheduled_goals_queryable = self.session.declare_queryable(
            f"cognitive/{character_name}/scheduled_goals",
            self._scheduled_goals_query_handler
        )

        self.task_wips_queryable = self.session.declare_queryable(
            f"cognitive/{character_name}/task_wips",
            self._task_wips_query_handler
        )

        self.concerns_queryable = self.session.declare_queryable(
            f"cognitive/{character_name}/concerns",
            self._concerns_query_handler
        )

        self.triage_status_queryable = self.session.declare_queryable(
            f"cognitive/{character_name}/triage_status",
            self._triage_status_query_handler
        )

        self.ooda_feed_queryable = self.session.declare_queryable(
            f"cognitive/{character_name}/ooda_feed",
            self._ooda_feed_query_handler
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
        # Queryables for cognitive graph exploration (for resource_browser graph tab)
        self.graph_subgraph_queryable = self.session.declare_queryable(
            f"cognitive/{character_name}/graph/subgraph",
            self._handle_graph_subgraph_query
        )
        self.graph_entities_queryable = self.session.declare_queryable(
            f"cognitive/{character_name}/graph/entities",
            self._handle_graph_entities_query
        )

        # Shutdown flags
        self.shutdown_requested = False
        self._shutting_down = False

        logger.info(f'🧠 Zenoh Executive Node initialized for character: {character_name}')
        logger.info(f'   - Subscribing to: cognitive/{character_name}/sense_data')
        logger.info(f'   - Subscribing to: cognitive/{character_name}/control/llm_toggle')
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
        """Handle save_all command - save all persistent state."""
        try:
            logger.info(f'💾 {self.character_name} received save_all command')
            if self.resource_manager:
                self.resource_manager.save_to_file()
                logger.info(f'💾 Saved resource manager state for {self.map_name}')
            if hasattr(self, 'world_model') and self.world_model:
                self.world_model.save()
                logger.info(f'💾 Saved world model for {self.character_name}')
        except Exception as e:
            logger.error(f'Error handling save command: {e}')

    def _handle_close_dialog(self, sample):
        """Handle close_dialog from UI (End conversation button).

        The archive callback (synthesize + concern model) is heavyweight and must
        NOT block the zenoh subscriber thread — otherwise subsequent zenoh
        callbacks (sense_data, etc.) are starved.  We therefore schedule archiving
        on a background thread and only do the lightweight bookkeeping here.
        """
        try:
            payload_bytes = sample.payload.to_bytes()
            data = json.loads(payload_bytes.decode('utf-8'))
            entity_name = data.get('entity_name', 'User')

            # Lightweight bookkeeping (fast, safe on zenoh thread)
            self._dialog_cooldowns[entity_name] = time.time()
            self._dialog_purposes.pop(entity_name, None)
            # End-dialog should immediately terminate any in-progress ask/wait.
            self.interrupt_requested = True
            if self.infospace_executor:
                self.infospace_executor.interrupt_requested = True
            if self.awaiting_ask_response:
                self.awaiting_ask_response = False
                self._ask_response_queue.put(None)  # Sentinel to unblock _execute_ask

            # Schedule close_dialog (which triggers archive) off the zenoh thread
            def _close_and_archive():
                try:
                    self.conversation_store.close_dialog(entity_name)
                except Exception as e:
                    logger.warning(f'Error in deferred close_dialog: {e}')
                finally:
                    logger.info(f'🔚 {self.character_name} closed dialog with {entity_name} (from UI)')
                    self._publish_execution_state()

            t = threading.Thread(target=_close_and_archive, daemon=True, name='close-dialog')
            t.start()
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
    
    def _start_sensors(self):
        """Spawn sensor threads owned by this agent.

        Called once from run() after core init is complete. Each sensor runs
        as a daemon thread sharing this agent's resource_manager,
        available_tools, and Zenoh session — so the rss-watcher (and any
        other resource-touching sensor) can write Notes directly without
        going through Zenoh or worrying about None references.

        Sensor metadata (the dict loaded by the launcher from src/sensors/)
        is passed in via character_config['_sensor_metadata_by_name']. The
        per-character sensor list is in character_config['sensors']. Both
        are populated in launcher.py before agent threads start.
        """
        all_sensors = self.character_config.get('_sensor_metadata_by_name') or {}
        char_sensors = self.character_config.get('sensors', []) or []
        if not all_sensors or not char_sensors:
            return

        try:
            from sensor_runner import SensorRunner
            from utils.sensor_loader import parse_schedule
        except ImportError as e:
            logger.warning(f"Sensor module import failed (non-fatal): {e}")
            return

        if not hasattr(self, '_sensor_threads'):
            self._sensor_threads = []

        # Use the launcher-provided shutdown event so the launcher's SIGINT
        # / SIGTERM handler stops sensors as well as the agent. If the agent
        # is running standalone (no launcher), fall back to a fresh event so
        # sensors at least have something to wait on.
        shutdown_event = getattr(self, '_shutdown_event', None)
        if shutdown_event is None:
            shutdown_event = threading.Event()

        for sensor_cfg in char_sensors:
            s_name = sensor_cfg.get('name', '')
            if not s_name:
                continue
            if s_name not in all_sensors:
                logger.warning(f"Sensor '{s_name}' declared for {self.character_name} but not found in src/sensors/")
                continue
            sensor_meta = all_sensors[s_name]

            # Build per-character overrides (mirrors what launcher.py used to do)
            overrides: Dict[str, Any] = {}
            if 'schedule' in sensor_cfg:
                secs = parse_schedule(sensor_cfg['schedule'])
                if secs is not None:
                    overrides['schedule_seconds'] = secs
            if 'parameters' in sensor_cfg:
                overrides['parameters'] = sensor_cfg['parameters']
            if 'gate' in sensor_cfg:
                overrides['gate'] = sensor_cfg['gate']
            if 'disposition' in sensor_cfg:
                overrides['disposition'] = sensor_cfg['disposition']

            try:
                runner = SensorRunner(
                    sensor_name=s_name,
                    sensor_meta=sensor_meta,
                    character_name=self.character_name,
                    scenario_overrides=overrides,
                    resource_manager=self.resource_manager,
                    zenoh_session=self.session,
                    available_tools=self.available_tools or {},
                    shutdown_event=shutdown_event,
                )
                t = threading.Thread(
                    target=runner.run,
                    name=f"sensor-{self.character_name}-{s_name}",
                    daemon=True,
                )
                t.start()
                self._sensor_threads.append(t)
                logger.info(f"Started sensor {s_name} for {self.character_name}")
            except Exception as e:
                logger.warning(f"Failed to start sensor {s_name} for {self.character_name}: {e}")
                traceback.print_exc()

        if self._sensor_threads:
            logger.info(f"{self.character_name}: started {len(self._sensor_threads)} sensor thread(s)")

    def run(self):
        """Main OODA loop."""
        try:
            logger.info('Executive Node running - press Ctrl+C to stop')

            # Announce character presence
            self._announce_character()
            time.sleep(0.1)
            time.sleep(1.0)

            # Recover orphaned establishing tasks from prior session
            self._recover_stalled_establishing_tasks()

            # Start sensor threads now that core OODA infrastructure (queues,
            # resource_manager, infospace_executor, available_tools) is fully
            # initialized. Sensors run as daemon threads sharing this agent's
            # resource_manager directly — no race, no IPC, no None reference.
            self._start_sensors()

            # Start main loop
            while not self.shutdown_requested:
                try:
                    self._main_loop_tick()
                except Exception as e:
                    traceback.print_exc()
                    logger.error(f'Error in main loop: {e}')
                time.sleep(0.2)
                
        except KeyboardInterrupt:
            logger.info('Executive Node shutting down...')
        finally:
            self.shutdown()
    
    def _main_loop_tick(self):
        """One iteration of the main loop.

        Structure:
          1. Goal completion check
          2. Task establishment advancement (milestone loop)
          3. Operational task execution dispatch (round-robin)
          4. Ask reply routing
          5. Skip if goal running
          6. OODA pipeline (only when truly idle)
        """
        # ── 1. Goal completion (thread join check) ───────────────────────
        if self._goal_done_event.is_set() and not self._is_goal_running():
            self._goal_done_event.clear()
            logger.info(f'✅ {self.character_name} goal thread completed')
            # Record result for operational task goal
            if self._operational_goal_waiting and self._operational_task_note:
                self._record_operational_goal_result()
                self._operational_goal_waiting = False
            # Clear scheduler executing_goal_id so /status doesn't show a phantom goal
            if hasattr(self, 'goal_scheduler') and self.goal_scheduler._executing_goal_id is not None:
                self.goal_scheduler._executing_goal_id = None
            self.execution_paused = True
            self.execution_mode = 'step'
            self._publish_execution_state()

        # ── 2. Task establishment advancement (single-threaded) ──────────
        if self.active_task_wip and not self.active_task_wip_waiting and not self._is_goal_running():
            self._advance_task_wip()

        # ── 3. Operational task dispatch (round-robin, if no establishment) ─
        if (not self.active_task_wip
                and not self._is_goal_running()
                and not self._operational_goal_waiting):
            # If a task was mid-cycle and its goal just completed, advance it
            if self._operational_task_note:
                self._advance_task_execution(self._operational_task_note)
            else:
                # Select next eligible task (round-robin by staleness)
                task = self._select_next_task()
                if task:
                    self._advance_task_execution(task.get('_note_name', ''))

        # ── 4. Route ask replies while goal thread is blocked on _execute_ask
        if self.awaiting_ask_response and self.text_input_queue:
            logger.debug(f'Ask routing: {len(self.text_input_queue)} items in queue, scanning for User reply')
            for i, sense_data in enumerate(self.text_input_queue):
                content = sense_data.get('content', '')
                try:
                    d = json.loads(content)
                    text, source = d.get('text', ''), d.get('source', 'unknown')
                except (json.JSONDecodeError, TypeError):
                    text, source = content, 'console'
                logger.debug(f'Ask routing: item {i} source={source} text={text[:60] if text else ""}')
                if text and text.strip() and source == 'User':
                    self.text_input_queue.pop(i)
                    self._ask_response_queue.put(text)
                    logger.info(f'✅ Ask reply routed: "{text[:60]}"')
                    break
        elif self.awaiting_ask_response:
            logger.debug(f'Ask routing: awaiting response but queue empty')

        # ── 5. Skip event processing while goal is running ──────────────
        if self._is_goal_running():
            return

        # ── 6. OODA pipeline (only when truly idle) ─────────────────────
        event = self._ooda_observe()
        if event is None:
            self._ooda_idle_tick()
            return

        # Reset per-turn metrics for this OODA cycle
        if self.infospace_executor:
            self.infospace_executor.turn_metrics = TurnMetrics()

        # ── Graph: observe ──
        self._graph_emit_observe(event)
        self._ooda_living_state.update_after_observe(event)
        oriented = self._ooda_orient(event)
        # Build concern descriptions map for living state content
        _concern_descs = {}
        try:
            for c in character_evaluator.DEFAULT_CHARACTER_CONCERNS:
                _concern_descs[c["id"]] = f"{c.get('label', '')} — {c.get('description', '')}"
            for c in self._derived_concern_model.get_concerns(active_only=True):
                _concern_descs[c.get("concern_id", "")] = (
                    f"{c.get('concern_label', '')} — {c.get('concern_description', '')}"
                )
        except Exception:
            pass
        self._ooda_living_state.update_after_orient(
            oriented, self._character_concern_activations, _concern_descs)
        # Refresh goals and user concerns in living state
        try:
            self._ooda_living_state.update_goals(
                self._all_scheduled_goals(), self._active_scheduled_goal_id)
            self._ooda_living_state.update_user_concerns_snapshot(
                self.user_concern_model.get_concerns() or [])
        except Exception:
            pass
        action = self._ooda_decide(oriented)
        # ── Graph: decide ──
        self._graph_emit_decide(action)
        # Record OODA event for UI feed
        self._record_ooda_event(event, oriented, action)
        self._ooda_act(action)
        self._ooda_living_state.update_after_act(action)

        # Emit per-turn latency summary (non-goal turns; goal turns emit in parse_and_set_goal)
        # Suppress CLI echo for timer ticks — log only.
        if self.infospace_executor and action.type != 'dispatch_goal':
            if self.infospace_executor.turn_metrics.llm_calls:
                _perf = self.infospace_executor.turn_metrics.summary()
                logger.info(_perf)
                if event.classification != 'timer':
                    print(_perf, flush=True)
        self._ooda_living_state.maybe_persist(
            self._write_named_note, self._derived_concern_model.get_concerns(),
            planner_summary=self._get_planner_summary())

    def _ooda_idle_tick(self) -> None:
        """Directed idle behavior: derived concern maintenance, triage, and living state persistence."""
        if not self._is_goal_running():
            try:
                uc = self.user_concern_model.get_concerns(active_only=True)
                self._derived_concern_model.update_from_idle_tick(
                    self._ooda_living_state, uc)
            except Exception as e:
                logger.debug(f"Idle tick derived concern update skipped: {e}")
            # Check for satisfied concerns whose revisit period has expired
            try:
                self._derived_concern_model.check_revisit_expirations()
            except Exception as e:
                logger.debug(f"Revisit expiration check skipped: {e}")
            # Concern triage: tick deferred timers, nominate from activations, run triage
            self._triage_idle_tick()
        try:
            dc = self._derived_concern_model.get_concerns()
        except Exception:
            dc = []
        self._ooda_living_state.maybe_persist(
            self._write_named_note, dc,
            planner_summary=self._get_planner_summary())
        # ── Graph: consolidation + periodic save (throttled) ──
        now = time.monotonic()
        try:
            if (now - self._last_graph_maintenance) >= self._graph_maintenance_interval:
                self._last_graph_maintenance = now
                if not self._is_goal_running():
                    self._cognitive_graph.consolidate()
                    # Clean up entity index for orphaned entities removed during consolidation
                    orphaned = getattr(self._cognitive_graph, '_orphaned_entities', [])
                    if orphaned:
                        for _, canonical in orphaned:
                            if canonical:
                                self._entity_index._entity_nodes.pop(canonical, None)
                        logger.info(f"🏷 Cleaned {len(orphaned)} orphaned entities from entity index")
                if self._cognitive_graph_path:
                    self._cognitive_graph.save(self._cognitive_graph_path)
        except Exception as e:
            logger.debug(f"Cognitive graph maintenance failed: {e}")
        # ── Entity index: process unextracted persistent notes ──
        try:
            if not self._is_goal_running():
                self._entity_index_process_persistent_notes()
                self._save_entity_index()
        except Exception as e:
            logger.debug(f"Entity index maintenance failed: {e}")

    def _resync_task_wip_counter(self):
        """Scan existing _task_wip_N named notes and set counter to max N found."""
        if not self.resource_manager:
            return
        max_n = 0
        for name in self.resource_manager.named_notes:
            if name.startswith('_task_wip_'):
                try:
                    n = int(name[len('_task_wip_'):])
                    max_n = max(max_n, n)
                except (ValueError, IndexError):
                    continue
        if max_n > self._task_wip_counter:
            self._task_wip_counter = max_n
            logger.info(f'Resynced _task_wip_counter to {max_n}')

    def _build_serviced_concern_ids(self) -> set:
        """Build set of concern IDs that have linked tasks (any non-terminal state)."""
        task_data = self._get_all_task_data()
        serviced = set()
        for t in task_data:
            if t.get('status') in ('abandoned', 'archived'):
                continue
            cid = t.get('linked_concern_id')
            if cid:
                serviced.add(cid)
            for cid in t.get('linked_concern_ids', []):
                serviced.add(cid)
        return serviced

    def _triage_idle_tick(self):
        """Concern triage integration: nominate from activations, tick deferrals, run triage."""
        self._concern_triage.tick_deferred()

        # H3: Timeout delegated concerns that never received a callback
        try:
            _DELEGATION_TIMEOUT_HOURS = 4
            _now = datetime.now()
            for c in self._derived_concern_model.get_concerns():
                if c.get('status') != 'delegated':
                    continue
                delegated_at = c.get('delegated_at')
                if not delegated_at:
                    continue
                elapsed_h = (_now - datetime.fromisoformat(delegated_at)).total_seconds() / 3600
                if elapsed_h >= _DELEGATION_TIMEOUT_HOURS:
                    c['status'] = 'active'
                    c['status_rationale'] = f'Delegation timed out after {elapsed_h:.0f}h — reactivated'
                    c['recency'] = _now.isoformat()
                    self._derived_concern_model._save()
                    logger.info(f'📋 Concern {c.get("concern_id")} delegation timed out, reactivated')
        except Exception as e:
            logger.debug(f'Delegation timeout check skipped: {e}')

        serviced_ids = self._build_serviced_concern_ids()

        # Activation-monitor nominations: check living state concern activations
        try:
            active_concerns = self._derived_concern_model.get_concerns(active_only=True)
            for ca in self._ooda_living_state.concern_activations:
                cid = ca.get('id', '')
                if cid in serviced_ids:
                    continue
                # Only nominate derived concerns (not built-in character concerns)
                concern = next((c for c in active_concerns if c.get('concern_id') == cid), None)
                if not concern:
                    continue
                _act = ca.get('activation', 0.0)
                _lbl = concern.get('concern_label', cid)
                self._concern_triage.nominate_from_activation(
                    concern_id=cid,
                    concern_label=_lbl,
                    concern_description=concern.get('concern_description', ''),
                    activation=_act,
                    trend=ca.get('trend', 'stable'),
                )
        except Exception as e:
            logger.debug(f'Triage activation nomination skipped: {e}')

        # Run triage if candidates are queued and cooldown has elapsed
        if self._concern_triage.should_run_triage():
            try:
                task_data = self._get_all_task_data()
                # Filter to non-terminal tasks for deduplication context
                live_tasks = [t for t in task_data
                              if t.get('status') in ('proposed', 'in_progress', 'establishing', 'active')]
                context = self._build_triage_context()
                decisions = self._concern_triage.run_triage(
                    existing_tasks=live_tasks,
                    agent_context=context,
                    llm_generate=self.llm_generate,
                )
                # self._handle_triage_decisions(decisions)  # disabled: concern→task initiation
            except Exception as e:
                logger.warning(f'Triage execution failed: {e}', exc_info=True)

    def _build_triage_context(self) -> str:
        """Build brief agent context string for the triage LLM prompt."""
        parts = []
        if self._is_goal_running():
            parts.append('Currently executing a goal.')
        else:
            parts.append('Idle — no goal running.')
        try:
            goals = self._all_scheduled_goals()
            active = [g for g in goals if g.get('status') == 'running']
            ready = [g for g in goals if g.get('status') == 'ready']
            parts.append(f'{len(active)} running, {len(ready)} ready goals.')
        except Exception:
            pass
        try:
            uc = self.user_concern_model.get_concerns(active_only=True) or []
            if uc:
                concern_details = []
                for c in uc[:5]:
                    label = c.get('concern_label', '?')
                    desc = c.get('concern_description', '')
                    concern_details.append(f'- {label}: {desc}' if desc else f'- {label}')
                parts.append('Active user concerns:\n' + '\n'.join(concern_details))
        except Exception:
            pass
        # M1: Include derived concern state so triage can see delegated/satisfied concerns
        try:
            dc = self._derived_concern_model.get_concerns() or []
            if dc:
                dc_lines = []
                for c in dc[:8]:
                    cid = c.get('concern_id', '?')
                    label = c.get('concern_label', '?')
                    status = c.get('status', '?')
                    linked = c.get('linked_task', '')
                    line = f'- {cid} [{status}] {label}'
                    if linked:
                        line += f' (task: {linked})'
                    if status == 'delegated':
                        line += f' (delegated to {c.get("delegated_to", "?")})'
                    dc_lines.append(line)
                parts.append('Derived concerns:\n' + '\n'.join(dc_lines))
        except Exception:
            pass
        return ' '.join(parts)

    # Name of the background agent that receives delegated concern tasks.
    # Set to None to disable delegation and fall back to local task creation.
    DELEGATE_AGENT = 'jill-offline'

    def _handle_triage_decisions(self, decisions):
        """Process triage decisions: dispatch to background agent or attach to existing task."""
        from concern_triage import TriageDecision
        for d in decisions:
            # ── Graph: triage decision ──
            self._graph_emit_triage_decision(d)
            if d.action == 'create_task':
                if self.DELEGATE_AGENT:
                    self._dispatch_to_delegate(d.concern_id, d.task_intention, d.reason)
                else:
                    self._create_proposed_task(d.concern_id, d.task_intention, d.reason)
            elif d.action == 'attach_to_task':
                self._attach_concern_to_task(d.concern_id, d.existing_task_id)
            # defer and dismiss are handled by ConcernTriage internally

    def _dispatch_to_delegate(self, concern_id: str, intention: str, reason: str):
        """Dispatch a concern-initiated task to the background agent."""
        delegate = self.DELEGATE_AGENT
        # Mark the concern as delegated (prevents triage re-dispatch)
        self._derived_concern_model.mark_delegated(concern_id, delegate)

        # Send /goal add command to the delegate agent's command channel
        callback_topic = f"cognitive/{self.character_name}/sense_data"
        command = {
            "cmd": "/goal add",
            "goal_text": intention,
            "source": self.character_name,
            "source_agent": self.character_name,
            "callback_topic": callback_topic,
            "callback_concern_id": concern_id,
        }
        try:
            self.session.put(
                f"cognitive/{delegate}/command",
                json.dumps(command).encode('utf-8'),
            )
            logger.info(f'📋 Dispatched concern task to {delegate}: "{intention[:80]}" (concern: {concern_id})')
        except Exception as e:
            logger.error(f'Failed to dispatch to {delegate}: {e}')
            # Revert concern to active so triage can retry
            concern = next((c for c in self._derived_concern_model.concerns
                            if c.get('concern_id') == concern_id), None)
            if concern:
                concern['status'] = 'active'
                concern['status_rationale'] = f'Delegation to {delegate} failed: {e}'
                self._derived_concern_model._save()
            return

        self._say_to_user(
            f"Delegated to {delegate}: {intention[:200]}\n"
            f"Reason: {reason}"
        )

    def _create_proposed_task(self, concern_id: str, intention: str, reason: str):
        """Create a proposed task from a triage decision. Awaits user approval."""
        self._task_wip_counter += 1
        wip_id = f"twip_{self._task_wip_counter}"
        note_name = f"_task_wip_{self._task_wip_counter}"
        now = datetime.now().isoformat()
        wip_content = {
            "task_wip_id": wip_id,
            "intention": intention,
            "status": "proposed",
            "phase": "proposed",
            "milestones_completed": [],
            "current_milestone": None,
            "accumulated_findings": [],
            "linked_concern_id": concern_id,
            "proposal_reason": reason,
            "created": now,
            "updated": now,
        }
        self.infospace_executor.execute_action({
            "type": "create-note",
            "value": json.dumps(wip_content),
            "name": note_name,
            "out": f"${note_name}",
        })
        self.infospace_executor.execute_action({
            "type": "persist",
            "target": f"${note_name}",
            "name": note_name,
        })
        # ── Graph: task created ──
        self._graph_emit_task_created(wip_id, intention, concern_id)
        logger.info(f'📋 Proposed task created: {note_name} — "{intention[:80]}" (concern: {concern_id})')
        self._say_to_user(
            f"Task proposed: {intention[:200]}\n"
            f"Reason: {reason}\n"
            f"Awaiting approval in Task Manager."
        )

    def _attach_concern_to_task(self, concern_id: str, task_wip_id: str):
        """Link a concern to an existing task."""
        if not self.resource_manager:
            return
        try:
            # Find the task WIP note by scanning named notes
            for name, note_id in list(self.resource_manager.named_notes.items()):
                if not name.startswith('_task_wip_'):
                    continue
                note_data = self.resource_manager.resource_registry.get(note_id)
                if not note_data:
                    continue
                content_str = note_data.get('properties', {}).get('content', '')
                if not content_str:
                    continue
                try:
                    t = json.loads(content_str)
                except (json.JSONDecodeError, TypeError):
                    continue
                if t.get('task_wip_id') != task_wip_id:
                    continue
                # Found the matching task — update linked concerns
                linked = t.get('linked_concern_ids', [])
                if t.get('linked_concern_id') and t['linked_concern_id'] not in linked:
                    linked = [t['linked_concern_id']] + linked
                if concern_id not in linked:
                    linked.append(concern_id)
                t['linked_concern_ids'] = linked
                t['updated'] = datetime.now().isoformat()
                self.infospace_executor.execute_action({
                    "type": "create-note",
                    "value": json.dumps(t),
                    "name": name,
                    "out": f"${name}",
                })
                self.infospace_executor.execute_action({
                    "type": "persist",
                    "target": f"${name}",
                    "name": name,
                })
                logger.info(f'📋 Concern {concern_id} attached to task {task_wip_id}')
                return
            logger.warning(f'Triage: task {task_wip_id} not found for concern attachment')
        except Exception as e:
            logger.warning(f'Triage: failed to attach concern {concern_id} to task {task_wip_id}: {e}')

    def _recover_stalled_establishing_tasks(self):
        """On startup, find task WIPs stuck in establishing (in_progress) and mark interrupted.

        After a restart, active_task_wip is None but task notes may still have
        status=in_progress from a prior session.  Mark them interrupted so they
        appear correctly in the UI and don't block new approvals.
        """
        if not self.resource_manager:
            return
        recovered = 0
        for name, note_id in list(self.resource_manager.named_notes.items()):
            if not name.startswith("_task_wip_"):
                continue
            try:
                res = self.resource_manager.get_resource(note_id)
                content = json.loads(res.get('properties', {}).get('content', '{}'))
                if content.get('status') == 'in_progress':
                    content['status'] = 'interrupted'
                    content['updated'] = datetime.now().isoformat()
                    content.setdefault('accumulated_findings', []).append(
                        'Interrupted: task was mid-establishment when system restarted')
                    self.infospace_executor.execute_action({
                        "type": "create-note",
                        "value": json.dumps(content),
                        "name": name,
                        "out": f"${name}",
                    })
                    self.infospace_executor.execute_action({
                        "type": "persist",
                        "target": f"${name}",
                        "name": name,
                    })
                    recovered += 1
                    logger.info(f'📋 Recovered stalled task {name}: marked interrupted')
            except Exception as e:
                logger.warning(f'Task recovery: error processing {name}: {e}')
        if recovered:
            logger.info(f'📋 Task recovery: {recovered} stalled establishing task(s) marked interrupted')

    def _approve_proposed_task(self, note_name: str, edited_intention: str = ''):
        """Approve a proposed task, transitioning it directly to operational state.

        Acquisition and establishment phases are skipped — the task intention
        (from triage or user edit) serves as the comprehensive spec.
        Called from the task manager UI via Zenoh control endpoint.
        """
        if not self.resource_manager:
            return
        note_id = self.resource_manager.named_notes.get(note_name)
        if not note_id:
            logger.warning(f'Task approve: note {note_name} not found')
            return
        try:
            res = self.resource_manager.get_resource(note_id)
            content = json.loads(res.get('properties', {}).get('content', '{}'))
        except Exception as e:
            logger.warning(f'Task approve: could not read {note_name}: {e}')
            return
        if content.get('status') not in ('proposed', 'interrupted'):
            logger.warning(f'Task approve: {note_name} is not in proposed/interrupted state (is {content.get("status")})')
            return

        # Allow the user to edit the intention at approval time
        if edited_intention:
            content['intention'] = edited_intention

        now = datetime.now().isoformat()

        # Determine cooldown based on task origin
        cooldown = 10  # default for user-initiated
        if content.get("linked_concern_id"):
            try:
                for c in self._derived_concern_model.get_concerns():
                    if c.get("concern_id") == content.get("linked_concern_id"):
                        if c.get("seeded"):
                            label = c.get("concern_label", "")
                            if "knowledge" in label.lower() or "improvement" in label.lower():
                                cooldown = 7200
                            else:
                                cooldown = 3600
                        else:
                            cooldown = 1800
                        break
            except Exception:
                pass

        # Transition directly to operational state — skip establishment
        content['status'] = 'active'
        content['phase'] = 'complete'
        content['updated'] = now
        content['lifecycle'] = 'operational'
        content['completion_summary'] = 'Direct to operational — no establishment phase'
        content['establishment_milestones'] = []
        content['establishment_findings'] = []
        content['execution_history'] = []
        content['last_executed'] = None
        content['execution_count'] = 0
        content['cooldown_seconds'] = cooldown
        content['cycle_state'] = 'idle'
        content['cycle_goals_completed'] = []
        content['cycle_findings'] = []

        self.infospace_executor.execute_action({
            "type": "create-note",
            "value": json.dumps(content),
            "name": note_name,
            "out": f"${note_name}",
        })
        self.infospace_executor.execute_action({
            "type": "persist",
            "target": f"${note_name}",
            "name": note_name,
        })
        logger.info(f'📋 Task approved (operational): {note_name} — "{content["intention"][:80]}"')
        self._say_to_user(f'Task approved: {content["intention"][:200]}\nReady for execution.')

    def _abandon_task(self, note_name: str, reason: str = ''):
        """Abandon a task at any lifecycle stage. Triggers distillation if there's work to learn from."""
        if not self.resource_manager:
            return
        note_id = self.resource_manager.named_notes.get(note_name)
        if not note_id:
            logger.warning(f'Task abandon: note {note_name} not found')
            return
        try:
            res = self.resource_manager.get_resource(note_id)
            content = json.loads(res.get('properties', {}).get('content', '{}'))
        except Exception as e:
            logger.warning(f'Task abandon: could not read {note_name}: {e}')
            return
        old_status = content.get('status', '')
        content['status'] = 'abandoned'
        content['abandon_reason'] = reason
        content['updated'] = datetime.now().isoformat()
        self.infospace_executor.execute_action({
            "type": "create-note",
            "value": json.dumps(content),
            "name": note_name,
            "out": f"${note_name}",
        })
        self.infospace_executor.execute_action({
            "type": "persist",
            "target": f"${note_name}",
            "name": note_name,
        })
        # If this was the active WIP, clear it
        if self.active_task_wip == note_name:
            self.active_task_wip = None
            self.active_task_wip_waiting = False
            self._task_wip_pre_resource_ids = None
        # Clear operational goal state if this task's goal is running
        if self._operational_task_note == note_name:
            self._operational_task_note = None
            self._operational_goal_waiting = False
        logger.info(f'📋 Task abandoned: {note_name} (was {old_status})')
        self._say_to_user(f'Task abandoned: {content.get("intention", "")[:200]}')


    def _apply_concern_recommendation(self, concern_id: str, recommendation: str, rationale: str):
        """Apply a distillation concern recommendation to the derived concern model."""
        op_map = {'resolve': 'satisfy_concern', 'abandon': 'abandon_concern'}
        op = op_map.get(recommendation)
        if not op:
            return  # 'keep_active' — no action needed
        try:
            patch = {
                'op': op,
                'concern_id': concern_id,
                'field_updates': {'status_rationale': rationale},
            }
            self._derived_concern_model._apply_patch(patch, f'distillation:{concern_id}')
            self._derived_concern_model._save()
        except Exception as e:
            logger.debug(f'Concern recommendation application skipped: {e}')

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

            # Also record as OODA feed entry so the feed stays populated
            # even while goals are running (when the OODA pipeline is paused)
            if hasattr(self, '_ooda_event_feed'):
                self._ooda_event_feed.append({
                    'timestamp': timestamp.isoformat(timespec='seconds'),
                    'source': action_data.get('status', ''),
                    'event_type': 'action',
                    'classification': normalized_type,
                    'action_choice': '',
                    'action_taken': normalized_type,
                    'concern_bumps': '',
                })
                if len(self._ooda_event_feed) > self._OODA_FEED_MAX:
                    self._ooda_event_feed = self._ooda_event_feed[-self._OODA_FEED_MAX:]
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
        # current_plan may be a list (plan_actions from generate_plan) or a dict; normalize for publishing
        plan_for_publish = self.current_plan
        if isinstance(plan_for_publish, list):
            plan_for_publish = {'plan': plan_for_publish}
        elif not isinstance(plan_for_publish, dict):
            plan_for_publish = {}
        try:
            current_plan_data = {
                'current_plan': json.dumps(plan_for_publish, indent=2) if plan_for_publish else '',
                'plan_data': plan_for_publish,
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

        # Clear cached goal entities so next goal gets fresh extraction
        if self.infospace_executor:
            self.infospace_executor._goal_entities = None

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
            'primary_product': self.current_plan.get('primary_product', ''),
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
        # Skip for scheduled goal executions — they are not user conversations.
        is_scheduled_goal = bool(self._active_scheduled_goal_id)
        if (not is_scheduled_goal) and (not last_say) and (not interrupted_final):
            # Prefer primary_product content over FINAL_ANSWER when available,
            # since FINAL_ANSWER is often empty/trivial while the artifact has the real data.
            primary_product = plan_result.get('primary_product', '')
            conv_text = final_thoughts_clean
            if primary_product and (not conv_text or len(conv_text) < 20) and self.resource_manager:
                try:
                    res = self.resource_manager.get_resource(primary_product)
                    if res:
                        content = str(getattr(res, 'content', '') or getattr(res, 'text', '') or '')
                        if not content:
                            props = getattr(res, 'properties', {}) or {}
                            content = str(props.get('text', '') or props.get('content', ''))
                        if content and len(content) >= 20:
                            conv_text = content[:1500]
                            logger.debug(f'Using primary_product {primary_product} content for conversation record')
                except Exception as e:
                    logger.debug(f'Could not load primary_product for conversation record: {e}')
            if conv_text and len(conv_text) >= 20 and conv_text.replace(' ', '').replace('.', '').replace(',', '').replace('!', '').replace('?', '').strip():
                logger.debug(f'Recording conversation response ({len(conv_text)} chars): {conv_text[:50]}...')
                self.conversation_store.record_outgoing("User", conv_text, act_type="response")
        
        # Publish result to action log for UI display.
        # Skip if the plan already communicated via say — avoids duplicate messages.
        # Fall back to primary_product content or FINAL_ANSWER when no say occurred.
        primary_product = plan_result.get('primary_product', '')
        if not interrupted_final and not last_say:
            product_content = ''
            if primary_product and self.resource_manager:
                try:
                    res = self.resource_manager.get_resource(primary_product)
                    if res:
                        product_content = str(getattr(res, 'content', '') or getattr(res, 'text', '') or '')
                        if not product_content:
                            props = getattr(res, 'properties', {}) or {}
                            product_content = str(props.get('text', '') or props.get('content', ''))
                except Exception as e:
                    logger.debug(f'Could not load primary_product content for delivery: {e}')

            if product_content and len(product_content) >= 20:
                # Deliver the actual artifact content
                display_text = product_content[:3000]
                result_action = {
                    'type': 'say',
                    'action_type': 'say',
                    'action_id': f'final_answer_{int(time.time() * 1000)}',
                    'timestamp': datetime.now().isoformat(),
                    'text': display_text,
                    'source': self.character_name,
                    'target': 'User',
                    'is_text_only': True
                }
                self.action_publisher.put(json.dumps(result_action))
                logger.info(f'📤 Published primary product content ({len(product_content)} chars) from {primary_product}')
            elif final_thoughts_clean and len(final_thoughts_clean) >= 20 and final_thoughts_clean.replace(' ', '').replace('.', '').replace(',', '').replace('!', '').replace('?', '').strip():
                # No product content — show FINAL_ANSWER text
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
                logger.info(f'📤 Published FINAL_ANSWER to action log: {final_thoughts_clean[:100]}...')

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

    def _purge_untagged_history_notes(self):
        """Remove conversation_history entries that have no concern_id tag (legacy pre-rollup notes)."""
        history_notes = self.conversation_store.get_history_notes()
        if not history_notes:
            return
        untagged = [n for n in history_notes if not n.get("concern_id")]
        if not untagged:
            return
        untagged_ids = [n["note_id"] for n in untagged]
        logger.info(f'🧹 Purging {len(untagged_ids)} untagged conversation_history entries (of {len(history_notes)} total)')
        self.conversation_store._remove_notes_from_collection(
            untagged_ids, collection_name="conversation_history", delete_notes=True
        )

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
        """Build situation context from persisted situation note.

        Refreshes collection item counts from live resource data so the
        prompt never shows stale counts (e.g. '0 items' for a collection
        that has grown since the situation note was last written).
        """
        if not self.situation_context:
            return ""
        text = self.situation_context
        # Refresh item counts for named collections from live resource data
        rm = self.resource_manager
        if rm:
            import re
            for name, cid in rm.named_collections.items():
                if name.startswith('_'):
                    continue
                res = rm.get_resource(cid)
                if not res:
                    continue
                live_count = res.get('properties', {}).get('item_count', 0)
                # Replace stale count patterns like:  Collection_2 "name" (0 items)
                pattern = re.escape(f'{cid} "{name}"') + r' \(\d+ items\)'
                replacement = f'{cid} "{name}" ({live_count} items)'
                text = re.sub(pattern, replacement, text)
        return text

    def _update_situation_note(self, goal_text: str, plan_result: dict):
        """Update the living context (_situation) note after goal completion.

        The note is primarily data-driven (resource inventory, goal outcomes).
        A light LLM pass extracts any cross-goal learnings from the completed goal
        and prunes stale entries from the learnings section.
        """
        if self.benchmark_mode or not self.infospace_executor:
            return
        try:
            status = plan_result.get('status', 'unknown')
            summary = (plan_result.get('final_thoughts') or '')[:400]

            # --- Section 1: Persistent resource inventory ---
            resource_lines = []
            rm = self.resource_manager
            if rm:
                for name, rid in sorted(rm.named_notes.items()):
                    if name.startswith('_'):
                        continue  # skip system internals
                    res = rm.get_resource(rid)
                    if not res:
                        continue
                    props = res.get('properties', {})
                    if not props.get('persistent', False):
                        continue
                    updated = props.get('updated', '')[:10]
                    label = f"  {rid} \"{name}\""
                    if updated:
                        label += f" (updated {updated})"
                    resource_lines.append(label)
                for name, cid in sorted(rm.named_collections.items()):
                    if name.startswith('_'):
                        continue
                    res = rm.get_resource(cid)
                    if not res:
                        continue
                    props = res.get('properties', {})
                    if not props.get('persistent', False):
                        continue
                    count = props.get('item_count', 0)
                    resource_lines.append(f"  {cid} \"{name}\" ({count} items)")

            # --- Section 2: Recent goal outcomes ---
            goal_lines = []
            try:
                scheduled = self._all_scheduled_goals()
                recent = [g for g in scheduled if g.get('status') in ('completed', 'failed')]
                recent.sort(key=lambda g: g.get('updated', ''), reverse=True)
                for g in recent[:5]:
                    name = g.get('name') or g.get('goal_text', '?')
                    product = g.get('primary_product', '')
                    g_status = g.get('status', '?')
                    triggered = g.get('triggered_by', '')
                    line = f"  {name} [{g_status}]"
                    if triggered:
                        line += f" (triggered by {triggered})"
                    if product:
                        line += f" → {product}"
                    goal_lines.append(line)
            except Exception:
                pass

            # --- Section 3: Learnings (LLM-maintained) ---
            # Extract existing learnings from current note
            current = self.situation_context or ""
            existing_learnings = ""
            marker = "## Learnings"
            if marker in current:
                existing_learnings = current[current.index(marker) + len(marker):].strip()

            # LLM pass: extract new learnings from the just-completed goal, prune stale ones
            new_learnings = existing_learnings
            if summary and len(summary) > 20:
                _MAX_LEARNINGS = 20
                prompt = (
                    "You maintain a short list of cross-goal learnings for a planning agent. "
                    "Each entry is a one-line fact useful across future goals "
                    "(e.g., user preferences, working data sources, useful resource names, "
                    "things that failed and shouldn't be retried the same way).\n\n"
                    "Do NOT include: goal status, project narratives, what to do next, "
                    "or anything already tracked by world_model.\n"
                    f"Keep max {_MAX_LEARNINGS} entries. Merge near-duplicates into a single entry. "
                    "Drop stale or redundant ones.\n"
                    "Output ONLY the bullet list (- item), or 'none' if nothing to retain.\n\n"
                    f"## EXISTING LEARNINGS\n{existing_learnings or '(none yet)'}\n\n"
                    f"## JUST COMPLETED\nGoal: {goal_text}\nOutcome: {status}\n"
                    f"Summary: {summary}\n\n"
                    "Write the updated learnings list.\n</end>"
                )
                with self.infospace_executor.turn_metrics.perf_phase("post_plan"):
                    response = self.infospace_executor.llm_generate(
                        prompt, max_tokens=912, temperature=0.2, stops=['</end>']
                    )
                if response.success and response.text and response.text.strip().lower() != 'none':
                    # Clean LLM output: keep only bullet lines, enforce hard cap
                    raw = response.text.strip()
                    cleaned_lines = []
                    for ln in raw.split('\n'):
                        stripped = ln.strip()
                        if not stripped or stripped.lower() == 'none':
                            continue
                        if stripped.startswith('- '):
                            cleaned_lines.append(stripped)
                        elif cleaned_lines:
                            # Continuation line — skip non-bullet noise
                            continue
                    # Hard cap: keep only first _MAX_LEARNINGS entries
                    cleaned_lines = cleaned_lines[:_MAX_LEARNINGS]
                    cleaned = '\n'.join(cleaned_lines).strip()
                    if cleaned:
                        new_learnings = cleaned

            # --- Assemble note ---
            parts = []
            if resource_lines:
                parts.append("## Persistent Resources\n" + "\n".join(resource_lines))
            if goal_lines:
                parts.append("## Recent Goal Outcomes\n" + "\n".join(goal_lines))
            if new_learnings:
                parts.append(f"## Learnings\n{new_learnings}")

            new_content = "\n\n".join(parts) if parts else "(no situation data)"

            # Backup and write
            self._write_named_note('_situation_prev', self.situation_context or '')
            self._write_named_note('_situation', new_content)
            self.situation_context = new_content
            logger.info(f'✓ Updated situation note ({len(new_content)} chars)')

            # Feed learnings into world_model as durable facts
            if new_learnings and hasattr(self, 'world_model') and self.world_model:
                try:
                    self._feed_learnings_to_world_model(new_learnings, goal_text, status)
                except Exception as e:
                    logger.debug(f'Learnings→world_model feed skipped: {e}')
        except Exception as e:
            logger.warning(f'Error updating situation note: {e}')

    def _feed_learnings_to_world_model(self, learnings_text: str, goal_text: str, status: str):
        """Feed situation learnings into world_model as candidate facts.

        Each bullet is submitted as a world_model_update. The world_model's
        own generalization check filters out episodic or session-specific items.
        """
        updates = []
        for line in learnings_text.split('\n'):
            line = line.strip()
            if line.startswith('- '):
                fact = line[2:].strip()
                if len(fact) > 10:
                    updates.append({
                        "fact": fact,
                        "polarity": "support",
                        "confidence": "medium",
                        "source": "learning",
                    })
        if updates:
            self.world_model.update({
                "world_model_updates": updates,
                "tool_insights": [],
            })
            logger.info(f'🌍 Fed {len(updates)} learnings to world_model for generalization check')

    def _consolidate_situation_note(self):
        """Final situation note consolidation at shutdown.

        Data sections (resources, goal outcomes) are rebuilt on next goal,
        so consolidation only needs to prune the learnings section.
        """
        if self.benchmark_mode or not self.infospace_executor:
            return
        if self.plan_counter == 0 and not self.situation_context:
            logger.info('No goals executed and no situation note — skipping consolidation')
            return
        try:
            current = self.situation_context or ""
            marker = "## Learnings"
            if marker not in current:
                logger.info('No learnings section to consolidate')
                return

            learnings = current[current.index(marker) + len(marker):].strip()
            if not learnings:
                return

            _MAX_LEARNINGS = 20
            prompt = (
                "You maintain a short list of cross-goal learnings for a planning agent. "
                "A session is ending. Prune entries that are stale, redundant, or too specific "
                "to a single past goal. Merge near-duplicates into a single entry. "
                f"Keep max {_MAX_LEARNINGS} durable, reusable facts.\n"
                "Output ONLY the bullet list (- item), or 'none' if nothing worth keeping.\n\n"
                f"## CURRENT LEARNINGS\n{learnings}\n\n"
                f"Session: {self.plan_counter} goal(s) attempted.\n"
                "Write the pruned learnings list.\n</end>"
            )
            response = self.infospace_executor.llm_generate(
                prompt, max_tokens=912, temperature=0.2, stops=['</end>']
            )
            if response.success and response.text:
                pruned = response.text.strip()
                if pruned.lower() == 'none':
                    pruned = ""
                else:
                    # Hard cap: keep only bullet lines, max _MAX_LEARNINGS
                    pruned_lines = [ln.strip() for ln in pruned.split('\n')
                                    if ln.strip().startswith('- ')]
                    pruned = '\n'.join(pruned_lines[:_MAX_LEARNINGS])
                # Rebuild note with pruned learnings — data sections will refresh on next goal
                new_content = current[:current.index(marker)].rstrip()
                if pruned:
                    new_content += f"\n\n## Learnings\n{pruned}"
                self._write_named_note('_situation_prev', self.situation_context or '')
                self._write_named_note('_situation', new_content)
                self.situation_context = new_content
                logger.info(f'✓ Consolidated situation note at shutdown ({len(new_content)} chars)')
            else:
                logger.warning('Situation consolidation LLM call failed, keeping existing note')
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

    def _initialize_scheduled_goals(self):
        """Ensure scheduled goals collection exists and initialize ID counter."""
        result = self.infospace_executor.execute_action({"type": "load", "target": SCHEDULED_GOALS_COLLECTION, "out": "$_scheduled_goals_col"})
        if result.get("status") != "success" or not result.get("resource_id"):
            self.infospace_executor.execute_action({"type": "create-collection", "name": SCHEDULED_GOALS_COLLECTION, "out": "$_scheduled_goals_col"})
            self.infospace_executor.execute_action({"type": "persist", "target": "$_scheduled_goals_col"})
            logger.info("Created _scheduled_goals collection")
        max_id = 0
        for name in list(self.resource_manager.named_notes.keys()):
            if name.startswith(SCHEDULED_GOAL_NOTE_PREFIX):
                try:
                    num = int(name[len(SCHEDULED_GOAL_NOTE_PREFIX):])
                    max_id = max(max_id, num)
                except ValueError:
                    pass
        self._scheduled_goal_counter = max_id

    def _scheduled_goal_note_name(self, goal_id: str) -> str:
        return SCHEDULED_GOAL_NOTE_PREFIX + goal_id.replace("goal_", "")

    def _scheduled_goals_note_ids(self) -> List[str]:
        col_id = self.resource_manager.named_collections.get(SCHEDULED_GOALS_COLLECTION)
        if not col_id and hasattr(self.resource_manager, "_resolve_resource_id"):
            col_id = self.resource_manager._resolve_resource_id(SCHEDULED_GOALS_COLLECTION)
        if col_id:
            res = self.resource_manager.get_resource(col_id)
            if res:
                content = res.get("properties", {}).get("content", [])
                if isinstance(content, list):
                    return content
        return [self.resource_manager.named_notes[n] for n in self.resource_manager.named_notes if n.startswith(SCHEDULED_GOAL_NOTE_PREFIX)]

    def _new_scheduled_goal(self, goal_id: str, goal_text: str, name: Optional[str] = None) -> Dict[str, Any]:
        now = datetime.now().isoformat()
        base_name = (name or goal_text or "Scheduled goal").strip()
        return {
            "goal_id": goal_id,
            "name": base_name[:120],
            "goal_text": goal_text,
            "status": "ready",
            "created": now,
            "updated": now,
            "schedule_mode": "manual",
            "run_at": "",
            "last_run_date": "",
            "cached_plan_actions": [],
            "execution_mode": "replan",
            "is_running": False,
            "last_result": "",
            "primary_product": "",
            "name_customized": False,
            # Vision criteria persistence (see incremental_planner._generate_vision)
            "vision_criteria": "",
            "vision_criteria_source": "",
            "vision_criteria_meta": {},
            # Per-run instrumentation surfaced for the benchmark harness
            "step_results": [],
            "last_run_mode": "",
            "last_quality_eval": "",
            "quality_status": "",
        }

    def _normalize_scheduled_goal(self, goal: Dict[str, Any]) -> Tuple[Dict[str, Any], bool]:
        changed = False
        if not isinstance(goal, dict):
            return self._new_scheduled_goal("goal_0", ""), True
        for key, default in {
            "goal_id": "",
            "name": "",
            "goal_text": "",
            "status": "ready",
            "created": datetime.now().isoformat(),
            "updated": datetime.now().isoformat(),
            "schedule_mode": "manual",
            "run_at": "",
            "last_run_date": "",
            "cached_plan_actions": [],
            "execution_mode": "replan",
            "is_running": False,
            "last_result": "",
            "primary_product": "",
            "name_customized": False,
            "vision_criteria": "",
            "vision_criteria_source": "",
            "vision_criteria_meta": {},
            "step_results": [],
            "last_run_mode": "",
            "last_quality_eval": "",
            "quality_status": "",
        }.items():
            if key not in goal:
                goal[key] = default
                changed = True
        if not isinstance(goal.get("cached_plan_actions"), list):
            goal["cached_plan_actions"] = []
            changed = True
        if not isinstance(goal.get("step_results"), list):
            goal["step_results"] = []
            changed = True
        if not isinstance(goal.get("vision_criteria_meta"), dict):
            goal["vision_criteria_meta"] = {}
            changed = True
        if goal.get("schedule_mode") not in ("manual", "auto", "recurring", "daily"):
            goal["schedule_mode"] = "manual"
            changed = True
        if goal.get("execution_mode") not in ("replan", "replay"):
            goal["execution_mode"] = "replan"
            changed = True
        return goal, changed

    def _save_scheduled_goal(self, goal: Dict[str, Any]):
        goal_id = goal["goal_id"]
        note_name = self._scheduled_goal_note_name(goal_id)
        content = json.dumps(goal, ensure_ascii=False, indent=2)
        result = self.infospace_executor.execute_action({"type": "load", "target": note_name, "out": "$_scheduled_goal_save_tmp"})
        if result.get("status") == "success" and result.get("resource_id"):
            self.resource_manager.update_note_content(result["resource_id"], content)
            return
        # Bypass execute_action to avoid _resolve_value corrupting JSON
        # when goal text contains $variable references (e.g. "$satsang")
        note_id = self.infospace_executor._persist_note(content, 'scheduled-goal', note_name=note_name)
        if note_id:
            self.resource_manager.mark_persistent(note_id, self.character_name)
        col_id = self.resource_manager.named_collections.get(SCHEDULED_GOALS_COLLECTION) or self.resource_manager._resolve_resource_id(SCHEDULED_GOALS_COLLECTION)
        if col_id and note_id:
            added, _, err = self.resource_manager.add_to_collection(col_id, note_id, self.character_name, operation="add")
            if not added:
                logger.warning(f"Failed to add scheduled goal note to {SCHEDULED_GOALS_COLLECTION}: {err}")

    def _all_scheduled_goals(self) -> List[Dict[str, Any]]:
        goals: List[Dict[str, Any]] = []
        for note_id in self._scheduled_goals_note_ids():
            content = self.infospace_executor._get_content(note_id)
            if not content:
                continue
            try:
                goal = json.loads(content) if isinstance(content, str) else content
            except (json.JSONDecodeError, TypeError):
                continue
            if not isinstance(goal, dict):
                continue
            goal, changed = self._normalize_scheduled_goal(goal)
            if changed and goal.get("goal_id"):
                self._save_scheduled_goal(goal)
            # Include the storage Note ID for UI reference
            goal["note_id"] = note_id
            goals.append(goal)
        goals.sort(key=lambda g: g.get("created", ""))
        return goals

    def _get_scheduled_goal(self, goal_id: str) -> Optional[Dict[str, Any]]:
        if not goal_id:
            return None
        for goal in self._all_scheduled_goals():
            if goal.get("goal_id") == goal_id:
                return goal
        return None

    def _update_scheduled_goal(self, goal_id: str, **fields) -> Optional[Dict[str, Any]]:
        goal = self._get_scheduled_goal(goal_id)
        if not goal:
            return None
        goal.update(fields)
        goal["updated"] = datetime.now().isoformat()
        self._save_scheduled_goal(goal)
        return goal

    def _delete_scheduled_goal(self, goal_id: str) -> bool:
        note_name = self._scheduled_goal_note_name(goal_id)
        note_id = self.resource_manager.named_notes.get(note_name)
        if not note_id and hasattr(self.resource_manager, "_resolve_resource_id"):
            note_id = self.resource_manager._resolve_resource_id(note_name)
        if not note_id:
            return False
        deleted, _ = self.infospace_executor.delete_resource_and_unbind(note_id)
        return bool(deleted)

    def _upsert_scheduled_goal(self, goal_text: str) -> Dict[str, Any]:
        cleaned = (goal_text or "").strip()
        if not cleaned:
            self._scheduled_goal_counter += 1
            goal_id = f"goal_{self._scheduled_goal_counter}"
            goal = self._new_scheduled_goal(goal_id, cleaned)
            self._save_scheduled_goal(goal)
            return goal
        for goal in self._all_scheduled_goals():
            if goal.get("goal_text", "").strip() == cleaned:
                updates = {"goal_text": cleaned, "status": "ready", "updated": datetime.now().isoformat()}
                if not goal.get("name_customized"):
                    updates["name"] = cleaned[:120]
                goal.update(updates)
                self._save_scheduled_goal(goal)
                return goal
        self._scheduled_goal_counter += 1
        goal_id = f"goal_{self._scheduled_goal_counter}"
        goal = self._new_scheduled_goal(goal_id, cleaned)
        self._save_scheduled_goal(goal)
        return goal

    def _set_scheduled_goal_result(self, goal_id: str, result: Dict[str, Any], used_cache: bool = False, pre_resource_ids: set = None):
        goal = self._get_scheduled_goal(goal_id)
        if not goal:
            return
        success = bool(result and result.get("success"))
        primary_product = (result.get("primary_product") if isinstance(result, dict) else "") or ""
        last_result_raw = (result.get("response") if isinstance(result, dict) else "") or (result.get("error") if isinstance(result, dict) else "") or ""
        # Strip leaked stop-sequence markers from goal results
        if last_result_raw:
            last_result_raw = last_result_raw.replace('</end>', '').replace('</end', '').strip()

        user_interrupted = bool(last_result_raw and "Interrupted by user" in last_result_raw)

        # Enrich last_result from primary_product content when the LLM
        # response is empty/trivial but an actual artifact was produced.
        if primary_product and (not last_result_raw or len(last_result_raw) < 20) and self.resource_manager:
            try:
                res = self.resource_manager.get_resource(primary_product)
                if res:
                    content = str(getattr(res, 'content', '') or getattr(res, 'text', '') or '')
                    if not content:
                        props = getattr(res, 'properties', {}) or {}
                        content = str(props.get('text', '') or props.get('content', ''))
                    if content and len(content) > len(last_result_raw):
                        last_result_raw = content[:1500].replace('</end>', '').replace('</end', '').strip()
                        logger.info(f'Enriched last_result from primary_product {primary_product} ({len(content)} chars)')
            except Exception as e:
                logger.debug(f'Could not load primary_product for last_result enrichment: {e}')

        if user_interrupted:
            goal_status = "ready"
            schedule_updates: Dict[str, Any] = {"schedule_mode": "manual"}
        else:
            goal_status = "completed" if success else "failed"
            schedule_updates = {}

        updates: Dict[str, Any] = {
            "is_running": False,
            "status": goal_status,
            "last_result": last_result_raw,
            "primary_product": primary_product,
            **schedule_updates,
        }
        if not used_cache and success:
            plan_actions = result.get("plan") if isinstance(result, dict) else None
            if isinstance(plan_actions, list):
                updates["cached_plan_actions"] = plan_actions
        # Pull per-run instrumentation off the executor for the harness.
        # All four are best-effort: missing attribute → field stays at its
        # current goal-record value, which defaults to "" / [] / {}.
        run_mode = ""
        step_results_snapshot: List[Any] = []
        last_quality_eval = ""
        steps_executed = 0
        if self.infospace_executor is not None:
            run_mode = getattr(self.infospace_executor, '_last_run_mode', "") or ""
            sr = getattr(self.infospace_executor, '_step_results', None)
            if isinstance(sr, list):
                step_results_snapshot = list(sr)
                steps_executed = len(sr)
            last_quality_eval = getattr(self.infospace_executor, '_last_quality_eval', "") or ""
        # Always overwrite per-run instrumentation, even with empty values.
        # An empty step_results list means "this run produced no per-step
        # data" (typically a planning run, since execute_plan_sync only fires
        # in replay) — that is the correct fact for this run, not a reason
        # to keep stale data from the previous run. Same logic for the other
        # three fields: each must reflect THIS run, not blend across runs.
        updates["last_run_mode"] = run_mode
        updates["step_results"] = step_results_snapshot
        updates["last_quality_eval"] = last_quality_eval
        # quality_status is the canonical pass/fail/needs_revision signal
        # from the run. Read from the result dict (set by both planning and
        # replay paths) and persist as a string; empty string when absent.
        qs = ""
        if isinstance(result, dict):
            qs = str(result.get("quality_status") or "")
        updates["quality_status"] = qs
        self._update_scheduled_goal(goal_id, **updates)
        # Emit END log bracket. duration_ms is computed from the monotonic
        # start that the run-path stamped on the executor; quality is the
        # planner/replay-derived quality_status from the result dict.
        try:
            duration_ms = -1
            if self.infospace_executor is not None:
                t0 = getattr(self.infospace_executor, '_goal_started_at', None)
                if t0 is not None:
                    duration_ms = int((time.monotonic() - t0) * 1000)
            quality = ""
            if isinstance(result, dict):
                quality = (result.get("quality_status") or "") or ""
            logger.info(
                f"[GOAL {goal_id} mode={run_mode or 'unknown'} END "
                f"status={updates['status']} steps={steps_executed} "
                f"duration_ms={duration_ms} quality={quality or 'none'}]"
            )
        except Exception as _bracket_err:
            logger.debug(f"END bracket emit failed: {_bracket_err}")
        # ── Graph: goal outcome ──
        graph_success = False if user_interrupted else bool(success)
        self._graph_emit_goal_outcome(goal_id, graph_success, last_result_raw, primary_product, result)
        if goal_id in self._scheduler_started_goals:
            self._scheduler_started_goals.discard(goal_id)
        scheduler_end_status = "interrupted" if user_interrupted else updates["status"]
        self._record_scheduler_event(
            "end",
            goal_id=goal_id,
            goal_name=goal.get("name", "") or goal.get("goal_text", "")[:80],
            status=scheduler_end_status,
            result=updates["last_result"],
        )
        # Update concern models from goal completion.
        # Autonomous goals (establishment milestones with task_wip_id) update only the
        # derived concern model — they are agent-internal activity, not user interactions.
        # User-initiated goals update both models.
        is_autonomous_goal = bool(goal.get("task_wip_id"))
        concern_success = False if user_interrupted else bool(success)
        try:
            full_goal_text = goal.get("goal_text", "") or goal.get("name", "")
            outcome = updates.get("last_result", "")[:800]
            # Fetch primary product content for richer concern context
            artifact_content = ''
            pp = updates.get("primary_product", "")
            if pp and self.resource_manager:
                try:
                    res = self.resource_manager.get_resource(pp)
                    if res:
                        props = res.get('properties', {}) if isinstance(res, dict) else {}
                        artifact_content = str(props.get('content', '') or props.get('text', ''))[:1000]
                except Exception:
                    pass
            if not is_autonomous_goal:
                self.user_concern_model.update_from_goal_completion(
                    goal_statement=full_goal_text,
                    outcome_summary=outcome,
                    goal_id=goal_id,
                    success=concern_success,
                    result_artifact=artifact_content,
                )
            else:
                logger.debug(f'Skipping user concern update for autonomous goal {goal_id}')
        except Exception as e:
            logger.warning(f'Error updating concern model from goal {goal_id}: {e}')
        # Derived concern model: always update (agent learns from all goal outcomes)
        try:
            self._derived_concern_model.update_from_goal_completion(
                goal_statement=full_goal_text,
                outcome_summary=outcome,
                goal_id=goal_id,
                success=concern_success,
                user_concerns=self.user_concern_model.get_concerns(),
                living_state=self._ooda_living_state,
            )
        except Exception as e:
            logger.warning(f'Error updating derived concerns from goal {goal_id}: {e}')
        if self._active_scheduled_goal_id == goal_id:
            self._active_scheduled_goal_id = None
        if hasattr(self, "goal_scheduler"):
            self.goal_scheduler.notify_step_completed(goal_id)
        # If this was a task milestone goal, update WIP Note and unblock milestone loop
        task_wip_id = goal.get("task_wip_id")
        if task_wip_id and self.active_task_wip:
            # Check if the milestone was interrupted (not just failed)
            last_result = updates.get("last_result", "")
            was_interrupted = "Interrupted by user" in last_result
            if was_interrupted:
                # Interrupt bubbles up: abort the entire task establishment
                logger.info(f'📋 Task WIP {task_wip_id}: milestone interrupted — aborting task')
                wip = self._read_task_wip()
                if wip:
                    wip["status"] = "interrupted"
                    wip["current_milestone"] = None
                    wip["updated"] = datetime.now().isoformat()
                    wip.setdefault("accumulated_findings", []).append(
                        "Task interrupted by user during milestone execution"
                    )
                    self._update_task_wip(wip)
                self.active_task_wip = None
                self.active_task_wip_waiting = False
                self._task_wip_pre_resource_ids = None
                self._say_to_user("Task establishment interrupted.")
            else:
                try:
                    wip = self._read_task_wip()
                    # Specification milestones carry user answers — allow more room
                    phase = (wip or {}).get("phase", "")
                    summary_limit = 1500 if phase == "specification" else 500
                    result_summary = last_result[:summary_limit] if last_result else ("success" if success else "failed")

                    # Fix B: If result_summary is empty/trivial but primary_product
                    # exists, load the resource content. This catches cases where the
                    # planner returned a status message but the actual data lives in
                    # a bound resource (e.g. user answers from an ask action).
                    if primary_product and (not result_summary or len(result_summary) < 20) and self.resource_manager:
                        try:
                            res = self.resource_manager.get_resource(primary_product)
                            if res:
                                content = str(getattr(res, 'content', '') or getattr(res, 'text', '') or '')
                                if not content:
                                    props = getattr(res, 'properties', {}) or {}
                                    content = str(props.get('text', '') or props.get('content', ''))
                                if content and len(content) > len(result_summary):
                                    result_summary = content[:summary_limit]
                                    logger.info(f'📋 Task WIP: enriched result_summary from primary_product {primary_product} ({len(content)} chars)')
                        except Exception as e:
                            logger.debug(f'Could not load primary_product content: {e}')

                    # Sanitize Note IDs → named references to prevent stale ID leakage
                    clean_summary = self._sanitize_note_ids(result_summary)
                    clean_goal_text = self._sanitize_note_ids(goal.get("goal_text", ""))
                    # Derive error_class from planner signals for structured recovery
                    error_class = ""
                    if not success:
                        qa = (result.get("quality_status") or "").lower()
                        va = (result.get("verification_answer") or "").lower()
                        if "interrupt" in (result.get("response") or "").lower():
                            error_class = "interrupted"
                        elif "inconclusive" in va or "partial" in va:
                            error_class = "partial"
                        elif qa in ("fail", "failed"):
                            error_class = "quality_fail"
                        elif any(w in clean_summary.lower() for w in ("timeout", "timed out")):
                            error_class = "timeout"
                        elif any(w in clean_summary.lower() for w in ("no results", "not found", "no relevant")):
                            error_class = "no_results"
                        else:
                            error_class = "tool_failure"
                    milestone_record = {
                        "goal_text": clean_goal_text,
                        "result_summary": clean_summary,
                        "status": "completed" if success else "failed",
                        "error_class": error_class,
                        "timestamp": datetime.now().isoformat(),
                    }
                    if wip:
                        wip.setdefault("milestones_completed", []).append(milestone_record)
                        wip["current_milestone"] = None
                        wip["updated"] = datetime.now().isoformat()
                        # Capture primary_product content in accumulated_findings
                        if primary_product and clean_summary and len(clean_summary) > 20:
                            wip.setdefault("accumulated_findings", []).append(
                                f"Milestone result: {clean_summary}"
                            )
                        elif primary_product:
                            # Use named note if available, else fall back to ID
                            artifact_name = self._sanitize_note_ids(primary_product)
                            wip.setdefault("accumulated_findings", []).append(
                                f"Milestone produced artifact: {artifact_name}"
                            )
                        # For specification phase, also capture the result as a finding
                        # so subsequent phases see user answers in ACCUMULATED FINDINGS
                        if phase == "specification" and result_summary and len(result_summary) > 20:
                            wip.setdefault("accumulated_findings", []).append(
                                f"User clarification: {result_summary}"
                            )
                        self._update_task_wip(wip)
                    logger.info(f'📋 Task WIP {task_wip_id}: milestone completed ({updates["status"]})')
                except Exception as e:
                    logger.warning(f'Error updating task WIP after milestone: {e}')
                self.active_task_wip_waiting = False

        # If this was an operational task-linked goal, update task execution history
        task_note_name = goal.get("task_context_note")
        if task_note_name and not task_wip_id:
            try:
                note_id = self.resource_manager.named_notes.get(task_note_name) if self.resource_manager else None
                if note_id:
                    res = self.resource_manager.get_resource(note_id)
                    content = getattr(res, 'content', '') if res else ''
                    task_data = json.loads(content) if content else None
                    if task_data and task_data.get("lifecycle") == "operational":
                        last_result = updates.get("last_result", "")
                        exec_record = {
                            "goal_id": goal_id,
                            "goal_text": goal.get("goal_text", "")[:200],
                            "timestamp": datetime.now().isoformat(),
                            "outcome": "success" if success else "failed",
                            "summary": (last_result or "")[:300],
                        }
                        history = task_data.get("execution_history", [])
                        history.append(exec_record)
                        # Ring buffer: keep last N entries
                        if len(history) > self._TASK_EXECUTION_HISTORY_MAX:
                            history = history[-self._TASK_EXECUTION_HISTORY_MAX:]
                        task_data["execution_history"] = history
                        task_data["last_executed"] = datetime.now().isoformat()
                        task_data["execution_count"] = task_data.get("execution_count", 0) + 1
                        task_data["updated"] = datetime.now().isoformat()
                        # Write back
                        self.resource_manager.update_note_content(note_id, json.dumps(task_data))
                        logger.info(f'📋 Task {task_note_name}: operational execution #{task_data["execution_count"]} recorded ({exec_record["outcome"]})')
            except Exception as e:
                logger.warning(f'Error updating operational task {task_note_name} after goal completion: {e}')

        # Clean up transient resources created during the goal
        if pre_resource_ids is not None and self.resource_manager:
            now_ids = set(self.resource_manager.resource_registry.keys())
            created_ids = now_ids - pre_resource_ids
            keep_ids = {primary_product} if primary_product else set()
            self._cleanup_transient_resources(created_ids, keep_ids, label=goal_id)
            # Batch-index only the notes that survived cleanup
            self.resource_manager.flush_deferred_indexes()

        # Announce completion to user (unless this is an internal task milestone)
        if not task_wip_id:
            goal_name = goal.get('name') or goal.get('goal_text', '')[:80] or goal_id
            pp_ref = f' → {primary_product}' if primary_product else ''
            if user_interrupted:
                self._say_to_user(f"Goal '{goal_name}' stopped (interrupted). Reset to ready.{pp_ref}")
            else:
                status_word = 'completed' if success else 'failed'
                self._say_to_user(f"Goal '{goal_name}' {status_word}.{pp_ref}")

        # Fire callback if this was a delegated task from another agent
        self._fire_delegation_callback(goal, False if user_interrupted else success, last_result_raw, primary_product)

        # Delete ephemeral goals (CLI/interpreted) on completion — they have no
        # persistent schedule value.  Task-linked goals are never ephemeral.
        if goal.get('ephemeral') and not task_wip_id and not goal.get('task_context_note') and not user_interrupted:
            try:
                self._delete_scheduled_goal(goal_id)
                logger.info(f'🗑 Deleted ephemeral goal {goal_id} after completion')
            except Exception as e:
                logger.debug(f'Failed to delete ephemeral goal {goal_id}: {e}')

        self._publish_execution_state()

    def _fire_delegation_callback(self, goal: Dict, success: bool,
                                   last_result: str, primary_product: str):
        """If this goal was delegated from another agent, notify the source agent."""
        callback_topic = goal.get('callback_topic')
        if not callback_topic:
            return
        concern_id = goal.get('callback_concern_id', '')
        source_agent = goal.get('source_agent', '')
        goal_name = goal.get('name') or goal.get('goal_text', '')[:120]
        status_word = 'completed' if success else 'failed'

        # Build a result summary
        summary = last_result[:500] if last_result else status_word
        if primary_product:
            summary += f'\nPrimary product: {primary_product}'

        # 1. Notify the source agent via its sense_data channel
        sense_payload = {
            'timestamp': datetime.now().isoformat(),
            'sequence_id': 0,
            'mode': 'text',
            'content': json.dumps({
                'source': self.character_name,
                'text': (
                    f"[Delegation result] Goal '{goal_name}' {status_word}.\n"
                    f"Concern: {concern_id}\n"
                    f"Summary: {summary}"
                ),
            }),
        }
        try:
            self.session.put(
                callback_topic,
                json.dumps(sense_payload).encode('utf-8'),
            )
            logger.info(f'📬 Sent delegation callback to {callback_topic} for concern {concern_id}')
        except Exception as e:
            logger.warning(f'Failed to send delegation callback: {e}')

        # 2. Resolve the delegated concern on the source agent via its command channel
        if concern_id and source_agent:
            resolve_cmd = {
                'cmd': '/concern resolve',
                'concern_id': concern_id,
                'source': self.character_name,
            }
            try:
                self.session.put(
                    f"cognitive/{source_agent}/command",
                    json.dumps(resolve_cmd).encode('utf-8'),
                )
                logger.info(f'📬 Sent concern resolve to {source_agent} for {concern_id}')
            except Exception as e:
                logger.warning(f'Failed to send concern resolve command: {e}')

    def _init_run_instrumentation(self, mode: str):
        """Reset per-run executor state and emit START log bracket.

        Called from convergence points so every code path that runs a goal
        gets the same instrumentation reset, regardless of which entry was
        used (/goal run, /goal add, sense-data callback, sensor trigger,
        chat handler, etc.):

          - parse_and_set_goal at entry  → mode='replan'  (planning paths)
          - _handle_goal_reuse at entry  → mode='replay'  (replay path)

        The reset is unconditional and cheap (four attribute assignments).
        It ensures _step_results, _last_run_mode, _last_quality_eval, and
        _goal_started_at always reflect the most recent run, never stale
        data from a prior goal. The START log bracket is gated on
        _active_scheduled_goal_id being set, since non-scheduled-goal
        callers of parse_and_set_goal (chat handlers, ad-hoc agent
        messages) should not pollute the [GOAL goal_X mode=...] stream.
        """
        if self.infospace_executor is None:
            return
        self.infospace_executor._last_run_mode = mode
        self.infospace_executor._goal_started_at = time.monotonic()
        self.infospace_executor._step_results = []
        self.infospace_executor._last_quality_eval = ""
        if mode == "replay":
            # Replay also needs _plan_actions reset because execute_plan_sync
            # is the only path that records actions in replay; planning runs
            # have generate_plan reset _plan_actions on their own.
            self.infospace_executor._plan_actions = []

        goal_id = getattr(self, '_active_scheduled_goal_id', None)
        if goal_id:
            logger.info(f"[GOAL {goal_id} mode={mode} START]")

    def _handle_goal_proceed(self, goal_id: str = None, source: str = "user"):
        if not goal_id:
            self._say_to_user("Please specify which goal to proceed, e.g. 'proceed goal_1'.")
            return
        goal = self._get_scheduled_goal(goal_id)
        if not goal:
            self._say_to_user(f"Goal '{goal_id}' not found.")
            return
        if self._is_goal_running():
            self._say_to_user(f"A goal is already running. Please wait for it to complete.")
            return
        # Goal execution is part of the ongoing conversation — don't close dialog.
        # Dialog is closed by /bye, shutdown, or UI end-conversation button.
        # Apply any pending LLM switch before starting a new goal
        self._apply_pending_llm_switch()
        self._active_scheduled_goal_id = goal_id
        goal_updates = {"is_running": True, "status": "running"}
        if source and source != "user":
            goal_updates["triggered_by"] = source
        self._update_scheduled_goal(goal_id, **goal_updates)
        if hasattr(self, 'goal_scheduler'):
            self.goal_scheduler._executing_goal_id = goal_id
            self.goal_scheduler._executing_is_autonomous = False
        # Attach evaluator assessment if available (from the eval that ran when input was received)
        if self._last_character_eval:
            self._update_scheduled_goal(goal_id, initial_assessment=self._last_character_eval)
        self._record_scheduler_event(
            "start",
            goal_id=goal_id,
            goal_name=goal.get("name", "") or goal.get("goal_text", "")[:80],
        )
        # Per-run instrumentation reset and START log are handled by
        # _init_run_instrumentation, called at the entry of parse_and_set_goal
        # (the planning convergence point) once the run thread starts.
        pre_resource_ids = set(self.resource_manager.resource_registry.keys()) if self.resource_manager else set()
        notify_user = source != "scheduler"
        goal_name = goal.get('name') or goal_id

        # Prepend task context if this goal has a task_context_note
        effective_goal_text = goal.get("goal_text", "")
        task_ctx_note = goal.get("task_context_note")
        if task_ctx_note and self.resource_manager:
            try:
                ctx_note_id = self.resource_manager.named_notes.get(task_ctx_note)
                if ctx_note_id:
                    ctx_data = self.resource_manager.resource_registry.get(ctx_note_id, {})
                    ctx_content = ctx_data.get("content", "")
                    if ctx_content:
                        effective_goal_text = (
                            f"TASK CONTEXT (from task establishment):\n{ctx_content}\n\n"
                            f"GOAL:\n{effective_goal_text}"
                        )
            except Exception as e:
                logger.warning(f'Error loading task context note {task_ctx_note}: {e}')

        def _run():
            # Clear last_say_text so we can detect whether *this* goal said anything
            self.last_say_text = ''
            result: Dict[str, Any] = {}
            try:
                result = self.parse_and_set_goal("", effective_goal_text) or {}
            except Exception as e:
                result = {"success": False, "error": str(e)}
            self._set_scheduled_goal_result(goal_id, result, used_cache=False, pre_resource_ids=pre_resource_ids)
            if notify_user:
                # Only say completion if *this* goal's plan didn't already publish a say
                already_said = (getattr(self, 'last_say_text', '') or '').strip()
                if not already_said:
                    status = "completed" if result.get("success") else "failed"
                    pp = (result.get("primary_product") if isinstance(result, dict) else "") or ""
                    if pp:
                        self._say_to_user(f"Goal '{goal_name}' {status}. → {pp}")
                    else:
                        self._say_to_user(f"Goal '{goal_name}' {status}.")
            return result

        self._run_goal_on_thread(_run)

    def _handle_goal_reuse(self, goal_id: str = None):
        if not goal_id:
            self._say_to_user("Please specify which goal to reuse, e.g. 'reuse goal_1'.")
            return
        goal = self._get_scheduled_goal(goal_id)
        if not goal:
            self._say_to_user(f"Goal '{goal_id}' not found.")
            return
        cached = goal.get("cached_plan_actions")
        if not cached or not isinstance(cached, list):
            self._say_to_user(f"Goal '{goal_id}' has no cached plan. Use 'proceed {goal_id}' to replan.")
            return
        if self._is_goal_running():
            self._say_to_user(f"A goal is already running. Please wait for it to complete.")
            return
        # Execute the cached plan via execute_plan_sync
        self._active_scheduled_goal_id = goal_id
        self._update_scheduled_goal(goal_id, is_running=True, status="running")
        if hasattr(self, 'goal_scheduler'):
            self.goal_scheduler._executing_goal_id = goal_id
            self.goal_scheduler._executing_is_autonomous = False
        self._record_scheduler_event(
            "start",
            goal_id=goal_id,
            goal_name=goal.get("name", "") or goal.get("goal_text", "")[:80],
        )
        # Per-run instrumentation reset and START log via the shared helper.
        # Replay is the only entry path that does not go through
        # parse_and_set_goal, so the call is explicit here.
        self._init_run_instrumentation("replay")

        # Ensure vision_criteria are populated for this goal (cache hit or
        # generate+persist). Returns the criteria string; '' means no criteria.
        # Failure inside _generate_vision is non-fatal — execution still proceeds.
        vision_criteria = ""
        if self.incremental_planner is not None:
            try:
                vision_criteria = self.incremental_planner._generate_vision(goal.get("goal_text", "")) or ""
            except Exception as e:
                logger.warning(f"[GOAL {goal_id}] _generate_vision (replay) failed: {e}")
                vision_criteria = ""

        pre_resource_ids = set(self.resource_manager.resource_registry.keys()) if self.resource_manager else set()
        goal_name = goal.get('name') or goal_id

        def _run():
            # Clear side-effect dedup caches so replay re-executes send-email, post-bluesky, etc.
            self.infospace_executor._side_effect_cache = {}
            self.infospace_executor._successful_side_effect_results = {}
            result = {}
            try:
                sync_result = self.infospace_executor.execute_plan_sync({"plan": cached})
                success = sync_result.get("status") == "success"
                response_text = f"Cached plan replay {'succeeded' if success else 'failed'}: {sync_result.get('reason', '')}".strip()

                # Quality evaluation: only override the execution-only baseline
                # when criteria + a bound $eval_target are both present.
                quality_status = "passed" if success else "failed"
                eval_text = ""
                try:
                    bindings_flat = self.infospace_executor.plan_bindings_flat
                    eval_target_id = bindings_flat.get('eval_target') if bindings_flat else None
                except Exception:
                    eval_target_id = None
                if vision_criteria and eval_target_id and isinstance(eval_target_id, str) and (
                    eval_target_id.startswith('Note_') or eval_target_id.startswith('Collection_')
                ):
                    try:
                        from incremental_planner import _vision_eval_check
                        eval_text = _vision_eval_check(vision_criteria, eval_target_id, self.infospace_executor) or ""
                        if eval_text:
                            # _vision_eval_check returns one line per criterion in the
                            # form "criterion_name: PASS|FAIL - reason". Count verdicts;
                            # any FAIL marks the run as needs_revision.
                            fail_count = 0
                            pass_count = 0
                            for line in eval_text.splitlines():
                                stripped = line.strip()
                                if not stripped or ':' not in stripped:
                                    continue
                                verdict = stripped.split(':', 1)[1].strip().upper()
                                if verdict.startswith('FAIL'):
                                    fail_count += 1
                                elif verdict.startswith('PASS'):
                                    pass_count += 1
                            if fail_count > 0:
                                quality_status = "needs_revision"
                            elif pass_count > 0:
                                quality_status = "passed"
                            # If neither matched (parser found no verdicts) leave the
                            # execution-only baseline ("passed"/"failed") in place.
                            logger.info(
                                f"[GOAL {goal_id}] vision eval (replay) → "
                                f"{quality_status} (pass={pass_count} fail={fail_count})"
                            )
                    except Exception as e:
                        logger.warning(f"[GOAL {goal_id}] vision eval (replay) crashed: {e}")

                # Stash the eval text so _set_scheduled_goal_result can persist it.
                self.infospace_executor._last_quality_eval = eval_text

                result = {
                    "success": success,
                    "plan": cached,
                    "response": response_text,
                    "quality_status": quality_status,
                }
            except Exception as e:
                logger.error(f"[GOAL {goal_id}] replay raised: {e}")
                logger.error(traceback.format_exc())
                result = {"success": False, "error": str(e)}
            self._set_scheduled_goal_result(goal_id, result, used_cache=True, pre_resource_ids=pre_resource_ids)
            status = "completed" if result.get("success") else "failed"
            self._say_to_user(f"Goal '{goal_name}' reuse {status}.")
            return result

        self._run_goal_on_thread(_run)

    def _handle_goal_terminate(self, goal_id: str = None):
        if not goal_id:
            self._say_to_user("Please specify which goal to terminate, e.g. 'terminate goal_1'.")
            return
        # Handle operational task goals (op__task_wip_N)
        if goal_id.startswith('op_'):
            # Force-clear all goal state regardless of which tracker thinks it's running
            self.interrupt_requested = True
            if self.infospace_executor:
                self.infospace_executor.interrupt_requested = True
            task_note = goal_id[3:]  # strip 'op_' prefix
            if self._operational_task_note == task_note:
                self._operational_task_note = None
                self._operational_goal_waiting = False
            # Clear scheduler state if it matches
            if hasattr(self, 'goal_scheduler') and self.goal_scheduler._executing_goal_id == goal_id:
                self.goal_scheduler._executing_goal_id = None
            self._publish_execution_state()
            self._say_to_user(f"Goal '{goal_id}' terminated.")
            logger.info(f'📋 Operational goal {goal_id} terminated by user')
            return
        goal = self._get_scheduled_goal(goal_id)
        if not goal:
            self._say_to_user(f"Goal '{goal_id}' not found.")
            return
        if goal.get("is_running") or self._active_scheduled_goal_id == goal_id:
            # Cooperative interrupt only: goal note + scheduler end + cleanup run in
            # _set_scheduled_goal_result when the goal thread finishes.
            self.interrupt_requested = True
            if self.infospace_executor:
                self.infospace_executor.interrupt_requested = True
            # Immediate task WIP abort so establishment state does not stall while the planner unwinds.
            if goal.get("task_wip_id") and self.active_task_wip:
                logger.info(f'📋 Task WIP {goal.get("task_wip_id")}: milestone interrupted — aborting task')
                wip = self._read_task_wip()
                if wip:
                    wip["status"] = "interrupted"
                    wip["current_milestone"] = None
                    wip["updated"] = datetime.now().isoformat()
                    wip.setdefault("accumulated_findings", []).append(
                        "Task interrupted: milestone goal interrupted by user"
                    )
                    self._update_task_wip(wip)
                self.active_task_wip = None
                self.active_task_wip_waiting = False
                self._task_wip_pre_resource_ids = None
                self._say_to_user("Task establishment interrupted.")
            else:
                self._say_to_user(f"Stopping '{goal.get('name') or goal_id}'…")
            self._publish_execution_state()
            return
        deleted = self._delete_scheduled_goal(goal_id)
        if deleted:
            if goal_id in self._scheduler_started_goals:
                self._scheduler_started_goals.discard(goal_id)
            if hasattr(self, "goal_scheduler"):
                self.goal_scheduler.notify_goal_terminal(goal_id)
            self._publish_execution_state()
            self._say_to_user(f"Goal '{goal.get('name') or goal_id}' has been removed.")
        else:
            self._say_to_user(f"Goal '{goal_id}' could not be removed.")

    def _handle_goal_cache_clear(self, goal_id: str):
        goal = self._update_scheduled_goal(goal_id, cached_plan_actions=[], execution_mode="replan", status="ready")
        if not goal:
            self._say_to_user(f"Goal '{goal_id}' not found.")
            return
        self._say_to_user(f"Goal '{goal.get('name') or goal_id}' cache cleared.")

    # ── Plan view / edit / approve ──────────────────────────────────

    @staticmethod
    def _format_plan_for_display(actions: list) -> str:
        """Pretty-print cached_plan_actions for human review."""
        import textwrap
        if not actions:
            return "(no cached plan)"
        parts = []
        for i, action in enumerate(actions):
            atype = action.get("type", "unknown")
            header = f"─── Step {i + 1}  [{atype}] ───"
            if atype == "_code_block_":
                source = action.get("source", "")
                # Normalize indentation and add line numbers
                lines = textwrap.dedent(source).strip().splitlines()
                numbered = [f"  {n + 1:3d} │ {line}" for n, line in enumerate(lines)]
                parts.append(f"{header}\n" + "\n".join(numbered))
            elif atype == "ask":
                parts.append(f"{header}\n  ask: {action.get('value', '')}")
            else:
                # Generic action: show key=value pairs
                detail = ", ".join(f"{k}={v!r}" for k, v in action.items() if k != "type")
                parts.append(f"{header}\n  {detail}")
        return "\n\n".join(parts)

    @staticmethod
    def _format_plan_for_editor(actions: list, goal: dict) -> str:
        """Serialize cached_plan_actions to a human-editable temp-file format.

        Format:
          # Goal: <goal_id> — <name>
          # execution_mode: <current>
          # Delete a step by removing its entire block (header through next header).
          # Reorder steps by moving blocks. Edit code freely.

          ### STEP 1 [_code_block_]
          <source code>

          ### STEP 2 [ask]
          <ask value>
        """
        import textwrap
        lines = [
            f"# Goal: {goal.get('goal_id', '?')} — {goal.get('name', '')}",
            f"# execution_mode: {goal.get('execution_mode', 'replan')}",
            "#",
            "# Instructions:",
            "#   - Delete a step by removing its entire block (### STEP header through next header).",
            "#   - Reorder steps by moving blocks.",
            "#   - Edit code freely within a step.",
            "#   - To set execution_mode, change the value on the execution_mode line above.",
            "#   - Lines starting with # outside of step blocks are comments (ignored).",
            "",
        ]
        for i, action in enumerate(actions):
            atype = action.get("type", "unknown")
            lines.append(f"### STEP {i + 1} [{atype}]")
            if atype == "_code_block_":
                source = textwrap.dedent(action.get("source", "")).strip()
                lines.append(source)
            elif atype == "ask":
                lines.append(action.get("value", ""))
            else:
                for k, v in action.items():
                    if k != "type":
                        lines.append(f"{k}: {v}")
            lines.append("")  # blank separator
        return "\n".join(lines)

    @staticmethod
    def _parse_plan_from_editor(text: str):
        """Parse the temp-file format back into (actions_list, execution_mode_or_None)."""
        import re
        execution_mode = None
        # Extract execution_mode from header comment
        for line in text.splitlines():
            m = re.match(r'^#\s*execution_mode:\s*(\S+)', line)
            if m:
                val = m.group(1).strip()
                if val in ("replan", "replay"):
                    execution_mode = val
                break

        # Split into step blocks
        step_pattern = re.compile(r'^###\s+STEP\s+\d+\s+\[(\w+)\]\s*$', re.MULTILINE)
        matches = list(step_pattern.finditer(text))
        # Find ### LEARNINGS header after the last step to delimit code body
        last_step_end = matches[-1].end() if matches else 0
        learnings_header = re.search(r'^###\s+LEARNINGS\s*$', text[last_step_end:], re.MULTILINE)
        text_end = last_step_end + learnings_header.start() if learnings_header else len(text)
        actions = []
        for idx, match in enumerate(matches):
            atype = match.group(1)
            start = match.end()
            end = matches[idx + 1].start() if idx + 1 < len(matches) else text_end
            body = text[start:end].strip()
            # Strip trailing comment lines that belong to next section
            body_lines = body.splitlines()
            while body_lines and body_lines[-1].startswith("#"):
                body_lines.pop()
            body = "\n".join(body_lines).strip()
            if atype == "_code_block_":
                actions.append({"type": "_code_block_", "source": body})
            elif atype == "ask":
                actions.append({"type": "ask", "value": body})
            else:
                action = {"type": atype}
                for line in body.splitlines():
                    if ":" in line:
                        k, v = line.split(":", 1)
                        action[k.strip()] = v.strip()
                actions.append(action)
        return actions, execution_mode

    def _cmd_goal_plan_show(self, data: dict) -> str:
        goal_id = (data.get('goal_id') or '').strip()
        if not goal_id:
            return 'Usage: /goal plan <goal_id>'
        goal = self._get_scheduled_goal(goal_id)
        if not goal:
            return f"Goal '{goal_id}' not found."
        actions = goal.get("cached_plan_actions") or []
        if not actions:
            return f"Goal '{goal_id}' has no cached plan."
        header = (
            f"Goal {goal_id}: {goal.get('name', '')}\n"
            f"execution_mode: {goal.get('execution_mode', 'replan')}  |  "
            f"{len(actions)} step(s)\n"
        )
        self._say_to_user(header + "\n" + self._format_plan_for_display(actions))
        return f"Displayed {len(actions)} cached plan step(s) for {goal_id}"

    def _goal_plan_path(self, goal_id: str) -> str:
        """Return the standard edit file path for a goal plan."""
        repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        return os.path.join(repo_root, f"goal_plan_{goal_id}.py")

    def _cmd_goal_plan_edit(self, data: dict) -> str:
        goal_id = (data.get('goal_id') or '').strip()
        if not goal_id:
            return 'Usage: /goal plan edit <goal_id>'
        goal = self._get_scheduled_goal(goal_id)
        if not goal:
            return f"Goal '{goal_id}' not found."
        actions = goal.get("cached_plan_actions") or []
        if not actions:
            return f"Goal '{goal_id}' has no cached plan to edit."

        content = self._format_plan_for_editor(actions, goal)
        plan_path = self._goal_plan_path(goal_id)

        try:
            with open(plan_path, 'w') as f:
                f.write(content)
        except Exception as e:
            return f"Error writing plan file: {e}"

        self._say_to_user(
            f"Plan for goal '{goal_id}' written to:\n  {plan_path}\n\n"
            f"Edit the file in your editor, then run:\n  /goal plan load {goal_id}"
        )
        return f"Plan written to {plan_path}"

    def _cmd_goal_plan_load(self, data: dict) -> str:
        goal_id = (data.get('goal_id') or '').strip()
        if not goal_id:
            return 'Usage: /goal plan load <goal_id>'
        goal = self._get_scheduled_goal(goal_id)
        if not goal:
            return f"Goal '{goal_id}' not found."

        plan_path = self._goal_plan_path(goal_id)
        if not os.path.exists(plan_path):
            return f"No plan file found at {plan_path}. Run /goal plan edit {goal_id} first."

        try:
            with open(plan_path, 'r') as f:
                edited = f.read()
        except Exception as e:
            return f"Error reading plan file: {e}"

        new_actions, new_mode = self._parse_plan_from_editor(edited)
        if not new_actions:
            return "No steps found in file — plan unchanged."

        updates = {"cached_plan_actions": new_actions}
        if new_mode:
            updates["execution_mode"] = new_mode
        self._update_scheduled_goal(goal_id, **updates)

        # Clean up the edit file
        try:
            os.unlink(plan_path)
        except OSError:
            pass

        mode_msg = f", execution_mode set to '{new_mode}'" if new_mode else ""
        self._say_to_user(
            f"Goal '{goal_id}' plan loaded: {len(new_actions)} step(s){mode_msg}."
        )
        return f"Plan loaded: {len(new_actions)} step(s){mode_msg}"

    def _cmd_goal_plan_review(self, data: dict) -> str:
        """Generate a review bundle for a goal's cached plan and last execution."""
        goal_id = (data.get('goal_id') or '').strip()
        if not goal_id:
            return 'Usage: /goal plan review <goal_id>'
        goal = self._get_scheduled_goal(goal_id)
        if not goal:
            return f"Goal '{goal_id}' not found."

        repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        review_path = os.path.join(repo_root, f"goal_review_{goal_id}.md")
        plan_path = self._goal_plan_path(goal_id)
        trace_path = os.path.join(repo_root, f"goal_trace_{goal_id}.txt")

        sections = []

        # ── Header ──
        sections.append(f"# Plan Review: {goal_id}")
        sections.append(f"Generated: {datetime.now().isoformat(timespec='seconds')}")
        sections.append(f"Character: {self.character_name}")
        sections.append("")

        # ── Goal ──
        sections.append("## Goal")
        sections.append(f"**Name:** {goal.get('name', '')}")
        sections.append(f"**Text:** {goal.get('goal_text', '')}")
        sections.append(f"**Status:** {goal.get('status', '')}")
        sections.append(f"**Execution mode:** {goal.get('execution_mode', 'replan')}")
        sections.append(f"**Last result:** {goal.get('last_result', '(none)')}")
        sections.append(f"**Primary product:** {goal.get('primary_product', '(none)')}")
        sections.append(f"**Created:** {goal.get('created', '')}")
        sections.append(f"**Updated:** {goal.get('updated', '')}")
        sections.append("")

        # ── Run Provenance ──
        # Captures what the most recent run looked like: which path executed,
        # what the quality verdict was, and which envision config produced
        # the criteria currently on the goal record. Empty fields render as
        # "(none)" so the section stays compact for goals that haven't run yet.
        sections.append("## Run Provenance")
        last_run_mode = goal.get("last_run_mode", "") or "(none)"
        quality_status = goal.get("quality_status", "") or "(none)"
        vc_source = goal.get("vision_criteria_source", "") or "(none)"
        vc_meta = goal.get("vision_criteria_meta", {}) or {}
        sections.append(f"**Last run mode:** {last_run_mode}")
        sections.append(f"**Quality status:** {quality_status}")
        sections.append(f"**Vision criteria source:** {vc_source}")
        if isinstance(vc_meta, dict) and vc_meta:
            gen_v = vc_meta.get("generator_version", "(none)")
            gen_model = vc_meta.get("model", "(none)")
            gen_at = vc_meta.get("generated_at", "(none)")
            gen_hash = vc_meta.get("goal_text_hash", "(none)")
            sections.append(f"**Vision generator version:** {gen_v}")
            sections.append(f"**Vision generator model:** {gen_model}")
            sections.append(f"**Vision generated at:** {gen_at}")
            sections.append(f"**Vision goal_text hash:** {gen_hash}")
        else:
            sections.append("**Vision criteria meta:** (none)")
        sections.append("")

        # ── Plan ──
        actions = goal.get("cached_plan_actions") or []
        sections.append(f"## Cached Plan ({len(actions)} steps)")
        if actions:
            sections.append(f"See: `{plan_path}`")
            sections.append("")
            sections.append("```")
            sections.append(self._format_plan_for_display(actions))
            sections.append("```")
        else:
            sections.append("(no cached plan)")
        sections.append("")

        # ── Step Outcomes (per-run instrumentation) ──
        # Populated by execute_plan_sync (replay mode) and by the planner
        # entry path (planning mode resets it but the planner loop doesn't
        # currently emit per-step entries — see step_results gap discussion).
        # For replay runs this is the equivalent of the planner trace.
        step_results = goal.get("step_results") or []
        sections.append(f"## Step Outcomes ({len(step_results)} steps)")
        if step_results:
            for sr in step_results:
                if not isinstance(sr, dict):
                    continue
                idx = sr.get("step_idx", "?")
                kind = sr.get("step_kind", "?")
                status = sr.get("status", "?")
                dur = sr.get("duration_ms", "?")
                line = f"- step {idx} [{kind}] → {status} ({dur} ms)"
                exc = sr.get("exception_type")
                if exc:
                    line += f" — exception: {exc}: {sr.get('exception_message', '')}"
                elif status == "failed":
                    reason = sr.get("reason") or ""
                    if reason:
                        line += f" — reason: {reason[:200]}"
                sections.append(line)
        else:
            sections.append("(no step_results recorded — goal has not run since the instrumentation landed, or last run was a planning run)")
        sections.append("")

        # ── Vision Criteria & Quality Eval ──
        # The rubric the run was scored against, plus the eval verdict text
        # from _vision_eval_check (replay) or the planner's STAGE2 vision
        # eval (planning). Empty criteria means envision returned "no failure
        # modes apply" — that is a valid configuration, not missing data.
        vc_text = goal.get("vision_criteria", "") or ""
        last_eval = goal.get("last_quality_eval", "") or ""
        sections.append("## Vision Criteria")
        if vc_text.strip():
            sections.append("```")
            sections.append(vc_text)
            sections.append("```")
        else:
            sections.append("(no criteria — envision returned no failure modes for this goal)")
        if last_eval.strip():
            sections.append("")
            sections.append("### Last Quality Eval")
            sections.append("```")
            sections.append(last_eval)
            sections.append("```")
        sections.append("")

        # ── Scheduler Events ──
        sections.append("## Scheduler Events")
        with self._scheduler_event_lock:
            goal_events = [e for e in self._scheduler_events if e.get("goal_id") == goal_id]
        if goal_events:
            for ev in goal_events[-10:]:
                sections.append(f"- [{ev.get('ts', '')}] {ev.get('event', '')} — {ev.get('status', '')} {ev.get('result', '')}")
        else:
            sections.append("(no scheduler events found for this goal)")
        sections.append("")

        # ── Execution Log (filtered) ──
        sections.append("## Execution Log (last run)")
        log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs', 'executive_node.log')
        log_lines = self._extract_goal_log(log_path, goal_id)
        if log_lines:
            sections.append(f"({len(log_lines)} lines)")
            sections.append("```")
            sections.extend(log_lines[-200:])  # cap at 200 lines
            sections.append("```")
        else:
            sections.append("(no log entries found — log may have been rotated or goal not yet run)")
        sections.append("")

        # ── Level 1 Structural Checks ──
        sections.append("## Structural Checks")
        checks = self._run_plan_structural_checks(actions, log_lines)
        if checks:
            for check in checks:
                sections.append(f"- {check}")
        else:
            sections.append("(no issues found)")
        sections.append("")

        # ── Available Tools ──
        sections.append("## Available Tools")
        sections.append("Tools the planner can use (verify plan tool calls against this list):")
        sections.append("")
        # Infospace primitives
        from infospace_executor import INFOSPACE_PRIMITIVES
        sections.append(f"**Primitives:** {', '.join(sorted(INFOSPACE_PRIMITIVES))}")
        sections.append("")
        # World + external tools
        if hasattr(self, 'infospace_executor') and self.infospace_executor:
            ext_tools = getattr(self.infospace_executor, 'available_tools', {}) or {}
            if ext_tools:
                tool_lines = []
                for tname in sorted(ext_tools.keys()):
                    tmeta = ext_tools[tname]
                    desc = ''
                    if isinstance(tmeta, dict):
                        desc = tmeta.get('description', '')
                    if desc:
                        tool_lines.append(f"- **{tname}**: {desc[:120]}")
                    else:
                        tool_lines.append(f"- **{tname}**")
                sections.append("**World/external tools:**")
                sections.extend(tool_lines)
            else:
                sections.append("(no external tools loaded)")
        sections.append("")

        # ── Trace Reference ──
        sections.append("## Planner Trace")
        if os.path.exists(trace_path):
            trace_size = os.path.getsize(trace_path)
            trace_line_count = sum(1 for _ in open(trace_path, encoding='utf-8'))
            sections.append(f"See: `{trace_path}` ({trace_line_count} lines, {trace_size:,} bytes)")
        else:
            # In replay mode the planner trace is not produced. Point the
            # reader at the per-step instrumentation above instead of
            # apologizing for missing data.
            if step_results:
                sections.append(
                    "(no trace file — last run was replay mode, which does not "
                    "produce a planner trace; see Step Outcomes section above for "
                    "per-step execution data)"
                )
            else:
                sections.append("(no trace file — goal may not have been run in replan mode yet)")
        sections.append("")

        # ── Review Prompt Reference ──
        prompt_path = os.path.join(repo_root, "plan_review_prompt.md")
        sections.append("## Review Guidelines")
        if os.path.exists(prompt_path):
            sections.append(f"See: `{prompt_path}`")
        else:
            sections.append("(plan_review_prompt.md not found in repo root)")
        sections.append("")

        # Write bundle
        try:
            with open(review_path, 'w', encoding='utf-8') as f:
                f.write("\n".join(sections))
        except Exception as e:
            return f"Error writing review bundle: {e}"

        # Always write the plan file (overwrite with current cached plan)
        if actions:
            try:
                with open(plan_path, 'w', encoding='utf-8') as f:
                    f.write(self._format_plan_for_editor(actions, goal))
            except Exception:
                pass

        # Build the Claude prompt for copy/paste
        claude_prompt = (
            f"review goal_review_{goal_id}.md, check the trace in "
            f"goal_trace_{goal_id}.txt (planning runs only — replay runs use "
            f"the Step Outcomes section of the bundle instead), and fix the plan in "
            f"goal_plan_{goal_id}.py. Use plan_review_prompt.md for "
            f"review guidelines. Verify all tool calls in the plan against the "
            f"Available Tools section and each tool's Skill.md parameter contract. "
            f"Use the Run Provenance section to confirm which mode last ran, "
            f"the Step Outcomes section for per-step status / duration / "
            f"exceptions, and the Vision Criteria + Last Quality Eval sections "
            f"for the rubric and scored verdict. Cross-check Step Outcomes "
            f"against Cached Plan to spot steps that failed silently or "
            f"diverged from the plan. "
            f"Update the prompt file with any new patterns. "
            f"Add a ### LEARNINGS block at the end of the plan file with "
            f"actionable insights for the world model. In addition, report any "
            f"potential codebase, planner prompt or instructions, or Skill.md "
            f"updates you find. You are not required to rewrite the plan unless "
            f"failures, incomplete coverage of the goal, or significant "
            f"improvements are found."
        )

        self._say_to_user(
            f"Review bundle for '{goal_id}' written to:\n"
            f"  {review_path}\n"
            f"  {plan_path}\n"
            f"  {trace_path if os.path.exists(trace_path) else '(trace not available)'}\n\n"
            f"Claude prompt (copy/paste):\n"
            f"  {claude_prompt}\n\n"
            f"After review:\n"
            f"  /goal plan commit {goal_id}"
        )
        return f"Review bundle written to {review_path}"

    @staticmethod
    def _extract_goal_log(log_path: str, goal_id: str) -> list:
        """Extract log lines bracketed by goal start/completion markers.

        Recognizes both the legacy free-form markers and the structured
        ``[GOAL goal_X mode=... START/END]`` brackets emitted by the run-path
        instrumentation. Either form is enough to bracket a slice; the loop
        always tracks the LAST start (newer runs overwrite older slices) and
        pairs it with the next end marker that appears.
        """
        if not os.path.exists(log_path):
            return []
        try:
            with open(log_path, 'r', encoding='utf-8', errors='replace') as f:
                all_lines = f.readlines()
        except Exception:
            return []

        new_start_marker = f"[GOAL {goal_id} mode="
        new_end_substr_a = f"[GOAL {goal_id} mode="
        # Find the LAST occurrence of goal start and its matching completion
        start_idx = None
        end_idx = None
        for i, line in enumerate(all_lines):
            is_new_start = (new_start_marker in line and "START]" in line)
            is_new_end = (new_end_substr_a in line and "END " in line)
            is_old_start = (
                goal_id in line and (
                    'replay requested' in line
                    or 'proceed requested' in line
                    or 'created and started' in line
                )
            )
            is_old_end = (start_idx is not None and 'goal thread completed' in line)
            if is_new_start or is_old_start:
                start_idx = i
                end_idx = None  # reset end for this new start
            elif is_new_end or is_old_end:
                end_idx = i
        if start_idx is not None:
            end_idx = end_idx or len(all_lines) - 1
            return [line.rstrip() for line in all_lines[start_idx:end_idx + 1]]
        return []

    @staticmethod
    def _run_plan_structural_checks(actions: list, log_lines: list) -> list:
        """Run Level 1 structural checks on a plan and its execution log."""
        issues = []
        if not actions:
            return ["No cached plan to check."]

        # Check for failed steps in log
        failed_steps = [l for l in log_lines if 'failed' in l.lower() and ('Code block' in l or 'step' in l.lower())]
        for line in failed_steps:
            issues.append(f"FAILED STEP: {line.strip()[-120:]}")

        # Check for unknown action types in log
        unknown = [l for l in log_lines if 'Unknown action type' in l]
        for line in unknown:
            issues.append(f"UNKNOWN ACTION: {line.strip()[-120:]}")

        # Check for empty notes created
        empty_notes = [l for l in log_lines if 'create-note' in l.lower() and ('value": ""' in l or "value\": ''" in l)]
        for line in empty_notes:
            issues.append(f"EMPTY NOTE CREATED: {line.strip()[-120:]}")

        # Check for HTTP errors in fetch
        http_errors = [l for l in log_lines if 'HTTP' in l and ('403' in l or '404' in l or '500' in l)]
        for line in http_errors:
            issues.append(f"HTTP ERROR: {line.strip()[-120:]}")

        # Static analysis of plan code
        all_bindings_written = set()
        all_bindings_read = set()
        import re
        for i, action in enumerate(actions):
            if action.get("type") == "_code_block_":
                source = action.get("source", "")
                # Find out= bindings (writes)
                for m in re.finditer(r'\bout\s*=\s*["\'](\$\w+)["\']', source):
                    all_bindings_written.add(m.group(1))
                # Find reads via get_text / get_json / get_items helpers.
                # All three accept a $var; the previous version of this
                # checker missed get_items, producing false UNUSED BINDING
                # flags for any Collection consumed via get_items in a
                # subsequent step.
                for m in re.finditer(r'\bget_(?:text|json|items)\s*\(\s*["\'](\$\w+)["\']', source):
                    all_bindings_read.add(m.group(1))
                # Find reads via tool() keyword arguments. Any kwarg name
                # other than 'out' that takes a "$var" value is a read —
                # most common is target=, but split has source=, filter
                # has predicate-target patterns, etc. Treating all non-out
                # kwargs uniformly catches them all without requiring a
                # hardcoded list of read-shaped kwarg names.
                for m in re.finditer(r'\b(\w+)\s*=\s*["\'](\$\w+)["\']', source):
                    if m.group(1) != 'out':
                        all_bindings_read.add(m.group(2))
                # Check for say with raw content (debug leftovers)
                if re.search(r'tool\s*\(\s*["\']say["\'].*(?:str\(|content|paper_text)', source):
                    issues.append(f"Step {i+1}: possible debug 'say' dumping raw content")

        # Check for written-but-never-read bindings (excluding common sinks)
        sinks = {'$obs_result', '$kb_written', '$kb_write', '$updated_processed'}
        orphaned = all_bindings_written - all_bindings_read - sinks
        for var in sorted(orphaned):
            issues.append(f"UNUSED BINDING: {var} is written but never read in a subsequent step")

        return issues

    def _cmd_goal_plan_approve(self, data: dict) -> str:
        goal_id = (data.get('goal_id') or '').strip()
        if not goal_id:
            return 'Usage: /goal plan approve <goal_id>'
        goal = self._get_scheduled_goal(goal_id)
        if not goal:
            return f"Goal '{goal_id}' not found."
        if not goal.get("cached_plan_actions"):
            return f"Goal '{goal_id}' has no cached plan to approve."
        self._update_scheduled_goal(goal_id, execution_mode="replay")
        self._say_to_user(
            f"Goal '{goal_id}' plan approved — execution_mode set to 'replay'. "
            f"Next /goal run {goal_id} will execute the cached plan."
        )
        return f"Goal {goal_id} plan approved (replay mode)"


    def _cmd_goal_plan_commit(self, data: dict) -> str:
        """Load reviewed plan, set replay mode, inject learnings into world model."""
        import re as _re
        goal_id = (data.get('goal_id') or '').strip()
        if not goal_id:
            return 'Usage: /goal plan commit <goal_id>'
        goal = self._get_scheduled_goal(goal_id)
        if not goal:
            return f"Goal '{goal_id}' not found."

        plan_path = self._goal_plan_path(goal_id)
        if not os.path.exists(plan_path):
            return f"No plan file found at {plan_path}. Run /goal plan review {goal_id} first."

        try:
            with open(plan_path, 'r') as f:
                content = f.read()
        except Exception as e:
            return f"Error reading plan file: {e}"

        # ── Parse plan actions ──
        new_actions, new_mode = self._parse_plan_from_editor(content)
        if not new_actions:
            return "No steps found in plan file."

        # ── Extract ### LEARNINGS block ──
        learnings = []
        learnings_match = _re.search(r'^###\s+LEARNINGS\s*$(.+)', content,
                                      _re.MULTILINE | _re.DOTALL)
        if learnings_match:
            for line in learnings_match.group(1).strip().splitlines():
                line = line.strip()
                if line.startswith('- '):
                    learnings.append(line[2:].strip())
                elif line and not line.startswith('#'):
                    learnings.append(line)

        # ── Load plan + set replay mode ──
        updates = {
            "cached_plan_actions": new_actions,
            "execution_mode": new_mode or "replay",
        }
        self._update_scheduled_goal(goal_id, **updates)

        # ── Inject learnings into world model ──
        wm_injected = 0
        ti_injected = 0
        if learnings and hasattr(self, 'world_model') and self.world_model:
            # Separate tool-specific insights from general facts
            # Lines mentioning a known tool name become tool_insights
            tool_names = set()
            if hasattr(self, 'infospace_executor') and self.infospace_executor:
                tool_names = set((self.infospace_executor.available_tools or {}).keys())
            world_model_updates = []
            tool_insights = []
            for learning in learnings:
                # Check if learning starts with a tool name followed by colon
                tool_match = None
                for tn in tool_names:
                    if learning.lower().startswith(f"{tn}:") or learning.lower().startswith(f"{tn} tool:"):
                        tool_match = tn
                        break
                if tool_match:
                    insight_text = learning.split(':', 1)[1].strip() if ':' in learning else learning
                    tool_insights.append({
                        "tool": tool_match,
                        "insight": insight_text,
                        "status": "constrained",
                    })
                    ti_injected += 1
                else:
                    world_model_updates.append({
                        "fact": learning,
                        "polarity": "support",
                        "source": "plan_review",
                        "confidence": "high",
                        "weight": 4,
                    })
                    wm_injected += 1

            if world_model_updates or tool_insights:
                reflection_frame = {
                    "world_model_updates": world_model_updates,
                    "tool_insights": tool_insights,
                }
                self.world_model.update(reflection_frame)

            # ── Auto-consolidate if planner-facing belief count exceeds threshold ──
            beliefs = self.world_model.get()
            belief_facts = beliefs.get("facts", [])
            if len(belief_facts) > 30:
                # Consolidate the raw facts that produced these beliefs
                raw = self.world_model.get_raw()
                raw_facts = raw.get("raw_data", {}).get("facts", [])
                self._consolidate_world_model_facts(raw_facts)

            self.world_model.save()

        # ── Clean up plan file ──
        try:
            os.unlink(plan_path)
        except OSError:
            pass

        parts = [f"Goal '{goal_id}' committed: {len(new_actions)} step(s), replay mode."]
        if wm_injected or ti_injected:
            parts.append(f"Injected {wm_injected} facts + {ti_injected} tool insights into world model.")
        if not learnings:
            parts.append("No ### LEARNINGS block found in plan file.")

        self._say_to_user("\n".join(parts))
        return f"Goal {goal_id} committed: plan loaded, replay mode set"

    def _consolidate_world_model_facts(self, raw_facts: list):
        """Batch-consolidate world model facts using a single LLM call.

        The LLM returns merge pairs (remove→keep) so evidence counts can be
        absorbed by the surviving fact rather than silently discarded.
        """
        if not raw_facts or len(raw_facts) <= 20:
            return
        fact_texts = [f.get("fact", "") for f in raw_facts if f.get("fact")]
        if not fact_texts:
            return
        numbered = "\n".join(f"{i+1}. {t}" for i, t in enumerate(fact_texts))
        prompt = (
            "Below is a numbered list of world-model facts. Some may be duplicates, "
            "near-duplicates, or subsumed by more specific facts.\n\n"
            "For each duplicate or subsumed entry, output a line:\n"
            "  REMOVE <n> INTO <m>\n"
            "meaning fact n is a duplicate/subset of fact m and should be merged into m.\n"
            "If nothing should be removed, respond with: NONE\n\n"
            f"{numbered}"
        )
        try:
            result = self.llm_generate(
                messages=[prompt],
                max_tokens=300,
                temperature=0.0,
            )
            if not result.success or not result.text:
                return
            response = result.text.strip()
            if response.upper() == "NONE":
                return
            import re as _re
            # Parse "REMOVE <n> INTO <m>" lines
            merge_pairs = []  # (remove_idx, keep_idx)
            for match in _re.finditer(r'REMOVE\s+(\d+)\s+INTO\s+(\d+)', response, _re.IGNORECASE):
                remove_idx = int(match.group(1)) - 1  # 1-based to 0-based
                keep_idx = int(match.group(2)) - 1
                if (0 <= remove_idx < len(raw_facts) and 0 <= keep_idx < len(raw_facts)
                        and remove_idx != keep_idx):
                    merge_pairs.append((remove_idx, keep_idx))

            if not merge_pairs:
                return

            # Absorb evidence from removed facts into their survivors
            indices_to_remove = set()
            for remove_idx, keep_idx in merge_pairs:
                if remove_idx in indices_to_remove:
                    continue  # already removed by an earlier merge
                removed = raw_facts[remove_idx]
                survivor = raw_facts[keep_idx]
                survivor["support_count"] = (
                    int(survivor.get("support_count", 0)) +
                    int(removed.get("support_count", 0))
                )
                survivor["contradiction_count"] = (
                    int(survivor.get("contradiction_count", 0)) +
                    int(removed.get("contradiction_count", 0))
                )
                # Merge source_counts
                s_sc = survivor.get("source_counts") or {}
                r_sc = removed.get("source_counts") or {}
                for src, cnt in r_sc.items():
                    s_sc[src] = int(s_sc.get(src, 0)) + int(cnt)
                survivor["source_counts"] = s_sc
                # Keep earliest first_observed and latest last_observed
                if removed.get("first_observed_at", "z") < survivor.get("first_observed_at", "z"):
                    survivor["first_observed_at"] = removed["first_observed_at"]
                if removed.get("last_observed_at", "") > survivor.get("last_observed_at", ""):
                    survivor["last_observed_at"] = removed["last_observed_at"]
                indices_to_remove.add(remove_idx)

            if indices_to_remove:
                new_facts = [f for i, f in enumerate(raw_facts) if i not in indices_to_remove]
                raw_data = self.world_model.get_raw().get("raw_data", {})
                raw_data["facts"] = new_facts
                logger.info(f"World model consolidated: merged {len(indices_to_remove)} facts into survivors "
                            f"({len(raw_facts)} → {len(new_facts)})")
        except Exception as e:
            logger.warning(f"World model consolidation failed: {e}")

    # ── Goal scheduler callbacks ────────────────────────────────────

    def _record_scheduler_event(self, event: str, goal_id: str = "", goal_name: str = "", status: str = "", result: str = "", reason: str = "", dedupe_key: str = ""):
        with self._scheduler_event_lock:
            if dedupe_key and dedupe_key in self._scheduler_event_keys:
                return
            if dedupe_key:
                self._scheduler_event_keys.add(dedupe_key)
            self._scheduler_events.append({
                "ts": datetime.now().isoformat(timespec="seconds"),
                "event": event,
                "goal_id": goal_id,
                "goal_name": goal_name,
                "status": status,
                "result": (result or "")[:180],
                "reason": (reason or "")[:120],
            })
            if len(self._scheduler_events) > self._scheduler_event_limit:
                self._scheduler_events = self._scheduler_events[-self._scheduler_event_limit:]

    def _scheduler_eligible_goals(self):
        """Return scheduled goals eligible for auto-proceed."""
        from datetime import date, datetime as _dt
        # (autonomous mode removed)
        eligible = []
        for goal in self._all_scheduled_goals():
            if goal.get("is_running"):
                continue
            mode = goal.get("schedule_mode", "manual")
            if mode in ("auto", "recurring"):
                eligible.append(goal)
            elif mode == "daily":
                run_at = goal.get("run_at", "")
                last_run = goal.get("last_run_date", "")
                today = date.today().isoformat()
                if not run_at or last_run == today:
                    continue
                now = _dt.now().strftime("%H:%M")
                if now < run_at:
                    continue
                # Skip if we're past run_at + interval (missed window)
                try:
                    h, m = map(int, run_at.split(":"))
                    deadline_min = h * 60 + m + int(self.goal_scheduler.interval // 60)
                    now_min = _dt.now().hour * 60 + _dt.now().minute
                    if now_min > deadline_min:
                        self._record_scheduler_event(
                            "skip",
                            goal_id=goal.get("goal_id", ""),
                            goal_name=goal.get("name", "") or goal.get("goal_text", "")[:80],
                            reason="outside run window",
                            dedupe_key=f"skip:{goal.get('goal_id', '')}:{today}",
                        )
                        continue
                except Exception:
                    pass
                eligible.append(goal)
        return eligible

    def _scheduler_proceed_goal(self, goal_id):
        """Proceed a goal via the command registry (called from scheduler thread)."""
        from datetime import date
        goal = self._get_scheduled_goal(goal_id)
        self._scheduler_started_goals.add(goal_id)
        self._record_scheduler_event(
            "start",
            goal_id=goal_id,
            goal_name=(goal.get("name", "") if goal else "") or (goal.get("goal_text", "")[:80] if goal else ""),
        )
        if goal and goal.get("schedule_mode") == "daily":
            self._update_scheduled_goal(goal_id, last_run_date=date.today().isoformat())
        self.execution_paused = False
        self._publish_execution_state()
        self._dispatch_command({'cmd': '/goal run', 'goal_id': goal_id, 'source': 'scheduler'})

    # ── Zenoh control subscriber shims ──────────────────────────────
    # These exist for backward compatibility with the web UI and task manager,
    # which publish to individual control topics. They all delegate to
    # _dispatch_command() so there's a single code path.

    def _zenoh_to_command(self, sample, cmd: str, extra_fields: dict = None):
        """Generic shim: parse Zenoh sample, merge cmd, dispatch."""
        try:
            data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            data['cmd'] = cmd
            if extra_fields:
                data.update(extra_fields)
            self._dispatch_command(data)
        except Exception as e:
            logger.error(f'Zenoh→command shim error ({cmd}): {e}')

    def _handle_scheduler_control(self, sample):
        """Zenoh callback for scheduler enable/disable/interval changes."""
        try:
            data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            # Scheduler control has a unique payload format — translate
            if 'enable' in data:
                action = 'on' if data['enable'] else 'off'
                self._dispatch_command({'cmd': '/scheduler', 'action': action})
            if 'interval' in data:
                self._dispatch_command({'cmd': '/scheduler', 'action': 'interval', 'interval': data['interval']})
        except Exception as e:
            logger.error(f'Error in scheduler control handler: {e}')

    def _handle_goal_schedule_mode(self, sample):
        self._zenoh_to_command(sample, '/goal mode')

    def _handle_goal_rename(self, sample):
        self._zenoh_to_command(sample, '/goal rename')

    def _handle_goal_execution_mode(self, sample):
        self._zenoh_to_command(sample, '/goal exec')

    def _handle_goal_text_update(self, sample):
        self._zenoh_to_command(sample, '/goal edit')

    def _handle_goal_cache(self, sample):
        try:
            data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            self._dispatch_command({'cmd': '/goal cache clear', 'goal_id': data.get('goal_id')})
        except Exception as e:
            logger.error(f'Error in goal_cache handler: {e}')

    def _handle_goal_interrupt(self, sample):
        try:
            data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            self._dispatch_command({'cmd': '/goal terminate', 'goal_id': data.get('goal_id')})
        except Exception as e:
            logger.error(f'Error in goal_interrupt handler: {e}')

    def _handle_goal_remove(self, sample):
        self._zenoh_to_command(sample, '/goal delete')

    def _handle_task_wip_delete(self, sample):
        self._zenoh_to_command(sample, '/task delete')

    def _handle_task_wip_interrupt(self, sample):
        self._zenoh_to_command(sample, '/task interrupt')

    def _cleanup_transient_resources(self, created_ids: set, keep_ids: set = None, label: str = ""):
        """
        Delete transient resources created during a goal, preserving
        persistent resources, system resources, and any explicitly kept IDs.
        """
        if not self.resource_manager or not created_ids:
            return
        keep_ids = set(keep_ids or ())
        _PRESERVED_COLLECTIONS = {"conversation", "conversation_history", "_tasks", "_scheduled_goals"}
        _PRESERVED_NOTES = {"_situation", "_situation_prev", "_user_concerns", "_ooda_state", "_derived_concerns"}
        _PRESERVED_NOTE_PREFIXES = ("_task_", "_scheduled_goal_")
        deleted = 0
        for resource_id in created_ids:
            if resource_id in keep_ids or resource_id == "Note_null":
                continue
            resource = self.resource_manager.get_resource(resource_id)
            if not resource:
                continue
            props = resource.get("properties", {})
            if props.get("persistent", False):
                continue
            note_name = props.get("note_name", "")
            collection_name = props.get("collection_name", "")
            # Preserve named Notes — if the planner named it, it's intentional output
            if note_name:
                continue
            if collection_name in _PRESERVED_COLLECTIONS:
                continue
            success, _ = self._delete_resource_and_unbind(resource_id)
            if success:
                deleted += 1
        if deleted:
            logger.info(f"🧹 Cleaned {deleted} transient resources{' for ' + label if label else ''}")

    def _say_to_user(self, text: str):
        """Send a message to user via the say action."""
        if self.infospace_executor:
            # Strip <think>...</think> reasoning blocks before sending to user
            import re
            text = re.sub(r'<think>.*?</think>', '', text, flags=re.DOTALL).strip()
            if text:
                self.infospace_executor.execute_action({"type": "say", "target": "user", "value": text})

    # ── Task WIP (milestone loop) methods ─────────────────────────────────

    _TASK_EXECUTION_HISTORY_MAX = 20  # ring buffer size for operational task execution history

    def _handle_trigger_task(self, template_name: str, sensor_content: str, sensor_data: dict):
        """Handle a sensor trigger-task event: create a task from a template if none in progress.

        Guards:
        - If a task matching this template is already active, drop the trigger.
        - Template must exist in character config task_templates.
        """
        # Look up task template from character config
        templates = self.character_config.get('task_templates', [])
        template = None
        for t in templates:
            if t.get('name') == template_name:
                template = t
                break
        if not template:
            logger.warning(f"trigger-task: template '{template_name}' not found in task_templates config")
            return

        # Task-in-progress guard: check if a task with this template is already active
        if self.resource_manager:
            for res_id, res_data in self.resource_manager.resource_registry.items():
                if not res_id.startswith('Note_'):
                    continue
                props = res_data.get('properties', {})
                name = props.get('name', '')
                if not name or not name.startswith('_task_wip_'):
                    continue
                try:
                    content = props.get('content', '{}')
                    wip = json.loads(content) if isinstance(content, str) else content
                    if (wip.get('status') == 'active'
                            and wip.get('_trigger_template') == template_name):
                        logger.info(f"trigger-task: task for '{template_name}' already active — dropping trigger")
                        return
                except (json.JSONDecodeError, TypeError):
                    continue

        # Build task intention from template + sensor context
        intention = template.get('intention', '')
        if sensor_content:
            intention = f"SENSOR CONTEXT:\n{sensor_content}\n\nTASK:\n{intention}"

        logger.info(f"trigger-task: creating task from template '{template_name}'")
        self._begin_task_from_template(intention, template_name, template)

    def _begin_task_from_template(self, intention: str, template_name: str, template: dict):
        """Create a task WIP Note from a sensor-triggered template."""
        self._task_wip_counter += 1
        wip_id = f"twip_{self._task_wip_counter}"
        note_name = f"_task_wip_{self._task_wip_counter}"
        now = datetime.now().isoformat()
        cooldown = template.get('cooldown_seconds', 300)
        wip_content = {
            "task_wip_id": wip_id,
            "intention": intention,
            "status": "active",
            "phase": "complete",
            "milestones_completed": [],
            "current_milestone": None,
            "accumulated_findings": [],
            "created": now,
            "updated": now,
            "lifecycle": "operational",
            "completion_summary": f"Sensor-triggered from template: {template_name}",
            "establishment_milestones": [],
            "establishment_findings": [],
            "execution_history": [],
            "last_executed": None,
            "execution_count": 0,
            "cooldown_seconds": cooldown,
            "cycle_state": "idle",
            "cycle_goals_completed": [],
            "cycle_findings": [],
            "_trigger_template": template_name,
        }
        self.infospace_executor.execute_action({
            "type": "create-note",
            "value": json.dumps(wip_content),
            "name": note_name,
            "out": f"${note_name}",
        })
        self.infospace_executor.execute_action({
            "type": "persist",
            "target": f"${note_name}",
            "name": note_name,
        })
        self.active_task_wip = None
        self.active_task_wip_waiting = False
        logger.info(f'📋 Sensor-triggered task created: {note_name} (template: {template_name})')
        self._say_to_user(f"[sensor-triggered task: {template_name}]")

    def _begin_task_establishment(self, user_text: str):
        """Create a task WIP Note and transition directly to operational state.

        Acquisition and establishment phases are skipped — the task intention
        serves as the comprehensive spec.  The operational tick loop manages
        multi-goal execution from here.
        """
        self._task_wip_counter += 1
        wip_id = f"twip_{self._task_wip_counter}"
        note_name = f"_task_wip_{self._task_wip_counter}"
        now = datetime.now().isoformat()
        wip_content = {
            "task_wip_id": wip_id,
            "intention": user_text,
            "status": "active",
            "phase": "complete",
            "milestones_completed": [],
            "current_milestone": None,
            "accumulated_findings": [],
            "created": now,
            "updated": now,
            # Operational lifecycle fields
            "lifecycle": "operational",
            "completion_summary": "Direct to operational — no establishment phase",
            "establishment_milestones": [],
            "establishment_findings": [],
            "execution_history": [],
            "last_executed": None,
            "execution_count": 0,
            "cooldown_seconds": 10,  # immediate first run
            "cycle_state": "idle",
            "cycle_goals_completed": [],
            "cycle_findings": [],
        }
        # Create and persist the WIP Note
        self.infospace_executor.execute_action({
            "type": "create-note",
            "value": json.dumps(wip_content),
            "name": note_name,
            "out": f"${note_name}",
        })
        self.infospace_executor.execute_action({
            "type": "persist",
            "target": f"${note_name}",
            "name": note_name,
        })
        # No active_task_wip — we're not establishing, the tick loop handles it
        self.active_task_wip = None
        self.active_task_wip_waiting = False
        logger.info(f'📋 Task created (operational): {note_name} — "{user_text[:80]}"')
        self._say_to_user(f"Task created: {user_text[:200]}\nReady for execution.")

    @staticmethod
    def _extract_json(text: str) -> str:
        """Strip markdown fences and whitespace to extract raw JSON."""
        s = text.strip()
        # Strip ```json ... ``` or ``` ... ```
        if s.startswith("```"):
            first_nl = s.find("\n")
            if first_nl != -1:
                s = s[first_nl + 1:]
            if s.endswith("```"):
                s = s[:-3]
        return s.strip()

    def _lookup_concern_context(self, concern_id: str) -> str:
        """Return 'label — description' for a derived concern, or '' if not found."""
        try:
            for c in self._derived_concern_model.get_concerns():
                if c.get("concern_id") == concern_id:
                    label = c.get("concern_label", "")
                    desc = c.get("concern_description", "")
                    return f"{label} — {desc}" if desc else label
        except Exception:
            pass
        return ""

    def _read_task_wip(self) -> Optional[Dict[str, Any]]:
        """Read the current task WIP Note content as a dict."""
        if not self.active_task_wip or not self.resource_manager:
            return None
        try:
            note_id = self.resource_manager.named_notes.get(self.active_task_wip)
            if not note_id:
                return None
            note_data = self.resource_manager.resource_registry.get(note_id)
            if not note_data:
                return None
            content = note_data.get("properties", {}).get("content", "")
            if not content:
                return None
            return json.loads(self._extract_json(content))
        except Exception as e:
            logger.warning(f'Error reading task WIP {self.active_task_wip}: {e}')
        return None

    def _get_all_task_data(self) -> list:
        """Read all task WIP notes (establishing + operational) as parsed dicts."""
        tasks = []
        if not self.resource_manager:
            return tasks
        try:
            for name, note_id in list(self.resource_manager.named_notes.items()):
                if not name.startswith('_task_wip_'):
                    continue
                note_data = self.resource_manager.resource_registry.get(note_id)
                if not note_data:
                    continue
                content = note_data.get('properties', {}).get('content', '')
                if not content:
                    continue
                try:
                    wip = json.loads(content) if isinstance(content, str) else content
                    if isinstance(wip, dict):
                        wip['_note_name'] = name
                        tasks.append(wip)
                except (json.JSONDecodeError, TypeError):
                    continue
        except Exception as e:
            logger.warning(f'Error reading task data: {e}')
        return tasks

    def _sanitize_note_ids(self, text: str) -> str:
        """Replace literal Note IDs (e.g., Note_827) with their named equivalents.

        Prevents stale ID references from leaking into milestone history,
        accumulated findings, and goal text — where they would confuse
        subsequent milestones that should load by name instead.
        """
        if not self.resource_manager or not text:
            return text
        # Build reverse map: Note_ID → name (for named notes only)
        id_to_name = {}
        for name, note_id in self.resource_manager.named_notes.items():
            if not name.startswith('_'):  # Skip internal system notes
                id_to_name[note_id] = name
        # Replace occurrences like "Note_827" with "note-name (Note_827)"
        # or just "note-name" to keep it clean
        for note_id, name in id_to_name.items():
            if note_id in text:
                text = text.replace(note_id, f'"{name}"')
        return text

    def _update_task_wip(self, wip: Dict[str, Any]):
        """Write updated WIP content back to the Note."""
        if not self.active_task_wip or not self.resource_manager:
            return
        wip["updated"] = datetime.now().isoformat()
        try:
            note_id = self.resource_manager.named_notes.get(self.active_task_wip)
            if note_id:
                self.resource_manager.update_note_content(note_id, json.dumps(wip))
        except Exception as e:
            logger.warning(f'Error updating task WIP {self.active_task_wip}: {e}')

    _ADVANCE_TASK_PROMPT = (
        "You are ESTABLISHING a recurring task — NOT executing it yet.\n"
        "Task establishment prepares everything so the task can run autonomously later.\n"
        "You will produce milestone goals that the planner executes one at a time.\n\n"
        "IMPORTANT: Do NOT submit a goal that performs the task itself. Each milestone\n"
        "should accomplish one establishment step. The task will be executed later as a\n"
        "recurring scheduled goal.\n\n"
        "TASK INTENTION:\n{intention}\n\n"
        "CURRENT PHASE: {phase}\n\n"
        "MILESTONES COMPLETED:\n{milestones}\n\n"
        "ACCUMULATED FINDINGS:\n{findings}\n\n"
        "MOST RECENT MILESTONE RESULT:\n{last_result}\n\n"
        "{phases}\n\n"
        "PHASE RULES:\n"
        "- You may only advance to the NEXT phase (no skipping).\n"
        "- COMPLETE is only valid from the activation phase.\n"
        "- If a phase has nothing to do, submit a brief milestone that records\n"
        "  'no setup needed' and advances to the next phase.\n\n"
        "NOTE REFERENCE RULE:\n"
        "- ALWAYS reference persistent notes by NAME (e.g., `check-health-digest`),\n"
        "  NEVER by Note ID (e.g., `Note_827`). Note IDs are ephemeral — they change\n"
        "  between milestones when notes are recreated. Use load(target=\"note-name\")\n"
        "  not load(target=\"Note_NNN\"). This applies to goal text, code, and any\n"
        "  references to notes created in previous milestones.\n\n"
        "MILESTONE PROGRESSION RULE:\n"
        "- Each new milestone MUST accomplish something NOT already covered by a\n"
        "  completed milestone above. Do NOT re-verify, re-test, or re-create\n"
        "  work that already succeeded.\n"
        "- If all necessary work for the current phase is done, advance to the\n"
        "  next phase immediately. Do NOT add redundant verification milestones.\n"
        "- A test that passed does not need to be re-run. A note that was created\n"
        "  and verified does not need re-verification.\n"
        "- When in doubt, advance. Over-verification wastes resources.\n\n"
        "{pre_context}"
        "Determine what to do next. Your options are:\n"
        "1. SUBMIT_GOAL: Submit a SHORT, FOCUSED milestone goal (one establishment step).\n"
        "   Keep goal text concise — one clear objective, not a multi-step execution plan.\n"
        "2. FALL_BACK: A previous milestone result was unexpected. Describe what to revisit.\n"
        "3. COMPLETE: The task is fully established. Provide a summary.\n\n"
        "Respond in this exact format:\n"
        "ACTION: <SUBMIT_GOAL | FALL_BACK | COMPLETE>\n"
        "PHASE: <which phase this belongs to>\n"
        "GOAL_TEXT: <full goal text if SUBMIT_GOAL, or explanation if FALL_BACK/COMPLETE>\n"
        "</end>\n"
    )

    def _advance_task_wip(self):
        """Milestone decision: read WIP, call llm_generate, parse, act."""
        wip = self._read_task_wip()
        if not wip:
            logger.warning('Task WIP read failed; clearing active_task_wip')
            self.active_task_wip = None
            self._task_wip_pre_resource_ids = None
            return

        # Format milestones and findings for prompt
        milestones = wip.get("milestones_completed", [])
        def _fmt_milestone(m):
            status = m.get('status', '?')
            err = m.get('error_class', '')
            tag = f"{status}/{err}" if err else status
            return f"- [{tag}] {m.get('goal_text', '')[:120]}: {m.get('result_summary', '')[:200]}"
        milestones_text = "None yet" if not milestones else "\n".join(
            _fmt_milestone(m) for m in milestones
        )
        findings = wip.get("accumulated_findings", [])
        findings_text = "None yet" if not findings else "\n".join(f"- {f}" for f in findings)

        # Last milestone result
        if milestones:
            last = milestones[-1]
            last_result = f"[{last.get('status', '?')}] {last.get('result_summary', 'No details')[:300]}"
        else:
            last_result = "None — this is the first milestone"

        # Spiral detection: if last 2+ milestones failed in the same phase, warn the LLM
        spiral_warning = ""
        if len(milestones) >= 2:
            recent_failed = []
            for m in reversed(milestones):
                if m.get("status") == "failed":
                    recent_failed.append(m)
                else:
                    break
            if len(recent_failed) >= 2:
                spiral_warning = (
                    f"\n⚠ REPEATED FAILURES: The last {len(recent_failed)} milestones in this phase "
                    f"all failed. Do NOT retry the same approach. Instead:\n"
                    f"  - Record what was learned as a finding (e.g., 'tool X does not support Y')\n"
                    f"  - Move to the next phase with what you have, OR\n"
                    f"  - COMPLETE with a note about the limitation\n"
                    f"Do NOT submit another goal that is substantially similar to the failed ones.\n"
                )

        # Phase milestone cap: auto-advance if too many milestones in one phase.
        # Caps are per-phase — establishment phases are bounded, operational execution is not.
        _PHASE_CAPS = {
            "specification": 1,
            "capability_evaluation": 3,
            "infrastructure_setup": 5,
            "activation": 1,
        }
        current_phase = wip.get("phase", "specification")
        phase_cap = _PHASE_CAPS.get(current_phase)
        # Count consecutive completed milestones from the end
        recent_successes_in_phase = 0
        for m in reversed(milestones):
            if m.get("status") == "completed":
                recent_successes_in_phase += 1
            else:
                break
        if phase_cap and recent_successes_in_phase >= phase_cap:
            _PHASE_ORDER = ["specification", "capability_evaluation", "infrastructure_setup", "activation", "complete"]
            cur_idx = _PHASE_ORDER.index(current_phase) if current_phase in _PHASE_ORDER else 0
            if current_phase == "activation" or cur_idx >= len(_PHASE_ORDER) - 2:
                # At activation — auto-complete the task
                logger.info(f'📋 Task WIP: phase cap reached at {current_phase}, auto-completing')
                self._complete_task_wip(wip, f"Auto-completed: {recent_successes_in_phase} successful milestones in {current_phase}")
                return
            else:
                next_phase = _PHASE_ORDER[cur_idx + 1]
                logger.info(
                    f'📋 Task WIP: phase cap reached ({recent_successes_in_phase} successes in {current_phase}), '
                    f'auto-advancing to {next_phase}')
                wip["phase"] = next_phase
                wip["accumulated_findings"].append(
                    f"Phase {current_phase} capped at {recent_successes_in_phase} milestones — auto-advanced to {next_phase}")
                self._update_task_wip(wip)
                return  # Will re-enter _advance_task_wip on next tick with new phase

        # Add user concerns for milestone framing
        concerns_text = ""
        try:
            active_concerns = self.user_concern_model.get_concerns(active_only=True) or []
            if active_concerns:
                concern_lines = []
                for c in active_concerns[:5]:
                    label = c.get("concern_label", "?")
                    desc = c.get("concern_description") or ""
                    concern_lines.append(f"  - {label}: {desc}")
                concerns_text = "\n".join(concern_lines)
        except Exception:
            pass

        # Build phases section — omit specification for triage-originated tasks
        is_autonomous = bool(wip.get("linked_concern_id"))
        phases_lines = ["PHASES (in required order — you MUST visit each phase before moving to the next):"]
        if not is_autonomous:
            phases_lines.append(
                "  specification — Ask the user ONE round of clarifying questions via `ask`.\n"
                "    Combine all questions into a single ask. What exactly should the task do?\n"
                "    What parameters, thresholds, or preferences matter? You get exactly one\n"
                "    ask — make it count. The user's answers will be recorded automatically\n"
                "    in the milestone history. Then advance to capability_evaluation.")
        phases_lines.append(
            "  capability_evaluation — Test whether the needed tools/data sources actually work.\n"
            "    E.g., can we fetch the right emails? Can we access the web page? Submit a small\n"
            "    probe goal that tries the key operation and reports what it found.")
        phases_lines.append(
            "  infrastructure_setup — Create any persistent notes, collections, or other state\n"
            "    the recurring task will need. If the task mentions creating or updating a named\n"
            "    Note or Collection, create it now. E.g., create a digest collection, a tracking\n"
            "    note, a named Note for storing results. Even if the phase has minimal work,\n"
            "    submit a goal that creates/verifies the required resources exist.")
        phases_lines.append("  activation — All establishment is done. Move to COMPLETE.")
        phases_lines.append("  complete — The task is fully established and ready to be scheduled.")
        phases_text = "\n".join(phases_lines)

        # Insert concerns, spiral warning, and anti-loop nudges before the
        # format template so structured output instructions remain at the end.
        pre_context = ""
        if concerns_text:
            pre_context += f"\nUSER CONCERNS (relevant context — what the user currently cares about):\n{concerns_text}\n"
        if spiral_warning:
            pre_context += spiral_warning

        # Anti-loop: if specification phase already has completed milestones
        # with user answers, strongly instruct the LLM to advance
        if current_phase == "specification" and milestones:
            spec_completed = sum(1 for m in milestones if m.get("status") == "completed")
            if spec_completed >= 1:
                pre_context += (
                    "\n⚠ SPECIFICATION COMPLETE: The user has already answered your questions "
                    f"({spec_completed} specification milestone(s) completed). Do NOT ask more "
                    "questions. Use the answers already in ACCUMULATED FINDINGS and advance to "
                    "capability_evaluation NOW.\n"
                )

        prompt = self._ADVANCE_TASK_PROMPT.format(
            intention=wip.get("intention", ""),
            phase=wip.get("phase", "specification"),
            milestones=milestones_text,
            findings=findings_text,
            last_result=last_result,
            phases=phases_text,
            pre_context=pre_context,
        )

        try:
            with self.infospace_executor.turn_metrics.perf_phase("task_wip"):
                resp = self.llm_generate([prompt], max_tokens=1012, temperature=0.3, stops=['</end>'])
            resp_text = getattr(resp, 'text', '') if hasattr(resp, 'text') else str(resp)
        except Exception as e:
            logger.error(f'Task WIP llm_generate failed: {e}')
            self._say_to_user(f"Task milestone planning failed: {e}")
            self.active_task_wip = None
            self._task_wip_pre_resource_ids = None
            return

        # Parse response
        action_match = re.search(r'ACTION:\s*(SUBMIT_GOAL|FALL_BACK|COMPLETE)', resp_text)
        phase_match = re.search(r'PHASE:\s*(\S+)', resp_text)
        goal_match = re.search(r'GOAL_TEXT:\s*(.+)', resp_text, re.DOTALL)

        action = action_match.group(1) if action_match else None
        new_phase = phase_match.group(1) if phase_match else wip.get("phase", "specification")
        goal_text = goal_match.group(1).strip() if goal_match else ""

        logger.info(f'📋 Task WIP advance: action={action} phase={new_phase} goal_text="{goal_text[:80]}"')

        # Phase ordering enforcement
        _PHASE_ORDER = ["specification", "capability_evaluation", "infrastructure_setup", "activation", "complete"]
        current_phase = wip.get("phase", "specification")
        cur_idx = _PHASE_ORDER.index(current_phase) if current_phase in _PHASE_ORDER else 0
        new_idx = _PHASE_ORDER.index(new_phase) if new_phase in _PHASE_ORDER else cur_idx

        if action == "COMPLETE" and current_phase != "activation":
            # Block premature completion — force through remaining phases
            next_idx = min(cur_idx + 1, len(_PHASE_ORDER) - 2)  # cap at activation
            forced_phase = _PHASE_ORDER[next_idx]
            logger.info(f'📋 Task WIP: COMPLETE rejected from phase {current_phase}, advancing to {forced_phase}')
            wip["phase"] = forced_phase
            wip["accumulated_findings"].append(
                f"Skipped to {forced_phase}: COMPLETE attempted from {current_phase}"
            )
            self._update_task_wip(wip)
            # Loop will re-enter _advance_task_wip on next tick
            return

        if action == "SUBMIT_GOAL" and new_idx > cur_idx + 1:
            # Block phase skipping — cap advance to next phase
            capped_phase = _PHASE_ORDER[cur_idx + 1]
            logger.info(f'📋 Task WIP: phase skip {current_phase}→{new_phase} capped to {capped_phase}')
            new_phase = capped_phase

        if action == "SUBMIT_GOAL" and goal_text:
            # Update WIP phase and current_milestone
            wip["phase"] = new_phase
            wip["current_milestone"] = goal_text[:500]
            self._update_task_wip(wip)

            # 8.1: Enrich goal_text with task context for the planner
            # (mirrors operational ## CONTEXT ## pattern)
            context_lines = [f"Task intention: {wip.get('intention', '')[:200]}"]
            if wip.get("linked_concern_id"):
                # 8.2: Include concern context so planner knows *why*
                concern_ctx = self._lookup_concern_context(wip["linked_concern_id"])
                if concern_ctx:
                    context_lines.append(f"Driving concern: {concern_ctx}")
            for f in (wip.get("accumulated_findings") or [])[-5:]:
                context_lines.append(f"- {f[:200]}")
            for m in milestones[-3:]:
                status = m.get("status", "?")
                summary = m.get("result_summary", "")[:150]
                context_lines.append(f"- [{status}] {m.get('goal_text', '')[:100]}: {summary}")
            goal_text = f"{goal_text}\n\n## CONTEXT ##\n" + "\n".join(context_lines)

            # --- Fix A: Direct ask for specification phase ---
            # If the goal is purely "ask the user clarifying questions" during
            # specification, skip the full planner and issue the ask directly.
            # This avoids content loss where the planner returns a status message
            # instead of the user's actual answers.
            if new_phase == "specification" and self._is_ask_only_goal(goal_text):
                # Anti-loop guard: if spec already has completed milestones,
                # don't ask again — force-advance to capability_evaluation
                spec_completed = sum(
                    1 for m in wip.get("milestones_completed", [])
                    if m.get("status") == "completed"
                )
                if spec_completed >= 1:
                    logger.info(
                        f'📋 Task WIP: blocking repeated spec ask '
                        f'({spec_completed} already completed), '
                        f'force-advancing to capability_evaluation')
                    wip["phase"] = "capability_evaluation"
                    wip["accumulated_findings"].append(
                        "Specification complete (user already answered) — "
                        "auto-advanced to capability_evaluation")
                    self._update_task_wip(wip)
                    return
                self._run_direct_ask_milestone(wip, goal_text)
                return

            # Create scheduled goal with task_wip_id linkage
            scheduled_goal = self._upsert_scheduled_goal(goal_text)
            goal_id = scheduled_goal["goal_id"]
            scheduled_goal["task_wip_id"] = wip.get("task_wip_id", "")
            self._save_scheduled_goal(scheduled_goal)

            self._active_scheduled_goal_id = goal_id
            self._update_scheduled_goal(goal_id, is_running=True, status="running")
            if hasattr(self, 'goal_scheduler'):
                self.goal_scheduler._executing_goal_id = goal_id
                self.goal_scheduler._executing_is_autonomous = False
            pre_resource_ids = set(self.resource_manager.resource_registry.keys()) if self.resource_manager else set()

            def _run_milestone_goal():
                result = self.parse_and_set_goal("", goal_text) or {}
                self._set_scheduled_goal_result(goal_id, result, used_cache=False, pre_resource_ids=pre_resource_ids)
                return result

            self.active_task_wip_waiting = True
            self._run_goal_on_thread(_run_milestone_goal)

        elif action == "FALL_BACK":
            # Mark last milestone as fell_back (preserve it for context) and revert phase
            if milestones:
                milestones[-1]["status"] = "fell_back"
            wip["phase"] = new_phase
            wip["accumulated_findings"].append(f"Fell back: {goal_text[:200]}")
            self._update_task_wip(wip)
            logger.info(f'📋 Task WIP: fell back — {goal_text[:80]}')
            # Loop will re-enter _advance_task_wip on next tick

        elif action == "COMPLETE":
            self._complete_task_wip(wip, goal_text)

        else:
            logger.warning(f'Task WIP: unparseable LLM response, aborting task')
            logger.warning(f'Response was: {resp_text[:500]}')
            self._say_to_user("Task establishment failed: could not determine next step.")
            self.active_task_wip = None
            self._task_wip_pre_resource_ids = None

    def _is_ask_only_goal(self, goal_text: str) -> bool:
        """Detect whether a goal is purely about asking the user questions."""
        lower = goal_text.strip().lower()
        # Catch goals that start with ask: "..." or ask("...")
        if re.match(r'^ask\s*[:(\[]', lower):
            return True
        ask_signals = ["ask the user", "clarify", "clarifying question", "confirm with the user"]
        return any(s in lower for s in ask_signals)

    def _run_direct_ask_milestone(self, wip: Dict[str, Any], goal_text: str):
        """Execute a specification-phase ask directly, bypassing the planner.

        Sends the goal text as a question to the user, blocks for a response,
        and records the full answer text in accumulated_findings. This avoids
        content loss that occurs when the planner's codegen returns a status
        message instead of the actual user answer.
        """
        task_wip_id = wip.get("task_wip_id", "")

        def _run():
            try:
                # Strip meta-phrasing — send the substantive questions to the user
                self._say_to_user("I have some questions before I can set up this task:")
                # Use the executor's ask machinery for blocking + UI integration
                result = self.infospace_executor.execute_action({
                    "type": "ask",
                    "value": goal_text,
                    "out": "$_spec_response",
                })
                response_text = ""
                if result.get("status") == "success":
                    # Load the response content from the bound note
                    rid = result.get("resource_id", "")
                    if rid and self.infospace_executor:
                        response_text = self.infospace_executor._get_content(rid) or ""
                        if not isinstance(response_text, str):
                            response_text = str(response_text)

                success = bool(response_text)
                summary_text = response_text[:1500] if response_text else "No response received"
                clean_summary = self._sanitize_note_ids(summary_text)
                milestone_record = {
                    "goal_text": goal_text[:200],
                    "result_summary": clean_summary,
                    "status": "completed" if success else "failed",
                    "timestamp": datetime.now().isoformat(),
                }
                wip_fresh = self._read_task_wip()
                if wip_fresh:
                    wip_fresh.setdefault("milestones_completed", []).append(milestone_record)
                    wip_fresh["current_milestone"] = None
                    wip_fresh["updated"] = datetime.now().isoformat()
                    wip_fresh.setdefault("accumulated_findings", []).append(
                        f"User clarification: {clean_summary}"
                    )
                    self._update_task_wip(wip_fresh)
                logger.info(f'📋 Task WIP {task_wip_id}: direct-ask milestone completed ({len(response_text)} chars)')
            except Exception as e:
                logger.error(f'Direct-ask milestone failed: {e}')
                traceback.print_exc()
            finally:
                self.active_task_wip_waiting = False

        self.active_task_wip_waiting = True
        self._run_goal_on_thread(_run)

    # ── Operational task execution (round-robin dispatch) ────────────────

    _OPERATIONAL_TASK_PROMPT = (
        "You are executing an autonomous task. The task intention is your complete spec.\n"
        "Break the work into focused goals, executing them one at a time.\n"
        "When the task objective is fully achieved, respond with CYCLE_DONE.\n\n"
        "TASK INTENTION:\n{intention}\n\n"
        "PRIOR FINDINGS (from earlier goals in this task):\n{establishment_findings}\n\n"
        "CURRENT TIME: {current_time}\n"
        "LAST ACTUAL WORK: {last_work_time}\n\n"
        "THIS CYCLE — GOALS COMPLETED:\n{cycle_goals}\n\n"
        "THIS CYCLE — FINDINGS:\n{cycle_findings}\n\n"
        "RECENT EXECUTION HISTORY (prior cycles):\n{execution_history}\n\n"
        "RULES:\n"
        "- Each goal should be a single, focused operation.\n"
        "- Reference notes by NAME, never by Note ID.\n"
        "- When a prior goal produced a named Note, reference it by name in the next goal\n"
        "  so the planner can load it. DO NOT re-search or re-fetch data that is already\n"
        "  available in a prior goal's result summary or named artifact.\n"
        "- Do not repeat work already done in THIS cycle.\n"
        "- You MUST submit at least one goal per cycle. CYCLE_DONE is only valid AFTER\n"
        "  at least one goal has been completed in this cycle.\n"
        "- Keep goal text concise — one clear objective.\n"
        "- Include SUCCESS_CRITERIA so the result can be evaluated.\n"
        "- Include TOOLS_HINT if you know which tools are needed.\n\n"
        "Respond in this exact format:\n"
        "ACTION: <SUBMIT_GOAL | CYCLE_DONE>\n"
        "GOAL_TEXT: <the goal — what to accomplish (one sentence)>\n"
        "SUCCESS_CRITERIA: <how to tell if the goal succeeded (one sentence)>\n"
        "TOOLS_HINT: <comma-separated tool names, or 'none'>\n"
        "</end>\n"
    )

    _CYCLE_REFLECTION_PROMPT = (
        "You are evaluating the result of a goal within a task execution cycle.\n\n"
        "GOAL: {goal_text}\n"
        "SUCCESS CRITERIA: {success_criteria}\n"
        "RESULT STATUS: {status}\n"
        "RESULT SUMMARY: {result_summary}\n\n"
        "Evaluate:\n"
        "ACHIEVED: <YES | PARTIAL | NO>\n"
        "EVIDENCE: <one sentence — what in the result proves achievement or failure>\n"
        "PROGRESS: <one sentence — how this advances the overall task intention>\n"
        "NEXT: <CONTINUE | RETRY | PIVOT | CYCLE_DONE>\n"
        "REASON: <one sentence — why this next action>\n"
        "</end>\n"
    )

    _MAX_GOALS_PER_CYCLE = 5  # Safety cap per execution cycle

    def _select_next_task(self) -> Optional[Dict[str, Any]]:
        """Select the next eligible operational task for execution (round-robin by staleness)."""
        try:
            active_tasks = [
                t for t in self._get_all_task_data()
                if t.get("status") == "active"
                and t.get("lifecycle") == "operational"
            ]
            if not active_tasks:
                return None

            # Check autonomy budget — only applies to concern-initiated tasks.
            # User-initiated tasks (no linked_concern_id) bypass the budget.
            budget_exhausted = (
                hasattr(self, 'goal_scheduler')
                and self.goal_scheduler.budget_remaining() <= 0
            )

            now_ts = time.time()
            eligible = []
            for t in active_tasks:
                if budget_exhausted and t.get("linked_concern_id"):
                    continue  # Skip autonomous tasks when budget is exhausted
                cycle_state = t.get("cycle_state", "idle")
                if cycle_state == "running":
                    eligible.append(t)
                elif cycle_state in ("idle", None):
                    last = t.get("last_executed")
                    cooldown = t.get("cooldown_seconds", 3600)
                    if last is None:
                        eligible.append(t)
                    else:
                        try:
                            last_ts = datetime.fromisoformat(last.replace('+00:00', '')).timestamp()
                            if (now_ts - last_ts) > cooldown:
                                eligible.append(t)
                        except (ValueError, TypeError):
                            eligible.append(t)

            if not eligible:
                return None

            # Sort: running tasks first, then by staleness
            eligible.sort(key=lambda t: (
                0 if t.get("cycle_state") == "running" else 1,
                t.get("last_executed") or "",
            ))
            selected = eligible[0]
            logger.info(
                f'📋 Task selected: {selected.get("_note_name", "?")} '
                f'(cycle={selected.get("cycle_state", "idle")}, '
                f'executions={selected.get("execution_count", 0)}, '
                f'eligible={len(eligible)}/{len(active_tasks)})')
            return selected
        except Exception as e:
            logger.debug(f'Task selection failed: {e}')
            return None

    def _advance_task_execution(self, task_note_name: str):
        """Advance an operational task — decide and dispatch next goal for this cycle.

        The outer planning loop: reads cycle state, asks the LLM for a structured
        goal specification, dispatches it via the inner planner, and evaluates
        results with structured reflection.
        """
        if not self.resource_manager:
            return
        note_id = self.resource_manager.named_notes.get(task_note_name)
        if not note_id:
            logger.warning(f'📋 Task {task_note_name}: not found, clearing')
            self._operational_task_note = None
            return
        note_data = self.resource_manager.resource_registry.get(note_id)
        if not note_data:
            self._operational_task_note = None
            return
        try:
            wip = json.loads(note_data.get('properties', {}).get('content', '{}'))
        except (json.JSONDecodeError, TypeError):
            self._operational_task_note = None
            return

        # Initialize cycle state if starting a new cycle
        if wip.get("cycle_state") in (None, "idle"):
            wip["cycle_state"] = "running"
            wip["cycle_goals_completed"] = []
            wip["cycle_findings"] = []
            wip["_cycle_stall_sig"] = ""
            wip["_cycle_stall_count"] = 0
            logger.info(f'📋 Task {task_note_name}: starting new cycle')
            self._write_operational_task(task_note_name, note_id, wip)

        # ── User input preemption: if user has queued text, yield to it ──
        cycle_goals = wip.get("cycle_goals_completed", [])
        if cycle_goals and self.text_input_queue:
            logger.info(
                f'📋 Task {task_note_name}: user input pending — ending cycle '
                f'to yield ({len(cycle_goals)} goals completed)')
            self._complete_task_cycle(task_note_name, note_id, wip,
                                      "Cycle yielded: user input pending")
            return

        # ── Awaiting-user-guidance stall: if last goal said it needs user
        #    action, don't keep looping — end the cycle ──
        if cycle_goals:
            last_summary = (cycle_goals[-1].get("result_summary", "") or "").lower()
            awaiting_signals = ["awaiting user", "manual intervention",
                                "user guidance", "user action", "move on"]
            if any(sig in last_summary for sig in awaiting_signals):
                logger.info(
                    f'📋 Task {task_note_name}: last goal awaiting user action — ending cycle')
                self._complete_task_cycle(task_note_name, note_id, wip,
                                          "Cycle ended: awaiting user guidance")
                return

        # Check if last goal's reflection said RETRY or PIVOT
        last_reflection = wip.get("_last_reflection", {})
        reflection_next = last_reflection.get("next", "")

        # Cycle goal cap
        cycle_goals = wip.get("cycle_goals_completed", [])
        if len(cycle_goals) >= self._MAX_GOALS_PER_CYCLE:
            logger.info(f'📋 Task {task_note_name}: cycle cap ({len(cycle_goals)} goals)')
            self._complete_task_cycle(task_note_name, note_id, wip,
                                      f"Cycle capped at {len(cycle_goals)} goals")
            return

        # If reflection said CYCLE_DONE, end the cycle
        if reflection_next == "CYCLE_DONE":
            reason = last_reflection.get("reason", "Reflection determined cycle complete")
            logger.info(f'📋 Task {task_note_name}: reflection says CYCLE_DONE — {reason[:80]}')
            self._complete_task_cycle(task_note_name, note_id, wip, reason)
            return

        # Stall detection: check if we're generating the same goal repeatedly
        stall_sig = wip.get("_cycle_stall_sig", "")
        stall_count = wip.get("_cycle_stall_count", 0)

        # Format prompt context — include result summaries and artifact refs
        def _fmt_cycle_goal(g):
            line = f"- [{g.get('status', '?')}] {g.get('goal_text', '')[:150]}"
            if g.get('reflection_achieved'):
                line += f" (ACHIEVED: {g['reflection_achieved']})"
            if g.get('primary_product_name'):
                line += f"\n  → Created persistent Note \"{g['primary_product_name']}\" (available via load)"
            if g.get('result_summary'):
                line += f"\n  Result: {g['result_summary'][:400]}"
            return line
        cycle_goals_text = "None yet" if not cycle_goals else "\n".join(
            _fmt_cycle_goal(g) for g in cycle_goals
        )
        cycle_findings = wip.get("cycle_findings", [])
        cycle_findings_text = "None yet" if not cycle_findings else "\n".join(
            f"- {f}" for f in cycle_findings[-8:]  # Last 8 findings to avoid prompt bloat
        )
        establishment_findings = wip.get("establishment_findings", [])
        est_text = "None" if not establishment_findings else "\n".join(
            f"- {f}" for f in establishment_findings[:5]
        )
        exec_history = wip.get("execution_history", [])
        history_text = "None" if not exec_history else "\n".join(
            f"- Cycle #{i+1}: {h.get('summary', '')[:150]}"
            for i, h in enumerate(exec_history[-3:])
        )

        # If reflection said PIVOT, inject that context
        pivot_context = ""
        if reflection_next == "PIVOT":
            pivot_context = (
                f"\n⚠ PREVIOUS GOAL FAILED — reflection recommended PIVOT.\n"
                f"Reason: {last_reflection.get('reason', 'unknown')}\n"
                f"Do NOT retry the same approach. Try a different strategy.\n"
            )

        # Compute last actual work time (last cycle that submitted at least one goal)
        last_work_time = "Never"
        for h in reversed(exec_history):
            if h.get("goals_count", 0) > 0:
                last_work_time = h.get("timestamp", "Unknown")
                break

        prompt = self._OPERATIONAL_TASK_PROMPT.format(
            intention=wip.get("intention", ""),
            establishment_findings=est_text,
            current_time=datetime.now().isoformat(timespec='minutes'),
            last_work_time=last_work_time,
            cycle_goals=cycle_goals_text,
            cycle_findings=cycle_findings_text,
            execution_history=history_text,
        )
        if pivot_context:
            prompt += pivot_context

        try:
            with self.infospace_executor.turn_metrics.perf_phase("task_exec"):
                resp = self.llm_generate([prompt], max_tokens=1012, temperature=0.3, stops=['</end>'])
            resp_text = getattr(resp, 'text', '') if hasattr(resp, 'text') else str(resp)
        except Exception as e:
            logger.error(f'📋 Task {task_note_name}: advance LLM failed: {e}')
            self._operational_task_note = None
            return

        # Parse structured response
        action_match = re.search(r'ACTION:\s*(SUBMIT_GOAL|CYCLE_DONE)', resp_text)
        goal_match = re.search(r'GOAL_TEXT:\s*(.+?)(?:\nSUCCESS_CRITERIA:|\nTOOLS_HINT:|\Z)', resp_text, re.DOTALL)
        criteria_match = re.search(r'SUCCESS_CRITERIA:\s*(.+?)(?:\nTOOLS_HINT:|\Z)', resp_text, re.DOTALL)
        tools_match = re.search(r'TOOLS_HINT:\s*(.+?)(?:\n|$)', resp_text)

        action = action_match.group(1) if action_match else None
        goal_text = goal_match.group(1).strip() if goal_match else ""
        success_criteria = criteria_match.group(1).strip() if criteria_match else ""
        tools_hint = tools_match.group(1).strip() if tools_match else ""

        logger.info(
            f'📋 Task {task_note_name}: advance decision={action} '
            f'goal="{goal_text[:60]}" criteria="{success_criteria[:60]}" '
            f'tools={tools_hint[:40]}')

        if action == "SUBMIT_GOAL" and goal_text:
            # Stall detection: normalize goal text and compare to previous
            normalized = re.sub(r'[^a-z0-9 ]', '', goal_text.lower())[:200]
            # Check exact match OR high word overlap (fuzzy stall)
            is_stall = False
            if stall_sig:
                if normalized == stall_sig:
                    is_stall = True
                else:
                    # Fuzzy: if >70% of words overlap, count as same goal
                    prev_words = set(stall_sig.split())
                    curr_words = set(normalized.split())
                    if prev_words and curr_words:
                        overlap = len(prev_words & curr_words)
                        max_len = max(len(prev_words), len(curr_words))
                        if max_len > 0 and overlap / max_len > 0.7:
                            is_stall = True
            if is_stall:
                stall_count += 1
                wip["_cycle_stall_count"] = stall_count
                if stall_count >= 2:
                    logger.warning(
                        f'📋 Task {task_note_name}: STALL detected — '
                        f'similar goal generated {stall_count + 1} times, ending cycle')
                    self._complete_task_cycle(task_note_name, note_id, wip,
                                              f"Stall: similar goal repeated {stall_count + 1} times")
                    return
            else:
                wip["_cycle_stall_sig"] = normalized
                wip["_cycle_stall_count"] = 0

            # Store goal spec for later reflection
            wip["current_milestone"] = goal_text[:500]
            wip["_current_success_criteria"] = success_criteria[:300]
            wip["_current_tools_hint"] = tools_hint[:200]
            wip["_last_reflection"] = {}
            self._write_operational_task(task_note_name, note_id, wip)

            # Append task intention + cycle context so the planner knows the
            # overarching task directives and what prior goals produced.
            intention = wip.get('intention', '')
            ctx_parts = []
            if intention:
                ctx_parts.append(f"TASK INTENTION (follow these directives):\n{intention[:2000]}")
            if cycle_goals:
                context_lines = []
                artifact_lines = []
                for g in cycle_goals:
                    summary = g.get('result_summary', '')[:400]
                    g_text = g.get('goal_text', '')[:100]
                    if summary:
                        context_lines.append(f"- [{g.get('status', '?')}] {g_text}\n  Result: {summary}")
                    if g.get('primary_product_name'):
                        artifact_lines.append(
                            f"- \"{g['primary_product_name']}\" — created by prior goal, load with: "
                            f"tool(\"load\", target=\"{g['primary_product_name']}\")")
                if context_lines:
                    ctx_parts.append("Prior goals completed in this cycle:\n" + "\n".join(context_lines))
                if artifact_lines:
                    ctx_parts.append(
                        "AVAILABLE ARTIFACTS (persistent Notes from prior goals — use these, do NOT redo the work):\n"
                        + "\n".join(artifact_lines))
            if ctx_parts:
                goal_text = f"{goal_text}\n\n## CONTEXT ##\n" + "\n\n".join(ctx_parts)

            # Run goal on thread
            pre_resource_ids = set(self.resource_manager.resource_registry.keys()) if self.resource_manager else set()

            def _run_operational_goal():
                logger.info(f'📋 {task_note_name} → launching goal: {goal_text[:200]}')
                result = self.parse_and_set_goal("", goal_text) or {}
                success = result.get('success', False)
                pp = result.get('primary_product', '')
                resp = (result.get('response', '') or '')[:200]
                logger.info(f'📋 {task_note_name} → goal result: success={success} product={pp} response={resp}')
                if pre_resource_ids is not None and self.resource_manager:
                    now_ids = set(self.resource_manager.resource_registry.keys())
                    created_ids = now_ids - pre_resource_ids
                    primary = result.get('primary_product', '')
                    keep = {primary} if primary else set()
                    self._cleanup_transient_resources(created_ids, keep,
                                                      label=f'op_{task_note_name}')
                    self.resource_manager.flush_deferred_indexes()
                return result

            self._operational_task_note = task_note_name
            self._operational_goal_waiting = True

            if hasattr(self, 'goal_scheduler'):
                self.goal_scheduler._executing_goal_id = f"op_{task_note_name}"
                if wip.get('linked_concern_id'):
                    self.goal_scheduler._executing_is_autonomous = True
                    self.goal_scheduler._autonomous_start_time = time.monotonic()

            self._run_goal_on_thread(_run_operational_goal)

        elif action == "CYCLE_DONE":
            # Reject zero-goal CYCLE_DONE — recurring tasks must do work each cycle
            if not cycle_goals:
                logger.warning(
                    f'📋 Task {task_note_name}: rejected zero-goal CYCLE_DONE, '
                    f'ending cycle without updating last_executed or execution_count')
                wip["cycle_state"] = "idle"
                wip["cycle_goals_completed"] = []
                wip["cycle_findings"] = []
                wip["current_milestone"] = None
                wip["_last_reflection"] = {}
                wip["_cycle_stall_sig"] = ""
                wip["_cycle_stall_count"] = 0
                self._write_operational_task(task_note_name, note_id, wip)
                self._operational_task_note = None
                self._operational_goal_waiting = False
                return
            logger.info(f'📋 Task {task_note_name}: LLM says CYCLE_DONE — "{goal_text[:80]}"')
            self._complete_task_cycle(task_note_name, note_id, wip, goal_text)

        else:
            logger.warning(f'📋 Task {task_note_name}: unparseable advance — "{resp_text[:200]}"')
            self._complete_task_cycle(task_note_name, note_id, wip,
                                      "Cycle ended: unparseable advance response")

    def _write_operational_task(self, note_name: str, note_id: str, wip: Dict[str, Any]):
        """Write updated operational task WIP back to its Note."""
        wip["updated"] = datetime.now().isoformat()
        try:
            self.resource_manager.update_note_content(note_id, json.dumps(wip))
        except Exception as e:
            logger.warning(f'Error updating operational task {note_name}: {e}')

    def _record_operational_goal_result(self):
        """Called when an operational goal completes. Runs structured reflection."""
        if not self._operational_task_note or not self.resource_manager:
            self._operational_task_note = None
            return
        note_name = self._operational_task_note
        note_id = self.resource_manager.named_notes.get(note_name)
        if not note_id:
            self._operational_task_note = None
            return
        try:
            note_data = self.resource_manager.resource_registry.get(note_id)
            wip = json.loads(note_data.get('properties', {}).get('content', '{}'))
        except Exception:
            self._operational_task_note = None
            return

        # Extract result from the completed goal
        result = self._goal_thread_result or {}
        success = result.get('success', False)
        response = result.get('response', '') or result.get('text', '') or ''
        if not response and isinstance(result, dict):
            response = result.get('primary_product', '') or str(result)[:1200]
        clean_response = self._sanitize_note_ids(str(response)[:1200])

        goal_text = wip.get("current_milestone", "")
        success_criteria = wip.get("_current_success_criteria", "")
        status_str = "completed" if success else "failed"

        # ── Structured reflection ────────────────────────────────────────
        reflection = {"achieved": "?", "evidence": "", "progress": "",
                      "next": "CONTINUE", "reason": ""}
        try:
            refl_prompt = self._CYCLE_REFLECTION_PROMPT.format(
                goal_text=goal_text[:300],
                success_criteria=success_criteria[:200],
                status=status_str,
                result_summary=clean_response[:800],
            )
            with self.infospace_executor.turn_metrics.perf_phase("task_reflect"):
                refl_resp = self.llm_generate([refl_prompt], max_tokens=712, temperature=0.2, stops=['</end>'])
            refl_text = getattr(refl_resp, 'text', '') if hasattr(refl_resp, 'text') else str(refl_resp)

            # Parse reflection fields
            for field, key in [('ACHIEVED', 'achieved'), ('EVIDENCE', 'evidence'),
                               ('PROGRESS', 'progress'), ('NEXT', 'next'), ('REASON', 'reason')]:
                m = re.search(rf'{field}:\s*(.+?)(?:\n|$)', refl_text)
                if m:
                    reflection[key] = m.group(1).strip()
            # Normalize NEXT to expected values
            next_val = reflection["next"].upper()
            if next_val not in ("CONTINUE", "RETRY", "PIVOT", "CYCLE_DONE"):
                reflection["next"] = "CONTINUE"
            else:
                reflection["next"] = next_val
        except Exception as e:
            logger.debug(f'📋 Task {note_name}: reflection failed: {e}')

        logger.info(
            f'📋 Task {note_name}: goal {status_str} | '
            f'achieved={reflection["achieved"]} | '
            f'next={reflection["next"]} | '
            f'evidence="{reflection["evidence"][:60]}"')

        # ── Record goal with reflection ──────────────────────────────────
        # Resolve primary_product to a persistent named Note if possible
        primary_product_id = result.get('primary_product', '')
        primary_product_name = ''
        if primary_product_id and self.resource_manager:
            for name, nid in self.resource_manager.named_notes.items():
                if nid == primary_product_id and not name.startswith('_'):
                    primary_product_name = name
                    break

        goal_record = {
            "goal_text": goal_text[:200],
            "result_summary": clean_response[:800],
            "status": status_str,
            "timestamp": datetime.now().isoformat(),
            "success_criteria": success_criteria[:200],
            "reflection_achieved": reflection["achieved"],
            "reflection_next": reflection["next"],
        }
        if primary_product_name:
            goal_record["primary_product_name"] = primary_product_name
        wip.setdefault("cycle_goals_completed", []).append(goal_record)

        # Add reflection evidence as a finding (more useful than raw summary)
        if reflection["evidence"]:
            wip.setdefault("cycle_findings", []).append(
                f'{reflection["achieved"]}: {reflection["evidence"][:200]}')
        elif clean_response and len(clean_response) > 10:
            wip.setdefault("cycle_findings", []).append(
                f"Goal result: {clean_response[:200]}")

        # Store reflection for next advance call to act on
        wip["_last_reflection"] = reflection
        wip["current_milestone"] = None

        # Record autonomous budget
        if hasattr(self, 'goal_scheduler'):
            self.goal_scheduler._record_autonomous_completion()
            self.goal_scheduler._executing_is_autonomous = False

        self._write_operational_task(note_name, note_id, wip)
        # Note: _operational_task_note stays set so next tick re-advances this task

    def _complete_task_cycle(self, note_name: str, note_id: str,
                             wip: Dict[str, Any], summary: str):
        """Complete one execution cycle of an operational task."""
        cycle_goals = wip.get("cycle_goals_completed", [])
        achieved_count = sum(1 for g in cycle_goals if g.get("reflection_achieved", "").startswith("YES"))
        cycle_record = {
            "timestamp": datetime.now().isoformat(),
            "goals_count": len(cycle_goals),
            "goals_achieved": achieved_count,
            "summary": self._sanitize_note_ids(summary[:300]),
            "outcome": "completed",
        }
        history = wip.get("execution_history", [])
        history.append(cycle_record)
        if len(history) > 20:
            history = history[-20:]
        wip["execution_history"] = history
        wip["execution_count"] = wip.get("execution_count", 0) + 1
        wip["last_executed"] = datetime.now().isoformat()
        wip["cycle_state"] = "idle"
        # H2: Promote durable cycle findings to establishment_findings before clearing
        cycle_findings = wip.get("cycle_findings", [])
        if cycle_findings:
            est = wip.setdefault("establishment_findings", [])
            est.extend(f"[cycle] {f}" for f in cycle_findings)
            wip["establishment_findings"] = est[-20:]  # cap at 20, oldest evicted
        wip["cycle_goals_completed"] = []
        wip["cycle_findings"] = []
        wip["current_milestone"] = None
        wip["_last_reflection"] = {}
        wip["_cycle_stall_sig"] = ""
        wip["_cycle_stall_count"] = 0

        # Tasks are one-shot: mark completed after cycle finishes.
        # The concern's revisit mechanism handles recurrence — when the
        # concern reactivates, triage will create a fresh task.
        wip["status"] = "completed"
        wip["cycle_state"] = "done"

        # Satisfy driving concern when cycle succeeds
        if wip.get("linked_concern_id"):
            try:
                self._derived_concern_model._apply_patch({
                    "op": "satisfy_concern",
                    "concern_id": wip["linked_concern_id"],
                    "field_updates": {
                        "status_rationale": f"Task completed ({achieved_count} goals achieved): {summary[:100]}",
                        "revisit_hours": 24,  # default revisit; concern model decides when to resurface
                    },
                }, f"task_cycle:{note_name}")
                self._derived_concern_model._save()
            except Exception as e:
                logger.debug(f"Concern satisfaction after task completion failed: {e}")

        self._write_operational_task(note_name, note_id, wip)
        self._operational_task_note = None
        self._operational_goal_waiting = False
        logger.info(
            f'📋 Task {note_name}: completed '
            f'({achieved_count}/{len(cycle_goals)} goals achieved) — {summary[:80]}')

    def _complete_task_wip(self, wip: Dict[str, Any], summary: str):
        """Finalize task establishment: set up operational state for tick-loop dispatch.

        Does NOT create a scheduled goal. The tick loop manages operational
        execution directly via _select_next_task / _advance_task_execution.
        """
        intention = wip.get("intention", "")

        # Determine cooldown based on task origin
        # User-initiated tasks (no linked concern) get a short cooldown;
        # concern-delegated tasks get longer cooldowns for background work.
        cooldown = 10  # default 10s for user-initiated tasks
        if wip.get("linked_concern_id"):
            # Check if it's a seed concern (longer cooldown)
            try:
                for c in self._derived_concern_model.get_concerns():
                    if c.get("concern_id") == wip.get("linked_concern_id"):
                        if c.get("seeded"):
                            label = c.get("concern_label", "")
                            if "knowledge" in label.lower() or "improvement" in label.lower():
                                cooldown = 7200  # 2 hours for knowledge improvement
                            else:
                                cooldown = 3600  # 1 hour for other seed concerns
                        else:
                            cooldown = 1800  # 30 min for event-triggered derived concerns
                        break
            except Exception:
                pass

        # Transition WIP Note to active (operational) lifecycle
        wip["status"] = "active"
        wip["phase"] = "complete"
        wip["completion_summary"] = summary
        wip["lifecycle"] = "operational"
        wip["establishment_milestones"] = wip.get("milestones_completed", [])
        wip["establishment_findings"] = wip.get("accumulated_findings", [])
        wip["execution_history"] = []
        wip["last_executed"] = None
        wip["execution_count"] = 0
        wip["cooldown_seconds"] = cooldown
        # Cycle state (managed by _advance_task_execution)
        wip["cycle_state"] = "idle"
        wip["cycle_goals_completed"] = []
        wip["cycle_findings"] = []
        self._update_task_wip(wip)

        # Clean up establishment artifacts (no operational_goal_id to exclude)
        self._cleanup_task_establishment_simple(wip)

        self.active_task_wip = None
        self.active_task_wip_waiting = False
        op_name = intention[:60]
        logger.info(f'📋 Task established: {op_name} (cooldown={cooldown}s, tick-managed)')
        self._say_to_user(
            f"Task established: {op_name}\n"
            f"Cooldown: {cooldown}s\n"
            f"Execution: managed by tick loop (round-robin)"
        )

    def _cleanup_task_establishment(self, wip: Dict[str, Any], operational_goal_id: str):
        """Remove milestone goals and transient resources from task establishment.

        Keeps:
        - The final operational scheduled goal (operational_goal_id)
        - The WIP note itself (referenced by task_context_note)
        - Any persistent resources created during infrastructure_setup
        - System collections (conversation, _tasks, _scheduled_goals, etc.)
        """
        task_wip_id = wip.get("task_wip_id", "")

        # 1. Delete milestone scheduled goals (not the final operational one)
        deleted_goals = 0
        for goal in self._all_scheduled_goals():
            if goal.get("task_wip_id") == task_wip_id and goal["goal_id"] != operational_goal_id:
                self._delete_scheduled_goal(goal["goal_id"])
                deleted_goals += 1

        # 2. Delete transient resources created during establishment
        pre_ids = getattr(self, "_task_wip_pre_resource_ids", None)
        if pre_ids is not None and self.resource_manager:
            now_ids = set(self.resource_manager.resource_registry.keys())
            created_ids = now_ids - pre_ids

            # Identify resources to keep
            wip_note_name = self.active_task_wip or ""
            wip_note_id = self.resource_manager.named_notes.get(wip_note_name, "")
            keep_ids = {wip_note_id} if wip_note_id else set()

            deleted_resources = 0
            for resource_id in created_ids:
                if resource_id in keep_ids:
                    continue
                resource = self.resource_manager.get_resource(resource_id)
                if not resource:
                    continue
                props = resource.get("properties", {})
                # Keep persistent resources (created during infrastructure_setup)
                if props.get("persistent", False):
                    continue
                # Keep scheduled goal notes (managed separately above)
                note_name = props.get("note_name", "")
                if note_name and note_name.startswith("_scheduled_goal_"):
                    continue
                success, _ = self._delete_resource_and_unbind(resource_id)
                if success:
                    deleted_resources += 1

            self._task_wip_pre_resource_ids = None
            logger.info(
                f"🧹 Task establishment cleanup: {deleted_goals} milestone goals, "
                f"{deleted_resources} transient resources removed"
            )
        elif deleted_goals:
            logger.info(f"🧹 Task establishment cleanup: {deleted_goals} milestone goals removed")

    def _cleanup_task_establishment_simple(self, wip: Dict[str, Any]):
        """Clean up establishment artifacts when no operational scheduled goal is created.

        Deletes ALL milestone goals for this task and transient resources.
        Used by the tick-managed execution model where _complete_task_wip
        does not create a scheduled goal.
        """
        task_wip_id = wip.get("task_wip_id", "")

        # Delete ALL milestone scheduled goals for this task
        deleted_goals = 0
        for goal in self._all_scheduled_goals():
            if goal.get("task_wip_id") == task_wip_id:
                self._delete_scheduled_goal(goal["goal_id"])
                deleted_goals += 1

        # Delete transient resources
        pre_ids = getattr(self, "_task_wip_pre_resource_ids", None)
        if pre_ids is not None and self.resource_manager:
            now_ids = set(self.resource_manager.resource_registry.keys())
            created_ids = now_ids - pre_ids
            wip_note_name = self.active_task_wip or ""
            wip_note_id = self.resource_manager.named_notes.get(wip_note_name, "")
            keep_ids = {wip_note_id} if wip_note_id else set()
            deleted_resources = 0
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
                if note_name and note_name.startswith("_scheduled_goal_"):
                    continue
                success, _ = self._delete_resource_and_unbind(resource_id)
                if success:
                    deleted_resources += 1
            self._task_wip_pre_resource_ids = None
            logger.info(
                f"🧹 Task establishment cleanup: {deleted_goals} milestone goals, "
                f"{deleted_resources} transient resources removed")
        elif deleted_goals:
            logger.info(f"🧹 Task establishment cleanup: {deleted_goals} milestone goals removed")

    # ── Chat-mode response (lightweight, no planning pipeline) ────────────

    def _handle_chat_response(self, text: str, source: str = 'User', assessment: Optional[Dict[str, Any]] = None):
        """Unified handler: respond to user text as chat, goal, or system command.

        Single LLM call that decides the response type (replaces the former
        interpret → envision → chat three-call pipeline).
        Assessment is provided by the Orient stage of the OODA pipeline.
        """
        self.conversation_store.record_incoming(source, text)

        # Build system prompt (character + setting + capabilities + drives + agent state)
        system_prompt = self._update_system_prompt()

        # Build orientation summary from evaluator assessment
        orientation = character_evaluator.build_orientation_summary(assessment, text)

        # Recent dialog history + prior session backfill
        recent_turns = ""
        entity_data = self.conversation_store.get_entity_context(source, limit=20, scope='current')
        if entity_data and 'conversation_history' in entity_data:
            for entry in entity_data['conversation_history'][-20:]:
                if isinstance(entry, dict) and 'source' in entry and 'text' in entry:
                    text_preview = str(entry['text'])[:200]
                    recent_turns += f"{entry['source']}: {text_preview}\n"
        # Prepend prior session summaries if available (oldest first for chronological order)
        # Filter out summaries tagged with goal_ids of completed/failed goals
        prior_summaries = entity_data.get('prior_session_summaries', []) if entity_data else []
        if prior_summaries:
            done_goal_ids = set()
            try:
                for g in self._all_scheduled_goals():
                    if g.get('status') in ('completed', 'failed'):
                        done_goal_ids.add(g.get('goal_id', ''))
                done_goal_ids.discard('')
            except Exception:
                pass
            prior_block = "PRIOR SESSIONS:\n"
            for ps in reversed(prior_summaries):
                if ps.get('goal_id') and ps['goal_id'] in done_goal_ids:
                    continue  # Skip summaries from completed goals
                prior_block += f"  [session summary] {ps['text'][:300]}\n"
            recent_turns = prior_block + "\n" + recent_turns

        # Render operational self-model for self-aware responses
        self_model_block = ""
        try:
            from ooda_snapshot_renderer import render_self_model_section
            dc = getattr(self, '_derived_concern_model', None)
            self_model_block = render_self_model_section(
                scheduler_status=self.goal_scheduler.get_status() if hasattr(self, 'goal_scheduler') else {},
                tasks=self._get_all_task_data(),
                derived_concerns=dc.get_concerns() if dc else [],
                scheduled_goals=list(self._all_scheduled_goals()),
                sensor_configs=self.sensor_configs,
                execution_mode=getattr(self, 'execution_mode', 'step'),
                tool_count=len(self.infospace_executor.available_tools) if self.infospace_executor else 0,
            )
        except Exception:
            pass

        # User concerns for grounding
        user_concerns_block = ""
        try:
            active_uc = self.user_concern_model.get_concerns(active_only=True) or []
            if active_uc:
                lines = ["## User Concerns"]
                for c in active_uc[:8]:
                    label = c.get("concern_label", "?")
                    desc = c.get("concern_description", "")
                    weight = c.get("weight", "")
                    status = c.get("status", "")
                    w_str = f" (weight={weight})" if weight else ""
                    lines.append(f"- {label}{w_str} [{status}]: {desc}")
                user_concerns_block = "\n".join(lines)
        except Exception:
            pass

        # Compact goals/tasks context for command resolution
        command_context = self._build_command_resolution_context()

        orientation_block = f"\n{orientation}\n\n" if orientation else "\n"
        self_model_insert = f"\n{self_model_block}\n\n" if self_model_block else ""
        concerns_insert = f"\n{user_concerns_block}\n\n" if user_concerns_block else ""
        user_prompt = (
            f"RECENT DIALOG:\n{recent_turns}\n"
            f"{orientation_block}"
            f"{self_model_insert}"
            f"{concerns_insert}"
            f"{command_context}\n"
            f"Message from {source}: {text}\n\n"
            f"Respond directly to what {source} said. Ground your response in your "
            f"actual operational state (concerns, tasks, recent goals, self-model) "
            f"rather than generic descriptions. Be concise and in character.\n"
            f"IMPORTANT: You CANNOT execute tools or run actions in this conversational turn. "
            f"NEVER claim to have run a tool, executed a check, or performed an action that "
            f"you did not actually perform.\n\n"
            f"RESPONSE MODES — choose exactly one:\n"
            f"1. CHAT: If the message is conversational, respond naturally.\n"
            f"2. GOAL: If the request requires tool execution (web search, file access, "
            f"email, code execution, etc.), respond ONLY with:\n"
            f"   [GOAL_NEEDED: <concise goal description>]\n"
            f"   Do NOT ask for confirmation — the system runs the goal automatically. "
            f"   Include enough detail for independent execution.\n"
            f"3. COMMAND: If the message maps to a system command (e.g., 'stop', 'run the "
            f"weather goal', 'save'), respond ONLY with:\n"
            f"   [COMMAND: </command with args>]\n"
            f"   Resolve references ('the weather goal') to actual IDs from the state above. "
            f"   If you cannot resolve a reference, treat as chat.\n"
            f"You may include a brief conversational preamble before a GOAL or COMMAND marker.\n"
            f"End your response with </end>"
        )

        try:
            result = self.llm_generate(
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt},
                ],
                max_tokens=400,
                temperature=0.7,
                stops=["</end>"],
            )
            if result.success and result.text:
                response = result.text
                # Truncate at </end or prompt-echo markers
                for marker in ('</end>', '</end', '\nUSER:', '\nASSISTANT:',
                               '\n## ORIENTATION', '\nRECENT DIALOG:',
                               '\n#Objectives', '\n#Constraints', '\n#Format',
                               '\nMessage from '):
                    idx = response.find(marker)
                    if idx >= 0:
                        response = response[:idx]
                response = response.strip()
                if not response:
                    logger.warning('Chat response empty after cleaning')
                else:
                    self._route_chat_response(response, source)
            else:
                logger.warning(f'Chat LLM call failed: {getattr(result, "error", "unknown")}')
        except Exception as e:
            logger.error(f'Error in chat response: {e}')
            traceback.print_exc()

    def _route_chat_response(self, response: str, source: str):
        """Route LLM response: extract GOAL_NEEDED/COMMAND markers or deliver as chat."""
        import re as _re

        # Check for [COMMAND: ...] marker
        cmd_match = _re.search(r'\[COMMAND:\s*(.+?)\]', response)
        if cmd_match:
            cmd_line = cmd_match.group(1).strip()
            clean_response = response[:cmd_match.start()].strip()
            if clean_response:
                self._say_to_user(clean_response)
                self.conversation_store.record_outgoing(source, clean_response, act_type="chat")
            from cli import _parse_command
            parsed = _parse_command(cmd_line)
            if parsed:
                parsed['source'] = 'User'
                self._say_to_user(f'[command: {cmd_line}]')
                logger.info(f'🔧 Chat dispatched command: {cmd_line}')
                self._dispatch_command(parsed)
            else:
                logger.warning(f'Chat emitted unparseable command: {cmd_line}')
                self._say_to_user(f'Could not parse command: {cmd_line}')
            return

        # Check for [GOAL_NEEDED: ...] marker
        goal_match = _re.search(r'\[GOAL_NEEDED:\s*(.+?)\]', response)
        if goal_match:
            goal_text = goal_match.group(1).strip()
            clean_response = response[:goal_match.start()].strip()
            if clean_response:
                self._say_to_user(clean_response)
                self.conversation_store.record_outgoing(source, clean_response, act_type="chat")
            self._say_to_user(f'[using tools: {goal_text[:120]}]')
            logger.info(f'🎯 Chat auto-escalated to goal: "{goal_text[:80]}"')
            cmd_data = {'cmd': '/goal add', 'goal_text': goal_text, 'source': 'User'}
            self._dispatch_command(cmd_data)
            return

        # Plain chat response
        self._say_to_user(response)
        self.conversation_store.record_outgoing(source, response, act_type="chat")
        logger.info(f'💬 {self.character_name} chat response to {source}: {response[:80]}...')

    def _build_command_resolution_context(self) -> str:
        """Build compact goals/tasks/concerns context for command resolution."""
        lines = ["## SYSTEM COMMANDS"]
        lines.append("Available commands (use with [COMMAND: ...] marker):")
        lines.append("  /goal add <text>, /goal run <goal_id>, /goal terminate <goal_id>")
        lines.append("  /goal delete <goal_id>, /goal mode <goal_id> manual|auto|recurring|daily")
        lines.append("  /task approve <name>, /task abandon <name>, /task run <name>")
        lines.append("  /concern close <id>, /concern reopen <id>")
        lines.append("  /stop, /continuous, /save, /shutdown, /bye")

        # Current goals for ID resolution
        try:
            scheduled = list(self._all_scheduled_goals())
            if scheduled:
                lines.append("Current goals:")
                for g in scheduled[:8]:
                    gid = g.get('goal_id', '?')
                    name = (g.get('name') or g.get('goal_text', '?'))[:60]
                    status = g.get('status', '?')
                    running = ' [RUNNING]' if g.get('is_running') else ''
                    lines.append(f"  {gid} [{status}]{running} \"{name}\"")
        except Exception:
            pass

        # Current tasks for name resolution
        try:
            tasks = self._get_all_task_data()
            active_tasks = [t for t in tasks if t.get('status') not in ('abandoned', 'archived')]
            if active_tasks:
                lines.append("Current tasks:")
                for t in active_tasks[:6]:
                    name = t.get('_note_name', '?')
                    status = t.get('status', '?')
                    intention = (t.get('intention', '') or '')[:60]
                    lines.append(f"  {name} [{status}] \"{intention}\"")
        except Exception:
            pass

        return "\n".join(lines)

    def _handle_sensor_alert_response(self, alert_text: str, sensor_name: str):
        """Respond to a sensor alert in character, without recording conversation turns."""
        system_prompt = self._update_system_prompt()
        user_prompt = (
            f"Sensor alert from {sensor_name}:\n{alert_text}\n\n"
            f"React briefly in character. This is an internal sensor notification, not a conversation.\n"
            f"End your response with </end>"
        )
        try:
            result = self.llm_generate(
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt},
                ],
                max_tokens=812,
                temperature=0.7,
                stops=['</end>'],
            )
            if result.success and result.text:
                response = result.text.strip()
                self._say_to_user(response)
                logger.info(f'🚨 {self.character_name} sensor alert response: {response[:80]}...')
            else:
                logger.warning(f'Sensor alert LLM call failed: {getattr(result, "error", "unknown")}')
        except Exception as e:
            logger.error(f'Error in sensor alert response: {e}')
            traceback.print_exc()

    def _handle_proactive_remark(self, text: str, source: str, rationale: str):
        """Generate an unsolicited observation based on Orient's assessment.

        Lightweight LLM call. The LLM can respond [SKIP] if on reflection
        the observation isn't worth sharing.
        """
        system_prompt = self._update_system_prompt()
        user_prompt = (
            f"You've noticed something in your ambient sensor data that may be worth "
            f"briefly mentioning to the user.\n\n"
            f"What you noticed: {rationale}\n"
            f"Source data: {str(text)[:500]}\n\n"
            f"If this is genuinely interesting or relevant to the user's known interests "
            f"or your active concerns, share a brief, natural observation (1-2 sentences). "
            f"Do not be sycophantic or over-eager. Be concise and in character.\n"
            f"If on reflection it's not worth mentioning, respond with exactly: [SKIP]\n</end>"
        )
        try:
            result = self.llm_generate(
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt},
                ],
                max_tokens=662,
                temperature=0.7,
                stops=['</end>'],
            )
            if result.success and result.text:
                response = result.text.strip()
                if '[SKIP]' in response:
                    logger.info(f'💭 {self.character_name} proactive remark skipped (LLM declined)')
                    return
                self._say_to_user(response)
                self._last_proactive_remark_at = time.monotonic()
                logger.info(f'💭 {self.character_name} proactive remark: {response[:80]}...')
            else:
                logger.debug(f'Proactive remark LLM call failed: {getattr(result, "error", "unknown")}')
        except Exception as e:
            logger.debug(f'Error in proactive remark: {e}')

    # ── Goal worker thread ────────────────────────────────────────────────

    def _run_goal_on_thread(self, fn, *args, **kwargs):
        """Execute a goal function on the worker thread.

        fn is called with *args/**kwargs.  Result stored in _goal_thread_result.
        _goal_done_event is set on completion (success or failure).
        """
        self._goal_done_event.clear()
        self._goal_thread_result = None
        # Mark as running so UI shows interrupt button
        self.execution_paused = False
        self.execution_mode = 'run'
        self._publish_execution_state()

        # Stash OODA snapshots on executor for planner/reflective-note consumption
        try:
            self.infospace_executor._ooda_living_state = self._ooda_living_state
            dc = getattr(self, '_derived_concern_model', None)
            self.infospace_executor._derived_concerns_snapshot = dc.get_concerns() if dc else []
            self.infospace_executor._user_concerns_snapshot = self.user_concern_model.get_concerns() or []
            self.infospace_executor._orient_assessment = self._last_character_eval
        except Exception:
            pass

        def _worker():
            try:
                result = fn(*args, **kwargs)
                self._goal_thread_result = result if result is not None else {}
            except Exception as e:
                logger.error(f'Goal thread error: {e}')
                traceback.print_exc()
                self._goal_thread_result = {'success': False, 'error': str(e)}
            finally:
                self._goal_done_event.set()

        self._goal_thread = threading.Thread(target=_worker, daemon=True, name='goal-worker')
        self._goal_thread.start()

    def _is_goal_running(self) -> bool:
        """True if a goal is currently executing on the worker thread."""
        return self._goal_thread is not None and self._goal_thread.is_alive()

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

    def _update_tom_from_turns(self, entity: str, raw_transcript: str):
        """Update Theory of Mind model for an entity from conversation turns.

        Loads the existing ToM note (_tom_{entity}), runs the discourse
        ToM update template, and persists the result.  Skips gracefully
        if the transcript is trivial.
        """
        if not raw_transcript or len(raw_transcript) < 40:
            return
        try:
            from discourse import DiscourseTracker
            dt = DiscourseTracker(self.llm_generate, self.character_name, entity)

            # Load previous ToM state from named Note (if any)
            tom_note_name = f"_tom_{entity.lower()}"
            previous_tom = ""
            try:
                load_result = self.infospace_executor.execute_action(
                    {"type": "load", "target": tom_note_name, "out": "$_tom_tmp"})
                if load_result.get('status') == 'success' and load_result.get('resource_id'):
                    previous_tom = self.infospace_executor._get_content(load_result['resource_id']) or ""
            except Exception:
                pass

            # Format dialog as list-of-dicts for DiscourseTracker
            dialog = []
            for line in raw_transcript.split('\n'):
                if ':' in line:
                    speaker, text = line.split(':', 1)
                    dialog.append({'source': speaker.strip(), 'text': text.strip()})

            if len(dialog) < 2:
                return

            # Build discourse state context including active user concerns
            discourse_ctx = ""
            try:
                active_uc = self.user_concern_model.get_concerns(active_only=True) or []
                if active_uc:
                    lines = [f"Active user concerns (what {entity} currently cares about):"]
                    for c in active_uc[:8]:
                        label = c.get("concern_label", "?")
                        weight = c.get("weight", "")
                        desc = c.get("concern_description", "")
                        lines.append(f"  - {label} (weight={weight}): {desc}")
                    discourse_ctx = "\n".join(lines)
            except Exception:
                pass

            # Run ToM update (single LLM call)
            tom_text = dt.update_tom_from_discourse_segment(
                dialog, entity, start=0, end=len(dialog) - 1,
                discourse_state=discourse_ctx, previous_tom_state=previous_tom)

            if tom_text and len(tom_text.strip()) > 20:
                self._write_named_note(tom_note_name, tom_text.strip())
                logger.info(f'🧠 ToM updated for {entity} ({len(tom_text)} chars)')
                # Emit to cognitive graph
                try:
                    nid = self._cognitive_graph.add_node(
                        "tom_update", f"ToM update for {entity}",
                        attrs={"entity": entity, "length": len(tom_text)})
                    # Link to most recent conversation turn node if available
                    if self._last_event_node:
                        self._cognitive_graph.add_edge(self._last_event_node, nid, "triggered_by")
                except Exception:
                    pass
        except Exception as e:
            logger.warning(f'ToM update for {entity} failed: {e}')

    def _archive_dialog(self, entity: str, dialog_id: str, note_ids: list):
        """Archive a single completed dialog: synthesize its turns into conversation_history.

        Called by conversation_store.close_dialog via the archive callback.
        note_ids are the Note IDs belonging to this dialog (still in the conversation collection).
        """
        if not self.infospace_executor:
            logger.warning('Infospace executor not available, skipping dialog archiving')
            return
        if not note_ids:
            return

        # Skip trivial conversations (single outgoing ask with no reply)
        if len(note_ids) == 1:
            turn = self.conversation_store._parse_turn_note(note_ids[0])
            if turn and turn.get("act_type") == "ask" and turn.get("direction") == "out":
                logger.info(f'Skipping archive for ask-only dialog {dialog_id}')
                return

        logger.info(f'📝 Archiving dialog {dialog_id} ({len(note_ids)} turns) for {entity}...')

        # Build raw turn transcript for the concern model (before summarization loses detail)
        raw_turn_lines = []
        for nid in note_ids:
            turn = self.conversation_store._parse_turn_note(nid)
            if turn:
                speaker = turn.get('source', '?')
                text = turn.get('text', '')
                raw_turn_lines.append(f"{speaker}: {text}")
        raw_transcript = "\n".join(raw_turn_lines)

        # Update Theory of Mind for this entity
        self._update_tom_from_turns(entity, raw_transcript)

        try:
            # Build a temporary collection binding for synthesize
            # Bind the note IDs so synthesize can read them
            binding_name = '_archive_dialog_src'
            conv_collection_id = self.resource_manager.named_collections.get("conversation")
            if not conv_collection_id:
                logger.warning('No conversation collection found, skipping dialog archiving')
                return

            # Create a temporary collection with just this dialog's notes
            success, tmp_coll_id, err, _ = self.resource_manager.create_collection(
                self.character_name, list(note_ids), "list", "conversation-store",
                f"dialog {dialog_id}", f"_tmp_dialog_{dialog_id}", {}
            )
            if not success or not tmp_coll_id:
                logger.warning(f'Failed to create temp collection for dialog archive: {err}')
                return

            self.infospace_executor.plan_bindings[0][binding_name] = tmp_coll_id

            # Update user concern model from raw dialog turns (before summarization)
            affected_concern_id = None
            if raw_transcript:
                try:
                    affected_concern_id = self.user_concern_model.update_from_conversation(
                        raw_transcript, dialog_id, entity
                    )
                except Exception as e:
                    logger.warning(f'Error updating concern model from dialog {dialog_id}: {e}')

            # Synthesize the dialog turns
            summarize_action = {
                "type": "synthesize",
                "target": f"${binding_name}",
                "focus": f"concise conversation summary between {self.character_name} and {entity}",
                "out": "$_dialog_summary"
            }
            summarize_result = self.infospace_executor.execute_action(summarize_action)

            if summarize_result.get('status') != 'success':
                logger.warning(f'Failed to synthesize dialog {dialog_id}: {summarize_result.get("reason", "unknown")}')
                return

            summary_note_id = summarize_result.get('resource_id')
            if not summary_note_id:
                summary_note_id = self.infospace_executor._get_binding('_dialog_summary')
            if not summary_note_id:
                logger.warning(f'Synthesize did not return a Note ID for dialog {dialog_id}')
                return

            # Verify content
            summary_content = self.infospace_executor._get_content(summary_note_id)
            if not summary_content or (isinstance(summary_content, str) and not summary_content.strip()):
                logger.warning(f'Summarization returned empty result for dialog {dialog_id}')
                return

            # Persist and add to conversation_history
            persist_action = {"type": "persist", "target": "$_dialog_summary"}
            self.infospace_executor.execute_action(persist_action)

            add_action = {
                "type": "add", "target": "conversation_history",
                "value": "$_dialog_summary", "out": "$conversation_history"
            }
            add_result = self.infospace_executor.execute_action(add_action)

            if add_result.get('status') == 'success':
                logger.info(f'✓ Archived dialog {dialog_id} ({len(note_ids)} turns) to conversation_history')

                # Tag the summary note with concern_id and register with concern model
                if affected_concern_id and summary_note_id:
                    self.conversation_store.tag_note_concern(summary_note_id, affected_concern_id)
                    self.user_concern_model.add_history_note_id(affected_concern_id, summary_note_id)
                    logger.info(f'  Tagged {summary_note_id} with concern {affected_concern_id}')

                    # Check if this concern needs second-level rollup
                    self._maybe_rollup_concern_history(affected_concern_id)

                # Tag with goal_id if a goal was active during this dialog
                active_goal_id = getattr(self, '_active_scheduled_goal_id', None)
                if active_goal_id and summary_note_id:
                    self.conversation_store.tag_note_goal(summary_note_id, active_goal_id)
                    logger.info(f'  Tagged {summary_note_id} with goal {active_goal_id}')
            else:
                logger.warning(f'Failed to add dialog summary to conversation_history: {add_result.get("reason", "unknown")}')

            # Delete the original turn notes (they've been synthesized)
            for nid in note_ids:
                ok, del_err = self.resource_manager.delete_resource(nid)
                if not ok:
                    logger.warning(f'Failed to delete archived turn note {nid}: {del_err}')

            # Clean up temp collection and bindings
            if tmp_coll_id and self.resource_manager:
                self.resource_manager.resource_registry.pop(tmp_coll_id, None)
                self.resource_manager.named_collections.pop(f"_tmp_dialog_{dialog_id}", None)
            for scope in self.infospace_executor.plan_bindings:
                scope.pop(binding_name, None)
                scope.pop('_dialog_summary', None)

        except Exception as e:
            logger.error(f'Error archiving dialog {dialog_id}: {e}')
            import traceback
            traceback.print_exc()

    def _maybe_rollup_concern_history(self, concern_id: str):
        """If a concern has accumulated enough history entries, roll them up into history_summary."""
        ROLLUP_THRESHOLD = 3  # Number of unconsolidated entries before triggering rollup

        concern = self.user_concern_model.get_concern(concern_id)
        if not concern:
            return
        history_note_ids = concern.get('history_note_ids', [])

        # Prune stale IDs (notes deleted elsewhere) before checking threshold
        live_ids = []
        stale_ids = []
        for nid in history_note_ids:
            if self.resource_manager and self.resource_manager.get_resource(nid):
                live_ids.append(nid)
            else:
                stale_ids.append(nid)
        if stale_ids:
            self.user_concern_model.clear_history_note_ids(concern_id, stale_ids)
            logger.info(f'Pruned {len(stale_ids)} stale history_note_ids from {concern_id}')

        if len(live_ids) < ROLLUP_THRESHOLD:
            return
        history_note_ids = live_ids

        logger.info(f'📝 Rolling up {len(history_note_ids)} history entries for concern {concern_id}')

        # Gather the content of all unconsolidated entries
        entry_texts = []
        for nid in history_note_ids:
            resource = self.resource_manager.get_resource(nid)
            if resource:
                content = resource.get('properties', {}).get('content', '')
                if content:
                    entry_texts.append(str(content))

        if not entry_texts:
            return

        # Build input for synthesis: existing summary + new entries
        existing_summary = concern.get('history_summary', '')
        label = concern.get('concern_label', 'unknown concern')

        synth_input = ''
        if existing_summary:
            synth_input += f"Previous summary:\n{existing_summary}\n\n"
        synth_input += "Recent conversations:\n" + "\n---\n".join(entry_texts)

        tmp_note_id = None
        tmp_coll_id = None
        result_id = None
        try:
            # Create a temporary note with the combined content for synthesis
            success, tmp_note_id, err, _ = self.resource_manager.create_note(
                self.character_name, synth_input, "text", "rollup",
                f"rollup input for {concern_id}", "", {}
            )
            if not success or not tmp_note_id:
                logger.warning(f'Failed to create rollup input note: {err}')
                return

            # Create temp collection for synthesize
            success, tmp_coll_id, err, _ = self.resource_manager.create_collection(
                self.character_name, [tmp_note_id], "list", "rollup",
                f"rollup for {concern_id}", f"_tmp_rollup_{concern_id}", {}
            )
            if not success or not tmp_coll_id:
                logger.warning(f'Failed to create rollup collection: {err}')
                return

            binding_name = '_rollup_src'
            self.infospace_executor.plan_bindings[0][binding_name] = tmp_coll_id

            synth_action = {
                "type": "synthesize",
                "target": f"${binding_name}",
                "focus": f"consolidated history of user concern: {label}",
                "out": "$_rollup_result"
            }
            synth_result = self.infospace_executor.execute_action(synth_action)

            if synth_result.get('status') != 'success':
                logger.warning(f'Failed to synthesize rollup for {concern_id}: {synth_result.get("reason")}')
                return

            result_id = synth_result.get('resource_id') or self.infospace_executor._get_binding('_rollup_result')
            if not result_id:
                logger.warning(f'Rollup synthesis returned no result for {concern_id}')
                return

            new_summary = self.infospace_executor._get_content(result_id)
            if not new_summary or not str(new_summary).strip():
                logger.warning(f'Rollup synthesis returned empty result for {concern_id}')
                return

            # Update the concern's history_summary
            self.user_concern_model.update_history_summary(concern_id, str(new_summary))

            # Remove the rolled-up entries from conversation_history and delete them
            self.conversation_store._remove_notes_from_collection(
                history_note_ids, collection_name="conversation_history", delete_notes=True
            )

            # Clear the note IDs from the concern
            self.user_concern_model.clear_history_note_ids(concern_id, history_note_ids)

            logger.info(f'✓ Rolled up {len(history_note_ids)} entries into history_summary for {concern_id}')

        except Exception as e:
            logger.error(f'Error during concern history rollup for {concern_id}: {e}')
            import traceback
            traceback.print_exc()
        finally:
            # Clean up temp resources and bindings
            if self.resource_manager:
                if tmp_coll_id:
                    self.resource_manager.resource_registry.pop(tmp_coll_id, None)
                self.resource_manager.named_collections.pop(f"_tmp_rollup_{concern_id}", None)
                if tmp_note_id:
                    self.resource_manager.delete_resource(tmp_note_id)
                if result_id:
                    self.resource_manager.delete_resource(result_id)
            for scope in self.infospace_executor.plan_bindings:
                scope.pop('_rollup_src', None)
                scope.pop('_rollup_result', None)

    def _archive_conversation(self):
        """Shutdown fallback: archive any remaining unarchived turns in the conversation collection."""
        logger.info(f'📝 Shutdown archive fallback for {self.character_name}...')

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

            # Check if collection is empty
            size_action = {"type": "size", "target": "$conv", "out": "$conv_size"}
            size_result = self.infospace_executor.execute_action(size_action)

            if size_result.get('status') != 'success':
                logger.warning(f'Failed to get conversation size: {size_result.get("reason", "unknown")}')
                return

            conv_size = size_result.get('value', 0)
            try:
                conv_size = int(conv_size) if isinstance(conv_size, str) else conv_size
            except (ValueError, TypeError):
                logger.warning(f'Invalid conversation size value: {conv_size}')
                return

            if conv_size == 0:
                logger.info('Conversation collection is empty (all dialogs already archived), skipping')
                return

            if conv_size == 1 and self.conversation_store.is_ask_only_conversation():
                logger.info('Conversation has only a single ask turn, skipping archiving')
                return

            logger.info(f'📝 Shutdown fallback: archiving {conv_size} remaining turns...')

            # Build raw transcript for ToM update before summarization
            try:
                conv_coll_id = self.resource_manager.named_collections.get("conversation")
                if conv_coll_id:
                    coll_data = self.resource_manager.get_resource(conv_coll_id)
                    turn_ids = (coll_data or {}).get('properties', {}).get('content', [])
                    raw_lines = []
                    for nid in turn_ids:
                        turn = self.conversation_store._parse_turn_note(nid)
                        if turn:
                            raw_lines.append(f"{turn.get('source', '?')}: {turn.get('text', '')}")
                    if raw_lines:
                        self._update_tom_from_turns("User", "\n".join(raw_lines))
            except Exception as e:
                logger.debug(f"Shutdown ToM update failed: {e}")

            # Synthesize whatever remains
            summarize_action = {"type": "synthesize", "target": "$conv", "focus": "concise conversation summary", "out": "$summary"}
            summarize_result = self.infospace_executor.execute_action(summarize_action)

            if summarize_result.get('status') != 'success':
                logger.warning(f'Failed to synthesize conversation: {summarize_result.get("reason", "unknown")}')
                return

            summary_note_id = summarize_result.get('resource_id')
            if not summary_note_id:
                summary_note_id = self.infospace_executor.plan_bindings_flat.get('summary')
            if not summary_note_id:
                logger.warning('Summarize tool did not return a Note ID')
                return

            summary_content = self.infospace_executor._get_content(summary_note_id)
            if not summary_content or (isinstance(summary_content, str) and not summary_content.strip()):
                logger.warning('Summarization returned empty result, skipping archiving')
                return

            persist_action = {"type": "persist", "target": "$summary"}
            self.infospace_executor.execute_action(persist_action)

            add_action = {"type": "add", "target": "conversation_history", "value": "$summary", "out": "$conversation_history"}
            add_result = self.infospace_executor.execute_action(add_action)

            if add_result.get('status') == 'success':
                logger.info(f'✓ Shutdown fallback: archived {conv_size} remaining turns to conversation_history')
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

    def _DEAD_process_text_input_original(self):
        """DEAD CODE — replaced by OODA pipeline. Retained for reference during verification."""
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
            etype, igc, ipl = self._character_eval_flags_from_process_input(source, clean_input)
            self._character_eval_run(etype, clean_input, source, is_goal_command=igc, is_proceed_like=ipl)
            
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
            
            # Handle scheduler-issued proceed/reuse commands
            if source == 'scheduler':
                cmd = clean_input.strip().lower()
                parts = clean_input.strip().split()
                target_id = parts[1] if len(parts) > 1 else None
                if cmd.startswith('proceed'):
                    if target_id and target_id.startswith('goal_'):
                        self._handle_goal_proceed(goal_id=target_id, source='scheduler')
                elif cmd.startswith('reuse'):
                    if target_id and target_id.startswith('goal_'):
                        self._handle_goal_reuse(goal_id=target_id)
                return

            # Handle special commands from User BEFORE processing as dialog
            if source == 'User':
                # Task establishment prefix
                if clean_input.lower().startswith('task:'):
                    task_text = clean_input[5:].strip()
                    if not task_text:
                        self._say_to_user("Please provide a task description after 'task:'.")
                        return
                    if self.active_task_wip:
                        self._say_to_user("A task is already being established. Please wait for it to complete.")
                        return
                    if self._is_goal_running():
                        self._say_to_user("A goal is currently running. Please wait for it to complete.")
                        return
                    self.conversation_store.close_dialog("User")
                    self._begin_task_establishment(task_text)
                    return
                elif _is_goal_cmd(clean_input):
                    goal_preview = clean_input[:80] + ('...' if len(clean_input) > 80 else '')
                    logger.info(f'📥 {self.character_name} Received goal: "{goal_preview}"')
                    self.conversation_store.close_dialog("User")
                    goal_text = clean_input[5:].strip()
                    scheduled_goal = self._upsert_scheduled_goal(goal_text)
                    goal_id = scheduled_goal["goal_id"]
                    if self._is_goal_running():
                        self._say_to_user("A goal is already running. Please wait for it to complete.")
                        return
                    self._active_scheduled_goal_id = goal_id
                    self._update_scheduled_goal(goal_id, is_running=True, status="running")
                    if hasattr(self, 'goal_scheduler'):
                        self.goal_scheduler._executing_goal_id = goal_id
                        self.goal_scheduler._executing_is_autonomous = False
                    if self._last_character_eval:
                        self._update_scheduled_goal(goal_id, initial_assessment=self._last_character_eval)
                    pre_resource_ids = set(self.resource_manager.resource_registry.keys()) if self.resource_manager else set()

                    def _run_user_goal():
                        result = self.parse_and_set_goal("", goal_text) or {}
                        self._set_scheduled_goal_result(goal_id, result, used_cache=False, pre_resource_ids=pre_resource_ids)
                        return result

                    self._run_goal_on_thread(_run_user_goal)
                    return  # Don't process as speech
                elif clean_input.strip().lower().startswith('proceed') or clean_input.strip().lower() == 'next step':
                    parts = clean_input.strip().split()
                    target_id = parts[1] if len(parts) > 1 else None
                    if target_id and target_id.startswith('goal_'):
                        self._handle_goal_proceed(goal_id=target_id)
                    return
                elif clean_input.strip().lower().startswith('terminate'):
                    parts = clean_input.strip().split()
                    target_id = parts[1] if len(parts) > 1 else None
                    if target_id and target_id.startswith('goal_'):
                        self._handle_goal_terminate(goal_id=target_id)
                    return
                elif clean_input.strip().lower().startswith('reuse'):
                    parts = clean_input.strip().split()
                    target_id = parts[1] if len(parts) > 1 else None
                    if target_id and target_id.startswith('goal_'):
                        self._handle_goal_reuse(goal_id=target_id)
                    return
                elif clean_input.strip().lower().startswith('clear-cache'):
                    parts = clean_input.strip().split()
                    goal_id = parts[1] if len(parts) > 1 and parts[1].startswith('goal_') else None
                    if goal_id:
                        self._handle_goal_cache_clear(goal_id=goal_id)
                    else:
                        self._say_to_user("Please specify which goal cache to clear, e.g. 'clear-cache goal_1'.")
                    return
                else:
                    # Regular user input
                    self._create_character_note()
                    # End-of-conversation keywords
                    end_phrases = ('goodbye', 'bye', 'end conversation')
                    if clean_input.strip().lower().rstrip('.,!') in end_phrases:
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
                # Record incoming message for conversation history
                self.conversation_store.record_incoming(source, clean_input, close=close_flag)
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
        entity_data = self.conversation_store.get_entity_context(source, limit=20, scope='current')
        if entity_data and 'conversation_history' in entity_data:
            for entry in entity_data['conversation_history'][-20:]:
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

    # ── Command registry ─────────────────────────────────────────────────

    def _build_command_registry(self) -> dict:
        """Build the command registry — the complete manifest of imperative operations.

        Returns a dict mapping command name to {handler, description, args}.
        Handler signature: handler(data: dict) -> Optional[str] (response text).
        """
        return {
            # ── Goals ──
            '/goal add': {
                'handler': self._cmd_goal_add,
                'description': 'Create a new goal',
                'args': ['goal_text'],
            },
            '/goal run': {
                'handler': self._cmd_goal_run,
                'description': 'Execute a goal',
                'args': ['goal_id'],
            },
            '/goal terminate': {
                'handler': self._cmd_goal_terminate,
                'description': 'Stop a running goal',
                'args': ['goal_id'],
            },
            '/goal rename': {
                'handler': self._cmd_goal_rename,
                'description': 'Rename a goal',
                'args': ['goal_id', 'name'],
            },
            '/goal edit': {
                'handler': self._cmd_goal_edit,
                'description': 'Update goal text',
                'args': ['goal_id', 'goal_text'],
            },
            '/goal delete': {
                'handler': self._cmd_goal_remove,
                'description': 'Delete a goal',
                'args': ['goal_id'],
            },
            '/goal remove': {
                'handler': self._cmd_goal_remove,
                'description': 'Delete a goal (alias for /goal delete)',
                'args': ['goal_id'],
            },
            '/goal mode': {
                'handler': self._cmd_goal_mode,
                'description': 'Set schedule mode (manual|auto|recurring|daily)',
                'args': ['goal_id', 'schedule_mode'],
            },
            '/goal exec': {
                'handler': self._cmd_goal_exec,
                'description': 'Set execution mode (replan|replay)',
                'args': ['goal_id', 'execution_mode'],
            },
            '/goal cache clear': {
                'handler': self._cmd_goal_cache_clear,
                'description': 'Clear cached plan for a goal',
                'args': ['goal_id'],
            },
            '/goal plan': {
                'handler': self._cmd_goal_plan_show,
                'description': 'Show cached plan steps for a goal',
                'args': ['goal_id'],
            },
            '/goal plan show': {
                'handler': self._cmd_goal_plan_show,
                'description': 'Show cached plan steps for a goal',
                'args': ['goal_id'],
            },
            '/goal plan edit': {
                'handler': self._cmd_goal_plan_edit,
                'description': 'Write cached plan to file for editing',
                'args': ['goal_id'],
            },
            '/goal plan load': {
                'handler': self._cmd_goal_plan_load,
                'description': 'Load edited plan from file',
                'args': ['goal_id'],
            },
            '/goal plan approve': {
                'handler': self._cmd_goal_plan_approve,
                'description': 'Approve cached plan (set execution_mode to replay)',
                'args': ['goal_id'],
            },
            '/goal plan review': {
                'handler': self._cmd_goal_plan_review,
                'description': 'Generate review bundle for plan analysis',
                'args': ['goal_id'],
            },
            '/goal plan commit': {
                'handler': self._cmd_goal_plan_commit,
                'description': 'Load plan, set replay, inject learnings',
                'args': ['goal_id'],
            },
            # ── Tasks ──
            '/task add': {
                'handler': self._cmd_task_add,
                'description': 'Create a new task',
                'args': ['intention'],
            },
            '/task approve': {
                'handler': self._cmd_task_approve,
                'description': 'Approve a proposed task',
                'args': ['note_name'],
            },
            '/task edit': {
                'handler': self._cmd_task_edit,
                'description': 'Edit task intention',
                'args': ['note_name', 'intention'],
            },
            '/task abandon': {
                'handler': self._cmd_task_abandon,
                'description': 'Abandon a task',
                'args': ['note_name'],
            },
            '/task delete': {
                'handler': self._cmd_task_delete,
                'description': 'Delete a task and its artifacts',
                'args': ['note_name'],
            },
            '/task interrupt': {
                'handler': self._cmd_task_interrupt,
                'description': 'Pause an active task',
                'args': ['note_name'],
            },
            '/task run': {
                'handler': self._cmd_task_run,
                'description': 'Run task now (clear cooldown)',
                'args': ['note_name'],
            },
            '/task cooldown': {
                'handler': self._cmd_task_cooldown,
                'description': 'Set task cooldown seconds',
                'args': ['note_name', 'cooldown_seconds'],
            },
            # ── Concerns ──
            '/concern close': {
                'handler': self._cmd_concern_manage,
                'description': 'Close a user concern',
                'args': ['concern_id', 'type'],
            },
            '/concern reopen': {
                'handler': self._cmd_concern_manage,
                'description': 'Reopen a user concern',
                'args': ['concern_id', 'type'],
            },
            '/concern resolve': {
                'handler': self._cmd_concern_manage,
                'description': 'Resolve/satisfy a derived concern',
                'args': ['concern_id', 'type'],
            },
            '/concern delete': {
                'handler': self._cmd_concern_manage,
                'description': 'Delete a concern',
                'args': ['concern_id', 'type'],
            },
            '/concern weight': {
                'handler': self._cmd_concern_manage,
                'description': 'Set concern weight',
                'args': ['concern_id', 'type', 'weight'],
            },
            '/concern activate': {
                'handler': self._cmd_concern_manage,
                'description': 'Reactivate a derived concern',
                'args': ['concern_id', 'type'],
            },
            '/concern revisit': {
                'handler': self._cmd_concern_manage,
                'description': 'Set revisit hours for a derived concern',
                'args': ['concern_id', 'type', 'revisit_hours'],
            },
            # ── System ──
            '/stop': {
                'handler': self._cmd_stop,
                'description': 'Halt current execution immediately',
                'args': [],
            },
            '/continuous': {
                'handler': self._cmd_continuous,
                'description': 'Toggle continuous execution',
                'args': [],
            },
            '/llm': {
                'handler': self._cmd_llm_toggle,
                'description': 'Toggle LLM mode (primary/alt)',
                'args': [],
            },
            '/delay': {
                'handler': self._cmd_delay,
                'description': 'Set turn delay seconds',
                'args': ['delay'],
            },
            '/scheduler': {
                'handler': self._cmd_scheduler,
                'description': 'Goal scheduler config (on|off|interval N)',
                'args': [],
            },
            '/clear': {
                'handler': self._cmd_clear,
                'description': 'Clear (world-model|map|transients|persistents)',
                'args': ['target'],
            },
            '/save': {
                'handler': self._cmd_save,
                'description': 'Save all data',
                'args': [],
            },
            '/shutdown': {
                'handler': self._cmd_shutdown,
                'description': 'Save and shutdown',
                'args': [],
            },
            '/bye': {
                'handler': self._cmd_bye,
                'description': 'End conversation',
                'args': [],
            },
            '/done': {
                'handler': self._cmd_bye,
                'description': 'Done with current topic (archives conversation)',
                'args': [],
            },
            '/next': {
                'handler': self._cmd_bye,
                'description': 'Move on to next topic (archives conversation)',
                'args': [],
            },
            '/action': {
                'handler': self._cmd_direct_action,
                'description': 'Execute a direct JSON action',
                'args': ['json_text'],
            },
        }

    def _dispatch_command(self, data: dict) -> Optional[str]:
        """Dispatch a structured command dict to the appropriate handler.

        Args:
            data: Must contain 'cmd' key (e.g. '/goal run'). Other keys are
                  passed to the handler as kwargs.

        Returns:
            Response text from the handler, or error message.
        """
        cmd = (data.get('cmd') or '').strip().lower()
        if not cmd:
            return 'No command specified.'

        # Look up handler — try exact match first, then prefix match for
        # compound commands like '/goal cache clear'
        entry = self._command_registry.get(cmd)
        if not entry:
            # Try longest prefix match for compound commands
            candidates = [(k, v) for k, v in self._command_registry.items() if cmd.startswith(k)]
            if candidates:
                entry = max(candidates, key=lambda x: len(x[0]))[1]

        if not entry:
            return f"Unknown command: {cmd}"

        source = data.get('source', 'User')
        try:
            return entry['handler'](data)
        except Exception as e:
            logger.error(f'Command dispatch error ({cmd}): {e}', exc_info=True)
            return f'Command failed: {e}'

    def _handle_command_message(self, sample):
        """Zenoh callback for the unified command channel."""
        try:
            data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            result = self._dispatch_command(data)
            if result:
                logger.info(f"Command result: {result[:120]}")
        except Exception as e:
            logger.error(f'Command message handler error: {e}')

    def get_command_list(self) -> list:
        """Return the command registry for help/discovery."""
        return [
            {'cmd': k, 'description': v['description'], 'args': v['args']}
            for k, v in self._command_registry.items()
        ]

    # ── Command handlers ─────────────────────────────────────────────────
    # Each handler accepts a dict and returns an optional response string.

    # -- Goals --

    def _cmd_goal_add(self, data: dict) -> str:
        goal_text = (data.get('goal_text') or '').strip()
        if not goal_text:
            return 'Usage: /goal add <goal text>'
        # Don't close dialog — the goal is part of the ongoing conversation.
        # Dialog is closed by /bye, shutdown, or UI end-conversation button.
        scheduled_goal = self._upsert_scheduled_goal(goal_text)
        goal_id = scheduled_goal["goal_id"]
        # User-created goals persist so they can be reused in goal chains.
        scheduled_goal['ephemeral'] = False
        # Store callback metadata if present (for delegated concern tasks)
        if data.get('callback_topic'):
            scheduled_goal['callback_topic'] = data['callback_topic']
        if data.get('callback_concern_id'):
            scheduled_goal['callback_concern_id'] = data['callback_concern_id']
        if data.get('source_agent'):
            scheduled_goal['source_agent'] = data['source_agent']
        self._save_scheduled_goal(scheduled_goal)
        if self._is_goal_running():
            self._say_to_user(f"Goal '{goal_id}' created but another goal is already running.")
            return f"Goal {goal_id} created (queued)"
        self._active_scheduled_goal_id = goal_id
        self._update_scheduled_goal(goal_id, is_running=True, status="running")
        if hasattr(self, 'goal_scheduler'):
            self.goal_scheduler._executing_goal_id = goal_id
            self.goal_scheduler._executing_is_autonomous = False
        pre = set(self.resource_manager.resource_registry.keys()) if self.resource_manager else set()
        def _run():
            # Clear last_say_text so we can detect whether *this* goal said anything
            self.last_say_text = ''
            result = self.parse_and_set_goal("", goal_text) or {}
            self._set_scheduled_goal_result(goal_id, result, used_cache=False, pre_resource_ids=pre)
            return result
        self._run_goal_on_thread(_run)
        return f"Goal {goal_id} created and started"

    def _cmd_goal_run(self, data: dict) -> str:
        goal_id = (data.get('goal_id') or '').strip()
        if not goal_id:
            return 'Usage: /goal run <goal_id> [replan|replay]'
        source = data.get('source', 'user')
        goal = self._get_scheduled_goal(goal_id)
        if not goal:
            return f"Goal '{goal_id}' not found."

        # Honor explicit execution_mode override from the command
        mode_override = data.get('execution_mode')
        if mode_override:
            self._update_scheduled_goal(goal_id, execution_mode=mode_override)
            goal['execution_mode'] = mode_override

        if goal.get("execution_mode") == "replay" and goal.get("cached_plan_actions"):
            self._handle_goal_reuse(goal_id=goal_id)
            return f"Goal {goal_id} replay requested"
        self._handle_goal_proceed(goal_id=goal_id, source=source)
        return f"Goal {goal_id} proceed requested"

    def _cmd_goal_terminate(self, data: dict) -> str:
        goal_id = (data.get('goal_id') or '').strip()
        if not goal_id:
            return 'Usage: /goal terminate <goal_id>'
        self._handle_goal_terminate(goal_id=goal_id)
        return f"Goal {goal_id} terminate requested"

    def _cmd_goal_rename(self, data: dict) -> str:
        goal_id = (data.get('goal_id') or '').strip()
        name = (data.get('name') or '').strip()
        if not goal_id or not name:
            return 'Usage: /goal rename <goal_id> <name>'
        self._update_scheduled_goal(goal_id, name=name[:120], name_customized=True)
        logger.info(f"Goal {goal_id} renamed to '{name[:120]}'")
        return f"Goal {goal_id} renamed"

    def _cmd_goal_edit(self, data: dict) -> str:
        goal_id = (data.get('goal_id') or '').strip()
        goal_text = (data.get('goal_text') or '').strip()
        if not goal_id or not goal_text:
            return 'Usage: /goal edit <goal_id> <goal text>'
        goal = self._get_scheduled_goal(goal_id)
        old_text = (goal.get("goal_text", "") if goal else "").strip()
        updates = {"goal_text": goal_text}
        if goal_text != old_text:
            updates["cached_plan_actions"] = []
            updates["execution_mode"] = "replan"
            updates["status"] = "ready"
        self._update_scheduled_goal(goal_id, **updates)
        logger.info(f"Goal {goal_id} text updated ({len(goal_text)} chars)")
        return f"Goal {goal_id} updated"

    def _cmd_goal_remove(self, data: dict) -> str:
        goal_id = (data.get('goal_id') or '').strip()
        if not goal_id:
            return 'Usage: /goal delete <goal_id>'
        goal = self._get_scheduled_goal(goal_id)
        if not goal:
            return f"Goal '{goal_id}' not found"
        # Interrupt if running
        if goal.get("is_running") or self._active_scheduled_goal_id == goal_id:
            self.interrupt_requested = True
            if self.infospace_executor:
                self.infospace_executor.interrupt_requested = True
        # If this goal is a task milestone, abort the task WIP
        if goal.get("task_wip_id") and self.active_task_wip:
            wip = self._read_task_wip()
            if wip:
                wip["status"] = "interrupted"
                wip["current_milestone"] = None
                wip["updated"] = datetime.now().isoformat()
                wip.setdefault("accumulated_findings", []).append(
                    "Task interrupted: milestone goal removed by user"
                )
                self._update_task_wip(wip)
            self.active_task_wip = None
            self.active_task_wip_waiting = False
            self._task_wip_pre_resource_ids = None
            self._say_to_user("Task establishment interrupted.")
        deleted = self._delete_scheduled_goal(goal_id)
        if not deleted:
            return f"Goal '{goal_id}' could not be removed"
        if self._active_scheduled_goal_id == goal_id:
            self._active_scheduled_goal_id = None
        if goal_id in self._scheduler_started_goals:
            self._scheduler_started_goals.discard(goal_id)
        if hasattr(self, "goal_scheduler"):
            self.goal_scheduler.notify_goal_terminal(goal_id)
        self._publish_execution_state()
        logger.info(f"Goal {goal_id} removed via command")
        return f"Goal '{goal.get('name') or goal_id}' removed"

    def _cmd_goal_mode(self, data: dict) -> str:
        goal_id = (data.get('goal_id') or '').strip()
        mode = (data.get('schedule_mode') or '').strip()
        if not goal_id or mode not in ('manual', 'auto', 'recurring', 'daily'):
            return 'Usage: /goal mode <goal_id> <manual|auto|recurring|daily>'
        updates = {"schedule_mode": mode}
        if mode == "daily":
            updates["run_at"] = data.get("run_at", "")
        self._update_scheduled_goal(goal_id, **updates)
        logger.info(f"Goal {goal_id} schedule_mode set to '{mode}'")
        return f"Goal {goal_id} mode set to {mode}"

    def _cmd_goal_exec(self, data: dict) -> str:
        goal_id = (data.get('goal_id') or '').strip()
        mode = (data.get('execution_mode') or '').strip()
        if not goal_id or mode not in ('replan', 'replay'):
            return 'Usage: /goal exec <goal_id> <replan|replay>'
        self._update_scheduled_goal(goal_id, execution_mode=mode)
        logger.info(f"Goal {goal_id} execution_mode set to '{mode}'")
        return f"Goal {goal_id} exec mode set to {mode}"

    def _cmd_goal_cache_clear(self, data: dict) -> str:
        goal_id = (data.get('goal_id') or '').strip()
        if not goal_id:
            return 'Usage: /goal cache clear <goal_id>'
        self._handle_goal_cache_clear(goal_id=goal_id)
        return f"Goal {goal_id} cache cleared"

    # -- Tasks --

    def _cmd_task_add(self, data: dict) -> str:
        intention = (data.get('intention') or '').strip()
        if not intention:
            return 'Usage: /task add <intention>'
        if self._is_goal_running():
            return 'A goal is running. Wait for it to complete before starting a task.'
        self._begin_task_establishment(intention)
        return f"Task created: {intention[:80]}"

    def _cmd_task_approve(self, data: dict) -> str:
        note_name = (data.get('note_name') or '').strip()
        if not note_name:
            return 'Usage: /task approve <note_name> [intention]'
        if not note_name.startswith('_task_wip_'):
            note_name = f'_task_wip_{note_name}'
        intention = data.get('intention', '')
        self._approve_proposed_task(note_name, intention)
        return f"Task {note_name} approval requested"

    def _cmd_task_edit(self, data: dict) -> str:
        note_name = (data.get('note_name') or '').strip()
        intention = (data.get('intention') or '').strip()
        if not note_name or not intention:
            return 'Usage: /task edit <note_name> <intention>'
        if not note_name.startswith('_task_wip_'):
            note_name = f'_task_wip_{note_name}'
        # Delegate to existing handler logic (without Zenoh sample)
        note_id = self.resource_manager.named_notes.get(note_name) if self.resource_manager else None
        if not note_id:
            return f"Task {note_name} not found"
        res = self.resource_manager.get_resource(note_id)
        content = json.loads(res.get('properties', {}).get('content', '{}'))
        content['intention'] = intention
        content['updated'] = datetime.now().isoformat()
        success, err = self.resource_manager.update_note_content(note_id, json.dumps(content))
        if not success:
            return f"Failed to update {note_name}: {err}"
        logger.info(f'📋 Task {note_name} intention edited via command')
        return f"Task {note_name} edited"

    def _cmd_task_abandon(self, data: dict) -> str:
        note_name = (data.get('note_name') or '').strip()
        if not note_name:
            return 'Usage: /task abandon <note_name> [reason]'
        if not note_name.startswith('_task_wip_'):
            note_name = f'_task_wip_{note_name}'
        reason = data.get('reason', 'abandoned via command')
        self._abandon_task(note_name, reason)
        return f"Task {note_name} abandoned"

    def _cmd_task_delete(self, data: dict) -> str:
        note_name = (data.get('note_name') or '').strip()
        if not note_name:
            return 'Usage: /task delete <note_name>'
        if not note_name.startswith('_task_wip_'):
            note_name = f'_task_wip_{note_name}'
        # Build a synthetic dict and reuse existing logic
        self._cmd_task_delete_impl(note_name)
        return f"Task {note_name} deleted"

    def _cmd_task_delete_impl(self, note_name: str):
        """Delete a task WIP and all its associated artifacts."""
        if not self.resource_manager:
            return
        note_id = self.resource_manager.named_notes.get(note_name)
        if not note_id:
            logger.warning(f"Task WIP note {note_name} not found")
            return
        note_data = self.resource_manager.resource_registry.get(note_id)
        wip = {}
        if note_data:
            content = note_data.get("properties", {}).get("content", "")
            try:
                wip = json.loads(content) if isinstance(content, str) else content
            except (json.JSONDecodeError, TypeError):
                pass

        task_wip_id = wip.get("task_wip_id", "")
        deleted_goals = []

        # Delete associated scheduled goals (milestone + final recurring)
        for goal in self._all_scheduled_goals():
            if (goal.get("task_wip_id") == task_wip_id and task_wip_id) or \
               goal.get("task_context_note") == note_name:
                self._delete_scheduled_goal(goal["goal_id"])
                deleted_goals.append(goal["goal_id"])

        # Delete the WIP note itself
        if self.infospace_executor:
            self.infospace_executor.delete_resource_and_unbind(note_id)

        # Clear active task state if this is the active task
        if self.active_task_wip == note_name:
            self.interrupt_requested = True
            if self.infospace_executor:
                self.infospace_executor.interrupt_requested = True
            self.active_task_wip = None
            self.active_task_wip_waiting = False
            self._task_wip_pre_resource_ids = None
        # Clear operational goal state if this task's goal is running
        if self._operational_task_note == note_name:
            self._operational_task_note = None
            self._operational_goal_waiting = False

        self._publish_execution_state()
        logger.info(f'📋 Task WIP {note_name} deleted via command (removed {len(deleted_goals)} goals)')

    def _cmd_task_interrupt(self, data: dict) -> str:
        note_name = (data.get('note_name') or '').strip()
        if not note_name:
            return 'Usage: /task interrupt <note_name>'
        if not note_name.startswith('_task_wip_'):
            note_name = f'_task_wip_{note_name}'
        if self.active_task_wip != note_name:
            return f"Task {note_name} is not the active task"
        self.interrupt_requested = True
        if self.infospace_executor:
            self.infospace_executor.interrupt_requested = True
        wip = self._read_task_wip()
        if wip:
            wip["status"] = "interrupted"
            wip["current_milestone"] = None
            wip["updated"] = datetime.now().isoformat()
            self._update_task_wip(wip)
        logger.info(f'📋 Task {note_name} interrupted via command')
        return f"Task {note_name} interrupted"

    def _cmd_task_run(self, data: dict) -> str:
        note_name = (data.get('note_name') or '').strip()
        if not note_name:
            return 'Usage: /task run <note_name>'
        if not note_name.startswith('_task_wip_'):
            note_name = f'_task_wip_{note_name}'
        if not self.resource_manager:
            return 'No resource manager'
        note_id = self.resource_manager.named_notes.get(note_name)
        if not note_id:
            return f"Task {note_name} not found"
        res = self.resource_manager.get_resource(note_id)
        content = json.loads(res.get('properties', {}).get('content', '{}'))
        if content.get('status') != 'active' or content.get('lifecycle') != 'operational':
            return f"Task {note_name} is not an active operational task"
        content['last_executed'] = None
        content['cooldown_until'] = None
        content['updated'] = datetime.now().isoformat()
        success, err = self.resource_manager.update_note_content(note_id, json.dumps(content))
        if not success:
            return f"Failed to update {note_name}: {err}"
        logger.info(f'📋 Task {note_name} run-now via command')
        return f"Task {note_name} marked for immediate execution"

    def _cmd_task_cooldown(self, data: dict) -> str:
        note_name = (data.get('note_name') or '').strip()
        cooldown = data.get('cooldown_seconds')
        if not note_name or cooldown is None:
            return 'Usage: /task cooldown <note_name> <seconds>'
        if not note_name.startswith('_task_wip_'):
            note_name = f'_task_wip_{note_name}'
        cooldown = int(cooldown)
        if not self.resource_manager:
            return 'No resource manager'
        note_id = self.resource_manager.named_notes.get(note_name)
        if not note_id:
            return f"Task {note_name} not found"
        res = self.resource_manager.get_resource(note_id)
        content = json.loads(res.get('properties', {}).get('content', '{}'))
        content['cooldown_seconds'] = cooldown
        content['updated'] = datetime.now().isoformat()
        success, err = self.resource_manager.update_note_content(note_id, json.dumps(content))
        if not success:
            return f"Failed to update {note_name}: {err}"
        logger.info(f'📋 Task {note_name} cooldown set to {cooldown}s via command')
        return f"Task {note_name} cooldown set to {cooldown}s"

    # -- Concerns --

    def _cmd_concern_manage(self, data: dict) -> str:
        concern_id = (data.get('concern_id') or '').strip()
        if not concern_id:
            return 'Usage: /concern <action> <concern_id> [args]'
        # Extract action from the cmd name: '/concern close' -> 'close'
        cmd = data.get('cmd', '')
        action = cmd.split()[-1] if cmd else ''
        concern_type = (data.get('type') or '').strip()
        if not concern_type:
            # Auto-detect type from concern_id prefix
            if concern_id.startswith('dconcern_'):
                concern_type = 'derived'
            else:
                concern_type = 'user'

        if concern_type == 'user':
            if action == 'close':
                for c in self.user_concern_model.concerns:
                    if c.get('concern_id') == concern_id:
                        c['status'] = 'closed'
                        c['end_disposition'] = 'resolved'
                        break
                self.user_concern_model._save()
                logger.info(f'📋 User concern {concern_id} closed via command')
            elif action == 'reopen':
                for c in self.user_concern_model.concerns:
                    if c.get('concern_id') == concern_id:
                        c['status'] = 'open'
                        break
                self.user_concern_model._save()
                logger.info(f'📋 User concern {concern_id} reopened via command')
            elif action == 'weight':
                weight = data.get('weight')
                if weight is not None:
                    for c in self.user_concern_model.concerns:
                        if c.get('concern_id') == concern_id:
                            c['weight'] = max(0.0, min(1.0, float(weight)))
                            break
                    self.user_concern_model._save()
                    logger.info(f'📋 User concern {concern_id} weight set to {weight}')
            elif action == 'delete':
                self.user_concern_model.concerns = [
                    c for c in self.user_concern_model.concerns
                    if c.get('concern_id') != concern_id
                ]
                self.user_concern_model._save()
                logger.info(f'📋 User concern {concern_id} deleted via command')
            else:
                return f"Unsupported action '{action}' for user concern"

        elif concern_type == 'derived':
            if action in ('resolve', 'satisfy'):
                patch = {
                    'op': 'satisfy_concern',
                    'concern_id': concern_id,
                    'field_updates': {'status_rationale': f'{action}d via command'},
                }
                revisit = data.get('revisit_hours')
                if revisit is not None:
                    patch['field_updates']['revisit_hours'] = float(revisit)
                self._derived_concern_model._apply_patch(patch, f'cmd:{action}')
                self._derived_concern_model._save()
                logger.info(f'📋 Derived concern {concern_id} {action}d via command')
            elif action == 'abandon':
                patch = {
                    'op': 'abandon_concern',
                    'concern_id': concern_id,
                    'field_updates': {'status_rationale': 'abandoned via command'},
                }
                self._derived_concern_model._apply_patch(patch, 'cmd:abandon')
                self._derived_concern_model._save()
                logger.info(f'📋 Derived concern {concern_id} abandoned via command')
            elif action == 'activate':
                patch = {
                    'op': 'activate_concern',
                    'concern_id': concern_id,
                    'field_updates': {'status_rationale': 'Reactivated via command'},
                }
                self._derived_concern_model._apply_patch(patch, 'cmd:reactivate')
                self._derived_concern_model._save()
                logger.info(f'📋 Derived concern {concern_id} reactivated via command')
            elif action == 'revisit':
                revisit = data.get('revisit_hours')
                if revisit is not None:
                    for c in self._derived_concern_model.concerns:
                        if c.get('concern_id') == concern_id:
                            c['revisit_hours'] = float(revisit)
                            break
                    self._derived_concern_model._save()
                    logger.info(f'📋 Derived concern {concern_id} revisit set to {revisit}h')
            elif action == 'delete':
                self._derived_concern_model.concerns = [
                    c for c in self._derived_concern_model.concerns
                    if c.get('concern_id') != concern_id
                ]
                self._derived_concern_model._save()
                logger.info(f'📋 Derived concern {concern_id} deleted via command')
            else:
                return f"Unsupported action '{action}' for derived concern"
        else:
            return f"Unknown concern type: {concern_type}"

        return f"Concern {concern_id} {action} done"

    # -- System --

    def _cmd_stop(self, data: dict) -> str:
        logger.info(f'⏹️ Stop command received by {self.character_name}')
        self.interrupt_requested = True
        if self.infospace_executor:
            self.infospace_executor.interrupt_requested = True
        self.execution_mode = 'step'
        self.execution_paused = True
        self.awaiting_user_input = False
        if self.awaiting_ask_response:
            self.awaiting_ask_response = False
            self._ask_response_queue.put(None)
        self._publish_execution_state()
        return "Execution stopped"

    def _cmd_continuous(self, data: dict) -> str:
        enable = data.get('enable')
        if enable is None:
            enable = not self.continuous_mode

        if enable:
            goal_text = None
            if self.current_goal:
                goal_text = self.current_goal.to_string()
            elif self.last_completed_goal_text:
                goal_text = self.last_completed_goal_text
            if goal_text:
                self.continuous_mode = True
                self.continuous_goal_text = goal_text
                if not self.current_goal and self.execution_paused:
                    goal_text_formatted = f"goal: {self.continuous_goal_text}"
                    self.parse_and_set_goal("", goal_text_formatted)
                self._publish_execution_state()
                return "Continuous mode enabled"
            else:
                self._publish_execution_state()
                return "Continuous mode: no current or last goal to repeat"
        else:
            self.continuous_mode = False
            self.continuous_goal_text = None
            self._publish_execution_state()
            return "Continuous mode disabled"

    def _cmd_llm_toggle(self, data: dict) -> str:
        new_mode = 'alt' if self.llm_mode == 'primary' else 'primary'
        if self._is_goal_running():
            self.llm_switch_pending = new_mode
            logger.info(f'🔄 {self.character_name} LLM switch to {new_mode} queued (goal in progress)')
            self._publish_execution_state()
            return f"LLM switch to {new_mode} queued (goal in progress)"
        else:
            self.llm_switch_pending = new_mode
            self._apply_pending_llm_switch()
            self._publish_execution_state()
            return f"LLM mode set to {new_mode}"

    def _cmd_delay(self, data: dict) -> str:
        delay = data.get('delay', 2.0)
        try:
            delay = float(delay)
        except (ValueError, TypeError):
            return 'Usage: /delay <seconds>'
        self.time_delay = delay
        logger.info(f'⏱️ Turn delay set to {delay}s for {self.character_name}')
        return f"Turn delay set to {delay}s"

    def _cmd_scheduler(self, data: dict) -> str:
        action = (data.get('action') or '').strip().lower()
        if action == 'on':
            self.goal_scheduler.set_enabled(True)
            self._publish_execution_state()
            return "Scheduler enabled"
        elif action == 'off':
            self.goal_scheduler.set_enabled(False)
            self._publish_execution_state()
            return "Scheduler disabled"
        elif action == 'interval':
            interval = data.get('interval')
            if interval is None:
                return 'Usage: /scheduler interval <seconds>'
            self.goal_scheduler.set_interval(float(interval))
            self._publish_execution_state()
            return f"Scheduler interval set to {interval}s"
        else:
            status = self.goal_scheduler.get_status()
            return (f"Scheduler: {'enabled' if status['enabled'] else 'disabled'}, "
                    f"interval={status['interval']}s, "
                    f"budget={status['budget_remaining_seconds']:.0f}s remaining")

    def _cmd_clear(self, data: dict) -> str:
        target = (data.get('target') or '').strip().lower()
        if target == 'world-model':
            self.handle_clear_world_model(None)
            return "World model cleared"
        elif target == 'map':
            self.handle_clear_map(None)
            return "Map cleared"
        elif target == 'transients':
            self.handle_clear_transients(None)
            return "Transients cleared"
        elif target == 'persistents':
            self.handle_clear_persistents(None)
            return "Persistents cleared"
        else:
            return 'Usage: /clear <world-model|map|transients|persistents>'

    def _cmd_save(self, data: dict) -> str:
        self._handle_save_command(None)
        return "Save requested"

    def _cmd_shutdown(self, data: dict) -> str:
        self._handle_save_command(None)
        self.shutdown_requested = True
        return "Save and shutdown requested"

    def _cmd_bye(self, data: dict) -> str:
        self.conversation_store.close_dialog("User")
        logger.info(f'📥 {self.character_name} User ended conversation via /bye')
        return "Conversation ended"

    def _cmd_direct_action(self, data: dict) -> str:
        json_text = (data.get('json_text') or '').strip()
        if not json_text:
            return 'Usage: /action <json>'
        try:
            action_dict = json.loads(json_text)
        except json.JSONDecodeError as e:
            return f'Invalid JSON: {e}'
        if self.infospace_executor:
            self.infospace_executor.execute_action(action_dict)
            return "Action executed"
        return "No executor available"

    # ── OODA pipeline ───────────────────────────────────────────────────

    def _ooda_observe(self) -> Optional[EventPacket]:
        """OBSERVE: Dequeue one event, classify it, return EventPacket or None."""

        # --- Helper: parse sense_data JSON into (text, source, close_flag) ---
        def _parse_sense(sense_data: dict) -> Tuple[str, str, bool]:
            content = sense_data.get('content', '')
            try:
                d = json.loads(content)
                return d.get('text', ''), d.get('source', 'unknown'), d.get('close', False)
            except (json.JSONDecodeError, TypeError):
                return content, 'console', False

        # Priority 1: sensor alerts
        if self._sensor_alert_queue:
            entry = self._sensor_alert_queue.pop(0)
            return EventPacket(
                event_type='sensor_event', classification='alert',
                content=entry.get('content', ''), source=f"sensor:{entry.get('sensor_name', 'unknown')}",
                raw_sense_data=entry,
            )

        # Priority 2: sensor triggers (reaction path)
        if self._sensor_trigger_queue:
            entry = self._sensor_trigger_queue.pop(0)
            return EventPacket(
                event_type='sensor_event', classification='trigger',
                content=entry.get('content', ''), source=f"sensor:{entry.get('sensor_name', 'unknown')}",
                raw_sense_data=entry, goal_name=entry.get('goal_name'),
                is_reaction=True,
            )

        # Priority 2b: sensor trigger-task (task creation path)
        if self._sensor_trigger_task_queue:
            entry = self._sensor_trigger_task_queue.pop(0)
            return EventPacket(
                event_type='sensor_event', classification='trigger_task',
                content=entry.get('content', ''), source=f"sensor:{entry.get('sensor_name', 'unknown')}",
                raw_sense_data=entry, task_template=entry.get('task_template'),
                is_reaction=True,
            )

        # Priority 3: sensor inform (context that Orient should assess)
        if self._sensor_inform_queue:
            entry = self._sensor_inform_queue.pop(0)
            return EventPacket(
                event_type='sensor_event', classification='inform',
                content=entry.get('content', ''), source=f"sensor:{entry.get('sensor_name', 'unknown')}",
                raw_sense_data=entry,
            )

        # Priority 4: text input queue
        if not self.text_input_queue:
            # Priority 5: periodic orientation timer — only when truly idle
            # (no goal running, no queued input). During goals or chat,
            # Orient gets events through the normal paths.
            if not self._is_goal_running():
                now = time.monotonic()
                if (now - self._last_orient_timer) >= self._orient_timer_interval:
                    self._last_orient_timer = now
                    # Skip timer event entirely when nothing has changed since
                    # last timer — avoids LLM call, state saves, and graph writes.
                    if not self._non_timer_event_since_last_orient:
                        return None
                    self._non_timer_event_since_last_orient = False
                    return EventPacket(
                        event_type='timer', classification='timer',
                        content='periodic orientation tick',
                        source='internal', raw_sense_data={},
                    )
            return None

        sense_data = self.text_input_queue[0]
        text, source, close_flag = _parse_sense(sense_data)
        if not text or not text.strip():
            self.text_input_queue.pop(0)
            return None

        # Ask-reply intercept
        if self.awaiting_ask_response:
            if source == 'User':
                self.text_input_queue.pop(0)
                return EventPacket(
                    event_type='user_text', classification='ask_reply',
                    content=text, source=source, raw_sense_data=sense_data,
                )
            return None  # ignore non-User while awaiting ask

        self.text_input_queue.pop(0)

        # Classify by source — no keyword/intent parsing.
        # All imperative commands arrive via the /command channel, not here.
        is_agent = source not in ('unknown', 'console', 'User', 'scheduler')

        if is_agent:
            classification = 'agent_message'
            event_type = 'user_text'
        elif source == 'User':
            classification = 'chat'
            event_type = 'user_text'
        else:
            # Unknown source — drop
            return None

        return EventPacket(
            event_type=event_type, classification=classification,
            content=text, source=source, raw_sense_data=sense_data,
            close_flag=close_flag,
        )

    def _ooda_orient(self, event: EventPacket) -> OrientedEvent:
        """ORIENT: Evaluate event significance via character evaluator. Single eval site."""
        assessment = None

        # Track non-timer events so _ooda_observe can skip idle timer ticks.
        if event.event_type != 'timer':
            self._non_timer_event_since_last_orient = True

        if self._character_eval_enabled():
            try:
                content = event.content
                source = event.source
                sensor_name = ''
                disposition = ''
                if event.event_type == 'sensor_event':
                    sensor_name = event.raw_sense_data.get('sensor_name', '')
                    disposition = event.raw_sense_data.get('disposition', '')

                nsrc = self._character_eval_normalize_source(source, sensor_name)

                uc: List[Dict[str, Any]] = []
                try:
                    uc = self.user_concern_model.get_concerns(active_only=False) or []
                except Exception:
                    pass

                # After the command registry refactor, imperative commands no longer
                # flow through observe/orient. Only sensor triggers remain as
                # goal-like events here.
                is_goal_command = False
                is_proceed_like = event.classification == 'trigger'

                ev = character_evaluator.build_event_dict(
                    event.event_type, content, nsrc,
                    disposition=disposition, sensor_name=sensor_name,
                    is_goal_command=is_goal_command, is_proceed_like=is_proceed_like,
                )
                target_goal_id = event.goal_id if is_proceed_like else None
                # Combine fixed character concerns with agent-derived concerns
                all_concerns = list(character_evaluator.DEFAULT_CHARACTER_CONCERNS)
                try:
                    all_concerns += self._derived_concern_model.get_concerns_for_evaluator()
                except Exception:
                    pass
                with self.infospace_executor.turn_metrics.perf_phase("orient"):
                    assessment = character_evaluator.evaluate(
                        ev, all_concerns, uc,
                        self._character_eval_build_goals_compact(),
                        self._character_eval_build_recent_context(),
                        self._character_eval_build_activity_state(),
                        None,
                        llm_generate=self.llm_generate,
                        target_goal_id=target_goal_id,
                    )
                character_evaluator.log_assessment(assessment)
            except Exception as e:
                logger.warning(f"Orient: character evaluation skipped: {e}")

        self._last_character_eval = assessment

        # Update character concern activations (exponential decay)
        if assessment:
            self._update_character_concern_activations(assessment)
            # Orient-stage triage nomination: strong bump on a cold concern
            self._triage_orient_nominations(assessment)

        # ── Graph: orient ──
        self._graph_emit_orient(event, assessment)

        return OrientedEvent(event=event, assessment=assessment)

    def _get_concern_weight(self, concern_id: str) -> float:
        """Look up a concern's weight. Derived concerns use their stored weight;
        fixed character concerns default to 0.5."""
        try:
            for c in self._derived_concern_model.get_concerns():
                if c.get('concern_id') == concern_id:
                    return float(c.get('weight', 0.5))
        except Exception:
            pass
        # Fixed character concerns (homeostasis, attend_to_user, etc.)
        return 0.5

    def _update_character_concern_activations(self, assessment: Dict[str, Any]):
        """Update running character concern activation levels from assessment.

        Bump magnitude is scaled by concern weight: bump * (1 + weight).
        A weight=1.0 concern gets 2x the bump; weight=0 gets 1x.
        """
        DECAY = 0.9
        BUMP = {'strong': 0.3, 'moderate': 0.15, 'weak': 0.05, 'none': 0.0}
        notes = assessment.get('notes', '')
        if 'concerns:' not in notes:
            # Decay all activations
            for cid in self._character_concern_activations:
                self._character_concern_activations[cid] *= DECAY
            return
        # Parse "concerns: homeostasis=strong, attend_to_user=moderate"
        for segment in notes.split(';'):
            segment = segment.strip()
            if segment.startswith('concerns:'):
                pairs = segment[len('concerns:'):].strip()
                for pair in pairs.split(','):
                    pair = pair.strip()
                    if '=' in pair:
                        cid, level = pair.rsplit('=', 1)
                        cid = cid.strip()
                        level = level.strip()
                        old = self._character_concern_activations.get(cid, 0.0)
                        weight = self._get_concern_weight(cid)
                        new_val = old * DECAY + BUMP.get(level, 0.0) * (1.0 + weight)
                        self._character_concern_activations[cid] = new_val
                        # ── Graph: concern change ──
                        if abs(new_val - old) >= self._cognitive_graph._config.get("concern_change_threshold", 0.05):
                            self._graph_emit_concern_change(cid, old, new_val, level)
                break

    def _triage_orient_nominations(self, assessment: Dict[str, Any]):
        """Check orient assessment for strong bumps on cold concerns — nominate for triage."""
        notes = assessment.get('notes', '')
        if 'concerns:' not in notes:
            return
        BUMP_LEVELS = {'strong', 'moderate', 'weak', 'none'}
        for segment in notes.split(';'):
            segment = segment.strip()
            if not segment.startswith('concerns:'):
                continue
            pairs = segment[len('concerns:'):].strip()
            for pair in pairs.split(','):
                pair = pair.strip()
                if '=' not in pair:
                    continue
                cid, level = pair.rsplit('=', 1)
                cid, level = cid.strip(), level.strip()
                if level not in BUMP_LEVELS:
                    continue
                activation = self._character_concern_activations.get(cid, 0.0)
                # Look up concern description from derived concerns
                desc, label = '', cid
                try:
                    for c in self._derived_concern_model.get_concerns(active_only=True):
                        if c.get('concern_id') == cid:
                            label = c.get('concern_label', cid)
                            desc = c.get('concern_description', '')
                            break
                except Exception:
                    pass
                self._concern_triage.nominate_from_orient(
                    concern_id=cid,
                    concern_label=label,
                    concern_description=desc,
                    activation=activation,
                    bump_level=level,
                )
                # ── Graph: triage nomination ──
                self._graph_emit_triage_nomination(cid, label, activation, level, "orient")
            break

    def _ooda_decide(self, oriented: OrientedEvent) -> Action:
        """DECIDE: Pure routing — map classification to Action. No LLM calls.

        After the command registry refactor, only these classifications arrive
        from observe: ask_reply, alert, trigger, inform, timer, agent_message, chat.
        inform and timer fall through Tier 1 to Tier 2 (Orient-driven decision).
        All imperative operations go through _dispatch_command() instead.
        """
        evt = oriented.event
        a = oriented.assessment

        if evt.classification == 'ask_reply':
            return Action('ask_reply', {'text': evt.content}, a)

        if evt.classification == 'trigger':
            goal_id = self._resolve_goal_id(evt.goal_name)
            if goal_id:
                # Honor the goal's execution_mode the same way /goal run does
                # (see _cmd_goal_run). If the goal is in replay mode and has
                # a cached plan, route the trigger to reuse; otherwise plan
                # from scratch. This lets sensor-triggered runs benefit from
                # the review cycle (a hand-fixed cached plan stays in effect
                # across triggers instead of being silently overwritten on
                # every fire).
                triggered_goal = self._get_scheduled_goal(goal_id)
                if (triggered_goal
                        and triggered_goal.get("execution_mode") == "replay"
                        and triggered_goal.get("cached_plan_actions")):
                    return Action('reuse_goal', {'goal_id': goal_id, 'source': 'sensor_trigger'}, a)
                return Action('proceed_goal', {'goal_id': goal_id, 'source': 'sensor_trigger'}, a)
            logger.warning(f"Decide: triggered goal '{evt.goal_name}' not found")
            return Action('no_action', {}, a)

        if evt.classification == 'trigger_task':
            return Action('trigger_task', {
                'task_template': evt.task_template,
                'sensor_content': evt.content,
                'sensor_data': evt.raw_sense_data,
            }, a)

        if evt.classification == 'alert':
            alert_text, sensor_name = self._format_alert_text(evt.raw_sense_data)
            return Action('alert_response', {'alert_text': alert_text, 'sensor_name': sensor_name}, a)

        if evt.classification == 'chat':
            return Action('chat_response', {'text': evt.content, 'source': evt.source}, a)

        if evt.classification == 'agent_message':
            return Action('agent_message', {'text': evt.content, 'source': evt.source,
                          'close_flag': evt.close_flag}, a)

        # Tier 2: consult Orient's assessment for unrouted events
        if a and (a.get('action_evaluation') or {}).get('action_choice', 'no_action') != 'no_action':
            return self._decide_tier2(oriented)

        return Action('no_action', {}, a)

    # ── Tier 2 Decide ──────────────────────────────────────────────────

    def _decide_tier2(self, oriented: OrientedEvent) -> Action:
        """Tier 2 Decide: consult Orient's assessment when Tier 1 has no route.

        Rules-based. Checks Orient's action recommendation against agent state
        and returns an appropriate instantiation.
        """
        a = oriented.assessment
        evt = oriented.event
        action_choice = (a.get('action_evaluation') or {}).get('action_choice', 'no_action')
        sal = a.get('salience_factors') or {}
        novelty = sal.get('novelty', 'none')
        persistence = sal.get('persistence_worthiness', 'none')

        # Parse concern bumps to detect "nothing matched"
        concern_bumps = self._parse_orient_concern_bumps(a)
        has_strong_match = any(v in ('strong', 'moderate') for v in concern_bumps.values())

        # Side effect: surface new concern if novel + persistent + no match
        if (novelty in ('moderate', 'high')
                and persistence in ('moderate', 'high')
                and not has_strong_match):
            self._trigger_concern_from_event(evt, a)

        # Instantiation: remark
        if action_choice == 'inform_user':
            if self._can_proactive_remark(novelty):
                return Action('proactive_remark', {
                    'text': evt.content,
                    'source': evt.source,
                    'rationale': a.get('overall_rationale', ''),
                }, a)
            return Action('no_action', {}, a)

        # Internal timer events should never trigger task proposals or goal formulation
        if evt.classification == 'timer':
            return Action('no_action', {}, a)

        # Instantiation: trigger an existing scheduled goal
        if action_choice == 'trigger_existing_goal':
            if not self._is_goal_running() and not self.active_task_wip:
                target_goal_id = getattr(evt, 'goal_id', None)
                if target_goal_id:
                    return Action('trigger_existing_goal', {
                        'goal_id': target_goal_id,
                        'source': evt.source,
                    }, a)
            return Action('no_action', {}, a)

        # Instantiation: formulate new goal
        if action_choice == 'formulate_new_goal':
            if not self._is_goal_running() and not self.active_task_wip:
                goal_text = self._formulate_goal_from_orient(evt, a)
                if goal_text:
                    return Action('dispatch_goal', {'goal_text': goal_text}, a)
            return Action('no_action', {}, a)

        return Action('no_action', {}, a)

    def _parse_orient_concern_bumps(self, assessment: Dict[str, Any]) -> Dict[str, str]:
        """Extract concern_id → bump_level from Orient's notes field."""
        notes = (assessment or {}).get('notes', '')
        bumps = {}
        if 'concerns:' not in notes:
            return bumps
        for segment in notes.split(';'):
            segment = segment.strip()
            if segment.startswith('concerns:'):
                for pair in segment[len('concerns:'):].strip().split(','):
                    pair = pair.strip()
                    if '=' in pair:
                        cid, level = pair.rsplit('=', 1)
                        bumps[cid.strip()] = level.strip()
                break
        return bumps

    def _can_proactive_remark(self, novelty: str) -> bool:
        """Check suppression guards for proactive remarks."""
        if self._is_goal_running() or self.active_task_wip:
            return False
        if novelty == 'none':
            return False
        now = time.monotonic()
        if (now - self._last_proactive_remark_at) < self._proactive_remark_cooldown:
            return False
        return True

    # ── Cognitive Graph Emission Helpers ──────────────────────────────

    def _graph_emit_observe(self, event):
        """Emit event node + conversation_turn for incoming messages."""
        try:
            g = self._cognitive_graph
            nid = g.add_node("event", event.content[:500],
                attrs={"event_type": event.event_type,
                       "classification": event.classification,
                       "source": event.source})
            self._last_event_node = nid
            # Conversation turn for user/agent messages
            if event.event_type == 'user_text':
                turn_nid = g.add_node("conversation_turn", event.content[:500],
                    attrs={"source": event.source, "direction": "in",
                           "entity": event.source})
                g.add_edge(turn_nid, nid, "triggered_by")
                # NER: extract entities from user input and link to graph
                self._extract_and_index_entities(event.content, "", turn_nid)
        except Exception as e:
            logger.debug(f"Graph observe emit failed: {e}")

    def _graph_emit_orient(self, event, assessment):
        """Emit assessment node linked to event node."""
        try:
            if not assessment:
                return
            g = self._cognitive_graph
            rationale = assessment.get('overall_rationale', '')
            action_eval = assessment.get('action_evaluation', {}) or {}
            action_choice = action_eval.get('action_choice', 'no_action')
            salience = assessment.get('salience_factors', {}) or {}
            # Build concern bumps dict from notes
            bumps = {}
            notes = assessment.get('notes', '')
            for seg in notes.split(';'):
                seg = seg.strip()
                if seg.startswith('concerns:'):
                    for pair in seg[len('concerns:'):].strip().split(','):
                        if '=' in pair:
                            k, v = pair.rsplit('=', 1)
                            bumps[k.strip()] = v.strip()
            nid = g.add_node("assessment", rationale[:500],
                attrs={"concern_bumps": bumps,
                       "action_choice": action_choice,
                       "salience": salience})
            self._last_assessment_node = nid
            if self._last_event_node:
                g.add_edge(self._last_event_node, nid, "observed")
        except Exception as e:
            logger.debug(f"Graph orient emit failed: {e}")

    def _graph_emit_concern_change(self, cid, old_val, new_val, level):
        """Emit concern_change node linked to assessment."""
        try:
            g = self._cognitive_graph
            # Look up label
            label = cid
            try:
                for c in self._derived_concern_model.get_concerns(active_only=True):
                    if c.get('concern_id') == cid:
                        label = c.get('concern_label', cid)
                        break
            except Exception:
                pass
            nid = g.add_node("concern_change",
                f"Concern '{label}' activation {old_val:.2f} → {new_val:.2f} ({level} bump)",
                attrs={"concern_id": cid, "concern_label": label,
                       "old_activation": round(old_val, 4),
                       "new_activation": round(new_val, 4),
                       "trigger": level})
            self._graph_node_by_key[f"concern_change:{cid}"] = nid
            if self._last_assessment_node:
                g.add_edge(self._last_assessment_node, nid, "bumped")
        except Exception as e:
            logger.debug(f"Graph concern_change emit failed: {e}")

    def _graph_emit_triage_nomination(self, cid, label, activation, bump_or_trend, source):
        """Emit triage_nomination node."""
        try:
            g = self._cognitive_graph
            nid = g.add_node("triage_nomination",
                f"Concern '{label}' nominated: activation={activation:.2f}, {source}",
                attrs={"concern_id": cid, "concern_label": label,
                       "activation": round(activation, 4),
                       "trend": str(bump_or_trend),
                       "nomination_source": source})
            self._graph_node_by_key[f"triage_nom:{cid}"] = nid
            # Link to concern_change if available
            cc_node = self._graph_node_by_key.get(f"concern_change:{cid}")
            if cc_node:
                g.add_edge(cc_node, nid, "nominated")
        except Exception as e:
            logger.debug(f"Graph triage nomination emit failed: {e}")

    def _graph_emit_triage_decision(self, d):
        """Emit triage_decision node linked to nomination."""
        try:
            g = self._cognitive_graph
            nid = g.add_node("triage_decision", getattr(d, 'reason', '')[:500],
                attrs={"concern_id": getattr(d, 'concern_id', ''),
                       "action": getattr(d, 'action', ''),
                       "task_intention": getattr(d, 'task_intention', '') or ''})
            nom_node = self._graph_node_by_key.get(f"triage_nom:{d.concern_id}")
            if nom_node:
                g.add_edge(nom_node, nid, "triaged")
        except Exception as e:
            logger.debug(f"Graph triage decision emit failed: {e}")

    def _graph_emit_decide(self, action):
        """Emit decision node linked to assessment."""
        try:
            g = self._cognitive_graph
            payload_summary = ''
            if action.payload:
                payload_summary = str(action.payload)[:200]
            # Build human-readable content (renderer prefixes with "Decided:")
            content = action.type
            if action.type in ('chat_response', 'agent_message'):
                source = (action.payload or {}).get('source', '')
                content = f"{action.type} to {source}" if source else action.type
            elif action.type == 'dispatch_goal':
                content = f"dispatch goal: {(action.payload or {}).get('goal_text', '')[:100]}"
            elif action.type == 'proceed_goal':
                content = f"proceed goal {(action.payload or {}).get('goal_id', '')}"
            elif action.type == 'reuse_goal':
                content = f"reuse goal {(action.payload or {}).get('goal_id', '')}"
            nid = g.add_node("decision", content[:500],
                attrs={"action_type": action.type,
                       "payload_summary": payload_summary})
            self._last_decision_node = nid
            if self._last_assessment_node:
                g.add_edge(self._last_assessment_node, nid, "decided_from")
        except Exception as e:
            logger.debug(f"Graph decide emit failed: {e}")

    def _graph_emit_action_result(self, action_type, success):
        """Emit action_result node linked to decision."""
        try:
            g = self._cognitive_graph
            nid = g.add_node("action_result",
                f"Executed {action_type}: {'success' if success else 'failed'}",
                attrs={"action_type": action_type, "success": success})
            if self._last_decision_node:
                g.add_edge(self._last_decision_node, nid, "executed")
        except Exception as e:
            logger.debug(f"Graph action_result emit failed: {e}")

    def _graph_emit_goal_launch(self, goal_id, goal_text, source):
        """Emit goal_launch node linked to decision."""
        try:
            g = self._cognitive_graph
            nid = g.add_node("goal_launch", (goal_text or '')[:500],
                attrs={"goal_id": goal_id, "source": source, "status": "active"})
            self._graph_node_by_key[f"goal:{goal_id}"] = nid
            if self._last_decision_node:
                g.add_edge(self._last_decision_node, nid, "spawned_goal")
            # NER: extract entities from goal text, link to graph, and cache
            # on executor so search_resources can use them without re-extracting
            if goal_text:
                entities = self._extract_and_index_entities(goal_text, "", nid)
                if self.infospace_executor and entities:
                    self.infospace_executor._goal_entities = entities
        except Exception as e:
            logger.debug(f"Graph goal_launch emit failed: {e}")

    def _graph_emit_goal_outcome(self, goal_id, success, last_result, primary_product, result):
        """Emit goal_outcome node and update goal_launch status."""
        try:
            g = self._cognitive_graph
            nid = g.add_node("goal_outcome", (last_result or '')[:500],
                attrs={"goal_id": goal_id, "success": success,
                       "primary_product": primary_product or '',
                       "quality_status": (result or {}).get("quality_status", ""),
                       "verification_answer": (result or {}).get("verification_answer", "")})
            launch_node = self._graph_node_by_key.get(f"goal:{goal_id}")
            if launch_node:
                g.add_edge(launch_node, nid, "produced")
                g.update_attrs(launch_node, {"status": "completed" if success else "failed"})
            # NER: extract entities from goal outcome
            if last_result:
                self._extract_and_index_entities((last_result or '')[:500], primary_product or '', nid)
        except Exception as e:
            logger.debug(f"Graph goal_outcome emit failed: {e}")

    def _graph_emit_task_created(self, wip_id, intention, concern_id):
        """Emit task_created node."""
        try:
            g = self._cognitive_graph
            nid = g.add_node("task_created", intention[:500],
                attrs={"task_wip_id": wip_id,
                       "linked_concern_id": concern_id,
                       "phase": "proposed"})
            self._graph_node_by_key[f"task:{wip_id}"] = nid
        except Exception as e:
            logger.debug(f"Graph task_created emit failed: {e}")

    def _graph_emit_concern_created(self, concern):
        """Emit concern_created node for a newly derived concern."""
        try:
            g = self._cognitive_graph
            cid = concern.get('concern_id', '')
            label = concern.get('concern_label', cid)
            desc = concern.get('concern_description', '')
            nid = g.add_node("concern_created", f"{label}: {desc}"[:500],
                attrs={"concern_id": cid, "concern_label": label,
                       "origin": concern.get('origin', ''),
                       "weight": concern.get('weight', 1.0)})
            self._graph_node_by_key[f"concern_change:{cid}"] = nid
            # Link to assessment that triggered creation
            if self._last_assessment_node:
                g.add_edge(nid, self._last_assessment_node, "concern_for")
            # NER: extract entities from concern label+description
            if label or desc:
                self._extract_and_index_entities(f"{label}: {desc}", "", nid)
        except Exception as e:
            logger.debug(f"Graph concern_created emit failed: {e}")

    # ── Entity Extraction (NER Pipeline) ────────────────────────────

    def _extract_and_index_entities(self, text: str, resource_id: str,
                                     graph_node: str = "") -> Optional[Dict]:
        """Extract entities from text and add to entity index + cognitive graph.

        Called from graph emit helpers (observe, goal_launch) and idle-tick
        persistent-note processing.  Runs synchronously but is lightweight
        (~256 max_tokens, temperature 0.1).

        Returns the extracted entities dict, or None on failure.
        """
        try:
            entities = self._entity_index.extract_entities(text, self.llm_generate)
            if any(entities.get(k) for k in ("people", "organizations", "locations", "topics")):
                self._entity_index.index_entities(entities, resource_id, graph_node)
                flat = [e for k in ("people", "organizations", "locations", "topics") for e in entities.get(k, [])]
                logger.info(f'🏷 NER: {flat}')
                return entities
        except Exception as e:
            logger.debug(f"Entity extraction failed: {e}")
        return None

    def _entity_index_process_persistent_notes(self):
        """Batch-extract entities from persistent Notes not yet processed.

        Called from idle tick.  Processes at most 3 notes per tick to avoid
        blocking the main loop.
        """
        if not self.resource_manager:
            return
        processed = 0
        for note_id, note_data in self.resource_manager.resource_registry.items():
            if processed >= 3:
                break
            if not note_id.startswith('Note_'):
                continue
            props = note_data.get('properties', {})
            if not props.get('persistent', False):
                continue
            if props.get('entities_extracted', False):
                continue
            # Skip internal notes
            name = props.get('note_name', '')
            if name.startswith('_'):
                continue
            content = str(props.get('content', ''))
            if len(content) < 30:
                continue
            # Create a graph node for the persistent note so entity edges can link to it
            try:
                note_nid = self._cognitive_graph.add_node(
                    "note", (name or note_id)[:200],
                    attrs={"resource_id": note_id, "note_name": name})
            except Exception:
                note_nid = ""
            self._extract_and_index_entities(content, note_id, note_nid)
            props['entities_extracted'] = True
            processed += 1
        if processed:
            logger.info(f"🏷 Entity extraction: processed {processed} persistent notes")

    def _save_entity_index(self):
        """Persist entity index to disk alongside cognitive graph."""
        try:
            if self._cognitive_graph_path:
                import json as _json
                ei_path = str(Path(self._cognitive_graph_path).parent / "entity_index.json")
                with open(ei_path, 'w') as f:
                    _json.dump(self._entity_index.to_dict(), f)
        except Exception as e:
            logger.debug(f"Entity index save failed: {e}")

    # ── Cognitive Graph Query Handlers (for resource browser graph tab) ──

    def _handle_graph_subgraph_query(self, query):
        """Handle subgraph expansion or semantic search over the cognitive graph."""
        try:
            payload = {}
            if query.payload:
                payload = json.loads(query.payload.to_bytes().decode('utf-8'))

            g = self._cognitive_graph
            MAX_NODES = 400

            if 'seed_types' in payload:
                # Seed by node type — return most recent nodes of those types
                # (no BFS needed; we're selecting directly by type)
                seed_types = set(payload['seed_types'])
                with g._lock:
                    type_nodes = [dict(n) for n in g._nodes.values()
                                  if n.get('type') in seed_types]
                # Sort by timestamp descending, take most recent up to MAX_NODES
                type_nodes.sort(key=lambda n: n.get('timestamp', ''), reverse=True)
                nodes = type_nodes[:MAX_NODES]
                node_ids = {n['node_id'] for n in nodes}
                # Include all edges between the selected nodes
                with g._lock:
                    edges = [dict(e) for e in g._edges.values()
                             if e['source'] in node_ids and e['target'] in node_ids]
            elif 'seed_ids' in payload:
                # Expand mode
                seed_ids = payload['seed_ids']
                max_hops = payload.get('max_hops', 2)
                edge_types = payload.get('edge_types')
                nodes, edges = g.expand_subgraph(seed_ids, max_hops, edge_types)
                nodes = nodes[:MAX_NODES]
                # Filter edges to only include nodes in the result set
                node_ids = {n['node_id'] for n in nodes}
                edges = [e for e in edges if e['source'] in node_ids and e['target'] in node_ids]
            elif 'query' in payload:
                # Search mode — semantic search then auto-expand 1 hop
                query_text = payload['query']
                k = payload.get('k', 10)
                type_filter = payload.get('type_filter')
                results = g.semantic_search(query_text, k=k, type_filter=type_filter)
                seed_ids = [nid for nid, score in results if score >= 0.3]
                if seed_ids:
                    nodes, edges = g.expand_subgraph(seed_ids, max_hops=1)
                    nodes = nodes[:MAX_NODES]
                    node_ids = {n['node_id'] for n in nodes}
                    edges = [e for e in edges if e['source'] in node_ids and e['target'] in node_ids]
                else:
                    nodes, edges = [], []
            else:
                nodes, edges = [], []

            response = {'success': True, 'nodes': nodes, 'edges': edges}
            query.reply(query.key_expr, json.dumps(response, default=str).encode('utf-8'))
        except Exception as e:
            logger.warning(f"Graph subgraph query failed: {e}")
            query.reply(query.key_expr, json.dumps(
                {'success': False, 'error': str(e)}).encode('utf-8'))

    def _handle_graph_entities_query(self, query):
        """Return entity index summary for the graph explorer."""
        try:
            ei_data = self._entity_index.to_dict()
            # Add mention counts for sizing
            entity_summary = []
            for name, resource_ids in ei_data.get('index', {}).items():
                graph_nid = ei_data.get('entity_nodes', {}).get(name, '')
                entity_summary.append({
                    'name': name,
                    'mention_count': len(resource_ids),
                    'graph_node_id': graph_nid,
                    'resource_ids': list(resource_ids)[:20],  # cap for payload size
                })
            response = {
                'success': True,
                'entities': entity_summary,
                'aliases': ei_data.get('aliases', {}),
            }
            query.reply(query.key_expr, json.dumps(response, default=str).encode('utf-8'))
        except Exception as e:
            logger.warning(f"Graph entities query failed: {e}")
            query.reply(query.key_expr, json.dumps(
                {'success': False, 'error': str(e)}).encode('utf-8'))

    def _record_ooda_event(self, event, oriented, action):
        """Record an OODA cycle event for the UI feed."""
        try:
            assessment = oriented.assessment or {}
            bumps = self._parse_orient_concern_bumps(assessment)
            # Compact representation of concern bumps (only non-none)
            bump_str = ', '.join(
                f'{k}={v}' for k, v in bumps.items() if v != 'none'
            ) if bumps else ''
            action_eval = (assessment.get('action_evaluation') or {})
            entry = {
                'timestamp': datetime.now().isoformat(timespec='seconds'),
                'source': event.source if event else '?',
                'event_type': event.event_type if event else '?',
                'classification': event.classification if event else '?',
                'action_choice': action_eval.get('action_choice', ''),
                'action_taken': action.type if action else '',
                'concern_bumps': bump_str,
            }
            self._ooda_event_feed.append(entry)
            if len(self._ooda_event_feed) > self._OODA_FEED_MAX:
                self._ooda_event_feed = self._ooda_event_feed[-self._OODA_FEED_MAX:]
        except Exception:
            pass

    def _trigger_concern_from_event(self, evt: EventPacket, assessment: Dict[str, Any]):
        """Ask derived concern model whether this event warrants a new concern."""
        try:
            content = evt.content if isinstance(evt.content, str) else str(evt.content)
            rationale = assessment.get('overall_rationale', '')
            interaction_text = (
                f"Observed event (source: {evt.source}):\n"
                f"{content[:500]}\n"
                f"Orient assessment: {rationale}"
            )
            evidence_ref = f"orient_event:{datetime.now().isoformat()}"
            uc = self.user_concern_model.get_concerns(active_only=False) or []
            # Snapshot concern IDs before update to detect new ones
            _pre_ids = {c.get('concern_id') for c in self._derived_concern_model.get_concerns()}
            self._derived_concern_model.update_from_event(interaction_text, uc, evidence_ref)
            # ── Graph: concern created ──
            try:
                for c in self._derived_concern_model.get_concerns():
                    cid = c.get('concern_id', '')
                    if cid and cid not in _pre_ids:
                        self._graph_emit_concern_created(c)
            except Exception:
                pass
        except Exception as e:
            logger.debug(f'Tier 2 concern trigger failed: {e}')

    def _formulate_goal_from_orient(self, evt: EventPacket, assessment: Dict[str, Any]) -> Optional[str]:
        """Generate goal text from Orient's assessment + event content."""
        content = evt.content if isinstance(evt.content, str) else str(evt.content)
        rationale = assessment.get('overall_rationale', '')
        try:
            result = self.llm_generate(
                messages=[
                    {"role": "system", "content":
                        "You formulate a concise goal statement for an autonomous agent. "
                        "The goal should be a single actionable sentence describing what to accomplish. "
                        "Respond with ONLY the goal text, nothing else. End with </end>."},
                    {"role": "user", "content":
                        f"Event: {content[:400]}\n"
                        f"Assessment: {rationale}\n\n"
                        f"Formulate a goal for the agent to pursue based on this event."},
                ],
                max_tokens=612,
                temperature=0.3,
                stops=['</end>'],
            )
            if result.success and result.text:
                return result.text.strip()
        except Exception as e:
            logger.debug(f'Tier 2 goal formulation failed: {e}')
        return None

    def _resolve_goal_id(self, ref: str) -> Optional[str]:
        """Resolve a goal reference (goal_id, name, or goal_text) to a goal_id."""
        if not ref:
            return None
        for goal in self._all_scheduled_goals():
            if (goal.get('goal_id') == ref
                    or goal.get('name') == ref
                    or goal.get('goal_text', '').strip() == ref):
                return goal.get('goal_id')
        return None

    def _format_alert_text(self, entry: dict) -> Tuple[str, str]:
        """Format a sensor alert entry into (alert_text, sensor_name)."""
        sensor_name = entry.get('sensor_name', 'unknown')
        summary = entry.get('content', '')
        data = entry.get('data', [])
        alert_text = f"[ALERT from sensor {sensor_name}]: {summary}"
        if data:
            for item in data:
                title = item.get('title', '')
                url = item.get('url', '')
                label = f"{title} — {url}" if title else url
                alert_text += f"\n  - {label}"
        return alert_text, sensor_name

    def _ooda_act(self, action: Action):
        """ACT: Execute the chosen action. Single dispatch point."""
        t = action.type
        p = action.payload

        if t == 'no_action':
            return

        if t == 'ask_reply':
            self._ask_response_queue.put(p['text'])
            return

        if t == 'alert_response':
            self._handle_sensor_alert_response(p['alert_text'], p['sensor_name'])
            return

        if t == 'dispatch_goal':
            scheduled_goal = self._upsert_scheduled_goal(p['goal_text'])
            goal_id = scheduled_goal["goal_id"]
            # Auto-created goals are ephemeral (deleted on completion)
            scheduled_goal['ephemeral'] = True
            self._save_scheduled_goal(scheduled_goal)
            # ── Graph: goal launch ──
            self._graph_emit_goal_launch(goal_id, p['goal_text'], "user")
            if self._is_goal_running():
                self._say_to_user("A goal is already running. Please wait for it to complete.")
                return
            self._active_scheduled_goal_id = goal_id
            self._update_scheduled_goal(goal_id, is_running=True, status="running")
            if action.assessment:
                self._update_scheduled_goal(goal_id, initial_assessment=action.assessment)
            pre = set(self.resource_manager.resource_registry.keys()) if self.resource_manager else set()
            goal_text = p['goal_text']
            def _run():
                result = self.parse_and_set_goal("", goal_text) or {}
                self._set_scheduled_goal_result(goal_id, result, used_cache=False, pre_resource_ids=pre)
                return result
            self._run_goal_on_thread(_run)
            return

        if t == 'trigger_existing_goal':
            self._handle_goal_proceed(goal_id=p['goal_id'], source=p.get('source', 'user'))
            return

        if t == 'trigger_task':
            template_name = p.get('task_template', '')
            self._handle_trigger_task(template_name, p.get('sensor_content', ''), p.get('sensor_data', {}))
            return

        if t == 'proceed_goal':
            if not p.get('goal_id'):
                self._say_to_user("Please specify which goal to proceed, e.g. 'proceed goal_1'.")
                return
            # ── Graph: goal launch (proceed) ──
            goal = self._get_scheduled_goal(p['goal_id'])
            self._graph_emit_goal_launch(
                p['goal_id'], (goal or {}).get('goal_text', ''), p.get('source', 'user'))
            self._handle_goal_proceed(goal_id=p['goal_id'], source=p.get('source', 'user'))
            return

        if t == 'reuse_goal':
            if not p.get('goal_id'):
                self._say_to_user("Please specify which goal to reuse, e.g. 'reuse goal_1'.")
                return
            # ── Graph: goal launch (reuse) ──
            goal = self._get_scheduled_goal(p['goal_id'])
            self._graph_emit_goal_launch(
                p['goal_id'], (goal or {}).get('goal_text', ''), p.get('source', 'user'))
            self._handle_goal_reuse(goal_id=p['goal_id'])
            return

        if t == 'chat_response':
            # Direct LLM chat — no planner, no goal machinery.
            # The planner is only invoked for explicit /commands or interpreted goals.
            source = p['source']
            user_text = p['text']
            logger.info(f'📥 {self.character_name} Direct chat response to {source}')
            with self.infospace_executor.turn_metrics.perf_phase("chat"):
                self._handle_chat_response(user_text, source, assessment=self._last_character_eval)
            self.execution_paused = False
            self._publish_execution_state()
            return

        if t == 'agent_message':
            # No graph action_result here — planner handles the actual response.
            self._create_character_note()
            with self.infospace_executor.turn_metrics.perf_phase("goal"):
                self._handle_agent_message(p['text'], p['source'], p.get('close_flag', False), action.assessment)
            return

        if t == 'proactive_remark':
            self._handle_proactive_remark(p['text'], p.get('source', ''), p.get('rationale', ''))
            return

        logger.warning(f"Act: unhandled action type '{t}'")

    def _handle_agent_message(self, text: str, source: str, close_flag: bool, assessment: Optional[Dict[str, Any]]):
        """Handle agent-to-agent message: record, dedup, envision, create dialog goal."""
        clean_input = text.strip().strip('"').strip("'")

        # Record incoming message
        self.conversation_store.record_incoming(source, clean_input, close=close_flag)

        # Dedup
        if self._last_agent_message.get(source) == clean_input:
            logger.info(f'📥 {self.character_name} Skipping duplicate message from {source}')
            return
        self._last_agent_message[source] = clean_input

        # Close flag: other agent ended conversation
        if close_flag:
            logger.info(f'📥 {self.character_name} Dialog closed by {source} (close flag)')
            self.conversation_store.close_dialog(source)
            self._dialog_cooldowns[source] = time.time()
            self._dialog_purposes.pop(source, None)
            return

        # Cooldown check
        cooldown_until = self._dialog_cooldowns.get(source, 0)
        if time.time() - cooldown_until < 10:
            logger.info(f'📥 {self.character_name} Suppressing re-open from {source} (cooldown)')
            return

        # Envision and create dialog goal
        purpose = self._dialog_purposes.get(source, '')
        envision = self._envision_conversation_turn(source, clean_input, purpose)
        goal_text = f"Continue dialog with {source}"
        context = (
            f"\n## CONTEXT ##\n"
            f"Their move: {envision['turn_intent']}\n"
            f"Your move: {envision['my_move']}\n"
            f"Message from {source}: {clean_input[:500]}"
        )
        logger.info(f'📥 {self.character_name} Processing agent message from {source}')
        self.execution_paused = False
        self._publish_execution_state()
        self.parse_and_set_goal("", f"{goal_text}{context}")

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
    
    def _get_planner_summary(self) -> str:
        """Return a one-line summary of current planner/goal state for OODA rendering."""
        try:
            if self.current_goal and self.current_goal.name != 'sleep':
                goal_label = self.current_goal.name[:60]
                if self.current_plan:
                    status = (self.current_plan.get('quality_status', 'in_progress')
                              if isinstance(self.current_plan, dict) else 'active')
                    return f"{status} — \"{goal_label}\""
                return f"active — \"{goal_label}\""
            return "idle"
        except Exception:
            return ""

    def _resolve_ooda_readable_note_id(self) -> str:
        """Resolve the Note ID of the readable OODA state snapshot, or '' if unavailable."""
        try:
            from ooda_living_state import READABLE_NOTE_NAME
            if self.resource_manager:
                rid = self.resource_manager.named_notes.get(READABLE_NOTE_NAME, '')
                return rid
        except Exception:
            pass
        return ''

    def _get_tom_content(self, entity: str) -> str:
        """Return persisted Theory of Mind content for an entity, or '' if unavailable."""
        try:
            tom_note_name = f"_tom_{entity.lower()}"
            if self.resource_manager:
                rid = self.resource_manager.named_notes.get(tom_note_name, '')
                if rid:
                    res = self.resource_manager.get_resource(rid)
                    if res:
                        return res.get('properties', {}).get('content', '')
        except Exception:
            pass
        return ''

    def _get_ooda_readable_content(self) -> str:
        """Return the rendered readable OODA state content, or '' if unavailable."""
        try:
            rid = self._resolve_ooda_readable_note_id()
            if rid and self.resource_manager:
                res = self.resource_manager.get_resource(rid)
                if res:
                    return res.get('properties', {}).get('content', '')
        except Exception:
            pass
        return ''

    def _build_agent_state_block(self) -> str:
        """Build the authoritative agent state block.

        This block is the single source of truth about what the agent is
        currently doing.  The character layer MUST defer to it rather than
        inferring task state from conversation history.
        """
        parts = ["## AGENT STATE (authoritative — your claims about what you are working on must be consistent with this)"]

        # Active goal
        if self.current_goal and self.current_goal.name != 'sleep':
            parts.append(f"Active goal: {self.current_goal.to_string()}")
        else:
            parts.append("Active goal: none")

        # Current plan (summary only — full plan is too large)
        if self.current_plan:
            plan_status = self.current_plan.get('quality_status', 'in_progress') if isinstance(self.current_plan, dict) else 'active'
            parts.append(f"Current plan: {plan_status}")
        else:
            parts.append("Current plan: none")

        # Last completed action
        if self.action_history:
            last_action = self.action_history[-1]
            result_str = self._truncate_result(last_action.result)
            parts.append(f"Last action: {last_action.action.get('type')}: {last_action.action.get('target')} — {result_str}")
        else:
            parts.append("Last action: none")

        # Scheduled goals — only upcoming/pending (completed goals are in Situation Awareness)
        try:
            scheduled = self._all_scheduled_goals()
            if scheduled:
                pending = [g for g in scheduled if g.get('status') in ('ready', 'pending', 'scheduled')]
                if pending:
                    lines = []
                    for g in pending[:5]:
                        name = g.get('name') or g.get('goal_text', '?')
                        if len(name) > 100:
                            name = name[:97] + '...'
                        lines.append(f"  - {name}")
                    parts.append("Upcoming goals:\n" + "\n".join(lines))
                else:
                    parts.append("Upcoming goals: none")
            else:
                parts.append("Upcoming goals: none")
        except Exception:
            parts.append("Upcoming goals: unavailable")

        # Recent sensor observations (inform disposition — rolling context)
        if self._sensor_inform_queue:
            lines = []
            for entry in self._sensor_inform_queue:
                sensor_name = entry.get('sensor_name', 'unknown')
                summary = entry.get('content', '')
                data = entry.get('data', [])
                if data:
                    # Structured: summary line + itemized data
                    lines.append(f"  - [{sensor_name}] {summary}")
                    for item in data:
                        title = item.get('title', '')
                        url = item.get('url', '')
                        desc = item.get('description', '')
                        label = f"{title} — {url}" if title else url
                        if desc:
                            label += f" ({desc[:120]})"
                        lines.append(f"      {label}")
                else:
                    # Legacy plain-text
                    lines.append(f"  - [{sensor_name}] {summary}")
            parts.append("Recent sensor observations:\n" + "\n".join(lines))

        return "\n".join(parts)

    def _build_process_block(self) -> str:
        """Build a process-oriented narrative of did/doing/will-do.

        Supplements the state block with temporal flow — connects the
        agent's recent past, current activity, and upcoming work into
        a compact narrative grounded in the concern→task hierarchy.
        """
        parts = ["## CURRENT PROCESS (what I did, am doing, and will do next)"]

        # Gather active operational tasks (used by all sections below)
        try:
            tasks = self._get_all_task_data()
            active_tasks = [t for t in tasks if t.get('status') == 'active'
                            and t.get('lifecycle') == 'operational']
        except Exception:
            active_tasks = []

        # ── WHY: trace current/recent work to its motivating concern ──
        why = ""
        try:
            # Find the most recently executed task
            if active_tasks:
                most_recent = max(active_tasks,
                                  key=lambda t: t.get('last_executed', '') or '')
                concern_id = most_recent.get('linked_concern_id', '')
                concern_label = ''
                if concern_id:
                    dc = getattr(self, '_derived_concern_model', None)
                    if dc:
                        for c in dc.get_concerns():
                            if c.get('concern_id') == concern_id:
                                concern_label = c.get('concern_label', concern_id)
                                break
                    if not concern_label:
                        concern_label = concern_id
                intention = most_recent.get('intention', '')[:100]
                if concern_label:
                    why = f"Why: {concern_label} (concern) → {intention} (task)"
                else:
                    why = f"Why: {intention} (task)"
        except Exception:
            pass
        if why:
            parts.append(why)

        # ── DID: most recent completed work ──
        did = ""
        try:
            if active_tasks:
                most_recent = max(active_tasks,
                                  key=lambda t: t.get('last_executed', '') or '')
                history = most_recent.get('execution_history', [])
                if history:
                    last_cycle = history[-1]
                    summary = last_cycle.get('summary', '')[:150]
                    goals_count = last_cycle.get('goals_count', 0)
                    achieved = last_cycle.get('goals_achieved', 0)
                    did = f"Did: {summary} ({achieved}/{goals_count} goals achieved)"
        except Exception:
            pass
        if not did:
            # Fallback to last action
            if self.action_history:
                last = self.action_history[-1]
                did = f"Did: {last.action.get('type', '?')}: {self._truncate_result(last.result)}"
        if did:
            parts.append(did)

        # ── DOING: current activity ──
        doing = ""
        if self.current_goal and self.current_goal.name != 'sleep':
            doing = f"Doing: Executing goal — {self.current_goal.to_string()[:120]}"
        elif self._operational_goal_waiting:
            doing = "Doing: Waiting for operational task goal to complete"
        elif self.active_task_wip:
            doing = f"Doing: Establishing task {self.active_task_wip}"
        else:
            doing = "Doing: Idle — monitoring OODA loop for events and sensor data"
        parts.append(doing)

        # ── WILL DO: next eligible tasks and upcoming work ──
        will_do = ""
        try:
            if active_tasks:
                now_ts = time.time()
                upcoming = []
                for t in active_tasks:
                    last = t.get('last_executed')
                    cooldown = t.get('cooldown_seconds', 3600)
                    intention = t.get('intention', '')[:80]
                    if last:
                        try:
                            last_ts = datetime.fromisoformat(
                                last.replace('+00:00', '')).timestamp()
                            remaining = max(0, cooldown - (now_ts - last_ts))
                            upcoming.append((remaining, intention))
                        except (ValueError, TypeError):
                            upcoming.append((0, intention))
                    else:
                        upcoming.append((0, intention))
                upcoming.sort(key=lambda x: x[0])
                if upcoming:
                    items = []
                    for remaining, intention in upcoming[:3]:
                        if remaining <= 0:
                            items.append(f"{intention} (eligible now)")
                        else:
                            mins = int(remaining / 60)
                            items.append(f"{intention} (in {mins}m)")
                    will_do = "Will do: " + "; ".join(items)
        except Exception:
            pass

        # Check for unserviced user concerns
        try:
            uc = self.user_concern_model.get_concerns(active_only=True) or []
            # Find concerns not linked to any active task
            serviced_ids = set()
            for t in active_tasks:
                cid = t.get('linked_concern_id', '')
                if cid:
                    serviced_ids.add(cid)
            unserviced = [c for c in uc
                          if c.get('concern_id') not in serviced_ids
                          and c.get('status') == 'open']
            if unserviced:
                labels = [c.get('concern_label', '?') for c in unserviced[:3]]
                will_do += f" | Unserviced user concerns: {', '.join(labels)}"
        except Exception:
            pass

        if will_do:
            parts.append(will_do)

        # ── Graph-derived cognitive context ──
        try:
            goal_text = ""
            if self.current_goal and self.current_goal.name != 'sleep':
                goal_text = self.current_goal.to_string()
            elif active_tasks:
                goal_text = active_tasks[0].get('intention', '')
            if goal_text:
                graph_ctx = self._cognitive_graph.assemble_context(
                    goal_text,
                    concern_activations=dict(self._character_concern_activations),
                    k=10, max_hops=1)
                if graph_ctx:
                    parts.append("\n## COGNITIVE CONTEXT (relevant history)")
                    parts.append(graph_ctx)
        except Exception as e:
            logger.debug(f"Graph context assembly failed: {e}")

        return "\n".join(parts)

    def _character_eval_enabled(self) -> bool:
        cfg = self.character_config.get("character_evaluator") or self.character_config.get("jill_evaluator") or {}
        return bool(cfg.get("enabled", True))

    def _character_eval_normalize_source(self, source: str, sensor_name: str = "") -> str:
        if source == "User":
            return "user"
        if source.startswith("sensor:"):
            return source
        if sensor_name and source in ("unknown", ""):
            return f"sensor:{sensor_name}"
        return source or "unknown"

    def _character_eval_build_goals_compact(self) -> List[Dict[str, Any]]:
        out: List[Dict[str, Any]] = []
        try:
            for g in self._all_scheduled_goals():
                out.append({
                    "goal_id": g.get("goal_id"),
                    "goal_label": g.get("name") or (g.get("goal_text") or "")[:120],
                    "goal_text": g.get("goal_text") or "",
                    "goal_status": g.get("status"),
                    "goal_summary": (g.get("last_result") or "")[:200],
                })
        except Exception:
            pass
        return out[:24]

    def _character_eval_build_recent_context(self, entity_for_dialog: str = "User") -> str:
        parts: List[str] = []
        try:
            entity_data = self.conversation_store.get_entity_context(entity_for_dialog, limit=20, scope="current")
            if entity_data and "conversation_history" in entity_data:
                for entry in entity_data["conversation_history"][-20:]:
                    if isinstance(entry, dict) and "source" in entry and "text" in entry:
                        parts.append(f"{entry['source']}: {str(entry['text'])[:200]}")
        except Exception:
            pass
        if self._sensor_inform_queue:
            for entry in self._sensor_inform_queue[-5:]:
                sn = entry.get("sensor_name", "?")
                parts.append(f"sensor[{sn}]: {str(entry.get('content', ''))[:120]}")
        return "\n".join(parts)

    def _character_eval_build_activity_state(self) -> str:
        try:
            return self._build_agent_state_block()[:2000]
        except Exception:
            return ""

    # _character_eval_flags_from_process_input DELETED — merged into _ooda_observe
    # _character_eval_run DELETED — inlined into _ooda_orient

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

        # Authoritative agent state — overrides conversation history
        system_prompt += f"\n{self._build_agent_state_block()}\n"
        # Process narrative — did/doing/will-do temporal flow
        system_prompt += f"\n{self._build_process_block()}\n"

        # Inline the readable OODA state snapshot
        ooda_content = self._get_ooda_readable_content()
        if ooda_content:
            system_prompt += f"\n{ooda_content}\n"

        # Inline Theory of Mind for User (if available)
        tom_content = self._get_tom_content("User")
        if tom_content:
            system_prompt += f"\n## Theory of Mind (your model of the User)\n{tom_content}\n"

        return system_prompt

    def _refresh_observations(self):
        """Refresh cached observations (system prompt + situation context). Used by _plan()."""
        try:
            # Use _update_system_prompt() to get fresh system prompt (includes ScienceWorld-populated setting)
            system_prompt = self._update_system_prompt()
            
            # Build dynamic user prompt from situation, entity context, and action history
            user_prompt = self.format_situation()
            
            # Theme-based conversation context from concern model
            concerns = self.user_concern_model.get_concerns() if self.user_concern_model else []
            themed_context = self.conversation_store.get_themed_context(concerns)
            if themed_context:
                user_prompt += '\n' + themed_context + '\n'

            self.observations = {'static': system_prompt, 'dynamic': user_prompt}
            return self.observations
        except Exception as e:
            logger.error(f'Error in _refresh_observations: {e}')
            traceback.print_exc()
            # Fallback: reuse existing observations if available, else minimal
            if not self.observations:
                self.observations = {'static': '', 'dynamic': self.format_situation()}
            return self.observations

    def _plan(self, template, goal: Goal):
        """Plan: Return existing plan or create single-action plan from goal."""
        # If we already have a plan, return it
        if self.current_plan is not None:
            logger.info(f'🎯 {self.character_name} continuing existing plan')
            return self.current_plan
        if not self.current_goal or self.current_goal.name == 'sleep':
            single_action = None
        else:
            # Reuse cached system prompt from _refresh_observations() rather
            # than rebuilding (avoids redundant _build_process_block / graph
            # context assembly).
            system_prompt = self.observations.get('static') or self._update_system_prompt()
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

            # Authoritative goal state — overrides any stale references in
            # conversation history below.
            recent_context += "\n# CURRENT GOAL STATE (authoritative — supersedes conversation history):\n"
            try:
                all_goals = list(self._all_scheduled_goals())
                active = [g for g in all_goals if g.get('status') in ('ready', 'running')]
                done = [g for g in all_goals if g.get('status') in ('completed', 'failed')]
                if active:
                    for g in active:
                        recent_context += f"  ACTIVE: {g.get('name', g.get('goal_text', '?'))} [{g.get('status')}]\n"
                else:
                    recent_context += "  No active goals.\n"
                if done:
                    for g in done[-3:]:
                        name = g.get('name', g.get('goal_text', '?'))
                        recent_context += f"  DONE: {name} [{g.get('status')}]\n"
            except Exception:
                recent_context += "  (goal state unavailable)\n"

            # Build set of done goal names for filtering stale history entries
            _done_goal_names = set()
            try:
                for g in self._all_scheduled_goals():
                    if g.get('status') in ('completed', 'failed'):
                        _done_goal_names.add(g.get('name', ''))
                        _done_goal_names.add(g.get('goal_text', ''))
                _done_goal_names.discard('')
            except Exception:
                pass

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

            # Add active user concerns for planner framing
            active_concerns = []
            try:
                active_concerns = self.user_concern_model.get_concerns(active_only=True) or []
                if active_concerns:
                    concern_lines = []
                    for c in active_concerns[:5]:
                        label = c.get("concern_label", "?")
                        desc = c.get("concern_description") or ""
                        weight = c.get("weight", "")
                        w_str = f" (weight={weight})" if weight else ""
                        concern_lines.append(f"  - {label}{w_str}: {desc}")
                    recent_context += "\n# User concerns (context for framing, not direct instructions):\n"
                    recent_context += "\n".join(concern_lines) + "\n"
            except Exception:
                pass

            # Add active derived concerns for planner framing
            try:
                dc = getattr(self, '_derived_concern_model', None)
                if dc:
                    active_dc = [c for c in dc.get_concerns()
                                 if c.get('status') in ('active', 'surfaced')]
                    if active_dc:
                        dc_lines = []
                        for c in active_dc[:5]:
                            label = c.get("concern_label", "?")
                            desc = c.get("concern_description") or ""
                            weight = c.get("weight", "")
                            origin = c.get("origin", "")
                            w_str = f" (weight={weight})" if weight else ""
                            dc_lines.append(f"  - {label}{w_str} [{origin}]: {desc}")
                        recent_context += "\n# Agent-derived concerns (my current operational priorities):\n"
                        recent_context += "\n".join(dc_lines) + "\n"
            except Exception:
                pass

            # Process narrative already included in system_prompt via
            # _update_system_prompt() → _build_process_block(). Not duplicated here.

            # Compute output size guidance from evaluator assessment + concerns
            output_guidance = None
            try:
                output_guidance = compute_output_guidance(
                    self._last_character_eval,
                    active_concerns,
                    goal_text,
                )
            except Exception:
                pass

            # Build context for planner (always needed for infospace)
            context = {
                'variables': self.infospace_executor.plan_bindings_flat if self.infospace_executor else {},
                'character_context': character_context,
                'situation_context': self._build_situation_context(),
                'recent_context': recent_context,
                'executor': self.infospace_executor,  # Pass executor for incremental planner
                'vision_goal': goal_text,
                'output_guidance': output_guidance,
                'orient_assessment': self._last_character_eval,
                'active_concerns': active_concerns,
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
            self.current_plan = result

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
            # Sensor reports: route by disposition into priority queues
            if source.startswith('sensor:') or (text_input and text_input.startswith('sensor ') and ' report\n' in text_input):
                # Skip heartbeats — they carry no content
                if isinstance(content_data, dict) and content_data.get('heartbeat'):
                    return

                disposition = content_data.get('disposition', 'inform') if isinstance(content_data, dict) else 'inform'
                sensor_name = source.replace('sensor:', '') if source.startswith('sensor:') else 'unknown'

                # Structured content (summary + data) vs legacy plain-text
                if isinstance(content_data, dict) and 'summary' in content_data:
                    sensor_content = content_data['summary']
                    sensor_data = content_data.get('data', [])
                else:
                    # Legacy: strip the "sensor X report [disp]\n" prefix
                    sensor_content = text_input
                    if '\n' in text_input:
                        sensor_content = text_input.split('\n', 1)[1]
                    sensor_data = []

                # Skip empty content — nothing useful to queue
                if not sensor_content and not sensor_data:
                    return

                sensor_entry = {'sensor_name': sensor_name, 'content': sensor_content, 'data': sensor_data, 'disposition': disposition}

                # Character evaluation happens in _ooda_orient when event is dequeued
                if disposition == 'alert':
                    self._sensor_alert_queue.append(sensor_entry)
                    logger.info(f'🚨 {self.character_name} sensor ALERT queued from {sensor_name}: {sensor_content[:80]}')
                elif disposition.startswith('trigger-task:'):
                    task_template = disposition.split(':', 1)[1]
                    sensor_entry['task_template'] = task_template
                    self._sensor_trigger_task_queue.append(sensor_entry)
                    logger.info(f'⚡ {self.character_name} sensor TRIGGER-TASK queued: {task_template} (from {sensor_name})')
                elif disposition.startswith('trigger:'):
                    goal_name = disposition.split(':', 1)[1]
                    sensor_entry['goal_name'] = goal_name
                    self._sensor_trigger_queue.append(sensor_entry)
                    logger.info(f'⚡ {self.character_name} sensor TRIGGER queued: {goal_name} (from {sensor_name})')
                else:
                    # 'inform' or unrecognized — rolling context (last 10)
                    self._sensor_inform_queue.append(sensor_entry)
                    if len(self._sensor_inform_queue) > 10:
                        self._sensor_inform_queue.pop(0)
                    logger.info(f'📡 {self.character_name} sensor inform from {sensor_name}: {sensor_content[:120]}')
                return

            # Process if we have text input
            if text_input and text_input.strip():
                self.text_input_queue.append(sense_data)
                input_preview = text_input[:60] + ('...' if len(text_input) > 60 else '')
                logger.info(f'📥 {self.character_name} Queued: "{input_preview}" (source: {source})')
                if len(self.text_input_queue) > 3:
                    logger.warning(f'⚠️ Text input queue size {len(self.text_input_queue)} > 3, dropping oldest')
                    self.text_input_queue.pop(0)
                
        except Exception as e:
            traceback.print_exc()
            logger.error(f'Error processing sense data: {e}')
    
    def handle_llm_toggle(self, sample):
        self._dispatch_command({'cmd': '/llm'})

    def _apply_pending_llm_switch(self):
        """Apply a pending LLM switch. Called at goal boundaries."""
        if self.llm_switch_pending is None:
            return
        new_mode = self.llm_switch_pending
        self.llm_switch_pending = None
        if new_mode == self.llm_mode:
            return
        alt_config = self.character_config.get('alt_llm_config', {})
        if new_mode == 'alt' and not alt_config:
            logger.warning(f'🔄 {self.character_name} LLM switch to alt requested but no alt_llm_config configured')
            return
        self.llm_mode = new_mode
        if hasattr(self, 'incremental_planner') and self.incremental_planner:
            self.incremental_planner.switch_llm(new_mode, alt_config)
        elif hasattr(self, 'infospace_executor') and self.infospace_executor:
            self.infospace_executor.switch_llm(new_mode)
        logger.info(f'🔄 {self.character_name} LLM switched to {new_mode}')
        self._publish_execution_state()
    
    def handle_continuous_toggle(self, sample):
        try:
            data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            data['cmd'] = '/continuous'
            self._dispatch_command(data)
        except Exception as e:
            logger.error(f'Error handling continuous toggle: {e}')
    
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
            
            PRESERVED_COLLECTIONS = {'conversation', 'conversation_history', '_tasks', '_scheduled_goals'}
            PRESERVED_NOTES = {'_situation', '_situation_prev', '_user_concerns', '_ooda_state', '_derived_concerns'}
            PRESERVED_NOTE_PREFIXES = ('_task_', '_scheduled_goal_')
            
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
            
            PRESERVED_COLLECTIONS = {'conversation', 'conversation_history', '_tasks', '_scheduled_goals'}
            PRESERVED_NOTES = {'_situation', '_situation_prev', '_user_concerns', '_ooda_state', '_derived_concerns'}
            PRESERVED_NOTE_PREFIXES = ('_task_', '_scheduled_goal_')
            
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
        self._dispatch_command({'cmd': '/stop'})

    def handle_interrupt_command(self, sample):
        # Interrupt is the same as stop — merged into /stop
        self._dispatch_command({'cmd': '/stop'})
    
    def _publish_execution_state(self):
        """Publish current execution state for UI."""
        try:
            active_dialog = False
            try:
                active_dialog = self.conversation_store.has_active_dialogs()
            except Exception:
                pass
            scheduler_state = self.goal_scheduler.get_status() if hasattr(self, 'goal_scheduler') else None
            if scheduler_state is not None:
                with self._scheduler_event_lock:
                    scheduler_state['events'] = list(self._scheduler_events[-20:])
            state_data = {
                'paused': self.execution_paused,
                'mode': self.execution_mode,
                'character': self.character_name,
                'continuous_mode': self.continuous_mode,
                'llm_mode': self.llm_mode,
                'llm_switch_pending': self.llm_switch_pending,
                'active_dialog': active_dialog,
                'goal_scheduler': scheduler_state,
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
    
    def _handle_resource_update_query(self, query):
        """Handle query to update Note content (for resource_browser)."""
        try:
            if not self.resource_manager:
                response = {
                    'success': False,
                    'error': 'Resource update only available for infospace characters'
                }
                query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
                return

            # Extract resource_id from query key (cognitive/{character}/resource/update/{resource_id})
            key_parts = str(query.key_expr).split('/')
            if len(key_parts) < 5:
                response = {
                    'success': False,
                    'error': 'Invalid resource update query format'
                }
                query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
                return

            resource_id = key_parts[-1]

            # Parse payload for new content
            payload_bytes = query.payload.to_bytes() if query.payload else b'{}'
            params = json.loads(payload_bytes.decode('utf-8')) if payload_bytes else {}
            new_content = params.get('content', '')

            success, error_msg = self.resource_manager.update_note_content(resource_id, new_content)

            if success:
                response = {
                    'success': True,
                    'message': f'Note {resource_id} updated'
                }
            else:
                response = {
                    'success': False,
                    'error': error_msg or f'Failed to update {resource_id}'
                }
            query.reply(query.key_expr, json.dumps(response).encode('utf-8'))
        except Exception as e:
            logger.error(f'Error handling resource update query: {e}')
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
        PRESERVED_COLLECTIONS = {'conversation', 'conversation_history', '_tasks', '_scheduled_goals'}
        PRESERVED_NOTES = {'_situation', '_situation_prev', '_user_concerns', '_ooda_state', '_derived_concerns'}
        PRESERVED_NOTE_PREFIXES = ('_task_', '_scheduled_goal_')
        
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
    
    def _scheduled_goals_query_handler(self, query):
        """Handle query for scheduled goals (includes active operational task goal)."""
        try:
            goals = self._all_scheduled_goals()
            # Include the active operational task goal so it's visible in /goals
            if self._operational_task_note and self._operational_goal_waiting:
                op_goal_id = f"op_{self._operational_task_note}"
                # Don't duplicate if somehow already in the list
                if not any(g.get('goal_id') == op_goal_id for g in goals):
                    op_goal = {
                        "goal_id": op_goal_id,
                        "name": f"task: {self._operational_task_note}",
                        "goal_text": getattr(self, '_operational_task_note', ''),
                        "status": "running",
                        "is_running": True,
                        "mode": "task",
                        "source": "task",
                    }
                    goals.append(op_goal)
            response = {"success": True, "goals": goals}
            query.reply(query.key_expr, json.dumps(response).encode("utf-8"))
        except Exception as e:
            logger.error(f"Error in scheduled goals query handler: {e}")
            response = {"success": False, "error": str(e)}
            query.reply(query.key_expr, json.dumps(response).encode("utf-8"))
    
    def _task_wips_query_handler(self, query):
        """Handle query for task WIP notes."""
        try:
            tasks = []
            if self.resource_manager:
                for name, note_id in list(self.resource_manager.named_notes.items()):
                    if not name.startswith("_task_wip_"):
                        continue
                    note_data = self.resource_manager.resource_registry.get(note_id)
                    if not note_data:
                        continue
                    content = note_data.get("properties", {}).get("content", "")
                    if not content:
                        continue
                    try:
                        wip = json.loads(content) if isinstance(content, str) else {}
                    except (json.JSONDecodeError, TypeError):
                        continue
                    wip["_note_name"] = name
                    wip["_note_id"] = note_id
                    wip["_is_active"] = (name == self.active_task_wip)
                    # Find associated scheduled goal (the final recurring one)
                    for goal in self._all_scheduled_goals():
                        if goal.get("task_context_note") == name:
                            wip["_scheduled_goal_id"] = goal.get("goal_id", "")
                            wip["_scheduled_goal_name"] = goal.get("name", "")
                            break
                    tasks.append(wip)
            tasks.sort(key=lambda t: t.get("created", ""), reverse=True)
            response = {"success": True, "tasks": tasks}
            query.reply(query.key_expr, json.dumps(response).encode("utf-8"))
        except Exception as e:
            logger.error(f"Error in task_wips query handler: {e}")
            response = {"success": False, "error": str(e)}
            query.reply(query.key_expr, json.dumps(response).encode("utf-8"))

    def _concerns_query_handler(self, query):
        """Handle query for all concerns (user + derived) with activation levels."""
        try:
            # User concerns
            user_concerns = []
            try:
                user_concerns = self.user_concern_model.get_concerns(active_only=False) or []
            except Exception:
                pass

            # Derived concerns
            derived_concerns = []
            try:
                derived_concerns = self._derived_concern_model.get_concerns() or []
            except Exception:
                pass

            # Activation levels from living state
            activations = {}
            for ca in self._ooda_living_state.concern_activations:
                activations[ca.get('id', '')] = {
                    'activation': ca.get('activation', 0.0),
                    'trend': ca.get('trend', 'stable'),
                }

            response = {
                'success': True,
                'user_concerns': user_concerns,
                'derived_concerns': derived_concerns,
                'activations': activations,
            }
            query.reply(query.key_expr, json.dumps(response, default=str).encode('utf-8'))
        except Exception as e:
            logger.error(f'Error in concerns query handler: {e}')
            query.reply(query.key_expr, json.dumps({'success': False, 'error': str(e)}).encode('utf-8'))

    def _triage_status_query_handler(self, query):
        """Handle query for triage system status including autonomy budget."""
        try:
            status = self._concern_triage.get_status()
            # Include autonomy budget from goal scheduler
            if hasattr(self, 'goal_scheduler'):
                sched = self.goal_scheduler.get_status()
                status['budget_remaining_seconds'] = sched.get('budget_remaining_seconds', 0)
                status['budget_total_seconds'] = sched.get('budget_total_seconds', 0)
                status['budget_window_seconds'] = sched.get('budget_window_seconds', 0)
            # Include task milestone activity log (suppressed say actions)
            status['task_activity_log'] = getattr(self, '_task_wip_activity_log', [])
            response = {'success': True, **status}
            query.reply(query.key_expr, json.dumps(response, default=str).encode('utf-8'))
        except Exception as e:
            logger.error(f'Error in triage_status query handler: {e}')
            query.reply(query.key_expr, json.dumps({'success': False, 'error': str(e)}).encode('utf-8'))

    def _ooda_feed_query_handler(self, query):
        """Handle query for recent OODA events."""
        try:
            response = {'success': True, 'events': list(self._ooda_event_feed)}
            query.reply(query.key_expr, json.dumps(response, default=str).encode('utf-8'))
        except Exception as e:
            query.reply(query.key_expr, json.dumps({'success': False, 'error': str(e)}).encode('utf-8'))

    def _handle_task_approve(self, sample):
        self._zenoh_to_command(sample, '/task approve')

    def _handle_task_abandon(self, sample):
        self._zenoh_to_command(sample, '/task abandon')

    def _handle_task_edit(self, sample):
        self._zenoh_to_command(sample, '/task edit')

    def _handle_task_cooldown(self, sample):
        self._zenoh_to_command(sample, '/task cooldown')

    def _handle_task_run_now(self, sample):
        self._zenoh_to_command(sample, '/task run')

    def _handle_concern_manage(self, sample):
        """Zenoh shim for concern management from Task Manager UI."""
        try:
            data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            action = data.get('action', '')
            # Map task manager action names to /concern commands
            action_map = {
                'close': 'close', 'reopen': 'reopen', 'resolve': 'resolve',
                'satisfy': 'resolve', 'abandon': 'abandon', 'delete': 'delete',
                'set_weight': 'weight', 'set_revisit': 'revisit',
                'activate_concern': 'activate',
            }
            cmd_action = action_map.get(action, action)
            data['cmd'] = f'/concern {cmd_action}'
            self._dispatch_command(data)
        except Exception as e:
            logger.warning(f'Concern manage error: {e}')

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
        # Reset per-run instrumentation at the planning convergence point so
        # every entry path (proceed, /goal add, sense-data, sensor trigger,
        # chat handler, etc.) gets the same fresh state — no stale step_results
        # or last_run_mode leaking from a prior goal's run.
        self._init_run_instrumentation("replan")

        # Reset per-turn metrics for this goal (may run on goal thread)
        if self.infospace_executor:
            self.infospace_executor.turn_metrics = TurnMetrics()

        # Defer note indexing during goal execution; batch-index survivors after cleanup
        if self.resource_manager:
            self.resource_manager.indexing_deferred = True

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

            # Refresh filesystem catalog so the planner sees current files
            if self.infospace_executor:
                self.infospace_executor._run_init_tool()

            if not self.observations:
                self._refresh_observations()
            self.current_goal = Goal(parsed_goal, [self.character_name], description='', termination='')

            # If continuous mode is active, update goal text for resubmission
            if self.continuous_mode:
                self.continuous_goal_text = self.current_goal.to_string()
                self.last_completed_goal_text = self.current_goal.to_string()  # Also update last completed
                logger.info(f'🔄 {self.character_name} continuous mode goal updated: {self.continuous_goal_text[:80]}...')

            # Skip goal rewriting and plan immediately (infospace mode)
            self._publish_goal(self.current_goal)
            logger.info(f'🧩 {self.character_name} infospace planning for goal: {parsed_goal}')
            with self.infospace_executor.turn_metrics.perf_phase("goal"):
                result = self._plan(template, self.current_goal)
            # Publish complete goal result for external consumers (e.g., eval scripts)
            self._publish_goal_result(result)

            self.current_plan = result
            if result.get('success', False):
                self._publish_current_plan()
                self.plan_just_generated = True
            else:
                if result.get('response') == "Interrupted by user.":
                    logger.info(f"Planning interrupted for {self.character_name}; clearing current plan for clean terminal ask turn.")
                    self.current_plan = None
                elif result.get('response'):
                    # Non-success with response (partial, quality gate fail, etc.)
                    # Still publish so _publish_plan_result can show the result
                    self._publish_current_plan()
                    logger.warning(
                        f"Plan returned non-success with response (quality_status={result.get('quality_status', 'unknown')}, "
                        f"verification_answer={result.get('verification_answer', '')}); not treating as internal error."
                    )
                else:
                    logger.error(f"Error in _plan: {result.get('error', 'Unknown error')}")
            return result

        except Exception as e:
            logger.error(f"Goal parsing failed for {self.character_name}: {e}")
            traceback.print_exc()
            return {"success": False, "error": str(e)}
        finally:
            # Ensure indexing is re-enabled even if goal failed or was interrupted
            if self.resource_manager and self.resource_manager.indexing_deferred:
                self.resource_manager.flush_deferred_indexes()
            # Emit per-turn latency summary for goal turns
            if self.infospace_executor and self.infospace_executor.turn_metrics.llm_calls:
                _perf = self.infospace_executor.turn_metrics.summary()
                logger.info(_perf)
                print(_perf, flush=True)

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
                # logger.info(f'👥 Retrieved entity context for {entity_name}')
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
            if hasattr(self, 'goal_scheduler'):
                self.goal_scheduler.stop()

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
            
            # Save world model
            if hasattr(self, 'world_model') and self.world_model:
                try:
                    self.world_model.save()
                    logger.info(f'🌍 Saved world_model for {self.character_name}')
                except Exception as e:
                    logger.error(f'Error saving world_model during shutdown: {e}')

            # Save cognitive graph
            try:
                if self._cognitive_graph_path:
                    self._cognitive_graph.consolidate()
                    self._cognitive_graph.save(self._cognitive_graph_path, force=True)
                    logger.info(f'🧠 Saved cognitive graph for {self.character_name}')
            except Exception as e:
                logger.error(f'Error saving cognitive graph during shutdown: {e}')

            # Save entity index
            try:
                self._save_entity_index()
                logger.info(f'🏷 Saved entity index for {self.character_name}')
            except Exception as e:
                logger.debug(f'Entity index save during shutdown failed: {e}')

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
