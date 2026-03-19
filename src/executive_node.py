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

SCHEDULED_GOAL_NOTE_PREFIX = "_scheduled_goal_"
SCHEDULED_GOALS_COLLECTION = "_scheduled_goals"


def _is_goal_cmd(s):
    return s and s.strip().lower().startswith('goal:')

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
            console_handler.setLevel(logging.INFO)
            logger.info(f'🔧 Debug mode enabled for {self.character_name}')
        
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
                    return self.infospace_executor._sglang_generate(messages, max_tokens, temperature, stops, is_json)
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
        self.conversation_store._archive_callback = self._archive_dialog

        from user_concern_model import UserConcernModel
        self.user_concern_model = UserConcernModel(
            resource_manager=self.resource_manager,
            character_name=self.character_name,
            llm_generate=llm_generate,
            infospace_executor=self.infospace_executor,
        )
        self.user_concern_model.load()

        self._scheduled_goal_counter = 0
        self._active_scheduled_goal_id = None
        self._initialize_scheduled_goals()

        # Initialize task scheduler (auto-proceed timer)
        from goal_scheduler import GoalScheduler
        sched_cfg = self.character_config.get('goal_scheduler', self.character_config.get('task_scheduler', {}))
        interval_min = sched_cfg.get('interval', 15)  # minutes (consistent with UI)
        self.goal_scheduler = GoalScheduler(
            character_name=self.character_name,
            interval=float(interval_min) * 60.0,  # convert to seconds
            enabled=sched_cfg.get('enabled', False),
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

        # Goal worker thread state
        self._goal_thread: Optional[threading.Thread] = None
        self._goal_thread_result: Optional[Dict[str, Any]] = None
        self._goal_done_event = threading.Event()

        # Sensor priority queues (observed while goal runs, drained on completion)
        self._sensor_alert_queue: list = []    # disposition='alert' — high priority
        self._sensor_trigger_queue: list = []  # disposition='trigger:X' — goal dispatch
        self._sensor_inform_queue: list = []   # disposition='inform' — rolling context (last 10)
        
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
        """Handle save_all command - save resource_manager state."""
        try:
            logger.info(f'💾 {self.character_name} received save_all command')
            if self.resource_manager:
                self.resource_manager.save_to_file()
                logger.info(f'💾 Saved resource manager state for {self.map_name}')
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
    
    def run(self):
        """Main OODA loop."""
        try:
            logger.info('Executive Node running - press Ctrl+C to stop')
            
            # Announce character presence
            self._announce_character()
            time.sleep(0.1)
            time.sleep(1.0)
            
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

        Responsibilities:
        - Route user text: ask-reply → goal thread, goal: prefix → goal thread, else → chat-mode
        - Route goal commands (proceed/reuse/terminate) → goal dispatch
        - Handle goal thread completion → drain sensor queues
        - Chat-mode: lightweight LLM response while goal thread is busy or idle
        """
        # ── 1. Check goal thread completion ──────────────────────────────
        if self._goal_done_event.is_set() and not self._is_goal_running():
            self._goal_done_event.clear()
            logger.info(f'✅ {self.character_name} goal thread completed')
            # Return to idle state so UI shows step/run buttons, not interrupt
            self.execution_paused = True
            self.execution_mode = 'step'
            # Scheduler notification handled by _set_scheduled_goal_result in worker
            self._publish_execution_state()
            # Drain sensor queues in priority order
            self._drain_sensor_queues()

        # ── 1b. Advance task milestone loop if idle ───────────────────────
        if self.active_task_wip and not self.active_task_wip_waiting and not self._is_goal_running():
            self._advance_task_wip()

        # ── 2. Process text_input_queue ──────────────────────────────────
        if self.text_input_queue:
            # Peek at first item to classify it
            sense_data = self.text_input_queue[0]
            content = sense_data.get('content', '')
            try:
                content_data = json.loads(content)
                text = content_data.get('text', '')
                source = content_data.get('source', 'unknown')
            except (json.JSONDecodeError, TypeError):
                text = content
                source = 'console'

            # 2a. Ask-reply: push response to blocking queue so planner thread unblocks
            if self.awaiting_ask_response:
                if source == 'User':
                    self.text_input_queue.pop(0)
                    logger.info(f'🚀 User reply to ask received, pushing to _ask_response_queue')
                    self._ask_response_queue.put(text)
                    # awaiting_ask_response is cleared by _execute_ask when it unblocks
                return  # While awaiting ask, ignore non-User input

            # 2b. Goal commands and explicit goals — dispatch to goal thread
            clean = text.strip().strip('"').strip("'")
            is_goal = _is_goal_cmd(text)
            is_proceed = clean.lower().startswith('proceed') or clean.lower() == 'next step'
            is_reuse = clean.lower().startswith('reuse')
            is_terminate = clean.lower().startswith('terminate')
            is_clear_cache = clean.lower().startswith('clear-cache')
            is_unblock = clean.lower().startswith('unblock')
            is_task_cmd = clean.startswith('task:') or clean.lower() in ('tasks', 'task list', 'list tasks')
            is_command = is_goal or is_proceed or is_reuse or is_terminate or is_clear_cache or is_unblock or is_task_cmd
            is_scheduler = source == 'scheduler'

            if is_command or is_scheduler:
                self.text_input_queue.pop(0)
                self._process_text_input_item(sense_data)
                return

            # 2c. Regular user text or agent message → chat-mode response
            if source == 'User':
                self.text_input_queue.pop(0)
                # Clear stale interrupt from prior End/Stop so chat LLM call succeeds
                self.interrupt_requested = False
                if self.infospace_executor:
                    self.infospace_executor.interrupt_requested = False
                self._handle_chat_response(text, source)
                self._publish_execution_state()
                return

            # 2d. Agent-to-agent messages — process through existing pipeline
            if source not in ('unknown', 'console'):
                self.text_input_queue.pop(0)
                self._process_text_input_item(sense_data)
                return

            # 2e. Unknown source — drop
            self.text_input_queue.pop(0)

        # ── 3. Run OODA observe (keeps observations fresh) ───────────────
        if not self._is_goal_running():
            self._run_ooda_loop()

    def _process_text_input_item(self, sense_data: dict):
        """Process a single text input item through the existing dispatch pipeline.

        Wraps the original _process_text_input logic but accepts the item directly
        instead of popping from queue (caller already popped it).
        """
        # Re-inject into queue head so _process_text_input can pop it
        self.text_input_queue.insert(0, sense_data)
        self._process_text_input()

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
        # Three cases:
        #   A) In-plan say already addressed user → skip (already visible)
        #   B) No in-plan say, but primary_product exists → show product reference
        #   C) No in-plan say, no product → show FINAL_ANSWER text
        primary_product = plan_result.get('primary_product', '')
        if last_say:
            # Case A: user was already addressed during plan execution
            if primary_product and primary_product != last_say:
                logger.debug(f'In-plan say covered response; primary product: {primary_product}')
        elif not interrupted_final and not is_scheduled_goal:
            if primary_product:
                # Case B: show the primary product as a clickable reference
                display_text = final_thoughts_clean or "Done."
                display_text = f"{display_text}\n→ {primary_product}"
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
                logger.info(f'📤 Published result with primary product: {primary_product}')
            elif final_thoughts_clean and len(final_thoughts_clean) >= 20 and final_thoughts_clean.replace(' ', '').replace('.', '').replace(',', '').replace('!', '').replace('?', '').strip():
                # Case C: no product, show FINAL_ANSWER text
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
        """Build situation context from persisted situation note."""
        if self.situation_context:
            return self.situation_context
        return ""

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
                    name = g.get('name') or g.get('goal_text', '?')[:60]
                    product = g.get('primary_product', '')
                    g_status = g.get('status', '?')
                    line = f"  {name} [{g_status}]"
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
                prompt = (
                    "You maintain a short list of cross-goal learnings for a planning agent. "
                    "Each entry is a one-line fact useful across future goals "
                    "(e.g., user preferences, working data sources, useful resource names, "
                    "things that failed and shouldn't be retried the same way).\n\n"
                    "Do NOT include: goal status, project narratives, what to do next, "
                    "or anything already tracked by world_model/tool_model.\n"
                    "Keep max 10 entries. Drop stale or redundant ones.\n"
                    "Output ONLY the bullet list (- item), or 'none' if nothing to retain.\n\n"
                    f"## EXISTING LEARNINGS\n{existing_learnings or '(none yet)'}\n\n"
                    f"## JUST COMPLETED\nGoal: {goal_text}\nOutcome: {status}\n"
                    f"Summary: {summary}\n\n"
                    "Write the updated learnings list."
                )
                response = self.infospace_executor.llm_generate(
                    prompt, max_tokens=400, temperature=0.2
                )
                if response.success and response.text and response.text.strip().lower() != 'none':
                    new_learnings = response.text.strip()

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
        except Exception as e:
            logger.warning(f'Error updating situation note: {e}')

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

            prompt = (
                "You maintain a short list of cross-goal learnings for a planning agent. "
                "A session is ending. Prune entries that are stale, redundant, or too specific "
                "to a single past goal. Keep only durable, reusable facts.\n"
                "Output ONLY the bullet list (- item), or 'none' if nothing worth keeping.\n\n"
                f"## CURRENT LEARNINGS\n{learnings}\n\n"
                f"Session: {self.plan_counter} goal(s) attempted.\n"
                "Write the pruned learnings list."
            )
            response = self.infospace_executor.llm_generate(
                prompt, max_tokens=400, temperature=0.2
            )
            if response.success and response.text:
                pruned = response.text.strip()
                if pruned.lower() == 'none':
                    pruned = ""
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
        }.items():
            if key not in goal:
                goal[key] = default
                changed = True
        if not isinstance(goal.get("cached_plan_actions"), list):
            goal["cached_plan_actions"] = []
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
        # If the goal was already terminated (interrupted) by the user, don't overwrite
        if goal.get("status") == "interrupted":
            logger.info(f'Goal {goal_id} already interrupted — skipping result update')
            return
        success = bool(result and result.get("success"))
        primary_product = (result.get("primary_product") if isinstance(result, dict) else "") or ""
        last_result_raw = (result.get("response") if isinstance(result, dict) else "") or (result.get("error") if isinstance(result, dict) else "") or ""

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
                        last_result_raw = content[:1500]
                        logger.info(f'Enriched last_result from primary_product {primary_product} ({len(content)} chars)')
            except Exception as e:
                logger.debug(f'Could not load primary_product for last_result enrichment: {e}')

        updates: Dict[str, Any] = {
            "is_running": False,
            "status": "completed" if success else "failed",
            "last_result": last_result_raw,
            "primary_product": primary_product,
        }
        if not used_cache and success:
            plan_actions = result.get("plan") if isinstance(result, dict) else None
            if isinstance(plan_actions, list):
                updates["cached_plan_actions"] = plan_actions
        self._update_scheduled_goal(goal_id, **updates)
        if goal_id in self._scheduler_started_goals:
            self._scheduler_started_goals.discard(goal_id)
        self._record_scheduler_event(
            "end",
            goal_id=goal_id,
            goal_name=goal.get("name", "") or goal.get("goal_text", "")[:80],
            status=updates["status"],
            result=updates["last_result"],
        )
        # Update user concern model from goal completion
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
            self.user_concern_model.update_from_goal_completion(
                goal_statement=full_goal_text,
                outcome_summary=outcome,
                goal_id=goal_id,
                success=bool(success),
                result_artifact=artifact_content,
            )
        except Exception as e:
            logger.warning(f'Error updating concern model from goal {goal_id}: {e}')
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

                    milestone_record = {
                        "goal_text": goal.get("goal_text", ""),
                        "result_summary": result_summary,
                        "status": "completed" if success else "failed",
                        "timestamp": datetime.now().isoformat(),
                    }
                    if wip:
                        wip.setdefault("milestones_completed", []).append(milestone_record)
                        wip["current_milestone"] = None
                        wip["updated"] = datetime.now().isoformat()
                        # Capture primary_product content in accumulated_findings
                        if primary_product and result_summary and len(result_summary) > 20:
                            wip.setdefault("accumulated_findings", []).append(
                                f"Milestone result: {result_summary}"
                            )
                        elif primary_product:
                            wip.setdefault("accumulated_findings", []).append(
                                f"Milestone produced artifact: {primary_product}"
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

        # Clean up transient resources created during the goal
        if pre_resource_ids is not None and self.resource_manager:
            now_ids = set(self.resource_manager.resource_registry.keys())
            created_ids = now_ids - pre_resource_ids
            keep_ids = {primary_product} if primary_product else set()
            self._cleanup_transient_resources(created_ids, keep_ids, label=goal_id)
        self._publish_execution_state()

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
        # Close any active User conversation when starting a goal
        if source != "scheduler" and source != "sensor_trigger":
            self.conversation_store.close_dialog("User")
        # Apply any pending LLM switch before starting a new goal
        self._apply_pending_llm_switch()
        self._active_scheduled_goal_id = goal_id
        self._update_scheduled_goal(goal_id, is_running=True, status="running")
        self._record_scheduler_event(
            "start",
            goal_id=goal_id,
            goal_name=goal.get("name", "") or goal.get("goal_text", "")[:80],
        )
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
            result: Dict[str, Any] = {}
            try:
                result = self.parse_and_set_goal("", effective_goal_text) or {}
            except Exception as e:
                result = {"success": False, "error": str(e)}
            self._set_scheduled_goal_result(goal_id, result, used_cache=False, pre_resource_ids=pre_resource_ids)
            if notify_user:
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
        self._record_scheduler_event(
            "start",
            goal_id=goal_id,
            goal_name=goal.get("name", "") or goal.get("goal_text", "")[:80],
        )
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
                result = {
                    "success": success,
                    "plan": cached,
                    "response": f"Cached plan replay {'succeeded' if success else 'failed'}: {sync_result.get('reason', '')}".strip(),
                    "quality_status": "passed" if success else "failed",
                }
            except Exception as e:
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
        goal = self._get_scheduled_goal(goal_id)
        if not goal:
            self._say_to_user(f"Goal '{goal_id}' not found.")
            return
        if goal.get("is_running") or self._active_scheduled_goal_id == goal_id:
            self.interrupt_requested = True
            if self.infospace_executor:
                self.infospace_executor.interrupt_requested = True
            self._update_scheduled_goal(goal_id, status="interrupted", is_running=False)
            if self._active_scheduled_goal_id == goal_id:
                self._active_scheduled_goal_id = None
            # If this goal is a task milestone, abort the task WIP
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
            if goal_id in self._scheduler_started_goals:
                self._scheduler_started_goals.discard(goal_id)
                self._record_scheduler_event(
                    "end",
                    goal_id=goal_id,
                    goal_name=goal.get("name", "") or goal.get("goal_text", "")[:80],
                    status="interrupted",
                )
            if hasattr(self, "goal_scheduler"):
                self.goal_scheduler.notify_goal_terminal(goal_id)
            self._publish_execution_state()
            self._say_to_user(f"Goal '{goal.get('name') or goal_id}' interrupted.")
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
        """Enqueue a synthetic goal proceed/reuse command (called from scheduler thread)."""
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
        # Respect execution_mode: replay uses cached plan, replan (default) replans from scratch
        exec_mode = goal.get("execution_mode", "replan") if goal else "replan"
        has_cache = bool(goal and isinstance(goal.get("cached_plan_actions"), list) and goal["cached_plan_actions"])
        verb = "reuse" if exec_mode == "replay" and has_cache else "proceed"
        command = json.dumps({"text": f"{verb} {goal_id}", "source": "scheduler"})
        self.text_input_queue.append({"content": command})
        self.execution_paused = False
        self._publish_execution_state()

    def _handle_scheduler_control(self, sample):
        """Zenoh callback for scheduler enable/disable/interval changes."""
        try:
            data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            if 'enable' in data:
                self.goal_scheduler.set_enabled(bool(data['enable']))
            if 'interval' in data:
                self.goal_scheduler.set_interval(float(data['interval']))
            self._publish_execution_state()
        except Exception as e:
            logger.error(f'Error in scheduler control handler: {e}')

    def _handle_goal_schedule_mode(self, sample):
        """Zenoh callback for per-goal schedule mode changes."""
        try:
            data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            goal_id = data.get("goal_id")
            mode = data.get("schedule_mode")
            if not goal_id or mode not in ("manual", "auto", "recurring", "daily"):
                logger.warning(f"Invalid goal_schedule_mode payload: {data}")
                return
            updates = {"schedule_mode": mode}
            if mode == "daily":
                updates["run_at"] = data.get("run_at", "")
            self._update_scheduled_goal(goal_id, **updates)
            logger.info(f"Goal {goal_id} schedule_mode set to '{mode}'")
        except Exception as e:
            logger.error(f"Error in goal_schedule_mode handler: {e}")

    def _handle_goal_rename(self, sample):
        """Zenoh callback for scheduled goal rename."""
        try:
            data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            goal_id = data.get("goal_id")
            name = (data.get("name") or "").strip()
            if not goal_id or not name:
                logger.warning(f"Invalid goal_rename payload: {data}")
                return
            self._update_scheduled_goal(goal_id, name=name[:120], name_customized=True)
            logger.info(f"Goal {goal_id} renamed to '{name[:120]}'")
        except Exception as e:
            logger.error(f"Error in goal_rename handler: {e}")

    def _handle_goal_execution_mode(self, sample):
        """Zenoh callback for per-goal execution mode changes (replan/replay)."""
        try:
            data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            goal_id = data.get("goal_id")
            mode = data.get("execution_mode")
            if not goal_id or mode not in ("replan", "replay"):
                logger.warning(f"Invalid goal_execution_mode payload: {data}")
                return
            self._update_scheduled_goal(goal_id, execution_mode=mode)
            logger.info(f"Goal {goal_id} execution_mode set to '{mode}'")
        except Exception as e:
            logger.error(f"Error in goal_execution_mode handler: {e}")

    def _handle_goal_text_update(self, sample):
        """Zenoh callback for updating a scheduled goal's goal_text."""
        try:
            data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            goal_id = data.get("goal_id")
            goal_text = (data.get("goal_text") or "").strip()
            if not goal_id or not goal_text:
                logger.warning(f"Invalid goal_text_update payload: {data}")
                return
            # Only clear cache if text actually changed
            goal = self._get_scheduled_goal(goal_id)
            old_text = (goal.get("goal_text", "") if goal else "").strip()
            updates = {"goal_text": goal_text}
            if goal_text != old_text:
                updates["cached_plan_actions"] = []
                updates["execution_mode"] = "replan"
                updates["status"] = "ready"
                logger.info(f"Goal {goal_id} text changed, cache cleared")
            self._update_scheduled_goal(goal_id, **updates)
            logger.info(f"Goal {goal_id} text updated ({len(goal_text)} chars)")
        except Exception as e:
            logger.error(f"Error in goal_text_update handler: {e}")

    def _handle_goal_cache(self, sample):
        """Zenoh callback for scheduled goal cache operations."""
        try:
            data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            goal_id = data.get("goal_id")
            action = data.get("action")
            if action != "clear" or not goal_id:
                logger.warning(f"Invalid goal_cache payload: {data}")
                return
            self._update_scheduled_goal(goal_id, cached_plan_actions=[], execution_mode="replan", status="ready")
            logger.info(f"Cleared cached plan_actions for {goal_id}")
        except Exception as e:
            logger.error(f"Error in goal_cache handler: {e}")

    def _handle_goal_interrupt(self, sample):
        """Zenoh callback to interrupt a scheduled goal."""
        try:
            data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            goal_id = data.get("goal_id")
            if not goal_id:
                logger.warning(f"Invalid goal_interrupt payload: {data}")
                return
            self._handle_goal_terminate(goal_id=goal_id)
        except Exception as e:
            logger.error(f"Error in goal_interrupt handler: {e}")

    def _handle_goal_remove(self, sample):
        """Zenoh callback to force-remove a scheduled goal."""
        try:
            data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            goal_id = data.get("goal_id")
            if not goal_id:
                logger.warning(f"Invalid goal_remove payload: {data}")
                return
            goal = self._get_scheduled_goal(goal_id)
            if not goal:
                logger.warning(f"Goal {goal_id} not found for remove")
                return
            if goal.get("is_running") or self._active_scheduled_goal_id == goal_id:
                self.interrupt_requested = True
                if self.infospace_executor:
                    self.infospace_executor.interrupt_requested = True
            # If this goal is a task milestone, abort the task WIP
            if goal.get("task_wip_id") and self.active_task_wip:
                logger.info(f'📋 Task WIP {goal.get("task_wip_id")}: milestone removed — aborting task')
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
                logger.warning(f"Goal {goal_id} could not be removed")
                return
            if self._active_scheduled_goal_id == goal_id:
                self._active_scheduled_goal_id = None
            if goal_id in self._scheduler_started_goals:
                self._scheduler_started_goals.discard(goal_id)
            if hasattr(self, "goal_scheduler"):
                self.goal_scheduler.notify_goal_terminal(goal_id)
            self._publish_execution_state()
            logger.info(f"Goal {goal_id} removed via control endpoint")
        except Exception as e:
            logger.error(f"Error in goal_remove handler: {e}")

    def _handle_task_wip_delete(self, sample):
        """Handle request to delete a task WIP and all its associated artifacts."""
        try:
            data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            note_name = data.get("note_name")
            if not note_name or not self.resource_manager:
                return

            # Read WIP content for cross-references
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

            # Clear active task WIP if this is the active one
            if self.active_task_wip == note_name:
                self.interrupt_requested = True
                if self.infospace_executor:
                    self.infospace_executor.interrupt_requested = True
                self.active_task_wip = None
                self.active_task_wip_waiting = False
                self._task_wip_pre_resource_ids = None

            self._publish_execution_state()
            logger.info(f"Task WIP {note_name} deleted (removed {len(deleted_goals)} goals: {deleted_goals})")
        except Exception as e:
            logger.error(f"Error in task_wip_delete handler: {e}")

    def _handle_task_wip_interrupt(self, sample):
        """Handle request to interrupt an active task WIP."""
        try:
            data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            note_name = data.get("note_name")
            if not note_name or self.active_task_wip != note_name:
                return
            # Interrupt any running milestone goal
            self.interrupt_requested = True
            if self.infospace_executor:
                self.infospace_executor.interrupt_requested = True
            # Update WIP note
            wip = self._read_task_wip()
            if wip:
                wip["status"] = "interrupted"
                wip["current_milestone"] = None
                wip.setdefault("accumulated_findings", []).append("Task interrupted by user via UI")
                self._update_task_wip(wip)
            self.active_task_wip = None
            self.active_task_wip_waiting = False
            self._task_wip_pre_resource_ids = None
            self._say_to_user("Task establishment interrupted.")
            self._publish_execution_state()
            logger.info(f"Task WIP {note_name} interrupted via UI")
        except Exception as e:
            logger.error(f"Error in task_wip_interrupt handler: {e}")

    def _cleanup_transient_resources(self, created_ids: set, keep_ids: set = None, label: str = ""):
        """
        Delete transient resources created during a goal, preserving
        persistent resources, system resources, and any explicitly kept IDs.
        """
        if not self.resource_manager or not created_ids:
            return
        keep_ids = set(keep_ids or ())
        _PRESERVED_COLLECTIONS = {"conversation", "conversation_history", "_tasks", "_scheduled_goals"}
        _PRESERVED_NOTES = {"_situation", "_situation_prev", "_user_concerns"}
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
            self.infospace_executor.execute_action({"type": "say", "target": "user", "value": text})

    # ── Task WIP (milestone loop) methods ─────────────────────────────────

    def _begin_task_establishment(self, user_text: str):
        """Create a task WIP Note and start the milestone loop."""
        self._task_wip_counter += 1
        wip_id = f"twip_{self._task_wip_counter}"
        note_name = f"_task_wip_{self._task_wip_counter}"
        now = datetime.now().isoformat()
        wip_content = {
            "task_wip_id": wip_id,
            "intention": user_text,
            "status": "in_progress",
            "phase": "specification",
            "milestones_completed": [],
            "current_milestone": None,
            "accumulated_findings": [],
            "created": now,
            "updated": now,
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
        self.active_task_wip = note_name
        self.active_task_wip_waiting = False
        # Snapshot resource IDs so we can clean up establishment artifacts later
        self._task_wip_pre_resource_ids = set(
            self.resource_manager.resource_registry.keys()
        ) if self.resource_manager else set()
        logger.info(f'📋 Task WIP created: {note_name} — "{user_text[:80]}"')
        self._say_to_user(f"Task received. Beginning establishment for: {user_text[:200]}")

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
        "PHASES (in required order — you MUST visit each phase before moving to the next):\n"
        "  specification — Clarify requirements with the user via `ask`. What exactly should\n"
        "    the task do? What parameters, thresholds, or preferences matter? The goal you\n"
        "    submit should use `ask` to have a conversation with the user and return the\n"
        "    collected answers as its final result. The answers will be recorded\n"
        "    automatically in the milestone history.\n"
        "  capability_evaluation — Test whether the needed tools/data sources actually work.\n"
        "    E.g., can we fetch the right emails? Can we access the web page? Submit a small\n"
        "    probe goal that tries the key operation and reports what it found.\n"
        "  infrastructure_setup — Create any persistent notes, collections, or other state\n"
        "    the recurring task will need. If the task mentions creating or updating a named\n"
        "    Note or Collection, create it now. E.g., create a digest collection, a tracking\n"
        "    note, a named Note for storing results. Even if the phase has minimal work,\n"
        "    submit a goal that creates/verifies the required resources exist.\n"
        "  activation — All establishment is done. Move to COMPLETE.\n"
        "  complete — The task is fully established and ready to be scheduled.\n\n"
        "PHASE RULES:\n"
        "- You may only advance to the NEXT phase (no skipping).\n"
        "- COMPLETE is only valid from the activation phase.\n"
        "- If a phase has nothing to do, submit a brief milestone that records\n"
        "  'no setup needed' and advances to the next phase.\n\n"
        "Determine what to do next. Your options are:\n"
        "1. SUBMIT_GOAL: Submit a SHORT, FOCUSED milestone goal (one establishment step).\n"
        "   Keep goal text concise — one clear objective, not a multi-step execution plan.\n"
        "2. FALL_BACK: A previous milestone result was unexpected. Describe what to revisit.\n"
        "3. COMPLETE: The task is fully established. Provide a summary.\n\n"
        "Respond in this exact format:\n"
        "ACTION: <SUBMIT_GOAL | FALL_BACK | COMPLETE>\n"
        "PHASE: <which phase this belongs to>\n"
        "GOAL_TEXT: <full goal text if SUBMIT_GOAL, or explanation if FALL_BACK/COMPLETE>\n"
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
        milestones_text = "None yet" if not milestones else "\n".join(
            f"- [{m.get('status', '?')}] {m.get('goal_text', '')[:120]}: {m.get('result_summary', '')[:200]}"
            for m in milestones
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

        prompt = self._ADVANCE_TASK_PROMPT.format(
            intention=wip.get("intention", ""),
            phase=wip.get("phase", "specification"),
            milestones=milestones_text,
            findings=findings_text,
            last_result=last_result,
        )
        if spiral_warning:
            prompt += spiral_warning

        try:
            resp = self.llm_generate([prompt], max_tokens=500, temperature=0.3)
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
            wip["current_milestone"] = goal_text[:200]
            self._update_task_wip(wip)

            # --- Fix A: Direct ask for specification phase ---
            # If the goal is purely "ask the user clarifying questions" during
            # specification, skip the full planner and issue the ask directly.
            # This avoids content loss where the planner returns a status message
            # instead of the user's actual answers.
            if new_phase == "specification" and self._is_ask_only_goal(goal_text):
                self._run_direct_ask_milestone(wip, goal_text)
                return

            # Create scheduled goal with task_wip_id linkage
            scheduled_goal = self._upsert_scheduled_goal(goal_text)
            goal_id = scheduled_goal["goal_id"]
            scheduled_goal["task_wip_id"] = wip.get("task_wip_id", "")
            self._save_scheduled_goal(scheduled_goal)

            self._active_scheduled_goal_id = goal_id
            self._update_scheduled_goal(goal_id, is_running=True, status="running")
            pre_resource_ids = set(self.resource_manager.resource_registry.keys()) if self.resource_manager else set()

            def _run_milestone_goal():
                result = self.parse_and_set_goal("", goal_text) or {}
                self._set_scheduled_goal_result(goal_id, result, used_cache=False, pre_resource_ids=pre_resource_ids)
                return result

            self.active_task_wip_waiting = True
            self._run_goal_on_thread(_run_milestone_goal)

        elif action == "FALL_BACK":
            # Remove last milestone and revert phase
            if milestones:
                wip["milestones_completed"] = milestones[:-1]
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
        lower = goal_text.lower()
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
                    if rid and self.resource_manager:
                        res = self.resource_manager.get_resource(rid)
                        if res:
                            response_text = str(getattr(res, 'content', '') or getattr(res, 'text', '') or '')
                            if not response_text:
                                props = getattr(res, 'properties', {}) or {}
                                response_text = str(props.get('text', '') or props.get('content', ''))

                success = bool(response_text)
                summary_text = response_text[:1500] if response_text else "No response received"
                milestone_record = {
                    "goal_text": goal_text[:200],
                    "result_summary": summary_text,
                    "status": "completed" if success else "failed",
                    "timestamp": datetime.now().isoformat(),
                }
                wip_fresh = self._read_task_wip()
                if wip_fresh:
                    wip_fresh.setdefault("milestones_completed", []).append(milestone_record)
                    wip_fresh["current_milestone"] = None
                    wip_fresh["updated"] = datetime.now().isoformat()
                    wip_fresh.setdefault("accumulated_findings", []).append(
                        f"User clarification: {summary_text}"
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

    def _complete_task_wip(self, wip: Dict[str, Any], summary: str):
        """Finalize task: create scheduled goal and update WIP Note."""
        intention = wip.get("intention", "")

        # Ask LLM to synthesize the operational goal from accumulated findings
        synth_prompt = (
            f"A task has been fully specified. Synthesize an operational goal that can be executed recurring.\n\n"
            f"INTENTION: {intention}\n"
            f"FINDINGS: {json.dumps(wip.get('accumulated_findings', []))}\n"
            f"MILESTONES: {json.dumps([m.get('goal_text', '') for m in wip.get('milestones_completed', [])])}\n\n"
            f"Respond with:\n"
            f"GOAL_TEXT: <the full recurring goal instruction>\n"
            f"NAME: <distinctive name that captures the specific data source AND target artifact,\n"
            f"  e.g. 'HF Daily Papers → Agent Architecture Notes', max 60 chars>\n"
            f"SCHEDULE: <manual | daily | recurring>\n"
        )
        try:
            resp = self.llm_generate([synth_prompt], max_tokens=300, temperature=0.3)
            resp_text = getattr(resp, 'text', '') if hasattr(resp, 'text') else str(resp)
        except Exception as e:
            logger.error(f'Task WIP completion synth failed: {e}')
            resp_text = f"GOAL_TEXT: {intention}\nNAME: {intention[:60]}\nSCHEDULE: manual"

        goal_match = re.search(r'GOAL_TEXT:\s*(.+?)(?:\n|$)', resp_text)
        name_match = re.search(r'NAME:\s*(.+?)(?:\n|$)', resp_text)
        sched_match = re.search(r'SCHEDULE:\s*(\S+)', resp_text)

        op_goal_text = goal_match.group(1).strip() if goal_match else intention
        op_name = name_match.group(1).strip()[:60] if name_match else intention[:60]
        schedule_mode = sched_match.group(1).strip().lower() if sched_match else "manual"
        if schedule_mode not in ("manual", "daily", "recurring", "auto"):
            schedule_mode = "manual"

        # Create the operational scheduled goal
        scheduled_goal = self._upsert_scheduled_goal(op_goal_text)
        goal_id = scheduled_goal["goal_id"]
        scheduled_goal["name"] = op_name
        scheduled_goal["name_customized"] = True
        scheduled_goal["schedule_mode"] = schedule_mode
        # Daily goals need a run_at time or they're never eligible for auto-execution
        if schedule_mode == "daily" and not scheduled_goal.get("run_at"):
            scheduled_goal["run_at"] = "09:00"
        scheduled_goal["task_context_note"] = self.active_task_wip
        self._save_scheduled_goal(scheduled_goal)

        # Update WIP Note to completed
        wip["status"] = "completed"
        wip["phase"] = "complete"
        wip["scheduled_goal_id"] = goal_id
        wip["completion_summary"] = summary
        self._update_task_wip(wip)

        # Clean up establishment artifacts
        self._cleanup_task_establishment(wip, goal_id)

        self.active_task_wip = None
        self.active_task_wip_waiting = False
        logger.info(f'📋 Task established: {op_name} (goal_id={goal_id}, schedule={schedule_mode})')
        self._say_to_user(
            f"Task established: {op_name}\n"
            f"Schedule: {schedule_mode}\n"
            f"Goal: {op_goal_text[:200]}"
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

    # ── Chat-mode response (lightweight, no planning pipeline) ────────────

    def _handle_chat_response(self, text: str, source: str = 'User'):
        """Respond to user via direct LLM call, bypassing the planning pipeline.

        Used when no goal: prefix is present.  Read-only — no resource mutations.
        """
        # Record the incoming turn so envision sees full history
        self.conversation_store.record_incoming(source, text)

        # Characterise the conversational moment (cheap LLM call)
        envision = self._envision_conversation_turn(source, text, "")

        # Build system prompt (character + setting + capabilities + drives + agent state)
        system_prompt = self._update_system_prompt()

        # Build user prompt with dialog history + envision guidance + user message
        recent_turns = ""
        entity_data = self.conversation_store.get_entity_context(source, limit=6, scope='current')
        if entity_data and 'conversation_history' in entity_data:
            for entry in entity_data['conversation_history'][-6:]:
                if isinstance(entry, dict) and 'source' in entry and 'text' in entry:
                    text_preview = str(entry['text'])[:200]
                    recent_turns += f"{entry['source']}: {text_preview}\n"

        user_prompt = (
            f"RECENT DIALOG:\n{recent_turns}\n"
            f"Their move: {envision['turn_intent']}\n"
            f"Your move: {envision['my_move']}\n\n"
            f"Message from {source}: {text}\n\n"
            f"Respond in character. Be concise."
        )

        try:
            result = self.llm_generate(
                messages=[system_prompt, user_prompt],
                max_tokens=400,
                temperature=0.7,
            )
            if result.success and result.text:
                response = result.text.strip()
                self._say_to_user(response)
                self.conversation_store.record_outgoing(source, response, act_type="chat")
                logger.info(f'💬 {self.character_name} chat response to {source}: {response[:80]}...')
            else:
                logger.warning(f'Chat LLM call failed: {getattr(result, "error", "unknown")}')
        except Exception as e:
            logger.error(f'Error in chat response: {e}')
            traceback.print_exc()

    def _handle_sensor_alert_response(self, alert_text: str, sensor_name: str):
        """Respond to a sensor alert in character, without recording conversation turns."""
        system_prompt = self._update_system_prompt()
        user_prompt = (
            f"Sensor alert from {sensor_name}:\n{alert_text}\n\n"
            f"React briefly in character. This is an internal sensor notification, not a conversation."
        )
        try:
            result = self.llm_generate(
                messages=[system_prompt, user_prompt],
                max_tokens=300,
                temperature=0.7,
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

    def _drain_sensor_queues(self):
        """Drain sensor priority queues after a goal completes.

        Order: alerts → triggers → informs.
        """
        # Alerts: generate chat-style acknowledgement (no conversation recording)
        while self._sensor_alert_queue:
            alert = self._sensor_alert_queue.pop(0)
            sensor_name = alert.get('sensor_name', 'unknown')
            summary = alert.get('content', '')
            data = alert.get('data', [])
            logger.info(f'🚨 {self.character_name} processing alert from {sensor_name}: {summary[:80]}')
            # Build alert text with structured data if available
            alert_text = f"[ALERT from sensor {sensor_name}]: {summary}"
            if data:
                for item in data:
                    title = item.get('title', '')
                    url = item.get('url', '')
                    label = f"{title} — {url}" if title else url
                    alert_text += f"\n  - {label}"
            # Respond to alert without recording conversation turns
            self._handle_sensor_alert_response(alert_text, sensor_name)

        # Triggers: dispatch the first eligible as next goal
        if self._sensor_trigger_queue:
            trigger = self._sensor_trigger_queue.pop(0)
            goal_name = trigger.get('goal_name', '')
            logger.info(f'⚡ {self.character_name} dispatching triggered goal: {goal_name}')
            # Find the scheduled goal matching trigger target
            for goal in self._all_scheduled_goals():
                if goal.get('name') == goal_name or goal.get('goal_text', '').strip() == goal_name:
                    goal_id = goal.get('goal_id')
                    if goal_id:
                        self._handle_goal_proceed(goal_id=goal_id, source='sensor_trigger')
                        break
            else:
                logger.warning(f'Triggered goal "{goal_name}" not found in scheduled goals')
            # Remaining triggers stay queued for next drain cycle

        # Informs: retained as rolling context in _build_agent_state_block (not drained)

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
            if raw_transcript:
                try:
                    self.user_concern_model.update_from_conversation(
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
            else:
                logger.warning(f'Failed to add dialog summary to conversation_history: {add_result.get("reason", "unknown")}')

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
            # In step mode, pause after plan execution (skip if already paused)
            if self.execution_mode == 'step' and not self.execution_paused:
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

        # Scheduled goals — full view for conversational grounding
        try:
            scheduled = self._all_scheduled_goals()
            if scheduled:
                def _goal_line(g):
                    name = g.get('name') or g.get('goal_text', '?')[:80]
                    status = g.get('status', '?')
                    result = g.get('last_result', '')
                    line = f"  - {name} [{status}]"
                    if result:
                        line += f" — {result[:100]}"
                    return line

                pending = [g for g in scheduled if g.get('status') in ('ready', 'pending', 'scheduled')]
                recent = [g for g in scheduled if g.get('status') in ('completed', 'failed', 'interrupted')]
                # Show most recently updated first for completed/failed
                recent.sort(key=lambda g: g.get('updated', ''), reverse=True)

                lines = []
                if pending:
                    lines.append("Upcoming/ready:")
                    lines.extend(_goal_line(g) for g in pending[:5])
                if recent:
                    lines.append("Recently completed/failed:")
                    lines.extend(_goal_line(g) for g in recent[:5])
                if lines:
                    parts.append("Scheduled goals:\n" + "\n".join(lines))
                else:
                    parts.append("Scheduled goals: none active or recent")
            else:
                parts.append("Scheduled goals: none")
        except Exception:
            parts.append("Scheduled goals: unavailable")

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
                        label = f"{title} — {url}" if title else url
                        lines.append(f"      {label}")
                else:
                    # Legacy plain-text
                    lines.append(f"  - [{sensor_name}] {summary}")
            parts.append("Recent sensor observations:\n" + "\n".join(lines))

        return "\n".join(parts)

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
                user_prompt += '\n## CONVERSATION HISTORY (for tone and continuity only — do NOT infer task state from this)'
                for i, memory in enumerate(entity_context['conversation_history']):
                    user_prompt += f"\n\t{memory['source']}: {memory['text'][:600]}..."
                user_prompt += '\n'

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
                'situation_context': self._build_situation_context(),
                'recent_context': recent_context,
                'executor': self.infospace_executor,  # Pass executor for incremental planner
                'vision_goal': goal_text,
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

                if disposition == 'alert':
                    self._sensor_alert_queue.append(sensor_entry)
                    logger.info(f'🚨 {self.character_name} sensor ALERT queued from {sensor_name}: {sensor_content[:80]}')
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
        """Handle LLM Primary/Alt toggle from UI.

        If no goal is running, applies immediately.
        If a goal is in progress, queues the switch for the next goal boundary.
        """
        try:
            if not hasattr(self, 'character_name'):
                logger.warning('🔄 LLM toggle received before initialization complete')
                return
            # Toggle: if currently primary, switch to alt; if alt, switch to primary
            new_mode = 'alt' if self.llm_mode == 'primary' else 'primary'
            if self._is_goal_running():
                self.llm_switch_pending = new_mode
                logger.info(f'🔄 {self.character_name} LLM switch to {new_mode} queued (goal in progress)')
            else:
                self.llm_switch_pending = new_mode
                self._apply_pending_llm_switch()
            self._publish_execution_state()
        except Exception as e:
            logger.error(f'Error handling LLM toggle: {e}')
            traceback.print_exc()

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
            
            PRESERVED_COLLECTIONS = {'conversation', 'conversation_history', '_tasks', '_scheduled_goals'}
            PRESERVED_NOTES = {'_situation', '_situation_prev', '_user_concerns'}
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
            PRESERVED_NOTES = {'_situation', '_situation_prev', '_user_concerns'}
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
        """Handle stop command - pause execution."""
        try:
            logger.info(f'⏹️ Stop command received by {self.character_name}')
            # Treat stop as an immediate interrupt signal so blocking waits can exit.
            self.interrupt_requested = True
            if self.infospace_executor:
                self.infospace_executor.interrupt_requested = True
            self.execution_mode = 'step'
            self.execution_paused = True
            # Note: continuous_mode is NOT cleared here - user must toggle it off explicitly
            self.awaiting_user_input = False  # Clear awaiting flag
            if self.awaiting_ask_response:
                self.awaiting_ask_response = False
                self._ask_response_queue.put(None)  # Sentinel to unblock _execute_ask
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
            if self.awaiting_ask_response:
                self.awaiting_ask_response = False
                self._ask_response_queue.put(None)  # Sentinel to unblock _execute_ask
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
        PRESERVED_NOTES = {'_situation', '_situation_prev', '_user_concerns'}
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
        """Handle query for scheduled goals."""
        try:
            goals = self._all_scheduled_goals()
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

            # Refresh filesystem catalog so the planner sees current files
            if self.infospace_executor:
                self.infospace_executor._run_init_tool()

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
