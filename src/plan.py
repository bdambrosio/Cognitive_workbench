from __future__ import annotations

from enum import Enum
from weakref import WeakValueDictionary
import os, sys
from datetime import timedelta, datetime
import logging
import json
import numpy as np
import zenoh
from zenoh import ConsolidationMode, QueryTarget
#from sentence_transformers import SentenceTransformer
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import hash_utils

from typing import Any, Dict, List
from utils.llm_api import LLM
from Messages import SystemMessage, UserMessage
from templates import PLAN_TEMPLATE, PHYSIOLOGICAL_STATES
# Type checking imports
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from executive_node import ZenohExecutiveNode


# Configure logging with unbuffered output
# Console handler with WARNING level (less verbose)
console_handler = logging.StreamHandler(sys.stdout)
console_handler.setLevel(logging.WARNING)

# File handler with INFO level (full logging)
file_handler = logging.FileHandler('logs/executive_node.log', mode='w')
file_handler.setLevel(logging.INFO)

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
    handlers=[console_handler, file_handler],
    force=True
)
logger = logging.getLogger('executive_node')


class Mode(Enum):
    Think = "Think"
    Say = "Say"
    Do = "Do" 
    Move = "Move"
    Look = "Look"
    Listen = "Listen"



def find_first_digit(s):
    for char in s:
        if char.isdigit():
            return char
    return None  # Return None if no digit is found

def datetime_handler(obj):
    if isinstance(obj, datetime):
        return obj.isoformat()  # More standard format
    elif isinstance(obj, timedelta):
        return obj.total_seconds()
    else:
        return str(obj) 

class Stack:
    def __init__(self):
        """Simple stack implementation"""
        self.stack = []
        #print("Stack initialized")  # Debug print

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
    _id_counter = 0
    _instances = WeakValueDictionary()  # id -> instance mapping that won't prevent garbage collection
    
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
    

from datetime import timedelta

def parse_duration(duration_str: str) -> timedelta:
    """Convert duration string to timedelta
    Args:
        duration_str: Either minutes as int ("5") or with units ("2 minutes")
    Returns:
        timedelta object
    """
    if isinstance(duration_str, timedelta):
        return duration_str
    
    try:
        # Try simple integer (minutes)
        return timedelta(minutes=int(duration_str))
    except ValueError:
        # Try "X units" format
        try:
            amount, unit = duration_str.strip().split()
            amount = int(amount)
            unit = unit.lower().rstrip('s')  # handle plural
            
            if unit == 'minute':
                return timedelta(minutes=amount)
            elif unit == 'hour':
                return timedelta(hours=amount)
            elif unit == 'day':
                return timedelta(days=amount)
            else:
                # Default to minutes if unit not recognized
                return timedelta(minutes=amount)
        except:
            # If all parsing fails, default to 1 minute
            return timedelta(minutes=1)
        
class Task:
    _id_counter = 0
    _instances = WeakValueDictionary()  # id -> instance mapping that won't prevent garbage collection
    
    def __init__(self, name, description, reason, actors, goal, termination=None, start_time=None, duration=None):
        Task._id_counter += 1
        self.id = f"t{Task._id_counter}"
        Task._instances[self.id] = self
        self.name = name
        self.description = description
        self.reason = reason
        self.start_time = start_time
        self.duration = parse_duration(duration)
        self.termination = termination
        self.goal = goal
        self.actors = actors
        self.needs = ''
        self.result = ''
        self.acts = []
        self.completion_statement = ''
        self.progress = 0

    def __eq__(self, other):
        if not isinstance(other, Task):
            return False
        return self.id == other.id

    def __hash__(self):
        return hash(self.id)
    
    @classmethod
    def get_by_id(cls, id: str):
        return cls._instances.get(id)
    
    def short_string(self):
        return f'{self.name}: {self.description} \n reason: {self.reason} \n termination: {self.termination}'

    def to_string(self):
        return f'Task {self.name}: {self.description}; actors: {[actor.name for actor in self.actors]}; reason: {self.reason}; termination: {self.termination}'
    
    def to_fullstring(self):
        return f'Task {self.name}: {self.description}\n   Reason: {self.reason}\n   Actors: {[actor.name for actor in self.actors]}\n    Start time: {self.start_time};  Duration: {self.duration}\n    Termination Criterion: {self.termination}'
 
    def test_termination(self, events=''):
        """Test if recent acts, events, or world update have satisfied termination"""
        pass
    
class Action:
    _id_counter = 0
    _instances = WeakValueDictionary()  # id -> instance mapping that won't prevent garbage collection
    
    def __init__(self, mode, action, actors, reason='', duration=1, source=None, target=None, result=''):
        Action._id_counter += 1
        self.id = f"a{Action._id_counter}"
        Action._instances[self.id] = self
        self.mode = mode
        self.action = action
        self.actors = actors
        self.reason = reason
        self.duration = parse_duration(duration)
        self.source = source  # a task
        self.target = target  # an actor
        self.result = result

    @classmethod
    def get_by_id(cls, id: str):
        return cls._instances.get(id)

    def to_string(self):
        return f'Action t{self.id} {self.mode}: {self.action}; reason: {self.reason}; result: {self.result}'

class Autonomy:
    def __init__(self, act=True, scene=True, signal=True, goal=True, task=True, action=True):
        self.act = act
        self.scene = scene
        self.signal = signal
        self.goal = goal
        self.task = task
        self.action = action
            
def validate_and_create_goal(character_name, goal_hash):
        """Validate a goal hash and create a goal object
        
        Args:
            goal_hash: Hash-formatted goal definition
            signalCluster: SignalCluster this goal is for
        """
        goal_name = hash_utils.find('goal', goal_hash)
        description = hash_utils.find('description', goal_hash)
        other_character_name = hash_utils.find('otherCharacterName', goal_hash)
        termination = hash_utils.find('termination', goal_hash)

        if other_character_name and other_character_name.strip().lower() != 'none':
            other_character_name = other_character_name.strip().capitalize()
        else:
            other_character_name = None

        if goal_name and description and termination:
            goal = Goal(name=goal_name, 
                        actors=[character_name, other_character_name] if other_character_name else [character_name],
                        description=description, 
                        termination=termination.replace('##','').strip()

            )
            return goal
        else:
            print(f"Warning: Invalid goal generation response for {goal_hash}") 
            return None


# ---------------------------------------------------------------------------
# public API
# ---------------------------------------------------------------------------

def parse_plan_json(plan_text):
    """
    Parse JSON plan string into internal plan structure.
    
    Accepts:
    - With prefix: 'plan: {"plan": [...]}'
    - Without prefix: '{"plan": [...]}'
    - Array format: '[{...}, {...}]'
    
    Returns:
        Dict with 'plan' key containing list of actions
    """
    plan_text = plan_text.strip()
    
    # Remove 'plan:' prefix if present
    if plan_text.startswith('plan:'):
        plan_text = plan_text[5:].strip()
    
    # Parse JSON
    parsed = json.loads(plan_text)
    
    # Ensure dict with 'plan' key
    if isinstance(parsed, list):
        return {'plan': parsed}
    
    if isinstance(parsed, dict):
        return parsed
    
    raise ValueError(f"Invalid plan format: expected dict or list, got {type(parsed).__name__}")


# Legacy text format parser - DEPRECATED
# Kept for reference, remove after migration complete
def parse_plan_text(plan_text):
    """DEPRECATED: Use parse_plan_json instead. Text format no longer supported."""
    logger.warning("parse_plan_text is deprecated - use parse_plan_json with JSON format")
    raise ValueError("Text plan format deprecated - use JSON format: plan: {\"plan\": [...]}")

def verify_plan(plan_json: Any) -> bool:
    """
    Validate a plan object against the expected JSON grammar.

    Parameters
    ----------
    plan_json : str | dict
        Either a JSON string or a Python dict already parsed from JSON.

    Returns
    -------
    bool
        True  -> syntactically valid
        False -> invalid
    """
    try:
        plan_dict = _load(plan_json)
        valid = _validate_top(plan_dict)
        if valid:
            logger.info(f'✅ Plan validation passed')
        else:
            logger.error(f'❌ Plan validation failed')
        return valid
    except Exception as e:
        logger.error(f'❌ Plan validation error: {e}')
        # Any raised error means invalid
        return False

# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

_ALLOWED_TYPES = {"action", "say", "move", "think", "take", "place", "inspect", "use", "scan", "while", "if", "near", "look", "wait"}
_ALLOWED_CONDITION_TYPES = {"near", "can_see", "has_item", "notnear", "cant_see", "hasnt_item", "at_location", "notat_location", "believes", "notbelieves", "bound", "notbound"}

REQ_KEYS = {
    "say": {"type", "target", "value"},
    "move": {"type", "target"},
    "think": {"type", "value"},
    "look": {"type", "target"},
    "take": {"type", "target", "prediction"},
    "place": {"type", "target", "prediction"},
    "inspect": {"type", "target", "reason", "prediction"},
    "use": {"type", "target", "reason", "out", "prediction"},
    "scan": {"type", "target", "out", "prediction"},
    "while": {"type", "body", "condition"},
    "wait": {"type", "condition"},
    "if": {"type", "condition", "then"},          # "else" is optional
    "near": {"type", "target"},
}

OPTIONAL_KEYS = {
    "say": {"reason", "value", "prediction"},
    "move": {"reason", "value", "prediction"},
    "think": {"reason", "target", "prediction"},
    "look": {"reason", "value", "prediction"},
    "take": {"reason", "value"},
    "place": {"reason", "value"},
    "inspect": {"reason", "value"},
    "use": {"reason", "value"},
    "scan": {"reason"},  # scan has no optional keys
    "while": {"reason", "value"},
    "wait": {"reason", "target", "value"},
    "if": {"else"},
    "near": {"reason", "value"},
    "look": {"reason", "value", "prediction"},
}

REQ_CONDITION_KEYS = {
    "near": {"target"},
    "can_see": {"target"},
    "has_item": {"target"},
    "at_location": {"target"},
    "believes": {"value"},
    "notnear": {"target"},
    "cant_see": {"target"},
    "hasnt_item": {"target"},
    "notat_location": {"target"},
    "notbelieves": {"value"},
    "bound": {"target"},
    "notbound": {"target"}
}

def _load(plan_json: Any) -> Dict[str, Any]:
    """Load from a JSON string if needed."""
    if isinstance(plan_json, str):
        try:
            return json.loads(plan_json)
        except json.JSONDecodeError:
            print(f"Warning: Invalid JSON in plan_json: {plan_json}")
            return None
    if isinstance(plan_json, dict):
        return plan_json
    if isinstance(plan_json, list):
        return plan_json

def _validate_top(plan) -> bool:
    if isinstance(plan, dict) and 'plan' in plan:
        plan_steps = plan['plan']
        if not _validate_steps(plan_steps):
            return False
        # Add variable reference validation
        return _validate_variable_references(plan_steps)
    if isinstance(plan, list):
        if not _validate_steps(plan):
            return False
        # Add variable reference validation
        return _validate_variable_references(plan)
    if isinstance(plan, dict):
        return _validate_step(plan)
    return False

def _validate_steps(steps: List[Any]) -> bool:
    """Validate each step object in a list, recursively."""
    for step in steps:
        if not _validate_step(step):
            return False
    return True

def _validate_step(step: Any) -> bool:
    if not isinstance(step, dict) and not isinstance(step, list):
        return False
    if isinstance(step, list):
        return _validate_steps(step)
    step_type = step.get("type", None)
    if step_type not in _ALLOWED_TYPES and not isinstance(step, list):
        logger.error(f'Invalid step type: {step_type}')
        return False

    # --- key checks --------------------------------------------------------
    req = REQ_KEYS[step_type]
    opt = OPTIONAL_KEYS[step_type]
    keys = set(step.keys())
    if not req.issubset(keys):
        logger.error(f'Missing required keys for step type: {step_type}')
        return False
    if not keys.issubset(req | opt):
        logger.error(f'Extra keys for step type: {step_type}')
        return False

    # --- per‑type structural checks ----------------------------------------

    if step_type == "while":
        return (
            isinstance(step["condition"], dict)
            and _validate_condition(step["condition"])
            and isinstance(step["body"], list)
            and _validate_steps(step["body"])
        )

    if step_type == "if":
        if not (isinstance(step["condition"], dict)) or not _validate_condition(step["condition"]) or not isinstance(step["then"], list):
            logger.error(f'Invalid condition for step type: {step_type}')
            return False
        if "else" in step and not isinstance(step["else"], list):
            logger.error(f'Invalid else for step type: {step_type}')
            return False
        return _validate_steps(step["then"]) and _validate_steps(step.get("else", []))

    if step_type == "wait":
        # condition can be a dict (preferred) or a simple string (legacy); validate dict via _validate_condition
        cond = step.get("condition")
        if isinstance(cond, dict):
            return _validate_condition(cond)
        # Accept string condition for backward compatibility
        return isinstance(cond, str) and len(cond) > 0

    if isinstance(step, list):
        return all(_validate_steps(element) for element in step)

    return True  # otherwise all ok

def _validate_condition(condition: Any) -> bool:
    if not isinstance(condition, dict):
        logger.error(f'Invalid condition for step type: {condition}')
        return False
    if condition.get("type", None) not in _ALLOWED_CONDITION_TYPES: 
        logger.error(f'Invalid condition type: {condition.get("type", None)}')
        return False
    keys = set(condition.keys())
    if not REQ_CONDITION_KEYS[condition.get("type", None)].issubset(keys):
        logger.error(f'Missing required keys for condition type: {condition.get("type", None)}')    
        return False
    return True

# ---------------------------------------------------------------------------
# Pure planner entry (for offline replanning/testing)
# ---------------------------------------------------------------------------

def generate_plan_with_context(
    character: ZenohExecutiveNode,
    current_activity: json,
    goal_text: str,
    prompt_text: str,
    percepts_at_plan: Any = None,
    server_name: str = "openai",
    model_name: str = "gpt-4.1",
) -> Dict[str, Any]:
    """Generate a plan using explicit context only (no node state).

    Inputs should come from a plan_log entry to ensure parity during verification.
    Returns a verified plan dict; returns {"plan": []} on failure.
    """
    llm = LLM(server_name=server_name, model_name=model_name)
    try:

        system_prompt = """Task: rewrite the following goal statement into a plan according to the PLAN_TEMPLATE below.
Respond only with a JSON plan according to the provided PLAN_TEMPLATE. No prose or code fences; end with </end>."""
        parts = []
        parts.append("#Your current goal is:\n" + (goal_text or ""))
        parts.append("\n----Situation Data (dynamic)----\n" + (prompt_text or "")+'\n----End of Situation Data----\n')
        if percepts_at_plan is not None:
            try:
                parts.append("\n#Percepts at plan start (JSON):\n" + json.dumps(percepts_at_plan, ensure_ascii=False, indent=2))
            except Exception:
                pass
        # Optionally include allowed scan targets (types) derived from middle ontology
        allowed_types = []
        if character.ontology:
            allowed_types.extend(character.ontology.get('nouns', {}).copy())
        allowed_types.extend(character.map_types.get('resource_types', []))
        #allowed_types.extend(character.map_types.get('terrain_types', []))
        allowed_types.extend(character.map_types.get('infrastructure_types', []))
        allowed_types.extend(character.map_types.get('property_types', []))
        allowed_types.extend(character.map_types.get('character_names', []))
        skills = character.map_types.get('skills', [])
        for skill in skills:
            allowed_types.append(skill['name'])
        parts.append("\n#ALLOWED SCAN TARGETS (types)\n" + "\n".join(allowed_types)+"\n##")
        parts.append("\n#Plan syntax specification:\n" + str(PLAN_TEMPLATE))
        parts.append("\nRespond only with JSON, no other text. End with </end>\n")
        user_prompt = "\n".join(parts)

        llm = LLM(server_name=server_name, model_name=model_name)
        response = llm.ask(
            {"skills": json.dumps(skills)},
            [SystemMessage(content=system_prompt), UserMessage(content=user_prompt)],
            max_tokens=1500,
            stops=['</end>'],
            is_json=True,
            log=True
        )
        if isinstance(response, dict):
            plan_candidate = response
        else:
            text = response if isinstance(response, str) else getattr(response, 'text', None)
            plan_candidate = json.loads(text) if text else {"plan": []}

        # Verify; attempt one minimal repair if invalid
        if not verify_plan(plan_candidate):
            try:
                broken_json = json.dumps(plan_candidate, ensure_ascii=False, indent=2)
            except Exception:
                broken_json = str(plan_candidate)
            repair_system = (
                "You are a strict plan repairer. Given the PLAN_TEMPLATE and a broken plan JSON, "
                "emit a corrected plan that is valid per the template. Output only JSON. Do not output any reasoning. End with </end>."
            )
            repair_user_parts = []
            repair_user_parts.append("PLAN_TEMPLATE:\n" + str(PLAN_TEMPLATE))
            repair_user_parts.append("\nBroken plan (JSON):\n" + broken_json)
            repair_user_parts.append("\nNote: The plan failed validation against PLAN_TEMPLATE. Repair to a valid JSON plan.")
            repair_user = "\n".join(repair_user_parts)
            repair_resp = llm.ask(
                {},
                [SystemMessage(content=repair_system), UserMessage(content=repair_user)],
                max_tokens=1200,
                stops=['</end>'],
                is_json=True,
            )
            if isinstance(repair_resp, dict):
                repaired = repair_resp
            else:
                rtext = repair_resp if isinstance(repair_resp, str) else getattr(repair_resp, 'text', None)
                repaired = json.loads(rtext) if rtext else {"plan": []}
            if verify_plan(repaired):
                return repaired
            return {"plan": []}
        return plan_candidate
    except Exception as e:
        logger.error(f'Error generating plan with context: {e}')
        return {"plan": []}

def _validate_variable_references(plan_steps: List[Any]) -> bool:
    """
    Validate that all variable references ($variable_name) are bound by scan actions.
    This is a basic validation - detailed dependency analysis is Phase 2.
    """
    bound_variables = set()
    variable_references = []
    
    def collect_variables_and_references(steps):
        for step in steps:
            if isinstance(step, dict):
                step_type = step.get('type', '')
                
                # Collect bound variables from scan actions
                if step_type == 'scan':
                    out_var = step.get('out', '')
                    if out_var:
                        bound_variables.add(out_var)
                
                # Collect variable references from targets
                target = step.get('target', '')
                if isinstance(target, str) and target.startswith('$'):
                    var_name = target[1:]  # Remove $ prefix
                    variable_references.append((var_name, step_type))
                
                # Handle nested structures
                if step_type == 'while' and 'body' in step:
                    collect_variables_and_references(step['body'])
                if step_type == 'if':
                    if 'then' in step:
                        collect_variables_and_references(step['then'])
                    if 'else' in step:
                        collect_variables_and_references(step['else'])
            elif isinstance(step, list):
                collect_variables_and_references(step)
    
    collect_variables_and_references(plan_steps)
    
    # Check that all referenced variables are bound
    for var_name, step_type in variable_references:
        if var_name not in bound_variables:
            logger.error(f'Variable ${var_name} referenced in {step_type} action but not bound by any scan action')
            return False
    
    return True

def is_near(character, target: str, negated: bool) -> bool:
    """Check if a target (resource or character) is near this character."""
    try:
        character_name = character.character_name
        for reply in character.session.get(f"cognitive/{character_name}/situation/proximity?target={target}&negated={negated}", target=QueryTarget.BEST_MATCHING, consolidation=ConsolidationMode.NONE, timeout=6.0 if not character.debug else 300.0):
            if reply.ok:
                data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                if data['success']:
                    return {'value': data['value'], 'binding': data['binding']}
            break
    except Exception as e:
        logger.error(f'Error checking proximity for {target}: {e}')
    return {'value': False, 'binding': None}

def _evaluate_condition(character: ZenohExecutiveNode, condition: dict, observations: dict) -> bool:
        """Evaluate a condition action dict using distributed node queries."""
        # The key to conditions is access to the character's beliefs and knowledge base.
        # Since this is distributed among nodes, we will used targeted queires by predicate.
        
        # Use shared utilities for validation and variable dereferencing
        from utils.condition_utils import validate_condition_structure, deref_plan_target, normalize_condition_type
        
        if not validate_condition_structure(condition):
            return {'value': False, 'binding': None}
        
        condition_type = condition['type']
        normalized_type, negated = normalize_condition_type(condition_type)
        if 'target' in condition:
            raw_target = condition['target']
            target = deref_plan_target(character.plan_bindings, raw_target)
        else:
            raw_target = condition.get('value', '')
            target = raw_target
        character_name = character.character_name
        result = {'value': False, 'binding': None}
        
        # Local fast-path helpers using last_situation_data
        def _target_matches(name: str, target_str: str) -> bool:
            if not isinstance(name, str) or not isinstance(target_str, str):
                return False
            # Case-insensitive comparison to handle hyphenated names
            return name.lower() == target_str.lower()
        
        # NEW: binding test conditions
        if normalized_type == 'bound':
            try:
                # Accept "$var" or plain var name; treat None/empty as unbound
                var_name = raw_target[1:] if isinstance(raw_target, str) and raw_target.startswith('$') else raw_target
                val = character.plan_bindings.get(var_name, None)
                is_bound = not (val is None or val == '' or val == [] or val == {})
                if not negated:  # 'bound'
                    return {'value': is_bound, 'binding': val if is_bound else None}
                else:            # 'notbound'
                    return {'value': (not is_bound), 'binding': None}
            except Exception:
                # On any error, treat as unbound
                return {'value': False if not negated else True, 'binding': None}
        
        def _local_find_visible(tgt: str):
            try:
                if not getattr(character, 'last_situation_data', None):
                    return None
                tgt_lower = tgt.lower() if isinstance(tgt, str) else tgt
                data = character.last_situation_data or {}
                for view in data.get('views', []):
                    if 'characters' in view:
                        for c in view['characters']:
                            nm = c.get('name', '')
                            if nm and (nm.lower() == tgt_lower or tgt_lower == 'person'):
                                return nm
                    if 'resources' in view:
                        for r in view['resources']:
                            nm = r.get('name', '')
                            if nm and _target_matches(nm, tgt):
                                return nm
                    if 'paths' in view:
                        for p in view['paths']:
                            nm = p.get('name', '')
                            if nm and _target_matches(nm, tgt):
                                return nm
                    if 'terrain' in view:
                        terr = view.get('terrain')
                        if isinstance(terr, str) and terr.lower() == tgt_lower:
                            return terr
                return None
            except Exception:
                return None
        
        def _local_find_near(tgt: str):
            try:
                if not getattr(character, 'last_situation_data', None):
                    return None
                tgt_lower = tgt.lower() if isinstance(tgt, str) else tgt
                data = character.last_situation_data or {}
                for view in data.get('views', []):
                    if 'characters' in view:
                        for c in view['characters']:
                            if float(c.get('distance', 20)) <= 2 and (c.get('name', '').lower() == tgt_lower or tgt_lower == 'person'):
                                return c.get('name', '')
                    if 'resources' in view:
                        for r in view['resources']:
                            if float(r.get('distance', 20)) <= 2:
                                nm = r.get('name', '')
                                if nm and _target_matches(nm, tgt):
                                    return nm
                    if 'paths' in view:
                        for p in view['paths']:
                            if float(p.get('distance', 20)) <= 2:
                                nm = p.get('name', '')
                                if nm and _target_matches(nm, tgt):
                                    return nm
                    if 'terrain' in view:
                        terr = view.get('terrain')
                        if isinstance(terr, str) and terr.lower() == tgt_lower:
                            return terr
                return None
            except Exception:
                return None
        
        def _local_find_at(tgt: str):
            try:
                if not getattr(character, 'last_situation_data', None):
                    return None
                tgt_lower = tgt.lower() if isinstance(tgt, str) else tgt
                data = character.last_situation_data or {}
                for view in data.get('views', []):
                    if 'characters' in view:
                        for c in view['characters']:
                            if float(c.get('distance', 20)) <= 1 and (c.get('name', '').lower() == tgt_lower or tgt_lower == 'person'):
                                return c.get('name', '')
                    if 'resources' in view:
                        for r in view['resources']:
                            if float(r.get('distance', 20)) <= 2:
                                nm = r.get('name', '')
                                if nm and _target_matches(nm, tgt):
                                    return nm
                    if 'paths' in view:
                        for p in view['paths']:
                            if float(p.get('distance', 20)) <= 2:
                                nm = p.get('name', '')
                                if nm and _target_matches(nm, tgt):
                                    return nm
                    if 'terrain' in view:
                        terr = view.get('terrain')
                        if isinstance(terr, str) and terr.lower() == tgt_lower:
                            return terr
                return None
            except Exception:
                return None
        
        try:
            # Query the appropriate node based on normalized condition type
            if normalized_type == 'near':
                # Treat exact inventory id as near
                try:
                    inv = getattr(character, 'inventory_cache', []) or []
                    if isinstance(target, str) and target in inv:
                        return {'value': (not negated), 'binding': target} if not negated else {'value': False, 'binding': None}
                except Exception:
                    pass
                # Local fast path in views
                binding = _local_find_near(target)
                if binding is not None:
                    return {'value': (not negated), 'binding': binding} if not negated else {'value': False, 'binding': None}
                # Fallback to distributed query
                try:
                    for reply in character.session.get(f"cognitive/{character_name}/situation/proximity?target={target}&negated={negated}", target=QueryTarget.BEST_MATCHING, consolidation=ConsolidationMode.NONE, timeout=6.0 if not character.debug else 300.0):
                        if reply.ok:
                            data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                            if data['success']:
                                return {'value': data['value'], 'binding': data['binding']}
                        break
                except Exception as e:
                    logger.error(f'Error querying proximity for {target}: {e}')
                # If no local match and negated, treat as True; otherwise False
                return {'value': True, 'binding': None} if negated else {'value': False, 'binding': None}
                    
            elif normalized_type == 'can_see':
                # Local fast path
                binding = _local_find_visible(target)
                if binding is not None:
                    return {'value': (not negated), 'binding': binding} if not negated else {'value': False, 'binding': None}
                # Fallback
                try:
                    for reply in character.session.get(f"cognitive/{character_name}/situation/visibility?target={target}&negated={negated}", target=QueryTarget.BEST_MATCHING, consolidation=ConsolidationMode.NONE, timeout=6.0 if not character.debug else 300.0):
                        if reply.ok:
                            data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                            if data['success']:
                                logger.info(f'Visibility query for {target}: {result}')
                                return {'value': data['value'], 'binding': data['binding']}
                        break
                except Exception as e:
                    logger.error(f'Error querying visibility for {target}: {e}')
                return {'value': True, 'binding': None} if negated else {'value': False, 'binding': None}
            
            elif normalized_type == 'has_item':
                # Prefer distributed memory_node for authoritative inventory
                try:
                    for reply in character.session.get(f"cognitive/{character_name}/memory/inventory?item={target}&negated={negated}", target=QueryTarget.BEST_MATCHING, consolidation=ConsolidationMode.NONE, timeout=6.0 if not character.debug else 300.0):
                        if reply.ok:
                            data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                            if data['success']:
                                result = {'value': data['value'], 'binding': data['binding']}
                                logger.info(f'Inventory query for {target}: {result}')
                                return result
                        break
                except Exception as e:
                    logger.error(f'Error querying inventory for {target}: {e}')
                return {'value': False, 'binding': None}
                    
            elif normalized_type == 'at_location':
                # Local fast path
                binding = _local_find_at(target)
                if binding is not None:
                    return {'value': (not negated), 'binding': binding} if not negated else {'value': False, 'binding': None}
                # Fallback
                try:
                    for reply in character.session.get(f"cognitive/{character_name}/situation/location?target={target}&negated={negated}", target=QueryTarget.BEST_MATCHING, consolidation=ConsolidationMode.NONE, timeout=6.0 if not character.debug else 300.0):
                        if reply.ok:
                            data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                            if data['success']:
                                result = {'value': data['value'], 'binding': data['binding']}
                                logger.info(f'Location query for {target}: {target}')
                                return result
                        break
                except Exception as e:
                    logger.error(f'Error querying location for {target}: {e}')
                return {'value': True, 'binding': None} if negated else {'value': False, 'binding': None}
                    
            elif normalized_type in ('believes', 'notbelieves'):
                try:
                    from Messages import SystemMessage, UserMessage
                    statement = condition.get('value','False')
                    sys_prompt = f"Question: Does the character believe: {statement}? That is, is that statement in or directly derivable from the users facts below?Answer Yes or No only."
                    user_prompt = f'User facts: {observations}\n\nDo not include any other text in your response, just Yes or No.'
                    resp = character.llm_client.generate([sys_prompt, user_prompt], max_tokens=10, stops=['</end>'])
                    text = resp if isinstance(resp, str) else getattr(resp, 'text', str(resp))
                    val = str(text).strip().lower().startswith('y')
                    return {'value': (not val) if negated else val, 'binding': None}
                except Exception as e:
                    logger.warning(f"believes check failed ({target}): {e}")
                    return {'value': False, 'binding': None}
            else:
                logger.error(f'Unknown condition type: {condition_type} (normalized: {normalized_type})')
                return {'value': False, 'binding': None}
                
        except Exception as e:
            logger.error(f'Error evaluating condition {condition_type}({target}): {e}')
            return {'value': False, 'binding': None}
        
        # If we get here, the query failed or returned no response
        logger.warning(f'Condition evaluation failed for {condition_type}({target}) - returning False')
        return {'value': False, 'binding': None}

def remaining_plan(original_plan: dict, step_stack) -> dict:
    """
    Extract the remaining plan based on current execution state.
    
    Parameters
    ----------
    original_plan : dict
        The complete original plan with 'plan' key containing list of steps
    step_stack : Stack
        The current execution stack with frame information
        
    Returns
    -------
    dict
        A valid plan containing the remaining steps to be executed
    """
    if not original_plan or 'plan' not in original_plan:
        return {'plan': []}
    
    if step_stack.is_empty():
        # No execution state, return original plan
        return original_plan
    
    # Get the stack entries to analyze current state
    stack_entries = step_stack.get_entries()
    if not stack_entries:
        return original_plan
    
    # Start with the original plan structure
    remaining_steps = []
    original_steps = original_plan['plan']
    
    # Process each frame in the stack to determine remaining steps
    for i, frame in enumerate(stack_entries):
        plan = frame['plan']
        idx = frame['idx']
        frame_type = frame['type']
        
        if frame_type == 'main':
            # Main plan frame - add remaining steps from current index
            if idx < len(plan):
                remaining_steps = plan[idx:]
            else:
                # Plan is complete
                remaining_steps = []
                
        elif frame_type == 'while':
            # Do-while frame - add the loop with remaining body steps
            if idx < len(plan):
                remaining_body = plan[idx:]
                # Reconstruct the while step with remaining body
                while_step = {
                    'type': 'while',
                    'body': remaining_body,
                    'condition': frame.get('condition', {})
                }
                remaining_steps = [while_step]
                
                # Add remaining steps from parent frame if any
                if i + 1 < len(stack_entries):
                    parent_frame = stack_entries[i + 1]
                    parent_plan = parent_frame['plan']
                    return_idx = frame.get('return_to', 0)
                    if return_idx < len(parent_plan):
                        remaining_steps.extend(parent_plan[return_idx:])
            else:
                # Loop body is complete, continue with parent frame
                if i + 1 < len(stack_entries):
                    parent_frame = stack_entries[i + 1]
                    parent_plan = parent_frame['plan']
                    return_idx = frame.get('return_to', 0)
                    if return_idx < len(parent_plan):
                        remaining_steps = parent_plan[return_idx:]
                        
        elif frame_type in ['if_then', 'if_else']:
            # If frame - add the appropriate branch with remaining steps
            if idx < len(plan):
                remaining_branch = plan[idx:]
                # Reconstruct the if step with the appropriate branch
                if_step = {
                    'type': 'if',
                    'condition': frame.get('condition', {}),
                    'then': remaining_branch if frame_type == 'if_then' else [],
                    'else': remaining_branch if frame_type == 'if_else' else []
                }
                remaining_steps = [if_step]
                
                # Add remaining steps from parent frame if any
                if i + 1 < len(stack_entries):
                    parent_frame = stack_entries[i + 1]
                    parent_plan = parent_frame['plan']
                    return_idx = frame.get('return_to', 0)
                    if return_idx < len(parent_plan):
                        remaining_steps.extend(parent_plan[return_idx:])
            else:
                # Branch is complete, continue with parent frame
                if i + 1 < len(stack_entries):
                    parent_frame = stack_entries[i + 1]
                    parent_plan = parent_frame['plan']
                    return_idx = frame.get('return_to', 0)
                    if return_idx < len(parent_plan):
                        remaining_steps = parent_plan[return_idx:]
    
    # Ensure we have a valid plan structure
    if not remaining_steps:
        return {'plan': []}
    
    # Validate the remaining plan
    remaining_plan_dict = {'plan': remaining_steps}
    if not verify_plan(remaining_plan_dict):
        # If validation fails, return empty plan
        logger.warning("Generated remaining plan failed validation, returning empty plan")
        return {'plan': []}
    
    return remaining_plan_dict
