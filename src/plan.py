from __future__ import annotations

from enum import Enum
from weakref import WeakValueDictionary
import os, sys
from datetime import timedelta, datetime
import logging
import json
import numpy as np
#from sentence_transformers import SentenceTransformer
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import hash_utils

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

import json
from typing import Any, Dict, List

# ---------------------------------------------------------------------------
# public API
# ---------------------------------------------------------------------------

def parse_plan_text(plan_text):
        """Parse plan text into internal plan structure matching the LLM format."""
        plan_text = plan_text.strip()
        
        # Handle single-line format: "plan: action(target, value)"
        if plan_text.startswith('plan:') and '\n' not in plan_text:
            # Single line format - extract the action part after 'plan:'
            action_part = plan_text[5:].strip()  # Remove 'plan:' and strip
            if action_part:
                # Parse as single action
                parsed_step = _parse_action_line(action_part, 1)
                return {"plan": [parsed_step]}
            else:
                return {"plan": []}  # Empty plan
        
        # Multi-line format
        lines = plan_text.split('\n')
        if not lines[0].strip() == 'plan:':
            raise ValueError("Plan must start with 'plan:'")
        
        plan_steps = []
        current_block = None
        current_block_steps = []
        
        for i, line in enumerate(lines[1:], 1):  # Skip 'plan:' line
            stripped_line = line.strip()
            if not stripped_line:
                continue
                
            # Calculate indentation (assuming 2 spaces per level)
            current_indent = len(line) - len(line.lstrip())
            
            if stripped_line == 'do:':
                # Start of do-while block
                current_block = 'do_while'
                current_block_steps = []
                
            elif stripped_line.startswith('while(') and stripped_line.endswith(')'):
                # End of do-while block
                if current_block != 'do_while':
                    raise ValueError(f"Line {i}: 'while' without matching 'do'")
                
                condition_text = stripped_line[6:-1]  # Extract condition from while(...)
                # Parse condition as an action
                condition_action = _parse_action_line(condition_text, i)
                plan_steps.append({
                    'type': 'do_while',
                    'body': current_block_steps,
                    'condition': condition_action
                })
                current_block = None
                current_block_steps = []
                
            elif stripped_line == 'if:':
                # Start of if block
                current_block = 'if'
                current_block_steps = []
                if_condition = None  # Store the condition separately
                if_step = None  # Initialize if_step
                
            elif stripped_line == 'else:':
                # Switch to else block
                if current_block != 'if':
                    raise ValueError(f"Line {i}: 'else' without matching 'if'")
                if if_condition is None:
                    raise ValueError(f"Line {i}: 'else' without condition in if block")
                
                # Create the if step with then part
                if_step = {
                    'type': 'if',
                    'condition': if_condition,
                    'then': current_block_steps
                }
                
                current_block = 'else'
                current_block_steps = []
                
            elif stripped_line == 'endif:':
                # End of if block
                if current_block not in ['if', 'else']:
                    raise ValueError(f"Line {i}: 'endif' without matching 'if'")
                
                if current_block == 'else':
                    # Add the else steps to the existing if step
                    if_step['else'] = current_block_steps
                    plan_steps.append(if_step)
                else:
                    # No else block, create and add the if step
                    if if_condition is None:
                        raise ValueError(f"Line {i}: 'endif' without condition in if block")
                    if_step = {
                        'type': 'if',
                        'condition': if_condition,
                        'then': current_block_steps
                    }
                    plan_steps.append(if_step)
                
                current_block = None
                current_block_steps = []
                if_condition = None
                if_step = None
                
            else:
                # Regular action or condition
                if current_block == 'if' and if_condition is None:
                    # First line in if block is the condition
                    if_condition = _parse_action_line(stripped_line, i)
                else:
                    # Regular action
                    parsed_step = _parse_action_line(stripped_line, i)
                    
                    if current_block:
                        # Add to current block
                        current_block_steps.append(parsed_step)
                    else:
                        # Add to main plan
                        plan_steps.append(parsed_step)
        
        # Check for unmatched blocks
        if current_block == 'do_while':
            raise ValueError("Unmatched 'do:' block without 'while(...)'")
        elif current_block in ['if', 'else']:
            raise ValueError("Unmatched 'if:' block without 'endif'")
        
        return {"plan": plan_steps}

def _parse_action_line(line, line_number):
        """Parse a single action line into step dictionary."""
        # Check if it's a regular action
        if '(' in line and line.endswith(')'):
            # Parse action(target, value) or action(target)
            paren_pos = line.find('(')
            action = line[:paren_pos].strip()
            args_str = line[paren_pos+1:-1]
            
            # Handle special case for 'say' action with colon syntax
            if action == 'say' and ':' in args_str:
                # say(Joe: Hello, how are you, my friend?)
                colon_pos = args_str.find(':')
                target = args_str[:colon_pos].strip()
                value = args_str[colon_pos+1:].strip()
            else:
                # Split arguments - handle commas in quoted strings
                args = []
                if args_str.strip():
                    # Simple split for now - could be enhanced for quoted strings later
                    raw_args = [arg.strip() for arg in args_str.split(',')]
                    args = raw_args
                
                # Determine target and value
                target = args[0] if len(args) > 0 and action != 'think' else ''
                value = args[1] if len(args) > 1 and action != 'think' else ''
                value = args[0] if len(args) > 0 and action == 'think' else ''
            
            return {
                'type': action,
                'target': target,
                'value': value
            }
        else:
            raise ValueError(f"Line {line_number}: Invalid action format '{line}'")

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
        return _validate_top(plan_dict)
    except Exception:
        # Any raised error means invalid
        return False

# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

_ALLOWED_TYPES = {"action", "say", "move", "think", "take", "inspect", "use", "do_while", "if", "near", "look"}
_ALLOWED_CONDITION_TYPES = {"near", "can_see", "has_item", "notnear", "cant_see", "hasnt_item", "at_location", "notat_location", "believes", "notbelieves"}

REQ_KEYS = {
    "say": {"type", "target", "value"},
    "move": {"type", "target"},
    "think": {"type", "value"},
    "look": {"type", "target"},
    "take": {"type", "target"},
    "inspect": {"type", "target"},
    "use": {"type", "target"},
    "do_while": {"type", "body", "condition"},
    "if": {"type", "condition", "then"},          # "else" is optional
    "near": {"type", "target"},
}

OPTIONAL_KEYS = {
    "say": {"reason", "value"},
    "move": {"reason", "value"},
    "think": {"reason", "target"},
    "look": {"reason", "value"},
    "take": {"reason", "value"},
    "inspect": {"reason", "value"},
    "use": {"reason", "value"},
    "do_while": {"reason", "value"},
    "if": {"else"},
    "near": {"reason", "value"},
    "look": {"reason", "value"},
}

REQ_CONDITION_KEYS = {
    "near": {"target"},
    "can_see": {"target"},
    "has_item": {"target"},
    "at_location": {"target"},
    "believes": {"target"},
    "notnear": {"target"},
    "cant_see": {"target"},
    "hasnt_item": {"target"},
    "notat_location": {"target"},
    "notbelieves": {"target"},
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
        return _validate_steps(plan['plan'])
    if isinstance(plan, list):
        return _validate_steps(plan)
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
        return False

    # --- key checks --------------------------------------------------------
    req = REQ_KEYS[step_type]
    opt = OPTIONAL_KEYS[step_type]
    keys = set(step.keys())
    if not req.issubset(keys):
        return False
    if not keys.issubset(req | opt):
        return False

    # --- per‑type structural checks ----------------------------------------

    if step_type == "do_while":
        return (
            isinstance(step["condition"], dict)
            and _validate_condition(step["condition"])
            and isinstance(step["body"], list)
            and _validate_steps(step["body"])
        )

    if step_type == "if":
        if not (isinstance(step["condition"], dict)) or not _validate_condition(step["condition"]) or not isinstance(step["then"], list):
            return False
        if "else" in step and not isinstance(step["else"], list):
            return False
        return _validate_steps(step["then"]) and _validate_steps(step.get("else", []))

    if isinstance(step, list):
        return all(_validate_steps(element) for element in step)

    return True  # otherwise all ok

def _validate_condition(condition: Any) -> bool:
    if not isinstance(condition, dict):
        return False
    if condition.get("type", None) not in _ALLOWED_CONDITION_TYPES:
        return False
    keys = set(condition.keys())
    if not REQ_CONDITION_KEYS[condition.get("type", None)].issubset(keys):
        return False
    return True

def is_near(character, target: str) -> bool:
    """Check if a target (resource or character) is near this character."""
    try:
        character_name = character.character_name
        for reply in character.session.get(f"cognitive/{character_name}/situation/proximity?target={target}", timeout=60.0):
            if reply.ok:
                data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                if data['success']:
                    return data['value']
            break
    except Exception as e:
        logger.error(f'Error checking proximity for {target}: {e}')
    return False

def _evaluate_condition(character: ZenohExecutiveNode, condition: dict) -> bool:
        """Evaluate a condition action dict using distributed node queries."""
        # The key to conditions is access to the character's beliefs and knowledge base.
        # Since this is distributed among nodes, we will used targeted queires by predicate.
        if not condition:
            logger.error('No condition provided {condition}')
            return False
        if not isinstance(condition, dict) or 'type' not in condition or 'target' not in condition:
            logger.error(f'Invalid condition: {condition}')
            return False
        
        condition_type = condition['type']
        target = condition['target']
        character_name = character.character_name
        
        try:
            # Query the appropriate node based on condition type
            if condition_type in ['near', 'notnear']:
                # Use the extracted is_near function
                result = is_near(character, target)
                return not result if condition_type == 'notnear' else result
                    
            elif condition_type in ['can_see', 'cant_see']:
                # Query situation_node for visibility
                for reply in character.session.get(f"cognitive/{character_name}/situation/visibility?target={target}", timeout=60.0):
                    if reply.ok:
                        data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                        if data['success']:
                            result = data['value']
                            logger.info(f'Visibility query for {target}: {result}')
                            return not result if condition_type == 'cant_see' else result
                    break
                    
            elif condition_type in ['has_item', 'hasnt_item']:
                # Query memory_node for inventory
                for reply in character.session.get(f"cognitive/{character_name}/memory/inventory?item={target}", timeout=60.0):
                    if reply.ok:
                        data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                        if data['success']:
                            result = data['value']
                            logger.info(f'Inventory query for {target}: {result}')
                            return not result if condition_type == 'hasnt_item' else result
                    break
                    
            elif condition_type in ['at_location', 'notat_location']:
                # Query situation_node for location
                for reply in character.session.get(f"cognitive/{character_name}/situation/location?target={target}", timeout=60.0):
                    if reply.ok:
                        data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                        if data['success']:
                            result = data['value']
                            logger.info(f'Location query for {target}: {result}')
                            return not result if condition_type == 'notat_location' else result
                    break
                    
            elif condition_type in ['believes', 'notbelieves']:
                # Stub implementation - user will implement this
                logger.warning(f'Belief condition {condition_type}({target}) not implemented - returning False')
                return False
            else:
                logger.error(f'Unknown condition type: {condition_type}')
                return False
                
        except Exception as e:
            logger.error(f'Error evaluating condition {condition_type}({target}): {e}')
            return False
        
        # If we get here, the query failed or returned no response
        logger.warning(f'Condition evaluation failed for {condition_type}({target}) - returning False')
        return False
