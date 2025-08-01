#!/usr/bin/env python3
"""
Zenoh Executive Node

This node implements the OODA loop for character decision-making and action execution.
Replaces ROS2 complexity with simple Zenoh pub/sub.
"""

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
from typing import Dict, List, Any, Union
import utils.hash_utils as hash_utils
import plan as plan_module

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

# Import LLM client
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__))))

try:
    from llm_client import ZenohLLMClient
    LLM_CLIENT_AVAILABLE = True
except ImportError as e:
    print(f"⚠️  LLM Client not available: {e}")
    LLM_CLIENT_AVAILABLE = False


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
            f"cognitive/{character_name}/situation/current_situation",
            self.situation_callback
        )
        
        # Publisher for actions (character-specific)
        self.action_publisher = self.session.declare_publisher(f"cognitive/{character_name}/action")
        
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
        
        # LLM client
        self.llm_client = None
        if LLM_CLIENT_AVAILABLE:
            self.llm_client = ZenohLLMClient(service_timeout=30.0)
        
        # Internal state
        self.action_counter = 0
        self.last_sense_data = None
        self.last_situation_data = None
        
        # OODA loop state
        self.current_goal = None
        self.observations = None
        self.interrupt_pending = False
        self.text_input_pending = False
        self.last_action = None
        
        # Plan execution state
        self.current_plan = None
        self.plan_state = {
            'step_stack': plan_module.Stack()
        }
        self.plan_bindings_cache = {}
        # Turn management
        self.turn_subscriber = self.session.declare_subscriber(
            "cognitive/map/turn/go",
            self.turn_callback
        )
        self.turn_complete_publisher = self.session.declare_publisher(
            f"cognitive/map/turn/complete/{character_name}"
        )
        self.waiting_for_turn = True
        self.current_turn_number = 0
        
        # Subscriber for shutdown commands (global)
        self.shutdown_subscriber = self.session.declare_subscriber(
            "cognitive/shutdown/executive",
            self.shutdown_callback
        )
        
        # Shutdown flag
        self.shutdown_requested = False
        
        # Register signal handlers for graceful shutdown
        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)
        
        logger.info(f'🧠 Zenoh Executive Node initialized for character: {character_name}')
        logger.info(f'   - Subscribing to: cognitive/{character_name}/sense_data')
        logger.info(f'   - Subscribing to: cognitive/{character_name}/situation')
        logger.info(f'   - Subscribing to: cognitive/map/turn/go')
        logger.info(f'   - Publishing to: cognitive/{character_name}/action')
        logger.info(f'   - Publishing to: cognitive/{character_name}/memory/store')
        logger.info(f'   - Publishing to: cognitive/{character_name}/text_input')
        logger.info(f'   - Publishing to: cognitive/{character_name}/goal')
        logger.info(f'   - Publishing to: cognitive/{character_name}/decided_action')
        logger.info(f'   - Publishing to: cognitive/map/turn/complete/{character_name}')
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully."""
        logger.warning(f'Received signal {signum}, initiating shutdown...')
        self.shutdown_requested = True
    
    def run(self):
        """Main OODA loop."""
        try:
            logger.warning('Executive Node running - press Ctrl+C to stop')
            
            # Announce character presence
            self._announce_character()
            
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
                            if source == 'ui': # other text inputs must be handled by OODA to observe dialog turn taking
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
            
            logger.warning(f'📤 {self.character_name} Sent text input to {target_character}: "{message}" (source: {self.character_name})')
            
        except Exception as e:
            logger.error(f'Error sending text input to {target_character}: {e}')

    def _publish_goal(self, goal):
        """Publish current goal to the goal topic for UI display."""
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
                'action': action['type'],
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

    def _run_ooda_loop(self):
        """Execute the OODA loop: Observe, Orient, Decide, Act."""
        try:
            logger.warning(f'🔄 {self.character_name} OODA loop running')
            if self.text_input_pending and (not self.last_action or (self.last_action['type'].lower() != 'response' and self.last_action['type'].lower() != 'say')):
                self.text_input_pending = False
                content = self.last_sense_data['content']
                try:
                    content_data = json.loads(content)
                    text_input = content_data.get('text', '')
                    source = content_data.get('source', 'unknown')
                except (json.JSONDecodeError, TypeError):
                    # Fallback to plain text (console input format)
                    text_input = content
                    source = 'console'
                responded = self.generate_speech(text_input, source, mode='respond')
                if responded:
                    return
            if self.interrupt_pending:
                self.interrupt_pending = False
                pass
            # Observe: Collect current situation and sense data
            observations = self._observe()
            
            # Orient: Assess current state and goals
            if self.interrupt_pending:
                self.interrupt_pending = False
                if self._handle_interrupt():
                    return
            goal = self._orient(observations)
            
            # Plan: Return existing plan or create single-action plan
            if self.interrupt_pending:
                self.interrupt_pending = False
                if self._handle_interrupt():
                    return
            plan = self._plan(goal)
            
            # Plan Step: Execute current step of plan
            if self.interrupt_pending:
                self.interrupt_pending = False
                if self._handle_interrupt():
                    return
            action = self._plan_step(plan)
            
            # Act: Execute the chosen action (if we have one)
            if action is not None:
                if self.interrupt_pending:
                    self.interrupt_pending = False
                    if self._handle_interrupt():
                        return
                self._act(action)
            
        except Exception as e:
            logger.error(f'Error in OODA loop: {e}')
            logger.error(traceback.format_exc())

    def format_situation(self):
        """Format the situation data for the LLM."""
        formatted_situation = ''
        if self.last_situation_data and self.last_situation_data.get('location'):
            formatted_situation += f"You are at location: {self.last_situation_data['location']}\n"
        if self.last_situation_data and self.last_situation_data.get('visible_characters'):
            formatted_situation += f"You notice the following characters: {', '.join(self.last_situation_data['visible_characters'])}\n"
        if self.last_situation_data and self.last_situation_data.get('look'):
            formatted_situation += f"You can see the following:\n\t{'\n\t'.join(self.last_situation_data['look'])}\n"
        
        # Add adjacent information
        if self.last_situation_data and self.last_situation_data.get('adjacent_to'):
            adjacent = self.last_situation_data['adjacent_to']
            if adjacent.get('resources'):
                formatted_situation += f"You are adjacent to these resources (available to take, inspect, or use): {', '.join(adjacent['resources'])}\n"
            if adjacent.get('characters'):
                formatted_situation += f"You are adjacent to these characters (available to interact with): {', '.join(adjacent['characters'])}\n"
        
        return formatted_situation

    def _observe(self):
        """Observe: Collect current situation and sense data. Stub."""
        if self.character_config.get('character', None):
            system_prompt = self.character_config['character']
        if self.character_config.get('drives', None):
            system_prompt += f"\n#Your drives are:\n\t{'\n\t'.join(self.character_config['drives'])}\n"
            
        # Build user prompt with context
        user_prompt = '' 
        user_prompt += self.format_situation()
        entity_context = None
        entity_context = self.get_entity_context(self.character_name, 10)
        if entity_context:
            user_prompt += f'\n#Your most recent thoughts include:'
            for i, memory in enumerate(entity_context['conversation_history']):  # Use last 2 memories
                user_prompt += f"\n\t{memory['source']}: {memory['text']}"
            user_prompt += '\n'
        self.observations = {'static': system_prompt, 'dynamic': user_prompt}
        return self.observations

    def _orient(self, observations: Dict[str, Any]):
        """Orient: Assess current state and drives"""
        """{'static': system_prompt, 'dynamic': user_prompt}"""
        if self.current_goal:
            return self.current_goal
        else:
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
"""
            # Make LLM call
            if self.llm_client and not self.shutdown_requested:
                # Use shorter timeout during shutdown
                timeout = 5.0 if self.shutdown_requested else None
                response = self.llm_client.generate(
                    messages=[system_prompt, user_prompt, directive],
                    max_tokens=100,
                    temperature=0.7,
                    timeout=timeout
                )

                if response.success:
                    logger.warning(f'🤖 {self.character_name} New Goal: {response.text.strip()}')
                    goals = []
                    forms = hash_utils.findall_forms(response.text)
                    for goal_hash in forms:
                        goal = plan_module.validate_and_create_goal(self.character_name, goal_hash)
                        if goal:
                            logger.warning(f'{self.character_name} generated goal: {goal.to_string()}')
                            self.current_goal = goal
                            self._publish_goal(goal)
                            return self.current_goal
                        else:
                            logger.error(f'Warning: Invalid goal generation response for {goal_hash}')
                else:
                    logger.error(f'LLM call failed: {response.error}')
                    self.current_goal = plan_module.Goal('sleep', actors=[self.character_name])
            else:   
                logger.error('LLM client not available')
                self.current_goal = plan_module.Goal('sleep', actors=[self.character_name])
                self._publish_goal(self.current_goal)
            return self.current_goal


    def _plan(self, goal):
        """Plan: Return existing plan or create single-action plan from goal."""
        # If we already have a plan, return it
        if self.current_plan is not None:
            return self.current_plan
        
        # No existing plan - create single-action plan using existing _decide logic
        plan_syntax = """
Task: Break down the user’s high‑level goal into a minimal plan in the JSON format specified below.
Output: only valid JSON – no prose, no code fences.

{
  "plan": [
    { "type": "move", "target": "…"},
    { "type": "say", "target": "…", "value": "…" },
    { "type": "think", "value": "…" },
    { "type": "take", "target": "…" },
    { "type": "inspect", "target": "…" },
    { "type": "use", "target": "…" },
    { "type": "do_while", "body": [ /* steps */ ], "condition": "…" },
    { "type": "if", "condition": "…", "then": [ /* steps */ ], "else": [ /* steps */ ] }
  ]
}

A plan must include no more than 10 steps including all nested do_while and if branches.
In the following, <resource_name>, <character_name> are placeholders only for KNOWN resources or characters, those appearing above.
Only dicts of the types below are allowed for the condition of do_while and if. Condition action type can only be one of the following:
 - "near": {"type": "near", "target": <resource name? or <character_name>} is for checking if the character is near a resource or character.
 - "can_see": {"type": "can_see", "target": <character_name>} is for checking if the character can see a character.
 - "has_item": {"type": "has_item", "target": <resource_name>} is for checking if the character has a resource in their inventory.
 - "at_location": {"type": "at_location", "target": <location_name>} is for checking if the character is at a location.
 - "believes": {"type": "believes", "target": <character_name>} is for checking if the character believes something about another character.
 - "notnear": {"type": "notnear", "target": <resource name? or <character_name>} is for checking if the character is not near a resource or character.
 - "cant_see": {"type": "cant_see", "target": <character_name>} is for checking if the character cannot see a character.
 - "hasnt_item": {"type": "hasnt_item", "target": <resource_name>} is for checking if the character does not have a resource in their inventory.
 - "notat_location": {"type": "notat_location", "target": <resource_name>} is for checking if the character is not at a location.
 - "notbelieves": {"type": "notbelieves", "target": <character_name>} is for checking if the character does not believe something about another character.

outside a do_while or if condition, "type" can take the values "say", "move", "think", "take", "inspect", or "use":
 - "say": { "type": "say", "target": "character_name", "value": "text to speak" } is for speaking to another character you can see. For a 'say' act, speak only for yourself, and do not include any other introductory, explanatory, discursive, or formatting text in your response.
 - "move": { "type": "move", "target": "cardinal_direction" or 'resource or character name'} is for moving in one of the 8 cardinal directions or in the direction of a resource or character.
 - "think": { "type": "think", "value": "text to think about" } is for thinking about a topic or question, attempting to derive new information, conclusions, or decisions from who you are and what you already explicitly know
 - "take": { "type": "take", "target": "resource_name" } is for adding some resource you see to your personal inventory. you must be 'near' the resource to take it.
 - "inspect": { "type": "inspect", "target": "resource_name" } is for inspecting a resource you see or one in your inventory to understand how to use it.
 - "use": { "type": "use", "target": "resource_name" } is for using a resource in a known way.

In general, a target can be a specific resource_name or character_name or a reasonable generalization thereof. 
For example, Berry2, Berry, Berries, Fruit, Food, etc are all valid targets. 
Likewise, Joe, character, person, etc are all valid targets.
However, in these latter cases, the specific resource or character bound to is indeterminate. 
For move, target can also be one of the 8 compass points.
For example, 
    {"type": "if", "condition": {"type": "near", "target": "Berry"}, "then": [{"type": "move", "target": "Berry"}]}
is a valid plan. Likewise,
    {"type": "if", "condition": {"type": "near", "target": "Joe"}, "then": [{"type": "move", "target": "Joe"}]}
is a valid plan.
Note that move only moves one step, so you must use a do_while to move repeatedly.

Some actions have conditions that must be met before they can be executed.
for example, you cannot take a resource unless you are near it.
you can accomplish this by using the "near" condition in a do_while. Assuming, for example, that Resource2 is in your situation view direction Northeast

{
  "plan": [
    { "type": "do_while", "body": [ { "type": "move", "target": "Northeast" } ], "condition": { "type": "notnear", "target": "Resource2" } },
    { "type": "take", "target": "Resource2" }
  ]
}

###
Allowed control‑flow primitives: sequential list (e.g.. [..., ...]), do_while, and two‑branch if (else is optional).
{
  "plan": [
    { "type": "action", "target": "…", "value": "…" },
    { "type": "do_while", "body": [ /* steps */ ], "condition": "…" },
    { "type": "if", "condition": "…", "then": [ /* steps */ ], "else": [ /* steps */ ] }
  ]
}

A plan must include no more than 10 steps including all nested do_while and if branches.
        """
        if not self.current_goal or self.current_goal.name == 'sleep':
            single_action = None
        else:
            system_prompt = self.observations['static']
            user_prompt = self.observations['dynamic']
            goal_prompt = f"\n\nYour current goal is: {goal.to_string()}"
            target = goal.actors[1] if len(goal.actors) > 1 else None
            if target:
                entity_context = self.get_entity_context(target, 10)
                if entity_context and len(entity_context['conversation_history']) > 0:
                    goal_prompt += f'your recent dialog with {target} has been:\n'
                    for i, memory in enumerate(entity_context['conversation_history']):  # Use last 2 memories
                        goal_prompt += f"\t{memory['source']}: {memory['text']}\n"

                    goal_prompt += "Don't repeat yourself.\n"

            if self.last_action and (self.last_action['type'].lower() == 'say' or self.last_action['type'].lower() == 'response'):
                directive = plan_syntax
            else:
                directive = plan_syntax
            # Make LLM call
            if self.llm_client and not self.shutdown_requested:
                response = self.llm_client.generate(
                    messages=[system_prompt, user_prompt, plan_syntax, directive],
                    max_tokens=1000,
                    temperature=0.7,
                    stops=['</end>']
                )

                if response.success:
                    logger.warning(f'🤖 {self.character_name} New Action: {response.text.replace('\n', ' ')}')
                    plan_candidate = None
                    valid = False
                    try:
                        plan_candidate = json.loads(response.text.strip())
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
            self.plan_bindings_cache = {}
        else:
            self.current_plan = {'plan': []}
        
        self._publish_current_plan()
        
        # Initialize plan state for new plan
        self.plan_state = {
            'step_stack': plan_module.Stack()
        }
        
        # Initialize plan bindings cache for target resolution
        self.plan_bindings_cache = {}
        
        return self.current_plan

    def _plan_step(self, plan):
        """Execute current step of plan and return next action using step counter stack."""
        # Extract plan steps from dict format
        if isinstance(plan, dict) and 'plan' in plan:
            plan_steps = plan['plan']
        else:
            plan_steps = plan
            
        if not plan_steps or len(plan_steps) == 0:
            return None
        
        step_stack = self.plan_state['step_stack']
        
        # Initialize stack if empty
        if step_stack.is_empty():
            step_stack.push(0)  # Start at step 0
        
        # Execute next step
        try:
            action = self._execute_next_step(plan_steps, step_stack)
            if action:
                self.current_action = action
                self._publish_decided_action(action)
            return action
        except Exception as e:
            logger.error(f'Error in plan execution: {e}')
            logger.error(traceback.print_exc())
            # Clear plan state on error
            self.current_plan = None
            self.plan_bindings_cache = {}
            self._publish_current_plan()
            self.plan_state = {
                'step_stack': plan_module.Stack()
            }
            # Clear plan bindings cache
            self.plan_bindings_cache = {}
            return None
    
    def _execute_next_step(self, plan, step_stack):
        """Execute next step using simple step counter stack."""
        if step_stack.is_empty():
            return None
        
        current_step = step_stack.peek()
        
        # Check if we've completed the current level
        if current_step >= len(plan):
            step_stack.pop()  # Exit this level
            if step_stack.is_empty():
                # Plan complete
                self.current_plan = None
                self.plan_bindings_cache = {}
                self._publish_current_plan()
                self.plan_state = None
                # Clear plan bindings cache
                self.plan_bindings_cache = {}
                return None
            else:
                # Continue at parent level
                return self._execute_next_step(plan, step_stack)
        
        step = plan[current_step]
        
        # Handle different step types
        if step['type'] in ['move', 'say', 'think', 'take', 'inspect', 'use', 'near', 'look']:
            # Execute action and advance
            step_stack.pop()  # Remove current step
            step_stack.push(current_step + 1)  # Push next step
            return step
        
        elif step['type'] == 'do_while':
            # Handle do-while loop
            condition_action = step.get('condition', None)
            condition_action = self._resolve_target(condition_action)
            if plan_module._evaluate_condition(self, condition_action):
                # Condition true, enter do body
                step_stack.push(0)  # Start at first action in do body
                return self._execute_next_step(step['body'], step_stack)
            else:
                # Condition false, skip do-while
                step_stack.pop()  # Remove current step
                step_stack.push(current_step + 1)  # Push next step
                return self._execute_next_step(plan, step_stack)
        
        elif step['type'] == 'if':
            # Handle if-then-else
            condition_action = step.get('condition', None)
            condition_action = self._resolve_target(condition_action)
            if plan_module._evaluate_condition(self, condition_action):
                # Condition true, execute then branch
                step_stack.push(0)  # Start at first action in then
                return self._execute_next_step(step['then'], step_stack)
            else:
                # Condition false, execute else branch if it exists
                if 'else' in step and step['else']:
                    step_stack.push(0)  # Start at first action in else
                    return self._execute_next_step(step['else'], step_stack)
                else:
                    # No else branch, skip if
                    step_stack.pop()  # Remove current step
                    step_stack.push(current_step + 1)  # Push next step
                    return self._execute_next_step(plan, step_stack)
        
        else:
            # Unknown step type, skip it
            logger.warning(f'Unknown plan step type: {step["type"]}')
            step_stack.pop()  # Remove current step
            step_stack.push(current_step + 1)  # Push next step
            return self._execute_next_step(plan, step_stack)
    

    def _act(self, action: Dict[str, Any]):
        """Act: Execute the chosen action."""

        action = self.current_action if self.current_action else action
        if action['type'].lower() == "sleep":
            action_data = {'type': 'sleep','action_id': self.action_counter,'timestamp': datetime.now().isoformat(),'target': self.character_name}
            self.last_action = action_data
            self.action_publisher.put(json.dumps(action_data))
            time.sleep(1)  # Sleep for 1 second
            return
        if action['type'].lower() == 'think':
            self.think_about(action['value'])
            return
            
        action = self._resolve_target(action)
        if action['type'].lower() == "move":
            move_target = action['target'].strip()
            move_direction = move_target.lower()
            
            # Step 1: Test for compass points first (case insensitive)
            cardinal_directions = ['north', 'northeast', 'southeast', 'south', 'southwest', 'northwest', 'east', 'west']
            if move_direction in cardinal_directions:
                self.move(move_direction)
                return
            else:
                logger.error(f'❌ Cannot move toward "{move_target}" - target not resolved or visible')
                return
        elif action['type'].lower() == "say":
            self.generate_speech(action['value'], action['target'], mode='say')            # Publish action (this will be picked up by action_display_node)

        elif action['type'].lower() == "think":
            self.think_about(action['value'])
        elif action['type'].lower() == "take":
            self.take(action['target'])
            self.action_counter += 1
        elif action['type'].lower() == "inspect":
            action_data = {'type': 'inspect','action_id': self.action_counter,'timestamp': datetime.now().isoformat(),'target': action['target']}
            self.last_action = action_data
            self.action_publisher.put(json.dumps(action_data))
            #self.inspect(action)
        elif action['type'].lower() == "use":
            # Create action - noop for now
            action_data = {'type': 'use','action_id': self.action_counter,'timestamp': datetime.now().isoformat(),'target': action['target']}
            self.last_action = action_data
            self.action_publisher.put(json.dumps(action_data))
            #self.use(action)
        logger.warning(f'📤 Published action: {action["type"]}')
        self.action_counter += 1

    def _handle_interrupt(self):
        """Handle interrupt from sense data or situation updates."""
        logger.info('🔄 Handling interrupt')
        # TODO: Implement interrupt handling logic
        pass
    
    def _complete_turn(self):
        """Complete the current turn and wait for next turn."""
        try:
            # Publish turn completion
            completion_data = {
                'character_name': self.character_name,
                'turn_number': self.current_turn_number,
                'timestamp': time.time()
            }
            self.turn_complete_publisher.put(json.dumps(completion_data).encode('utf-8'))
            
            logger.warning(f'✅ {self.character_name} completed turn {self.current_turn_number}')
            
            # Wait for next turn
            self.waiting_for_turn = True
            
        except Exception as e:
            logger.error(f'Error completing turn: {e}')
    
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
                logger.warning(f'📥 {self.character_name} Received text input: "{text_input}" (source: {source})')
                
        except Exception as e:
            logger.error(f'Error processing sense data: {e}')
    
    def situation_callback(self, sample):
        """Handle incoming situation data."""
        try:
            situation_data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            logger.warning(f'📊 {self.character_name} Received situation update')
            
            # Store situation data for potential use in LLM processing
            self.last_situation_data = situation_data
            
            # Trigger interrupt for OODA loop
            self.interrupt_pending = True
            
        except Exception as e:
            logger.error(f'Error processing situation data: {e}')
    
    def turn_callback(self, sample):
        """Handle turn "GO" signals."""
        try:
            data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            turn_number = data.get('turn_number', 0)
            active_characters = data.get('active_characters', [])
            
            if self.character_name in active_characters:
                self.current_turn_number = turn_number
                self.waiting_for_turn = False
                logger.warning(f'🚦 Turn {turn_number} started for {self.character_name}')
                
                # Set interrupt flag to start OODA loop
                self.interrupt_pending = True
            else:
                logger.debug(f'Turn {turn_number} started but {self.character_name} not included')
            
        except Exception as e:
            logger.error(f'Error handling turn signal: {e}')
    
    def shutdown_callback(self, sample):
        """Handle shutdown command from UI."""
        try:
            logger.warning(f'🔌 {self.character_name} Executive Node received shutdown command')
            self.shutdown_requested = True
        except Exception as e:
            logger.error(f'Error in shutdown callback: {e}')
    
    def parse_and_set_plan(self, plan_text):
        """Parse plan input from UI and set current plan."""
        try:
            parsed_plan = plan_module.parse_plan_text(plan_text)
            self.current_plan = parsed_plan
            self.plan_bindings_cache = {}
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
        if source == 'ui' and clean_input.startswith('plan:'):
            self.parse_and_set_plan(clean_input)
            return
        
        try:
            if mode == 'respond':
                logger.info(f'Responding to: "{text_input}" from {source}')
            else:
                logger.info(f'Saying intent: "{text_input}" to {source}')
            
            # Check if dialog should naturally end
            if mode == 'respond':
                are_we_done = self.check_natural_dialog_end(source, text_input)
                logger.warning(f'🤖 {self.character_name} Dialog end check: {are_we_done}')
                if are_we_done:
                    action_data = {'type': 'dialog_end','action_id': f'action_{self.action_counter}','timestamp': datetime.now().isoformat(),'input': text_input,'text': 'Done','source': source}
                    self.last_action = action_data
                    self.action_publisher.put(json.dumps(action_data))
                    logger.info(f'📤 Published action: {action_data["action_id"]}')
                    return False
            
            # Get recent memory entries for context
            recent_memories = self._get_recent_chat_memories(3)
            # Get entity context if source is not console
            entity_context = self.get_entity_context(source, 10)
            
            # Simple, focused prompt
            if mode == 'say':
                system_prompt = f"""Your task is to say the following:
    "{text_input}" 
updated to reflect the current context described below.

you are:
"""
            else:
                system_prompt = f"""Your task is to respond to:
    "{text_input}"
in the current context described below.

you are:
"""
            if self.character_config.get('character', None):
                system_prompt += self.character_config['character']
            if self.character_config.get('drives', None):
                system_prompt += f"\n\nYour drives are:\n\t{'\n\t'.join(self.character_config['drives'])}\n"
            
            # Build user prompt with context
            user_prompt = '' 
            user_prompt += self.format_situation()
            if entity_context and isinstance(entity_context, dict):
                conversation_history = entity_context.get('conversation_history', [])
                if isinstance(conversation_history, list):
                    user_prompt += f"Your recent conversation with {source} has been:\n"
                    for i, memory in enumerate(conversation_history):
                        if isinstance(memory, dict) and 'source' in memory and 'text' in memory:
                            user_prompt += f"{memory['source']}: {memory['text']}\n"
            user_prompt +=  """Speak in a conversational manner in your own voice.
Do not include any other introductory, explanatory, discursive, or formatting text in your response.
End your text with: </end>"""
                    
            # Make LLM call
            if self.llm_client and not self.shutdown_requested:
                # Use shorter timeout during shutdown
                timeout = 5.0 if self.shutdown_requested else None
                response = self.llm_client.generate(
                    messages=[system_prompt, user_prompt],
                    max_tokens=400,
                    temperature=0.7,
                    timeout=timeout,
                    stops=['</end>']
                )

                if response.success:
                    text_to_send = response.text.strip()
                    if not text_to_send.lower().startswith('done'):
                        self.send_text_input(source, text_to_send)
                        logger.warning(f'Responding to: "{text_input}" from {source}: {text_to_send}')

                    # Create action
                    action_data = {
                        'type': 'say' if mode == 'say' else 'response',
                        'action_id': f'action_{self.action_counter}',
                        'timestamp': datetime.now().isoformat(),
                        'input': text_input,
                        'text': response.text.strip(),
                        'source': self.character_name if mode == 'say' else source,
                        'target': source if mode == 'say' else None
                     }
                    self.last_action = action_data
                    # Publish action (this will be picked up by memory_node and action_display_node)
                    self.action_publisher.put(json.dumps(action_data))
                    logger.info(f'📤 Published action: {action_data["action_id"]}')
                    
                    self.action_counter += 1
                else:
                    logger.error(f'LLM call failed: {response.error}')
            else:
                logger.error('LLM client not available')
                
        except Exception as e:
            logger.error(f'Error in LLM processing: {e}')
            logger.error(traceback.format_exc())
        return True

    
    def _resolve_target(self, action: Dict[str, Any]) -> Union[str, bool]:
        """
        Resolve abstract target name to specific instance reference using plan bindings cache.
        This ONLY handles abstract -> specific resolution (e.g., "Berry" -> "Berry23").
        
        Args:
            action: Action dictionary containing 'type' and 'target', or condition dictionary
            
        Returns:
            Resolved target name (str) or False if resolution fails
        """
        action_type = action.get('type', 'unknown')
        raw_target = action.get('target', None)
        
        if not raw_target:
            return action
            
        # Check plan bindings cache first
        cache_key = raw_target
        if cache_key in self.plan_bindings_cache:
            cached_result = self.plan_bindings_cache[cache_key]
            logger.debug(f'🎯 Cache hit: {raw_target} -> {cached_result}')
            action_copy = action.copy()
            action_copy['target'] = cached_result
            return action_copy
        
        logger.debug(f'🔍 Resolving target "{raw_target}" for action type "{action_type}"')
        resolved_target = False
        
        try:
            # Context-aware resolution based on action type
            if action['type'].lower() == "move":
                move_target = action['target'].strip()
                move_direction = move_target.lower()
                # Step 1: Test for compass points first (case insensitive)
                cardinal_directions = ['north', 'northeast', 'southeast', 'south', 'southwest', 'northwest', 'east', 'west']
                if move_direction in cardinal_directions:
                    return action
                else:
                    resolved_target = self._resolve_resource_instance(raw_target) or self._resolve_character_instance(raw_target)
                    if not resolved_target:
                        return action
                    self.plan_bindings_cache[cache_key] = resolved_target
                    move_direction = self._find_target_direction(resolved_target)
                    if move_direction:
                        action_copy = action.copy()
                        action_copy['target'] = move_direction
                        return action_copy
                    else:
                        return action
            if action_type in ['take', 'inspect', 'use', 'has_item', 'hasnt_item']:
                # Resource actions/conditions - resolve to specific resource instance
                resolved_target = self._resolve_resource_instance(raw_target) or self._resolve_character_instance(raw_target)
                action_copy = action.copy()
                action_copy['target'] = resolved_target
                return action_copy
                
            elif action_type in ['say', 'can_see', 'cant_see']:
                # Character actions/conditions - resolve to specific character name
                resolved_target = self._resolve_character_instance(raw_target)
                action_copy = action.copy()
                action_copy['target'] = resolved_target
                return action_copy
                
            elif action_type in ['near', 'notnear']:
                # Proximity conditions - could be character or resource
                resolved_target = self._resolve_resource_instance(raw_target) or self._resolve_character_instance(raw_target)
                action_copy = action.copy()
                action_copy['target'] = resolved_target
                return action_copy
            elif action_type in ['at_location', 'notat_location']:
                # Location conditions - pass through for now
                resolved_target = self._resolve_resource_instance(raw_target) or self._resolve_character_instance(raw_target)
                action_copy = action.copy()
                action_copy['target'] = resolved_target
                return action_copy
            else:
                logger.warning(f'❓ Unknown action type for resolution: {action_type}')
                resolved_target = raw_target  # Pass through unchanged
                action_copy = action.copy()
                action_copy['target'] = resolved_target
                return action_copy
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
            logger.warning(f'❌ Failed to resolve "{raw_target}" ({action_type})')
            
        return action
    
    
    def _resolve_resource_instance(self, raw_target: str) -> Union[str, bool]:
        """Resolve abstract resource name to specific resource instance."""
        try:
            # First try exact match validation
            for reply in self.session.get(f"cognitive/map/resource/{raw_target}", timeout=2.0):
                if reply.ok:
                    data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                    if data.get('success'):
                        logger.debug(f'✅ Exact resource instance: {raw_target}')
                        return raw_target
                break
            
            # TODO: Fuzzy matching stub - implement abstract -> specific resolution
            # e.g., "Berry" -> "Berry23", "Fruit" -> "Apple1", etc.
            logger.debug(f'🚧 Fuzzy resource resolution stub for: {raw_target}')
            return False
            
        except Exception as e:
            logger.error(f'Error resolving resource instance {raw_target}: {e}')
            return False
    
    def _resolve_character_instance(self, raw_target: str) -> Union[str, bool]:
        """Resolve abstract character name to specific character instance."""
        try:
            # For characters, exact names are typically used
            # Just validate character exists in some form
            canonical_name = raw_target.capitalize()
            logger.debug(f'✅ Character instance: {canonical_name}')
            return canonical_name
            
            # TODO: Fuzzy matching stub - implement abstract -> specific resolution  
            # e.g., "Person" -> "Joe", "Guard" -> "Alice", etc.
            
        except Exception as e:
            logger.error(f'Error resolving character instance {raw_target}: {e}')
            return False
    
    def _find_target_direction(self, target: str) -> str:
        """Find which direction a target (character or resource) is visible in."""
        try:
            # Query situation node for current situation data
            for reply in self.session.get(f"cognitive/{self.character_name}/situation/current_situation", timeout=5.0):
                if reply.ok:
                    situation_data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                    if not situation_data.get('success'):
                        logger.warning(f'Failed to get situation data: {situation_data.get("error", "Unknown error")}')
                        return None
                    
                    situation = situation_data.get('situation', {})
                    look_data = situation.get('look', {})
                    
                    if not look_data:
                        logger.warning(f'No look data available for target search')
                        return None
                    
                    # Search through each direction for the target
                    directions = ['North', 'Northeast', 'East', 'Southeast', 'South', 'Southwest', 'West', 'Northwest']
                    target_lower = target.lower()
                    
                    for direction in directions:
                        direction_data = look_data.get(direction, {})
                        
                        # Check resources in this direction
                        if 'resources' in direction_data:
                            for resource in direction_data['resources']:
                                resource_id = resource.get('id', '') if isinstance(resource, dict) else str(resource)
                                if resource_id.lower() == target_lower or target_lower in resource_id.lower():
                                    logger.info(f'🎯 Found target "{target}" as resource in direction: {direction}')
                                    return direction.lower()
                        
                        # Check characters in this direction
                        if 'characters' in direction_data:
                            for character in direction_data['characters']:
                                character_name = character.get('name', '') if isinstance(character, dict) else str(character)
                                if character_name.lower() == target_lower or target_lower in character_name.lower():
                                    logger.info(f'🎯 Found target "{target}" as character in direction: {direction}')
                                    return direction.lower()
                    
                    logger.warning(f'❌ Target "{target}" not found in any visible direction')
                    return None
                
                break  # Only process first reply
            
            logger.warning(f'❌ No response from situation query for target "{target}"')
            return None
            
        except Exception as e:
            logger.error(f'Error finding direction for target "{target}": {e}')
            return None

    def move(self, move_direction: str):
        """Move the character in the specified direction."""
        logger.warning(f'Moving {move_direction}')
        
        try:
            # Query map node to move the agent
            move_data = {'direction': move_direction}
            for reply in self.session.get(f"cognitive/map/agent/{self.character_name}/move", payload=json.dumps(move_data).encode('utf-8'), timeout=5.0):
                try:
                    if reply.ok:
                        move_result = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                        if move_result.get('success'):
                            logger.info(f'✅ Move successful: {move_direction}')
                        else:
                            logger.warning(f'❌ Move failed: {move_result.get("error", "Unknown error")}')
                    else:
                        logger.error(f'❌ Move query failed for {self.character_name}')
                except Exception as e:
                    logger.error(f'Error parsing move response: {e}')
                    break
            
            # Create and publish action data
            action_data = {
                'type': 'move',
                'direction': move_direction,
                'action_id': f"move_{int(time.time())}",
                'timestamp': datetime.now().isoformat()
            }
            self.last_action = action_data
            self.action_publisher.put(json.dumps(action_data).encode('utf-8'))
            logger.info(f'📤 Published move action: {move_direction}')
            
        except Exception as e:
            logger.error(f'Error in move operation: {e}')
    
    def take(self, target: str):
        """Take a resource and add it to inventory."""
        try:
            # First validate that the target exists and is a resource
            resource_exists = False
            for reply in self.session.get(f"cognitive/map/resource/{target}", timeout=2.0):
                if reply.ok:
                    data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                    if data.get('success'):
                        resource_exists = True
                        logger.debug(f'✅ Validated {target} is a resource')
                    else:
                        logger.warning(f'❌ Cannot take {target} - not a resource or does not exist')
                        return False
                break

            if not resource_exists:
                logger.warning(f'❌ Cannot take {target} - resource validation failed')
                return False

            # Validate that the target is near
            if not plan_module.is_near(self, target):
                logger.warning(f'❌ Cannot take {target} - not near resource')
                return False

            # The take action will be published via the normal action_publisher
            # and memory_node will see it and handle adding to inventory

            logger.info(f'📦 Taking {target} for {self.character_name}')

            # Create and publish action data for logging/display
            action_data = {
                'type': 'take',
                'target': target,
                'action_id': f"take_{int(time.time())}",
                'timestamp': datetime.now().isoformat(),
                'character': self.character_name
            }
            self.last_action = action_data
            self.action_publisher.put(json.dumps(action_data))
            logger.info(f'📤 Published take action: {target}')

            # Remove the resource from the map
            for reply in self.session.get(f"cognitive/map/resource/remove/{target}", timeout=2.0):
                if reply.ok:
                    data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                    if data.get('success'):
                        logger.info(f'🗑️ Removed {target} from map')
                    else:
                        logger.warning(f'⚠️ Failed to remove {target} from map: {data.get("error", "Unknown error")}')
                else:
                    logger.warning(f'⚠️ Failed to remove {target} from map - no response')
                break

            return True

        except Exception as e:
            logger.error(f'Error in take operation for {target}: {e}')
            return False
    
    def think_about(self, value: str):
        """Think about a value."""
        logger.warning(f'Thinking about: {value}')
        try:
            # Get recent memory entries for context
            recent_memories = self._get_recent_chat_memories(3)
            
            # Get entity context if source is not console
            entity_context = None
            entity_context = self.get_entity_context(self.character_name, 10)
            
            # Simple, focused prompt
            if self.character_config.get('character', None):
                system_prompt = self.character_config['character']
            if self.character_config.get('drives', None):
                system_prompt += f"\n\nYour drives are: {self.character_config['drives']}"
            
            # Build user prompt with context
            user_prompt = '' 
            user_prompt += self.format_situation()
            if entity_context and isinstance(entity_context, dict):
                conversation_history = entity_context.get('conversation_history', [])
                if isinstance(conversation_history, list):
                    for i, memory in enumerate(conversation_history):
                        if isinstance(memory, dict) and 'source' in memory and 'text' in memory:
                            user_prompt += f"{memory['source']}: {memory['text']}\n"
            directive = f"""You are thinking about: {value}. Derive new information, insights, goals, or conclusions based on your memories, drives, and the current situation.
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
                    timeout=timeout
                )

                if response.success:
                    logger.warning(f'🤖 {self.character_name} New Thought: {response.text}')
                    thought = hash_utils.find('thought', response.text)
                    if not thought:
                        logger.error(f'No thought found in LLM response: {response.text}')
                        return

                    # Create action
                    action_data = {
                        'type': 'think',
                        'action_id': f'action_{self.action_counter}',
                        'timestamp': datetime.now().isoformat(),
                        'input': value,
                        'text': response.text.strip(),
                        'source': self.character_name
                     }
                    self.last_action = action_data                    
                    # Publish action (this will be picked up by memory_node and action_display_node)
                    self.action_publisher.put(json.dumps(action_data))
                    logger.info(f'📤 Published action: {action_data["action_id"]}')
                    
                    self.action_counter += 1
                else:
                    logger.error(f'LLM call failed: {response.error}')
            else:
                logger.error('LLM client not available')
                
        except Exception as e:
            logger.error(f'Error in LLM processing: {e}')
            logger.error(traceback.format_exc())
        pass
    
    def _get_recent_chat_memories(self, num_entries: int) -> List[Dict[str, Any]]:
        """Get recent memory entries using Zenoh queries."""
        try:
            # Query short-term memory
            entries = []
            for reply in self.session.get(f"cognitive/{self.character_name}/memory/chat/*", timeout=3.0):
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
    
    def get_entity_context(self, entity_name: str, limit: int = 20) -> Dict[str, Any]:
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
            for reply in self.session.get(f"cognitive/{self.character_name}/memory/entity/{entity_name}?query=dialog&limit={limit}", timeout=3.0):
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
            query_url = f"cognitive/{self.character_name}/memory/entity/{entity_name}?query=natural_dialog_end&input_text={encoded_text}"
            
            for reply in self.session.get(query_url, timeout=10.0):
                try:
                    if reply.ok:
                        data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                        if data['success']:
                            logger.info(f'🤔 Natural dialog end check for {entity_name}: {data.get("should_end", False)}')
                            return data.get('should_end', False)
                        else:
                            logger.debug(f'Natural dialog end query failed for {entity_name}: {data.get("error", "Unknown error")}')
                            return False
                except Exception as e:
                    logger.error(f'Error parsing natural dialog end response for {entity_name}: {e}')
                    return False
            
            logger.debug(f'No response received for natural dialog end query: {entity_name}')
            return False
            
        except Exception as e:
            logger.error(f'Error querying natural dialog end for {entity_name}: {e}')
            return False
    
    def shutdown(self):
        """Clean shutdown."""
        logger.info('Shutting down Executive Node...')
        
        # Set shutdown flag to prevent new operations
        self.shutdown_requested = True
        
        # Clean up LLM client first (this cancels pending requests)
        if self.llm_client:
            try:
                self.llm_client.cleanup()
                logger.info('LLM client cleanup completed')
            except Exception as e:
                logger.error(f'Error cleaning up LLM client: {e}')
        
        # Close Zenoh session more carefully
        try:
            # Wait longer for cleanup to avoid Zenoh panics
            time.sleep(2.0)
            self.session.close()
            logger.info('Zenoh session closed')
        except Exception as e:
            logger.error(f'Error closing Zenoh session: {e}')
        
        logger.info('Executive Node shutdown complete')


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