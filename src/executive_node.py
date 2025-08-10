#!/usr/bin/env python3
"""
Zenoh Executive Node

This node implements the OODA loop for character decision-making and action execution.
Replaces ROS2 complexity with simple Zenoh pub/sub.
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
import utils.hash_utils as hash_utils
import plan as plan_module
from dataclasses import dataclass

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

PLAN_SYNTAX = """
Task: Break down the user's high‑level goal into a minimal plan in the JSON format specified below.
Output: only valid JSON – no prose, no code fences.

{
  "plan": [
    { "type": "move", "target": "…"},
    { "type": "say", "target": "…", "value": "…" },
    { "type": "think", "value": "…" },
    { "type": "take", "target": "…" },
    { "type": "inspect", "target": "…" },
    { "type": "use", "target": "…" },
    { "type": "while", "condition": "…" , "body": [ /* steps */ ]},
    { "type": "if", "condition": "…", "then": [ /* steps */ ], "else": [ /* steps */ ] }
  ]
}

A plan must include at least 1 step and no more than 6 steps including all nested while and if branches.
In the following, <resource_name>, <character_name> are placeholders only for KNOWN resources, characters, or maptypes, those appearing above.
Only dicts of the types below are allowed for the condition of while and if. Condition action type can only be one of the following:
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

outside a while or if condition, "type" can take the values "say", "move", "think", "take", "inspect", or "use":
 - "say": { "type": "say", "target": "character_name", "value": "text to speak" } 
     for speaking to another character you can see. Use this to seek information, respond, inform the other character, or to maintain 'social chatter' to stay aligned.
     For a 'say' act, speak only for yourself, and do not include any other introductory, explanatory, discursive, or formatting text in your response.
 - "move": { "type": "move", "target": "cardinal_direction" or 'resource or character name'} 
     For moving in one of the 8 cardinal directions or in the direction of a resource or character.
    You can only move in the direction of a resource, character, or terrain type if you can see it.
 - "think": { "type": "think", "value": "text to think about" } 
     For thinking about a topic or question, attempting to derive new information, conclusions, or decisions from who you are and what you already explicitly know
 - "take": { "type": "take", "target": "resource_name" } 
     For adding some resource you see to your personal inventory. you must be 'near' the resource to take it.
 - "inspect": { "type": "inspect", "target": "resource_name" } 
     For inspecting a resource you see or one in your inventory to understand how to use it.
 - "use": { "type": "use", "target": "resource_name" } 
     For using a resource in your inventory in a known way.

In general, a target can be a specific resource_name or character_name or a map type generalization. 
For example, Berry2, Berry, Joe, Clearing (assuming it is a terrain type) are all valid targets. 
However, if the target is not an instance, the specific resource or character bound to the target is indeterminate. 
For move, target can also be one of the 8 compass points.

For example, 
    {"type": "if", "condition": {"type": "near", "target": "Berry"}, "then": [{"type": "move", "target": "Berry"}]}
is a valid plan. Likewise,
    {"type": "if", "condition": {"type": "near", "target": "Joe"}, "then": [{"type": "move", "target": "Joe"}]}
is a valid plan.
Note that move only moves one step. You can use a while to move repeatedly.

Some actions have conditions that must be met before they can be executed.
for example, you cannot take a resource unless you are near it.
you can accomplish this by using the "near" condition in a while form. 
Assuming, for example, that Cave2 is in your situation view direction Northeast, the following plan will move you to Cave2:

{
  "plan": [
    { "type": "while", "body": [ { "type": "move", "target": "Northeast" } ], "condition": { "type": "notnear", "target": "Cave2" } },
    { "type": "take", "target": "Cave2" }
  ]
}

###
Allowed control‑flow primitives: sequential list (e.g.. [..., ...]), while, and two‑branch if (else is optional).
{
  "plan": [
    { "type": "action", "target": "…", "value": "…" },
    { "type": " while", "body": [ /* steps */ ], "condition": "…" },
    { "type": "if", "condition": "…", "then": [ /* steps */ ], "else": [ /* steps */ ] }
  ]
}

A plan must include no more than 6 steps including all nested while and if branches.
A plan must not contain sequential adjacent say actions.
"""

@dataclass
class ActionRecord:
    """Record of an action and its result."""
    action: Dict[str, Any]
    result: Optional[str]
    timestamp: datetime

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
        self.action_history = []  # List of ActionRecord instances
        
        # Plan execution state
        self.current_plan = None
        self.plan_state = None
        self.plan_bindings_cache = {}
        self.plan_summary_completed = False  # Track if current plan has been summarized
        self.plan_summary = None
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
        for reply in self.session.get("cognitive/map/types", timeout=2.0):
            if reply.ok:
                data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                if data.get('success'):
                    self.map_types = data
                    break
        self.inspections = {} # cache of inspections

        # Register signal handlers for graceful shutdown
        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)
        
        logger.info(f'🧠 Zenoh Executive Node initialized for character: {character_name}')
        logger.info(f'   - Subscribing to: cognitive/{character_name}/sense_data')
        logger.info(f'   - Subscribing to: cognitive/{character_name}/situation/update')
        logger.info(f'   - Subscribing to: cognitive/map/turn/go')
        logger.info(f'   - Subscribing to: cognitive/{character_name}/end_dialog')
        logger.info(f'   - Publishing to: cognitive/{character_name}/action')
        logger.info(f'   - Publishing to: cognitive/{character_name}/situation/request_update')
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
                        self._act(action)

                        
                return

            # Orient: Assess current state and goals
            goal = self._orient(observations)

            # Plan: Return existing plan or create single-action plan
            plan = self._plan(goal)
            if not plan:
                self._complete_turn()
                return

            # Plan Step: Execute current step of plan
            action = self._plan_step(plan)

            # Act: Execute the chosen action (if we have one)
            if action is not None:
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
            formatted_situation += f"You can see {len(self.last_situation_data)} people: {', and '.join(self.last_situation_data['characters'])}\n"
        if self.last_situation_data and self.last_situation_data.get('look'):
            formatted_situation += f"You can see the following:\n\t{'\n\t'.join(self.last_situation_data['look'])}\n"
        
        # Add adjacent information
        if self.last_situation_data and self.last_situation_data.get('adjacent_to'):
            adjacent = self.last_situation_data['adjacent_to']
            if adjacent.get('resources'):
                formatted_situation += f"You are adjacent to these resources (available to take, inspect, or use): {', '.join(adjacent['resources'])}\n"
            if adjacent.get('characters'):
                formatted_situation += f"You are adjacent to these characters (available to interact with): {', '.join(adjacent['characters'])}\n"

        if self.last_situation_data and self.last_situation_data.get('characters'):
            for character_name in self.last_situation_data['characters']:
                entity_context = self.get_entity_context(character_name, 10)
                if entity_context:
                    formatted_situation += f"\nYou can see {character_name}, with whom you have had the following conversation history:\n"
                    for memory in entity_context['conversation_history']: 
                        formatted_situation += f"\n\t{memory['source']}: {memory['text']}"
                    formatted_situation += '\n'

        if self.last_situation_data and self.last_situation_data.get('views'):
            formatted_situation += f"\nYou can see the following:\n"+json.dumps(self.last_situation_data['views'], indent=2)

        if self.inspections:
            formatted_situation += f"\nYou have inspected the following:\n"
            for target, inspection in self.inspections.items():
                formatted_situation += f"\n\t{target}: {inspection}"
            formatted_situation += '\n'

        return formatted_situation

    def _update_system_prompt(self):
        """Update the system prompt with the current situation."""
        system_prompt = self.observations['static']
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
                for reply in self.session.get("cognitive/map/types", timeout=2.0):
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
            for reply in self.session.get(f"cognitive/{self.character_name}/memory/inventory", timeout=2.0):
                if reply.ok:
                    data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                    if data.get('success'):
                        inventory.append(data.get('value', []))
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

    def _orient(self, observations: Dict[str, Any]):
        """Orient: Assess current state and drives"""
        """{'static': system_prompt, 'dynamic': user_prompt}"""
        if self.current_goal:
            return self.current_goal
        else:
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

                    goal_prompt += "Don't repeat yourself.\n"

            if self.action_history and (self.action_history[-1].action['type'].lower() == 'say' or self.action_history[-1].action['type'].lower() == 'response'):
                directive = f"""\nrespond only with the JSON plan, no other text.\n"""
            else:
                directive = f"""\nrespond only with the JSON plan, no other text.\n"""
            # Make LLM call
            if self.llm_client and not self.shutdown_requested:
                response = self.llm_client.generate(
                    messages=[system_prompt, user_prompt, PLAN_SYNTAX, directive],
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
        
        self.plan_summary_completed = False  # Reset for new plan
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
        system_prompt = 'Your task is to replan given your current goal, your remaining current plan, and new input provided by the user.\n' + self._update_system_prompt()
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

#The new input causing you to replan is:
{reason}

 respond only with new JSON plan, no other text.\n"""

        # Make LLM call
        if self.llm_client and not self.shutdown_requested:
            response = self.llm_client.generate(
                messages=[system_prompt, user_prompt, PLAN_SYNTAX, directive],
                max_tokens=1000,
                temperature=0.7,
                is_json=True,
                stops=['</end>']
            )

            if response.success:
                logger.debug(f'🤖 {self.character_name} New Action: {response.text}')
                plan_candidate = None
                valid = False
                try:
                    plan_candidate = response.text #json.loads(response.text.strip())
                    valid = plan_module.verify_plan(plan_candidate)
                    logger.warning(f'🤖 {self.character_name} Replan candidate: {plan_candidate}')
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
            
        self.plan_summary_completed = False  # Reset for new plan
        self._publish_current_plan()
        # Initialize plan state for new plan
        self.plan_state = {'step_stack': plan_module.Stack()}
        self.plan_bindings_cache = {}      
        return self.current_plan
        
    def _plan_step(self, plan):
        """Execute current step of plan and return next action using frame-based stack."""
        # Extract plan steps from dict format
        if isinstance(plan, dict) and 'plan' in plan:
            plan_steps = plan['plan']
        else:
            plan_steps = plan
            
        if not plan_steps or len(plan_steps) == 0:
            return None
        
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
    
    def _execute_next_step(self, step_stack):
        """Execute next step using frame-based stack."""
        if step_stack.is_empty():
            return None
        
        current_frame = step_stack.peek()
        plan = current_frame['plan']
        idx = current_frame['idx']
        
        # Check if we've completed the current level
        if idx >= len(plan):
            # Handle frame completion based on type
            if current_frame['type'] == 'while':
                # While body completed, increment iteration count
                current_frame['iteration_count'] += 1
                
                # Check iteration limit first
                if current_frame['iteration_count'] >= current_frame['max_iterations']:
                    # Max iterations reached, exit loop
                    step_stack.pop()  # Remove while frame
                    if step_stack.is_empty():
                        # Plan complete
                        self._summarize_plan_execution()
                        self.current_plan = None
                        self.plan_bindings_cache = {}
                        self.current_goal = None
                        self._publish_current_plan()
                        self.plan_state = None
                        return None
                    else:
                        # Continue at parent level
                        parent_frame = step_stack.peek()
                        parent_frame['idx'] = current_frame['return_to']
                        return self._execute_next_step(step_stack)
                else:
                    # Test original condition
                    condition_action = current_frame['condition']
                    resolved_target = self._resolve_target(condition_action)
                    #if plan_module._evaluate_condition(self, condition_action, resolved_target): # _resolve_target returns non False if condition is met!
                    if resolved_target:
                        # Condition true, repeat loop
                        current_frame['idx'] = 0  # Reset to start of body
                        return self._execute_next_step(step_stack)
                    else:
                        # Condition false, exit loop
                        step_stack.pop()  # Remove while frame
                        if step_stack.is_empty():
                            # Plan complete
                            self._summarize_plan_execution()
                            self.current_plan = None
                            self.plan_bindings_cache = {}
                            self.current_goal = None
                            self._publish_current_plan()
                            self.plan_state = None
                            return None
                        else:
                            # Continue at parent level
                            parent_frame = step_stack.peek()
                            parent_frame['idx'] = current_frame['return_to']
                            return self._execute_next_step(step_stack)
            else:
                # Regular frame completed, pop and continue
                step_stack.pop()
                if step_stack.is_empty():
                    # Plan complete
                    self._summarize_plan_execution()
                    self.current_plan = None
                    self.plan_bindings_cache = {}
                    self.current_goal = None
                    self._publish_current_plan()
                    self.plan_state = None
                    return None
                else:
                    # Continue at parent level
                    parent = step_stack.peek()
                    parent['idx'] = current_frame.get('return_to', parent['idx'])   # ⟵ NEW
                    return self._execute_next_step(step_stack)
        
        step = plan[idx]
        
        # Handle different step types
        if step['type'] in ['move', 'say', 'think', 'take', 'inspect', 'use', 'near', 'look']:
            # Execute action and advance
            current_frame['idx'] = idx + 1
            return step
        
        elif step['type'] == 'while':
            # Handle while loop - test condition first
            condition_action = step.get('condition', None)
            resolved_target = self._resolve_target(condition_action)
            if plan_module._evaluate_condition(self, condition_action, resolved_target):
                # Condition true, enter loop
                while_frame = {
                    'plan': step['body'],
                    'idx': 0,
                    'type': 'while',
                    'condition': condition_action,
                    'return_to': idx + 1,  # Where to go when loop exits
                    'iteration_count': 0,  # Track current iteration
                    'max_iterations': 5    # Maximum allowed iterations
                }
                step_stack.push(while_frame)
                return self._execute_next_step(step_stack)
            else:
                # Condition false, skip loop entirely
                current_frame['idx'] = idx + 1
                return self._execute_next_step(step_stack)
        
        elif step['type'] == 'if':
            # Handle if-then-else
            cond = step['condition']
            branch = 'then' if plan_module._evaluate_condition(self, cond, self._resolve_target(cond)) else 'else'
            if branch == 'then' or step.get('else'):
                step_stack.push({
                    'plan'      : step[branch],
                    'idx'       : 0,
                    'type'      : 'if_then' if branch == 'then' else 'if_else',
                    'return_to' : idx + 1
                })
            else:
                current_frame['idx'] = idx + 1
            return self._execute_next_step(step_stack)
        
        else:
            # Unknown step type, skip it
            logger.warning(f'Unknown plan step type: {step["type"]}')
            current_frame['idx'] = idx + 1
            return self._execute_next_step(step_stack)
    

    def _act(self, action: Dict[str, Any]):
        """Act: Execute the chosen action."""

        action = self.current_action if self.current_action else action
        
        # Create action record
        action_record = ActionRecord(
            action=action,
            result=None,  # Will be set below
            timestamp=datetime.now()
        )
        self.action_history.append(action_record)
        
        if action['type'].lower() == "sleep":
            action_data = {'type': 'sleep','action_id': self.action_counter,'timestamp': datetime.now().isoformat(),'target': self.character_name}
            self.action_publisher.put(json.dumps(action_data))
            action_record.result = 'slept'
            time.sleep(1)  # Sleep for 1 second
            return
        if action['type'].lower() == 'think':
            thought = self.think_about(action)
            return
            
        resolved_target = self._resolve_target(action)
        if not resolved_target:
            logger.error(f'❌ Cannot resolve target for action: {action}')
        if action['type'].lower() == "move":
            move_target = resolved_target.strip() if resolved_target else ''
            move_direction = move_target.lower()
            
            # Step 1: Test for compass points first (case insensitive)
            cardinal_directions = ['north', 'northeast', 'southeast', 'south', 'southwest', 'northwest', 'east', 'west']
            if move_direction in cardinal_directions:
                self.move(move_direction)
                # Request situation/map update for UI immediately after move
                try:
                    self.map_update_request_publisher.put(json.dumps({'type': 'step_look'}))
                except Exception:
                    pass
                return
            else:
                logger.error(f'❌ Cannot move toward "{move_target}" - target not resolved or visible, choosing random direction')
                self.move(random.choice(cardinal_directions))
                try:
                    self.map_update_request_publisher.put(json.dumps({'type': 'step_look'}))
                except Exception:
                    pass
                return
        elif action['type'].lower() == "say":
            self.generate_speech(action['value'], resolved_target if resolved_target else action['target'], mode='say')            # Publish action (this will be picked up by action_display_node)

        elif action['type'].lower() == "take":
            self.take(resolved_target)
            self.action_counter += 1
        elif action['type'].lower() == "inspect":
            # Publish initial inspect action
            action_data = {'type': 'inspect','action_id': self.action_counter,'timestamp': datetime.now().isoformat(),'target': resolved_target}
            self.action_publisher.put(json.dumps(action_data))
            # Perform inspect which may populate last_action_result
            self.inspect(resolved_target)
            # Publish result if available so UI can display it
            try:
                result_text = action_record.result if isinstance(action_record.result, str) else ''
                if result_text and result_text.strip():
                    result_action = {
                        'type': 'inspect',
                        'action_id': f"inspect_{int(time.time())}",
                        'timestamp': datetime.now().isoformat(),
                        'target': resolved_target,
                        'llm_response': result_text
                    }
                    self.action_publisher.put(json.dumps(result_action))
            except Exception:
                pass
        elif action['type'].lower() == "use":
            # Create action - noop for now
            action_data = {'type': 'use','action_id': self.action_counter,'timestamp': datetime.now().isoformat(),'target': resolved_target}
            action_record.result = 'not yet implemented'
            self.action_publisher.put(json.dumps(action_data))
            #self.use(action)
        # Request situation/map update for UI after non-move actions that may affect visibility/adjacency
        try:
            if action['type'].lower() in ["take", "inspect", "use"]:
                self.map_update_request_publisher.put(json.dumps({'type': 'step_look'}))
        except Exception:
            pass

        logger.warning(f'📤 Published action: {action["type"]}')
        self.action_counter += 1

    def _handle_interrupt(self):
        """Handle interrupt from sense data or situation updates."""
        logger.info('🔄 Handling interrupt')
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
        logger.info(f'📝 Summary prompt prepared for {self.character_name}')
        logger.debug(f'Summary prompt: {summary_prompt}')
        
        # Mark as completed to prevent redundant calls
        self.action_history = []
        self.plan_summary_completed = True
    
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
            
        except Exception as e:
            logger.error(f'Error processing situation data: {e}')
    
    def turn_callback(self, sample):
        """Handle turn "GO" signals."""
        try:
            logger.warning(f'🚦 Turn {self.current_turn_number} received by {self.character_name}')
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
    
    def _dialog_end_callback(self, sample):
        """Handle end dialog query from other characters."""
        try:
            # Extract the other character name from the JSON payload
            data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            other_name = data.get('other_name', 'unknown')
            if other_name == 'unknown':
                logger.error(f'Error in end dialog callback: no other character name found in payload')
                return
                
            logger.warning(f'💬 {self.character_name} received end dialog from {other_name}')
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
                self._replan(self.current_goal, 'Goal set by user')
            return
        except Exception as e:
            logger.error(f"Goal parsing failed for {self.character_name}: {e}")
            traceback.print_exc()
            return

    def parse_and_set_plan(self, plan_text):
        """Parse plan input from UI and set current plan."""
        try:
            parsed_plan = plan_module.parse_plan_text(plan_text)
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
                logger.warning(f'🤖 {self.character_name} Dialog end check: {are_we_done}')
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
                        logger.warning(f'Responding to: "{text_input}" from {source}: {text_to_send}')

                else:
                    logger.error(f'LLM call failed: {response.error}')
            else:
                logger.error('LLM client not available, skipping LLM call')
                
        except Exception as e:
            logger.error(f'Error in LLM processing: {e}')
            logger.error(traceback.format_exc())
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
                logger.warning(f'❓ Unknown action type for resolution: {action_type}')
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
            logger.warning(f'❌ Failed to resolve "{raw_target}" ({action_type})')
            
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
                        if resource.get('distance', 20) <= 2 and ((not any(ch.isdigit() for ch in target) and resource['name'].startswith(target)) 
                            or resource['name'] == target):                            
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
            for reply in self.session.get(f"cognitive/{self.character_name}/memory/inventory?item={raw_target}", timeout=2.0):
                if reply.ok:
                    data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                    if data.get('success'):
                        logger.debug(f'✅ Exact resource instance: {raw_target}')
                        return raw_target
                break
            
            # First try exact match validation
            for reply in self.session.get(f"cognitive/map/bind/resource/{raw_target}", timeout=2.0):
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
            for reply in self.session.get(f"cognitive/{self.character_name}/situation/current_situation", timeout=5.0):
                if reply.ok:
                    situation_data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                    if not situation_data.get('success'):
                        logger.warning(f'Failed to get situation data: {situation_data.get("error", "Unknown error")}')
                        return None
                    
                    situation = situation_data.get('situation', {})
                    views = situation.get('views', {})
                    
                    if not views:
                        logger.warning(f'No views available for target search')
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
            resource_exists = False
            for reply in self.session.get(f"cognitive/map/resource/{target}", timeout=2.0):
                if reply.ok:
                    data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                    if data.get('success'):
                        resource_exists = True
                        logger.debug(f'✅ Validated {target} is a resource')
                break
            if not resource_exists:
                logger.warning(f'❌ Cannot take {target} - resource validation failed')
                if self.action_history:
                    self.action_history[-1].result = f'cannot take {target} - resource validation failed'
                return False

            # Validate that the target is near
            if not plan_module.is_near(self, target):
                logger.warning(f'❌ Cannot take {target} - not near resource')
                if self.action_history:
                    self.action_history[-1].result = f'cannot take {target} - not near resource'
                return False

            logger.info(f'📦 Taking {target} for {self.character_name}')
            if self.action_history:
                self.action_history[-1].result = f'taking {target}'
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
    
    def inspect(self, target: str):
        """Learn about a resource."""
            # Create and publish action data for logging/display
        action_data = {
                'type': 'inspect',
                'target': target,
                'action_id': f"take_{int(time.time())}",
                'timestamp': datetime.now().isoformat(),
                'character': self.character_name
        }
        try:
            system_prompt = self._update_system_prompt()
            user_prompt = self.observations['dynamic']

            directive = f"""You are inspecting: {target}.\n\n 
respond with a short text description of the resource consistent with its name and situated wrt the information in the situation.
The description should particularly highlight information relevant to your drives, goal, and current plan.
It should also include some 'color' to enrich your perception of the resource in the context of the current situation.
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
                    timeout=timeout
                )
                if response.success:
                    logger.warning(f'🤖 {self.character_name} Inspected {target}:\n\t {response.text}')
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

    def think_about(self, action: dict):
        """Think about a value."""
        logger.warning(f'Thinking about: {action}')
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
                    timeout=timeout
                )

                if response.success:
                    logger.warning(f'🤖 {self.character_name} New Thought: {response.text}')
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
            query_url = f"cognitive/{self.character_name}/memory/entity/{entity_name}?query=natural_dialog_end&input_text={encoded_text}&context={self.observations['static']}"
            
            for reply in self.session.get(query_url, timeout=10.0):
                try:
                    if reply.ok:
                        data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                        if data['success']:
                            logger.info(f'🤔 Natural dialog end check for {entity_name}: {data.get("should_end", True)}')
                            return data.get('should_end', True)
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
    
    def shutdown(self):
        """Clean shutdown."""
        if self._shutting_down:
            return
        self._shutting_down = True
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