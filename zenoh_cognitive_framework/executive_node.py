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
from typing import Dict, List, Any
import utils.hash_utils as hash_utils

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
            f"cognitive/{character_name}/situation",
            self.situation_callback
        )
        
        # Publisher for actions (character-specific)
        self.action_publisher = self.session.declare_publisher(f"cognitive/{character_name}/action")
        
        # Publisher for memory storage (character-specific)
        self.memory_publisher = self.session.declare_publisher(f"cognitive/{character_name}/memory/store")
        
        # Publisher for text input to other characters (character-specific)
        self.text_input_publisher = self.session.declare_publisher(f"cognitive/{character_name}/text_input")
        
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
                    time.sleep(0.1)
                elif self.interrupt_pending:
                    self._handle_interrupt()
                    self.interrupt_pending = False
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

    def _run_ooda_loop(self):
        """Execute the OODA loop: Observe, Orient, Decide, Act."""
        try:
            logger.warning(f'🔄 {self.character_name} OODA loop running')
            if self.text_input_pending:
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
                self.respond(text_input, source)
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
            
            # Decide: Choose next action
            if self.interrupt_pending:
                self.interrupt_pending = False
                if self._handle_interrupt():
                    return
            action = self._decide(goal)
            
            # Act: Execute the chosen action
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
            formatted_situation += f"You notice the following characters: {self.last_situation_data['visible_characters']}\n"
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
            system_prompt += f"\n\nYour drives are:\n\t{'\n\t'.join(self.character_config['drives'])}"
            
        # Build user prompt with context
        user_prompt = '' 
        user_prompt += self.format_situation()
        entity_context = None
        entity_context = self.get_entity_context(self.character_name, 10)
        if entity_context:
            for i, memory in enumerate(entity_context['conversation_history']):  # Use last 2 memories
                user_prompt += f"{memory['source']}: {memory['text']}\n"
        self.observations = {'static': system_prompt, 'dynamic': user_prompt}
        return self.observations

    def _orient(self, observations: Dict[str, Any]):
        """Orient: Assess current state and drives"""
        """{'static': system_prompt, 'dynamic': user_prompt}"""
        if self.current_goal and False:
            return self.current_goal
        else:
            system_prompt = observations['static']
            user_prompt = observations['dynamic']
            directive = """What would you like to do next? 
Respond with a simple statement of no more than 20 words stating your highest priority goal given who you are, your drives, your past, your memories, and the current situation you find yourself in.
If there is an unanswered question awaiting your response, you may choose to respond to it as your first goal.
Respond using the following hash-formatted text:
#goal <your goal here>
#target <character_name or resource_name>
##
Do not include any other introductory, explanatory, discursive, or formatting text in your response.

"""
            # Make LLM call
            if self.llm_client and not self.shutdown_requested:
                # Use shorter timeout during shutdown
                timeout = 5.0 if self.shutdown_requested else None
                response = self.llm_client.generate(
                    messages=[system_prompt, user_prompt, directive],
                    max_tokens=400,
                    temperature=0.7,
                    timeout=timeout
                )

                if response.success:
                    logger.warning(f'🤖 {self.character_name} New Goal: {response.text.strip()}')
                    self.current_goal = hash_utils.find('goal', response.text.strip())
                    self.current_target = hash_utils.find('target', response.text.strip())
                else:
                    logger.error(f'LLM call failed: {response.error}')
                    self.current_goal = 'sleep'
            else:   
                logger.error('LLM client not available')
                self.current_goal = 'sleep'
            return self.current_goal

    def _decide(self, goal: str):
        """Decide: Choose next action. Stub."""
        if not self.current_goal or self.current_goal == 'sleep':
            return None
        else:
            system_prompt = self.observations['static']
            user_prompt = self.observations['dynamic']
            goal_prompt = f"\n\nYour current goal is: {goal}"
            if self.current_target:
                entity_context = self.get_entity_context(self.current_target, 10)
                if entity_context and len(entity_context['conversation_history']) > 0:
                    goal_prompt += f'your recent dialog with {self.current_target} has been:\n'
                    for i, memory in enumerate(entity_context['conversation_history']):  # Use last 2 memories
                        goal_prompt += f"\t{memory['source']}: {memory['text']}\n"

                    goal_prompt += "Don't repeat yourself.\n"
            directive = """What would you like to do next to progress towards your goal? 
Respond with a selection from the action set:

Say - speak to another character you can see. For a 'say' act, speak only for yourself, and do not include any other introductory, explanatory, discursive, or formatting text in your response.
Move - Move in one of the 8 cardinal directions
Think - think about a topic or question, attempting to derive new information, conclusions, or decisions from who you are and what you already explicitly know
Take - add some resource you see to your personal inventory
Inspect - inspect a resource you see or one in your inventory to understand how to use it.
Use - use a resource in a known way.

Select an action given who you are, your drives, and the current situation you find yourself in.

"""
            if self.last_action and self.last_action['action'].lower() == 'say':
                directive = """Do NOT attempt to say anything. Respond using the following hash-formatted text:
#action Sleep / Move / Think / Take / Inspect / Use
#target na / cardinal_direction / 'self' / resource_name / resource_name / resource_name
#value na / reason for moving (5 words max) / text to think about (10 words max) / reason for taking (5 words max) / reason for inspecting (5 words max) / target (character_name or resource_name)
##
Do not include any other introductory, explanatory, discursive, or formatting text in your response.
End your response with: 
</end>
"""
            else:
                directive = """Respond using the following hash-formatted text:
#action Say / Move / Think / Take / Inspect / Use
#target character_name / cardinal_direction / 'self' / resource_name / resource_name / resource_name
#value text to speak / reason for moving (5 words max) / text to think about (10 words max) / reason for taking (5 words max) / reason for inspecting (5 words max) / target (character_name or resource_name)
##
Do not include any other introductory, explanatory, discursive, or formatting text in your response.
End your response with: 
</end>

"""         # Make LLM call
            if self.llm_client and not self.shutdown_requested:
                response = self.llm_client.generate(
                    messages=[system_prompt, user_prompt, goal_prompt,directive],
                    max_tokens=400,
                    temperature=0.7,
                    stops=['</end>']
                )

                if response.success:
                    logger.warning(f'🤖 {self.character_name} New Action: {response.text.replace('\n', ' ')}')
                    action_hash = hash_utils.findall_forms(response.text)
                    if action_hash and len(action_hash) > 0:
                        action = hash_utils.find('action', action_hash[0])
                        target = hash_utils.find('target', action_hash[0])
                        value = hash_utils.find('value', action_hash[0])
                        if not action or not target or not value:
                            logger.error(f'No action, target, or value found in LLM response: {response.text}')
                            self.current_action = {'action': 'sleep', 'target': 'self', 'value': ''}
                        else:
                            self.current_action = {'action': action, 'target': target, 'value': value}
                    else:
                        logger.error(f'No action found in LLM response: {response.text}')
                        self.current_action = {'action': 'sleep', 'target': 'self', 'value': ''}
                else:
                    logger.error(f'LLM call failed: {response.error}')
                    self.current_action = {'action': 'sleep', 'target': 'self', 'value': ''}
            else:   
                logger.error('LLM client not available')
                self.current_action = {'action': 'sleep', 'target': 'self', 'value': ''}
            return self.current_action

    def _act(self, action: Dict[str, Any]):
        """Act: Execute the chosen action."""
        self.last_action = action
        if action['action'].lower() == "sleep":
            time.sleep(1)  # Sleep for 1 second
        elif action['action'].lower() == "move":
            move_direction = action['target'].lower().strip()
            if move_direction in ['north', 'northeast', 'southeast', 'south', 'southwest', 'northwest', 'east', 'west']:
                logger.warning(f'{action}')
                self.move(move_direction)
            else:
                logger.error(f'Unknown move direction: {move_direction}')
            self.action_counter += 1
        elif action['action'].lower() == "say":
            # Create action
            action_data = {
                        'type': 'say',
                        'action_id': f'action_{self.action_counter}',
                        'timestamp': datetime.now().isoformat(),
                        'source': self.character_name,
                        'text': action['value']                
                        }
                    
            # Publish action (this will be picked up by action_display_node)
            self.action_publisher.put(json.dumps(action_data))
            self.send_text_input(action['target'], action['value'])
        elif action['action'].lower() == "think":
            self.think_about(action['value'])
            self.action_counter += 1    
        elif action['action'].lower() == "take":
            action_data = {'type': 'take','action_id': self.action_counter,'timestamp': datetime.now().isoformat(),'target': action['target']}
            self.action_publisher.put(json.dumps(action_data))
            #self.take(action['target'])
            self.action_counter += 1
        elif action['action'].lower() == "inspect":
            action_data = {'type': 'inspect','action_id': self.action_counter,'timestamp': datetime.now().isoformat(),'target': action['target']}
            self.action_publisher.put(json.dumps(action_data))
            #self.inspect(action)
            self.action_counter += 1
        elif action['action'].lower() == "use":
            # Create action
            action_data = {'type': 'use','action_id': self.action_counter,'timestamp': datetime.now().isoformat(),'target': action['target']}
            self.action_publisher.put(json.dumps(action_data))
            #self.use(action)
        logger.warning(f'📤 Published action: {action}')
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
    
    def respond(self, text_input: str, source: str):
        """Respond to text input from someone."""
        try:
            logger.info(f'Responding to: "{text_input}" (from {source})')
            
            # Get recent memory entries for context
            recent_memories = self._get_recent_chat_memories(3)
            
            # Get entity context if source is not console
            entity_context = None
            if source != 'console':
                entity_context = self.get_entity_context(source, 10)
            
            # Simple, focused prompt
            system_prompt = """You are a helpful AI assistant. 
            Analyze the input and provide a clear, actionable response.
            Focus on being helpful and direct."""
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
            user_prompt +=  """Respond to the dialog to date. If you have not yet spoken, speak for yourself. If the dialog has reached a natural end, respond with 'Done'. 
Do not include any other introductory, explanatory, discursive, or formatting text in your response.
End your response with: </end>"""
                    
            # Make LLM call
            if self.llm_client and not self.shutdown_requested:
                # Use shorter timeout during shutdown
                timeout = 5.0 if self.shutdown_requested else None
                response = self.llm_client.generate(
                    messages=[system_prompt, user_prompt],
                    max_tokens=400,
                    temperature=0.7,
                    timeout=timeout
                )

                if response.success:
                    text_to_send = response.text.strip()
                    if not text_to_send.lower().startswith('done'):
                        self.send_text_input(source, text_to_send)
                        logger.warning(f'Responding to: "{text_input}" from {source}: {text_to_send}')

                    # Create action
                    action_data = {
                        'type': 'response',
                        'action_id': f'action_{self.action_counter}',
                        'timestamp': datetime.now().isoformat(),
                        'input': text_input,
                        'text': response.text.strip(),
                        'source': source
                     }
                    
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
            
            self.action_publisher.put(json.dumps(action_data).encode('utf-8'))
            logger.info(f'📤 Published move action: {move_direction}')
            
        except Exception as e:
            logger.error(f'Error in move operation: {e}')
    
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
            # Query entity data from memory node - try without query parameters first
            for reply in self.session.get(f"cognitive/{self.character_name}/memory/entity/{entity_name}", timeout=3.0):
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