from __future__ import annotations

from enum import Enum
import random
import subprocess
import time
import argparse
import yaml
from weakref import WeakValueDictionary
import os, sys
from datetime import timedelta, datetime
import logging
import json
import numpy as np
import zenoh
#from sentence_transformers import SentenceTransformer
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from Messages import SystemMessage
from llm_client import ZenohLLMClient
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
map_types = None

# Default activity used when no suitable activities are found
DEFAULT_ACTIVITY = {
    "name": "goal_fallback",
    "category": "Cognitive/Inner",
    "tags": ["mental", "solo"],
    "when": "opportunistic",
    "where": ["anywhere"],
    "start": ["no_activity_available"],
    "stop": ["goal_created"],
    "duration": [5, 10],  # Short duration
    "needs": [],
    "steps": ["GOAL_NEEDED"],  # Distinguished step name
    "importance": 0.1,  # Low importance
    "habit": 0.0
}

ontology_prompt = """You are creating an ontology for everyday activities. Attend especially to the era, setting, and locale, these will often be fantasy or historical settings.
INPUT:
#Setting: a string including era, locale, climate, tech, norms:
    {setting}
##

#Additional information about the setting:
    {map_types}
##
    
#Character: roles, obligations, skills, preferences, constraints:
    {character}
##
    
# Other Characters:
    {other_characters}
##

Include mental, physical, and social 'activities' that are possible and consisten with the character in this setting in your ontology.

OUTPUT (JSON):

- places[] - places the character can be in the setting
- tools[] - tools the character can use in the setting
- roles[] - roles the character can play in the setting
- norms_rules[] - norms and rules the character must follow in the setting
- hazards[] - hazards the character must avoid in the setting
- social_graph[] - social graph of the character in the setting
- time_windows - Either:
    - One of: dawn,morning, afternoon, dusk, evening, night
    - [start,end] (24 hour clock)
- affordances - e.g. place->[activities_possible]

respond only with the JSON, no other text.
end your response with </end>
"""
llm_client = None

def get_ontology(context, character_name, character, other_characters, map_types_str):
    """Get the ontology for a given setting and character."""

    other_characters_str = ''
    for other_character_name, other_character_desc in other_characters.items():
        if other_character_name != character_name:
            other_characters_str += f"\n\t{other_character_name}: {other_character_desc}"

    return ontology_prompt.format(setting=context, character=character, other_characters=other_characters_str, map_types=map_types_str)

def get_ontology_from_zenoh(context, character_name, character, other_characters):
    global llm_client
    """Get the ontology for a given setting and character from Zenoh."""
    # Initialize Zenoh session
    if map_types:
        map_types_str = f'\n#Available map types:'
        if map_types.get('terrain_types'):
            map_types_str += f"\n\tTerrains: {', '.join(map_types['terrain_types'])}"
        if map_types.get('pathway_types'):
            map_types_str += f"\n\tPathways {', '.join(map_types['infrastructure_types'])}"
        if map_types.get('property_types'):
            map_types_str += f"\n\tProperties: {', '.join(map_types['property_types'])}"
        if map_types.get('resource_types'):
            map_types_str += f"\n\tResources: {', '.join(map_types['resource_types'])}"
        map_types_str += '\n'
    else:
        map_types_str = ''

    prompt = get_ontology(context, character_name, character, other_characters, map_types_str=map_types_str)

    if llm_client:
        response = llm_client.generate(
                    messages=[prompt],
                    max_tokens=1500,
                    temperature=0.7,
                    stops=['</end>'],
                    is_json=True
                )

    if response.success:
        print(f'🤖  Activity taxonomy: \n{response.text}')
        return response.text
    else:
        print(f'❌ Failed to get ontology: {response.error}')
        return None

def get_activities(context, character_name, character, other_characters, ontology):
    prompt = """You are generating a comprehensive activity list for a character. 
CHARACTER: 
{{$character}}
##

TASK: Generate 24-30 activities as a JSON object, with the keys being the activity names. Distribute across these categories:
- Physiological ADLs (eating, drinking, sleeping) - 3-4 activities situated in the setting and your available roles
- Instrumental ADLs/Logistics (planning, preparing) - 2-3 activities  situated in the setting and your available roles
- Mobility & Transport  - 2-3 activities situated in the setting and your available roles
- Role/Production - 3-4 activities situated in the setting and your available roles
- Learning & Practice - 1-2 activities situated in the setting and your available roles
- Cognitive/Inner - 1-2 activities situated in the setting and your available roles
- Recreation & Leisure - 2-3 activities situated in the setting and your available roles
- Safety/Emergency - 2-3 activities, pro-active or reactive, situated in the setting and your available roles
- Health Management (self-care) - 2-3 activities situated in the setting and your available roles
- Maintenance & Repair - 2-3 activities situated in the setting and your available roles
##

<period> is one of: dawn, morning, afternoon, dusk, evening, night
<season> is one of: winter, spring, summer, autumn

Each Activity should conform to this schema:
{
  "name": "string",
  "category": <category from above>,
  "tags": ["physical","mental","social","solo","outdoors","survival","routine"],
  "when": "daily@<period> | opportunistic | seasonal@<season>", 
  "where": ["place ids"],
  "start": ["simple predicates (time/window, place, weather, drive)"],
  "stop": ["simple predicates or duration cap"],
  "duration": [min_minutes, max_minutes],
  "needs": ["tools or innate capacities"],
  "steps": ["3–5 terse steps; optional #safe markers"],
  "importance": 0.0,
  "habit": 0.0
}

The following ontology can be used to fill in the details of activities:
ONTOLOGY: 
{{$ontology}}
##

Social activities are a priority. Be sure to include social activities in your output. 
Social activites are only possible with other characters. 
#Characters you can interact with:
{{$other_characters}}
##

#Step Guidelines:
Generate steps that can serve as achievable planning goals. Each step should:

- Be outcome-focused rather than method-focused: "acquire drinkable water" rather than "purify water using specific method"
- Have clear success criteria: "locate edible resources within area" rather than "find food" (too vague)
- Be situationally adaptable: "prepare food for consumption" rather than "cook food over fire" (assumes fire available)
- Avoid meta-cognitive abstractions: "identify today's priorities" rather than "set goals" (too abstract)
- Specify scope when needed: "inspect personal equipment for damage" rather than "inspect gear" (scope unclear)

Do not include any other text, introductory, explanatory, markdown, etc.
End with </end>.
"""
    other_characters_str = ''
    for other_character_name, other_character_desc in other_characters.items():
        if other_character_name != character_name:
            other_characters_str += f"\n\t{other_character_name}: {other_character_desc}"

    if llm_client:
        response = llm_client.generate(
                    messages=[prompt],
                    bindings={"ontology": json.dumps(ontology, indent=2), "character": character, "other_characters": other_characters_str  },
                    max_tokens=6000,
                    temperature=0.7,
                    stops=['</end>'],
                    is_json=True
                )

    if response.success:
        print(f'🤖  Activities: \n{response.text}')
        return response.text
    else:
        print(f'❌ Failed to get activities: {response.error}')
        return None

def load_scenario(scenario_path):
    """Load scenario YAML file and return its contents."""
    try:
        with open(scenario_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception as e:
        print(f"❌ Error loading scenario file {scenario_path}: {e}")
        return None

def save_activities(character_name, activities, scenario_dir):
    """Save activities to a JSON file in the scenarios directory."""
    filename = f"{character_name}-activities.json"
    filepath = os.path.join(scenario_dir, filename)
    try:
        with open(filepath, 'w') as file:
            json.dump(activities, file, indent=2)
        print(f"✅ Saved activities for {character_name} to {filepath}")
    except Exception as e:
        print(f"❌ Error saving activities for {character_name}: {e}")

class ActivityManager:
    def __init__(self, executive_node: ZenohExecutiveNode, activities: dict, llm_client: ZenohLLMClient, map_types: dict):
        self.executive_node = executive_node
        self.activities = activities
        self.llm = llm_client
        self.map_types = map_types
        self.current_activity = None
        self.current_activity_state = None
        self.current_activity_next_step = None
        self.activity_history = []
        self.previous_activity = None
        
        # Time advancement management
        self.time_advanced_subscriber = self.executive_node.session.declare_subscriber(
            "cognitive/map/time_advanced",
            self.time_advanced_callback
        )
        self.current_time = None  # Will be set when we receive time updates
        
    def _format_situation_data(self):
        """Build structured situation data for activity selection"""
        situation = {}
        
        # Use the same data source as format_situation: last_situation_data
        if hasattr(self.executive_node, 'last_situation_data') and self.executive_node.last_situation_data:
            last_data = self.executive_node.last_situation_data
            
            # Extract location information
            if last_data.get('location'):
                situation['current_location'] = {
                    'terrain': last_data['views'][0]['terrain'],  # This is the terrain string
                    'coordinates': last_data['location']  # Default coordinates
                }
            
            # Extract visible characters
            if last_data.get('visible_characters'):
                situation['characters'] = last_data['visible_characters']
            else:
                situation['characters'] = []
            
            # Extract look/visible areas
            if last_data.get('look'):
                situation['visible_areas'] = []
                for item in last_data['look']:
                    # Parse the look data into structured format
                    # Assuming look items are strings like "mushroom29 (5 away)"
                    situation['visible_areas'].append({
                        'resources': [{'name': item, 'distance': 5}],  # Default distance
                        'direction': 'current'
                    })
            else:
                situation['visible_areas'] = []
            
            # Extract adjacent information
            if last_data.get('adjacent_to'):
                adjacent = last_data['adjacent_to']
                if adjacent.get('resources'):
                    situation['adjacent_resources'] = adjacent['resources']
                else:
                    situation['adjacent_resources'] = []
                
                if adjacent.get('characters'):
                    situation['adjacent_characters'] = adjacent['characters']
                else:
                    situation['adjacent_characters'] = []
            else:
                situation['adjacent_resources'] = []
                situation['adjacent_characters'] = []
            
            # Extract views if available
            if last_data.get('views'):
                situation['views'] = last_data['views']
            else:
                situation['views'] = {}
            entity_context = self.executive_node.get_entity_context(self.executive_node.character_name, 10)
            situation['thoughts'] = 'No thoughts available'
            if entity_context:
                thoughts = f'\n#Your most recent thoughts include:'
                for i, memory in enumerate(entity_context['conversation_history']):  # Use last 2 memories
                    thoughts += f"\n\t{memory['source']}: {memory['text']}"
                thoughts += '\n'
                situation['thoughts'] = thoughts
            # get inventory
            situation['inventory'] = []
            for reply in self.executive_node.session.get(f"cognitive/{self.executive_node.character_name}/memory/inventory", timeout=2.0 if not self.executive_node.debug else 600.0):
                if reply.ok:
                    data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                    if data.get('success'):
                        value = data.get('value', [])
                        if isinstance(value, list):
                            situation['inventory'].extend(value)
                        elif value:
                            situation['inventory'].append(value)
 
        else:
            # Default values if no situation data available
            situation['current_location'] = {'terrain': 'unknown', 'coordinates': [0, 0]}
            situation['characters'] = []
            situation['visible_areas'] = []
            situation['adjacent_resources'] = []
            situation['adjacent_characters'] = []
            situation['views'] = {}
        
        # Get simulation time from map_node
        try:
            for time_reply in self.executive_node.session.get("cognitive/map/simulation_time", timeout=70.0):
                if time_reply.ok:
                    time_data = json.loads(time_reply.ok.payload.to_bytes().decode('utf-8'))
                    if time_data.get('success'):
                        situation['time_info'] = time_data.get('time_info', {})
                        situation['weather'] = time_data.get('weather', 'unknown')
                        situation['datetime'] = datetime.fromisoformat(situation['time_info'].get('datetime')) if situation['time_info'].get('datetime') else datetime.now()
                        break
                    else:
                        logger.error(f"Failed to get time data: {time_data}")
                        situation['time_info'] = {'period': 'unknown', 'season': 'unknown'}
                        situation['weather'] = 'unknown'
                        situation['datetime'] = datetime.now()
                else:
                    logger.error(f"Failed to get time data: {time_reply}")
                    situation['time_info'] = {'period': 'unknown', 'season': 'unknown'}
                    situation['weather'] = 'unknown'
                    situation['datetime'] = datetime.now()
        except Exception as e:
            logger.warning(f"Failed to get time data: {e}")
            situation['time_info'] = {'period': 'unknown', 'season': 'unknown'}
            situation['weather'] = 'unknown'
        
        # Set default values for fields that don't exist in the data
        situation['threats'] = []  # No threats data available
        situation['injuries'] = []  # No injuries data available
        
        # Get inspection data
        if hasattr(self.executive_node, 'inspections'):
            situation['inspections'] = self.executive_node.inspections
        else:
            situation['inspections'] = {}
        
        # Get plan summary if available
        if hasattr(self.executive_node, 'plan_summary'):
            situation['plan_summary'] = self.executive_node.plan_summary
        else:
            situation['plan_summary'] = None
        
        return situation
        
    def has_active_activity(self) -> bool:
        """Check if an activity is currently in progress"""
        return self.current_activity is not None and self.current_activity_state is not None
        
    def select_activity(self, active_goal: dict=None):
        """Select NEW activity only - returns (first_step, activity) tuple"""
        # Only called when no current activity
        if self.current_activity:
            logger.warning(f"select_activity called but {self.executive_node.character_name} already has active activity")
            return None, None
        
        # Build structured situation data
        situation = self._format_situation_data()
        
        # Check if we have activities to select from
        if self.activities is None:
            return None, None
        
        if self.current_activity_state:
            self.previous_activity_state = self.current_activity
        # 1. Algorithmic pre-filtering
        candidates = self._pre_filter(situation)
        if len(candidates) > 7:
            candidates = random.sample(candidates, 7)
        
        # 2. LLM contextual scoring  
        candidates = self._score_contextual_alignment(candidates, situation, active_goal)
        
        # 3. Final ranking and selection
        selected_activity = self._rank_and_select(candidates)
        if not selected_activity:
            # No suitable activity found - use default fallback
            logger.info(f"🔄 No suitable activities found for {self.executive_node.character_name}, using fallback")
            selected_activity = DEFAULT_ACTIVITY
        
        # Initialize the new activity
        instantiated_activity = self._instantiate_activity(selected_activity, situation)
        self.current_activity = instantiated_activity
        self.current_activity_state = {
            'start_time': situation.get('datetime', 'unknown'),
            'current_step_index': 0,
            'status': 'active'
        }
        
        # Return first step of the new activity
        first_step = self.activity_step()
        logger.info(f"🎯 {self.executive_node.character_name} selected new activity: {instantiated_activity['name']}")
        return first_step, self.current_activity
    
    def _pre_filter(self, situation):
        """Fast algorithmic filtering"""
        time_info = situation['time_info']
        candidates = []
        
        for activity in self.activities.values():
            # Create a copy to avoid modifying the original
            activity_copy = activity.copy()
            
            # Temporal compatibility
            if not self._time_compatible(activity_copy, time_info):
                continue
                
            # Location compatibility
            if not self._location_compatible(activity_copy, situation):
                continue
                
            # Basic resource availability
            if not self._resources_available(activity_copy, situation):
                continue
                
            # Cooldown check (if recently completed)
            if self._in_cooldown(activity_copy, time_info):
                continue
                
            candidates.append(activity_copy)
            
        logger.info(f"Pre-filtered activities: {len(candidates)}")
        return candidates
    
    def _time_compatible(self, activity, current_time):
        """Check if activity timing matches current time"""
        when = activity.get('when', None)
        if not when:
            when = 'opportunistic'
        current_period = current_time['period']  # morning, midday, evening, night
        current_season = current_time['season']  # winter, spring, summer, autumn
        
        if when == 'opportunistic':
            return True
        elif when.startswith('daily@'):
            target_time = when.split('@')[1]
            return current_period == target_time
        elif when.startswith('seasonal@'):
            target_season = when.split('@')[1] 
            return current_season == target_season
        
        return False
    
    def _location_compatible(self, activity, situation):
        """Check if current location supports activity"""
        current_terrain = situation.get('current_location', {}).get('terrain', 'unknown')
        views = situation.get('views', {})
        resources = []
        for view in views:
            if view['direction'] == 'Current':
                resources = view.get('resources', [])
                break
        at_resource_locations = []
        for resource in resources:
            at_resource_locations.append(resource.get('name'))
        valid_locations = activity.get('where', [])
        compatible = current_terrain in valid_locations
        if not compatible:
            for resource in at_resource_locations:
                if resource in valid_locations:
                    compatible = True
                    break
        return compatible
    
    def _resources_available(self, activity, situation):
        """Check if needed resources are available"""
        needs = activity['needs']
        # Basic needs like 'hands', 'feet', 'brain', 'eyes' always available
        basic_capabilities = {'hands', 'feet', 'brain', 'eyes'}
        resource_types = self.map_types.get('resource_types', [])
        
        for need in needs:
            if need in basic_capabilities:
                continue
            if need not in resource_types:
                continue
            if not self.executive_node._resolve_visible_instance(False, need) \
                and not self.executive_node._resolve_inventory_instance(False, need):
                return False
        return True
    
    def _in_cooldown(self, activity, current_time):
        """Check if activity was recently completed (simplified)"""
        if len(self.activity_history) > 0 and self.activity_history[-1]['name'] == activity['name']:
            return True
        if len(self.activity_history) > 1 and self.activity_history[-2]['name'] == activity['name']:
            return True
        # Implement based on your execution history tracking
        return False
    
    def _score_contextual_alignment(self, candidates, situation, active_goal):
        """Use LLM for complex contextual scoring"""
        for activity in candidates:
            activity['alignment_score'] = self._score_activity_contextual_alignment(activity, situation)
        
        return candidates
    
    def _instantiate_activity(self, activity, situation):
        """Instantiate an activity"""
        return activity
        prompt = f"""Your task is to instantiate the set of steps for this activity, for this character, in this situation.

#Character: {self.executive_node.character_name}\n
{self.executive_node.character_config['character']}\n
##
#Drives: {self.executive_node.character_config['drives']}\n
###
Inventory:\n\t{'\n\t'.join(situation.get('inventory', []))}\n
Situation: {situation}\n
Activity: \n{json.dumps(activity, indent=2)}
Steps:\n{'\n\t'.join(activity['steps'])}

'Instantiate' means make a general step into a specific goal in this context, 
for example by naming specific characters or resource instances already known to the character.
Return ONLY the JSON object with the instantiated steps.
Do not include any other text, introductory, explanatory, markdown, etc.
End with </end>.
Instantiate the steps for this activity:
        """
        response = self.llm.generate([prompt],max_tokens=100,temperature=0.7,is_json=True, stops=['</end>'])
        if response.success:
            return response.text
        else:
            return activity
    
    def _score_activity_contextual_alignment(self, activity, situation):
        """LLM evaluates how well activity serves character drives"""
        prompt = f"""You are considering the following activity: 
        
#Activity: {activity['name']}
{json.dumps(activity, indent=2)}\
##

In the following context:
#Context:
Character: {self.executive_node.character_name}\n
{self.executive_node.character_config['character']}\n
Drives: {self.executive_node.character_config['drives']}\n
Other characters present: {situation.get('characters', [])}\n

Resources available: {self._summarize_resources(situation)}
Time: {situation.get('time_info', {})}
Weather: {situation.get('weather', 'unknown')}
Location: {situation.get('current_location', {})}
Visible resources: {self._summarize_resources(situation)}

#You have two tasks:
#Your first task is to evaluate how well this activity serves the character's drives.
Character drives: {self.executive_node.character_config['drives']}

Rate 0-1 how well this activity serves the character's drives.
If the activity serves all the character's drives completely, return a drive score of 1.0.
If the activity serves none of the character's drives in any way, return a drive score of 0.0.
Otherwise, compute a drive score between 0.0 and 1.0 based on how well the activity serves the character's drives.
"""

        prompt += f"""##

#Your second task is to rate the consistency of this activity with the overall current situation.

If the fit is perfect, return a consistency score of 1.0.
If the activity is inconsistent with the situation and totally inappropriate, return a consistency score of 0.0.
Otherwise, compute a consistency score between 0.0 and 1.0 based on degree of fit.
      
Do not show any reasoning steps or thinking or chain-of-thought. 
Return the two scores using the following hash-formatted text, where each tag is preceded by a # and followed by a single space, followed by its content:
#drive 0.0-1.0
#consistency 0.0-1.0
##

Do not include any other text, introductory, explanatory, markdown, etc.
End with </end>.
"""
        response = self.llm.generate([prompt],max_tokens=100,temperature=0.7, stops=['</end>'])
        if response.success:
            response_text = response.text
            drive = hash_utils.find('drive', response_text)
            try:
                drive = float(drive)
            except:
                drive = 0.5
            consistency = hash_utils.find('consistency', response_text)
            try:
                consistency = float(consistency)
            except:
                consistency = 0.5
            return {'drive': drive, 'consistency': consistency}
        else:
            return {'drive': 0.5, 'consistency': 0.5}
    
    def _rank_and_select(self, candidates):
        """Final scoring and ranking"""
        for activity in candidates:
            # Weighted combination of scores
            activity['final_score'] = (
                activity['importance'] * 0.30 +           # Base importance
                activity['habit'] * 0.15 +                # Habit strength  
                activity['alignment_score']['drive'] * 0.30 +          # Drive alignment
                activity['alignment_score']['consistency'] * 0.20 +          # Situational fit
                0.05 if 'social' in activity['tags'] else 0.00     # Social stuff is important!
            )
        
        # Sort by final score
        ranked = sorted(candidates, key=lambda a: a['final_score'], reverse=True)
        
        # Return top choice, or None if no good candidates
        return ranked[0] if ranked and ranked[0]['final_score'] > 0.3 else None
    
    def _summarize_resources(self, situation):
        """Helper to summarize visible resources for LLM"""
        resources = []
        for direction_data in situation['views']:
            for resource in direction_data.get('resources', []):
                resources.append(f"{resource['name']} ({resource['distance']} away)")
        return resources[:20]  # Limit for prompt brevity

    def continue_activity(self):
        """Decide whether current activity should continue or stop"""
        if not self.current_activity:
            return False, "No current activity"
        
        situation = self._format_situation_data()
        
        # Check if activity has natural completion conditions
        if self._activity_completed(self.current_activity, situation):
            return False, "Activity completed naturally"
        
        # Check if stop conditions are met
        if self._stop_conditions_met(self.current_activity, situation):
            return False, "Stop conditions met"
        
        # Check if activity should be interrupted by higher priority goals
        if self._should_interrupt(situation):
            return False, "Interrupted by higher priority goal"
        
        # Check if activity duration limits are reached
        if self._duration_exceeded(situation):
            return False, "Duration limit reached"
        
        # Check if character needs are satisfied
        if self._needs_satisfied(situation):
            return False, "Character needs satisfied"
        
        # Activity should continue
        return True, "Activity continuing"
    
    def _activity_completed(self, activity, situation):
        """Check if activity has reached its natural completion"""
        # Check if all steps are completed
        if hasattr(activity, 'completed_steps') and len(activity['completed_steps']) >= len(activity['steps']):
            return True
        
        # Check if target conditions are met
        if 'target_condition' in activity:
            return self._evaluate_condition(activity['target_condition'], situation)
        
        return False
    
    def _stop_conditions_met(self, activity, situation):
        """Check if any stop conditions are met"""
        if 'stop' not in activity:
            return False
        
        for stop_condition in activity['stop']:
            if self._evaluate_condition(stop_condition, situation):
                return True
        
        return False
    
    def _should_interrupt(self, situation):
        """Check if activity should be interrupted by higher priority concerns"""
        # Check if there's an emergency or urgent situation
        if self._emergency_detected(situation):
            return True
        
        # Check if character's drives are better served by stopping
        if self._drives_better_served_stopping(situation):
            return True
        
        return False
    
    def _duration_exceeded(self, situation):
        """Check if activity has exceeded its duration limits"""
        if 'duration' not in self.current_activity:
            return False
        
        if not self.current_activity_state or 'start_time' not in self.current_activity_state:
            return False
        
        min_duration, max_duration = self.current_activity['duration']
        current_time = situation['datetime']
        current_duration = current_time - self.current_activity_state['start_time']
        if type(max_duration) == str and all(c.isdigit() for c in max_duration):
            max_duration = int(max_duration)
        if max_duration < 7*len(self.current_activity['steps']):
            max_duration = 7*len(self.current_activity['steps'])
        if type(max_duration) == int:
            max_duration = timedelta(minutes=max_duration)
        if type(max_duration) != timedelta:
            return False
        if max_duration and current_duration > max_duration:
            return True
        
        return False
    
    def _needs_satisfied(self, situation):
        """Check if the character's needs that led to this activity are satisfied"""
        # This is a simplified check - in practice, you'd track what need the activity was addressing
        if 'addressing_need' in self.current_activity:
            need = self.current_activity['addressing_need']
            return self._need_satisfied(need, situation)
        
        return False
    
    def _evaluate_condition(self, condition, situation, time_info=None):
        """Evaluate a condition string against current situation"""
        # Simple condition evaluation - could be enhanced with LLM reasoning
        if 'weather' in condition and 'weather' in situation:
            return condition in str(situation['weather'])
        if 'time' in condition and time_info:
            return condition in str(time_info.get('period', ''))
        if 'location' in condition and 'current_location' in situation:
            return condition in str(situation['current_location'])
        
        return False
    
    def _emergency_detected(self, situation):
        """Check if there's an emergency requiring immediate attention"""
        # Check for immediate threats, injuries, or urgent situations
        threats = situation.get('threats', [])
        injuries = situation.get('injuries', [])
        
        return len(threats) > 0 or len(injuries) > 0
    
    def _goal_urgency_increased(self, goal, situation):
        """Check if the active goal has become more urgent"""
        # This would depend on your goal urgency tracking system
        return False  # Placeholder
    
    def _drives_better_served_stopping(self, situation):
        """Check if stopping the activity would better serve character drives"""
        # This could use LLM reasoning to evaluate drive satisfaction
        return False  # Placeholder
    
    def _need_satisfied(self, need, situation):
        """Check if a specific need is satisfied"""
        # This would depend on your need tracking system
        return False  # Placeholder
       
    def activity_step(self):
        """Return the current step to perform, or None if no activity running"""
        if not self.current_activity or not self.current_activity_state:
            return None
        
        steps = self.current_activity.get('steps', [])
        current_index = self.current_activity_state['current_step_index']
        
        if current_index >= len(steps):
            return None

        step = {
            'name': steps[current_index],
            'description': steps[current_index],
            'actors': [self.executive_node.character_name],
            'termination': f'{steps[current_index]} completed'
        }
        return step
    
    def step_completion(self, status, reason=None):
        """Handle step completion - success advances, failure abandons activity"""
        if not self.current_activity or not self.current_activity_state:
            return None, None
        
        if status == 'success':
            # Advance to next step
            self.current_activity_state['current_step_index'] += 1
            
            # Check if activity is complete
            steps = self.current_activity.get('steps', [])
            if self.current_activity_state['current_step_index'] >= len(steps):
                self._complete_activity()
                return None, None  # Activity completed
            else:
                # Activity continues - return next step
                next_step = self.activity_step()
                return next_step, self.current_activity
        else:
            # Step failed - abandon activity
            self._abandon_activity(reason)
            return None, None
    
    def _complete_activity(self):
        """Mark activity as successfully completed"""
        if not self.current_activity or not self.current_activity_state:
            return
        
        # Store in history
        history_entry = {
            'name': self.current_activity['name'],
            'steps_completed': len(self.current_activity.get('steps', [])),
            'start_time': self.current_activity_state['start_time'],
            'status': 'completed'
        }
        self.activity_history.append(history_entry)
        
        # Clear current activity
        self.current_activity = None
        self.current_activity_state = None
    
    def _abandon_activity(self, reason):
        """Abandon current activity due to step failure"""
        if not self.current_activity or not self.current_activity_state:
            return
        
        # Store in history
        history_entry = {
            'name': self.current_activity['name'],
            'steps_completed': self.current_activity_state['current_step_index'],
            'start_time': self.current_activity_state['start_time'],
            'status': 'abandoned',
            'reason': reason
        }
        self.activity_history.append(history_entry)
        
        # Clear current activity
        self.current_activity = None
        self.current_activity_state = None
    
    def time_advanced_callback(self, sample):
        """Handle time advancement notifications from map_node."""
        try:
            data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            self.time_info = data.get('new_time_info', {})
            
            if self.time_info and 'datetime' in self.time_info:
                # Store the current simulation time
                self.current_time = datetime.fromisoformat(self.time_info['datetime'])
                logger.info(f'⏰ ActivityManager for {self.executive_node.character_name} received time update: {self.current_time}')
            else:
                logger.warning(f'ActivityManager received time_advanced but no valid time_info in data')
                
        except Exception as e:
            logger.error(f'Error in ActivityManager time_advanced callback: {e}')
    
    def get_next_step_or_activity_name(self):
        """
        If there is a current activity and it is active and there is a current_step,
        returns the next step if there is one, or the activity['name'] if not.
        If any of the conditions fail, returns None.
        The current_step pointer is NOT advanced.
        """
        # Check if there is a current activity
        if not self.current_activity:
            return None
        
        # Check if the activity is active
        if not self.current_activity_state or self.current_activity_state.get('status') != 'active':
            return None
        
        # Check if there is a current_step
        current_step_index = self.current_activity_state.get('current_step_index')
        if current_step_index is None:
            return None
        
        steps = self.current_activity.get('steps', [])
        
        # Check if there is a next step
        if current_step_index + 1 < len(steps):
            # Return the next step name
            return steps[current_step_index + 1]
        else:
            # No next step, return activity name
            return self.current_activity['name']
    
    def get_default_activity(self, character, situation):
        """Get a default activity for a character in a situation"""
        prompt = f"""
        Character: {character['character']} with drives: {character['drives']}
        Situation: {situation}
        """
        return self.llm.generate(prompt)
    
# Usage example:
def run_activity_selection(character, situation, time_info, activities):
    manager = ActivityManager(activities, llm_client)
    selected_activity = manager.select_activity(character, situation, time_info)
    
    if selected_activity:
        print(f"Selected activity: {selected_activity['name']}")
        return selected_activity
    else:
        print("No suitable activity found - falling back to default behavior")
        return manager.get_default_activity(character, situation)

def main():
    global map_types
    parser = argparse.ArgumentParser(description='Generate activities for characters in a scenario')
    parser.add_argument('scenario', help='Path to scenario YAML file')
    args = parser.parse_args()
    
    # Load scenario file
    scenario = load_scenario(args.scenario)
    if not scenario:
        return
    
    # Get setting from scenario (default to empty string if not present)
    setting = scenario.get('setting', '')
    if not setting:
        print("⚠️  No 'setting' parameter found in scenario file")
        setting = "modern era, western, moderate climate"
    
    # Get characters from scenario
    characters = scenario.get('characters', {})
    if not characters:
        print("❌ No characters found in scenario file")
        return
    
    print(f"📖 Processing scenario: {args.scenario}")
    print(f"🌍 Setting: {setting}")
    print(f"👥 Characters: {', '.join(characters.keys())}")
    
    # Initialize LLM client
    global llm_client
    config = zenoh.Config()
    session = zenoh.open(config)
    llm_client = ZenohLLMClient(service_timeout=240.0)
    
        # Launch map node (required for situation awareness)
    try:
        map_args = [sys.executable, 'map_node.py']
        map_file = scenario.get('map', None)
        if map_file:
            # Decision on reuse/new world is handled early in main()
            world_name = map_file.replace('.py', '')
            map_args.extend(['-m', map_file])
        # Propagate optional debug flag to disable map turn timeouts
        env = os.environ.copy()
        if env.get('CWB_DEBUG', ''):
            logger.info('Debug mode enabled via CWB_DEBUG - map turn timeout will be disabled')
        map_process = subprocess.Popen(map_args, env=env)
        shared_processes.append(map_process)
        logger.info(f'✅ Map Node launched' + (f' with map: {map_file}' if map_file else ''))
            
        # Wait for map node to be ready (check for initialization message)
        logger.info('⏳ Waiting for Map Node to initialize...')
        time.sleep(5)  # Give map node time to start up
    except Exception as e:
        logger.error(f'❌ Failed to launch Map Node: {e}')

    if not map_types:
        for reply in session.get("cognitive/map/types", timeout=25.0):
            if reply.ok:
                data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                if data.get('success'):
                    map_types = data
                    break

    # Get scenario directory for output files
    scenario_dir = os.path.dirname(args.scenario)
    
    # Process each character
    for character_name, character_data in characters.items():
        print(f"\n🎭 Processing character: {character_name}")
        
        # Skip manual characters
        if character_data.get('manual', False):
            print(f"⏭️  Skipping manual character: {character_name}")
            continue
        
        # Get character description
        character_desc = character_data.get('character', '')
        if not character_desc:
            print(f"⚠️  No character description for {character_name}, skipping")
            continue
        
        # Get ontology
        print(f"🔍 Getting ontology for {character_name}...")
        ontology = get_ontology_from_zenoh(setting, character_name, character_desc, characters)
        if not ontology:
            print(f"❌ Failed to get ontology for {character_name}, skipping")
            continue
        
        # Get activities
        print(f"🎯 Getting activities for {character_name}...")
        activities = get_activities(setting, character_name, character_desc, characters, ontology)
        if not activities:
            print(f"❌ Failed to get activities for {character_name}, skipping")
            continue
        
        # Save activities to file
        save_activities(character_name, activities, scenario_dir)
    
    print(f"\n✅ Finished processing scenario: {args.scenario}")

if __name__ == "__main__":
    shared_processes = []
    try:
        llm_cmd = [sys.executable, 'llm_service_node.py', '--server-name', 'vllm', '--model-name', 'anthropic/claude-sonnet-4']
        #if model_name:
        #    llm_cmd.extend(['--model-name', model_name])
            
        llm_process = subprocess.Popen(llm_cmd)
        shared_processes.append(llm_process)
        time.sleep(5)
        print(f'✅ LLM Service Node launched with server: vllm')
    except Exception as e:
        logger.error(f'❌ Failed to launch LLM Service Node: {e}')

    # Run main function
    main()
    
    print('finished ontology generation')
    for process in shared_processes:
        process.terminate()
    print('terminated shared processes')