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
from utils.zenoh_utils import datetime_handler
import plan as plan_module
from dataclasses import dataclass
import os

# Configure logging with unbuffered output
# Console handler with WARNING level (less verbose)
console_handler = logging.StreamHandler(sys.stdout)
console_handler.setLevel(logging.WARNING)

# File handler with INFO level (full logging)
file_handler = logging.FileHandler('logs/executive_node.log', mode='w')
file_handler.setLevel(logging.WARNING)
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

# Import LLM client
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__))))

try:
    from llm_client import ZenohLLMClient
    LLM_CLIENT_AVAILABLE = True
except ImportError as e:
    print(f"⚠️  LLM Client not available: {e}")
    LLM_CLIENT_AVAILABLE = False

ONTOLOGY_TEMPLATE = """You are creating an ontology for everyday activities. Attend especially to the era, setting, and locale, these will often be fantasy or historical settings.
INPUT:
#Setting: a string including era, locale, climate, tech, norms:
    {{$setting}}
##

#Additional information about the setting:
    {{$map_types}}
##
    
#Character personality:
{{$character}}
##

#Character drives:    
{{$character_drives}}
##
    
# Other Characters:
    {{$other_characters}}
##

Include mental, physical, and social 'activities' that are possible and consisten with the character in this setting in your ontology.

OUTPUT (JSON):

- places[] - places the character can be in the setting - map terrain types or resource types only
- tools[] - tools the character can use in the setting - resource types or items from character status or inventory
- roles[] - roles the character can adopt in the setting
- norms_rules[] - norms and rules the character must follow in the setting - character personality or map norms
- hazards[] - hazards the character must avoid in the setting - map terrains, resources, hazards, or specific other characters
- social_graph[] - social graph of the character in the setting - character personality or map social graph
- time_windows - Either:
    - One of: dawn, morning, afternoon, dusk, evening, night
- affordances - e.g. place->[activities_possible]

respond only with the JSON, no other text.
end your response with </end>
"""
llm_client = None

ACTIVITIES_TEMPLATE = """You are generating a comprehensive activity list for a character. 
CHARACTER: 
{{$character}}
{{$character_drives}}
{{$map_types}}
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
  "where": ["place ids"], - drawn from ontology places
  "duration": [min_minutes, max_minutes],
  "needs": ["tools or innate capacities"], - drawn from ontology tools or character status or inventory
  "steps": ["3–5 terse steps that will serve as planning goals"],
  "importance": 0.0,
  "habit": 0.0
}

The following ontology must be used to fill in the details of activities:
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

GOAL_TEMPLATE = """What is the most relevant thing you should work on next? 
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

PLAN_TEMPLATE = """Task: Decompose down your current goal into a minimal plan in the JSON format specified below.
Output: only valid JSON – no prose, no code fences.

{
  "plan": [
    { "type": "move", "target": "…"},
    { "type": "say", "target": "…", "value": "…" },
    { "type": "scan", "target": "resource_type", "out": "variable_name to assign the scan result to" },
    { "type": "wait", "condition": "condition_name", "target": "condition_target" },
    { "type": "think", "value": "…" },
    { "type": "take", "target": "…"},
    { "type": "inspect", "target": "…", "reason": "…"},
    { "type": "use", "target": "…", "reason": "…"},
    { "type": "while", "condition": "…" , "body": [ /* steps */ ]},
    { "type": "if", "condition": "…", "then": [ /* steps */ ], "else": [ /* steps */ ] }
  ]
}

Example workflow using scan and variables:
{
  "plan": [
    { "type": "scan", "target": "Berries", "out": "found_berries" },
    { "type": "move", "target": "$found_berries" },
    { "type": "take", "target": "$found_berries" }
  ]
}

A plan must include at least 1 step and no more than 6 steps including all nested while and if branches.
In the following, <resource_name>, <character_name> are placeholders only for KNOWN resources, characters, or maptypes, those appearing above.
Variables bound by scan actions can be referenced in subsequent actions and conditions using $variable_name syntax.
Only dicts of the types below are allowed for the condition of while and if. Condition action type can only be one of the following:
 - "near": {"type": "near", "target": <resource name> or <character_name> or "$variable_name"} is for checking if the character is near a resource or character.
 - "can_see": {"type": "can_see", "target": <character_name> or "$variable_name"} is for checking if the character can see a character.
 - "has_item": {"type": "has_item", "target": <resource_name> or "$variable_name"} is for checking if the character has a resource in their inventory.
 - "at_location": {"type": "at_location", "target": <location_name>} is for checking if the character is at a location.
 - "believes": {"type": "believes", "target": <character_name>} is for checking if the character believes something about another character.
 - "notnear": {"type": "notnear", "target": <resource name> or <character_name> or "$variable_name"} is for checking if the character is not near a resource or character.
 - "cant_see": {"type": "cant_see", "target": <character_name> or "$variable_name"} is for checking if the character cannot see a character.
 - "hasnt_item": {"type": "hasnt_item", "target": <resource_name> or "$variable_name"} is for checking if the character does not have a resource in their inventory.
 - "notat_location": {"type": "notat_location", "target": <location_name> or "$variable_name"} is for checking if the character is not at a location.
 - "notbelieves": {"type": "notbelieves", "target": <character_name>} is for checking if the character does not believe something about another character.

outside a while or if condition, "type" can take the values "say", "move", "think", "take", "inspect", "use", or "scan":
 - "move": { "type": "move", "target": "cardinal_direction" or 'resource or character name'} 
     Move one step in one of the 8 cardinal directions or in the direction of a resource or character.
    You can only move in the direction of a resource, character, or terrain type if you can see it.
 - "say": { "type": "say", "target": "character_name", "value": "text to speak" } 
     for speaking to another character you can see. Use this to seek information, respond, inform the other character, or to maintain 'social chatter' to stay aligned.
     For a 'say' act, speak only for yourself, and do not include any other introductory, explanatory, discursive, or formatting text in your response.
 - "scan": { "type": "scan", "target": "resource_type", "out": "variable_name to assign the scan result to" }
     Scan a resource type to find the first matching instance in your current situation.
 - "wait": { "type": "wait", "condition": "condition_name", "target": "condition_target" }
     Wait for a condition to be true. The condition must be one of the Condition actions listed earlier.
 - "think": { "type": "think", "value": "text to think about" } 
     Think about a topic or question, attempting to derive new information, conclusions, or decisions from who you are and what you already explicitly know
 - "take": { "type": "take", "target": "resource_name" } 
     Add a resource you to your personal inventory. you must be near the resource to take it.
 - "inspect": { "type": "inspect", "target": "resource_name", "reason": "what is it you are hoping to learn? - 5 words max"} 
     Inspect a resource or character to learn something about it. Must be 'near' the resource or character to inspect it. reason focuses the inspection on some specific aspect of the resource or character.
 - "use": { "type": "use", "target": "resource_name", "reason": "what outcome do you hope to achieve? - 5 words max"} 
     Using a resource in your inventory. You may reuse resources.You must be 'near' the resource to use it. You may want to inspect the resource first to learn the effect of using it.

A plan must include no more than 8 steps including all nested while and if branches.
A plan must not contain sequential say actions.
"""