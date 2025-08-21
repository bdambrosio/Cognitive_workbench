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
from activity import ActivityManager
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

PLAN_SYNTAX = """
Task: Break down the user's high‑level goal into a minimal plan in the JSON format specified below.
Output: only valid JSON – no prose, no code fences.

{
  "plan": [
    { "type": "move", "target": "…"},
    { "type": "say", "target": "…", "value": "…" },
    { "type": "think", "value": "…" },
    { "type": "take", "target": "…"},
    { "type": "inspect", "target": "…", "reason": "…"},
    { "type": "use", "target": "…", "reason": "…"},
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
 - "inspect": { "type": "inspect", "target": "resource_name", "reason": "what is it you are hoping to learn? - 5 words max"} 
     For inspecting a resource you see or one in your inventory to understand how to use it.
 - "use": { "type": "use", "target": "resource_name", "reason": "what outcome do you hope to achieve? - 5 words max"} 
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
