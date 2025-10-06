from __future__ import annotations

from enum import Enum
import random
import subprocess
import time
import argparse
import traceback
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
from utils import hash_utils
from utils.state_utils import calculate_state_activity_alignment, get_known_states
from utils.format_utils import format_middle_nouns, format_middle_verbs
# Type checking imports
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from executive_node import ZenohExecutiveNode

# Configure logging with unbuffered output
# Console handler with WARNING level (less verbose)
console_handler = logging.StreamHandler(sys.stdout)
console_handler.setLevel(logging.INFO)

# File handler with INFO level (full logging)
file_handler = logging.FileHandler('logs/discourse.log', mode='w')
file_handler.setLevel(logging.INFO)

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
    handlers=[console_handler, file_handler],
    force=True
)
logger = logging.getLogger('discourse')

# Add dedicated handler for llm_api logger
llm_api_logger = logging.getLogger('llm_api')
llm_api_file_handler = logging.FileHandler('logs/llm_api.log', mode='a')
llm_api_file_handler.setLevel(logging.INFO)
llm_api_file_handler.setFormatter(logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s', '%Y-%m-%d %H:%M:%S'))
llm_api_logger.addHandler(llm_api_file_handler)
llm_api_logger.setLevel(logging.INFO)

# Add dedicated handler for llm_client logger
llm_client_logger = logging.getLogger('llm_client')
llm_client_file_handler = logging.FileHandler('logs/llm_client.log', mode='a')
llm_client_file_handler.setLevel(logging.INFO)
llm_client_file_handler.setFormatter(logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s', '%Y-%m-%d %H:%M:%S'))
llm_client_logger.addHandler(llm_client_file_handler)
llm_client_logger.setLevel(logging.INFO)
from llm_client import ZenohLLMClient

map_types = None


DISCOURSE_TEMPLATE="""You are analyzing a conversation to identify discourse objects - the structured elements that speakers create through dialogue.

CONVERSATION SEGMENT:
{{$recent_turns}}

TASK: Extract all discourse objects created in this exchange. For each object, provide:

1. OBJECT_ID: unique identifier (format: obj_[timestamp]_[type]_[number])
2. TYPE: exactly one of:
   - commitment: promise, offer, pledge, agreement to do something
   - question: explicit question or implicit uncertainty needing resolution
   - position: stance, claim, or assertion about how things are/should be
   - topic: subject matter introduced for discussion
   - proposal: suggestion for action or decision
   - agreement: mutual acceptance of a proposition or plan
   - disagreement: explicit rejection or contradiction
   - request: ask for information or action (not a question about the world)
   - information: factual information or data

3. CREATOR: who introduced it (format: "self" or "other" or "joint")

4. TURN_NUMBER: which turn introduced it (integer)

5. CONTENT_SUMMARY: concise description of the object (1-2 sentences max)

6. DIRECTED_AT: who is the object addressed to ("self", "other", "mutual", "none")

7. STATUS: current state:
   - pending: awaiting response/resolution
   - accepted: acknowledged/agreed by recipient
   - rejected: explicitly declined/disagreed
   - resolved: question answered, issue settled
   - active: commitment in force, topic under discussion
   - abandoned: dropped without resolution
   
8. DEPENDENCIES: list of OBJECT_IDs this depends on (empty list if none)

9. ATTRIBUTES: type-specific fields:
   For commitments:
     - temporal_scope: when (immediate/by_date/ongoing/conditional)
     - strength: confidence level (firm/tentative/conditional)
   For questions:
     - answer_type: what kind of answer (yes_no/factual/explanation/preference)
   For positions:
     - evidence_provided: was justification given (true/false)
   For proposals:
     - decision_required: does this need explicit acceptance (true/false)

OUTPUT FORMAT (valid JSON only):
{
  "objects": [
    {
      "object_id": "obj_1696534821_commitment_1",
      "type": "commitment",
      "creator": "self",
      "turn_number": 3,
      "content_summary": "Will deliver the draft report by Friday EOD",
      "directed_at": "other",
      "status": "active",
      "dependencies": [],
      "attributes": {
        "temporal_scope": "by_date",
        "strength": "firm"
      }
    },
    {
      "object_id": "obj_1696534822_question_1",
      "type": "question",
      "creator": "other",
      "turn_number": 4,
      "content_summary": "Asked whether the report should include quarterly breakdown",
      "directed_at": "self",
      "status": "pending",
      "dependencies": ["obj_1696534821_commitment_1"],
      "attributes": {
        "answer_type": "yes_no"
      }
    }
  ],
  "metadata": {
    "turns_analyzed": 5,
    "reflection_timestamp": "2025-10-05T14:30:00Z"
  }
}

IMPORTANT INSTRUCTIONS:
- Only extract objects explicitly created in THIS segment, not prior context
- If an object is referenced but was created earlier, note it in dependencies but don't re-extract
- Distinguish between implicit objects (position implied by argument) and explicit ones
- When status is ambiguous, prefer "pending" over "active"
- If multiple people co-create an object (e.g., reaching agreement), use "joint" as creator
- Include ALL object types present, even if some categories are empty
- Ensure object_id is truly unique by including timestamp

###################### END OF TEMPLATE ######################

EXISTING OBJECTS IN CURRENT DISCOURSE:
{{$current_objects}}

EXISTING OBJECTS FROM PRIOR CONVERSATIONS WITH THIS PERSON:
{{$relevant_historical_objects}}

CONVERSATION SEGMENT:
{{$recent_turns}}

TASK: For each discourse object in this segment, determine:
- Is this a NEW object? → extract as before
- Is this a REFERENCE to existing object? → provide object_id only
- Is this an UPDATE to existing object? → provide object_id + updated fields

OUTPUT FORMAT:
{
  "new_objects": "...as specified above...",
  "references": [
    {
      "object_id": "obj_1696534805_commitment_1",
      "turn_number": 7,
      "reference_type": "fulfillment|reminder|query|challenge"
    }
  ],
  "updates": [
    {
      "object_id": "obj_1696534800_proposal_1", 
      "turn_number": 8,
      "updated_fields": {
        "status": "accepted",
        "content_summary": "Proposal to launch feature in 6 weeks (revised from 4)"
      }
    }
  ]
}
"""

class DiscourseTracker:
    def __init__(self, llm_client, self_character_name: str, other_character_name: str):
        self.llm_client = llm_client
        self.self_character = self_character_name
        self.other_character = other_character_name
        self.objects = {}
        self.turn = 0

    def add_object(self, object):
        self.objects[object.id] = object

    def update_object(self, object_id, updated_fields):
        self.objects[object_id].update(updated_fields)

    def get_objects(self):
        return self.objects

    def get_object(self, object_id):
        return self.objects[object_id]

    def get_turn(self):
        return self.turn

    def increment_turn(self):
        self.turn += 1

    def get_recent_objects(self):
        return [obj for obj in self.objects.values() if obj.turn > (self.turn - 10)]

    def get_active_objects(self):
        return [obj for obj in self.objects.values() if obj.status in ['active', 'pending']]

    def get_recent_turns(self):
        return [obj for obj in self.objects.values() if obj.turn > (self.turn - 10)]

    def get_active_turns(self):
        return [obj for obj in self.objects.values() if obj.status in ['active', 'pending']]

    def get_recent_objects(self):
        return [obj for obj in self.objects.values() if obj.turn > (self.turn - 10)]

    def get_active_objects(self):
        return [obj for obj in self.objects.values() if obj.status in ['active', 'pending']]

    def get_recent_turns(self):
        return [obj for obj in self.objects.values() if obj.turn > (self.turn - 10)]

    def get_active_turns(self):
        return [obj for obj in self.objects.values() if obj.status in ['active', 'pending']]    

    def build_extraction_prompt(self, recent_turns):
        system_prompt = DISCOURSE_TEMPLATE
        user_prompt = self.build_extraction_user_prompt(recent_turns)
        return system_prompt, user_prompt

    def build_extraction_user_prompt(self, recent_turns):
        # Include only objects from current conversation
        active_objects = [
            obj for obj in self.objects.values()
            if obj.status in ['active', 'pending']
        ]
        
        # Further filter to recently mentioned (last 10 turns)
        recent_objects = [
            obj for obj in active_objects
            if obj.turn > (self.current_turn - 10)
        ]
        
        return json.dumps([obj.to_dict() for obj in recent_objects], indent=2)

    def extract_objects(self, recent_turns):
        recent_objects  = self.build_extraction_prompt(recent_turns)
        response = self.llm_client.ask({'recent_turns': recent_turns, 
                                'current_objects': '', 
                                'relevant_historical_objects': '',
                                }, 
                            [SystemMessage(content=DISCOURSE_TEMPLATE)],
                            max_tokens=3000,
                            temp=0.4,
                            stops=['</end>'],
                            is_json=True,
                            log=True, trace=True)
        if isinstance(response, dict):
            print(json.dumps(response, indent=2))
        else:
            print(response)
        return response


#]Conversation ID: conv_forest_2025_001
#Participants: Joe, Samantha
#Context: Joe and Samantha are hiking partners who met through a meetup group 3 weeks ago. They've been lost for about 2 hours.conversation = [

turns = {

"Turn 1": {"speaker": "Samantha", "text": "Okay, we need to figure this out. I think we should head downhill - water flows down, and streams usually lead to civilization. What do you think?"} ,
"Turn 2": {"speaker": "Joe", "text": "That makes sense in theory, but I'm pretty sure the trail we came in on was heading uphill toward the ridge. If we go down, we might end up deeper in the valley, farther from the parking lot."},
"Turn 3": {"speaker": "Samantha", "text": "Hmm. Do you actually remember which direction the parking lot is from here? Because I'll be honest, I've lost track."},
"Turn 4": {"speaker": "Joe", "text": "Not precisely, but I remember we crossed that creek about an hour into the hike, and it was on our left. If we can find the creek again and keep it on our right, we should retrace our steps."},
"Turn 5": {"speaker": "Samantha", "text": "That... actually sounds reasonable. But how do we find the creek? I haven't heard water in like an hour."},
"Turn 6": {"speaker": "Joe", "text": "Look, I have a compass. See? North is that way. The creek was flowing roughly northeast when we crossed it. So if we head northwest, we should intersect it."} ,
"Turn 7": {"speaker": "Samantha", "text": "Wait, you've had a compass this whole time? Why didn't you mention that earlier?  (slight edge to her voice)"},
"Turn 8": {"speaker": "Joe", "text": "I... sorry, I thought I did? I've been trying to work out the bearings in my head. I should have just shown you right away. My bad."} ,
"Turn 9": {"speaker": "Samantha", "text": "Okay. So you're saying we go northwest to find the creek, then follow it southwest back to where we crossed it?"},
"Turn 10": {"speaker": "Joe", "text": "Exactly. And I promise I'll keep you updated on what I'm thinking - no more keeping information to myself. Deal?"},
"Turn 11": {"speaker": "Samantha", "text": "Deal. But before we start walking, can I see the compass? I want to make sure I understand how to read it too, in case we get separated or something."},
"Turn 12": {"speaker": "Joe", "text": "Of course. Here - the red needle always points north. See how it's pointing that way? So northwest is between north and west, about 45 degrees from north."},
"Turn 13": {"speaker": "Samantha", "text": "Got it. Okay, I'm going to trust your plan. Let's head northwest and look for the creek. But if we don't find it in 30 minutes, we should reconsider, agreed?"},
"Turn 14": {"speaker": "Joe", "text": "Agreed. 30 minutes, and if we don't find water or hear it, we stop and reassess. I'll set a timer on my phone."},    
"Turn 15": {"speaker": "Samantha", "text": "Wait - you have your phone? Does it have signal?"},
"Turn 16": {"speaker": "Joe", "text": "No signal, hasn't had any for the past hour. But I figured we could use it for the timer and the flashlight if we need it. Battery's at 40%."},
"Turn 17": {"speaker": "Samantha", "text": "Okay. Good to know. Look, I know I'm being a bit suspicious here, but we only met a few weeks ago and I'm just... trying to make sure we're making smart decisions together. You understand, right?"},  
"Turn 18": {"speaker": "Joe", "text": "Totally understand. If I were in your shoes, I'd be asking the same questions. How about this - every 10 minutes or so, we check in with each other about what we're seeing and thinking. Full transparency."},
"Turn 19": {"speaker": "Samantha", "text": "I appreciate that. And for what it's worth, I think your plan is solid. I'm sorry if I came off as not trusting you - I'm just scared and trying to stay rational."},
"Turn 20": {"speaker": "Joe", "text": "No apology needed. Being cautious is smart. Alright, let's do this - northwest for 30 minutes, check-ins every 10 minutes, and we reassess if we don't find the creek. Ready?"},
"Turn 21": {"speaker": "Samantha", "text": "Ready. Lead the way - and thanks for being patient with my questions."}
}

def main():
    llm_client = ZenohLLMClient(server_name='vllm', model_name='gpt-4o-mini', service_timeout=240.0)

    discourse_tracker = DiscourseTracker(llm_client, "Joe", "Samantha")
    discourse_tracker.extract_objects(turns)
    print(discourse_tracker.get_objects())

if __name__ == "__main__":
    main()