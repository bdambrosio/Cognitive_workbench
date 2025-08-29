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

OUTPUT (JSON) (All names - places, tools, roles, norms, hazards, social_graph - must be unique and must consistent of letters only - no spaces or special characters)

- places[] - places the character can be in the setting - map terrain types or resource types only
- tools[] - tools the character can use in the setting, including edible resource types available in the setting (e.g. Berries)- resource types or items from character status or inventory
- roles[] - roles the character can adopt in the setting
- norms_rules[] - norms and rules the character must follow in the setting - character personality or map norms
- hazards[] - hazards the character must avoid in the setting - map terrains, resources, hazards, or specific other characters
- social_graph[] - social graph of the character in the setting - character personality or map social graph
- time_windows - Either:
    - One of: dawn, morning, afternoon, dusk, evening, night
- affordances - e.g. place->[activities_possible]
- states[] - physiological and psychological states the character can experience, including: {{$states}}

respond only with the JSON, no other text.
end your response with </end>
"""
llm_client = None

MIDDLE_ONTOLOGY_TEMPLATE = prompt ="""You are constructing TWO DIRECTED ACYCLIC GRAPHS (DAGs) that form a middle-layer ontology:
(1) a VERB graph (action frames/operators), and
(2) a NOUN graph (entity/resource/type categories).
These bridge from activity steps (in activities.json) to lower-level concepts WITHOUT producing executable plans.

INPUTS
-------
# ONTOLOGY (JSON)
{{$ontology}}

# CHARACTER DRIVES
{{$character_drives}}

# MAP TYPES
{{$map_types}}

# PLAN TEMPLATE
{{$plan_template}}

GOAL
----
Return a compact, reusable, scenario-aware pair of DAGs that are bounded above by the ontology.json and character drives, and from below by map types and PLAN_TEMPLATE.
- Support multi-level decomposition via "decomposition_patterns" that point to NEXT-LOWER-LEVEL TERMS (not primitives).
- Remain acyclic so that iterative refinement or compilation can topologically traverse them later.
- Provide selectional constraints linking verbs to compatible noun classes (typing only).
- Are modest in size: ≈ 20–40 verb nodes and ≈ 30–60 noun nodes.
- Are bounded in both width (breadth of categories) and height (depth of decomposition ≤ 3).
- **Every verb node must include a `hint_anchor` field naming its central primitive family (`move`, `take`, `place`, `inspect`, `use`, `scan`, `say`, `think`). Verbs with no plausible anchor are excluded unless justified by a character drive.**
- **Every noun node must eventually subset into either a `map_types` resource/terrain or a role/state from ontology.json. No free-floating nouns.**
- **For each character drive, ensure at least one verb+noun path exists that plausibly fulfills it (e.g., hunger → Acquire+Edible).**
- **Exclude verbs not supported by any activity step, ontology term, or drive.**

OUTPUT (STRICT JSON ONLY; NO MARKDOWN)
--------------------------------------
{
  "version": "0.4",
  "setting_notes": "≤ 60 words about setting cues, drives, and ontology inputs shaping coverage",
  "constraints": {
    "verb_nodes_max": 40,
    "noun_nodes_max": 60,
    "edge_kinds": ["refines","composes","precondition_of","enables","equivalent_to","alias_of","incompatible_with","member_of","subset_of"],
    "acyclic": true,
    "max_depth": 3
  },

  "manifest": {
    "verbs": [
      {"id": "Acquire", "gloss": "obtain control of an entity"},
      {"id": "Consume", "gloss": "ingest food or drink"},
      {"id": "Negotiate", "gloss": "exchange offers socially"},
      ...
    ],
    "nouns": [
      {"id": "Edible", "gloss": "any item that can be eaten"},
      {"id": "Tool", "gloss": "artifact usable for work"},
      {"id": "Authority", "gloss": "person with higher status"},
      ...
    ]
  },

  "verbs": {
    "nodes": [
      {
        "id": "Acquire",
        "aliases": ["get","pick_up","take_possession"],
        "args": ["$agent","$entity","$source?","$location?"],
        "role_constraints": {"$entity": {"noun_types_any_of": ["Portable","Edible","Tool"]}},
        "decomposition_patterns": [{"sequence":["Locate","Approach","Collect"]}],
        "hint_anchor": "take"
      }
      /* … 20–40 verb nodes total … */
    ],
    "edges": [ ... ]
  },

  "nouns": {
    "nodes": [ ... ],
    "edges": [ ... ]
  },

  "cross_links": [ ... ],

  "coverage": { ... },

  "invariants": [
    "No cycles in either graph (verbs or nouns).",
    "Every decomposition_patterns.sequence references existing nodes in the same graph.",
    "No execution details or primitives appear; this is not a plan.",
    "Selectional constraints refer only to noun node ids from manifest.",
    "Aliases collapse near-synonyms to reduce proliferation.",
    "Prefer general terms; introduce scenario-specific leaves only when required by drives or activities.",
    "Decomposition depth ≤ 3.",
    "**Every verb node must include a `hint_anchor` with one of {move,take,place,inspect,use,scan,say,think}, unless justified by a drive.**",
    "**Every noun node must eventually subset into either a `map_types` resource/terrain or a role/state from ontology.json.**",
    "**Each drive has at least one verb+noun path that fulfills it.**",
    "**Exclude verbs not supported by any activity step, ontology term, or drive.**"
  ],

  "qa_checks": [
    "Topological order exists for both graphs.",
    "≥ 80% of activity steps can be paraphrased as paths in verbs+nouns.",
    "Every verb argument type must be in noun manifest.",
    "No noun orphaned from all verbs unless marked 'latent'.",
    "Domain quotas: ≤5 verbs per domain (perception, locomotion, manipulation, social, cognitive)."
  ]
}

GUIDANCE
--------
1) STEP 1: Propose a manifest of verb and noun IDs + glosses. Use only these IDs thereafter.
2) STEP 2: Build verbs by mining activity steps; keep them general. Collapse near-synonyms into aliases. Use drives to filter: if no active drive supports a candidate verb, exclude it.
3) STEP 3: Build nouns from ontology affordances, map resource types, and activity objects. Ground all in manifest.
4) Use decomposition_patterns ONLY to point to lower-level nodes (verbs→verbs, nouns→nouns). No primitives.
5) Keep DAGs shallow (≤3 layers). Prefer “refines/subset_of” for taxonomy; “composes/precondition_of” for procedural relation.
6) Ensure each drive is supported by at least one verb path (e.g., hunger→Acquire+Consume, safety→DetectThreat+RespondToThreat, social→Greet+Negotiate).
7) Output STRICTLY in JSON format above.

"""

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
  "name": "string", ** even though the activity name is also the object key, also include it inside the object.
  "category": <category from above>,
  "tags": ["physical","mental","social","solo","outdoors","survival","routine"],
  "when": "daily@<period> | opportunistic | seasonal@<season>", 
  "where": ["place ids"], - drawn from ontology places
  "duration": [min_minutes, max_minutes],
  "needs": ["tools or innate capacities"], - drawn from ontology tools or character status or inventory
  "states_addressed": [one or more items from the ontology states field], - what physiological/psychological needs this activity satisfies
  "steps": ["3–5 terse steps that will serve as planning goals"],
  "importance": 0.0,
  "habit": 0.0
}

The following ontologies must be used to fill in the details of activities:
- ONTOLOGY: 
{{$ontology}}
##
- MIDDLE ONTOLOGY:
{{$middle_ontology}}
##

- BASE TYPES:
{{$map_types}}

- BASE ACTIONS:
{{$plan_template}}
##

Use 'higher-level' nouns and verbs from the middle ontology to fill in the details of activities when possible.
Include self-care actions that remediate physiological needs (e.g. eating reduces hunger, rest reduces fatigue, injury or sickness). Make sure to specify which states each activity addresses in the "states_addressed" field.

#Characters you can interact with (if any):
{{$other_characters}}
##

#Step Guidelines:
Generate steps that can serve as achievable planning goals. Each step should:

- Be outcome-focused rather than method-focused: "acquire drinkable water" rather than "purify water using specific method"
- Have clear success criteria: "locate edible resources within area" rather than "find food" (too vague)
- Be situationally adaptable: "prepare food for consumption" rather than "cook food over fire" (assumes fire available)
- Avoid meta-cognitive abstractions: "identify today's priorities" rather than "set goals" (too abstract)
- Specify scope when needed: "inspect personal equipment for damage" rather than "inspect gear" (scope unclear)

#State Alignment Guidelines:
- Activities that address urgent physiological needs (high hunger, fatigue, injury) will be prioritized by the system
- Use "states_addressed" to specify which needs the activity satisfies (e.g., eating activities address "hunger")
- Consider the character's current state when designing activities - hungry characters should have access to hunger-reducing activities

Do not include any other text, introductory, explanatory, markdown, etc.
End with </end>.
"""

REWRITE_TEMPLATE = """You are rewriting a single activity step using the middle-layer ontology.

INPUTS
------
#Activity name:
{{$activity_name}}

#All steps (for context on pre/post conditions):
{{$activity_steps}}

#Step to rewrite:
{{$step_to_rewrite}}

#Ontology (verbs + nouns, including decomposition patterns):
{{$middle_ontology}}

TASK
----
Rewrite the given step by replacing verbs and nouns with their next-level decompositions from the ontology.
- Only expand downward (more specific). Do not generalize or invent.
- Reuse terms that are already leaves.
- Produce exactly one rewritten step string.
- Indicate whether ALL verbs and nouns in the rewritten step are leaves.

OUTPUT (strict JSON only)
-------------------------
{
  "rewritten_step": "<string>",
  "all_leaves": true | false
}

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
8. Consider your current state (e.g., hunger, fatigue, injury); survival-critical state should bias goals toward remediation.

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

Meta-spec (strictly observe these rules in your output):
{
  "actions": ["move","say","think","take","place", "wait", "inspect","use","scan","while","if"],
  "conditions": [
    "near","notnear","can_see","cant_see","has_item","hasnt_item","at_location","notat_location","believes","notbelieves"
  ],
  "required_fields": {
    "move": ["type","target"],
    "say": ["type","target","value"],
    "think": ["type","value"],
    "take": ["type","target"],
    "place": ["type","target"],
    "wait": ["type","condition"],
    "inspect": ["type","target","reason"],
    "use": ["type","target","reason"],
    "scan": ["type","target","out"],
    "while": ["type","condition","body"],
    "if": ["type","condition","then"]
  },
  "variables": {"syntax": "$name", "must_be_bound_before_use": true},
  "max_steps": 8,
  "expand_patterns": true,
  "no_sequential_say": true
}

{
  "plan": [
    { "type": "move", "target": "…"},
    { "type": "say", "target": "…", "value": "…" },
    { "type": "scan", "target": "resource_type", "out": "variable_name to assign the scan result to" },
    { "type": "wait", "condition": "..." },
    { "type": "think", "value": "…" },
    { "type": "take", "target": "…"},
    { "type": "inspect", "target": "…", "reason": "…"},
    { "type": "use", "target": "…", "reason": "…"},
    { "type": "place", "target": "…"},
    { "type": "while", "condition": "…" , "body": [ /* steps */ ]},
    { "type": "if", "condition": "…", "then": [ /* steps */ ], "else": [ /* steps */ ] }
  ]
}

Example workflow using scan and variables:
{
  "plan": [
    { "type": "scan", "target": "Berries", "out": "found_berries" },
    { "type": "move", "target": "$found_berries" },
    { "type": "take", "target": "$found_berries" },
    { "type": "use", "target": "$found_berries", "reason": "eat to reduce hunger" }
  ]
}

Patterns you may reuse (guidance only – always expand to primitive steps in output):
 - approach(X): while notnear(X): move(X)
 - obtain(X): while notnear(X): move(X), then take(X)
 - consume(X, reason): while notnear(X): move(X), then take(X), then use(X, reason)

Do not output macros; always expand patterns into primitive steps in the final JSON.

Worked example using a loop to approach a distant target:
{
  "plan": [
    { "type": "scan", "target": "Berries", "out": "found_berries" },
    { "type": "while", "condition": { "type": "notnear", "target": "$found_berries" }, "body": [
      { "type": "move", "target": "$found_berries" }
    ]},
    { "type": "take", "target": "$found_berries" },
    { "type": "use", "target": "$found_berries", "reason": "eat to reduce hunger" }
  ]
}

A plan must include at least 1 step and no more than 8 steps including all nested while and if branches.
In the following, <resource_name>, <character_name> are placeholders only for KNOWN resources, characters, or maptypes, those appearing above.
Variables bound by scan actions can be referenced in subsequent actions and conditions using $variable_name syntax.
Only dicts of the types below are allowed for the condition of while and if. Condition action type can only be one of the following:
 - "near": {"type": "near", "target": <resource name> or <character_name> or "$variable_name"} is for checking if the character is near a resource or character.
 - "can_see": {"type": "can_see", "target": <character_name> or "$variable_name"} is for checking if the character can see a character.
 - "has_item": {"type": "has_item", "target": <resource_name> or "$variable_name"} is for checking if the character has a resource in their inventory.
 - "at_location": {"type": "at_location", "target": <location_name>} is for checking if the character is at a location.
 - "believes": {"type": "believes", "target": <statement>} is for checking if the character believes a statement.
 - "notnear": {"type": "notnear", "target": <resource name> or <character_name> or "$variable_name"} is for checking if the character is not near a resource or character.
 - "cant_see": {"type": "cant_see", "target": <character_name> or "$variable_name"} is for checking if the character cannot see a character.
 - "hasnt_item": {"type": "hasnt_item", "target": <resource_name> or "$variable_name"} is for checking if the character does not have a resource in their inventory.
 - "notat_location": {"type": "notat_location", "target": <location_name> or "$variable_name"} is for checking if the character is not at a location.
 - "notbelieves": {"type": "notbelieves", "target": <statement>} is for checking if the character does not believe a statement.
 - "bound": {"type": "bound", "target": "$variable_name"} is true when the variable has a binding (not None/empty) in the current plan.
 - "notbound": {"type": "notbound", "target": "$variable_name"} is true when the variable has no binding in the current plan.

outside a while, if, or wait condition, "type" can take the values "say", "move", "think", "take", "place", "inspect", "use", or "scan":
 - "move": { "type": "move", "target": "cardinal_direction" or 'resource or character name'} 
     Move one step in one of the 8 cardinal directions or in the direction of a resource or character.
    You can only move in the direction of a resource, character, or terrain type if you can see it.
 - "say": { "type": "say", "target": "character_name", "value": "text to speak" } 
     for speaking to another character you can see. Use this to seek information, respond, inform the other character, or to maintain 'social chatter' to stay aligned.
     For a 'say' act, speak only for yourself, and do not include any other introductory, explanatory, discursive, or formatting text in your response.
 - "scan": { "type": "scan", "target": "resource_type", "out": "variable_name to assign the scan result to" }
     Scan a resource type to find the nearest matching instance in your current situation.
 - "wait": { "type": "wait", "condition": "..." }
     Wait for a condition to be true. The condition must be one of the Condition actions listed earlier.
 - "think": { "type": "think", "value": "text to think about" } 
     Think about a topic or question, attempting to derive new information, conclusions, or decisions from who you are and what you already explicitly know
 - "take": { "type": "take", "target": "resource_name" } 
     Add a resource you to your personal inventory. you must be near the resource to take it.
 - "place": { "type": "place", "target": "resource_name" } 
     Remove a resource from your personal inventory and place it in the setting at your current location.
 - "inspect": { "type": "inspect", "target": "resource_name", "reason": "what is it you are hoping to learn? - 5 words max"} 
     Inspect a resource or character to learn something about it. Must be 'near' the resource or character to inspect it. reason focuses the inspection on some specific aspect of the resource or character.
 - "use": { "type": "use", "target": "resource_name", "reason": "what outcome do you hope to achieve? - 5 words max"} 
     Using a resource in your inventory. You may reuse resources. You must be 'near' the resource to use it. You may want to inspect the resource first to learn the effect of using it. Some resources are consumables; using edible resources (e.g., Berries) can reduce hunger.

A plan must include no more than 8 steps including all nested while and if branches.
A plan must not contain sequential say actions.
"""