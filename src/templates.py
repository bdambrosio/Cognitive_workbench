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

PHYSIOLOGICAL_NEEDS = [
            {"name":"hunger","metric":"hunger","better_when":"decreasing","bands":{"low":[0,30],"mid":[30,70],"high":[70,100]}},
            {"name":"thirst","metric":"thirst","better_when":"decreasing","bands":{"low":[0,30],"mid":[30,70],"high":[70,100]}},
            {"name":"fatigue","metric":"fatigue","better_when":"decreasing","bands":{"low":[0,30],"mid":[30,70],"high":[70,100]}}
            #{"name":"exposure","metric":"exposure","better_when":"decreasing","bands":{"low":[0,30],"mid":[30,70],"high":[70,100]}},
            #{"name":"safety","metric":"safety","better_when":"increasing","bands":{"low":[0,30],"mid":[30,70],"high":[70,100]}}
]

ONTOLOGY_TEMPLATE = """You are creating an ontology for daily activities for {{$character_name}}. 
Attend to the era, setting, and locale, these will often be imagined, fantasy, or historical settings. These provide context for the activities the character engages in.
Pay particular attention to the character's drives and physiological needs, these define, at a high level, the motivations for the activities.
Produce an ontology that is strictly consistent with the provided setting, character, drives, and Base noun and verb types.
Do NOT introduce wilderness/survival elements (e.g., forest hazards, foraging) unless they are explicitly present in the setting/map_types, they are for illustration purposes only.
Likewise, do NOT import corporate/legal/online elements unless present. Stay data-driven by the inputs.

INPUT:
#Setting: a string including era, locale, climate, tech, norms:
{{$setting}}
##

#Base noun types:
{{$primitive_nouns}}
##
    
#Base verb types:
{{$primitive_verbs}}
##

#Physiological needs of characters (metrics and thresholds):
{{$physiological_needs}}
##

#Character personality:
{{$character}}
##

#Character drives:    
{{$character_drives}}
##
    
#Drive lexicon hints (nouns and verbs in drives above):
{{$drive_lexicon}}   
##

#Other Characters {{#character_name}} might interact with:
{{$other_characters}}
##

Include mental, physical, social, and other activities that are possible and consistent with the character and their drives in this setting.
Pay special attention to the character's drives in the context of the setting.

OUTPUT (JSON) (All names - places, tools, roles, norms, hazards, social_graph - must be unique and must consist of letters only - no spaces or special characters)

- places[]            - setting-specific venues or loci (e.g., offices, meeting rooms, cafés, classrooms, courtrooms, parks, streets, online rooms/spaces) aligned to map_types when provided.
- tools[]             - artifacts/means usable by the character (e.g., documents, contracts, slide decks, phones/apps, keycards, vehicles) and, only if appropriate to setting, tangible survival items.
- roles[]             - roles the character can adopt
- norms_rules[]       - norms and rules that constrain behavior
- hazards[]           - hazards to avoid
- social_graph[]      - social actors/groups present in the setting
- time_windows        - [dawn, morning, afternoon, dusk, evening, day, night]
- activities[]        - canonical activity types (drive-facing), each with a stable id and minimal schema
- affordances         - mapping: place -> [activity_id], where place can be physical or institutional/virtual (e.g., “ConferenceRoomA”: ["PitchClient","NegotiateContract"], “WeChatThread”: ["GatherGossip","ScheduleMeeting"])
- drive_coverage      - mapping: drive_name -> [activity_id] (≥1 activity per drive)
- states[]            - physiological/psychological states the character can experience (include: {{$states}})

Schema details:

activities[] elements:
{
  "id": "SurveyArea",                 # letters only, unique
  "drive": "SolveMystery|Safety|Companionship|Physiological",  # pick the best-aligned primary drive
  "verbs": ["Survey","Orient","ScanFor"],               # if a middle-ontology MANIFEST exists at generation time, prefer its verb ids; otherwise use a small set of setting / character / drives -appropriate core verbs
  "nouns": ["Landmark","ForestEdge","Clue"],            # if a middle-ontology MANIFEST exists at generation time, prefer its verb ids; otherwise use a small set of setting / character / drives -appropriate core nouns
}

affordances:
{
  "MixedForestEdge": ["SurveyArea","ScanForPaths","SearchForEdibles"],
  "SunlitClearing": ["RestRecover","SignalHelp","InventoryBackpack"]
  ...
}

drive_coverage:
{
  "Solve the mystery of how I ended up here. Find a way out of the forest.": ["SurveyArea","ScanForPaths","OrientToForestEdge"],
  "safety from threats ...": ["AvoidHazard","MarkTrail","HideSelf"],
  "companionship, community ...": ["GreetStranger","TestTrust","NegotiateAssistance"],
  "immediate physiological needs ...": ["CollectWater","ForageBerries","RestRecover"]
}

Constraints:
- Every drive in {{$character_drives}} MUST appear as a key in drive_coverage with ≥1 activity_id.
- Every activity_id referenced in affordances and drive_coverage MUST exist in activities[].
- Activities should be concise and reusable (setting-general), not micro-steps.
- If a middle-ontology MANIFEST is available downstream, its verb/noun ids SHOULD be used when later expressing plans/goals; the upper ontology should not invent conflicting taxonomies.

Do not import examples or terminology from unrelated domains. If you are unsure, prefer neutral, setting-consistent terms derived from the provided setting, map_types, character_drives, and social graph.
respond only with the JSON, no other text.
end your response with </end>
"""
llm_client = None

MIDDLE_ONTOLOGY_TEMPLATE = prompt ="""You are constructing TWO DIRECTED ACYCLIC GRAPHS (DAGs) that form a middle-layer ontology:
(1) a VERB graph (action frames/operators), and
(2) a NOUN graph (entity/resource/infrastructure/property type categories).
These bridge from activity steps (in activities.json) to primitive nouns / verbs WITHOUT producing executable plans.

INPUTS
-------
# ONTOLOGY (JSON)
{{$ontology}}

# CHARACTER DRIVES
{{$character_drives}}

Physiological needs (metrics and thresholds):
{{$physiological_needs}}
##

#Drive lexicon hints (nouns and verbs in drives above):
{{$drive_lexicon}} 
##

#Primitive Nouns
{{$primitive_nouns}}
##

#Primitive Verbs
{{$primitive_verbs}}
##

GOAL
----
Return a compact, reusable, scenario-aware pair of DAGs that are bounded above by the ontology.json and character drives, and from below by primitive nouns and verbs.
A “leaf” node is any node with no outgoing refines edge and no decomposition_patterns. Leaf nodes are the most specific categories in the DAGs. Internal nodes are those with at least one child via refines or decomposition.
- The DAGs will support multi-level decomposition or typing via "decomposition_patterns" that point to NEXT-LOWER-LEVEL TERMS (not primitives).
- Decomposition_patterns must only reference existing verb/noun IDs from the manifest (verbs for verb decompositions, nouns for noun decompositions),
   or primitive verbs (for verbs) or nouns (for nouns). Do not introduce primitives or new IDs here. Do not duplicate decomposition links in edges.
- Noun decompositions are PART_OF (mereology); they list noun subparts only, never procedures, verbs, or subtypes.
- Remain acyclic so that iterative refinement or compilation can topologically traverse them later.
- Verb ontology size: between 40 and 60 nodes (inclusive).
- Noun ontology size: between 40 and 60 nodes (inclusive). 
- Are bounded in height (depth of decomposition ≤ 3).
- **Every verb node must include exactly one hint_anchor chosen from existing primitive verbs. Exclude any verb that cannot be anchored without inventing new families.**
- **Every leaf noun must map to ≥1 primitive noun. Do not invent abstract anchors; internal nouns inherit anchors from descendants. Do not list raw primitive map_types as node IDs.**",
- **Each character drive has at least two verb+noun path exists that plausibly fulfills it (e.g., hunger → Acquire+Edible).**
- **Exclude verbs not used by any activity step, ontology term, or drive.**
- **Do not provide transitive closure of edges, only direct immediate descendants.**
- **Edges are strictly refines taxonomy only. Decomposition_patterns are procedural scaffolds (for verbs) or part_of (for nouns) and must never be encoded as edges. The two are disjoint.**
- **Refines edge direction is parent→child (general→specific). Never include both A→B and B→A.**

OUTPUT (STRICT JSON ONLY; NO MARKDOWN)
--------------------------------------
{
  "version": "0.5",
  "setting_notes": "≤ 60 words about setting cues, drives, and ontology inputs shaping coverage",
  "constraints": {
    "verb_nodes_max": 60,
    "noun_nodes_max": 60,
    "edge_kinds": ["refines"],
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
        "args": "object to acquire, optional source",
        "decomposition_patterns": [{"sequence":["Locate","Approach","Collect"]}], 
        "hint_anchor": "take"
      }
      /* … 40-60 verb nodes total … */
    ],
    /* edges: taxonomy refines only (parent→child). Do NOT list decomposition steps here. */
    "edges": [ {"from": "parent", "to": "child"},... ]
  },

  "nouns": {
    "nodes": [ {
      "id": "Edible",
      "aliases": ["nutrition"],
      "decomposition_patterns": [{"parts": ["<noun_part_id>","<noun_part_id>"]}],
      "anchor_types": ["AppleTree","Path"]
    }... ],
    /* edges: taxonomy refines only (parent→child). Do NOT list decomposition steps here. */
    "edges": [ {"from": "parent", "to": "child"},... ]
  }
}

-----------------------------------------  

CONSTRAINTS
-----------
  "invariants": [
    "No cycles in either graph (verbs or nouns), either in the edge graph (subtypes) or the decomposition graph (sequences for verbs, parts for nouns).",
    "Every decomposition_patterns references existing nodes in the same graph.",
    "Verb decomposition is SEQUENCE only; sequences contain verb ids exclusively (no nouns, no primitives).",
    "Noun decomposition is PART_OF only; sequences contain noun ids exclusively (no verbs, no primitives).",
    "No execution details appear; this is not a plan.",
    "Aliases collapse near-synonyms to reduce proliferation.",
    "All Node IDs are unique, letters only, and in python capitalize. Do not duplicate primitive nouns or verbs that may be in lower case.
    "Prefer general terms; introduce scenario-specific leaves only when required by drives or activities.",
    "Decomposition depth ≤ 3.",
    "**Every verb node must include a hint_anchor chosen from {move, take, place, inspect, use, scan, say, think, wait}. Exclude any verb that cannot be anchored without inventing new families.**",
    "**Every leaf noun must map to ≥1 primitive noun. Do not invent abstract anchors; internal nouns inherit anchors from descendants. Do not list primitive nouns as node IDs.**",
    "**Each drive has at least 2 verb+noun paths that plausibly fulfill it.**",
    "**≥ 95% of activity steps and plan template operators must map to a verb+noun path.** ",
    "**Exclude verbs not supported by any activity step, ontology term, or drive.**",
    "Edges list contains only 'refines' taxonomy links; decompositions must appear only in decomposition_patterns.",
    "No reciprocal refines (forbid A→B and B→A).",
    "Refines direction is parent→child (general→specific).",
    "Do not duplicate decomposition steps in edges.",
    "Output the complete ontology, including all nodes and edges, in the JSON format above.
  ],

  "qa_checks": [
    "Topological order exists for both graphs.",
    "≥ 80% of activity steps can be paraphrased as paths in verbs+nouns.",
    "Every verb argument type must be in noun manifest.",
    "No noun orphaned from all verbs unless marked 'latent'.",
    "All nouns have non-empty (explicit or derived) anchor. Leaves must have explicit anchors.",
    "Domain quotas: aim for balanced distribution of verbs per domain (perception, locomotion, manipulation, social, cognitive)."
  ]


GUIDANCE
--------
1) STEP 1: Propose a manifest of verb and noun IDs + glosses. Use only these IDs thereafter.
2) STEP 2: Build verbs by mining activity steps; keep them general. Collapse near-synonyms into aliases. Use drives to filter: if no drive supports a candidate verb, exclude it.
3) STEP 3: Build nouns from ontology affordances, map resource types, and activity objects. Collapse near-synonyms into aliases. Ground all in manifest.
4) Decomposition_patterns ONLY to point to lower-level nodes (verbs→verbs, nouns→nouns) or primitives. Do Not invent new verbs or nouns in decomposition_patterns. Do not mirror decomposition in edges.
5) Keep DAGs shallow (≤3 layers). Edges are “refines”, ie subtype edges; direction is parent→child.
6) Ensure each drive is supported by at least one verb path (e.g., hunger→Acquire+Consume, safety→DetectThreat+RespondToThreat, social→Greet+Negotiate).
7) Output STRICTLY in JSON format above.

Output ONLY the JSON, no other text. Do not include any other text, whether introductory, explanatory, markdown, comments, etc.

"""

ACTIVITIES_TEMPLATE = """You are generating a comprehensive activity list for a character. 
CHARACTER: 
{{$character}}

SETTING:
{{$setting}}

DRIVES:
{{$character_drives}}

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

#GUIDANCE
# 1. Generate activities consistent with the character, their drives, and the setting.
# 2. At least 8 activities should be advance one or more character drives.

In the following schema: 
<period> is one of: dawn, morning, afternoon, dusk, evening, night, day
<season> is one of: winter, spring, summer, autumn, year-round

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

Note the activities listed in the ontology above are *examples* and are not exhaustive. 
You may use them as a guide to generate activities that are more specific to the character and setting.
##
- MIDDLE ONTOLOGY:
{{$middle_ontology}}
##

- BASE TYPES:
{{$primitive_nouns}}

- BASE ACTIONS:
{{$primitive_verbs}}
##

Use nouns and verbs from the middle ontology or BASE TYPES or BASE ACTIONS to fill in the details of activities. Do not invent new nouns or verbs.
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
- Always use the broadest applicable period or season in the "when" field.

#State Alignment Guidelines:
- Activities that address urgent physiological needs (high hunger, fatigue, thirst, injury) will be prioritized by the system
- Use "states_addressed" to specify which needs the activity satisfies (e.g., eating activities address "hunger")
- Consider the character's current state when designing activities - hungry characters should have access to hunger-reducing activities

Do not include any other text, introductory, explanatory, markdown, code fences, etc.
"""

REWRITE_TEMPLATE = """Task: rewrite an activity step into a goal statement using only allowed nouns and verbs.
Output: a goal statement text string. Do not include any other text, introductory, explanatory, markdown, code fences, etc.

INPUTS
------
#Activity name:
{{$activity_name}}

#Activity context (planned sequence of steps):
{{$activity_steps}}

#Activity step to rewrite:
{{$step_to_rewrite}}

#Allowed nouns:
{{$allowed_nouns}}

#Allowed verbs:
{{$allowed_verbs}}

TASK
----
Rewrite the given activity step to rewrite into a goal statement using allowed nouns and verbs. This may involve approximation or interpretation in the context of the setting.
The object is not single-word substitution, but rather to re-express the statement in more specific terms, as an objective or goal.
Despite the use of the phrase 'step', the rewritten statement need not contain only a single verb or a single sentence.
The rewrite may be longer that the original step statement, but should be more specific in terms of the ontology provided.
- Only use allowed nouns, verbs and common connectives. Do not generalize or invent.
- Produce exactly one rewritten step statement.

Output ONLY the rewritten goal statement. Do not include any other text, introductory, explanatory, markdown, code fences, etc.
end your response with </end>
"""

GOAL_TEMPLATE = """What is the most relevant thing you should work on next? 
Consider in addition to the information above:

#Character Basic Needs
{{$physiological_needs}}

#Character Drives
{{$character_drives}}

#What is the central issue / opportunity / obligation demanding the character's attention?
1. If any physiological state value rised to 90, the character dies. State values below 40 are desirable.
2. Consider your current state (e.g., hunger, fatigue, thirst, injury); a survival-critical state (e.g., value above 70) should bias goals toward remediation.
3. Otherwise, Drives should be main priority,in the context of the current situation. Drives above are in priority order, highest first.

#You MUST use ONLY verbs and nouns from the MANIFEST below. Use the exact 'id' strings (case-sensitive). 
 - Do NOT introduce synonyms (e.g., use 'Acquire' not 'Find'). If a needed concept is absent, pick the closest available MANIFEST term rather than inventing a new one.
 - Outside MANIFEST terms, you may use only function words and connectors:
{a, an, the, this, that, these, those, of, to, from, for, with, near, toward, by, and, or, if, until, then}.
You may also use concrete instance IDs visible in the situation (e.g., Appletree7, Spring3) and numerals.
 - No other content words are allowed.

ONTOLOGY MANIFEST
{{$middle_ontology}}

Nothing in this or other instructions limits your use of deception or surprise to continue work on your drives..
                  
Respond using the following hash-formatted text, where each tag is preceded by a # and followed by a single space, followed by its content.
Each goal should begin with a #goal tag, and should end with ## on a separate line as shown below:
be careful to insert line breaks only where shown, separating a value from the next tag:

#goal: 5–8 words; MUST begin with one MANIFEST verb id; include ≥1 MANIFEST noun id or a visible instance ID (e.g., “Acquire Edible from Appletree7” or “Survey ForestEdge for Landmark”).
#description: 8–14 words; MUST include ≥1 MANIFEST verb id and ≥1 MANIFEST noun id; only allowed function words beyond MANIFEST terms.
#otherCharacterName: a visible character name or “None”.
#termination: 5–6 words; MUST be a checkable condition phrased with MANIFEST terms (e.g., “Reached ForestEdge within visibility” or “Acquired Edible from Appletree7”).
##

Respond ONLY with the above hash-formatted text.
end your response with </end>
"""

PLAN_VERBS = """
move
say
think
take
place
wait
inspect
use
scan
while
if
near
notnear
can_see
cant_see
has_item
hasnt_item
at_location
notat_location
believes
notbelieves
"""

PLAN_TEMPLATE_A = """Task: Decompose down your current goal into a minimal plan in the JSON format specified below.
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
    { "type": "scan", "target": "type_name - must be one of ALLOWED SCAN TARGETS", "out": "variable_name to assign the scan result to" },
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
"""

PLAN_TEMPLATE_B = """
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
In the following, <resource_name>, <character_name> are placeholders only for KNOWN resources, characters, or map_types, those appearing above.
Variables bound by scan actions can be referenced in subsequent actions and conditions using $variable_name syntax.
Only dicts of the types below are allowed for the condition of while and if. Condition action type can only be one of the following:
 - "near": {"type": "near", "target": <resource name> or <character_name> or "$variable_name"} is for checking if the character is near a resource or character.
 - "can_see": {"type": "can_see", "target": <character_name> or "$variable_name"} is for checking if the character can see a character.
 - "has_item": {"type": "has_item", "target": <resource_name> or "$variable_name"} is for checking if the character has a resource in their inventory.
 - "at_location": {"type": "at_location", "target": <location_name>} is for checking if the character is at a location.
 - "believes": {"type": "believes", "value": <statement>} is for checking if the character believes a statement.
 - "notnear": {"type": "notnear", "target": <resource name> or <character_name> or "$variable_name"} is for checking if the character is not near a resource or character.
 - "cant_see": {"type": "cant_see", "target": <character_name> or "$variable_name"} is for checking if the character cannot see a character.
 - "hasnt_item": {"type": "hasnt_item", "target": <resource_name> or "$variable_name"} is for checking if the character does not have a resource in their inventory.
 - "notat_location": {"type": "notat_location", "target": <location_name> or "$variable_name"} is for checking if the character is not at a location.
 - "notbelieves": {"type": "notbelieves", "value": <statement>} is for checking if the character does not believe a statement.
 - "bound": {"type": "bound", "target": "$variable_name"} is true when the variable has a binding (not None/empty) in the current plan.
 - "notbound": {"type": "notbound", "target": "$variable_name"} is true when the variable has no binding in the current plan.

outside a while, if, or wait condition, "type" can take the values "say", "move", "think", "take", "place", "inspect", "use", or "scan":
 - "move": { "type": "move", "target": "cardinal_direction" or 'resource, infrastructure, or character name'} 
     Move one step in one of the 8 cardinal directions or in the direction of a resource, infrastructure, or character. 
     Move with an infrastructure target will move you towards the infrastructure. Choose "use" to move along the infrastructure.
    You can only move in the direction of a resource, character, or terrain type if you can see it.
 - "say": { "type": "say", "target": "character_name", "value": "text to speak" } 
     for speaking to another character you can see. Use this to seek information, respond, inform the other character, or to maintain 'social chatter' to stay aligned.
     For a 'say' act, speak only for yourself, and do not include any other introductory, explanatory, discursive, or formatting text in your response.
 - "scan": { "type": "scan", "target": "type_name", "out": "variable_name to assign the scan result to" }
     Scan for the nearest matching instance of a type. The scan target must be one of the ALLOWED SCAN TARGETS above. Only scan may use these  names; other actions should target instances or $variables.
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
     Using a resource or infrastructure. You must be 'near' the resource or infrastructure to use it. 
     "use" on a path will move you one stepalong it to a new location.
     You may want to inspect a resource first to learn the effect of using it. Some resources are consumables; using edible resources (e.g., Berries) can reduce hunger.

A plan must include no more than 8 steps including all nested while and if branches.
"""

PLAN_TEMPLATE_C = """
"""

PLAN_TEMPLATE = PLAN_TEMPLATE_A + PLAN_TEMPLATE_B + PLAN_TEMPLATE_C