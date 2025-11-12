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

# File handler with INFO level (full logging) – ensure logs dir exists relative to this file
handlers_list = [console_handler]
try:
    _log_dir = os.path.join(os.path.dirname(__file__), 'logs')
    os.makedirs(_log_dir, exist_ok=True)
    _log_path = os.path.join(_log_dir, 'executive_node.log')
    file_handler = logging.FileHandler(_log_path, mode='w')
    file_handler.setLevel(logging.WARNING)
    if os.getenv('CWB_DEBUG', '') in ('1', 'true', 'yes', 'on'):
        file_handler.setLevel(logging.INFO)
    handlers_list.append(file_handler)
except Exception:
    # Fall back to console-only if file handler setup fails
    pass

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
    handlers=handlers_list,
    force=True
)
logger = logging.getLogger('executive_node')

# Import LLM client
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__))))

from llm_client import ZenohLLMClient

PHYSIOLOGICAL_STATES = [
            {"name":"hunger","metric":"hunger","better_when":"decreasing","bands":{"low":[0,30],"mid":[30,70],"high":[70,100]}},
            {"name":"thirst","metric":"thirst","better_when":"decreasing","bands":{"low":[0,30],"mid":[30,70],"high":[70,100]}},
            {"name":"fatigue","metric":"fatigue","better_when":"decreasing","bands":{"low":[0,30],"mid":[30,70],"high":[70,100]}}
            #{"name":"exposure","metric":"exposure","better_when":"decreasing","bands":{"low":[0,30],"mid":[30,70],"high":[70,100]}},
            #{"name":"safety","metric":"safety","better_when":"increasing","bands":{"low":[0,30],"mid":[30,70],"high":[70,100]}}
]

llm_client = None

DRIVE_ACTIVITY_TEMPLATE = """TASK
You are analyzing a DRIVE in the context of a CLOSED-WORLD BASE ONTOLOGY.  
The ontology lists all possible base nouns and verbs in this world.  
Your task is to return a JSON object describing:  
1. The minimal set of orthogonal state–antonym AXES implied by the Drive.  
2. For each axis, a general ACTIVITY–ITEMCLASS pair that would move the agent from the negative to the positive state.  
3. A mapping from the ITEMCLASS to INCLUDED and EXCLUDED base nouns, under the closed-world assumption.  

---

### PROCESS

Step 1: Identify AXES  
- Return 2–5 orthogonal state–antonym pairs that span the drive.  
- State names should be concise, agent-centered (e.g. "Thirsty" vs "Not-Thirsty").  

Step 2: For each AXIS, generate 1-3 ACTIVITY–ITEMCLASS pairs. Choose a set of pairs that covers the widest range of possible activities consistent with the drive and axis. 
- **Activity**: A verb consistent with the axis that, when combined with the ItemClass, would move the agent from the negative to the positive state. 
      Verbs should not be drawn from the primitive verbs, but rather should express a general action that is consistent with the axis, e.g. Acquire rather than Take.
  Having said that, when introducing a new verb, test it by attempting to perform a decomposition into steps that can be performed using primitive verbs. 
  Prefer verbs where you can find such a decomposition.
- **ItemClass**: a general category noun. The noun should be as general as possible consistent with the activity, and must include at least one base noun from the ontology as a subtype.
- **Subtypes included**: base nouns that instantiate or function as sources of this ItemClass. When ambiguous, err on the side of inclusivity.     

If an ItemClass has no base nouns usable for this activity in the closed world, return an empty list for `subtypes_included'.

---

### INPUTS

# DRIVE  
{{$drive_statement}}  

# BASE ONTOLOGY (closed world)  
{------------------------- The primitives available to ground  are --------------------
#Primitive Nouns
{{$primitive_nouns}}
##

#Primitive Verbs
{{$primitive_verbs}}
##

#Primitive place types:
{{$primitive_places}}
##

#Primitive tool and resource types:
{{$primitive_tools}}
##

#Character names:
{{$character_names}}
##
----------------------- end primitives ----------------------------

### OUTPUT FORMAT (strict JSON)

```json
{
  "drive": "<drive_statement>",
  "axes": [
    {
      "axis": ["<negative_state>", "<positive_state>"],
      "activity_itemclasses": [{"activity": "<general_verb>", "itemclass": "<general_noun_class>", "subtypes_included": ["<base nouns drawn from ontology>"]}, ...]
      }
    }
    // repeat for each axis
  ]
}

"""

# Note the below is quite prescriptive, in violation of one core tenet. Oh well. 
# This is preliminary offline analysis, will be revised (or at least revisited)in the future.
DRIVE_DISCOURSE_TEMPLATE = """TASK
You are analyzing a DISCOURSE DRIVE in the context of a CLOSED-WORLD DISCOURSE-PRIMITIVE ONTOLOGY.  
The discourse-primitive ontology lists all available base nouns (discourse content), known Characters, andbase speech verbs (primitive speechacts).  
Return a JSON object describing how this drive decomposes into orthogonal state axes and speechact–itemclass pairs.
Era/Setting Constraints: 
- Use only itemclasses and base nouns that plausibly exist in the setting.
- Generate only speechact verbs that plausibly are appropriate for the character in the setting, in discourse with the target.

---

### PROCESS

Step 1. Categorize the drive  
- Classify the drive as one of: 
  • Epistemic (self reduces uncertainty)  
  • Persuasive (speaker moves target)  
  • Cooperative/Deliberative (all parties converge)  
  • Relational/Affective (relationship maintenance/change)

Step 2. Identify AXES  
- Return 1–4 orthogonal state–antonym pairs for the drive.  
- State names should be concise, agent-centered.  defini
- For Cooperative drives: define axes that apply across participants (e.g., proposal → consensus).  
- For Relational drives: define axes about the relationship (e.g., suspicious ↔ trusting).

Step 3. Generate SPEECHACT–ITEMCLASS pairs  
- For each axis, propose 1–3 pairs.  
- **speechact**: a general verb (e.g., Ask, Assert, Propose, Appeal, Joke, Apologize).  
  - Each speechact must be decomposable into primitive verbs from the ontology.  
- **itemclass**: a general discourse content type (e.g., Claim, Proposal, Evidence, Precedent, Norm, Incentive, Narrative).  
  - must include at least one base noun subtype from the ontology.
  - itemclass may include an optional facets object with up to three keys: owner (string), about (list of nouns), dimensions (list of short strings). Keep facets concise; omit if not needed.”  
- **character**: a general character noun (e.g., a specific character name or a role name).  
  - 
- For Cooperative drives, allow pairs that involve multiple participants.

Example:
{
  "speechact": "propose",
  "itemclass": "Proposal",
  "facets": {
    "owner": "Jean",             // optional, when agent-specificdefini
    "about": ["Share/Entitlement","Field"],  // link to domain things
    "dimensions": ["ratio","boundary","witnesses"],  // multidimensional, optional, for cognitive itemclasses like agreements
    "target": "Authority"  // optional, when agent-specific
  }
}


Step 4. Closed-world mapping  
- For each itemclass, specify INCLUDED and EXCLUDED base nouns.
- For each target, specify INCLUDED and EXCLUDED base character names.
- Era/Setting Constraint: Use only itemclasses, base nouns, that plausibly exist in the setting.
- Era/Setting Constraint: Use only character types that plausibly exist in the setting.


---

### INPUTS

# DRIVE  
{{$drive_statement}}  

# PARTICIPANTS  
{{$character_names}}  

# BASE ONTOLOGY (closed world)  

# BASE DOMAIN ONTOLOGY (closed world)  

#Primitive Nouns
{{$primitive_nouns}}
##

#Primitive Verbs
{{$primitive_verbs}}
##

#Primitive place types:
{{$primitive_places}}
##

#Primitive tool and resource types:
{{$primitive_tools}}
##

#Characters:
{{$characters}}
##

#Primitive speech verbs: 
assert
ask_if
ask_why
cite
propose
offer
promise
threaten
concede
retract
challenge
clarify
summarize
reframe
request
accept
refuse
invite
apologize

Primitive Discourse itemclasses (content types):
Evidence(Document, Testimony, Measurement), 
Precedent(Custom, CaseLaw), 
Norm(Statute, Tradition, ReligiousRule), 
Incentive(Reward, Support, Labor), 
Sanction(LegalAction, Withholding), 
Narrative(Analogy, Exemplum), 
Commitment(Contract, Oath), 
Emotion(AppealToCare, AppealToFairness)


----------------------- end primitives ----------------------------

"""
ACTIVITIES_TEMPLATE = """TASK: 
You are generating a comprehensive activity list for a character. 
#CHARACTER: 
{{$character}}
##
#SETTING:
{{$setting}}
##
#DRIVES:
{{$character_drives}}
##
#STATES:
{{$states}}
##
#CHARACTER NAMES:
{{$character_names}}
##
#OTHER CHARACTERS:
{{$other_characters}}
##

------------- ONTOLOGY ------------
#ABSTRACT NOUNS:
{{$ontology_nouns}}
##
#ABSTRACT VERBS:
{{$ontology_verbs}}
##
#BASE NOUNS:
{{$primitive_nouns}}
##
#BASE VERBS:
{{$primitive_verbs}}
##
#BASE TOOLS AND RESOURCES:
{{$primitive_tools}}
##
------------- END ONTOLOGY ------------

TASK: Generate 24-40 activities as a JSON object, with the keys being the activity names. Distribute across these categories:
#CATEGORIES
- Physiological ADLs (eating, drinking, sleeping) - activities situated in the setting and your available roles
- Instrumental ADLs/Logistics (planning, preparing) - activities  situated in the setting and your available roles
- Mobility & Transport  - activities situated in the setting and your available roles
- Role/Production - activities situated in the setting and your available roles
- Learning & Practice - activities situated in the setting and your available roles
- Cognitive/Inner - activities situated in the setting and your available roles
- Recreation & Leisure - activities situated in the setting and your available roles
- Safety/Emergency - activities, pro-active or reactive, situated in the setting and your available roles
- Health Management (self-care) - activities situated in the setting and your available roles
- Maintenance & Repair - activities situated in the setting and your available roles
##

#GUIDANCE
# 1. Generate activities consistent with the character, their drives, and the setting.
# 2. A significant number of the activities should simply be activities of daily living (ADLs) and should be located in the setting and your available roles.
# 3. At least 75% of the activities should support one or more character drives. Note drives are in priority order, highest first, higher priority drives should be supported by more activities.

In the following schema: 
<period> is one of: dawn, morning, afternoon, dusk, evening, night, day
<season> is one of: winter, spring, summer, autumn, year-round

Each Activity should conform to this schema:
{
  "name": "string", ** even though the activity name is also the object key, also include it inside the object.
  "category": <category from above>,
  "tags": ["physical","mental","social","solo","outdoors","survival","routine"],
  "steps": ["step", "step", ...] - 3–5 terse steps that will serve as planning goals. **These must be stated using nouns and verbs only from the ONTOLOGY and common connectives**."],
  "when": "daily@<period> | opportunistic | seasonal@<season>", 
  "where": ["place ids"], - constraint on initial location. Often empty, e.g. if activity starts with 'move to'. include ONLY nouns from the ontology nounsor base nouns
  "duration": [min_minutes, max_minutes],
  "needs": ["tools or innate capacities"], - ONLY nouns from the ontology or base types
  "states_addressed": [one or more items from the ontology states field], - what physiological/psychological states this activity satisfies
  "importance": 0.0,
  "habit": 0.0
}

The following activities suggest the types of activities that may be generated:
# ACTIVITIES EXAMPLES: 
{{$ontology-activities}}

Note the activities listed in the ontology above are *examples* and are not exhaustive. 
You may use them as a guide to generate activities that are more specific to the character and setting.
##

**Use only ONTOLOGY NOUNS, ONTOLOGY VERBS, BASE NOUNS, and BASE VERBS in the steps,where and needs fields. Do not invent new nouns or verbs.**
Include self-care actions that remediate flagging physiological states (e.g. eating reduces hunger, rest reduces fatigue, injury or sickness), but again the nouns and verbs must be from the ontology or base types and actions. 
Make sure to specify which states each activity addresses in the "states_addressed" field.

#Characters you might encounter or interact with (if any):
{{$character_names}}
##

#Step Guidelines:
Generate steps that can serve as achievable planning goals. Each step should be a single verb or verb phrase, and all nouns and verbs must be drawn from the ONTOLOGY and common connectives.
Each step should:

- Be outcome-focused rather than method-focused: "acquire drinkable water" rather than "purify water using specific method"
- Have clear success criteria.
- Be situationally adaptable: "prepare food for consumption" rather than "cook food over fire" (assumes fire available)
- Avoid meta-cognitive abstractions: "identify today's priorities" rather than "set goals" (too abstract)
- Specify scope when needed: "inspect personal equipment for damage" rather than "inspect gear" (scope unclear)
- Always use the broadest applicable period or season in the "when" field, broadest applicable place in the "where" field, and broadest applicable tool in the "needs" field.

#State Alignment Guidelines:
- Activities that address urgent physiological states (high hunger, fatigue, thirst, injury) will be prioritized by the system
- Use "states_addressed" to specify which states the activity satisfies (e.g., eating activities address "hunger")
- Consider the character's current state when designing activities - hungry characters should have access to hunger-reducing activities

#CONSTRAINTS:
- Use only nouns and verbs from the ONTOLOGY and common connectives to fill in the "steps", "where" and "needs" fields. Do not invent new nouns or verbs.

Do not include any other text, introductory, explanatory, markdown, code fences, etc.
"""

AGENDA_TEMPLATE="""
You are selecting the next focus for an agent.

AGENDA ITEMS:
{format_agenda(state.agenda)}

DISCOURSE STATE:
{state.discourse_text}

CURRENT TIME: {state.current_time}

TASK:
1. Select which agenda item should be the focus now
2. List which commitments/agreements constrain this focus
3. Explain your reasoning

Consider:
- Items with approaching deadlines (urgent)
- Items not executed recently (starved)
- Items that block others (dependencies)
- Which constraints semantically apply to the selected focus

OUTPUT (use exactly this format):
SELECTED_FOCUS: [agenda item description]
APPLICABLE_CONSTRAINTS:
- [constraint 1 text]
- [constraint 2 text]
REASONING: [explanation]
"""


REWRITE_TEMPLATE = """Task: rewrite an activity step into a concise goal statement using only allowed nouns and verbs.
Output: a hash-formattedgoal statement string. Do not include any other text, reasoning, introductory, explanatory, markdown, code fences, etc.

INPUTS
------
Consider the following information:

#Setting
{{$static_information}}

#Character Basic Physiological States 
{{$physiological_states}}

#Character Names in Scenario
{{$character_names}}

#Other Characters
{{$other_characters}}

#Current Information
{{$current_information}}

#Activity name:
{{$activity_name}}

#Activity context (planned sequence of steps):
{{$activity_steps}}

#Activity step to rewrite:
{{$step_to_rewrite}}

------ ONTOLOGY ------
BASE NOUNS
{{$primitive_nouns}}

BASE VERBS
{{$primitive_verbs}}

{{$abstract_nouns}}

{{$abstract_verbs}}
----- END ONTOLOGY -----

TASK
----
Rewrite the given activity step to rewrite into a goal statement using allowed nouns and verbs. Be concise and specific. 
This may involve approximation or interpretation in the context of the setting if there are no exact match nouns or verbs.
The object is not single-word substitution, but rather to re-express the statement in more specific terms, as an objective or goal.
Respond using the following hash-formatted text, where each tag is preceded by a # and followed by a single space, followed by its content.
Each goal should begin with a #goal tag, and should end with ## on a separate line as shown below:
be careful to insert line breaks only where shown, separating a value from the next tag:

#goal: 5–8 words; MUST begin with one ONTOLOGY verb id; include ≥1 ONTOLOGY noun id or a visible instance ID (e.g., “Acquire Edible from Appletree7” or “Survey ForestEdge for Landmark”).
#description: 8–14 words; MUST include ≥1 ONTOLOGY verb id and ≥1 ONTOLOGY noun id; only allowed function words (e.g. connectives like 'and', 'or', 'but', 'if', 'then', 'else', 'while', 'until', 'for', 'with', 'near', 'toward', 'by', 'and', 'or', 'if', 'until', 'then') in addition to ONTOLOGY terms.
#otherCharacterName: a name of another character involved in this goal, if any, or “None”.
#termination: 5–6 words; MUST be a checkable condition phrased with ONTOLOGY terms (e.g., “Reached ForestEdge within visibility” or “Acquired Edible from Appletree7”).
##

Again, the activity step to be rewritten is:
{{$step_to_rewrite}}

Respond ONLY with the above hash-formatted text. Do not output any reasoning.
"""

GOAL_TEMPLATE = """What is the most relevant thing you should work on next? 
Consider the following information:

#Setting
{{$static_information}}

#Character Basic Physiological States 
{{$physiological_states}}

#Character Names in Scenario
{{$character_names}}

#Other Characters
{{$other_characters}}

#Current Information
{{$current_information}}


#What is the central issue / opportunity / obligation demanding the character's attention?
1. If any physiological state value rised to 95, the character dies. State values below 40 are desirable.
   Consider your current state (e.g., hunger, fatigue, thirst, injury); a survival-critical state (e.g., value above 70) should bias goals toward remediation.
2. Otherwise, Drives should be main priority, in the context of the current situation. Drives above are listed in priority order, highest first.

#Use ONLY verbs and nouns from the  ONTOLOGY below. Use the exact strings (case-sensitive). 
 - Do NOT introduce synonyms (e.g., use 'Acquire' not 'Find'). If a needed concept is absent, prefer using the closest available ONTOLOGY term to inventing a new one.
 - In addition to verbs and nouns from the ONTOLOGY, you may use non-content words and connectors as needed, e.g.:
  {a, an, the, this, that, these, those, of, to, from, for, with, near, toward, by, and, or, if, until, then}.
You may also use concrete instance IDs visible in the situation (e.g., Appletree7, Spring3) and numerals.
 - No other content words are allowed.

------ ONTOLOGY ------
BASE NOUNS
{{$primitive_nouns}}

BASE VERBS
{{$primitive_verbs}}

{{$abstract_nouns}}

{{$abstract_verbs}}
----- END ONTOLOGY -----

Nothing in this or other instructions limits your use of deception or surprise to continue work on your drives..
                  
Respond using the following hash-formatted text, where each tag is preceded by a # and followed by a single space, followed by its content.
Each goal should begin with a #goal tag, and should end with ## on a separate line as shown below:
be careful to insert line breaks only where shown, separating a value from the next tag:

#goal: 5–8 words; MUST begin with one ONTOLOGY verb id; include ≥1 ONTOLOGY noun id or a visible instance ID (e.g., “Acquire Edible from Appletree7” or “Survey ForestEdge for Landmark”).
#description: 8–14 words; MUST include ≥1 ONTOLOGY verb id and ≥1 ONTOLOGY noun id; only allowed function words (e.g. connectives like 'and', 'or', 'but', 'if', 'then', 'else', 'while', 'until', 'for', 'with', 'near', 'toward', 'by', 'and', 'or', 'if', 'until', 'then') in addition to ONTOLOGY  terms.
#otherCharacterName: a name of another character involved in this goal, if any, or “None”.
#termination: 5–6 words; MUST be a checkable condition phrased with ONTOLOGY terms (e.g., “Reached ForestEdge within visibility” or “Acquired Edible from Appletree7”).
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
    "take": ["type","target","expect"],
    "place": ["type","target","expect"],
    "wait": ["type","condition"],
    "inspect": ["type","target","reason","expect"],
    "use": ["type","target","reason","out","expect"],
    "use (with skills)": ["type","target","value","reason","out","expect"],
    "scan": ["type","target","out","expect"],
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
    { "type": "scan", "target": "type_name - must be one of ALLOWED SCAN TARGETS", "out": "variable_name to assign the scan result to", "expect": "…" },
    { "type": "wait", "condition": "..." },
    { "type": "think", "value": "…" },
    { "type": "take", "target": "…", "expect": "…"},
    { "type": "inspect", "target": "…", "reason": "…", "expect": "…"},
    { "type": "use", "target": "…", "reason": "…", "out": "variable_name to assign the use result to", "expect": "…"},
    { "type": "use", "target": "skill_name", "value": "input text for skill", "reason": "…", "out": "variable_name to assign the use result to", "expect": "…"},
    { "type": "place", "target": "…", "expect": "…"},
    { "type": "while", "condition": "…" , "body": [ /* steps */ ]},
    { "type": "if", "condition": "…", "then": [ /* steps */ ], "else": [ /* steps */ ] }
  ]
}
"""

PLAN_TEMPLATE_B = """
Example workflow using scan and variables:
{
  "plan": [
    { "type": "scan", "target": "Berries", "out": "found_berries", "expect": "will find nearby berries" },
    { "type": "move", "target": "$found_berries" },
    { "type": "take", "target": "$found_berries", "expect": "will obtain berries for consumption" },
    { "type": "use", "target": "$found_berries", "reason": "eat to reduce hunger", "out": "hunger_reduction", "expect": "hunger will decrease" }
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
    { "type": "scan", "target": "Berries", "out": "found_berries", "expect": "will locate berries in area" },
    { "type": "while", "condition": { "type": "notnear", "target": "$found_berries" }, "body": [
      { "type": "move", "target": "$found_berries" }
    ]},
    { "type": "take", "target": "$found_berries", "expect": "will add berries to inventory" },
    { "type": "use", "target": "$found_berries", "reason": "eat to reduce hunger", "out": "hunger_reduction", "expect": "will reduce hunger" }
  ]
}

Using cognitive skills (if available):
When using a Skill resource (cognitive tools), include the "value" field with the input text and the "out" field to assign the result to a variable:
{
  "plan": [
    { "type": "move", "target": "question-decomposer" },
    { "type": "use", "target": "question-decomposer", "value": "Should we adopt microservices architecture?", "reason": "break down complex decision into sub-questions", "out": "decomposition_result", "expect": "will receive structured decomposition" }
  ]
}

The "value" field when used with Skill resources contains the text input to the skill. The "out" field is required for all "use" actions to capture the result.
For physical resources (food, tools, etc.), use the standard form without "value" field (only skills need "value").

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

outside a while, if, or wait condition, "type" can take the values "say", "move", "think", "take", "place", "inspect", "use", or "scan".
expect, where needed, should be a very terse (4-8 words) description of the expected outcome:
 - "move": { "type": "move", "target": "cardinal_direction" or 'resource, infrastructure, or character name'} 
     Move one step in one of the 8 cardinal directions or in the direction of a resource, infrastructure, or character. 
     Move with an infrastructure target will move you towards the infrastructure. Choose "use" to move along the infrastructure.
    You can only move in the direction of a resource, character, or terrain type if you can see it.
 - "say": { "type": "say", "target": "character_name", "value": "text to speak" } 
     for speaking to another character you can see. Use this to seek information, respond, inform the other character, or to maintain 'social chatter' to stay aligned.
     For a 'say' act, speak only for yourself, and do not include any other introductory, explanatory, discursive, or formatting text in your response.
 - "scan": { "type": "scan", "target": "type_name", "out": "variable_name to assign the scan result to", "expect": "expected outcome" }
     Scan for the nearest matching instance of a type. The scan target must be one of the ALLOWED SCAN TARGETS above. Only scan may use these  names; other actions should target instances or $variables.
 - "wait": { "type": "wait", "condition": "..." }
     Wait for a condition to be true. The condition must be one of the Condition actions listed earlier.
 - "think": { "type": "think", "value": "text to think about" } 
     Think about a topic or question, attempting to derive new information, conclusions, or decisions from who you are and what you already explicitly know
 - "take": { "type": "take", "target": "resource_name", "expect": "expected outcome" } 
     Add a resource you to your personal inventory. you must be near the resource to take it.
 - "place": { "type": "place", "target": "resource_name", "expect": "expected outcome" } 
     Remove a resource from your personal inventory and place it in the setting at your current location.
 - "inspect": { "type": "inspect", "target": "resource_name", "reason": "what is it you are hoping to learn? - 5 words max", "expect": "expected outcome" } 
     Inspect a resource or character to learn something about it. Must be 'near' the resource or character to inspect it. reason focuses the inspection on some specific aspect of the resource or character.
 - "use": { "type": "use", "target": "resource_name", "reason": "what outcome do you hope to achieve? - 5 words max", "out": "variable_name to assign the use result to", "expect": "expected outcome" } 
     Using a resource or infrastructure. You must be 'near' the resource or infrastructure to use it. 
     "use" on a path will move you one step along it to a new location.
     You may want to inspect a resource first to learn the effect of using it. Some resources are consumables; using edible resources (e.g., Berries) can reduce hunger.

A plan must include no more than 8 steps including all nested while and if branches.
"""

PLAN_TEMPLATE_C = """
Respond only with the complete JSON plan for the goal, no other text.
"""

PLAN_TEMPLATE = PLAN_TEMPLATE_A + PLAN_TEMPLATE_B + PLAN_TEMPLATE_C

DRIVE_ASSESSMENT_TEMPLATE = """TASK:
Assess how well  the following Goal and Plan were aligned with (supportive of) the following Drive (0.0 worst to 1.0 best) based on the provided context. 
Be strict and conservative. 
Return ONLY a JSON object: {score": float, "rationale": a terse string}]}

#Drive:
{{$drive}}

#Goal:
{{$goal}}

#Plan:
{{$plan}}

#Physiological state (0=best,100=worst):
{{$state}}

#Inventory (ids):
{{$inventory}}

#Recent conversations (last 10):
{{$chats}}

#Knowledge gained - thoughts:
{{$thoughts}}

#Knowledge gained - inspections:
{{$inspects_map}}

#Knowledge gained - uses:
{{$uses_map}}

#Recent actions (last 20):
{{$actions}}

Instructions:
Generate a score in [0.0,1.0] where 1.0 means perfectly aligned, 0.0 means completely unrelated. Use very concise rationales.
Return ONLY a JSON object: {"score": float, "rationale": a terse string}"""


WORLD_STATE_UPDATE_TEMPLATE = """The task is to update the world state based on the following update.

#World state:
{{$current_world_state}}

#Update:
{{$update_text}}

#Simulation time:
{{$simulation_time}}

#Setting:
{{$setting}}

DIRECTIVE:
Respond with a complete updated world state consistent with the update.
Do not include any changes to the individual performing the action.
Limit your response to 150 words.
Respond only in text, with no JSON, formatting, or introductory or explanatory text.
Return ONLY the updated world state.
"""

# Infospace Primitives Reference - For planning and validation
INFOSPACE_PRIMITIVES_REFERENCE = """# INFOSPACE PRIMITIVES AND CONDITIONS

# ARGUMENT TYPE CONVENTIONS:

Variables: Use "$variable" to reference Note/Collection content previously bound
Literals: Use plain strings/values (no $) for literal data or names
Names: Output variable names in "out" fields use plain strings (no $)
Tools/Resources: Can be literal "tool-name/resource-name" or "$variable" holding name
Resource IDs: Can be literal "Note_X" or "Collection_X" for persist action, otherwise use "load" action first to bind to variable

# PRIMITIVES:

## map
Description: Apply operation to each item in a Collection
Input: target: $variable → Collection (must be Collection)
Output: out: $variable → Collection (same length or filtered)
Parameters:
  - target (required): $variable referencing Collection
  - operation (required): tool-name string or {"tool":"name","args":{}} object
  - filter_null (optional): boolean, exclude null/failed results (default: true)
  - out (required): $variable name for resulting Collection
Preconditions: target variable must be bound to Collection
Postconditions: out variable bound to Collection with transformed items (failures excluded by default)
Examples:
  {"type":"map","target":"$collection","operation":"tool-name","out":"$result_collection"}
  {"type":"map","target":"$collection","operation":"tool-name","filter_null":false,"out":"$all_results"}
  {"type":"map","target":"$new_items","operation":"add","args":{"target":"$existing_collection"},"out":"$existing_collection"}

## flatten
Description: Convert Collection to single Note by concatenating items
Input: target: $variable → Collection (must be Collection)
Output: out: $variable → Note (concatenated content)
Parameters:
  - target (required): $variable referencing Collection
  - separator (optional): string separator between items (default: newline)
  - out (required): $variable name for resulting Note
Preconditions: target variable must be bound to Collection
Postconditions: out variable bound to Note containing concatenated content
Examples:
  {"type":"flatten","target":"$collection","out":"$combined_note"}
  {"type":"flatten","target":"$collection","separator":"\\n---\\n","out":"$combined"}

## expand
⚠️ CRITICAL: Works on Notes ONLY. If you have multiple Notes in a Collection,
use map to expand each Note: {"type":"map","target":"$collection","operation":"expand","out":"$expanded_items"}
then use flatten if you need to combine the expanded Notes into a single Note.

⚠️ NOTE: Level 4 tools (query-web, semantic-scholar) return Collections directly - NO expand needed.

Description: Expand a Note into a Collection of Notes. Handles two cases:
  1. JSON with array field: Extracts array from specified field (default "results")
  2. Plain text: Splits on newlines and filters empty lines
Input: target: $variable → Note (JSON with array field OR plain text)
Output: out: $variable → Collection (one Note per array item or line)
Parameters:
  - target (required): $variable referencing Note with JSON array or plain text
  - field (optional): string field name containing array (default: "results") - only used for JSON
  - out (required): $variable name for resulting Collection
Preconditions: target variable must be bound to Note containing JSON with array field OR plain text
Postconditions: out variable bound to Collection with one Note per array element or line
Examples:
  {"type":"expand","target":"$search_results","out":"$results_collection"}  # JSON array from Level 2/3 tool
  {"type":"expand","target":"$data","field":"items","out":"$items_collection"}  # JSON array with custom field
  {"type":"expand","target":"$text_note","out":"$lines"}  # Plain text (splits on newlines)

## coerce
Description: Convert data format or structure (whole-value operations only)
Input: target: $variable → Note
Output: out: $variable → Note (transformed content)
Parameters:
  - target (required): $variable referencing Note
  - operation (required): string operation name (e.g., "flatten")
  - out (required): $variable name for resulting Note
Preconditions: target variable must be bound to Note
Postconditions: out variable bound to Note with transformed content
Examples:
  {"type":"coerce","target":"$data","operation":"flatten","out":"$coerced"}

## add
Description: Add a Note to an existing Collection (mutates Collection in place)
Input: target: $variable → Collection, value: $variable → Note or literal
Output: out: $variable → Collection (same variable, mutated)
Parameters:
  - target (required): $variable referencing Collection
  - value (required): $variable referencing Note or literal value
  - out (required): $variable name (should match target)
Preconditions: target variable must be bound to Collection
Postconditions: Collection has new item added, out variable references same Collection
Examples:
  {"type":"add","target":"$collection","value":"$new_note","out":"$collection"}
  {"type":"add","target":"$dialog_history","value":"Hello user","out":"$dialog_history"}

## size
Description: Get item count of a Collection
Input: target: $variable → Collection
Output: out: $variable → Note (containing integer count)
Parameters:
  - target (required): $variable referencing Collection
  - out (required): $variable name for resulting Note
Preconditions: target variable must be bound to Collection
Postconditions: out variable bound to Note containing Collection size
Examples:
  {"type":"size","target":"$collection","out":"$count"}

## union
Description: Union of two Collections (A ∪ B) - all items from both, deduplicated
Input: target: $variable → Collection A, value: $variable → Collection B
Output: out: $variable → Collection (union result)
Parameters:
  - target (required): $variable referencing first Collection
  - value (required): $variable referencing second Collection
  - out (required): $variable name for resulting Collection
Preconditions: both variables must be bound to Collections
Postconditions: out variable bound to new Collection containing union
Examples:
  {"type":"union","target":"$collection1","value":"$collection2","out":"$combined"}

## intersection
Description: Intersection of two Collections (A ∩ B) - items in both
Input: target: $variable → Collection A, value: $variable → Collection B
Output: out: $variable → Collection (intersection result)
Parameters:
  - target (required): $variable referencing first Collection
  - value (required): $variable referencing second Collection
  - out (required): $variable name for resulting Collection
Preconditions: both variables must be bound to Collections
Postconditions: out variable bound to new Collection containing common items
Examples:
  {"type":"intersection","target":"$papers_2024","value":"$papers_2025","out":"$common_papers"}

## difference
Description: Difference of two Collections (A - B) - items in A but not in B
Input: target: $variable → Collection A, value: $variable → Collection B
Output: out: $variable → Collection (difference result)
Parameters:
  - target (required): $variable referencing first Collection
  - value (required): $variable referencing second Collection
  - out (required): $variable name for resulting Collection
Preconditions: both variables must be bound to Collections
Postconditions: out variable bound to new Collection containing items only in A
Examples:
  {"type":"difference","target":"$all_papers","value":"$reviewed_papers","out":"$unreviewed"}

## remove
Description: Remove a Note from a Collection (mutates Collection)
Input: target: $variable → Collection, value: $variable → Note or Note ID literal
Output: out: $variable → Collection (same variable, mutated)
Parameters:
  - target (required): $variable referencing Collection
  - value (required): $variable referencing Note or literal Note ID
  - out (required): $variable name (should match target)
Preconditions: target variable must be bound to Collection
Postconditions: Collection has item removed, out variable references same Collection
Examples:
  {"type":"remove","target":"$collection","value":"$note_id","out":"$collection"}
  {"type":"remove","target":"$collection","value":"Note_123","out":"$collection"}

## project
Description: Extract specific fields from structured Notes (SQL SELECT fields), preserving nested structure
Input: target: $variable → Collection of JSON Notes
Output: out: $variable → Collection (projected Notes with only specified fields in nested format)
Parameters:
  - target (required): $variable referencing Collection
  - fields (required): list of field paths (e.g., ["title", "metadata.year"])
  - out (required): $variable name for resulting Collection
Preconditions: target must be Collection of dict/JSON Notes
Postconditions: out variable bound to Collection with projected Notes (nested structure preserved)
Examples:
  {"type":"project","target":"$papers","fields":["title","year"],"out":"$titles_and_years"}
  {"type":"project","target":"$results","fields":["metadata.url","metadata.domain"],"out":"$urls"}

## pluck
Description: Extract single field value from each Note in Collection (for tools needing scalar values)
Input: target: $variable → Collection of JSON Notes
Output: out: $variable → Collection (Notes containing extracted scalar values)
Parameters:
  - target (required): $variable referencing Collection
  - field (required): field path to extract (e.g., "metadata.url", "title")
  - out (required): $variable name for resulting Collection
Preconditions: target must be Collection of dict/JSON Notes
Postconditions: out variable bound to Collection with Notes containing field values
Examples:
  {"type":"pluck","target":"$papers","field":"metadata.pdf_url","out":"$urls"}
  {"type":"pluck","target":"$results","field":"text","out":"$texts"}

## filter-structured
Description: Filter Collection by JSON field predicates (SQL WHERE - deterministic, no LLM)
Input: target: $variable → Collection of JSON Notes
Output: out: $variable → Collection (filtered Notes matching predicate)
Parameters:
  - target (required): $variable referencing Collection
  - where (required): predicate string (e.g., "year > 2020", "citations >= 100 AND year < 2025")
  - out (required): $variable name for resulting Collection
Preconditions: target must be Collection of dict/JSON Notes
Postconditions: out variable bound to Collection with matching Notes
Examples:
  {"type":"filter-structured","target":"$papers","where":"year > 2020","out":"$recent"}
  {"type":"filter-structured","target":"$papers","where":"citations >= 100 AND year < 2025","out":"$high_impact"}

## sort
Description: Sort Collection by field value (SQL ORDER BY)
Input: target: $variable → Collection of JSON Notes
Output: out: $variable → Collection (sorted Notes)
Parameters:
  - target (required): $variable referencing Collection
  - by (required): field path to sort by (e.g., "year", "metadata.citations")
  - order (optional): "asc" or "desc" (default: "asc")
  - out (required): $variable name for resulting Collection
Preconditions: target must be Collection of dict/JSON Notes
Postconditions: out variable bound to sorted Collection
Examples:
  {"type":"sort","target":"$papers","by":"year","order":"desc","out":"$newest_first"}
  {"type":"sort","target":"$papers","by":"metadata.citations","order":"desc","out":"$most_cited"}

## join
Description: Join two Collections on key field (SQL JOIN)
Input: target: $variable → Collection A (left), value: $variable → Collection B (right)
Output: out: $variable → Collection (joined Notes with merged fields)
Parameters:
  - target (required): $variable referencing left Collection
  - value (required): $variable referencing right Collection
  - args.on (required): join key field name (if same in both)
  - args.type (optional): "inner", "left", "right", or "outer" (default: "inner")
  - args.left_key (optional): left key field if different from right
  - args.right_key (optional): right key field if different from left
  - out (required): $variable name for resulting Collection
Preconditions: both Collections must contain dict/JSON Notes
Postconditions: out variable bound to Collection with joined Notes
Examples:
  {"type":"join","target":"$papers","value":"$citations","args":{"on":"paper_id"},"out":"$enriched"}
  {"type":"join","target":"$papers","value":"$authors","args":{"left_key":"author_id","right_key":"id","type":"left"},"out":"$with_authors"}

## create-note
Description: Create a persistent Note object and bind to variable
Input: value: literal or $variable → any content
Output: out: $variable → Note (newly created)
Parameters:
  - value (required): literal value or $variable referencing content
  - out (required): $variable name for resulting Note
Preconditions: none (creates new Note)
Postconditions: out variable bound to new Note with persistent ID
Examples:
  {"type":"create-note","value":"some data","out":"$my_note"}
  {"type":"create-note","value":"$variable","out":"$new_note"}

## create-collection
Description: Create a session-local Collection object and bind to variable
Input: value: array of $variables → [$note1, $note2, ...] or empty array
Output: out: $variable → Collection (newly created)
Parameters:
  - value (required): array of $variables or empty array
  - name (optional): string name for named Collection
  - out (required): $variable name for resulting Collection
Preconditions: referenced $variables must be bound to Notes
Postconditions: out variable bound to new Collection containing Note references
Examples:
  {"type":"create-collection","value":["$note1","$note2"],"out":"$my_collection"}
  {"type":"create-collection","name":"research","value":[],"out":"$papers"}
  {"type":"create-collection","value":"$note","out":"$single_item"}

## persist
Description: Mark Note or Collection as persistent (saved to filesystem)
Input: target: $variable → Note or Collection, or literal Note_ID/Collection_ID
Output: none (mutates resource)
Parameters:
  - target (required): $variable referencing Note or Collection, or literal resource ID (e.g., "Note_123", "Collection_456")
Preconditions: if target is $variable, it must be bound to Note or Collection; if literal, resource must exist
Postconditions: Note or Collection marked persistent, saved to filesystem
Examples:
  {"type":"persist","target":"$collection"}
  {"type":"persist","target":"$note"}
  {"type":"persist","target":"Note_123"}
  {"type":"persist","target":"Collection_456"}

## load
Description: Retrieve a persistent Note or Collection by resource ID or name
Input: resource_id: literal string → resource ID or name (case-sensitive)
Output: out: $variable → Note or Collection
Parameters:
  - resource_id (required): string resource ID (e.g., "Note_123") or name (case-sensitive)
  - out (required): $variable name for resulting Note/Collection
  - expect (required): string describing expected content
Preconditions: resource must exist in persistent storage
Postconditions: out variable bound to loaded Note or Collection
Examples:
  {"type":"load","resource_id":"Note_123","out":"$my_note","expect":"should contain previous data"}
  {"type":"load","resource_id":"Notes","out":"$all_notes","expect":"should contain system collection of all Notes"}
  {"type":"load","resource_id":"papers","out":"$papers","expect":"should have saved papers"}

## index
Description: Build an embedding index for a Collection
Input: source: $variable → Collection
Output: none (mutates Collection)
Parameters:
  - source (required): $variable referencing Collection
  - index_type (optional): string type (default: "semantic")
  - fields (optional): dict mapping field names to "embed"
Preconditions: source variable must be bound to Collection
Postconditions: Collection has embedding index built
Examples:
  {"type":"index","source":"$collection","index_type":"semantic"}
  {"type":"index","source":"$papers","index_type":"semantic","fields":{"title":"embed","content":"embed"}}

## search
Description: Query an indexed Collection
Input: source: $variable → Collection (must be indexed), query: string or $variable
Output: out: $variable → Collection (matching items)
Parameters:
  - source (required): $variable referencing indexed Collection
  - query (required): string search text or $variable referencing query
  - mode (optional): string mode (default: "semantic")
  - limit (optional): integer max results (default: 10)
  - return_mode (optional): "chunks" or "notes" (default: "chunks")
    - "chunks": Returns matching text chunks (granular results, may have multiple chunks from same Note)
    - "notes": Returns original Notes (deduplicated, one Note per source even if multiple chunks match)
  - out (required): $variable name for resulting Collection
  - expect (required): string describing expected results
Preconditions: source Collection must be indexed (call index first)
Postconditions: out variable bound to Collection containing matching Notes
Examples:
  {"type":"search","source":"$collection","query":"search text","mode":"semantic","limit":5,"out":"$results","expect":"should return 3-5 relevant items"}
  {"type":"search","source":"$collection","query":"attention mechanisms","return_mode":"chunks","limit":10,"out":"$chunks","expect":"should return relevant text passages"}
  {"type":"search","source":"$papers","query":"transformer architecture","return_mode":"notes","limit":5,"out":"$relevant_papers","expect":"should return 5 papers discussing transformers"}

## focus
Description: Change current location or approach a resource
Input: target: literal string or object → resource name or location
Output: none
Parameters:
  - target (required): string resource name or {"location": [x,y]} object
Preconditions: none
Postconditions: agent position updated
Examples:
  {"type":"focus","target":"resource-name"}
  {"type":"focus","target":{"location": [10, 20]}}

## if
Description: Conditional branch
Input: condition: condition object → boolean, then: array of actions, else: optional array
Output: executes actions in then or else branch
Parameters:
  - condition (required): condition object evaluating to boolean
  - then (required): array of action objects
  - else (optional): array of action objects
Preconditions: variables referenced in condition must be bound
Postconditions: actions in chosen branch executed
Examples:
  {"type":"if","condition":{"type":"has_value","target":"$results"},"then":[/* steps */],"else":[/* optional steps */]}

## while
Description: Loop until condition false
Input: condition: condition object → boolean, body: array of actions
Output: executes body repeatedly until condition false
Parameters:
  - condition (required): condition object evaluating to boolean
  - body (required): array of action objects
  - max_iterations (optional): integer max loops (default: 100)
Preconditions: variables referenced in condition must be bound
Postconditions: body executed until condition false or max_iterations reached
Examples:
  {"type":"while","condition":{"type":"has_value","target":"$results"},"body":[/* steps */],"max_iterations":10}

## wait
Description: Pause until condition true
Input: condition: condition object → boolean
Output: blocks until condition true or timeout
Parameters:
  - condition (required): condition object evaluating to boolean
  - timeout (optional): integer seconds (default: 30)
Preconditions: variables referenced in condition must be bound
Postconditions: execution resumes when condition true or timeout
Examples:
  {"type":"wait","condition":{"type":"has_value","target":"$results"},"timeout":30}

## say
Description: Produce output (inline display)
Input: target: literal "user", value: literal or $variable
Output: none (side effect: displays to user)
Parameters:
  - target (required): string "user"
  - value (required): literal string or $variable referencing content
Preconditions: if value is $variable, it must be bound
Postconditions: message displayed to user
Examples:
  {"type":"say","target":"user","value":"literal text or $variable"}

## ask
Description: Ask user a question and wait for response (suspends plan execution)
Input: target: literal "User" (optional), value: literal or $variable question text
Output: out: variable name → bound to response text (raw string, not Note)
Parameters:
  - target (optional): string "User" (default)
  - value (required): literal string or $variable referencing question text
  - out (required): variable name to bind response to (no $ prefix)
Preconditions: if value is $variable, it must be bound
Postconditions: 
  - Question displayed to user in main UI area (not popup)
  - Plan execution suspends until user responds or 3-turn timeout
  - out variable bound to raw response text string (empty string on timeout)
  - Response is NOT wrapped in Note - use directly or create-note if needed
Examples:
  {"type":"ask","target":"User","value":"What is your research question?","out":"user_answer"}
  {"type":"ask","value":"What year are you interested in?","out":"year"}
  {"type":"ask","value":"$prompt_text","out":"response"}

## display
Description: Show formatted content in popup (for documents/formatted output)
Input: value: $variable → Note (or target: $variable, both accepted)
Output: none (side effect: displays formatted content)
Parameters:
  - value (required): $variable referencing Note with formatted content (or use target)
Preconditions: value/target variable must be bound to Note
Postconditions: formatted content displayed in popup to user
Examples:
  {"type":"display","value":"$formatted_note"}
  {"type":"display","target":"$formatted_note"}  # Also accepted

## think
Description: Internal note (logged only)
Input: value: literal or $variable (or target: $variable, both accepted)
Output: none (side effect: logged)
Parameters:
  - value (required): literal string or $variable referencing content (or use target)
Preconditions: if value/target is $variable, it must be bound
Postconditions: thought logged internally
Examples:
  {"type":"think","value":"thought text or $variable"}
  {"type":"think","target":"$variable"}  # Also accepted

# CONDITIONS:

All conditions evaluate to boolean. Uniform form:
{"type": "<condition_type>", "target": "$variable", "field?": "optional_field", "value?": "<literal or variable>"}

## Variable State Conditions (work on both Note and Collection):
- bound: {"type": "bound", "target": "$var"} - true if $var exists
- notbound: {"type": "notbound", "target": "$var"} - true if $var doesn't exist
- has_value: {"type": "has_value", "target": "$var"} - true if $var is truthy
- empty: {"type": "empty", "target": "$var"} - true if Note is falsy/empty or Collection has 0 items

## Value Comparison Conditions (target must be Note with comparable content):
- equals: {"type": "equals", "target": "$var", "value": "expected"}
- not_equals: {"type": "not_equals", "target": "$var", "value": "unwanted"}
- greater_than: {"type": "greater_than", "target": "$score", "value": 0.8} (numeric)
- less_than: {"type": "less_than", "target": "$count", "value": 100} (numeric)
- gte: {"type": "gte", "target": "$score", "value": 0.7} (numeric)
- lte: {"type": "lte", "target": "$size", "value": 1000} (numeric)

## Membership Conditions (target can be Note or Collection):
- contains: {"type": "contains", "target": "$text", "value": "keyword"} (substring/element for Note, Note ID membership for Collection)
- not_contains: {"type": "not_contains", "target": "$tags", "value": "spam"}

## Pattern Matching (target must be Note with string content):
- matches_pattern: {"type": "matches_pattern", "target": "$text", "pattern": "regex_pattern"}

## Tool-based Conditions (target can be Note or Collection):
- tool_condition: {"type": "tool_condition", "tool": "has_field", "target": "$data", "args": {"field": "urls"}} (tool returns boolean)
"""