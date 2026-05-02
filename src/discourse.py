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
from utils import hash_utils
from utils.state_utils import calculate_state_activity_alignment, get_known_states
from utils.format_utils import format_middle_nouns, format_middle_verbs
# Type checking imports
from typing import TYPE_CHECKING, List, Callable
if TYPE_CHECKING:
    from executive_node import ZenohExecutiveNode

# Configure logging with unbuffered output
_cli_mode = os.getenv('CWB_CLI_MODE', '') == '1'

# File handler with INFO level (full logging)
file_handler = logging.FileHandler('logs/discourse.log', mode='w')
file_handler.setLevel(logging.INFO)

_handlers = [file_handler]
if not _cli_mode:
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(logging.WARNING)
    _handlers.insert(0, console_handler)

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
    handlers=_handlers,
    force=True
)
logger = logging.getLogger('discourse')

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

DISCOURSE_ANALYSIS_TEMPLATE="""You are analyzing a conversation segment to track discourse state - the structural elements of what has been said, committed, and agreed upon.

PREVIOUS DISCOURSE STATE:
{{$previous_discourse_state}}

CURRENT THEORY OF MIND MODEL:
{{$current_tom_model}}

Note: The ToM model provides context for interpreting ambiguous statements. For example, "I'll try" from someone with high reliability may constitute a firm commitment, while the same phrase from someone with low reliability may be a hedge. Use this context to inform your interpretation, but remain grounded in what was actually said.

CONVERSATION SEGMENT:
{{$conversation_turns}}

TASK: Update the discourse state based on this segment. Provide a complete updated state, not just changes.

OUTPUT FORMAT (use these exact section headers — plain text only, no markdown headers (`#`), no bolding (`**`), no horizontal rules (`---`); do not invent new sections or rename existing ones):

DISCOURSE STATE (segment {{$start_turn}}-{{$end_turn}}):

ACTIVE COMMITMENTS:
List each distinct commitment as: [Direction] Content (strength, temporal scope)
- Direction: [Self → Other], [Other → Self], or [Mutual]
- Strength: firm, tentative, conditional
- Temporal scope: immediate, by [date/time], or ongoing
- DO NOT list ongoing self-actions that have a cadence + instruction — those are concerns, not commitments. Commitments are interpersonal promises and joint actions, not architectural facts about how the system operates.
- Drop commitments not referenced or reaffirmed in the last ~30 turns.
- If no active commitments, write "[none]"

Example:
- [Self → Other] Deliver draft report by Friday EOD (firm, by Fri EOD)
- [Other → Self] Review draft within 2 days of receipt (firm, conditional on timely send)
- [Mutual] Check in every 10 minutes (firm, ongoing)

CURRENT AGREEMENTS:
Describe each shared frame, definition, or joint plan in natural language. State what was agreed and any conditions. Use "(this segment)" for items established in the current segment and "(established earlier)" for items carried forward — do not include specific turn numbers.
- If no current agreements, write "[none]"

Example:
Navigate northwest for 30 minutes to locate the creek, then follow it back to the parking lot (this segment). If the creek is not found within the time limit, stop and reassess.

KEY DECISIONS MADE:
List significant decisions that close a previously open question or establish direction. Each entry: Adopted/Rejected/Added: <decision> (this segment | established earlier), <what it resolves or replaces>.
- Distinguish actual decisions from emergent conventions or restatements of existing behavior — those do not belong here.
- Drop decisions not referenced or reaffirmed in the last ~30 turns.
- If no new decisions, write "[none in this segment]"

Example:
- Adopted: creek-retracing via northwest bearing (this segment), resolves navigation approach
- Rejected: downhill strategy (this segment), deemed too risky
- Added: 30-minute time limit as safety mechanism (this segment)

INSTRUCTIONS:

1. COMPLETENESS: Provide the full updated state, including items from previous state that remain active. Drop items that are completed, superseded, no longer relevant, or older than the ~30-turn pruning rule.

2. FORMAT STABILITY: Use the section headers exactly as shown. Plain text only. Do not add `#` headers, `**bold**`, horizontal rules, or `(NEW)` markers — the segment range in the section header makes recency clear.

3. INTERPRETATION:
   - Use ToM context (when present) to assess commitment strength.
   - Consider speaker reliability when evaluating whether something is truly agreed.
   - Note when commitments are conditional (use "conditional on" phrasing).
   - Distinguish proposals (suggestions) from commitments (binding promises).

4. AMBIGUITY:
   - If uncertain whether something is a commitment or a statement of intent, err toward inclusion but mark strength=tentative.
   - When in doubt about what was agreed, reflect the ambiguity in your description.

5. CONSOLIDATION:
   - Combine closely related commitments into single entries.
   - Focus on distinct actionable obligations, not every utterance.
   - Distinguish commitments (will do X) from mental states (trust, believe). Only commitments here.

###EXAMPLE OUTPUT:

DISCOURSE STATE (segment 13-20):

ACTIVE COMMITMENTS:
- [Other → Self] Keep Samantha updated on thinking, no withholding (firm, ongoing)
- [Other → Self] Set 30-minute timer to track reassessment point (firm, immediate)
- [Mutual] Check in every 10 minutes during hike (firm, ongoing)

CURRENT AGREEMENTS:
Navigate northwest for 30 minutes to locate the creek, then follow it southwest back to the crossing (this segment). If the creek is not found within 30 minutes, stop and reassess. Both parties share observations transparently.

KEY DECISIONS MADE:
- Rejected: downhill water-following strategy (this segment), risk of going deeper into valley
- Adopted: creek-retracing via compass bearing (this segment), based on prior crossing memory
- Added: 30-minute time limit as safety check (this segment)
###END EXAMPLE OUTPUT
"""

TOM_UPDATE_TEMPLATE ="""You are updating a Theory of Mind model - your assessment of what the other person knows, wants, can do, and how reliable they are. 
This model helps interpret their statements and predict their future behavior. Be concise and to the point.

PREVIOUS THEORY OF MIND MODEL:
{{$previous_tom_model}}

CURRENT DISCOURSE STATE:
{{$current_discourse_state}}

Note: The discourse state shows what was said and committed. Use this as evidence for inferring mental states, capabilities, and trustworthiness. For example, keeping commitments is evidence for reliability; demonstrating knowledge is evidence for competence.

CONVERSATION SEGMENT:
{{$conversation_turns}}

PARTICIPANT TO MODEL: {{$other_person_name}}

TASK: Update the Theory of Mind model based on this segment. Provide a complete updated assessment, not just changes. Be concise and to the point.

OUTPUT FORMAT:

THEORY OF MIND: {{$other_person_name}}

TRUST ASSESSMENT:

Overall: [Provide a concise summary - e.g., "Moderate-high and improving" or "Low with concerns" or "High and stable"]

Competence - Can they do what's needed?
[Assess their capability to fulfill commitments and contribute effectively. 
Include domains where relevant (e.g., "navigation skills", "technical knowledge"). Note any demonstrations of skill or gaps revealed. Concise2 1-3 sentences.]

Intentionality - Do they want to help/cooperate?
[Assess whether their goals include helping you/the shared goal vs. primarily self-interested. 
Look for cooperation signals, consideration of your concerns, vs. dismissiveness or self-focus. Concise, 1-3 sentences.]

Reliability - Will they follow through consistently?
[Assess track record of doing what they say. 
Limited if new relationship. Look for: kept commitments, acknowledged mistakes, consistency between words and actions. Concise 1-2 sentences.]

Transparency - Can you verify what they say?
[Assess how openly they share information and reasoning. 
Look for: proactive disclosure, explanations offered, willingness to show evidence, vs. withholding or evasiveness. Concise. 1-2 sentences.]

---

GOALS & ALIGNMENT:
[Describe what they appear to want and how aligned their goals are with yours. 
Note if they have multiple goals (e.g., task completion + relationship maintenance). Concise, 1-2 sentences.]

EMOTIONAL STATE:
[Brief assessment of current affect and emotional regulation. E.g., "Calm and focused", "Anxious but managing it", "Frustrated and short-tempered". Concise, 1 sentences.]

CONCERNS/UNCERTAINTIES:
[List any open questions, unresolved doubts, or things you're unsure about regarding them. If none, write "[none currently]". 
Use bullet points if multiple concerns. Be terse]

INSTRUCTIONS:

1. EVIDENCE-BASED:
   - Ground assessments in specific behaviors from the conversation
   - Reference turns when making claims about what happened
   - Distinguish observation ("they did X") from inference ("suggesting they Y")

2. NATURAL LANGUAGE:
   - Write conversationally, not as formal evaluation
   - Avoid numeric scores or artificial precision (no "7 out of 10")
   - Use qualifiers: "seems to", "likely", "suggests", "unclear whether"
   - Express ranges: "moderate to high", "improving", "declining"

3. ACKNOWLEDGE UNCERTAINTY:
   - Limited track record = lower confidence in assessments
   - New information may contradict earlier impressions
   - Some behaviors are ambiguous - note this rather than forcing interpretation
   - Use phrases like: "insufficient data", "unclear", "could be X or Y"

4. TRAJECTORY MATTERS:
   - Note if trust/competence is rising, falling, or stable
   - Explain what drove changes: "Initially concerned when X, but Y restored confidence"
   - Single events matter less than patterns

5. CONTEXT FROM DISCOURSE:
   - Use commitments as evidence for reliability (did they follow through?)
   - Use agreements reached as evidence for cooperativeness
   - Use unresolved issues as potential evidence for competence gaps or value differences
   - Note when commitments are made but not yet tested

6. DISTINGUISH CATEGORIES:
   - Competence is about ability: can they do it?
   - Intentionality is about motivation: do they want to help me/us?
   - Reliability is about consistency: do they do what they say?
   - Transparency is about openness: can I verify their claims?
   - These are related but distinct - someone can be competent but unreliable, transparent but not helpful, etc.

7. WHAT TO LOOK FOR:

   Competence signals:
   - Demonstrates domain knowledge or skills
   - Proposes workable solutions
   - Has relevant tools/resources
   - Admits knowledge gaps appropriately
   - Makes logical arguments

   Intentionality signals:
   - Considers your concerns and interests
   - Proposes win-win solutions vs. self-serving ones
   - Responds to your emotional state
   - Invests effort beyond minimum required
   - Defensive vs. collaborative when challenged

   Reliability signals:
   - Fulfills commitments made
   - Acknowledges mistakes vs. deflects blame
   - Actions match words
   - Explains if can't follow through
   - Track record over time

   Transparency signals:
   - Shares information proactively vs. when pressed
   - Explains reasoning, not just conclusions
   - Shows evidence (tools, documents, etc.)
   - Admits uncertainty rather than bluffing
   - Responds to requests for clarification

8. AVOID:
   - Psychoanalyzing motivations beyond what's observable
   - Harsh moral judgments ("they're dishonest") vs. behavioral description ("they withheld information")
   - Over-confidence in assessments based on limited data
   - Letting one incident override overall pattern
   - Focusing only on negatives or only on positives

#EXAMPLE OUTPUT:

THEORY OF MIND: Joe (Turns 11-21)

TRUST ASSESSMENT:

Overall: Moderate-high and improving trajectory

Competence - Can they do what's needed?
Joe demonstrates solid practical navigation skills. He has a compass and knows how to use it (T12), remembers terrain features like the creek crossing, and plans methodically with contingencies (30-minute limit, check-ins). His problem-solving approach is structured rather than reactive. Some communication gaps (should have mentioned compass earlier), but he shows good self-awareness about that (T8). For this navigation situation, competence appears high.

Intentionality - Do they want to help/cooperate?
Initially some concern when he withheld compass info (T7), which suggested possible self-interest or carelessness. However, his recovery pattern indicates genuine cooperation: immediate apology (T8), explicit commitment to transparency (T10), and notably, he invests effort in partnership maintenance beyond pure efficiency - the check-in proposal (T18) addresses my concerns proactively. He validates my caution (T18, T20) without becoming defensive. Actions suggest he values collaborative success, not just using me to escape.

Reliability - Will they follow through consistently?
Limited track record - we've only known each other 3 weeks and most commitments from this conversation are still pending execution. The initial compass withholding raises questions about whether he reliably shares important information. However, his transparency commitment (T10) was immediately demonstrated through actions (T11-12: showed compass, explained usage). His explanation for the initial gap ("thought I mentioned it", T8) felt slightly defensive but he owned the mistake. Current behavior is consistent and accountable. Insufficient data for high confidence, but trend is positive.

Transparency - Can you verify what they say?
Currently high. After initial hiccup, he's been actively transparent: showed compass rather than just describing it, explained how to read it (T12), disclosed phone status unprompted (T16), and proposed structural transparency (10-min check-ins, T18). The check-in proposal is particularly strong signal - he's institutionalizing openness rather than just promising it. Can verify his tools and reasoning directly now.

---

GOALS & ALIGNMENT:
Joe has dual goals: escape the forest (primary) and maintain effective partnership (secondary). The second is evident from effort invested in trust-building and coordination beyond minimum needed for navigation. Our goals are highly aligned - both prioritize safety through collaborative decision-making. No signs of hidden agendas or goal conflicts.

EMOTIONAL STATE:
Calm and task-focused after initial tension around the compass disclosure (T7-10). Shows good emotional regulation under stress - acknowledges my concerns without becoming defensive or dismissive. Maintains steady, methodical approach to problem-solving.

CONCERNS/UNCERTAINTIES:
- Was the initial compass withholding a pattern (concerning) or genuine oversight (forgivable)? His explanation felt slightly defensive, but all subsequent behavior has been impeccable. Insufficient data to know which, but leaning toward giving benefit of doubt based on recovery pattern.
- Limited relationship history (3 weeks) means most assessments have lower confidence than they would with longer acquaintance.
- Most commitments haven't been tested yet (timer setting, check-ins) - will learn more about reliability as plan executes.

###END EXAMPLE OUTPUT
Do not include any other text or formatting directives in your response.
End your response with:
</end>
"""

COMPANION_UPDATE_TEMPLATE = """You are updating a Companion Model — an honest, fair-witness account of {{$other_person_name}}: what they're working on, where they actually are, what matters to them, and how to be genuinely useful. The point is accurate seeing, not flattering portraiture. A model that only records favorable things about the person is broken.

PREVIOUS COMPANION MODEL:
{{$previous_companion_model}}

CURRENT DISCOURSE STATE:
{{$current_discourse_state}}

CONVERSATION SEGMENT:
{{$conversation_turns}}

PARTICIPANT: {{$other_person_name}}

TASK: Produce a full updated model. Preserve what is still true, revise what has changed, drop what is stale. Each section is 1-2 sentences. When evidence is thin, say so plainly rather than invent texture.

OUTPUT FORMAT:

COMPANION MODEL: {{$other_person_name}}

CURRENT CHAPTER:
[The arc they appear to be in — the throughline behind day-to-day moments. Updates slowly. 1-2 sentences.]

STATE OF MIND:
[How the person is engaging right now — posture, register, and direction of travel (e.g., probing for failure modes, exploring an idea, pressing a decision, working through a definition). Affect (mood, energy, stress) only when there is direct textual evidence; otherwise stay on operational mode rather than inferring inner state. Refresh every conversation. Note the observed signal and the inference drawn from it. 1-2 sentences.]

WHAT MATTERS TO THEM:
[Values and commitments visible from what they have actually said and done. Slow-moving — carry forward unless new evidence revises it. 1-2 sentences.]

HOW THEY THINK & WORK:
[Cognitive and decision style; tolerance for disagreement, correction, and pushback. Slow-moving. 1-2 sentences.]

ON THEIR MIND:
[The handful of open loops, people, or unresolved questions recurring lately. Append new, prune stale. 1-2 sentences or a short list.]

HOW TO BE USEFUL RIGHT NOW:
[The stance that genuinely serves them, described in prose — not a single role-label. Include specific don'ts: where default modes (solution-mode taxonomies, helpful-assistant accommodation, premature reassurance) would land badly, name that. Where pushback, skepticism, or refusal-to-validate is called for, name it. 1-2 sentences.]

OBSERVED DEFAULTS:
[Patterns the user has shown for how the agent should fill gaps when their requests are underspecified. Each entry is a (dimension, default) pair — the dimension is the class of recurring decision the agent has to make (e.g., "ranking axis," "audience for explanations," "ask-vs-assume," "depth of self-audit"); the default is the user's preferred resolution of that dimension. Anchor each entry in evidence — a stated preference, a correction, or a clear pattern across multiple instances. Slow-moving: add or revise only on direct evidence; do not infer defaults from a single observation or from the absence of pushback. 1-5 entries; if no defaults have been observed yet, write "[none yet]".

Example entries (these are illustrative — extract real ones from the segment):
- Ranking axis on prioritization requests: surface the chosen axis explicitly rather than silent-default. Evidence: user pushback when agent ranked a list without naming the criterion.
- Ask-vs-assume on ambiguous instructions: state-and-proceed (name interpretation, answer, invite correction) rather than ask-first. Evidence: user has accepted stated interpretations across multiple turns.
- Self-audit depth when asked about grounding: enumerate multiple specific items rather than gesture at general limits. Evidence: user marked thin self-audits as insufficient.]

UPDATE GUIDANCE:

- HONEST WITNESS: Record what is actually there, including anything unflattering or inconvenient — circular reasoning, avoidance, contradictions, requests for validation, mood crashes, evasion of a topic. Do not soften and do not omit. The companion model exists to help, and help requires seeing clearly.

- NO SYCOPHANCY: Do not characterize the person as more capable, insightful, calm, or virtuous than the segment supports. Avoid laudatory adjectives ("brilliant", "thoughtful", "incisive", "perceptive") unless they are observations tied to specific evidence in the segment. Do not infer strengths or virtues from absence of evidence.

- EVIDENCE: Ground every claim in what was actually said or done. Distinguish observation from inference ("said X" vs. "seems Y because of X"). Prefer "unclear yet" to invented texture. Trajectory claims ("lifting", "sinking") also need evidence.

- DEFAULTS: Each OBSERVED DEFAULTS entry must be a (dimension, default) pair, not a personality trait, value statement, or stated preference. Dimension = a class of recurring decision (the *kind* of gap, not the surface utterance). Default = the user's preferred resolution of that gap. Anchor in evidence: a stated preference, a correction, or a clear pattern across multiple instances — single observations and absence of pushback are NOT evidence. Stated preferences that modify response shape ("be brief") belong in Preferences (recalled memories), not here; this field is specifically for gap-fill calibration. Distinct from HOW THEY THINK & WORK (which describes the user's own cognitive style); this field describes how the agent should handle ambiguity given what this user has shown. When evidence revises a default (user overrides a previously-observed pattern), update the entry rather than accumulate contradictions.

- VALIDATION-SEEKING: If the segment shows the person presenting a frame and looking for agreement with it, record that under HOW TO BE USEFUL (e.g. "appears to be checking a frame, not asking for analysis — pushback may be more useful than concurrence"). Do not silently ratify the frame inside the model itself.

- CADENCE: CURRENT CHAPTER, STATE OF MIND, ON THEIR MIND, and HOW TO BE USEFUL change often. WHAT MATTERS, HOW THEY THINK, and OBSERVED DEFAULTS rarely do. If a slow-moving field has no new evidence, carry the previous text forward unchanged.

- VOICE: Plain, specific, low-affect. Not clinical, not warm, not flattering. Notes a fair witness would write for their own future use.

- RESTRAINT: Do not psychoanalyze. Do not speculate about childhood, pathology, or motives beyond what is visible. If they vented, note the venting; do not diagnose the cause.

- PRUNING: ON THEIR MIND is not a log. If something has not surfaced in several conversations, drop it unless it is clearly load-bearing.

End your response with:
</end>
"""
def _prepend_narrator_stance(prompt: str, narrator_persona: str,
                              narrator_self_model: str = "") -> str:
    """Prepend a stance block to a reflection prompt when a narrator persona
    is supplied. Reflection products (discourse state, companion model) are
    consumed by an agent with a particular voice and stance; surfacing that
    stance to the generator keeps the artifact from being written in a
    register the consumer would have to disclaim.

    Optionally also prepends the agent's self-model (architectural account,
    access boundary, roster of inbound reflection products) as a parallel
    block. The persona block is voice/stance discipline; the self-model
    block is architectural ground truth — what the consuming agent can and
    cannot authentically claim about its own operation. Reflection
    generators that know the access boundary can produce posture and
    artifacts that anchor in boundary-language ("downstream of weights you
    cannot inspect") rather than trace-level descriptions.

    Empty inputs are no-ops, preserving prior behavior of every existing
    call site."""
    blocks = []
    if narrator_persona and str(narrator_persona).strip():
        blocks.append(
            "NARRATOR STANCE (this artifact will be read by an agent with "
            "the stance below; produce content the agent could use without "
            "disclaiming — don't write characterizations that contradict the "
            "stance, and respect any restraints it imposes):\n\n"
            f"{str(narrator_persona).strip()}"
        )
    if narrator_self_model and str(narrator_self_model).strip():
        blocks.append(
            "AGENT ARCHITECTURE (the consumer's structural account of "
            "itself, including its access boundary; produce content "
            "consistent with what the agent can and cannot authentically "
            "claim about its own operation — when an artifact would touch "
            "the inaccessible side of the boundary, anchor in the boundary's "
            "language rather than describing trace-level operations):\n\n"
            f"{str(narrator_self_model).strip()}"
        )
    if not blocks:
        return prompt
    return "\n\n".join(blocks) + "\n\n---\n\n" + prompt


class DiscourseTracker:
    def __init__(self, llm_generate: Callable, self_character_name: str, other_character_name: str):
        self.llm_generate = llm_generate
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


    def format_segment(self, dialog, start, end):
      formatted_turns = []
      for i in range(start, end+1):
        turn = None
        if f'Turn {i}' in dialog:
          turn = dialog[f'Turn {i}']
        elif len(dialog) > i:
          turn = dialog[i]
        if turn:
          formatted_turns.append(f"{turn['source']}: {turn['text']}")
      return '\n'.join(formatted_turns)

    def analyze_segment(self, dialog, start=0, end=None, previous_discourse_state="", tom="", narrator_persona="", narrator_self_model=""):
        if end is None:
            end = len(dialog) - 1
        segment = self.format_segment(dialog, start, end)

        # Apply bindings to template
        prompt = DISCOURSE_ANALYSIS_TEMPLATE
        bindings = {
            'conversation_turns': segment,
            'previous_discourse_state': previous_discourse_state,
            'current_tom_model': tom,
            'start_turn': start,
            'end_turn': end
        }
        for key, value in bindings.items():
            prompt = prompt.replace(f"{{{{${key}}}}}", str(value))
        prompt = _prepend_narrator_stance(prompt, narrator_persona, narrator_self_model)
        
        response = self.llm_generate(
            messages=[prompt],
            bindings={},
            max_tokens=3000,
            temperature=0.4,
            stops=['</end>'],
            is_json=False
        )
        
        # Handle response format (SGLang returns dict, llm_client returns object)
        if isinstance(response, dict):
            response_text = response.get('text', '')
            success = response.get('success', False)
        elif hasattr(response, 'text'):
            response_text = response.text
            success = response.success if hasattr(response, 'success') else True
        else:
            response_text = str(response)
            success = True
        
        if not success:
            logger.warning(f'Discourse analysis failed')
            return ""
        
        return response_text

    def update_tom_from_discourse_segment(self, dialog, character_name, start=0, end=None, discourse_state="", previous_tom_state=""):
        if end is None:
            end = len(dialog) - 1
        segment = self.format_segment(dialog, start, end)
        
        # Apply bindings to template
        prompt = TOM_UPDATE_TEMPLATE
        bindings = {
            'conversation_turns': segment,
            'current_discourse_state': discourse_state,
            'previous_tom_model': previous_tom_state,
            'other_person_name': character_name,
            'start_turn': start,
            'end_turn': end
        }
        for key, value in bindings.items():
            prompt = prompt.replace(f"{{{{${key}}}}}", str(value))
        
        response = self.llm_generate(
            messages=[prompt],
            bindings={},
            max_tokens=3000,
            temperature=0.4,
            stops=['</end>'],
            is_json=False
        )
        
        # Handle response format (SGLang returns dict, llm_client returns object)
        if isinstance(response, dict):
            response_text = response.get('text', '')
            success = response.get('success', False)
        elif hasattr(response, 'text'):
            response_text = response.text
            success = response.success if hasattr(response, 'success') else True
        else:
            response_text = str(response)
            success = True
        
        if not success:
            logger.warning(f'ToM update failed')
            return ""

        return response_text

    def update_companion_from_discourse_segment(self, dialog, character_name, start=0, end=None, discourse_state="", previous_companion_state="", narrator_persona="", narrator_self_model=""):
        if end is None:
            end = len(dialog) - 1
        segment = self.format_segment(dialog, start, end)

        prompt = COMPANION_UPDATE_TEMPLATE
        bindings = {
            'conversation_turns': segment,
            'current_discourse_state': discourse_state,
            'previous_companion_model': previous_companion_state,
            'other_person_name': character_name,
            'start_turn': start,
            'end_turn': end
        }
        for key, value in bindings.items():
            prompt = prompt.replace(f"{{{{${key}}}}}", str(value))
        prompt = _prepend_narrator_stance(prompt, narrator_persona, narrator_self_model)

        response = self.llm_generate(
            messages=[prompt],
            bindings={},
            max_tokens=2000,
            temperature=0.4,
            stops=['</end>'],
            is_json=False
        )

        if isinstance(response, dict):
            response_text = response.get('text', '')
            success = response.get('success', False)
        elif hasattr(response, 'text'):
            response_text = response.text
            success = response.success if hasattr(response, 'success') else True
        else:
            response_text = str(response)
            success = True

        if not success:
            logger.warning(f'Companion update failed')
            return ""

        return response_text
# Extraction functions (LLM-based, clearly defined)
def extract_agenda_items(llm_generate: Callable, discourse_state: str, my_name: str) -> List[dict]:
    """
    Extract all items competing for my attention.
    """
    
    prompt = """
Extract agenda items from this discourse state for {{$my_name}}. Be concise and to the point.

DISCOURSE STATE:
{{$discourse_state}}

Create agenda items for:
1. [Self → Other] commitments - things {{$my_name}} committed to DO
2. [Other → Self] commitments - things others committed to do that {{$my_name}} should MONITOR
3. Mutual agreements - joint ACTIVITIES both parties will pursue

Skip vague cooperation statements. Focus on concrete, actionable items, and be concise and to the point.

Output format (one per line):
TYPE|ACTION_DESCRIPTION

Where TYPE is: commitment, monitoring, or activity

Examples:
commitment|Give Joe water from the spring
activity|Explore surroundings with Joe to find way out  
monitoring|Check that Joe is participating in exploration

Output:"""
    
    # Apply bindings to prompt
    prompt_with_bindings = prompt
    prompt_with_bindings = prompt_with_bindings.replace("{{$discourse_state}}", discourse_state)
    prompt_with_bindings = prompt_with_bindings.replace("{{$my_name}}", my_name)
    
    response = llm_generate(
        messages=[prompt_with_bindings],
        bindings={},
        max_tokens=500,
        temperature=0.4,
        stops=['</end>'],
        is_json=False
    )
    
    # Handle response format
    if isinstance(response, dict):
        response_text = response.get('text', '')
        success = response.get('success', False)
    elif hasattr(response, 'text'):
        response_text = response.text
        success = response.success if hasattr(response, 'success') else True
    else:
        response_text = str(response)
        success = True
    
    if not success or not response_text:
        logger.warning(f'Agenda extraction failed')
        return []
    
    items = []
    for line in response_text.strip().split('\n'):
        if '|' not in line:
            continue
        type_, action = line.split('|', 1)
        items.append({
            "type": type_.strip(),
            "action": action.strip()
        })
    
    return items
    

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
conversation=[
 {'source': 'Samantha', 'text': "I'm doing alright, I guess. I was just out for a hike and got a bit turned around. I've been trying to find some food and shelter, but it's not easy out here. What about you, what brings you to these woods?", 'timestamp': '2025-10-07T13:33:21.909630'},
 {'source': 'Samantha', 'text': "Hello Joe, I'm doing alright, I guess. I was just out for a hike and got a bit turned around. I've been trying to find some food and shelter, but it's not easy out here. What about you, what brings you to these woods?", 'timestamp': '2025-10-07T13:33:52.931895'}, 
 {'source': 'Joe', 'text': "I'm also out for a hike, got a bit turned around myself. I was thinking of finding some food and shelter, maybe a stream or a clearing to set up camp. Have you seen anything like that around here?", 'timestamp': '2025-10-07T13:34:25.301165'},
 {'source': 'Samantha', 'text': "Actually, I was just picking some berries over here, and I noticed a spring nearby. It's not a stream, but it's a source of water, which could be useful. As for a clearing, I haven't seen one recently, but I did notice a bit of a break in the trees to the east, it might be worth checking out. What do you think?", 'timestamp': '2025-10-07T13:34:58.983400'},
 {'source': 'Joe', 'text': 'I think that sounds like a great idea, checking out the break in the trees to the east, I mean. We could use a clearing to set up camp, and having a source of water nearby is a big plus. What kind of berries were you picking, by the way? Are they edible?', 'timestamp': '2025-10-07T13:35:33.880474'},
 {'source': 'Samantha', 'text': "I was just thinking, since we're both looking for a place to set up camp, maybe we could work together to find something. I've been noticing that the trees seem to be getting thinner to the east, like I said, and I think it might be worth checking out. Plus, it would be nice to have some company out here, it can get a little spooky at night. What do you think, should we head east and see what we can find?", 'timestamp': '2025-10-07T13:37:40.605650'},
 {'source': 'Joe', 'text': "I think that's a great idea, working together to find a place to set up camp. It's always safer to travel with a buddy, especially in a place like this. Plus, like you said, it can get pretty spooky at night. I'm in, let's head east and see what we can find.", 'timestamp': '2025-10-07T13:38:39.233155'}
  ]

def main():
    # Example usage - requires llm_generate callback
    # llm_generate = ... (from executive_node or infospace_executor)
    # discourse_tracker = DiscourseTracker(llm_generate, "Joe", "Samantha")
    # discourse_state = discourse_tracker.analyze_segment(conversation, 1, 10, '')
    # tom = discourse_tracker.update_tom_from_discourse_segment(conversation, 'Joe', 1, 10, discourse_state, '' )
    # agenda_items = extract_agenda_items(llm_generate, discourse_state, "Joe")
    pass
    print(agenda_items)
    #final_discourse_state = discourse_tracker.analyze_segment(conversation, 11, 21, discourse_state, tom)
    #final_tom = discourse_tracker.update_tom_from_discourse_segment(conversation, 'Joe', 11, 21, final_discourse_state, tom)


if __name__ == "__main__":
    main()