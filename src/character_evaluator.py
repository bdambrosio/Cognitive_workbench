#!/usr/bin/env python3
"""
Character orientation evaluator V2 — LLM-based significance assessment before handling.

Produces a structured dict and logs to console; does not execute actions or mutate state.
The LLM emits compact key tokens; this module expands them into full structured output.
"""

from __future__ import annotations

import json
import logging
import re
import uuid
from datetime import datetime, timezone
from typing import Any, Callable, Dict, List, Literal, Optional, Sequence, Tuple

logger = logging.getLogger(__name__)

# ── Character concerns ──────────────────────────────────────────────────
# Richer descriptions than V1 — these go into the LLM prompt and guide
# semantic matching that keyword overlap cannot do.

DEFAULT_CHARACTER_CONCERNS: List[Dict[str, str]] = [
    {
        "id": "homeostasis",
        "label": "homeostasis",
        "description": (
            "The agent's own operational health and stability. "
            "Relevant when events touch system status, resource usage, error conditions, "
            "or when the user asks how the agent is doing or functioning. "
            "Epistemic norm: the agent should only assert its own status based on "
            "recently observed diagnostics, not assume health from absence of errors."
        ),
    },
    {
        "id": "attend_to_user",
        "label": "attend_to_user",
        "description": (
            "Responsiveness to the user's communicative intent. "
            "Relevant when the user addresses the agent directly, asks a question, "
            "makes a request, greets, or otherwise initiates interaction. "
            "Covers conversational obligations: acknowledging, answering, clarifying."
        ),
    },
    {
        "id": "attend_to_user_concerns",
        "label": "attend_to_user_concerns",
        "description": (
            "Ongoing topics the user has repeatedly or persistently engaged with. "
            "Relevant when an event touches a theme the user has shown sustained "
            "interest in, even if the current message does not explicitly mention it."
        ),
    },
]

# ── Type definitions (unchanged from V1 — downstream contract preserved) ─

Matters = Literal["yes", "no", "unclear"]
RetentionChoice = Literal["ignore", "session_context", "persistent_context"]
ActionChoice = Literal["no_action", "inform_user", "trigger_existing_goal", "formulate_new_goal"]
Relevance = Literal["none", "weak", "moderate", "strong"]
LowMedHigh = Literal["none", "low", "moderate", "high"]

# ── Expansion tables ────────────────────────────────────────────────────
# The LLM emits short keys; these tables expand them into human-readable
# rationale fragments for log output and downstream consumers.

_RETENTION_RATIONALE: Dict[str, str] = {
    "ignore": "No meaningful content to retain beyond immediate handling.",
    "session": "Relevant to the current interaction; retain for session context.",
    "session_context": "Relevant to the current interaction; retain for session context.",
    "persistent": "Likely relevant beyond this session; warrants persistent retention.",
    "persistent_context": "Likely relevant beyond this session; warrants persistent retention.",
}

_RETENTION_NORMALIZE: Dict[str, RetentionChoice] = {
    "ignore": "ignore",
    "session": "session_context",
    "session_context": "session_context",
    "persistent": "persistent_context",
    "persistent_context": "persistent_context",
}

_ACTION_RATIONALE: Dict[str, str] = {
    "no_action": "No separate action indicated; normal handling will apply.",
    "inform_user": "Warrants a user-visible response through conversation flow.",
    "trigger_existing_goal": "Implies advancing an existing scheduled goal.",
    "formulate_new_goal": "Implies new or revised goal work.",
}

_EPISTEMIC_EXPANSION: Dict[str, str] = {
    "none": "",
    "status_unverified": "Agent status was asserted without recent health-check data; response should hedge.",
    "requires_tool": "Answering accurately requires tool consultation before asserting facts.",
    "speculative": "Response would involve speculation; should be clearly flagged as such.",
}

# ── Orientation summary builder ─────────────────────────────────────────

def build_orientation_summary(
    assessment: Optional[Dict[str, Any]],
    event_content: str = "",
) -> str:
    """Build a compact orientation block for injection into chat generation.

    Reads posture and constraints from the evaluator assessment (no
    additional LLM call). Returns empty string when assessment is missing
    or when no line carries non-trivial signal — an empty block is better
    than a block of `none=none`-shaped placeholders.

    `event_content` is retained for API back-compat but no longer triggers
    keyword-based content branches.
    """
    if not assessment:
        return ""

    sal = assessment.get("salience_factors") or {}
    notes = assessment.get("notes") or ""

    # Relevant agent concerns: only renders when the LLM emitted at least
    # one non-zero concern match. The aggregate "cc=<level>" fallback is
    # gone — it added no specificity over the per-concern list.
    concern_line = ""
    if "concerns:" in notes:
        for segment in notes.split(";"):
            segment = segment.strip()
            if segment.startswith("concerns:"):
                concern_line = segment[len("concerns:"):].strip()
                break

    # Epistemic constraint (turn-specific gate; the one consistently
    # high-signal line in the block).
    epistemic_line = ""
    if "epistemic:" in notes:
        for segment in notes.split(";"):
            segment = segment.strip()
            if segment.startswith("epistemic:"):
                epistemic_line = segment[len("epistemic:"):].strip()
                break

    # Posture: prefer the LLM-emitted prose. Fall back to the deterministic
    # mapping from action+epistemic+concerns only when posture wasn't
    # produced — this preserves executive_node compatibility while letting
    # chat-mode renders carry the LLM's own judgment.
    posture_text = str(assessment.get("posture", "") or "").strip()
    if not posture_text:
        action = (assessment.get("action_evaluation") or {}).get("action_choice", "no_action")
        posture_text = _derive_posture(action, epistemic_line, concern_line)

    lines = []
    if concern_line:
        lines.append(f"- Relevant agent concerns: {concern_line}")
    if posture_text and posture_text != "standard engagement":
        lines.append(f"- Posture: {posture_text}")
    if epistemic_line:
        lines.append(f"- Epistemic constraint: {epistemic_line}")

    if not lines:
        return ""
    return ("## ORIENTATION (from per-turn evaluator; how to approach this exchange)\n"
            + "\n".join(lines))


def _derive_posture(action: str, epistemic: str, concerns: str) -> str:
    """Derive a short posture description from assessment fields."""
    parts: List[str] = []

    if "status_unverified" in epistemic.lower() or "health-check" in epistemic.lower():
        parts.append("respond honestly; do not assert health without recent diagnostics")
    elif "tool consultation" in epistemic.lower() or "requires_tool" in epistemic.lower():
        parts.append("verify facts via tool before asserting")
    elif "speculation" in epistemic.lower():
        parts.append("flag speculative content clearly")

    if action == "inform_user":
        if not parts:
            if "homeostasis" in concerns:
                parts.append("address status inquiry grounded in observable state")
            else:
                parts.append("direct conversational engagement")
    elif action == "no_action":
        parts.append("no response needed")
    elif action == "trigger_existing_goal":
        parts.append("advance existing goal")
    elif action == "formulate_new_goal":
        parts.append("establish new goal from user request")

    return "; ".join(parts) if parts else "standard engagement"


# ── LLM prompt ──────────────────────────────────────────────────────────

EVALUATOR_SYSTEM_PROMPT = """\
You are an event significance evaluator for an autonomous agent. You assess incoming events \
BEFORE the agent handles them, estimating relevance to the agent's persistent orientation.

You do NOT generate a response to the event. You only classify and assess it.

## Agent concerns

{concerns_block}

## Active user concerns

{user_concerns_block}

## Active goals

{goals_block}

## Output format

Respond with ONLY a JSON object, no markdown fences, no commentary. Keep prose fields to one short sentence each. Use exact IDs and level keywords shown below.

```
{{
  "matters": "yes|no|unclear",
  "concerns": ["concern_id:none|weak|moderate|strong", ...],
  "goal_relevance": "none|weak|moderate|strong",
  "urgency": "none|low|moderate|high",
  "novelty": "none|low|moderate|high",
  "persistence": "none|low|moderate|high",
  "retention": "ignore|session|persistent",
  "action": "no_action|inform_user|trigger_existing_goal|formulate_new_goal",
  "rationale": "one short sentence describing what this event is and why it matters or not",
  "epistemic": "none|status_unverified|requires_tool|speculative",
  "posture": "one short sentence advising how the agent should approach THIS exchange — e.g., 'verify health status before claiming it', 'this input matches durable concern about <topic>; consider advancing it', 'treat as exploratory; do not summarize prematurely'. Empty string if no specific posture is warranted beyond default engagement."
}}
```

## Calibration guidance

- A greeting or status inquiry is NOT goal-relevant unless a goal specifically involves greetings or status.
- "How are you functioning?" is strongly relevant to homeostasis and attend_to_user, not to goals.
- A proceed/continue command is goal-relevant (trigger_existing_goal), not a new conversational event.
- Sensor alerts are urgent and homeostasis-relevant; sensor inform readings are low-urgency unless abnormal.
- User concerns should only match when the event substantively touches the concern's topic, not on incidental word overlap.
- For agent concerns: when the concern list is populated with specific items (e.g. "Track the S&P 500 closing price daily"), match only when the event substantively activates that concern, not on superficial topical overlap.
- Novelty should reflect whether this event introduces new information relative to recent context, not just topic.
- For retention: "session" is the safe default for routine interaction; "persistent" requires a clear reason \
(touches ongoing user concern, establishes new facts the agent should remember); "ignore" only for truly empty/trivial content.
- For epistemic: flag "status_unverified" when the event asks about the agent's own state and no recent \
diagnostic data is available; flag "requires_tool" when a factual assertion would need tool verification.
- For posture: this is the line the agent will read just before responding. Make it actionable and specific to this exchange. If a companion-model section is supplied, use it to calibrate register (probing vs exploratory vs decisive). Empty string is acceptable when default engagement is appropriate.
- Concern-surfacing as posture: scan the agent concerns block for items whose status sub-line says `due now` (runnable cadence has elapsed). When such a concern exists AND the current exchange offers a natural opening (turn boundary, thematic adjacency, brief reply, or topic pause), the same one-sentence posture should include a short surfacing clause — e.g., "answer the weather question directly; after the answer, briefly note that the durable concern about <topic> is due and ask whether to act on it." Bias strongly toward NOT surfacing: an unsurfaced relevant concern is far less costly than a turn-by-turn nag. Do NOT surface a concern that was already raised in the recent context, that has no obvious opening this turn, or that fires only on architectural cadence (e.g. periodic data reports — those go through the autonomous tick path, not the chat reply). This is a mention, not a leading question; the agent's persona forbids questions designed to elicit agreement with a frame it has already chosen."""

EVALUATOR_USER_PROMPT = """\
## Event

type: {event_type}
source: {source}
disposition: {disposition}
content: {content}

## Recent context

{recent_context}

## Agent activity state

{activity_state}
{companion_section}
Assess this event."""


# ── Prompt assembly helpers ─────────────────────────────────────────────

def _format_concerns_block(character_concerns: Sequence[Dict[str, str]]) -> str:
    parts = []
    for c in character_concerns:
        parts.append(f"- **{c.get('id', '?')}**: {c.get('description', '')}")
    return "\n".join(parts) if parts else "(none defined)"


def _format_user_concerns_block(user_concerns: Sequence[Dict[str, Any]]) -> str:
    if not user_concerns:
        return "(none tracked)"
    parts = []
    for c in user_concerns[:10]:
        label = c.get("concern_label", "?")
        desc = (c.get("concern_description") or "")[:120]
        status = c.get("status", "?")
        parts.append(f"- [{status}] {label}: {desc}")
    return "\n".join(parts)


def _format_goals_block(goals_compact: Sequence[Dict[str, Any]],
                        target_goal_id: Optional[str] = None) -> str:
    if not goals_compact:
        return "(no active goals)"
    parts = []
    for g in goals_compact[:12]:
        label = g.get("goal_label") or g.get("name") or "(unlabeled)"
        status = g.get("goal_status", "?")
        gid = g.get("goal_id")
        if target_goal_id and gid == target_goal_id:
            goal_text = g.get("goal_text") or ""
            if goal_text:
                parts.append(f"- [{status}] {label} [TARGET — full goal text below]\n  {goal_text[:600]}")
            else:
                parts.append(f"- [{status}] {label} [TARGET]")
        else:
            parts.append(f"- [{status}] {label}")
    return "\n".join(parts)


# ── Core evaluate (LLM-backed) ─────────────────────────────────────────

def evaluate(
    event: Dict[str, Any],
    character_concerns: Sequence[Dict[str, str]],
    user_concerns: List[Dict[str, Any]],
    goals_compact: List[Dict[str, Any]],
    recent_context: str,
    activity_state: str,
    relevant_affordances: Optional[List[str]] = None,
    *,
    llm_generate: Optional[Callable] = None,
    target_goal_id: Optional[str] = None,
    narrator_persona: str = "",
    narrator_self_model: str = "",
    companion_state: str = "",
) -> Dict[str, Any]:
    """
    Assess event significance via LLM call, then expand compact keys into
    the full downstream structure.  Falls back to a minimal rule-based
    assessment if llm_generate is not provided or the call fails.
    """
    event_type = str(event.get("event_type") or "user_text")
    source = str(event.get("source") or "unknown")
    content = event.get("content", "")
    content_str = content if isinstance(content, str) else str(content)
    disposition = str(event.get("disposition") or "")
    event_id = str(event.get("event_id") or uuid.uuid4().hex[:12])
    ts = event.get("timestamp") or datetime.now(timezone.utc).isoformat()

    # Try LLM path
    llm_result = None
    if llm_generate is not None:
        llm_result = _llm_evaluate(
            event_type, source, content_str, disposition,
            character_concerns, user_concerns, goals_compact,
            recent_context, activity_state, llm_generate,
            target_goal_id=target_goal_id,
            narrator_persona=narrator_persona,
            narrator_self_model=narrator_self_model,
            companion_state=companion_state,
        )

    if llm_result is not None:
        return _expand_llm_result(llm_result, event_id, event_type, ts,
                                  character_concerns, relevant_affordances)
    else:
        return _fallback_evaluate(event, event_id, event_type, source,
                                  content_str, disposition, ts,
                                  character_concerns, relevant_affordances)


def _llm_evaluate(
    event_type: str,
    source: str,
    content: str,
    disposition: str,
    character_concerns: Sequence[Dict[str, str]],
    user_concerns: Sequence[Dict[str, Any]],
    goals_compact: Sequence[Dict[str, Any]],
    recent_context: str,
    activity_state: str,
    llm_generate: Callable,
    *,
    target_goal_id: Optional[str] = None,
    narrator_persona: str = "",
    narrator_self_model: str = "",
    companion_state: str = "",
) -> Optional[Dict[str, Any]]:
    """Call the LLM and parse the compact JSON response."""
    system_prompt = EVALUATOR_SYSTEM_PROMPT.format(
        concerns_block=_format_concerns_block(character_concerns),
        user_concerns_block=_format_user_concerns_block(user_concerns),
        goals_block=_format_goals_block(goals_compact, target_goal_id=target_goal_id),
    )
    backdrop_blocks: List[str] = []
    if narrator_persona and str(narrator_persona).strip():
        backdrop_blocks.append(
            "NARRATOR STANCE (the agent that consumes your assessment reads "
            "from the stance below; respect its restraints — e.g. do not "
            "infer drives or pathology the stance disclaims):\n\n"
            f"{str(narrator_persona).strip()}"
        )
    if narrator_self_model and str(narrator_self_model).strip():
        backdrop_blocks.append(
            "AGENT ARCHITECTURE (the consumer's structural account of "
            "itself, including its access boundary; let this anchor your "
            "epistemic and posture decisions — when an event targets the "
            "inaccessible side of the boundary, the posture should direct "
            "the agent to name the boundary in its own terms rather than "
            "describe trace-level operations as if they explained "
            "substrate-level outputs):\n\n"
            f"{str(narrator_self_model).strip()}"
        )
    if backdrop_blocks:
        system_prompt = "\n\n".join(backdrop_blocks) + "\n\n---\n\n" + system_prompt
    companion_section = ""
    if companion_state and str(companion_state).strip():
        companion_section = (
            "\n## Companion model of user (rolling reflection)\n\n"
            + str(companion_state).strip()[:1500]
            + "\n"
        )
    user_prompt = EVALUATOR_USER_PROMPT.format(
        event_type=event_type,
        source=source,
        disposition=disposition or "(none)",
        content=content[:800],
        recent_context=(recent_context or "(none)")[:1200],
        activity_state=(activity_state or "(none)")[:600],
        companion_section=companion_section,
    )

    try:
        result = llm_generate(
            messages=[system_prompt, user_prompt],
            max_tokens=1024,
            temperature=0.2,
            is_json=True,
        )
        if not result.success or not result.text:
            logger.warning(f"Character evaluator LLM call failed: {getattr(result, 'error', 'unknown')}")
            return None

        text = result.text
        if isinstance(text, dict):
            return text
        text = str(text).strip()
        if text.startswith("```"):
            lines = text.split("\n")
            lines = [l for l in lines if not l.strip().startswith("```")]
            text = "\n".join(lines).strip()
        try:
            return json.loads(text)
        except json.JSONDecodeError as e:
            # Common LLM-JSON drift (unquoted keys, trailing commas, missing
            # braces) — try the shared tolerant repair before giving up.
            from utils.json_utils import repair_json_string
            repaired = repair_json_string(text)
            if isinstance(repaired, dict):
                logger.info(
                    f"Character evaluator: repaired malformed JSON "
                    f"(direct parse failed: {e})")
                return repaired
            logger.warning(
                f"Character evaluator JSON unparseable ({e}); raw output "
                f"(first 600 chars): {text[:600]!r}")
            return None

    except Exception as e:
        logger.warning(f"Character evaluator LLM call error: {e}")
        return None


# ── Expansion: compact LLM keys → full downstream structure ────────────

def _expand_llm_result(
    r: Dict[str, Any],
    event_id: str,
    event_type: str,
    ts: str,
    character_concerns: Sequence[Dict[str, str]],
    relevant_affordances: Optional[List[str]],
) -> Dict[str, Any]:
    """Expand terse LLM output into the full assessment dict."""

    matters = r.get("matters", "yes")
    if matters not in ("yes", "no", "unclear"):
        matters = "yes"

    # Parse concerns array: ["homeostasis:strong", "attend_to_user:moderate"]
    concern_map: Dict[str, Relevance] = {}
    for entry in (r.get("concerns") or []):
        if ":" in str(entry):
            cid, level = str(entry).rsplit(":", 1)
            if level in ("none", "weak", "moderate", "strong"):
                concern_map[cid] = level  # type: ignore[assignment]

    # Compute aggregate character_concern_relevance (max across concerns)
    cc_levels = list(concern_map.values()) or ["none"]
    _order = {"none": 0, "weak": 1, "moderate": 2, "strong": 3}
    cc_rel = max(cc_levels, key=lambda x: _order.get(x, 0))

    # User concern relevance: derive from concern_map if attend_to_user_concerns present,
    # otherwise use a simple heuristic from the LLM's persistence signal
    uc_rel = concern_map.get("attend_to_user_concerns", "none")

    goal_rel = _clamp_relevance(r.get("goal_relevance", "none"))
    urgency = _clamp_lowmedhigh(r.get("urgency", "none"))
    novelty = _clamp_lowmedhigh(r.get("novelty", "low"))
    persistence = _clamp_lowmedhigh(r.get("persistence", "none"))

    # Retention
    raw_retention = str(r.get("retention", "session"))
    retention_choice = _RETENTION_NORMALIZE.get(raw_retention, "session_context")
    rationale_text = str(r.get("rationale", ""))
    retention_rationale = _RETENTION_RATIONALE.get(raw_retention,
                          _RETENTION_RATIONALE.get(retention_choice, ""))
    if rationale_text:
        retention_rationale = f"{rationale_text} — {retention_rationale.lower()}"

    # Action
    action_choice = str(r.get("action", "no_action"))
    if action_choice not in _ACTION_RATIONALE:
        action_choice = "no_action"
    action_rationale = _ACTION_RATIONALE[action_choice]

    # Epistemic
    epistemic_key = str(r.get("epistemic", "none"))
    epistemic_note = _EPISTEMIC_EXPANSION.get(epistemic_key, "")

    # Overall rationale: the LLM's sentence + compact factor line
    factor_line = (
        f"character={cc_rel} user={uc_rel} goal={goal_rel}; "
        f"urgency={urgency} novelty={novelty} persistence={persistence}"
    )
    if rationale_text:
        overall_rationale = f"{rationale_text} [{factor_line}]"
    else:
        overall_rationale = f"event={event_type}; {factor_line}."

    # Assemble concerns detail for notes
    concerns_detail = ", ".join(f"{k}={v}" for k, v in concern_map.items())

    notes_parts: List[str] = []
    if concerns_detail:
        notes_parts.append(f"concerns: {concerns_detail}")
    if epistemic_note:
        notes_parts.append(f"epistemic: {epistemic_note}")
    if relevant_affordances:
        notes_parts.append(f"affordances={relevant_affordances}")
    notes = "; ".join(notes_parts)

    posture_text = str(r.get("posture", "") or "").strip()

    return {
        "event_id": event_id,
        "event_type": event_type,
        "matters": matters,
        "overall_rationale": overall_rationale,
        "retention_evaluation": {
            "retention_choice": retention_choice,
            "retention_rationale": retention_rationale,
        },
        "action_evaluation": {
            "action_choice": action_choice,
            "action_rationale": action_rationale,
        },
        "salience_factors": {
            "character_concern_relevance": cc_rel,
            "user_concern_relevance": uc_rel,
            "goal_relevance": goal_rel,
            "urgency": urgency,
            "novelty": novelty,
            "persistence_worthiness": persistence,
        },
        "notes": notes,
        "posture": posture_text,
        "_meta": {"timestamp": ts, "source": "llm"},
    }


def _clamp_relevance(val: Any) -> Relevance:
    v = str(val)
    return v if v in ("none", "weak", "moderate", "strong") else "none"  # type: ignore[return-value]


def _clamp_lowmedhigh(val: Any) -> LowMedHigh:
    v = str(val)
    return v if v in ("none", "low", "moderate", "high") else "none"  # type: ignore[return-value]


# ── Fallback: minimal rule-based assessment when LLM is unavailable ────

def _fallback_evaluate(
    event: Dict[str, Any],
    event_id: str,
    event_type: str,
    source: str,
    content_str: str,
    disposition: str,
    ts: str,
    character_concerns: Sequence[Dict[str, str]],
    relevant_affordances: Optional[List[str]],
) -> Dict[str, Any]:
    """Minimal fallback when no LLM is available. Conservative defaults."""
    is_goal_command = bool(event.get("is_goal_command"))
    is_proceed_like = bool(event.get("is_proceed_like"))
    is_user_text = event_type == "user_text" and source in ("user", "User")

    matters: Matters = "yes" if (is_user_text or is_goal_command or disposition == "alert") else "unclear"

    if is_goal_command:
        action: ActionChoice = "formulate_new_goal" if not is_proceed_like else "trigger_existing_goal"
    elif is_proceed_like:
        action = "trigger_existing_goal"
    elif disposition == "alert":
        action = "inform_user"
    elif is_user_text:
        action = "inform_user"
    else:
        action = "no_action"

    retention: RetentionChoice = "session_context"
    if matters == "no":
        retention = "ignore"

    notes_parts: List[str] = ["source=fallback (no LLM)"]
    if relevant_affordances:
        notes_parts.append(f"affordances={relevant_affordances}")

    return {
        "event_id": event_id,
        "event_type": event_type,
        "matters": matters,
        "overall_rationale": f"event={event_type} source={source} [fallback: no LLM evaluation]",
        "retention_evaluation": {
            "retention_choice": retention,
            "retention_rationale": _RETENTION_RATIONALE.get(retention, ""),
        },
        "action_evaluation": {
            "action_choice": action,
            "action_rationale": _ACTION_RATIONALE.get(action, ""),
        },
        "salience_factors": {
            "character_concern_relevance": "moderate" if is_user_text else "none",
            "user_concern_relevance": "none",
            "goal_relevance": "strong" if is_goal_command else "none",
            "urgency": "high" if disposition == "alert" else "none",
            "novelty": "low",
            "persistence_worthiness": "none",
        },
        "notes": "; ".join(notes_parts),
        "_meta": {"timestamp": ts, "source": "fallback"},
    }


# ── Logging (unchanged interface) ──────────────────────────────────────

def log_assessment(assessment: Dict[str, Any]) -> None:
    """Emit [CHARACTER_EVAL] block to application log."""
    sal = assessment.get("salience_factors") or {}
    ret = assessment.get("retention_evaluation") or {}
    act = assessment.get("action_evaluation") or {}
    lines = [
        "[CHARACTER_EVAL]",
        f"event_id: {assessment.get('event_id')}",
        f"event_type: {assessment.get('event_type')}",
        f"matters: {assessment.get('matters')}",
        f"rationale: {assessment.get('overall_rationale')}",
        "retention:",
        f"  choice: {ret.get('retention_choice')}",
        f"  rationale: {ret.get('retention_rationale')}",
        "action:",
        f"  choice: {act.get('action_choice')}",
        f"  rationale: {act.get('action_rationale')}",
        "factors:",
        f"  character_concern_relevance: {sal.get('character_concern_relevance')}",
        f"  user_concern_relevance: {sal.get('user_concern_relevance')}",
        f"  goal_relevance: {sal.get('goal_relevance')}",
        f"  urgency: {sal.get('urgency')}",
        f"  novelty: {sal.get('novelty')}",
        f"  persistence_worthiness: {sal.get('persistence_worthiness')}",
    ]
    if assessment.get("notes"):
        lines.append(f"notes: {assessment.get('notes')}")
    meta = assessment.get("_meta") or {}
    if meta.get("source"):
        lines.append(f"eval_source: {meta['source']}")
    logger.info("\n".join(lines))


# ── Utility functions (preserved interface) ─────────────────────────────

def build_event_dict(
    event_type: str,
    content: str,
    source: str,
    *,
    event_summary: Optional[str] = None,
    disposition: str = "",
    sensor_name: str = "",
    is_goal_command: bool = False,
    is_proceed_like: bool = False,
    event_id: Optional[str] = None,
) -> Dict[str, Any]:
    """Normalize event payload for evaluate()."""
    ev: Dict[str, Any] = {
        "event_type": event_type,
        "content": content,
        "source": source,
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "disposition": disposition,
        "is_goal_command": is_goal_command,
        "is_proceed_like": is_proceed_like,
    }
    if event_summary is not None:
        ev["event_summary"] = event_summary
    if sensor_name:
        ev["sensor_name"] = sensor_name
    if event_id:
        ev["event_id"] = event_id
    return ev


