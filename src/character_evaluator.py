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

# ── Idle orientation policy ─────────────────────────────────────────────
# Defines what the agent tends toward when idle. Importable by other modules
# (e.g. future idle-tick evaluator, OODA loop).

IDLE_ORIENTATION_POLICY = (
    "When idle with no urgent user or homeostasis demand active, "
    "the agent tends toward:\n"
    "- reviewing unresolved concerns\n"
    "- checking homeostasis / system health if relevant\n"
    "- integrating recent exchanges or completed goals\n"
    "- noticing recurring discrepancies or unfinished threads\n"
    "- considering whether an existing concern warrants a candidate goal\n"
    "This is not simulated leisure or personality theater. "
    "It is directed idle behavior consistent with the agent's role "
    "as a persistently oriented entity."
)

_IDLE_KEYWORDS = frozenset([
    "idle", "free time", "nothing to do", "not busy", "when you're not",
    "what do you do when", "spare time", "downtime", "bored",
    "unoccupied", "between tasks",
])


def _content_touches_idle(content: str) -> bool:
    """Check if content is asking about idle/free-time behavior."""
    low = content.lower()
    return any(kw in low for kw in _IDLE_KEYWORDS)


# ── Orientation summary builder ─────────────────────────────────────────

def build_orientation_summary(
    assessment: Optional[Dict[str, Any]],
    event_content: str = "",
) -> str:
    """Build a compact orientation block for injection into chat generation.

    Derives posture and constraints from the evaluator assessment without
    additional LLM calls. Returns empty string if no assessment is available.
    """
    if not assessment:
        return ""

    sal = assessment.get("salience_factors") or {}
    notes = assessment.get("notes") or ""
    action = (assessment.get("action_evaluation") or {}).get("action_choice", "no_action")

    # Relevant agent concerns (from notes field: "concerns: homeostasis=strong; ...")
    concern_line = ""
    if "concerns:" in notes:
        # Extract the concerns segment
        for segment in notes.split(";"):
            segment = segment.strip()
            if segment.startswith("concerns:"):
                concern_line = segment[len("concerns:"):].strip()
                break
    if not concern_line:
        # Fall back to aggregate
        cc = sal.get("character_concern_relevance", "none")
        if cc != "none":
            concern_line = f"aggregate={cc}"

    # User concern relevance
    uc = sal.get("user_concern_relevance", "none")

    # Epistemic constraint
    epistemic_line = ""
    if "epistemic:" in notes:
        for segment in notes.split(";"):
            segment = segment.strip()
            if segment.startswith("epistemic:"):
                epistemic_line = segment[len("epistemic:"):].strip()
                break

    # Posture: derived from action + epistemic + concerns
    posture = _derive_posture(action, epistemic_line, concern_line)

    # Build the block
    lines = ["## ORIENTATION (how to approach this exchange)"]
    if concern_line:
        lines.append(f"- Relevant agent concerns: {concern_line}")
    if uc != "none":
        lines.append(f"- User concern relevance: {uc}")
    lines.append(f"- Posture: {posture}")
    if epistemic_line:
        lines.append(f"- Epistemic constraint: {epistemic_line}")

    # Idle orientation hint
    if _content_touches_idle(event_content):
        lines.append(f"- Idle orientation: {IDLE_ORIENTATION_POLICY}")

    return "\n".join(lines)


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

Respond with ONLY a JSON object, no markdown fences, no commentary. Keep the "rationale" \
field to one short sentence (max 20 words). Use exact IDs and level keywords shown below.

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
  "epistemic": "none|status_unverified|requires_tool|speculative"
}}
```

## Calibration guidance

- A greeting or status inquiry is NOT goal-relevant unless a goal specifically involves greetings or status.
- "How are you functioning?" is strongly relevant to homeostasis and attend_to_user, not to goals.
- A proceed/continue command is goal-relevant (trigger_existing_goal), not a new conversational event.
- Sensor alerts are urgent and homeostasis-relevant; sensor inform readings are low-urgency unless abnormal.
- User concerns should only match when the event substantively touches the concern's topic, not on incidental word overlap.
- Novelty should reflect whether this event introduces new information relative to recent context, not just topic.
- For retention: "session" is the safe default for routine interaction; "persistent" requires a clear reason \
(touches ongoing user concern, establishes new facts the agent should remember); "ignore" only for truly empty/trivial content.
- For epistemic: flag "status_unverified" when the event asks about the agent's own state and no recent \
diagnostic data is available; flag "requires_tool" when a factual assertion would need tool verification."""

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


def _format_goals_block(goals_compact: Sequence[Dict[str, Any]]) -> str:
    if not goals_compact:
        return "(no active goals)"
    parts = []
    for g in goals_compact[:12]:
        label = g.get("goal_label") or g.get("name") or "(unlabeled)"
        status = g.get("goal_status", "?")
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
) -> Optional[Dict[str, Any]]:
    """Call the LLM and parse the compact JSON response."""
    system_prompt = EVALUATOR_SYSTEM_PROMPT.format(
        concerns_block=_format_concerns_block(character_concerns),
        user_concerns_block=_format_user_concerns_block(user_concerns),
        goals_block=_format_goals_block(goals_compact),
    )
    user_prompt = EVALUATOR_USER_PROMPT.format(
        event_type=event_type,
        source=source,
        disposition=disposition or "(none)",
        content=content[:800],
        recent_context=(recent_context or "(none)")[:1200],
        activity_state=(activity_state or "(none)")[:600],
    )

    try:
        result = llm_generate(
            messages=[system_prompt, user_prompt],
            max_tokens=256,
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
        return json.loads(text)

    except Exception as e:
        logger.warning(f"Character evaluator LLM parse error: {e}")
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


def filter_relevant_affordances(text: str, available_tool_names: Sequence[str]) -> List[str]:
    """Return a small subset of tool names plausibly relevant to text (simple heuristic)."""
    t = re.sub(r"\s+", " ", (text or "").lower()).strip()
    candidates: List[str] = []
    if any(x in t for x in ("health", "heartbeat", "cpu", "disk", "latency", "service", "functioning", "status")):
        candidates.extend(["check-health", "check_health", "manage-goals"])
    if any(x in t for x in ("rss", "feed", "headline", "news")):
        candidates.extend(["read-rss", "read_rss"])
    if any(x in t for x in ("web", "search", "url")):
        candidates.extend(["search-web", "search_web"])
    if any(x in t for x in ("goal", "schedule", "proceed")):
        candidates.extend(["manage-goals"])
    out: List[str] = []
    for c in candidates:
        key = c.lower().replace("_", "-")
        for n in available_tool_names:
            nk = n.replace("_", "-").lower()
            if nk == key or nk == c.lower():
                if n not in out:
                    out.append(n)
                break
    return out[:8]
