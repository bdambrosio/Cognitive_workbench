"""
OODA Planner — strategic-level incremental planner for the cognitive agent.

Phase 1: Communication ownership.
  The OODA planner owns all user-facing communication. The goal executor
  (infospace_executor) calls into this module rather than publishing
  directly to Zenoh. Default policy: passthrough (forward say/ask to user
  as-is).

Phase 2: Action schemas, context assembly, event-action history.
  Defines the OODA planner's action vocabulary (9 actions), builds the
  context prefix from live state, and maintains a rolling history of
  event-action cycles with progressive rollup.

Phase 3+: Continuous planning loop, sleep/consolidation.
"""

import json
import logging
import queue
import time
from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Dict, List, Optional

logger = logging.getLogger(__name__)

_ASK_TIMEOUT_SECONDS = 300  # 5 minutes

# ── Action Schemas ──────────────────────────────────────────────────────────
# Each schema defines an action the OODA planner can generate.
# These are rendered into the system prompt as the planner's "skill set."

OODA_ACTION_SCHEMAS = {
    "submit-goal": {
        "description": "Submit a bounded goal for the goal executor to plan and execute.",
        "fields": {
            "goal_text": "str — concrete, actionable goal description",
            "expected_outcome": "str — what success looks like",
            "concern_id": "str | null — which concern this serves",
            "priority": "str — 'normal' or 'high'",
        },
    },
    "update-concern": {
        "description": "Modify a concern's state: adjust weight, change status, or add notes.",
        "fields": {
            "concern_id": "str — which concern to update",
            "updates": "dict — fields to change (weight, status, notes, etc.)",
        },
    },
    "update-user-model": {
        "description": "Record an observation about the user's current state, interests, or needs.",
        "fields": {
            "observation": "str — what you observed about the user",
        },
    },
    "configure-sensor": {
        "description": "Adjust sensor parameters, enable/disable a sensor, or change its disposition.",
        "fields": {
            "sensor_name": "str — which sensor to configure",
            "changes": "dict — parameter changes (schedule, disposition, enabled, etc.)",
        },
    },
    "say": {
        "description": "Send a message to the user. Use for proactive engagement, status updates, or remarks.",
        "fields": {
            "text": "str — message content",
        },
    },
    "ask": {
        "description": "Pose a question to the user. Use when you need information for a strategic decision.",
        "fields": {
            "text": "str — the question",
        },
    },
    "wait": {
        "description": "Explicitly decide to do nothing. Record why inaction is the right choice.",
        "fields": {
            "reason": "str — why waiting is appropriate right now",
        },
    },
    "reflect": {
        "description": "Write a strategic observation to your own context. Not an external action — a way to reason on paper before committing.",
        "fields": {
            "observation": "str — your strategic thought",
        },
    },
    "sleep": {
        "description": "Initiate a context consolidation cycle. Use when attention has drifted across too many concerns or context is growing large.",
        "fields": {
            "consolidation_note": "str — brief summary of where things stand for your future self",
        },
    },
}


def render_action_schemas_for_prompt() -> str:
    """Render action schemas as a compact block for the system prompt."""
    lines = []
    for name, schema in OODA_ACTION_SCHEMAS.items():
        fields_str = ", ".join(
            f'"{k}": {v}' for k, v in schema["fields"].items()
        )
        lines.append(f'- **{name}**: {schema["description"]}')
        lines.append(f'  Format: {{"action": "{name}", {fields_str}}}')
    return "\n".join(lines)


# ── System Prompt ───────────────────────────────────────────────────────────

OODA_SYSTEM_PROMPT = """\
You are the strategic planner for a cognitive agent. Each cycle you \
receive the current situation and choose ONE action. You do not execute \
goals — you submit them to a separate goal executor.

## Constraint

You cannot use tools directly. Your only way to make things happen in \
the world is submit-goal. Everything else you do is internal (update \
concerns, reflect) or communicative (say, ask).

## Actions

""" + render_action_schemas_for_prompt() + """

## How to Think Each Cycle

1. Read the current event and its assessment.
2. Check the concern landscape — what is active, rising, or recently changed?
3. Check the goal field — is a goal running? Did one just complete?
4. Choose ONE action based on what matters most right now.

## Key Patterns

**Wait is the default.** Only act when you have a clear reason. A rising \
concern will still be there next cycle. Premature action wastes the \
executor's bandwidth.

**Do NOT narrate your internal state to the user.** Concern activations, \
triage nominations, and periodic ticks are internal signals. The user \
does not want to hear "I'm monitoring health" or "a concern was \
activated." Use say only when you have substantive new information for \
them — a goal completed with a useful result, a user-facing situation \
changed, or the user asked a question that a completed goal now \
answers. When in doubt about saying something internal, wait.

**Write good goals.** A goal must be specific enough for an executor that \
cannot see your reasoning. Include:
- What to do (concrete actions; specific sources when the goal needs input)
- How to do it well (user preferences, known tool behaviors, quality expectations)
- What to produce (a Note, a summary, a comparison)
- What to do with the result (share with user, update a collection)

BAD: "Look into the user's weather concern"
GOOD: "Search for Berkeley CA 3-day weather forecast using search-web. \
Create a Note with temperature ranges and precipitation probability. \
The user prefers concise bullet-point summaries."

**Results are auto-shared.** When a goal completes successfully, the \
system automatically tells the user the result. You do not need to \
say the result yourself. After a goal completes, your job is to \
update-concern (mark satisfied if the result was adequate, or plan a \
follow-up if it wasn't).

**Manage concerns actively.** After a successful goal, mark the \
motivating concern as satisfied. If a goal failed or only partially \
addressed the concern, decide whether to retry, adjust the approach, \
or escalate to the user via ask.

**Do not repeat yourself.** Check event-action history before acting. \
If you already submitted a goal for a concern and it is still running, \
wait. If you already said something to the user, do not say it again.

## Output

Respond with a single JSON object. No markdown, no commentary, no \
explanation — just the action.
"""


# ── Event-Action History ────────────────────────────────────────────────────

@dataclass
class OodaCycle:
    """One cycle of the OODA planner: event -> action -> result."""
    timestamp: str
    event_summary: str          # what happened (type, source, content preview)
    assessment_summary: str     # character_evaluator result, one line
    action: Dict[str, Any]      # the JSON action chosen
    result_summary: str         # what happened after (goal outcome, concern updated, etc.)
    concern_ids: List[str]      # which concerns were involved
    content_summary: str = ""   # substantive content: what was this ABOUT?


class OodaHistory:
    """Rolling history of OODA cycles with progressive rollup.

    Recent cycles are kept in full. Older cycles are compressed to
    one-line summaries. Oldest cycles are aggregated by concern.

    The history is rendered as part of the "user prompt" (the evolving
    context that grows each cycle).
    """

    def __init__(self, full_window: int = 3, summary_window: int = 15):
        self._cycles: List[OodaCycle] = []
        self._full_window = full_window          # last N cycles kept in full
        self._summary_window = summary_window    # next N kept as summaries
        # Older than full+summary are aggregated by concern

    def append(self, cycle: OodaCycle) -> None:
        self._cycles.append(cycle)

    @property
    def cycle_count(self) -> int:
        return len(self._cycles)

    def render(self) -> str:
        """Render the history for the LLM context.

        Returns a text block with three sections:
        1. Full expansion of recent cycles
        2. One-line summaries of older cycles
        3. Concern-level aggregation of oldest cycles
        """
        if not self._cycles:
            return "(no prior cycles)"

        total = len(self._cycles)
        full_start = max(0, total - self._full_window)
        summary_start = max(0, full_start - self._summary_window)

        parts = []

        # Aggregated (oldest)
        if summary_start > 0:
            agg = self._aggregate(self._cycles[:summary_start])
            if agg:
                parts.append("### Aggregated History")
                parts.append(agg)

        # Summaries (middle)
        if summary_start < full_start:
            parts.append("### Recent Summary")
            for c in self._cycles[summary_start:full_start]:
                parts.append(self._summarize_cycle(c))

        # Full expansion (most recent)
        if full_start < total:
            parts.append("### Current Cycles")
            for c in self._cycles[full_start:total]:
                parts.append(self._expand_cycle(c))

        return "\n\n".join(parts)

    def _expand_cycle(self, c: OodaCycle) -> str:
        """Full expansion of a single cycle."""
        action_str = json.dumps(c.action, indent=None)
        lines = [
            f"**[{c.timestamp}]** Event: {c.event_summary}",
            f"  Assessment: {c.assessment_summary}",
            f"  Action: {action_str}",
            f"  Result: {c.result_summary}",
        ]
        if c.content_summary:
            lines.append(f"  Content: {c.content_summary}")
        if c.concern_ids:
            lines.append(f"  Concerns: {', '.join(c.concern_ids)}")
        return "\n".join(lines)

    def _summarize_cycle(self, c: OodaCycle) -> str:
        """One-line summary of a cycle."""
        action_name = c.action.get("action", "?")
        content = c.content_summary or c.event_summary
        concerns = ", ".join(c.concern_ids) if c.concern_ids else "none"
        return f"- [{c.timestamp}] {action_name}: {content[:100]} (concerns: {concerns})"

    def _aggregate(self, cycles: List[OodaCycle]) -> str:
        """Aggregate oldest cycles by concern with content summaries."""
        if not cycles:
            return ""
        # Group by concern
        by_concern: Dict[str, List[OodaCycle]] = {}
        unconcerned: List[OodaCycle] = []
        for c in cycles:
            if c.concern_ids:
                for cid in c.concern_ids:
                    by_concern.setdefault(cid, []).append(c)
            else:
                unconcerned.append(c)

        lines = []
        for cid, group in sorted(by_concern.items()):
            actions = {}
            content_bits = []
            for c in group:
                a = c.action.get("action", "?")
                actions[a] = actions.get(a, 0) + 1
                if c.content_summary:
                    content_bits.append(c.content_summary[:80])
            action_counts = ", ".join(f"{a}x{n}" for a, n in actions.items())
            content_preview = "; ".join(content_bits[:3])
            if len(content_bits) > 3:
                content_preview += f" (+{len(content_bits)-3} more)"
            lines.append(
                f"- Concern {cid}: {len(group)} cycles ({action_counts}). "
                f"Content: {content_preview or 'routine'}"
            )
        if unconcerned:
            lines.append(
                f"- Unconcerned: {len(unconcerned)} cycles "
                f"({', '.join(c.action.get('action', '?') for c in unconcerned[:5])})"
            )
        return "\n".join(lines)


# ── Context Assembly ────────────────────────────────────────────────────────

def assemble_ooda_context(
    *,
    character_desc: str,
    capabilities_desc: str,
    setting_desc: str,
    concern_landscape: str,
    goal_field: str,
    sensor_state: str,
    cognitive_graph_context: str,
    history: OodaHistory,
    current_event: str = "",
) -> str:
    """Build the "user prompt" for the OODA planner LLM call.

    This is the evolving context that changes each cycle. Combined with
    OODA_SYSTEM_PROMPT (fixed), it forms the full prompt.

    Args:
        character_desc: who this agent is
        capabilities_desc: what this agent can do
        setting_desc: the world this agent operates in
        concern_landscape: formatted concern state (user + derived + activations)
        goal_field: running goal, queued goals, recent outcomes
        sensor_state: active sensors and recent readings
        cognitive_graph_context: rendered relevant history from cognitive graph
        history: OodaHistory instance with prior cycles
        current_event: the event being processed this cycle (if any)
    """
    sections = []

    sections.append("## Identity\n" + character_desc.strip())
    sections.append("## Capabilities\n" + capabilities_desc.strip())
    sections.append("## Setting\n" + setting_desc.strip())
    sections.append("## Concern Landscape\n" + concern_landscape)
    sections.append("## Goal Field\n" + goal_field)

    if sensor_state:
        sections.append("## Sensor State\n" + sensor_state)

    if cognitive_graph_context:
        sections.append("## Relevant History (from cognitive graph)\n"
                        + cognitive_graph_context)

    sections.append("## Event-Action History\n" + history.render())

    if current_event:
        sections.append("## Current Event\n" + current_event)

    sections.append(
        "## Your Turn\n"
        "Based on the current event and context above, choose ONE action. "
        "Respond with a single JSON object."
    )

    return "\n\n".join(sections)


def build_concern_landscape(
    user_concerns: List[Dict],
    derived_concerns: List[Dict],
    activations: Dict[str, float],
    activation_trends: Dict[str, str],
) -> str:
    """Format the concern landscape for the OODA context.

    Combines user concerns, derived concerns, and activation data
    into a readable block.
    """
    lines = []

    if user_concerns:
        lines.append("**User Concerns:**")
        for c in user_concerns:
            label = c.get("concern_label", "?")
            desc = c.get("concern_description", "")
            status = c.get("status", "open")
            weight = c.get("weight", 0)
            lines.append(f"- {label} [{status}, weight={weight:.1f}]: {desc[:120]}")

    if derived_concerns:
        lines.append("\n**Derived Concerns:**")
        for c in derived_concerns:
            cid = c.get("concern_id", "?")
            label = c.get("concern_label", "?")
            status = c.get("status", "?")
            desc = c.get("concern_description", "")
            act = activations.get(cid, 0.0)
            trend = activation_trends.get(cid, "stable")
            lines.append(
                f"- {label} [{status}, activation={act:.2f} {trend}]: {desc[:120]}"
            )

    if not user_concerns and not derived_concerns:
        lines.append("No active concerns.")

    return "\n".join(lines)


def build_goal_field(
    running_goal: Optional[Dict] = None,
    recent_outcomes: Optional[List[Dict]] = None,
) -> str:
    """Format the goal field for the OODA context."""
    lines = []
    if running_goal:
        lines.append(
            f"**Running:** {running_goal.get('goal_text', '?')[:150]} "
            f"(id={running_goal.get('goal_id', '?')})"
        )
    else:
        lines.append("**Running:** none")

    if recent_outcomes:
        lines.append("**Recent outcomes:**")
        for o in recent_outcomes[-3:]:
            status = o.get("status", "?")
            text = o.get("goal_text", "?")[:100]
            product = o.get("primary_product", "")
            lines.append(f"- [{status}] {text}")
            if product:
                lines.append(f"  Product: {product}")
    return "\n".join(lines)


def build_sensor_state(
    sensor_configs: List[Dict],
    recent_readings: Optional[List[Dict]] = None,
) -> str:
    """Format sensor state for the OODA context."""
    lines = []
    if sensor_configs:
        lines.append("**Active sensors:**")
        for s in sensor_configs:
            name = s.get("name", "?")
            sched = s.get("schedule", "?")
            disp = s.get("disposition", "?")
            lines.append(f"- {name} (every {sched}, disposition={disp})")

    if recent_readings:
        lines.append("\n**Recent readings:**")
        for r in recent_readings[-5:]:
            src = r.get("sensor_name", "?")
            content = r.get("content", "")[:120]
            lines.append(f"- [{src}] {content}")

    return "\n".join(lines) if lines else ""


class OodaPlanner:
    """Strategic planner that owns user-facing communication.

    In Phase 1, this is a thin mediation layer: the goal executor calls
    ``handle_goal_say`` / ``handle_goal_ask`` instead of publishing directly,
    and the default policy is passthrough.  The main loop routes user
    replies to ``route_ask_reply`` when ``awaiting_ask_response`` is set.
    """

    def __init__(
        self,
        agent_name: str,
        session,                    # Zenoh session
        action_publisher,           # Zenoh publisher for action channel
        conversation_store,         # ConversationStore instance
        executive_node,             # reference for action_counter, dialog state
    ):
        self.agent_name = agent_name
        self.session = session
        self.action_publisher = action_publisher
        self.conversation_store = conversation_store
        self.executive_node = executive_node

        # Ask state — mirrors what was on executive_node, now owned here
        self.awaiting_ask_response = False
        self._ask_response_queue: queue.Queue = queue.Queue()

        # Phase 2: event-action history (continuous across ticks)
        self.history = OodaHistory()

        # Phase 3: accumulated user prompt (appended each cycle)
        self._accumulated_context: Optional[str] = None

        # Pending goal completion events (set by executive_node on goal done)
        self._pending_goal_result: Optional[Dict[str, Any]] = None

    # ── Strategic planning step (Phase 3) ───────────────────────────────

    def step(
        self,
        event_summary: str,
        assessment_summary: str,
        assessment: Optional[Dict[str, Any]] = None,
    ) -> Optional[Dict[str, Any]]:
        """Run one OODA planning cycle: assemble context, call LLM, dispatch action.

        Called from the main loop for strategic events (not chat/alert).
        Returns the dispatched action result, or None on failure.

        Args:
            event_summary: compact description of the triggering event
            assessment_summary: one-line character_evaluator result
            assessment: full assessment dict (for concern_ids extraction)
        """
        en = self.executive_node

        # ── Assemble context from live state ────────────────────────────
        char_config = en.character_config

        # Concern landscape
        user_concerns = []
        try:
            user_concerns = en.user_concern_model.get_concerns(active_only=True) or []
        except Exception:
            pass

        derived_concerns = []
        try:
            dc = getattr(en, '_derived_concern_model', None)
            derived_concerns = dc.get_concerns() if dc else []
        except Exception:
            pass

        activations = dict(getattr(en, '_character_concern_activations', {}))
        trends = {}
        try:
            ls = getattr(en, '_ooda_living_state', None)
            if ls:
                for ca in getattr(ls, 'concern_activations', []):
                    cid = ca.get('id', '')
                    if cid:
                        trends[cid] = ca.get('trend', 'stable')
        except Exception:
            pass

        concern_landscape = build_concern_landscape(
            user_concerns, derived_concerns, activations, trends,
        )

        # Goal field
        running_goal = None
        recent_outcomes = []
        try:
            if en._is_goal_running() and en.current_goal:
                running_goal = {
                    'goal_text': en.current_goal.to_string(),
                    'goal_id': getattr(en, '_active_scheduled_goal_id', ''),
                }
            # Gather recent completed goals
            for g in en._all_scheduled_goals():
                if g.get('status') in ('completed', 'failed'):
                    recent_outcomes.append(g)
            recent_outcomes = recent_outcomes[-3:]
        except Exception:
            pass

        goal_field_str = build_goal_field(running_goal, recent_outcomes)

        # Sensor state
        sensor_state_str = build_sensor_state(
            getattr(en, 'sensor_configs', []),
            list(getattr(en, '_sensor_inform_queue', [])),
        )

        # Cognitive graph context
        graph_ctx = ""
        try:
            cg = getattr(en, '_cognitive_graph', None)
            if cg and event_summary:
                graph_ctx = cg.assemble_context(
                    event_summary,
                    concern_activations=activations,
                    k=10, max_hops=1,
                )
        except Exception as e:
            logger.debug(f"Cognitive graph context assembly failed: {e}")

        # Check for pending goal completion
        current_event = f"Event: {event_summary}\nAssessment: {assessment_summary}"
        if self._pending_goal_result:
            gr = self._pending_goal_result
            self._pending_goal_result = None
            current_event += (
                f"\n\nGoal completed: {gr.get('goal_text', '?')[:150]}"
                f"\n  Status: {gr.get('status', '?')}"
                f"\n  Product: {gr.get('primary_product', 'none')}"
                f"\n  Summary: {gr.get('result_summary', '')[:200]}"
            )
            if gr.get('concern_id'):
                current_event += f"\n  Concern: {gr['concern_id']}"

        # Build the full context
        context = assemble_ooda_context(
            character_desc=char_config.get('character', ''),
            capabilities_desc=char_config.get('capabilities', ''),
            setting_desc=char_config.get('setting', ''),
            concern_landscape=concern_landscape,
            goal_field=goal_field_str,
            sensor_state=sensor_state_str,
            cognitive_graph_context=graph_ctx,
            history=self.history,
            current_event=current_event,
        )

        # ── Context budget tracking ────────────────────────────────────
        sys_chars = len(OODA_SYSTEM_PROMPT)
        ctx_chars = len(context)
        total_chars = sys_chars + ctx_chars
        # Rough token estimate: ~3.5 chars per token for English
        est_tokens = total_chars // 3
        logger.info(
            f"OODA context: {total_chars} chars (~{est_tokens} tok) "
            f"[system={sys_chars} context={ctx_chars} "
            f"history={self.history.cycle_count} cycles]"
        )

        # ── LLM call ───────────────────────────────────────────────────
        try:
            result = en.llm_generate(
                messages=[
                    {"role": "system", "content": OODA_SYSTEM_PROMPT},
                    {"role": "user", "content": context},
                ],
                max_tokens=300,
                temperature=0.3,
                is_json=True,
            )
        except Exception as e:
            logger.error(f"OODA planner LLM call failed: {e}")
            return None

        if not getattr(result, 'success', False) or not getattr(result, 'text', ''):
            logger.warning(
                f"OODA planner LLM call unsuccessful: "
                f"{getattr(result, 'error', 'no text')}"
            )
            return None

        # ── Parse action ────────────────────────────────────────────────
        action = self._parse_action(result.text)
        if action is None:
            logger.warning(f"OODA planner: failed to parse action from: {result.text[:200]}")
            return None

        logger.info(f"OODA planner action: {json.dumps(action)}")

        # ── Dispatch ────────────────────────────────────────────────────
        dispatch_result = self.dispatch_action(action)

        # ── Record cycle ────────────────────────────────────────────────
        concern_ids = []
        if assessment:
            try:
                notes = assessment.get('notes', '')
                # Extract concern IDs from assessment notes
                if 'concerns:' in notes:
                    concern_part = notes.split('concerns:')[1].split(';')[0]
                    for token in concern_part.split(','):
                        token = token.strip()
                        if '=' in token:
                            concern_ids.append(token.split('=')[0].strip())
            except Exception:
                pass
        # Also include concern from the action itself
        action_concern = action.get('concern_id')
        if action_concern and action_concern not in concern_ids:
            concern_ids.append(action_concern)

        # Build content summary from action details
        content_summary = self._build_content_summary(action, dispatch_result)

        cycle = OodaCycle(
            timestamp=datetime.now().strftime("%H:%M:%S"),
            event_summary=event_summary[:150],
            assessment_summary=assessment_summary[:150],
            action=action,
            result_summary=dispatch_result.get('summary', '')[:200],
            concern_ids=concern_ids,
            content_summary=content_summary,
        )
        self.history.append(cycle)

        return dispatch_result

    def _parse_action(self, text) -> Optional[Dict[str, Any]]:
        """Parse LLM response as a JSON action object.

        text may be a str (raw LLM output) or an already-parsed dict/list
        (when the backend's is_json mode returns parsed JSON directly).
        """
        # Already parsed by backend
        if isinstance(text, dict):
            return text if 'action' in text else None
        if isinstance(text, list):
            # Take first dict with 'action' key
            for item in text:
                if isinstance(item, dict) and 'action' in item:
                    return item
            return None

        raw = str(text).strip()
        # Strip markdown fences if present
        if raw.startswith('```'):
            first_nl = raw.find('\n')
            if first_nl >= 0:
                raw = raw[first_nl + 1:]
            if raw.endswith('```'):
                raw = raw[:-3]
            raw = raw.strip()

        # Handle already-parsed JSON (some backends return parsed)
        if isinstance(raw, dict):
            return raw if 'action' in raw else None

        try:
            parsed = json.loads(raw)
            if isinstance(parsed, dict) and 'action' in parsed:
                return parsed
        except (json.JSONDecodeError, TypeError):
            pass

        # Try extracting first JSON object from response
        start = raw.find('{')
        end = raw.rfind('}')
        if start >= 0 and end > start:
            try:
                parsed = json.loads(raw[start:end + 1])
                if isinstance(parsed, dict) and 'action' in parsed:
                    return parsed
            except (json.JSONDecodeError, TypeError):
                pass

        return None

    def _build_content_summary(
        self, action: Dict, result: Dict,
    ) -> str:
        """Build a content-aware summary of what this cycle was ABOUT."""
        action_type = action.get('action', '')
        if action_type == 'submit-goal':
            return action.get('goal_text', '')[:120]
        elif action_type == 'say':
            return f"Said: {action.get('text', '')[:100]}"
        elif action_type == 'ask':
            reply = result.get('summary', '')
            return f"Asked: {action.get('text', '')[:60]} → {reply[:60]}"
        elif action_type == 'wait':
            return action.get('reason', '')[:120]
        elif action_type == 'reflect':
            return action.get('observation', '')[:120]
        elif action_type == 'update-concern':
            cid = action.get('concern_id', '?')
            return f"Updated concern {cid}: {json.dumps(action.get('updates', {}))[:80]}"
        else:
            return result.get('summary', '')[:120]

    def inject_goal_completion(self, goal_result: Dict[str, Any]) -> None:
        """Called by executive_node when a goal completes.

        The result is stored for the next OODA cycle's context so the
        planner can update concerns.

        For OODA-initiated goals (those with a concern_id), auto-shares
        the result with the user since they didn't see the executor's
        output directly. Chat-initiated goals ([GOAL_NEEDED] path) have
        no concern_id — results already reached the user via the
        executor, so we don't duplicate.
        """
        self._pending_goal_result = goal_result
        status = goal_result.get('status', '?')
        goal_text = goal_result.get('goal_text', '?')
        result_summary = goal_result.get('result_summary', '')
        concern_id = goal_result.get('concern_id', '')

        logger.info(
            f"OODA planner: goal completion — "
            f"{status}: {goal_text[:80]} "
            f"(concern={concern_id or 'none'})"
        )

        # Auto-share ONLY for autonomous OODA-initiated goals (concern_id set).
        # Chat-initiated goals already delivered their result via the executor.
        if concern_id and status in ('completed', 'success') and result_summary:
            share_text = result_summary[:500]
            if len(result_summary) > 500:
                share_text += "..."
            self.say_to_user(share_text)
            logger.info(f"OODA planner: auto-shared OODA-initiated goal result")

    # ── Persistence (sleep/wake) ──────────────────────────────────────

    _STATE_NOTE_NAME = "_ooda_planner_state"

    def save_state(self, write_named_note_fn) -> None:
        """Persist planner history to a named Note.

        Called during shutdown so the planner can resume with context
        on the next startup.
        """
        if not self.history._cycles:
            logger.info("OODA planner: no cycles to persist")
            return

        cycles_data = []
        for c in self.history._cycles:
            cycles_data.append({
                "timestamp": c.timestamp,
                "event_summary": c.event_summary,
                "assessment_summary": c.assessment_summary,
                "action": c.action,
                "result_summary": c.result_summary,
                "concern_ids": c.concern_ids,
                "content_summary": c.content_summary,
            })

        state = {
            "version": 1,
            "cycle_count": len(cycles_data),
            "cycles": cycles_data,
        }

        try:
            write_named_note_fn(self._STATE_NOTE_NAME, json.dumps(state))
            logger.info(
                f"OODA planner: saved {len(cycles_data)} cycles "
                f"to {self._STATE_NOTE_NAME}"
            )
        except Exception as e:
            logger.warning(f"OODA planner: save_state failed: {e}")

    def load_state(self, executor) -> None:
        """Load planner history from a named Note on startup.

        Args:
            executor: InfospaceExecutor for loading the Note.
        """
        try:
            result = executor.execute_action({
                "type": "load",
                "target": self._STATE_NOTE_NAME,
                "out": "$_ooda_planner_tmp",
            })
            if result.get("status") != "success":
                logger.info("OODA planner: no saved state found (first run)")
                return

            resource_id = result.get("resource_id", "")
            if not resource_id:
                return

            # Read the note content
            note_data = executor.resource_manager.get_resource(resource_id)
            if not note_data:
                return
            content = (
                getattr(note_data, "content", None)
                or (note_data.get("properties", {}) if isinstance(note_data, dict) else {}).get("content", "")
            )
            if not content:
                return

            state = json.loads(content) if isinstance(content, str) else content
            if not isinstance(state, dict) or "cycles" not in state:
                return

            count = 0
            for c in state["cycles"]:
                self.history.append(OodaCycle(
                    timestamp=c.get("timestamp", "?"),
                    event_summary=c.get("event_summary", ""),
                    assessment_summary=c.get("assessment_summary", ""),
                    action=c.get("action", {}),
                    result_summary=c.get("result_summary", ""),
                    concern_ids=c.get("concern_ids", []),
                    content_summary=c.get("content_summary", ""),
                ))
                count += 1

            logger.info(f"OODA planner: loaded {count} cycles from prior session")

        except Exception as e:
            logger.info(f"OODA planner: load_state failed: {e}")

    def get_recent_activity_summary(self, n: int = 5) -> str:
        """Return compact summary of recent OODA cycles for chat context.

        Used by _handle_chat_response to give the chat generator
        awareness of recent strategic activity.
        """
        if not self.history._cycles:
            return ""
        recent = self.history._cycles[-n:]
        lines = ["## Recent Agent Activity"]
        for c in recent:
            lines.append(self.history._summarize_cycle(c))
        return "\n".join(lines)

    # ── User-facing communication ───────────────────────────────────────

    def say_to_user(
        self,
        text: str,
        target: str = 'User',
        close: bool = False,
    ) -> None:
        """Publish a message to the user (or another agent) via Zenoh.

        This is the single site for outbound agent communication.
        """
        if target.lower() == 'user':
            target = 'User'

        content_payload = {'source': self.agent_name, 'text': text}
        if close:
            content_payload['close'] = True
        sense_data = {
            'timestamp': datetime.now().isoformat(),
            'sequence_id': 0,
            'mode': 'text',
            'content': json.dumps(content_payload),
        }
        self.session.put(
            f"cognitive/{target}/sense_data",
            json.dumps(sense_data),
        )

        # Publish to action channel for UI display
        action_data = {
            'type': 'say',
            'action_type': 'say',
            'action_id': f'action_{self.executive_node.action_counter}',
            'timestamp': datetime.now().isoformat(),
            'text': text,
            'source': self.agent_name,
            'target': target,
            'is_text_only': True,
        }
        self.action_publisher.put(json.dumps(action_data))
        self.executive_node.action_counter += 1
        if hasattr(self.executive_node, 'last_say_text'):
            self.executive_node.last_say_text = text
        if self.conversation_store:
            self.conversation_store.record_outgoing(
                target=target,
                text=text,
                act_type='say',
                close=close,
            )

        logger.info(f"Say [{target}]: {text}")

        # Dialog close handling for non-User targets
        if close and target.lower() != 'user':
            try:
                if self.conversation_store:
                    self.conversation_store.close_dialog(target)
                self.executive_node._dialog_cooldowns[target] = time.time()
                self.executive_node._dialog_purposes.pop(target, None)
                logger.info(f"Say [{target}]: dialog closed (close flag)")
            except Exception as e:
                logger.warning(f"Say close_dialog failed for {target}: {e}")

    def ask_user(self, question_text: str, target: str = 'User') -> Optional[str]:
        """Publish a question and block until the user responds or timeout.

        Returns the response text, or None on timeout.
        Called from the goal executor thread; the main loop routes replies
        via ``route_ask_reply``.
        """
        # Signal BEFORE publishing to prevent race
        self.awaiting_ask_response = True
        self.executive_node.execution_paused = True
        self.executive_node._publish_execution_state()

        # Drain stale items
        while not self._ask_response_queue.empty():
            try:
                self._ask_response_queue.get_nowait()
            except queue.Empty:
                break

        # Publish question
        action_data = {
            'type': 'ask',
            'action_type': 'ask',
            'action_id': f'action_{self.executive_node.action_counter}',
            'timestamp': datetime.now().isoformat(),
            'text': str(question_text),
            'source': self.agent_name,
            'target': target,
            'is_text_only': True,
        }
        self.action_publisher.put(json.dumps(action_data))
        self.executive_node.action_counter += 1
        if self.conversation_store:
            self.conversation_store.record_outgoing(
                target=target,
                text=str(question_text),
                act_type='ask',
                close=False,
            )

        # For non-User targets, send question to their sense_data
        if target != 'User':
            sense_data = {
                'timestamp': datetime.now().isoformat(),
                'sequence_id': 0,
                'mode': 'text',
                'content': json.dumps({
                    'source': self.agent_name,
                    'text': str(question_text),
                }),
            }
            self.session.put(
                f"cognitive/{target}/sense_data",
                json.dumps(sense_data),
            )

        logger.info(
            f"Ask: '{question_text}' -> blocking for response "
            f"(timeout={_ASK_TIMEOUT_SECONDS}s)"
        )

        try:
            response_text = self._ask_response_queue.get(
                timeout=_ASK_TIMEOUT_SECONDS
            )
        except queue.Empty:
            response_text = None

        # Clear blocking state
        self.awaiting_ask_response = False
        self.executive_node.execution_paused = False
        self.executive_node._publish_execution_state()

        if response_text is None:
            logger.info("Ask: timed out or interrupted waiting for response")
        else:
            logger.info(f"Ask: received response ({len(response_text)} chars)")

        return response_text

    # ── Goal executor signal handlers (default passthrough) ─────────────

    def handle_goal_say(
        self,
        text: str,
        target: str = 'User',
        close: bool = False,
    ) -> None:
        """Handle a say signal from the goal executor.

        Phase 1: passthrough — forward to user as-is.
        Future: OODA planner can suppress, batch, or add context.
        """
        self.say_to_user(text, target=target, close=close)

    def handle_goal_ask(
        self,
        question_text: str,
        target: str = 'User',
    ) -> Optional[str]:
        """Handle an ask signal from the goal executor.

        Phase 1: passthrough — forward to user, return response.
        Future: OODA planner can answer from context or redirect.

        Blocks the calling (goal executor) thread until a response arrives.
        """
        return self.ask_user(question_text, target=target)

    # ── Action dispatch (Phase 2) ──────────────────────────────────────

    def dispatch_action(self, action: Dict[str, Any]) -> Dict[str, Any]:
        """Dispatch a parsed OODA action to its handler.

        Returns a result dict with at least {status, summary}.
        Phase 2: only submit-goal and say/ask are wired. Other actions
        return stubs. Phase 3 will complete the wiring.
        """
        action_type = action.get("action", "")
        handler = self._action_handlers.get(action_type)
        if handler:
            return handler(action)
        logger.warning(f"Unknown OODA action: {action_type}")
        return {"status": "error", "summary": f"Unknown action: {action_type}"}

    @property
    def _action_handlers(self) -> Dict[str, Any]:
        return {
            "submit-goal": self._action_submit_goal,
            "say": self._action_say,
            "ask": self._action_ask,
            "wait": self._action_wait,
            "reflect": self._action_reflect,
            "update-concern": self._action_update_concern,
            "update-user-model": self._action_stub,
            "configure-sensor": self._action_stub,
            "sleep": self._action_stub,
        }

    def _action_submit_goal(self, action: Dict) -> Dict[str, Any]:
        """Submit a goal for direct execution (no scheduled goal persistence).

        The goal runs on the executive_node's worker thread. This method
        does NOT block — it submits and returns. The result arrives later
        as a goal-completion event in the OODA context.
        """
        goal_text = action.get("goal_text", "")
        concern_id = action.get("concern_id")
        expected_outcome = action.get("expected_outcome", "")

        if not goal_text:
            return {"status": "error", "summary": "submit-goal requires goal_text"}

        en = self.executive_node
        if en._is_goal_running():
            return {"status": "blocked", "summary": "Goal already running"}

        logger.info(
            f"OODA submit-goal: \"{goal_text[:100]}\" "
            f"(concern={concern_id or 'none'})"
        )

        # Run via existing infrastructure — parse_and_set_goal on worker thread
        en._run_goal_on_thread(
            en.parse_and_set_goal,
            en.character_config.get("template", ""),
            goal_text,
        )

        # Mark autonomous action for self-reporting
        try:
            ls = getattr(en, '_ooda_living_state', None)
            if ls:
                ls.record_operational_marker(
                    "last_autonomous_action",
                    summary=f"submit-goal: {goal_text[:150]}",
                    concern_id=concern_id or None,
                )
        except Exception:
            pass

        return {
            "status": "submitted",
            "summary": f"Goal submitted: {goal_text[:120]}",
            "concern_id": concern_id,
            "expected_outcome": expected_outcome,
        }

    def _action_say(self, action: Dict) -> Dict[str, Any]:
        text = action.get("text", "")
        if not text:
            return {"status": "error", "summary": "say requires text"}
        self.say_to_user(text)
        return {"status": "success", "summary": f"Said: {text[:80]}"}

    def _action_ask(self, action: Dict) -> Dict[str, Any]:
        text = action.get("text", "")
        if not text:
            return {"status": "error", "summary": "ask requires text"}
        # Note: this blocks the OODA planner until the user responds.
        # In Phase 3, the planning loop handles this appropriately.
        response = self.ask_user(text)
        if response is None:
            return {"status": "timeout", "summary": "Ask timed out"}
        return {"status": "success", "summary": f"User replied: {response[:80]}"}

    def _action_wait(self, action: Dict) -> Dict[str, Any]:
        reason = action.get("reason", "no reason given")
        logger.info(f"OODA wait: {reason}")
        return {"status": "success", "summary": f"Waiting: {reason}"}

    def _action_reflect(self, action: Dict) -> Dict[str, Any]:
        observation = action.get("observation", "")
        logger.info(f"OODA reflect: {observation[:120]}")
        return {"status": "success", "summary": f"Reflected: {observation[:80]}"}

    def _action_update_concern(self, action: Dict) -> Dict[str, Any]:
        """Update a concern's state via the derived concern model.

        Supports: satisfy (mark as addressed), activate, abandon,
        and weight/description updates.
        """
        concern_id = action.get("concern_id", "")
        updates = action.get("updates", {})
        if not concern_id:
            return {"status": "error", "summary": "update-concern requires concern_id"}

        en = self.executive_node
        dcm = getattr(en, '_derived_concern_model', None)
        if not dcm:
            return {"status": "error", "summary": "No derived concern model available"}

        status_change = updates.get("status")
        reason = updates.get("reason", updates.get("notes", ""))

        if status_change == "satisfied":
            revisit_hours = float(updates.get("revisit_hours", 24))
            patch = {
                "concern_id": concern_id,
                "op": "satisfy_concern",
                "why_this_satisfy": reason,
                "field_updates": {"revisit_hours": revisit_hours},
            }
            dcm._apply_patch(patch, evidence_ref="ooda_planner")
            dcm._save()
            return {
                "status": "success",
                "summary": f"Concern {concern_id} satisfied (revisit in {revisit_hours}h): {reason[:80]}",
            }
        elif status_change == "active":
            patch = {
                "concern_id": concern_id,
                "op": "activate_concern",
                "why_this_activate": reason,
                "field_updates": {},
            }
            dcm._apply_patch(patch, evidence_ref="ooda_planner")
            dcm._save()
            return {
                "status": "success",
                "summary": f"Concern {concern_id} activated: {reason[:80]}",
            }
        elif status_change == "abandoned":
            patch = {
                "concern_id": concern_id,
                "op": "abandon_concern",
                "why_this_abandon": reason,
                "field_updates": {},
            }
            dcm._apply_patch(patch, evidence_ref="ooda_planner")
            dcm._save()
            return {
                "status": "success",
                "summary": f"Concern {concern_id} abandoned: {reason[:80]}",
            }
        elif updates:
            # Weight or description update without status change
            for concern in dcm.concerns:
                if concern.get("concern_id") == concern_id:
                    if "weight" in updates:
                        concern["weight"] = float(updates["weight"])
                    if "description" in updates:
                        concern["concern_description"] = updates["description"]
                    if "notes" in updates:
                        concern.setdefault("history_summary", "")
                        concern["history_summary"] += f"\n{reason}" if reason else ""
                    dcm._save()
                    return {
                        "status": "success",
                        "summary": f"Concern {concern_id} updated: {json.dumps(updates)[:80]}",
                    }
            return {"status": "error", "summary": f"Concern {concern_id} not found"}
        else:
            return {"status": "error", "summary": "update-concern requires updates dict"}

    def _action_stub(self, action: Dict) -> Dict[str, Any]:
        """Stub for actions not yet wired."""
        action_type = action.get("action", "?")
        logger.info(f"OODA action stub: {action_type} (not yet implemented)")
        return {"status": "stub", "summary": f"{action_type} not yet implemented"}

    # ── Main loop integration ───────────────────────────────────────────

    def route_ask_reply(self, text: str) -> None:
        """Route a user reply to the pending ask.

        Called by the main loop when it detects a User message while
        ``awaiting_ask_response`` is True.
        """
        self._ask_response_queue.put(text)
        logger.info(f"Ask reply routed: \"{text[:60]}\"")
