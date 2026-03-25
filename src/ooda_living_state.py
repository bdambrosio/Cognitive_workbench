#!/usr/bin/env python3
"""
OODA Living State — continuously maintained record of the agent's orientation.

Updated incrementally at each OODA phase (observe/orient/act). No LLM calls;
all updates complete in microseconds. Persisted as a named Note (_ooda_state)
with debouncing.
"""

from __future__ import annotations

import json
import logging
import time
from collections import deque
from datetime import datetime, timezone
from typing import Any, Callable, Dict, List, Optional, Sequence

logger = logging.getLogger(__name__)

NOTE_NAME = "_ooda_state"
READABLE_NOTE_NAME = "_ooda_state_readable"

# Trend thresholds for activation history
_TREND_RISING = 0.05
_TREND_FALLING = -0.05
_ACTIVATION_HISTORY_LEN = 5
_TRANSITIONS_MAXLEN = 12
_EPISTEMIC_MAXLEN = 5
_PERSIST_DEBOUNCE_SECS = 2.0


def _trend_label(history: Sequence[float]) -> str:
    """Compute trend from activation history: rising, falling, or stable."""
    if len(history) < 3:
        return "stable"
    mid = len(history) // 2
    early = sum(list(history)[:mid]) / mid
    late = sum(list(history)[mid:]) / (len(history) - mid)
    delta = late - early
    if delta > _TREND_RISING:
        return "rising"
    elif delta < _TREND_FALLING:
        return "falling"
    return "stable"


def _trend_arrow(trend: str) -> str:
    return {"rising": "\u2191", "falling": "\u2193", "stable": "\u2192"}.get(trend, "")


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _now_hms() -> str:
    return datetime.now().strftime("%H:%M:%S")


class OodaLivingState:
    """Living OODA orientation state, updated at each loop phase."""

    def __init__(self) -> None:
        # Orientation landscape
        self.concern_activations: List[Dict[str, Any]] = []
        # [{id, activation, trend, description?}] sorted descending by activation
        self.user_concern_snapshot: List[Dict[str, Any]] = []
        # [{id, label, status, weight, stance}]
        self.goal_field: List[Dict[str, Any]] = []
        # [{goal_id, label, status}]
        self.foregrounded_goal_id: Optional[str] = None

        # Last event interpretation
        self.last_event: Dict[str, Any] = {}

        # Recent transitions ring buffer
        self.transitions: deque = deque(maxlen=_TRANSITIONS_MAXLEN)

        # Epistemic boundary markers
        self.epistemic_markers: List[str] = []

        # Internal: activation history for trend computation
        self._activation_history: Dict[str, deque] = {}

        # Persistence bookkeeping
        self._dirty: bool = False
        self._last_persist_time: float = 0.0

    # ── Update methods (called from _main_loop_tick) ───────────────────

    def update_after_observe(self, event) -> None:
        """Record what arrived from the Observe phase.

        Args:
            event: EventPacket dataclass instance.
        """
        content_preview = (getattr(event, "content", "") or "")[:80]
        self.last_event = {
            "event_type": getattr(event, "event_type", ""),
            "classification": getattr(event, "classification", ""),
            "source": getattr(event, "source", ""),
            "content_preview": content_preview,
            "goal_id": getattr(event, "goal_id", None),
            "timestamp": _now_iso(),
        }
        self._dirty = True

    def update_after_orient(
        self,
        oriented,
        concern_activations: Dict[str, float],
        concern_descriptions: Optional[Dict[str, str]] = None,
    ) -> None:
        """Record assessment and update activation trends.

        Args:
            oriented: OrientedEvent dataclass (event + assessment).
            concern_activations: The executive_node's _character_concern_activations dict.
            concern_descriptions: Optional {concern_id: "label — description"} for content.
        """
        assessment = getattr(oriented, "assessment", None) or {}
        _descs = concern_descriptions or {}

        # Extract assessment summary
        self.last_event["assessment"] = {
            "matters": assessment.get("matters", ""),
            "action": (assessment.get("action_evaluation") or {}).get("action_choice", ""),
            "goal_relevance": (assessment.get("salience_factors") or {}).get("goal_relevance", ""),
            "urgency": (assessment.get("salience_factors") or {}).get("urgency", ""),
            "rationale": assessment.get("overall_rationale", ""),
        }

        # Update activation history and compute trends
        for cid, value in concern_activations.items():
            if cid not in self._activation_history:
                self._activation_history[cid] = deque(maxlen=_ACTIVATION_HISTORY_LEN)
            self._activation_history[cid].append(value)

        # Build sorted activation list with trends and descriptions
        self.concern_activations = []
        for cid, value in sorted(
            concern_activations.items(), key=lambda x: x[1], reverse=True
        ):
            history = self._activation_history.get(cid, deque())
            trend = _trend_label(history)
            entry: Dict[str, Any] = {
                "id": cid,
                "activation": round(value, 3),
                "trend": trend,
            }
            if cid in _descs:
                entry["description"] = _descs[cid]
            self.concern_activations.append(entry)

        # Extract epistemic markers from assessment notes
        notes = assessment.get("notes", "")
        if "epistemic:" in notes:
            for segment in notes.split(";"):
                segment = segment.strip()
                if segment.startswith("epistemic:"):
                    marker = segment[len("epistemic:"):].strip()
                    if marker and marker != "none":
                        self.epistemic_markers.append(marker)
                        if len(self.epistemic_markers) > _EPISTEMIC_MAXLEN:
                            self.epistemic_markers = self.epistemic_markers[-_EPISTEMIC_MAXLEN:]
                    break

        self._dirty = True

    def update_after_act(self, action) -> None:
        """Record action taken and append transition to ring buffer.

        Args:
            action: Action dataclass (type, payload, assessment).
        """
        action_type = getattr(action, "type", "unknown")
        self.last_event["action_taken"] = action_type

        # Build transition line with content context
        ts = _now_hms()
        goal_id = self.last_event.get("goal_id") or ""
        goal_part = f" {goal_id}" if goal_id else ""

        # Content hint (first ~40 chars of event content)
        content_preview = self.last_event.get("content_preview", "")
        content_hint = f' "{content_preview[:40]}"' if content_preview else ""

        # Top concern with trend arrow
        concern_part = ""
        if self.concern_activations:
            top = self.concern_activations[0]
            concern_part = f" | {top['id']}{_trend_arrow(top['trend'])}"

        # Resulting state hint
        result_hint = action_type
        if action_type in ("proceed_goal", "dispatch_goal", "reuse_goal"):
            result_hint = "running"
        elif action_type == "chat_response":
            result_hint = "responded"
        elif action_type == "no_action":
            result_hint = "idle"
        elif action_type == "terminate_goal":
            result_hint = "terminated"

        line = f"[{ts}] {action_type}{goal_part}{content_hint}{concern_part} | \u2192 {result_hint}"
        self.transitions.append(line)
        self._dirty = True

    def update_user_concerns_snapshot(self, concerns: Sequence[Dict[str, Any]]) -> None:
        """Refresh user concern snapshot with content details."""
        self.user_concern_snapshot = [
            {
                "id": c.get("concern_id", ""),
                "label": c.get("concern_label", ""),
                "status": c.get("status", "?"),
                "weight": c.get("weight", 0),
                "stance": c.get("stance", ""),
            }
            for c in concerns[:10]
        ]
        self._dirty = True

    def update_goals(
        self,
        goals: Sequence[Dict[str, Any]],
        foregrounded_id: Optional[str] = None,
    ) -> None:
        """Refresh goal field snapshot with content."""
        self.goal_field = []
        for g in goals[:12]:
            self.goal_field.append({
                "goal_id": g.get("goal_id"),
                "label": g.get("name") or (g.get("goal_text") or ""),
                "goal_text": g.get("goal_text") or "",
                "status": g.get("status", "?"),
            })
        self.foregrounded_goal_id = foregrounded_id
        self._dirty = True

    # ── Persistence ────────────────────────────────────────────────────

    def maybe_persist(
        self,
        write_named_note_fn: Callable[[str, str], Any],
        derived_concerns: Optional[Sequence[Dict[str, Any]]] = None,
        planner_summary: str = "",
    ) -> None:
        """Save to named Note if dirty and debounce interval has elapsed.

        Args:
            write_named_note_fn: Callable that persists (name, content).
            derived_concerns: Optional derived concern list for rendered snapshot.
            planner_summary: One-line summary of current planner state (e.g. "in_progress step 3/16").
        """
        if not self._dirty:
            return
        now = time.monotonic()
        if now - self._last_persist_time < _PERSIST_DEBOUNCE_SECS:
            return
        try:
            data = self.to_dict()
            content = json.dumps(data, default=str)
            write_named_note_fn(NOTE_NAME, content)
            # Persist a self-explanatory markdown note for planner/human consumption
            readable = self._render_readable_markdown(
                derived_concerns or [], planner_summary=planner_summary)
            write_named_note_fn(READABLE_NOTE_NAME, readable)
            self._dirty = False
            self._last_persist_time = now
        except Exception as e:
            logger.warning(f"OodaLivingState: persist failed: {e}")

    def _compute_activation_delta(self, concern_id: str) -> float:
        """Return the change in activation from the previous tick for a concern.

        Returns 0.0 if fewer than 2 history entries exist.
        """
        history = self._activation_history.get(concern_id)
        if not history or len(history) < 2:
            return 0.0
        h = list(history)
        return h[-1] - h[-2]

    def _render_markdown(self, derived_concerns: Sequence[Dict[str, Any]]) -> str:
        """Render a human-readable markdown summary of current orientation state."""
        parts: List[str] = []

        # Concern activations
        if self.concern_activations:
            lines = ["## Concern Activations"]
            for a in self.concern_activations:
                arrow = _trend_arrow(a.get("trend", "stable"))
                desc = a.get("description", "")
                desc_part = f" — {desc}" if desc else ""
                lines.append(f"- **{a['id']}** ({a.get('activation', 0)}{arrow}){desc_part}")
            parts.append("\n".join(lines))

        # Derived concerns
        active_derived = [c for c in derived_concerns if c.get("status") in ("surfaced", "active")]
        if active_derived:
            lines = ["## Derived Concerns"]
            for c in active_derived:
                label = c.get("concern_label", "?")
                desc = c.get("concern_description", "")
                status = c.get("status", "?")
                origin = c.get("origin", "?")
                parent = c.get("parent_user_concern_id")
                parent_part = f", derived from {parent}" if parent else ""
                lines.append(f"- **{label}** [{status}, {origin}{parent_part}]: {desc}")
            parts.append("\n".join(lines))

        # User concerns
        if self.user_concern_snapshot:
            lines = ["## User Concerns"]
            for c in self.user_concern_snapshot:
                label = c.get("label", "?")
                status = c.get("status", "?")
                weight = c.get("weight", 0)
                stance = c.get("stance", "")
                stance_part = f", {stance}" if stance else ""
                lines.append(f"- [{status}] **{label}** (weight={weight}{stance_part})")
            parts.append("\n".join(lines))

        # Goals
        if self.goal_field:
            lines = ["## Goals"]
            for g in self.goal_field:
                gid = g.get("goal_id", "?")
                label = g.get("label", "?")
                status = g.get("status", "?")
                fg = " [FOREGROUNDED]" if gid == self.foregrounded_goal_id else ""
                goal_text = g.get("goal_text", "")
                text_part = f": {goal_text}" if goal_text and goal_text != label else ""
                lines.append(f"- [{status}] {gid} \"{label}\"{fg}{text_part}")
            parts.append("\n".join(lines))

        # Last event
        if self.last_event:
            lines = ["## Last Event"]
            et = self.last_event.get("event_type", "")
            cls = self.last_event.get("classification", "")
            src = self.last_event.get("source", "")
            preview = self.last_event.get("content_preview", "")
            lines.append(f"- **{et}/{cls}** from {src}: \"{preview}\"")
            assessment = self.last_event.get("assessment", {})
            if assessment:
                rationale = assessment.get("rationale", "")
                lines.append(
                    f"- Assessment: matters={assessment.get('matters', '?')}, "
                    f"goal_relevance={assessment.get('goal_relevance', '?')}, "
                    f"urgency={assessment.get('urgency', '?')}, "
                    f"action={assessment.get('action', '?')}"
                )
                if rationale:
                    lines.append(f"- Rationale: {rationale}")
            action = self.last_event.get("action_taken", "")
            if action:
                lines.append(f"- Action taken: {action}")
            parts.append("\n".join(lines))

        # Transitions
        transitions = list(self.transitions)
        if transitions:
            lines = ["## Recent Transitions"]
            for t in reversed(transitions):
                lines.append(t)
            parts.append("\n".join(lines))

        # Epistemic markers
        if self.epistemic_markers:
            lines = ["## Epistemic Boundaries"]
            for m in self.epistemic_markers:
                lines.append(f"- {m}")
            parts.append("\n".join(lines))

        return "\n\n".join(parts)

    def _render_readable_markdown(
        self,
        derived_concerns: Sequence[Dict[str, Any]],
        planner_summary: str = "",
    ) -> str:
        """Render a self-explanatory markdown snapshot of the agent's OODA state.

        Captures both the current state AND the dynamics (deltas, trends) of
        the OODA loop. Intended for planner and human consumption.
        """
        parts: List[str] = []
        ts = _now_iso()

        # ── Orientation Summary ───────────────────────────────────────────
        # A 1-2 sentence snapshot: what is foregrounded, what just happened.
        summary_bits: List[str] = []

        fg_goal = None
        if self.foregrounded_goal_id and self.goal_field:
            fg_goal = next(
                (g for g in self.goal_field
                 if g.get("goal_id") == self.foregrounded_goal_id),
                None,
            )
        if fg_goal:
            summary_bits.append(
                f"Foregrounded goal: \"{fg_goal.get('label', '?')}\" [{fg_goal.get('status', '?')}].")
        elif any(g.get("status") == "running" for g in self.goal_field):
            running = [g for g in self.goal_field if g.get("status") == "running"]
            summary_bits.append(
                f"Running: \"{running[0].get('label', '?')}\".")

        if self.concern_activations:
            top = self.concern_activations[0]
            top_label = top.get("description", top["id"]).split(" — ")[0]
            summary_bits.append(
                f"Dominant concern: {top_label} ({top.get('activation', 0)}{_trend_arrow(top.get('trend', 'stable'))}).")

        if self.last_event:
            action_taken = self.last_event.get("action_taken", "")
            src = self.last_event.get("source", "")
            if action_taken:
                summary_bits.append(f"Last action: {action_taken} (from {src}).")

        if planner_summary:
            summary_bits.append(f"Planner: {planner_summary}.")

        header = (
            f"# OODA State\n"
            f"Continuously updated orientation record — state and dynamics of the agent's attention, goals, and concerns.\n"
            f"Updated: {ts}\n\n"
            f"## Orientation Summary\n"
        )
        header += " ".join(summary_bits) if summary_bits else "No active orientation."
        parts.append(header)

        # ── Foreground ────────────────────────────────────────────────────
        fg_lines: List[str] = [
            "## Foreground",
            "What is driving current behavior: the active goal and dominant concern.",
        ]
        if fg_goal:
            goal_text = fg_goal.get("goal_text", "")
            if goal_text and goal_text != fg_goal.get("label", ""):
                fg_lines.append(f"- **Goal**: {fg_goal.get('label', '?')} [{fg_goal.get('status', '?')}]: {goal_text}")
            else:
                fg_lines.append(f"- **Goal**: {fg_goal.get('label', '?')} [{fg_goal.get('status', '?')}]")
        elif self.goal_field:
            running = [g for g in self.goal_field if g.get("status") == "running"]
            if running:
                g = running[0]
                goal_text = g.get("goal_text", "")
                if goal_text and goal_text != g.get("label", ""):
                    fg_lines.append(f"- **Goal**: {g.get('label', '?')} [running]: {goal_text}")
                else:
                    fg_lines.append(f"- **Goal**: {g.get('label', '?')} [running]")
            else:
                fg_lines.append("- **Goal**: none running")

        if self.concern_activations:
            top = self.concern_activations[0]
            cid = top["id"]
            activation = top.get("activation", 0)
            delta = self._compute_activation_delta(cid)
            delta_str = f" (Δ{delta:+.3f})" if delta != 0 else ""
            desc = top.get("description", cid)
            fg_lines.append(
                f"- **Concern**: {desc} — activation {activation}"
                f"{_trend_arrow(top.get('trend', 'stable'))}{delta_str}")

        if planner_summary:
            fg_lines.append(f"- **Planner**: {planner_summary}")
        parts.append("\n".join(fg_lines))

        # ── Attention Dynamics ────────────────────────────────────────────
        # All concerns with nonzero activation or nonzero delta, sorted by activation.
        if self.concern_activations:
            lines = [
                "## Attention",
                "What the agent is attending to. Activation reflects attentional pull; "
                "delta (Δ) shows change since last tick.",
            ]
            for a in self.concern_activations:
                cid = a["id"]
                activation = a.get("activation", 0)
                trend = a.get("trend", "stable")
                delta = self._compute_activation_delta(cid)
                # Skip negligible concerns (low activation AND no recent change)
                if activation < 0.05 and abs(delta) < 0.01:
                    continue
                desc = a.get("description", cid)
                # Use short label (before " — ") for compactness
                short_label = desc.split(" — ")[0] if " — " in desc else desc
                delta_str = f", Δ{delta:+.3f}" if delta != 0 else ""
                arrow = _trend_arrow(trend)
                lines.append(
                    f"- **{short_label}** — {activation}{arrow}{delta_str}")
            if len(lines) > 2:  # more than just header
                parts.append("\n".join(lines))

        # ── Goals (tiered) ────────────────────────────────────────────────
        if self.goal_field:
            running = [g for g in self.goal_field if g.get("status") == "running"]
            ready = [g for g in self.goal_field if g.get("status") in ("ready", "pending", "scheduled")]
            recent_done = [g for g in self.goal_field
                           if g.get("status") in ("completed", "failed")][-3:]  # last 3

            lines = [
                "## Goals",
                "Running goals are being actively executed. Ready goals await dispatch. "
                "Recently completed goals shown for context.",
            ]
            if running:
                for g in running:
                    label = g.get("label", "?")
                    goal_text = g.get("goal_text", "")
                    fg = " **[FOREGROUNDED]**" if g.get("goal_id") == self.foregrounded_goal_id else ""
                    if goal_text and goal_text != label:
                        lines.append(f"- [running]{fg} **{label}**: {goal_text}")
                    else:
                        lines.append(f"- [running]{fg} {label}")
            else:
                lines.append("- No goals currently running.")

            if ready:
                for g in ready:
                    label = g.get("label", "?")
                    goal_text = g.get("goal_text", "")
                    if goal_text and goal_text != label:
                        lines.append(f"- [ready] **{label}**: {goal_text}")
                    else:
                        lines.append(f"- [ready] {label}")

            if recent_done:
                for g in recent_done:
                    label = g.get("label", "?")
                    status = g.get("status", "?")
                    lines.append(f"- [{status}] {label}")

            # Count of omitted completed goals
            total_done = len([g for g in self.goal_field
                              if g.get("status") in ("completed", "failed")])
            omitted = total_done - len(recent_done)
            if omitted > 0:
                lines.append(f"- ({omitted} earlier completed goals omitted)")
            parts.append("\n".join(lines))

        # ── User Concerns (filtered) ─────────────────────────────────────
        if self.user_concern_snapshot:
            ongoing = [c for c in self.user_concern_snapshot
                       if c.get("status") in ("open", "active")]
            closed = [c for c in self.user_concern_snapshot
                      if c.get("status") == "closed"]

            lines = [
                "## User Concerns",
                "Topics the user has raised. Weight reflects importance; stance reflects attitude. "
                "Only ongoing concerns shown in full; closed concerns summarized.",
            ]
            if ongoing:
                for c in ongoing:
                    label = c.get("label", "?")
                    weight = c.get("weight", 0)
                    stance = c.get("stance", "")
                    stance_part = f", {stance}" if stance else ""
                    lines.append(f"- [ongoing] **{label}** (weight={weight}{stance_part})")
            else:
                lines.append("- No ongoing user concerns.")

            if closed:
                labels = [c.get("label", "?") for c in closed]
                lines.append(f"- Closed ({len(closed)}): {'; '.join(labels)}")
            parts.append("\n".join(lines))

        # ── Derived Concerns ──────────────────────────────────────────────
        active_derived = [c for c in derived_concerns
                          if c.get("status") in ("surfaced", "active")]
        if active_derived:
            lines = [
                "## Derived Concerns",
                "Concerns inferred from user behavior or context. "
                "May drive proactive action without explicit instruction.",
            ]
            for c in active_derived:
                label = c.get("concern_label", "?")
                desc = c.get("concern_description", "")
                parent = c.get("parent_user_concern_id")
                origin_part = f" (from: {parent})" if parent else ""
                lines.append(f"- **{label}**{origin_part}: {desc}")
            parts.append("\n".join(lines))

        # ── Last Orientation Step ─────────────────────────────────────────
        if self.last_event:
            lines = [
                "## Last Orientation Step",
                "What happened on the most recent OODA tick: the event observed, "
                "how it was assessed, and what action was taken.",
            ]
            et = self.last_event.get("event_type", "")
            cls = self.last_event.get("classification", "")
            src = self.last_event.get("source", "")
            preview = self.last_event.get("content_preview", "")
            lines.append(f"- Event: **{et}/{cls}** from {src}: \"{preview}\"")

            assessment = self.last_event.get("assessment", {})
            if assessment:
                matters = assessment.get("matters", "?")
                goal_rel = assessment.get("goal_relevance", "?")
                urgency = assessment.get("urgency", "?")
                action_choice = assessment.get("action", "?")
                lines.append(
                    f"- Assessment: matters={matters}, goal_relevance={goal_rel}, "
                    f"urgency={urgency}, action={action_choice}")
                rationale = assessment.get("rationale", "")
                if rationale:
                    lines.append(f"- Rationale: {rationale}")

            # Show activation changes from this tick
            deltas = []
            for a in self.concern_activations:
                cid = a["id"]
                d = self._compute_activation_delta(cid)
                if abs(d) >= 0.01:
                    short = a.get("description", cid).split(" — ")[0]
                    deltas.append(f"{short} {d:+.3f}")
            if deltas:
                lines.append(f"- Activation changes: {', '.join(deltas)}")

            action_taken = self.last_event.get("action_taken", "")
            if action_taken:
                lines.append(f"- Action taken: {action_taken}")
            parts.append("\n".join(lines))

        # ── Recent Transitions ────────────────────────────────────────────
        transitions = list(self.transitions)
        if transitions:
            lines = [
                "## Recent Transitions",
                "Log of recent observe-orient-act cycles, most recent first.",
            ]
            for t in reversed(transitions):
                lines.append(t)
            parts.append("\n".join(lines))

        # ── Unresolved / Opaque ───────────────────────────────────────────
        # Collect: epistemic markers + goals stuck in ready state
        unresolved_lines: List[str] = []
        if self.epistemic_markers:
            for m in self.epistemic_markers:
                unresolved_lines.append(f"- {m}")
        # Flag ready goals that might be stalled (heuristic: ready but not foregrounded
        # while other goals are running)
        if self.goal_field:
            ready_goals = [g for g in self.goal_field
                           if g.get("status") in ("ready", "pending")]
            running_goals = [g for g in self.goal_field
                             if g.get("status") == "running"]
            if ready_goals and running_goals:
                for g in ready_goals:
                    unresolved_lines.append(
                        f"- Goal waiting: \"{g.get('label', '?')}\" (ready but not foregrounded)")

        if unresolved_lines:
            lines = [
                "## Unresolved / Opaque",
                "Things the agent cannot currently determine, or items that may need attention.",
            ]
            lines.extend(unresolved_lines)
            parts.append("\n".join(lines))

        return "\n\n".join(parts)

    def load(self, infospace_executor) -> bool:
        """Load state from named Note. Returns True if loaded successfully."""
        try:
            result = infospace_executor.execute_action({
                "type": "load", "target": NOTE_NAME, "out": "$_ooda_state_tmp",
            })
            if not result or result.get("status") != "success":
                return False
            rid = result.get("resource_id")
            if not rid:
                return False
            content = infospace_executor._get_content(rid)
            if not content:
                return False
            data = json.loads(content) if isinstance(content, str) else content
            if isinstance(data, dict):
                self.from_dict(data)
                return True
        except Exception as e:
            logger.info(f"OodaLivingState: no prior state to load ({e})")
        return False

    # ── Serialization ──────────────────────────────────────────────────

    def to_dict(self) -> Dict[str, Any]:
        return {
            "concern_activations": self.concern_activations,
            "user_concern_snapshot": self.user_concern_snapshot,
            "goal_field": self.goal_field,
            "foregrounded_goal_id": self.foregrounded_goal_id,
            "last_event": self.last_event,
            "transitions": list(self.transitions),
            "epistemic_markers": self.epistemic_markers,
            "_activation_history": {
                k: list(v) for k, v in self._activation_history.items()
            },
            "updated": _now_iso(),
        }

    def from_dict(self, data: Dict[str, Any]) -> None:
        self.concern_activations = data.get("concern_activations", [])
        # Handle both old format (dict of counts) and new format (list of concern details)
        uc_data = data.get("user_concern_snapshot", [])
        self.user_concern_snapshot = uc_data if isinstance(uc_data, list) else []
        self.goal_field = data.get("goal_field", [])
        self.foregrounded_goal_id = data.get("foregrounded_goal_id")
        self.last_event = data.get("last_event", {})
        self.transitions = deque(
            data.get("transitions", []), maxlen=_TRANSITIONS_MAXLEN
        )
        self.epistemic_markers = data.get("epistemic_markers", [])
        # Restore activation history
        hist = data.get("_activation_history", {})
        self._activation_history = {
            k: deque(v, maxlen=_ACTIVATION_HISTORY_LEN)
            for k, v in hist.items()
        }
        self._dirty = False
