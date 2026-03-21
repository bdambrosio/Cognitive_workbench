#!/usr/bin/env python3
"""
OODA Snapshot Renderer — on-demand markdown rendering of the living OODA state.

Pure functions, no LLM calls. Transforms OodaLivingState + concern data into
LLM-legible text for planner reflective state and evaluator context.
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Sequence


def _trend_arrow(trend: str) -> str:
    return {"rising": "\u2191", "falling": "\u2193", "stable": "\u2192"}.get(trend, "")


def render_reflective_snapshot(
    living_state,
    derived_concerns: Sequence[Dict[str, Any]],
    user_concerns: Sequence[Dict[str, Any]],
    agent_state_block: str = "",
) -> str:
    """Render full reflective snapshot as markdown for planner consumption.

    Args:
        living_state: OodaLivingState instance (or None).
        derived_concerns: List of derived concern dicts.
        user_concerns: List of user concern dicts.
        agent_state_block: Output of _build_agent_state_block() (optional).

    Returns:
        Markdown string suitable for injection into planner reflective state.
    """
    if living_state is None:
        return ""

    parts: List[str] = []

    # ── Orientation Landscape ──────────────────────────────────────────
    landscape_lines: List[str] = []

    # Concern activations with trends
    activations = getattr(living_state, "concern_activations", [])
    if activations:
        items = []
        for a in activations:
            arrow = _trend_arrow(a.get("trend", "stable"))
            items.append(f"{a['id']}({a.get('activation', 0)}{arrow})")
        landscape_lines.append(f"- Concern activations: {', '.join(items)}")

    # Derived concerns
    active_derived = [
        c for c in derived_concerns
        if c.get("status") in ("surfaced", "active")
    ]
    if active_derived:
        items = [f"{c.get('concern_label', '?')} ({c.get('status', '?')})" for c in active_derived]
        landscape_lines.append(f"- Derived concerns: [{', '.join(items)}]")

    # User concern snapshot
    uc_snapshot = getattr(living_state, "user_concern_snapshot", {})
    if uc_snapshot:
        items = [f"{count} {status}" for status, count in sorted(uc_snapshot.items())]
        landscape_lines.append(f"- User concerns: [{', '.join(items)}]")

    # Goal field
    goal_field = getattr(living_state, "goal_field", [])
    fg_id = getattr(living_state, "foregrounded_goal_id", None)
    if goal_field:
        status_counts: Dict[str, int] = {}
        fg_label = ""
        for g in goal_field:
            s = g.get("status", "?")
            status_counts[s] = status_counts.get(s, 0) + 1
            if g.get("goal_id") == fg_id:
                fg_label = f"{g.get('goal_id', '?')} \"{g.get('label', '')[:40]}\""
        parts_g = []
        if fg_label:
            parts_g.append(f"foregrounded: {fg_label}")
        for s, n in sorted(status_counts.items()):
            parts_g.append(f"{n} {s}")
        landscape_lines.append(f"- Goals: [{', '.join(parts_g)}]")

    if landscape_lines:
        parts.append("## Orientation Landscape\n" + "\n".join(landscape_lines))

    # ── Last Event ─────────────────────────────────────────────────────
    last_event = getattr(living_state, "last_event", {})
    if last_event:
        event_lines: List[str] = []
        et = last_event.get("event_type", "")
        cls = last_event.get("classification", "")
        src = last_event.get("source", "")
        preview = last_event.get("content_preview", "")
        event_lines.append(f"- {et}/{cls} from {src}: \"{preview}\"")

        assessment = last_event.get("assessment", {})
        if assessment:
            matters = assessment.get("matters", "")
            goal_rel = assessment.get("goal_relevance", "")
            action = assessment.get("action", "")
            event_lines.append(f"- Assessment: matters={matters}, goal_relevance={goal_rel}, action={action}")

        action_taken = last_event.get("action_taken", "")
        if action_taken:
            goal_id = last_event.get("goal_id") or ""
            event_lines.append(f"- Action: {action_taken}{' ' + goal_id if goal_id else ''}")

        parts.append("## Last Event\n" + "\n".join(event_lines))

    # ── Recent Transitions ─────────────────────────────────────────────
    transitions = list(getattr(living_state, "transitions", []))
    if transitions:
        # Show most recent first
        parts.append("## Recent Transitions\n" + "\n".join(reversed(transitions)))

    # ── Epistemic Boundaries ───────────────────────────────────────────
    markers = getattr(living_state, "epistemic_markers", [])
    if markers:
        marker_lines = [f"- {m}" for m in markers]
        parts.append("## Epistemic Boundaries\n" + "\n".join(marker_lines))

    return "\n\n".join(parts)


def render_evaluator_context(
    living_state,
    derived_concerns: Sequence[Dict[str, Any]],
) -> str:
    """Render compact orientation context for character evaluator enrichment.

    Returns a short string (~200 chars) summarizing current orientation trends.
    """
    if living_state is None:
        return ""

    parts: List[str] = []

    # Top 3 concern activations with trends
    activations = getattr(living_state, "concern_activations", [])
    if activations:
        items = []
        for a in activations[:3]:
            arrow = _trend_arrow(a.get("trend", "stable"))
            items.append(f"{a['id']}{arrow}")
        parts.append(f"orientation=[{','.join(items)}]")

    # Active derived concerns count
    active_derived = [
        c for c in derived_concerns
        if c.get("status") in ("surfaced", "active")
    ]
    if active_derived:
        parts.append(f"derived_concerns={len(active_derived)}")

    # Foregrounded goal
    fg_id = getattr(living_state, "foregrounded_goal_id", None)
    if fg_id:
        parts.append(f"fg_goal={fg_id}")

    return "; ".join(parts)
