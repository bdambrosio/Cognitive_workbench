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

    # Concern activations with trends and descriptions
    activations = getattr(living_state, "concern_activations", [])
    if activations:
        for a in activations:
            arrow = _trend_arrow(a.get("trend", "stable"))
            desc = a.get("description", "")
            desc_part = f": {desc}" if desc else ""
            landscape_lines.append(f"- **{a['id']}** ({a.get('activation', 0)}{arrow}){desc_part}")

    # Derived concerns
    active_derived = [
        c for c in derived_concerns
        if c.get("status") in ("surfaced", "active")
    ]
    if active_derived:
        for c in active_derived:
            label = c.get("concern_label", "?")
            desc = c.get("concern_description", "")
            status = c.get("status", "?")
            landscape_lines.append(f"- Derived: **{label}** [{status}]: {desc}")

    # User concern snapshot
    uc_snapshot = getattr(living_state, "user_concern_snapshot", [])
    if uc_snapshot:
        if isinstance(uc_snapshot, list):
            for c in uc_snapshot:
                label = c.get("label", "?")
                status = c.get("status", "?")
                weight = c.get("weight", 0)
                landscape_lines.append(f"- User: [{status}] **{label}** (w={weight})")
        elif isinstance(uc_snapshot, dict):
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


# ── Self-Model Rendering ──────────────────────────────────────────────────

def render_self_model_section(
    scheduler_status: Optional[Dict[str, Any]] = None,
    tasks: Optional[Sequence[Dict[str, Any]]] = None,
    derived_concerns: Optional[Sequence[Dict[str, Any]]] = None,
    scheduled_goals: Optional[Sequence[Dict[str, Any]]] = None,
    sensor_configs: Optional[Sequence[Dict[str, Any]]] = None,
    execution_mode: str = "step",
    tool_count: int = 0,
) -> str:
    """Render the operational self-model section for planner consumption.

    Target budget: ~550 tokens. Returns markdown string or "" if no data.
    """
    parts: List[str] = []
    tasks = list(tasks or [])
    derived_concerns = list(derived_concerns or [])
    sensor_configs = list(sensor_configs or [])
    scheduled_goals = list(scheduled_goals or [])
    scheduler_status = scheduler_status or {}

    # ── How I Work (semi-static, ~100 tokens) ─────────────────────────
    task_count = len(tasks)
    sensor_count = len(sensor_configs)
    parts.append(
        "### How I Work\n"
        "I operate via a continuous OODA loop. I pursue sustained activities through tasks — "
        "persistent commitments linked to concerns. Each task accumulates state across executions "
        "and manages a sequence of goals. Goals are my atomic unit of action (planned and executed "
        f"step-by-step). I have {tool_count} tools and {sensor_count} sensor(s). "
        "I can have multiple active tasks but execute only one goal at a time."
    )

    # ── Current Operational State (~60 tokens) ────────────────────────
    sched_enabled = scheduler_status.get("enabled", False)
    sched_interval = scheduler_status.get("interval", 0)
    sched_executing = scheduler_status.get("executing_goal_id")
    sched_state = "executing" if sched_executing else "idle"
    paused = "paused" if execution_mode == "step" else "running"

    operational = [t for t in tasks if t.get("lifecycle") == "operational"]
    establishing = [t for t in tasks if t.get("lifecycle") != "operational"
                    and t.get("status") == "in_progress"]

    state_lines = [
        f"- Scheduler: {'enabled' if sched_enabled else 'disabled'}, {int(sched_interval)}s interval, {sched_state}",
        f"- Active tasks: {len(operational)} operational, {len(establishing)} establishing",
        f"- Execution mode: {paused}",
    ]
    parts.append("### Current Operational State\n" + "\n".join(state_lines))

    # ── My Commitments (active tasks + concern gaps, ~60 tokens/task) ─
    commitment_lines: List[str] = []
    # Build concern→task map for gap detection
    concern_to_task: Dict[str, str] = {}
    for t in tasks:
        cid = t.get("linked_concern_id")
        if cid:
            concern_to_task[cid] = t.get("task_wip_id", t.get("task_id", "?"))

    for t in tasks:
        wip_id = t.get("task_wip_id", t.get("task_id", "?"))
        intention = t.get("intention", "?")
        lifecycle = t.get("lifecycle", "establishing")
        linked = t.get("linked_concern_id", "")

        if lifecycle == "operational":
            sched = t.get("schedule_mode", "manual")
            last_exec = t.get("last_executed")
            exec_count = t.get("execution_count", 0)
            history = t.get("execution_history", [])
            last_summary = ""
            if history:
                last_entry = history[-1]
                last_summary = f" → {last_entry.get('outcome', '?')}"
                s = last_entry.get("summary", "")
                if s:
                    last_summary += f": {s[:60]}"

            last_str = f"Last: {last_exec[:16]}{last_summary}" if last_exec else "Never executed"
            commitment_lines.append(
                f"- **{wip_id}** [operational{', ' + linked if linked else ''}]\n"
                f"  Intention: {intention}\n"
                f"  Schedule: {sched} | {last_str} | {exec_count} runs"
            )
        else:
            phase = t.get("phase", "?")
            status = t.get("status", "?")
            milestones = t.get("milestones_completed", [])
            last_ms = ""
            if milestones:
                last_ms = f"\n  Last milestone: {milestones[-1].get('goal_text', '')[:60]} ({milestones[-1].get('status', '?')})"
            commitment_lines.append(
                f"- **{wip_id}** [establishing, phase: {phase}, {status}]\n"
                f"  Intention: {intention}{last_ms}"
            )

    # Concern gaps: active/surfaced concerns with no task
    for c in derived_concerns:
        if c.get("status") not in ("surfaced", "active"):
            continue
        cid = c.get("concern_id", "")
        if cid in concern_to_task:
            continue
        label = c.get("concern_label", "?")
        weight = c.get("weight", 0)
        commitment_lines.append(
            f"- {cid} \"{label}\" [active, w={weight}] — NO TASK"
        )

    if commitment_lines:
        parts.append("### My Commitments (Active Tasks)\n" + "\n".join(commitment_lines))
    else:
        parts.append("### My Commitments (Active Tasks)\nNo active tasks or unserviced concerns.")

    # ── My Sensing (~20 tokens/sensor) ────────────────────────────────
    if sensor_configs:
        sensor_lines = []
        for s in sensor_configs:
            name = s.get("name", "?")
            stype = s.get("type", "?")
            disp = s.get("disposition", "?")
            sched_s = s.get("schedule_seconds", 0)
            sensor_lines.append(f"- {name}: {stype}-type, disposition={disp}, every {int(sched_s)}s")
        parts.append("### Active Sensors\n" + "\n".join(sensor_lines))

    return "## Operational Self-Model\n\n" + "\n\n".join(parts)
