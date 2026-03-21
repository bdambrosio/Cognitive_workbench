"""Unit tests for OODA snapshot renderer."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from ooda_living_state import OodaLivingState
from ooda_snapshot_renderer import render_reflective_snapshot, render_evaluator_context


# ── Stub helpers ──────────────────────────────────────────────────────

def _make_populated_state():
    """Create a living state with representative data."""
    state = OodaLivingState()
    state.concern_activations = [
        {"id": "attend_to_user_concerns", "activation": 0.42, "trend": "rising",
         "description": "attend_to_user_concerns — Ongoing user topics"},
        {"id": "homeostasis", "activation": 0.21, "trend": "stable",
         "description": "homeostasis — Operational health"},
        {"id": "attend_to_user", "activation": 0.08, "trend": "falling",
         "description": "attend_to_user — Responsiveness to user"},
    ]
    state.user_concern_snapshot = [
        {"id": "c1", "label": "solar panel output", "status": "ongoing", "weight": 0.8, "stance": "concerned"},
        {"id": "c2", "label": "battery health", "status": "dormant", "weight": 0.3, "stance": "exploratory"},
    ]
    state.goal_field = [
        {"goal_id": "goal_13", "label": "research topic X", "status": "running"},
        {"goal_id": "goal_14", "label": "write report", "status": "ready"},
    ]
    state.foregrounded_goal_id = "goal_13"
    state.last_event = {
        "event_type": "goal_initiation",
        "classification": "proceed",
        "source": "User",
        "content_preview": "proceed goal_13",
        "goal_id": "goal_13",
        "assessment": {
            "matters": "yes",
            "goal_relevance": "strong",
            "action": "trigger_existing_goal",
            "urgency": "moderate",
            "rationale": "User wants to proceed with research",
        },
        "action_taken": "proceed_goal",
    }
    state.transitions.append("[17:55:10] chat | attend_to_user\u2191 | \u2192 responded")
    state.transitions.append("[17:56:25] proceed_goal goal_13 | attend_user_concerns\u2191 | \u2192 running")
    state.epistemic_markers = ["status_unverified"]
    return state


# ── Reflective snapshot tests ─────────────────────────────────────────

def test_render_reflective_snapshot_contains_sections():
    state = _make_populated_state()
    derived = [
        {"concern_label": "solar_monitoring", "status": "active", "concern_id": "dc1"},
    ]
    user_concerns = [{"status": "ongoing"}, {"status": "dormant"}]

    result = render_reflective_snapshot(state, derived, user_concerns)

    assert "## Orientation Landscape" in result
    assert "## Last Event" in result
    assert "## Recent Transitions" in result
    assert "## Epistemic Boundaries" in result


def test_render_reflective_snapshot_concern_activations():
    state = _make_populated_state()
    result = render_reflective_snapshot(state, [], [])

    assert "attend_to_user_concerns" in result
    assert "0.42" in result
    assert "Ongoing user topics" in result  # description included
    assert "homeostasis" in result
    assert "attend_to_user" in result


def test_render_reflective_snapshot_derived_concerns():
    state = _make_populated_state()
    derived = [
        {"concern_label": "monitor solar", "concern_description": "Track output drops", "status": "active"},
        {"concern_label": "old thing", "concern_description": "Done", "status": "resolved"},  # should be excluded
        {"concern_label": "new idea", "concern_description": "Fresh insight", "status": "surfaced"},
    ]
    result = render_reflective_snapshot(state, derived, [])

    assert "monitor solar" in result
    assert "[active]" in result
    assert "new idea" in result
    assert "[surfaced]" in result
    assert "old thing" not in result


def test_render_reflective_snapshot_goal_field():
    state = _make_populated_state()
    result = render_reflective_snapshot(state, [], [])

    assert "foregrounded: goal_13" in result
    assert "research topic X" in result


def test_render_reflective_snapshot_last_event():
    state = _make_populated_state()
    result = render_reflective_snapshot(state, [], [])

    assert "proceed goal_13" in result
    assert "matters=yes" in result
    assert "trigger_existing_goal" in result


def test_render_reflective_snapshot_transitions_reversed():
    state = _make_populated_state()
    result = render_reflective_snapshot(state, [], [])

    # Most recent should appear first
    lines = result.split("\n")
    transition_start = None
    for i, line in enumerate(lines):
        if "## Recent Transitions" in line:
            transition_start = i
            break
    assert transition_start is not None
    # First transition after header should be the most recent
    assert "17:56:25" in lines[transition_start + 1]


def test_render_reflective_snapshot_epistemic():
    state = _make_populated_state()
    result = render_reflective_snapshot(state, [], [])

    assert "status_unverified" in result


def test_render_reflective_snapshot_none_state():
    result = render_reflective_snapshot(None, [], [])
    assert result == ""


def test_render_reflective_snapshot_empty_state():
    state = OodaLivingState()
    result = render_reflective_snapshot(state, [], [])
    # Should not crash, may be minimal
    assert isinstance(result, str)


# ── Evaluator context tests ───────────────────────────────────────────

def test_render_evaluator_context():
    state = _make_populated_state()
    derived = [
        {"status": "active", "concern_label": "a"},
        {"status": "surfaced", "concern_label": "b"},
        {"status": "resolved", "concern_label": "c"},
    ]

    result = render_evaluator_context(state, derived)

    assert "orientation=" in result
    assert "attend_to_user_concerns\u2191" in result
    assert "derived_concerns=2" in result
    assert "fg_goal=goal_13" in result


def test_render_evaluator_context_compact():
    state = _make_populated_state()
    result = render_evaluator_context(state, [])
    # Should be reasonably compact
    assert len(result) < 300


def test_render_evaluator_context_none_state():
    result = render_evaluator_context(None, [])
    assert result == ""
