"""Unit tests for OodaLivingState."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from collections import deque
from dataclasses import dataclass
from typing import Any, Dict, Optional

from ooda_living_state import OodaLivingState, _trend_label, _trend_arrow


# ── Stub dataclasses matching executive_node shapes ───────────────────

@dataclass
class FakeEvent:
    event_type: str = "user_text"
    classification: str = "chat"
    content: str = "hello"
    source: str = "User"
    raw_sense_data: dict = None
    goal_id: Optional[str] = None

    def __post_init__(self):
        if self.raw_sense_data is None:
            self.raw_sense_data = {}


@dataclass
class FakeOrientedEvent:
    event: Any = None
    assessment: Optional[Dict[str, Any]] = None


@dataclass
class FakeAction:
    type: str = "chat_response"
    payload: dict = None
    assessment: Optional[Dict[str, Any]] = None

    def __post_init__(self):
        if self.payload is None:
            self.payload = {}


# ── Trend computation tests ───────────────────────────────────────────

def test_trend_label_rising():
    history = deque([0.1, 0.15, 0.2, 0.3, 0.4])
    assert _trend_label(history) == "rising"


def test_trend_label_falling():
    history = deque([0.4, 0.35, 0.3, 0.2, 0.1])
    assert _trend_label(history) == "falling"


def test_trend_label_stable():
    history = deque([0.3, 0.3, 0.3, 0.3, 0.3])
    assert _trend_label(history) == "stable"


def test_trend_label_short_history():
    assert _trend_label(deque([0.1])) == "stable"
    assert _trend_label(deque([0.1, 0.5])) == "stable"


def test_trend_arrow():
    assert _trend_arrow("rising") == "\u2191"
    assert _trend_arrow("falling") == "\u2193"
    assert _trend_arrow("stable") == "\u2192"
    assert _trend_arrow("unknown") == ""


# ── Update after observe ──────────────────────────────────────────────

def test_update_after_observe():
    state = OodaLivingState()
    event = FakeEvent(content="proceed goal_3", classification="proceed", goal_id="goal_3")
    state.update_after_observe(event)

    assert state.last_event["event_type"] == "user_text"
    assert state.last_event["classification"] == "proceed"
    assert state.last_event["content_preview"] == "proceed goal_3"
    assert state.last_event["goal_id"] == "goal_3"
    assert state._dirty


# ── Update after orient ───────────────────────────────────────────────

def test_update_after_orient_activations():
    state = OodaLivingState()
    event = FakeEvent()
    state.update_after_observe(event)

    oriented = FakeOrientedEvent(
        event=event,
        assessment={
            "matters": "yes",
            "salience_factors": {"goal_relevance": "strong", "urgency": "moderate"},
            "action_evaluation": {"action_choice": "trigger_existing_goal"},
            "overall_rationale": "User wants to proceed",
            "notes": "concerns: attend_to_user=strong, homeostasis=none; epistemic: none",
        },
    )
    activations = {"attend_to_user": 0.45, "homeostasis": 0.1}
    state.update_after_orient(oriented, activations)

    # Should have sorted activations
    assert len(state.concern_activations) == 2
    assert state.concern_activations[0]["id"] == "attend_to_user"
    assert state.concern_activations[0]["activation"] == 0.45
    assert state.concern_activations[1]["id"] == "homeostasis"

    # Assessment summary should be captured
    assert state.last_event["assessment"]["matters"] == "yes"
    assert state.last_event["assessment"]["goal_relevance"] == "strong"


def test_update_after_orient_epistemic_markers():
    state = OodaLivingState()
    event = FakeEvent()
    state.update_after_observe(event)

    oriented = FakeOrientedEvent(
        event=event,
        assessment={
            "notes": "concerns: homeostasis=weak; epistemic: status_unverified",
        },
    )
    state.update_after_orient(oriented, {"homeostasis": 0.2})

    assert "status_unverified" in state.epistemic_markers


def test_update_after_orient_epistemic_none_ignored():
    state = OodaLivingState()
    event = FakeEvent()
    state.update_after_observe(event)

    oriented = FakeOrientedEvent(
        event=event,
        assessment={"notes": "concerns: homeostasis=weak; epistemic: none"},
    )
    state.update_after_orient(oriented, {"homeostasis": 0.2})

    assert len(state.epistemic_markers) == 0


# ── Update after act ──────────────────────────────────────────────────

def test_update_after_act_transition():
    state = OodaLivingState()
    event = FakeEvent(goal_id="goal_3", classification="proceed")
    state.update_after_observe(event)

    # Fake orient with activations
    oriented = FakeOrientedEvent(event=event, assessment={})
    state.update_after_orient(oriented, {"attend_to_user": 0.5})

    action = FakeAction(type="proceed_goal")
    state.update_after_act(action)

    assert len(state.transitions) == 1
    line = state.transitions[0]
    assert "proceed_goal" in line
    assert "goal_3" in line
    assert "\u2192 running" in line
    assert state.last_event["action_taken"] == "proceed_goal"


def test_transitions_ring_buffer_limit():
    state = OodaLivingState()
    for i in range(20):
        event = FakeEvent()
        state.update_after_observe(event)
        state.update_after_orient(FakeOrientedEvent(event=event, assessment={}), {})
        state.update_after_act(FakeAction(type="no_action"))

    assert len(state.transitions) == 12  # maxlen


# ── Goals and user concern snapshot ───────────────────────────────────

def test_update_goals():
    state = OodaLivingState()
    goals = [
        {"goal_id": "goal_1", "name": "Test goal", "status": "ready"},
        {"goal_id": "goal_2", "name": "Another", "status": "completed"},
    ]
    state.update_goals(goals, foregrounded_id="goal_1")

    assert len(state.goal_field) == 2
    assert state.foregrounded_goal_id == "goal_1"
    assert state.goal_field[0]["label"] == "Test goal"


def test_update_user_concerns_snapshot():
    state = OodaLivingState()
    concerns = [
        {"status": "ongoing"},
        {"status": "ongoing"},
        {"status": "dormant"},
    ]
    state.update_user_concerns_snapshot(concerns)

    assert state.user_concern_snapshot == {"ongoing": 2, "dormant": 1}


# ── Serialization round-trip ──────────────────────────────────────────

def test_serialization_round_trip():
    state = OodaLivingState()
    event = FakeEvent(content="test", goal_id="goal_5")
    state.update_after_observe(event)
    state.update_after_orient(
        FakeOrientedEvent(event=event, assessment={"notes": "concerns: homeostasis=moderate"}),
        {"homeostasis": 0.3, "attend_to_user": 0.1},
    )
    state.update_after_act(FakeAction(type="chat_response"))
    state.update_goals([{"goal_id": "g1", "name": "G", "status": "ready"}], "g1")
    state.update_user_concerns_snapshot([{"status": "ongoing"}])

    data = state.to_dict()
    restored = OodaLivingState()
    restored.from_dict(data)

    assert restored.concern_activations == state.concern_activations
    assert restored.foregrounded_goal_id == "g1"
    assert len(restored.transitions) == 1
    assert restored.user_concern_snapshot == {"ongoing": 1}
    assert len(restored._activation_history) == 2


# ── Persistence debouncing ────────────────────────────────────────────

def test_maybe_persist_respects_debounce():
    state = OodaLivingState()
    state._dirty = True
    writes = []

    def mock_write(name, content):
        writes.append((name, content))

    # First persist should work
    state.maybe_persist(mock_write)
    assert len(writes) == 1
    assert not state._dirty

    # Immediate second should be skipped (not dirty)
    state._dirty = True
    state.maybe_persist(mock_write)
    # Debounce interval not elapsed
    assert len(writes) == 1
    assert state._dirty  # still dirty


def test_maybe_persist_skips_clean():
    state = OodaLivingState()
    state._dirty = False
    writes = []

    def mock_write(name, content):
        writes.append(1)

    state.maybe_persist(mock_write)
    assert len(writes) == 0
