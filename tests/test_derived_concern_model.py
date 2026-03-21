"""Unit tests for DerivedConcernModel."""

import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from derived_concern_model import (
    DerivedConcernModel,
    ORIGIN_VALUES,
    STATUS_VALUES,
    _MAX_ACTIVE_CONCERNS,
)


# ── Stub helpers ──────────────────────────────────────────────────────

class FakeResourceManager:
    def __init__(self):
        self.named_notes = {}
        self.resource_registry = {}

    def update_note_content(self, note_id, content):
        self.resource_registry[note_id] = {"content": content}
        return True, None

    def get_resource(self, rid):
        return self.resource_registry.get(rid)


class FakeExecutor:
    def __init__(self):
        self._notes = {}
        self._counter = 0

    def execute_action(self, action):
        if action["type"] == "load":
            target = action["target"]
            if target in self._notes:
                return {"status": "success", "resource_id": target}
            return {"status": "failed"}
        if action["type"] == "create-note":
            self._counter += 1
            rid = f"note_{self._counter}"
            self._notes[rid] = action.get("value", "")
            return {"status": "success", "resource_id": rid}
        if action["type"] == "persist":
            return {"status": "success"}
        return {"status": "failed"}

    def _get_content(self, resource_id):
        return self._notes.get(resource_id, "")


# ── Patch application tests ──────────────────────────────────────────

def test_apply_surface_concern():
    rm = FakeResourceManager()
    model = DerivedConcernModel(rm, "test", None, FakeExecutor())

    patch = {
        "op": "surface_concern",
        "new_concern": {
            "concern_label": "monitor solar output",
            "concern_description": "User has ongoing concern about solar panels",
            "weight": 0.7,
            "origin": "user_concern_derived",
            "parent_user_concern_id": "concern_1",
        },
        "why_this_surface": "test",
    }
    model._apply_patch(patch, "test_ref")

    assert len(model.concerns) == 1
    c = model.concerns[0]
    assert c["concern_id"] == "dconcern_1"
    assert c["concern_label"] == "monitor solar output"
    assert c["status"] == "surfaced"
    assert c["origin"] == "user_concern_derived"
    assert c["parent_user_concern_id"] == "concern_1"
    assert c["weight"] == 0.7
    assert c["evidence_refs"] == ["test_ref"]


def test_apply_activate_concern():
    rm = FakeResourceManager()
    model = DerivedConcernModel(rm, "test", None, FakeExecutor())

    # Surface first
    model.concerns = [{
        "concern_id": "dconcern_1",
        "concern_label": "test",
        "concern_description": "test desc",
        "weight": 0.5,
        "origin": "orientation_derived",
        "parent_user_concern_id": None,
        "status": "surfaced",
        "status_rationale": "",
        "recency": "2024-01-01",
        "touch_count": 1,
        "evidence_refs": [],
        "history_summary": "",
    }]

    patch = {
        "op": "activate_concern",
        "concern_id": "dconcern_1",
        "field_updates": {"weight": 0.8, "status_rationale": "committing"},
        "why_this_activate": "test",
    }
    model._apply_patch(patch, "ref2")

    assert model.concerns[0]["status"] == "active"
    assert model.concerns[0]["weight"] == 0.8
    assert model.concerns[0]["touch_count"] == 2


def test_apply_resolve_concern():
    rm = FakeResourceManager()
    model = DerivedConcernModel(rm, "test", None, FakeExecutor())
    model.concerns = [{
        "concern_id": "dconcern_1",
        "status": "active",
        "touch_count": 2,
        "evidence_refs": [],
        "concern_label": "x",
        "concern_description": "x",
        "weight": 0.5,
        "origin": "orientation_derived",
        "parent_user_concern_id": None,
        "status_rationale": "",
        "recency": "",
        "history_summary": "",
    }]

    patch = {
        "op": "resolve_concern",
        "concern_id": "dconcern_1",
        "field_updates": {"status_rationale": "done"},
        "why_this_resolve": "goal completed it",
    }
    model._apply_patch(patch, "ref3")

    assert model.concerns[0]["status"] == "resolved"


def test_apply_abandon_concern():
    rm = FakeResourceManager()
    model = DerivedConcernModel(rm, "test", None, FakeExecutor())
    model.concerns = [{
        "concern_id": "dconcern_1",
        "status": "surfaced",
        "touch_count": 1,
        "evidence_refs": [],
        "concern_label": "x",
        "concern_description": "x",
        "weight": 0.3,
        "origin": "orientation_derived",
        "parent_user_concern_id": None,
        "status_rationale": "",
        "recency": "",
        "history_summary": "",
    }]

    patch = {
        "op": "abandon_concern",
        "concern_id": "dconcern_1",
        "field_updates": {"status_rationale": "no longer relevant"},
        "why_this_abandon": "test",
    }
    model._apply_patch(patch, "ref4")

    assert model.concerns[0]["status"] == "abandoned"


def test_surface_respects_capacity():
    rm = FakeResourceManager()
    model = DerivedConcernModel(rm, "test", None, FakeExecutor())

    # Fill to capacity with active concerns
    for i in range(_MAX_ACTIVE_CONCERNS):
        model.concerns.append({
            "concern_id": f"dconcern_{i+1}",
            "status": "active",
            "concern_label": f"c{i}",
            "concern_description": "",
            "weight": 0.5,
            "origin": "orientation_derived",
            "parent_user_concern_id": None,
            "status_rationale": "",
            "recency": "",
            "touch_count": 1,
            "evidence_refs": [],
            "history_summary": "",
        })

    # Try to surface one more — should be rejected
    patch = {
        "op": "surface_concern",
        "new_concern": {
            "concern_label": "overflow",
            "concern_description": "should not be added",
            "weight": 0.5,
            "origin": "orientation_derived",
        },
        "why_this_surface": "test",
    }
    model._apply_patch(patch, "ref")

    assert len(model.concerns) == _MAX_ACTIVE_CONCERNS  # not added


def test_no_change_patch():
    rm = FakeResourceManager()
    model = DerivedConcernModel(rm, "test", None, FakeExecutor())
    model._apply_patch({"op": "no_change", "why_no_change": "nothing to do"}, "ref")
    assert len(model.concerns) == 0


def test_invalid_status_rejected():
    rm = FakeResourceManager()
    model = DerivedConcernModel(rm, "test", None, FakeExecutor())
    model.concerns = [{
        "concern_id": "dconcern_1",
        "status": "active",
        "touch_count": 1,
        "evidence_refs": [],
        "concern_label": "x",
        "concern_description": "x",
        "weight": 0.5,
        "origin": "orientation_derived",
        "parent_user_concern_id": None,
        "status_rationale": "",
        "recency": "",
        "history_summary": "",
    }]

    # Try to set an invalid status via merge
    model._merge_updates(model.concerns[0], {"status": "bogus"}, "ref")
    assert model.concerns[0]["status"] == "active"  # unchanged


# ── Rate limiting ─────────────────────────────────────────────────────

def test_idle_tick_rate_limiting():
    rm = FakeResourceManager()
    calls = []

    def fake_llm(*args, **kwargs):
        calls.append(1)

    model = DerivedConcernModel(rm, "test", fake_llm, FakeExecutor())
    user_concerns = [{"concern_id": "c1", "concern_label": "test", "status": "ongoing", "weight": 0.5}]

    # First call should trigger (sets _last_idle_update)
    # But it will fail because fake_llm doesn't return proper result
    model.update_from_idle_tick(None, user_concerns)

    # Second immediate call should be rate-limited
    model.update_from_idle_tick(None, user_concerns)

    # Only 1 LLM call should have been attempted
    assert len(calls) == 1


def test_idle_tick_skips_without_user_concerns():
    rm = FakeResourceManager()
    calls = []

    def fake_llm(*args, **kwargs):
        calls.append(1)

    model = DerivedConcernModel(rm, "test", fake_llm, FakeExecutor())
    model.update_from_idle_tick(None, [])  # empty user concerns

    assert len(calls) == 0


# ── Evaluator format ─────────────────────────────────────────────────

def test_get_concerns_for_evaluator_format():
    rm = FakeResourceManager()
    model = DerivedConcernModel(rm, "test", None, FakeExecutor())
    model.concerns = [
        {
            "concern_id": "dconcern_1",
            "concern_label": "monitor solar",
            "concern_description": "Track solar output drops",
            "status": "active",
            "weight": 0.8,
            "origin": "user_concern_derived",
        },
        {
            "concern_id": "dconcern_2",
            "concern_label": "resolved thing",
            "concern_description": "Done",
            "status": "resolved",
            "weight": 0.9,
        },
        {
            "concern_id": "dconcern_3",
            "concern_label": "surfaced thing",
            "concern_description": "New",
            "status": "surfaced",
            "weight": 0.3,
            "origin": "orientation_derived",
        },
    ]

    result = model.get_concerns_for_evaluator()

    # Should only include active+surfaced, sorted by weight desc
    assert len(result) == 2
    assert result[0]["id"] == "dconcern_1"  # higher weight
    assert result[1]["id"] == "dconcern_3"
    # Check format matches DEFAULT_CHARACTER_CONCERNS
    assert "id" in result[0]
    assert "label" in result[0]
    assert "description" in result[0]
    assert "Agent-derived concern" in result[0]["description"]


def test_get_concerns_for_evaluator_max_count():
    rm = FakeResourceManager()
    model = DerivedConcernModel(rm, "test", None, FakeExecutor())
    for i in range(10):
        model.concerns.append({
            "concern_id": f"dconcern_{i+1}",
            "concern_label": f"c{i}",
            "concern_description": f"desc{i}",
            "status": "active",
            "weight": 0.5 + i * 0.01,
            "origin": "orientation_derived",
        })

    result = model.get_concerns_for_evaluator(max_count=3)
    assert len(result) == 3


# ── get_concerns filtering ───────────────────────────────────────────

def test_get_concerns_active_only():
    rm = FakeResourceManager()
    model = DerivedConcernModel(rm, "test", None, FakeExecutor())
    model.concerns = [
        {"concern_id": "d1", "status": "active"},
        {"concern_id": "d2", "status": "resolved"},
        {"concern_id": "d3", "status": "surfaced"},
        {"concern_id": "d4", "status": "abandoned"},
    ]

    active = model.get_concerns(active_only=True)
    assert len(active) == 2
    assert {c["concern_id"] for c in active} == {"d1", "d3"}

    all_c = model.get_concerns(active_only=False)
    assert len(all_c) == 4


# ── Parse patch ───────────────────────────────────────────────────────

def test_parse_patch_valid():
    rm = FakeResourceManager()
    model = DerivedConcernModel(rm, "test", None, FakeExecutor())

    patch = model._parse_patch('{"op": "no_change", "why_no_change": "ok"}')
    assert patch is not None
    assert patch["op"] == "no_change"


def test_parse_patch_dict_input():
    rm = FakeResourceManager()
    model = DerivedConcernModel(rm, "test", None, FakeExecutor())

    patch = model._parse_patch({"op": "surface_concern", "new_concern": {}})
    assert patch is not None


def test_parse_patch_invalid_op():
    rm = FakeResourceManager()
    model = DerivedConcernModel(rm, "test", None, FakeExecutor())

    patch = model._parse_patch('{"op": "bogus_op"}')
    assert patch is None


def test_parse_patch_strips_markdown():
    rm = FakeResourceManager()
    model = DerivedConcernModel(rm, "test", None, FakeExecutor())

    text = '```json\n{"op": "no_change", "why_no_change": "x"}\n```'
    patch = model._parse_patch(text)
    assert patch is not None
    assert patch["op"] == "no_change"
