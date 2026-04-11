"""Unit tests for bench/harvester.py::resolve_goal_handle.

Uses a FakeSession that yields canned Zenoh-shaped replies. No launcher
required.
"""
import json
import os
import sys
from types import SimpleNamespace

import pytest

# bench/ and src/ on sys.path. src/ is needed because harvester imports
# utils.zenoh_utils on module load.
_HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(_HERE, "..", "bench"))
sys.path.insert(0, os.path.join(_HERE, "..", "src"))

pytest.importorskip("zenoh")  # harvester can't load without it

import harvester  # noqa: E402


def _reply(goals):
    """Build a SimpleNamespace that quacks like a Zenoh reply with a
    payload encoding {"success": True, "goals": [...]}.
    """
    payload_bytes = json.dumps({"success": True, "goals": goals}).encode("utf-8")
    return SimpleNamespace(
        ok=SimpleNamespace(
            payload=SimpleNamespace(to_bytes=lambda b=payload_bytes: b)
        )
    )


class FakeSession:
    def __init__(self, replies):
        self._replies = replies

    def get(self, key, timeout=3.0):
        return iter(self._replies)


def test_resolve_by_goal_id_returns_record():
    session = FakeSession([_reply([
        {"goal_id": "goal_17", "name": "G01"},
        {"goal_id": "goal_18", "name": "G02"},
    ])])
    record = harvester.resolve_goal_handle(session, "Jill", "goal_18")
    assert record is not None
    assert record["goal_id"] == "goal_18"
    assert record["name"] == "G02"


def test_resolve_by_name_returns_record():
    session = FakeSession([_reply([
        {"goal_id": "goal_17", "name": "G01"},
        {"goal_id": "goal_18", "name": "G02"},
    ])])
    record = harvester.resolve_goal_handle(session, "Jill", "G01")
    assert record is not None
    assert record["goal_id"] == "goal_17"


def test_resolve_ambiguous_name_raises_with_candidates():
    session = FakeSession([_reply([
        {"goal_id": "goal_5", "name": "G01"},
        {"goal_id": "goal_17", "name": "G01"},
    ])])
    with pytest.raises(ValueError) as exc_info:
        harvester.resolve_goal_handle(session, "Jill", "G01")
    msg = str(exc_info.value)
    assert "ambiguous" in msg.lower()
    assert "goal_5" in msg
    assert "goal_17" in msg


def test_resolve_unknown_returns_none():
    session = FakeSession([_reply([
        {"goal_id": "goal_17", "name": "G01"},
    ])])
    record = harvester.resolve_goal_handle(session, "Jill", "G99")
    assert record is None
