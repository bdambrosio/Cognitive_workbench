"""Unit tests for disposition shadow-state capture and render
(src/chat/disposition.py, docs/learned-disposition-design.md step 1).

Same discipline as test_concern_dynamics: object.__new__(ChatLoop) with
only the attributes under test, a stubbed backend, a throwaway scratch
world — never live world state.
"""

import json
import os
import shutil
import sys
import threading
import uuid
from datetime import datetime, timedelta, timezone
from pathlib import Path

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from chat.chat_loop import ChatLoop, _AGENT_CONCERN_SERVICE_FULL
from chat.disposition import (
    derive_act_history,
    derive_precedent_density,
    derive_user_rhythm,
    load_states,
    render_disposition_state,
)
from infospace_resource_manager import InfospaceResourceManager


class StubBackend:
    def __init__(self, response):
        self.response = response
        self.calls = 0

    def chat(self, messages, **kwargs):
        self.calls += 1
        return self.response

    server = "local"
    model = "stub"


class StubStore:
    def __init__(self, turns):
        self.turns = turns

    def get_recent_turns(self, entity, limit=20, scope="all"):
        return self.turns[-limit:]


@pytest.fixture
def loop(tmp_path):
    world = f"pytest_scratch_{uuid.uuid4().hex[:8]}"
    mgr = InfospaceResourceManager(world, world_config={"world_name": world})
    inst = object.__new__(ChatLoop)
    inst.character_name = "Tester"
    inst.resource_manager = mgr
    inst._faiss_lock = threading.Lock()
    inst._agent_concerns_collection_id = "Collection_dummy"
    mgr.resource_registry["Collection_uc"] = {
        "name": "Collection_uc",
        "type": mgr.resource_types.Collection,
        "location": (0, 0),
        "description": "test user_concerns",
        "remove_on_take": False,
        "properties": {"content": [], "format": "list",
                       "collection_name": "user_concerns",
                       "kind": "user_concerns"},
    }
    inst._user_concerns_collection_id = "Collection_uc"
    inst.backend = StubBackend('{"verdict": "fire"}')
    inst._autonomy_log_path = lambda: tmp_path / "autonomy.jsonl"
    inst._disposition_state_path = lambda: tmp_path / "disposition_state.jsonl"
    inst._load_pending_fire_outcomes = lambda: [{"fire_id": "a"}, {"fire_id": "b"}]
    inst._get_threads = lambda statuses=("active",): []
    now = datetime.now()
    inst.store = StubStore([
        {"source": "User", "direction": "in", "text": "how's the PV controller?",
         "timestamp": (now - timedelta(minutes=47)).isoformat()},
        {"source": "Tester", "direction": "out", "text": "steady since Tuesday.",
         "timestamp": (now - timedelta(minutes=46)).isoformat()},
    ])
    inst._companion_state = {"User": "wants terse answers today"}
    inst._discourse_state = {"User": "debugging the inverter logs"}
    yield inst
    shutil.rmtree(Path(__file__).parent.parent / "scenarios" / world,
                  ignore_errors=True)


def make_concern(loop, activation=0.8, extra=None):
    props = {
        "kind": "agent_concern",
        "status": "active",
        "activation": activation,
        "instruction": "check the thing",
        "rhythm_hours": 24,
        "exclude_from_index": True,
    }
    props.update(extra or {})
    ok, nid, err, _ = loop.resource_manager.create_note(
        "Tester", "concern text", "text", "pytest", "", "", props)
    assert ok, err
    return nid


def rows(loop):
    return load_states(loop._disposition_state_path())


# ── capture ────────────────────────────────────────────────────────────

def test_triage_writes_exactly_one_state_row_with_verdict(loop):
    nid = make_concern(loop)
    assert loop._triage_fire_candidate(nid, "concern text", "instr") == "fire"

    got = rows(loop)
    assert len(got) == 1
    r = got[0]
    assert r["verdict"] == "fire"
    assert r["schema_version"] == 1
    assert r["concern"]["id"] == nid
    assert r["concern"]["instruction"] == "instr"
    assert r["situation"]["pending_unjudged_fires"] == 2
    assert r["situation"]["user_last_turn_text"] == "how's the PV controller?"
    # ~47 minutes, allowing for test runtime.
    assert 2700 < r["situation"]["user_last_turn_age_s"] < 2900
    assert r["situation"]["companion_state"] == "wants terse answers today"
    assert [t["direction"] for t in r["situation"]["recent_exchange"]] == ["in", "out"]


def test_snapshot_precedes_verdict_mutation(loop):
    """A 'reset' verdict decrements activation; the row must hold the
    pre-decision value, and the prior (not the new) defer reason."""
    nid = make_concern(loop, activation=0.8, extra={"triage_reason": "old reason"})
    loop.backend = StubBackend('{"verdict": "reset", "reason": "nothing needed"}')

    assert loop._triage_fire_candidate(nid, "concern text", "instr") == "reset"

    props = loop.resource_manager.get_resource(nid)["properties"]
    assert props["activation"] == pytest.approx(0.8 - _AGENT_CONCERN_SERVICE_FULL)
    r = rows(loop)[0]
    assert r["concern"]["activation"] == pytest.approx(0.8)
    assert r["concern"]["prior_defer_reason"] == "old reason"
    assert r["verdict_reason"] == "nothing needed"


def test_cached_defer_writes_no_row(loop):
    """Only real evaluations get a row — cached defers would flood the
    set with near-duplicate states carrying no fresh decision."""
    nid = make_concern(loop, extra={
        "triage_verdict": "defer",
        "triage_at": datetime.now(timezone.utc).isoformat(),
        "triage_reason": "nothing new",
    })
    assert loop._triage_fire_candidate(nid, "concern text", "instr") == "defer"
    assert loop.backend.calls == 0
    assert rows(loop) == []


def test_capture_failure_does_not_disturb_triage(loop):
    """Capture is read-only telemetry: if every source it reads is gone,
    triage still returns its verdict."""
    nid = make_concern(loop)
    loop.store = None
    loop._load_pending_fire_outcomes = lambda: (_ for _ in ()).throw(RuntimeError("boom"))
    assert loop._triage_fire_candidate(nid, "concern text", "instr") == "fire"
    assert rows(loop) == []


# ── derivation ─────────────────────────────────────────────────────────

def _ev(event, ago_h, **kw):
    d = {"event": event,
         "ts": (datetime.now(timezone.utc) - timedelta(hours=ago_h)).isoformat()}
    d.update(kw)
    return d


def test_derive_act_history_counts_and_streak():
    now = datetime.now(timezone.utc)
    events = [
        _ev("fire", 200, concern_id="C1"),
        _ev("fire_outcome", 199, concern_id="C1", outcome="helped"),
        _ev("fire", 100, concern_id="C1"),
        _ev("fire", 50, concern_id="C2"),
        _ev("fire", 10, concern_id="C1"),
        _ev("fire_outcome", 9, concern_id="C1", outcome="ignored"),
        _ev("fire", 5, concern_id="C2"),
        _ev("fire_outcome", 4, concern_id="C2", outcome="ignored"),
        _ev("deferred", 3, concern_id="C1"),
        _ev("triage", 2, concern_id="C1", verdict="defer"),
    ]
    d = derive_act_history(events, "C1", now)

    assert d["fires_24h"] == 2          # C1@10h, C2@5h
    assert d["fires_24h_concern"] == 1
    assert d["fires_7d_concern"] == 2   # 100h and 10h
    assert d["deferred_24h"] == 1
    assert d["consecutive_ignored"] == 2
    assert d["outcomes_concern"] == ["helped", "ignored"]
    assert d["triage_verdicts_concern"] == ["defer"]
    assert d["hit_rate_n"] == 3
    assert d["hit_rate_good"] == 1
    # Fires after the last 'helped' for C1: the 100h and 10h fires.
    assert d["fires_since_progress"] == 2


def test_derive_act_history_is_causal():
    """Events at or after the reference instant must not be counted."""
    now = datetime.now(timezone.utc)
    events = [_ev("fire", 1, concern_id="C1"), _ev("fire", -1, concern_id="C1")]
    assert derive_act_history(events, "C1", now)["fires_24h"] == 1


def test_derive_user_rhythm_needs_history():
    now = datetime.now(timezone.utc)
    thin = [{"direction": "in", "timestamp": (now - timedelta(hours=i)).isoformat()}
            for i in range(5)]
    assert derive_user_rhythm(thin, now)["activity_level"] is None

    dense = [{"direction": "in",
              "timestamp": (now - timedelta(minutes=15 * i)).isoformat()}
             for i in range(60)]
    d = derive_user_rhythm(dense, now)
    assert d["activity_level"] in ("high", "medium", "low")
    assert d["typical_gap_s"] is not None


def test_precedent_density_without_embedder_is_zero():
    r = {"ts": datetime.now(timezone.utc).isoformat(), "concern": {"text": "x"}}
    assert derive_precedent_density([r], r, None) == {"n_precedents": 0, "n_judged": 0}


# ── render ─────────────────────────────────────────────────────────────

def test_render_is_honest_about_missing_history(loop):
    nid = make_concern(loop)
    loop._triage_fire_candidate(nid, "concern text", "instr")
    text = render_disposition_state(rows(loop)[0])

    assert "== concern ==" in text and "== situation ==" in text
    assert "states like this: insufficient history" in text
    assert "recent read on this user: nothing judged yet" in text
    assert "user usually active at this hour: unknown" in text
    assert "2 fires still unacknowledged" in text
    assert 'user last said: "how\'s the PV controller?"' in text


def test_render_uses_derived_history(loop):
    nid = make_concern(loop)
    loop._triage_fire_candidate(nid, "concern text", "instr")
    text = render_disposition_state(rows(loop)[0], {
        "activity_level": "high",
        "typical_gap_s": 900,
        "fires_24h": 4,
        "fires_24h_baseline": 6.0,
        "outcomes_recent": ["helped", "ignored", "ignored"],
        "outcomes_concern": ["ignored", "helped"],
        "fires_since_progress": 3,
        "hit_rate_n": 8,
        "hit_rate_good": 5,
        "n_precedents": 6,
        "n_judged": 2,
    })

    assert "user usually active at this hour: high" in text
    assert "typical gap at this hour: 15m" in text
    assert "my last 24h: 4 fires (typical 6.0) · last 3 landed: helped, ignored, ignored" in text
    assert "fires since last real progress: 3" in text
    assert "outcomes for this concern: ignored, helped" in text
    assert "recent read on this user: last 8 judged acts — 5 landed well" in text
    assert "states like this: 6 precedents in 90d, 2 judged" in text
