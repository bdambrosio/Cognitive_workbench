"""Unit tests for fire-outcome capture (docs/fire-outcome-capture.md,
phase 1): pending-registry registration (incl. the silent 'unobservable'
path), per-user-turn aging + expiry to 'unobserved', reflection stage-6
judgment parsing, fire_id join integrity, and empty-registry prompt
stability.

Uses object.__new__(ChatLoop) with only the attributes the methods under
test touch, a stubbed LLM, and a throwaway scratch world — never live
world state.
"""

import json
import os
import shutil
import sys
import threading
import uuid
from datetime import datetime, timedelta, timezone
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from chat.chat_loop import (
    ChatLoop,
    _CONCERN_INSTRUCTION_NARROWNESS_RULE,
    _FIRE_OUTCOME_DIGEST_CHARS,
    _FIRE_OUTCOME_EXPIRY_DAYS,
    _FIRE_OUTCOME_EXPIRY_TURNS,
    _FIRE_OUTCOME_MAX_PER_REFLECTION,
)
from chat.concerns import _FIRE_DIGEST_MAX_ITEMS
from infospace_resource_manager import InfospaceResourceManager


class StubBackend:
    """Counts calls; returns a queue of canned responses (last one sticks)."""

    def __init__(self, responses):
        self.responses = list(responses)
        self.calls = 0

    def chat(self, messages, **kwargs):
        self.calls += 1
        if len(self.responses) > 1:
            return self.responses.pop(0)
        return self.responses[0]


@pytest.fixture
def loop(tmp_path):
    """Minimal ChatLoop instance over a scratch world."""
    world = f"pytest_scratch_{uuid.uuid4().hex[:8]}"
    mgr = InfospaceResourceManager(world, world_config={"world_name": world})
    inst = object.__new__(ChatLoop)
    inst.character_name = "Tester"
    inst.resource_manager = mgr
    inst._faiss_lock = threading.Lock()
    inst._agent_concerns_collection_id = "Collection_dummy"
    inst.backend = StubBackend(["{}"])
    inst._autonomy_log_path = lambda: tmp_path / "autonomy.jsonl"
    inst._pending_fire_outcomes_path = (
        lambda: tmp_path / "pending_fire_outcomes.json")
    yield inst
    scenario_dir = Path(__file__).parent.parent / "scenarios" / world
    shutil.rmtree(scenario_dir, ignore_errors=True)


def make_concern(loop, activation=0.8, instruction="check the thing",
                 extra=None):
    props = {
        "kind": "agent_concern",
        "status": "active",
        "activation": activation,
        "instruction": instruction,
        "rhythm_hours": 24,
        "exclude_from_index": True,
    }
    props.update(extra or {})
    ok, nid, err, _ = loop.resource_manager.create_note(
        "Tester", "concern text", "text", "pytest", "", "", props)
    assert ok, err
    return nid


def read_events(tmp_path):
    p = tmp_path / "autonomy.jsonl"
    if not p.exists() or not p.read_text().strip():
        return []
    return [json.loads(l) for l in p.read_text().strip().splitlines()]


def read_pending(loop):
    return loop._load_pending_fire_outcomes()


# ── registration ───────────────────────────────────────────────────────

def test_registration_creates_pending_record(loop, tmp_path):
    nid = make_concern(loop)
    loop._register_fire_outcome("fid-1", nid, "respond",
                                "I checked the thing; all healthy.", False)

    recs = read_pending(loop)
    assert len(recs) == 1
    rec = recs[0]
    assert rec["fire_id"] == "fid-1"
    assert rec["concern_id"] == nid
    assert rec["concern_text"] == "concern text"
    assert rec["exit_reason"] == "respond"
    assert rec["reply_digest"] == "I checked the thing; all healthy."
    assert rec["intentionally_silent"] is False
    assert rec["user_turns_since"] == 0
    datetime.fromisoformat(rec["fired_at"])  # parseable timestamp
    # No outcome event yet — the fire is pending, not resolved.
    assert read_events(tmp_path) == []


def test_reply_digest_capped(loop):
    nid = make_concern(loop)
    loop._register_fire_outcome("fid-1", nid, "max_iters", "x" * 1000, False)
    rec = read_pending(loop)[0]
    assert len(rec["reply_digest"]) == _FIRE_OUTCOME_DIGEST_CHARS
    assert rec["exit_reason"] == "max_iters"


def test_silent_fire_unobservable_never_registered(loop, tmp_path):
    nid = make_concern(loop)
    loop._register_fire_outcome("fid-s", nid, "respond", "(no reply)", True)

    # Structurally unobservable: resolved at registration, never pending.
    assert read_pending(loop) == []
    events = read_events(tmp_path)
    assert len(events) == 1
    ev = events[0]
    assert ev["event"] == "fire_outcome"
    assert ev["fire_id"] == "fid-s"
    assert ev["concern_id"] == nid
    assert ev["outcome"] == "unobservable"
    assert ev["valence"] is None
    assert ev["user_impact"] is None
    # Process fields still present.
    assert ev["exit_reason"] == "respond"
    assert ev["latency_turns"] == 0
    assert "observed_at" in ev


# ── aging + expiry ─────────────────────────────────────────────────────

def test_aging_increments_then_turn_expiry(loop, tmp_path):
    nid = make_concern(loop)
    loop._register_fire_outcome("fid-1", nid, "respond", "reply", False)

    for expected in range(1, _FIRE_OUTCOME_EXPIRY_TURNS):
        loop._age_pending_fire_outcomes()
        recs = read_pending(loop)
        assert len(recs) == 1
        assert recs[0]["user_turns_since"] == expected
        assert read_events(tmp_path) == []

    # The turn that reaches the cap expires the record to 'unobserved'.
    loop._age_pending_fire_outcomes()
    assert read_pending(loop) == []
    events = read_events(tmp_path)
    assert len(events) == 1
    ev = events[0]
    assert ev["event"] == "fire_outcome"
    assert ev["fire_id"] == "fid-1"
    assert ev["outcome"] == "unobserved"
    assert ev["valence"] is None
    assert ev["user_impact"] is None
    assert ev["latency_turns"] == _FIRE_OUTCOME_EXPIRY_TURNS


def test_aging_wall_clock_expiry(loop, tmp_path):
    nid = make_concern(loop)
    loop._register_fire_outcome("fid-old", nid, "respond", "reply", False)
    # Backdate fired_at past the wall-clock cap.
    recs = read_pending(loop)
    recs[0]["fired_at"] = (
        datetime.now(timezone.utc)
        - timedelta(days=_FIRE_OUTCOME_EXPIRY_DAYS + 1)).isoformat()
    loop._save_pending_fire_outcomes(recs)

    loop._age_pending_fire_outcomes()

    assert read_pending(loop) == []
    events = read_events(tmp_path)
    assert len(events) == 1
    assert events[0]["outcome"] == "unobserved"
    assert events[0]["latency_turns"] == 1  # expired on the first user turn


# ── stage-6 judgment application ───────────────────────────────────────

def register_fires(loop, n, prefix="fid"):
    nid = make_concern(loop)
    fids = [f"{prefix}-{i}" for i in range(n)]
    for fid in fids:
        loop._register_fire_outcome(fid, nid, "respond", f"reply {fid}", False)
    return nid, fids


def test_judgment_valid_resolves_and_writes_event(loop, tmp_path):
    nid, (f1, f2) = register_fires(loop, 2)
    loop._age_pending_fire_outcomes()  # latency_turns = 1
    pending = read_pending(loop)

    resolved = loop._apply_fire_outcome_judgments([
        {"fire_id": f1, "outcome": "helped", "valence": 0.7,
         "user_impact": -0.2, "evidence": "user said the alert was useful"},
    ], pending)

    assert resolved == [f1]
    remaining = read_pending(loop)
    assert [r["fire_id"] for r in remaining] == [f2]
    events = read_events(tmp_path)
    assert len(events) == 1
    ev = events[0]
    assert ev["event"] == "fire_outcome"
    assert ev["fire_id"] == f1
    assert ev["concern_id"] == nid
    assert ev["concern_text"] == "concern text"
    assert ev["exit_reason"] == "respond"
    assert ev["outcome"] == "helped"
    assert ev["valence"] == pytest.approx(0.7)
    assert ev["user_impact"] == pytest.approx(-0.2)  # ledger-relative signs
    assert ev["evidence"] == "user said the alert was useful"
    assert ev["latency_turns"] == 1
    assert "observed_at" in ev


def test_judgment_malformed_skipped_and_values_clamped(loop, tmp_path):
    _nid, (f1, f2, f3) = register_fires(loop, 3)
    pending = read_pending(loop)

    # Non-list payloads are ignored outright.
    assert loop._apply_fire_outcome_judgments("not a list", pending) == []
    assert loop._apply_fire_outcome_judgments(None, pending) == []

    resolved = loop._apply_fire_outcome_judgments([
        "not a dict",                                     # skipped
        {"fire_id": "fid-unknown", "outcome": "helped"},  # unknown → skipped
        {"fire_id": f1, "outcome": "amazing"},            # bad vocab → skipped
        {"fire_id": f2, "outcome": "HINDERED",            # case-normalized
         "valence": 5, "user_impact": -3,                 # clamped to [-1,1]
         "evidence": "e" * 500},                          # capped at 200
        {"fire_id": f3, "outcome": "ignored",
         "valence": "not a number"},                      # unparseable → null
    ], pending)

    assert resolved == [f2, f3]
    # Skipped entries stay pending — they age out naturally.
    assert [r["fire_id"] for r in read_pending(loop)] == [f1]
    ev2, ev3 = read_events(tmp_path)
    assert ev2["outcome"] == "hindered"
    assert ev2["valence"] == 1.0
    assert ev2["user_impact"] == -1.0
    assert len(ev2["evidence"]) == 200
    assert ev3["outcome"] == "ignored"
    assert ev3["valence"] is None


def test_judgment_over_cap(loop, tmp_path):
    _nid, fids = register_fires(loop, _FIRE_OUTCOME_MAX_PER_REFLECTION + 2)
    pending = read_pending(loop)

    resolved = loop._apply_fire_outcome_judgments(
        [{"fire_id": f, "outcome": "neutral"} for f in fids], pending)

    assert len(resolved) == _FIRE_OUTCOME_MAX_PER_REFLECTION
    assert len(read_events(tmp_path)) == _FIRE_OUTCOME_MAX_PER_REFLECTION
    # Surplus judgments stay pending for a later reflection.
    assert len(read_pending(loop)) == 2


# ── reflection stage 6: prompt assembly + payload routing ──────────────

_EMPTY_PAYLOAD = {
    "frame": "none", "memories": [], "user_concerns": [],
    "user_concerns_updated": [], "user_concerns_closed": [],
    "agent_concerns": [],
}


def wire_reflection(loop, payload, convo_text="thanks, that was useful"):
    """Stub the attributes _reflect_and_remember touches around the LLM
    call; capture the exact messages sent. Returns the capture dict."""
    loop._memories_collection_id = "Collection_mem"
    loop._companion_state = {}
    loop._build_dialog = (
        lambda source, limit=4: [{"source": "User", "text": convo_text}])
    loop._top_active_agent_concerns = lambda n=10: []
    loop._top_active_user_concerns = lambda n=10: []
    loop._recall = lambda *a, **k: []
    captured = {}

    def fake_generate(messages, **kwargs):
        captured["system"] = messages[0]["content"]
        captured["user"] = messages[1]["content"]
        return SimpleNamespace(success=True, text=payload, error=None)

    loop._llm_generate = fake_generate
    return captured


def test_reflection_prompt_stable_with_empty_registry(loop):
    captured = wire_reflection(loop, dict(_EMPTY_PAYLOAD))

    loop._reflect_and_remember("User")

    # Byte-identical to pre-capture behavior (KV-cache stability; existing
    # benchmarks unaffected): no stage-6 rule, no pending section, no
    # fire_outcomes key.
    assert captured["system"] == ChatLoop._REFLECT_SYS.format(
        character="Tester", entity="User",
        narrowness_rule=_CONCERN_INSTRUCTION_NARROWNESS_RULE)
    assert captured["user"] == (
        "## Latest exchange\nUser: thanks, that was useful\n\n"
        "Return the JSON object now (keys: frame, memories, user_concerns, "
        "user_concerns_updated, user_concerns_closed, agent_concerns, "
        "agent_concerns_closed, obligations_update). All "
        "lists empty if frame≠none or nothing qualifies.")


def test_reflection_stage6_section_and_judgment(loop, tmp_path):
    nid = make_concern(loop)
    loop._register_fire_outcome("fid-1", nid, "respond",
                                "PV voltage nominal at 54.2V", False)
    loop._age_pending_fire_outcomes()
    payload = dict(_EMPTY_PAYLOAD)
    payload["fire_outcomes"] = [
        {"fire_id": "fid-1", "outcome": "helped", "valence": 0.9,
         "user_impact": 0.4, "evidence": "user thanked Tester for the check"}]
    captured = wire_reflection(loop, payload)

    loop._reflect_and_remember("User")

    # Stage-6 rule rides the system prompt as a suffix (prefix unchanged).
    base_sys = ChatLoop._REFLECT_SYS.format(
        character="Tester", entity="User",
        narrowness_rule=_CONCERN_INSTRUCTION_NARROWNESS_RULE)
    assert captured["system"].startswith(base_sys)
    assert "STAGE 6 — fire outcomes" in captured["system"]
    # Pending section: fire_id, concern text, reply digest, turns-ago.
    assert "## Recent autonomous acts awaiting outcome" in captured["user"]
    assert ("- [fid-1] concern: concern text — Tester did/said: "
            "PV voltage nominal at 54.2V (1 user turns ago)") in captured["user"]
    # Key list gains fire_outcomes.
    assert ("agent_concerns_closed, obligations_update, "
            "fire_outcomes)") in captured["user"]
    # Judged record left the registry and landed in autonomy.jsonl.
    assert read_pending(loop) == []
    events = read_events(tmp_path)
    assert len(events) == 1
    assert events[0]["event"] == "fire_outcome"
    assert events[0]["fire_id"] == "fid-1"
    assert events[0]["outcome"] == "helped"
    assert events[0]["latency_turns"] == 1


def test_reflection_frame_suppression_keeps_pending(loop, tmp_path):
    nid = make_concern(loop)
    loop._register_fire_outcome("fid-1", nid, "respond", "reply", False)
    payload = dict(_EMPTY_PAYLOAD)
    payload["frame"] = "roleplay"
    payload["fire_outcomes"] = [{"fire_id": "fid-1", "outcome": "helped"}]
    wire_reflection(loop, payload)

    loop._reflect_and_remember("User")

    # frame≠none suppresses ALL writes, stage 6 included — the record
    # stays pending and will age out if never judged.
    assert [r["fire_id"] for r in read_pending(loop)] == ["fid-1"]
    assert read_events(tmp_path) == []


# ── fire_id join integrity: trace record ↔ fire event ↔ outcome ────────

def test_fire_id_join_integrity(loop, tmp_path):
    nid = make_concern(loop)
    loop._autonomy_enabled = True
    loop._grow_agent_concerns_per_tick = lambda: None
    loop._check_and_fire_agent_concerns = (
        lambda: [(nid, "concern text", "check the thing")])
    loop._triage_fire_candidate = lambda *a, **k: "fire"
    loop._load_reasoning_records = lambda: []
    # _write_reasoning_history dependencies.
    loop._turn_seq = 0
    loop._last_inject_trace_seqs = []
    loop._companion_state = {}
    loop._discourse_state = {}
    loop._reasoning_trace_path = lambda: tmp_path / "reasoning_trace.jsonl"

    seen = {}

    def fake_turn(source, text, close, autonomous=False,
                  autonomous_concern_id=None, fire_id=None, **kwargs):
        # Stand-in for _process_user_turn_inner's writes, same fire_id
        # plumbing the real turn uses.
        seen["fire_id"] = fire_id
        loop._write_reasoning_history(
            source, text, [], "the reply", "respond",
            autonomous=True, fire_id=fire_id)
        loop._register_fire_outcome(
            fire_id, autonomous_concern_id, "respond", "the reply", False)

    loop._process_user_turn = fake_turn

    loop._handle_tick()

    fid = seen["fire_id"]
    assert fid  # minted at dispatch and passed to the turn
    # 1) The dispatch's autonomy event carries it.
    fire_events = [e for e in read_events(tmp_path) if e["event"] == "fire"]
    assert len(fire_events) == 1
    assert fire_events[0]["fire_id"] == fid
    # 2) The react-trace record carries it.
    trace = [json.loads(l) for l in
             (tmp_path / "reasoning_trace.jsonl").read_text().splitlines()]
    assert trace[-1]["fire_id"] == fid
    # 3) The outcome record carries it after judgment.
    pending = read_pending(loop)
    assert [r["fire_id"] for r in pending] == [fid]
    loop._apply_fire_outcome_judgments(
        [{"fire_id": fid, "outcome": "neutral"}], pending)
    outcome_events = [e for e in read_events(tmp_path)
                      if e["event"] == "fire_outcome"]
    assert len(outcome_events) == 1
    assert outcome_events[0]["fire_id"] == fid
    assert outcome_events[0]["concern_id"] == nid


def test_trace_record_has_no_fire_id_on_user_turns(loop, tmp_path):
    # Trace schema stability: user turns (fire_id=None) don't grow a key.
    loop._turn_seq = 0
    loop._last_inject_trace_seqs = []
    loop._companion_state = {}
    loop._discourse_state = {}
    loop._reasoning_trace_path = lambda: tmp_path / "reasoning_trace.jsonl"

    loop._write_reasoning_history("User", "hello", [], "hi", "respond")

    rec = json.loads(
        (tmp_path / "reasoning_trace.jsonl").read_text().splitlines()[-1])
    assert "fire_id" not in rec


# ── fire digest: one-shot surfacing into the user-turn prompt ───────────

def test_take_unsurfaced_marks_caps_and_orders(loop):
    nid = make_concern(loop)
    for i in range(_FIRE_DIGEST_MAX_ITEMS + 2):
        loop._register_fire_outcome(f"fid-{i}", nid, "respond",
                                    f"reply {i}", False)

    first = loop._take_unsurfaced_pending_fires()
    assert [r["fire_id"] for r in first] == ["fid-0", "fid-1", "fid-2"]
    # Marks persist in the registry; the remainder stays unsurfaced.
    recs = read_pending(loop)
    assert [bool(r.get("surfaced")) for r in recs] == [
        True, True, True, False, False]

    second = loop._take_unsurfaced_pending_fires()
    assert [r["fire_id"] for r in second] == ["fid-3", "fid-4"]
    assert loop._take_unsurfaced_pending_fires() == []
    # Surfacing never removes records — judgment/expiry do that.
    assert len(read_pending(loop)) == 5


def test_take_unsurfaced_empty_registry(loop):
    assert loop._take_unsurfaced_pending_fires() == []


def test_render_pending_fires_block(loop):
    nid = make_concern(loop)
    loop._register_fire_outcome("fid-1", nid, "respond",
                                "Morning reading is on the screen.", False)
    loop._age_pending_fire_outcomes()
    pending = loop._take_unsurfaced_pending_fires()

    block = loop._render_pending_fires_block(pending)
    assert block.startswith(
        "## My recent autonomous acts the user may not have seen")
    assert "- [1 user turn ago] concern text" in block
    assert "I said/did: Morning reading is on the screen." in block
