"""Unit tests for agent-concern evidence bumps, fire-time triage caching,
and per-concern WIP continuity.

Uses object.__new__(ChatLoop) with only the attributes the methods under
test touch, a stubbed LLM backend, and a throwaway scratch world — never
live world state.
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

from chat.concerns import _TRIAGE_MAX_CONSECUTIVE_DEFERS
from chat.chat_loop import (
    ChatLoop,
    _AGENT_CONCERN_BUMP_AMOUNT,
    _AGENT_CONCERN_SERVICE_FULL,
    _USER_CONCERN_BUMP_AMOUNT,
    _USER_CONCERN_BUMP_MAX_PER_TURN,
    _USER_CONCERN_DECAY_PER_TURN,
    _USER_CONCERN_HIGH_STRENGTH,
    _USER_CONCERN_STALE_DAYS,
    _triage_defer_cooldown_hours,
)
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
    # Minimal user_concerns collection injected directly (bypasses
    # create_collection's auto-indexing so the embedder never loads).
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
    inst.backend = StubBackend(['{"verdict": "fire"}'])
    inst._autonomy_log_path = lambda: tmp_path / "autonomy.jsonl"
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


# ── evidence bumps ─────────────────────────────────────────────────────

def test_bump_raises_activation_and_clears_triage_cache(loop):
    nid = make_concern(loop, activation=0.4, extra={
        "triage_verdict": "defer",
        "triage_at": datetime.now(timezone.utc).isoformat(),
        "triage_reason": "nothing new",
    })
    # Stub the semantic search: one hit pointing at our concern.
    loop.resource_manager.search_collection = (
        lambda *a, **k: (True, [{"metadata": {"source_note_id": nid}}], None))

    loop._bump_agent_concerns_on_input("input touching the concern domain")

    props = loop.resource_manager.get_resource(nid)["properties"]
    assert props["activation"] == pytest.approx(0.4 + _AGENT_CONCERN_BUMP_AMOUNT)
    assert "last_bumped_at" in props
    for f in ("triage_verdict", "triage_at", "triage_reason"):
        assert f not in props


def test_bump_caps_at_one_and_skips_inactive(loop):
    nid = make_concern(loop, activation=0.95, extra={"status": "satisfied"})
    loop.resource_manager.search_collection = (
        lambda *a, **k: (True, [{"metadata": {"source_note_id": nid}}], None))
    loop._bump_agent_concerns_on_input("anything")
    assert loop.resource_manager.get_resource(nid)["properties"]["activation"] == 0.95


# ── triage verdicts + cache ────────────────────────────────────────────

def test_triage_defer_caches_and_suppresses_llm(loop):
    nid = make_concern(loop)
    loop.backend = StubBackend(['{"verdict": "defer", "reason": "nothing new"}'])

    assert loop._triage_fire_candidate(nid, "concern text", "instr") == "defer"
    assert loop.backend.calls == 1
    props = loop.resource_manager.get_resource(nid)["properties"]
    assert props["triage_verdict"] == "defer"
    assert props["triage_reason"] == "nothing new"

    # Second triage within cooldown: cache hit, no LLM call.
    assert loop._triage_fire_candidate(nid, "concern text", "instr") == "defer"
    assert loop.backend.calls == 1


def test_triage_cache_expires_after_cooldown(loop):
    nid = make_concern(loop)
    cooldown = _triage_defer_cooldown_hours(24)
    stale = (datetime.now(timezone.utc)
             - timedelta(hours=cooldown + 0.1)).isoformat()
    props = loop.resource_manager.get_resource(nid)["properties"]
    props.update({"triage_verdict": "defer", "triage_at": stale,
                  "triage_reason": "old"})
    loop.backend = StubBackend(['{"verdict": "fire"}'])

    assert loop._triage_fire_candidate(nid, "concern text", "instr") == "fire"
    assert loop.backend.calls == 1  # cache expired → LLM re-asked
    # fire clears the stale cache fields
    assert "triage_verdict" not in loop.resource_manager.get_resource(nid)["properties"]


def test_triage_reset_decrements_without_dispatch(loop):
    nid = make_concern(loop, activation=0.9)
    loop.backend = StubBackend(['{"verdict": "reset", "reason": "all healthy"}'])

    assert loop._triage_fire_candidate(nid, "concern text", "instr") == "reset"
    props = loop.resource_manager.get_resource(nid)["properties"]
    assert props["activation"] == pytest.approx(0.9 - _AGENT_CONCERN_SERVICE_FULL)
    assert "last_fired_at" in props


def test_triage_fails_open_to_fire(loop):
    nid = make_concern(loop)
    loop.backend = StubBackend(["not json at all"])
    assert loop._triage_fire_candidate(nid, "concern text", "instr") == "fire"


def test_defers_are_counted_and_a_run_fires_anyway(loop):
    """The cooldown re-asks the question; it does not make the answer
    falsifiable. A concern whose precondition is "does this tool work"
    has no measured state to check it against — the only probe is the
    call being deferred — so the reason outlives the fault. Note_3980
    deferred the X feed 60 hours running on a 401 that had been fixed
    days earlier. Past the cap, triage fires without asking."""
    nid = make_concern(loop, activation=0.9)
    loop.backend = StubBackend(['{"verdict": "defer", "reason": "tool broken"}']
                               * (_TRIAGE_MAX_CONSECUTIVE_DEFERS + 1))
    props = loop.resource_manager.get_resource(nid)["properties"]

    for n in range(1, _TRIAGE_MAX_CONSECUTIVE_DEFERS + 1):
        props.pop("triage_at", None)          # cooldown elapsed each time
        assert loop._triage_fire_candidate(nid, "c", "i") == "defer"
        assert props["triage_defers"] == n

    props.pop("triage_at", None)
    assert loop._triage_fire_candidate(nid, "c", "i") == "fire"
    # The reason goes with the count: handed to the next triage as "my own
    # reason for deferring last time", it is what anchors the run.
    for f in ("triage_defers", "triage_reason", "triage_verdict"):
        assert f not in props


def test_a_fire_verdict_clears_the_defer_run(loop):
    nid = make_concern(loop, activation=0.9, extra={"triage_defers": 3})
    loop.backend = StubBackend(['{"verdict": "fire"}'])
    assert loop._triage_fire_candidate(nid, "c", "i") == "fire"
    assert "triage_defers" not in loop.resource_manager.get_resource(nid)["properties"]


def test_an_evidence_bump_clears_the_defer_run(loop):
    nid = make_concern(loop, activation=0.4, extra={"triage_defers": 4})
    loop.resource_manager.search_collection = (
        lambda *a, **k: (True, [{"metadata": {"source_note_id": nid}}], None))
    loop._bump_agent_concerns_on_input("new evidence in the concern domain")
    assert "triage_defers" not in loop.resource_manager.get_resource(nid)["properties"]


def test_triage_events_logged(loop, tmp_path):
    nid = make_concern(loop)
    loop.backend = StubBackend(['{"verdict": "defer", "reason": "later"}'])
    loop._triage_fire_candidate(nid, "concern text", "instr")
    records = [json.loads(l) for l in
               (tmp_path / "autonomy.jsonl").read_text().splitlines()]
    assert records[-1]["event"] == "triage"
    assert records[-1]["verdict"] == "defer"


# ── WIP continuity ─────────────────────────────────────────────────────

def test_root_concern_id_walks_successor_chain(loop):
    root = make_concern(loop)
    child = make_concern(loop, extra={"successor_of": root, "successor_depth": 1})
    grandchild = make_concern(loop, extra={"successor_of": child, "successor_depth": 2})
    assert loop._root_concern_id(grandchild) == root
    assert loop._root_concern_id(root) == root


def test_wip_update_lands_on_root(loop):
    root = make_concern(loop)
    child = make_concern(loop, extra={"successor_of": root, "successor_depth": 1})
    loop.backend = StubBackend(["Established X; still need Y."])

    loop._update_concern_wip(child, "fire instruction",
                             [("THOUGHT", "looked at X"), ("OBS", "X is fine")],
                             "reply text", "respond")

    root_props = loop.resource_manager.get_resource(root)["properties"]
    assert root_props["wip"] == "Established X; still need Y."
    assert "wip_updated_at" in root_props
    child_props = loop.resource_manager.get_resource(child)["properties"]
    assert "wip" not in child_props


def test_wip_used_in_triage_prompt(loop):
    nid = make_concern(loop, extra={"wip": "Already checked the logs."})

    captured = {}

    class CapturingBackend(StubBackend):
        def chat(self, messages, **kwargs):
            captured["user"] = messages[-1]["content"]
            return super().chat(messages, **kwargs)

    loop.backend = CapturingBackend(['{"verdict": "fire"}'])
    loop._triage_fire_candidate(nid, "concern text", "instr")
    assert "Already checked the logs." in captured["user"]


# ── user-concern aging: bump cap, stale sweep, reflection close ────────

def make_user_concern(loop, text="a user concern", strength=0.5, extra=None):
    props = {
        "kind": "user_concern",
        "status": "active",
        "strength": strength,
        "exclude_from_index": True,
    }
    props.update(extra or {})
    ok, nid, err, _ = loop.resource_manager.create_note(
        "Tester", text, "text", "pytest", "", "", props)
    assert ok, err
    loop.resource_manager.resource_registry[
        loop._user_concerns_collection_id]["properties"]["content"].append(nid)
    return nid


def test_user_bump_capped_per_turn(loop):
    nids = [make_user_concern(loop, f"concern {i}") for i in range(5)]
    hits = [{"metadata": {"source_note_id": n}} for n in nids]
    loop.resource_manager.search_collection = lambda *a, **k: (True, hits, None)

    loop._bump_user_concerns_on_input("anything")

    strengths = [loop.resource_manager.get_resource(n)["properties"]["strength"]
                 for n in nids]
    bumped = [s for s in strengths if s > 0.5]
    assert len(bumped) == _USER_CONCERN_BUMP_MAX_PER_TURN
    # ranked results: the first N get the bump, at the new amount
    assert strengths[:3] == pytest.approx([0.5 + _USER_CONCERN_BUMP_AMOUNT] * 3)
    assert strengths[3:] == [0.5, 0.5]


def test_stale_sweep_to_satisfied(loop):
    now = datetime.now(timezone.utc)
    stale = make_user_concern(loop, "stale topic", extra={
        "last_bumped_at": (now - timedelta(days=_USER_CONCERN_STALE_DAYS + 1)).isoformat()})
    fresh = make_user_concern(loop, "fresh topic", extra={
        "last_bumped_at": (now - timedelta(days=1)).isoformat()})
    closed = make_user_concern(loop, "already closed", extra={
        "status": "satisfied",
        "last_bumped_at": (now - timedelta(days=99)).isoformat()})

    loop._decay_user_concerns_per_turn()

    stale_p = loop.resource_manager.get_resource(stale)["properties"]
    assert stale_p["status"] == "satisfied"
    assert stale_p["strength"] == 0.5  # swept, not decayed
    fresh_p = loop.resource_manager.get_resource(fresh)["properties"]
    assert fresh_p["status"] == "active"
    assert fresh_p["strength"] == pytest.approx(0.5 - _USER_CONCERN_DECAY_PER_TURN)
    assert loop.resource_manager.get_resource(closed)["properties"]["status"] == "satisfied"


def test_parse_reflection_payload_closed_list():
    frame, mems, ucs, updated, closed, acs = ChatLoop._parse_reflection_payload({
        "frame": "none", "memories": [], "user_concerns": [],
        "user_concerns_closed": ["topic A", {"text": "topic B"}],
        "agent_concerns": [],
    })
    assert closed == ["topic A", "topic B"]
    # legacy payloads without the key still parse
    frame, mems, ucs, updated, closed, acs = ChatLoop._parse_reflection_payload(
        {"frame": "none", "memories": [], "user_concerns": [], "agent_concerns": []})
    assert closed == [] and updated == []


def test_parse_reflection_payload_context_pairs():
    frame, mems, ucs, updated, closed, acs = ChatLoop._parse_reflection_payload({
        "frame": "none", "memories": [],
        "user_concerns": [
            {"text": "topic A", "context": "wants help with A"},
            {"text": "topic B"},          # context optional on adds
            "bare string topic",          # legacy bare string still accepted
        ],
        "user_concerns_updated": [
            {"text": "topic C", "context": "now resolved core question"},
            {"text": "topic D"},          # no context → dropped (meaningless)
            "bare string",                # bare string → dropped
        ],
        "agent_concerns": [],
    })
    assert ucs == [("topic A", "wants help with A"), ("topic B", ""),
                   ("bare string topic", "")]
    assert updated == [("topic C", "now resolved core question")]


def test_resolve_and_close_user_concern(loop):
    nid = make_user_concern(loop, "S&P 500 daily performance")
    shown = [(nid, "S&P 500 daily performance", 0.8, {})]

    # exact-text match (case-insensitive) — no semantic search needed
    rid = loop._resolve_user_concern_by_text("s&p 500 daily performance", shown)
    assert rid == nid
    ok, err = loop._set_concern_status(rid, "satisfied")
    assert ok, err
    assert loop.resource_manager.get_resource(nid)["properties"]["status"] == "satisfied"

    # paraphrase falls back to the recurrence search
    other = make_user_concern(loop, "deployment tooling frustration")
    loop.resource_manager.search_collection = (
        lambda *a, **k: (True, [{"metadata": {"source_note_id": other}}], None))
    assert loop._resolve_user_concern_by_text("frustration with deploy tools", []) == other

    # no match → None (closure skipped, never guessed)
    loop.resource_manager.search_collection = lambda *a, **k: (True, [], None)
    assert loop._resolve_user_concern_by_text("totally unrelated", []) is None


# ── heat coupling: high-strength crossing bumps the reviewer ───────────

def make_agent_collection(loop, note_ids):
    """Register a real agent_concerns collection (the fixture default is
    a dangling id) containing the given notes."""
    mgr = loop.resource_manager
    mgr.resource_registry["Collection_ac"] = {
        "name": "Collection_ac",
        "type": mgr.resource_types.Collection,
        "location": (0, 0),
        "description": "test agent_concerns",
        "remove_on_take": False,
        "properties": {"content": list(note_ids), "format": "list",
                       "collection_name": "agent_concerns",
                       "kind": "agent_concerns"},
    }
    loop._agent_concerns_collection_id = "Collection_ac"


def test_high_strength_crossing_bumps_reviewer(loop):
    reviewer = make_concern(loop, activation=0.3, extra={
        "user_model_reviewer": True,
        "triage_verdict": "defer",
        "triage_at": datetime.now(timezone.utc).isoformat(),
        "triage_reason": "nothing new",
    })
    bystander = make_concern(loop, activation=0.3)
    make_agent_collection(loop, [reviewer, bystander])
    crossing = make_user_concern(
        loop, "hot topic", strength=_USER_CONCERN_HIGH_STRENGTH - 0.05)
    loop.resource_manager.search_collection = (
        lambda *a, **k: (True, [{"metadata": {"source_note_id": crossing}}], None))

    loop._bump_user_concerns_on_input("input about the hot topic")

    rp = loop.resource_manager.get_resource(reviewer)["properties"]
    assert rp["activation"] == pytest.approx(0.3 + _AGENT_CONCERN_BUMP_AMOUNT)
    for f in ("triage_verdict", "triage_at", "triage_reason"):
        assert f not in rp
    assert loop.resource_manager.get_resource(
        bystander)["properties"]["activation"] == 0.3


def test_no_reviewer_bump_without_crossing(loop):
    reviewer = make_concern(loop, activation=0.3,
                            extra={"user_model_reviewer": True})
    make_agent_collection(loop, [reviewer])
    # One concern already above the threshold, one staying below: a bump
    # to either is not a crossing.
    above = make_user_concern(loop, "already hot",
                              strength=_USER_CONCERN_HIGH_STRENGTH + 0.05)
    below = make_user_concern(loop, "still cool", strength=0.2)
    hits = [{"metadata": {"source_note_id": n}} for n in (above, below)]
    loop.resource_manager.search_collection = lambda *a, **k: (True, hits, None)

    loop._bump_user_concerns_on_input("anything")

    assert loop.resource_manager.get_resource(
        reviewer)["properties"]["activation"] == 0.3


def test_seed_sync_flags_existing_reviewer_note(loop):
    existing = make_concern(loop, activation=0.5)
    name = ChatLoop._seed_concern_name(0)
    loop.resource_manager.named_notes[name] = existing

    loop._seed_concerns_from_config({"concerns": [
        {"text": "review what the user has been tracking",
         "user_model_reviewer": True},
    ]})

    props = loop.resource_manager.get_resource(existing)["properties"]
    assert props["user_model_reviewer"] is True
    assert props["activation"] == 0.5  # untouched beyond the flag


# ── user-concern context: birth, recurrence refresh, explicit update ───

def test_add_user_concern_stores_context_and_description(loop):
    loop.resource_manager.search_collection = lambda *a, **k: (True, [], None)
    nid = loop._add_user_concern(
        "vector store choice", entity="User",
        context="User is weighing FAISS vs alternatives. Wants a sounding board.")
    note = loop.resource_manager.get_resource(nid)
    props = note["properties"]
    assert props["context"].startswith("User is weighing FAISS")
    assert "context_updated_at" in props
    assert note["description"] == props["context"][:140]


def test_recurrence_refreshes_context_newest_wins(loop):
    existing = make_user_concern(loop, "vector store choice", strength=0.4,
                                 extra={"context": "old reading"})
    loop.resource_manager.search_collection = (
        lambda *a, **k: (True, [{"metadata": {"source_note_id": existing}}], None))
    rid = loop._add_user_concern("choosing a vector store", entity="User",
                                 context="new reading: decided on FAISS, now tuning")
    assert rid == existing
    note = loop.resource_manager.get_resource(existing)
    assert note["properties"]["context"].startswith("new reading")
    assert note["properties"]["strength"] == pytest.approx(
        0.4 + _USER_CONCERN_BUMP_AMOUNT)
    assert note["description"].startswith("new reading")
    # recurrence WITHOUT context keeps the stored one
    loop._add_user_concern("choosing a vector store", entity="User")
    assert loop.resource_manager.get_resource(
        existing)["properties"]["context"].startswith("new reading")


def test_update_user_concern_context(loop):
    nid = make_user_concern(loop, "topic", extra={"context": "v1"})
    assert loop._update_user_concern_context(nid, "v2 — moved on to triage design")
    note = loop.resource_manager.get_resource(nid)
    assert note["properties"]["context"].startswith("v2")
    assert note["description"].startswith("v2")
    assert not loop._update_user_concern_context(nid, "   ")
    assert not loop._update_user_concern_context("Note_missing", "x")


def test_user_concerns_for_evaluator_shape(loop):
    make_user_concern(loop, "topic A", strength=0.9,
                      extra={"context": "wants help"})
    make_user_concern(loop, "topic B", strength=0.5)
    out = loop._user_concerns_for_evaluator()
    assert out[0] == {"concern_label": "topic A",
                      "concern_description": "wants help",
                      "status": "open"}
    assert out[1]["concern_label"] == "topic B"
    assert out[1]["concern_description"] == ""


def test_render_user_concerns_block_includes_context():
    block = ChatLoop._render_user_concerns_block("User", [
        ("Note_1", "topic A", 0.9, {"context": "evidence + stance here"}),
        ("Note_2", "topic B", 0.5, {}),
    ])
    assert "- [0.90] topic A" in block
    assert "    evidence + stance here" in block
    assert "- [0.50] topic B" in block


# ── capability gaps → self-extension concern (Phase 2a) ────────────────

def _inject_agent_collection(loop):
    """Real agent_concerns collection so _iter_active_agent_concerns walks
    it (mirrors the fixture's user_concerns injection — no embedder)."""
    mgr = loop.resource_manager
    mgr.resource_registry["Collection_ac"] = {
        "name": "Collection_ac",
        "type": mgr.resource_types.Collection,
        "location": (0, 0),
        "description": "test agent_concerns",
        "remove_on_take": False,
        "properties": {"content": [], "format": "list",
                       "collection_name": "agent_concerns",
                       "kind": "agent_concerns"},
    }
    loop._agent_concerns_collection_id = "Collection_ac"


def _add_to_agent_collection(loop, nid):
    loop.resource_manager.resource_registry["Collection_ac"][
        "properties"]["content"].append(nid)


def test_capability_gap_records_to_self_extension_concern(loop, tmp_path):
    _inject_agent_collection(loop)
    nid = make_concern(loop, activation=0.2, extra={"self_extension": True})
    _add_to_agent_collection(loop, nid)

    assert loop._self_extension_concern_id() == nid

    loop._record_capability_gap(
        "I had to convert timezones by hand because I have no tool for it.")

    props = loop.resource_manager.get_resource(nid)["properties"]
    # Gap lands in WIP (rides into the fire frame) ...
    assert "no tool for it" in props["wip"]
    assert "GAP NOTED" in props["wip"]
    assert "wip_updated_at" in props
    # ... and evidence-bumps activation (recurring gaps fire ahead of rhythm).
    assert props["activation"] == pytest.approx(0.2 + _AGENT_CONCERN_BUMP_AMOUNT)

    # A second gap appends, not overwrites.
    loop._record_capability_gap("I could not fetch the weather.")
    wip = loop.resource_manager.get_resource(nid)["properties"]["wip"]
    assert "no tool for it" in wip and "fetch the weather" in wip

    # Distinct autonomy.jsonl events for the eval surface.
    recs = [json.loads(l) for l in
            (tmp_path / "autonomy.jsonl").read_text().strip().splitlines()]
    gap_events = [r for r in recs if r.get("event") == "capability_gap"]
    assert len(gap_events) == 2
    assert gap_events[0]["concern_id"] == nid
    assert "no tool" in gap_events[0]["gap"]


def test_capability_gap_noop_without_seed(loop, tmp_path):
    _inject_agent_collection(loop)
    nid = make_concern(loop, activation=0.5)  # plain, not self_extension
    _add_to_agent_collection(loop, nid)

    assert loop._self_extension_concern_id() is None
    loop._record_capability_gap("some gap")  # no-op, no crash

    props = loop.resource_manager.get_resource(nid)["properties"]
    assert "wip" not in props
    assert props["activation"] == 0.5
    p = tmp_path / "autonomy.jsonl"
    assert not p.exists() or not p.read_text().strip()


def test_capability_gap_empty_is_ignored(loop, tmp_path):
    _inject_agent_collection(loop)
    nid = make_concern(loop, activation=0.3, extra={"self_extension": True})
    _add_to_agent_collection(loop, nid)
    loop._record_capability_gap("   ")
    props = loop.resource_manager.get_resource(nid)["properties"]
    assert "wip" not in props
    assert props["activation"] == 0.3


# ── WIP reviewer: inventory collection + prompt block ──────────────────

def test_collect_concern_wip_filters_orders_excludes(loop):
    _inject_agent_collection(loop)
    n_hi = make_concern(loop, activation=0.9,
                        extra={"wip": "pending: follow up on X"})
    n_lo = make_concern(loop, activation=0.3,
                        extra={"wip": "established: how to do Y"})
    n_bare = make_concern(loop, activation=0.7)  # no WIP — dropped
    n_rev = make_concern(loop, activation=0.5,
                         extra={"wip": "reviewed everything",
                                "wip_reviewer": True})
    for nid in (n_hi, n_lo, n_bare, n_rev):
        _add_to_agent_collection(loop, nid)

    inv = loop._collect_concern_wip(exclude_id=n_rev)
    assert [t[0] for t in inv] == [n_hi, n_lo]  # activation-descending
    assert inv[0][3] == "pending: follow up on X"
    assert inv[0][1] == "concern text"


def test_render_wip_review_block(loop):
    block = loop._render_wip_review_block([
        ("Note_1", "topic A", 0.9, "line one\nline two"),
        ("Note_2", "topic B", 0.4, "single"),
    ])
    assert block.startswith(
        "## Work-in-progress across my active concerns (for this review)")
    assert "- [0.90] topic A" in block
    assert "    line one" in block and "    line two" in block
    assert "- [0.40] topic B" in block


# ── agent-concern closures (reflection retire path) ────────────────────

def test_agent_concern_closure_abandons_and_logs(loop, tmp_path):
    _inject_agent_collection(loop)
    nid = make_concern(loop, activation=0.6)
    _add_to_agent_collection(loop, nid)
    shown = [(nid, "concern text", 0.6, {})]

    closed = loop._apply_agent_concern_closures(["Concern Text"], shown)

    assert closed == ["Concern Text"]  # case-insensitive exact match
    props = loop.resource_manager.get_resource(nid)["properties"]
    assert props["status"] == "abandoned"
    recs = [json.loads(l) for l in
            (tmp_path / "autonomy.jsonl").read_text().strip().splitlines()]
    assert recs[-1]["event"] == "concern_abandoned"
    assert recs[-1]["concern_id"] == nid
    assert recs[-1]["via"] == "reflection"


def test_agent_concern_closure_refuses_seed(loop, tmp_path):
    _inject_agent_collection(loop)
    nid = make_concern(loop, activation=0.6, extra={"seed": True})
    _add_to_agent_collection(loop, nid)
    shown = [(nid, "concern text", 0.6, {})]

    closed = loop._apply_agent_concern_closures(["concern text"], shown)

    assert closed == []
    props = loop.resource_manager.get_resource(nid)["properties"]
    assert props["status"] == "active"
    p = tmp_path / "autonomy.jsonl"
    assert not p.exists() or not p.read_text().strip()


def test_agent_concern_closure_skips_unmatched_and_bad_input(loop):
    _inject_agent_collection(loop)
    nid = make_concern(loop, activation=0.6)
    _add_to_agent_collection(loop, nid)
    shown = [(nid, "concern text", 0.6, {})]

    assert loop._apply_agent_concern_closures(
        ["something never shown"], shown) == []
    assert loop._apply_agent_concern_closures(None, shown) == []
    assert loop._apply_agent_concern_closures("not a list", shown) == []
    props = loop.resource_manager.get_resource(nid)["properties"]
    assert props["status"] == "active"


# ── seed instruction sync from YAML ────────────────────────────────────

def test_seed_instruction_syncs_from_yaml(loop):
    nid = make_concern(loop, instruction="OLD procedure",
                       extra={"seed": True})
    loop.resource_manager.named_notes["chat:agent_concern:seed:0"] = nid

    loop._seed_concerns_from_config({"concerns": [
        {"text": "concern text", "instruction": "NEW procedure"}]})

    props = loop.resource_manager.get_resource(nid)["properties"]
    assert props["instruction"] == "NEW procedure"

    # YAML-less instruction never deletes an existing one.
    loop._seed_concerns_from_config({"concerns": [{"text": "concern text"}]})
    props = loop.resource_manager.get_resource(nid)["properties"]
    assert props["instruction"] == "NEW procedure"


# ── intentional yield (deliberate multi-loop continuation) ─────────────
# The `yield` ReAct action ends an autonomous run at a chosen boundary and
# spawns a successor concern carrying the agent's own `next` instruction
# verbatim — same creation path and depth cap as the reactive max_iters
# route, but no synthesizer LLM pass.

from chat.concerns import (  # noqa: E402
    _AGENT_CONCERN_FIRE_THRESHOLD,
    _CONCERN_SUCCESSOR_MAX_DEPTH,
)


def _mute_indexer(loop):
    """Successor creation goes through create_note, which would index the
    new Note (embedder load). Not what these tests exercise."""
    loop.resource_manager.resource_indexer.index_note = lambda *a, **k: None


def test_yield_spawns_successor_verbatim_no_llm(loop):
    _mute_indexer(loop)
    parent = make_concern(loop)
    loop.backend = StubBackend(["must not be consulted"])
    nxt = "Finish steps 4-6: place the boiler row at (12,4); ids are in WIP."
    succ = loop._spawn_successor_from_yield(parent, nxt)
    assert succ
    # Verbatim handoff: the synthesizer LLM is never called.
    assert loop.backend.calls == 0
    props = loop.resource_manager.get_resource(succ)["properties"]
    assert props["instruction"] == nxt
    assert props["successor_of"] == parent
    assert props["successor_depth"] == 1
    # Primed AT threshold, not 0.1 below: the same-tick re-fire this
    # margin guarded against is prevented structurally instead
    # (_handle_tick computes its fire list before dispatching).
    assert props["activation"] == pytest.approx(_AGENT_CONCERN_FIRE_THRESHOLD)
    assert loop._root_concern_id(succ) == parent


def test_yield_successor_depth_capped(loop):
    _mute_indexer(loop)
    parent = make_concern(
        loop, extra={"successor_depth": _CONCERN_SUCCESSOR_MAX_DEPTH})
    assert loop._spawn_successor_from_yield(parent, "keep going") is None


def test_yield_empty_next_spawns_nothing(loop):
    _mute_indexer(loop)
    parent = make_concern(loop)
    assert loop._spawn_successor_from_yield(parent, "   ") is None
    assert loop._spawn_successor_from_yield(parent, None) is None


def test_service_yield_full_decrement(loop):
    nid = make_concern(loop, activation=0.8)
    loop._service_agent_concern(nid, "yield")
    props = loop.resource_manager.get_resource(nid)["properties"]
    assert props["activation"] == pytest.approx(
        max(0.0, 0.8 - _AGENT_CONCERN_SERVICE_FULL))
    assert props["last_fired_at"]


def test_tool_catalog_always_offers_yield(loop):
    # User-turn yield extension: the action is offered on every turn,
    # not just autonomous fires.
    loop._discovered_tools = {}
    loop._omitted_tools = []
    loop._get_external_repo = lambda: None
    assert '"yield"' in loop._build_react_tool_catalog()


def test_user_yield_spawns_fresh_concern(loop):
    _mute_indexer(loop)
    loop.backend = StubBackend(["must not be consulted"])
    nxt = "Extract the plates, craft the drill, place it at (-54, -31)."
    nid = loop._spawn_concern_from_user_yield(
        "restore the burner drill you deleted", nxt)
    assert nid
    assert loop.backend.calls == 0  # verbatim, no synthesizer
    props = loop.resource_manager.get_resource(nid)["properties"]
    assert props["instruction"] == nxt
    assert "successor_of" not in props  # fresh root, not a successor
    # Primed AT threshold so the next tick fires it — verified live
    # 2026-08-16: spawn 16:29:11, triage fire 16:29:14.
    assert props["activation"] == pytest.approx(_AGENT_CONCERN_FIRE_THRESHOLD)
    # Fresh root: its own future yields start a normal successor chain.
    assert loop._root_concern_id(nid) == nid


def test_reflection_cannot_abandon_a_yield_continuation(loop):
    """Reflection must not retire continuation work.

    Observed live 2026-08-16 in an isolated single-agent world: a
    user-turn yield spawned Note_4 at 16:29:11, triage fired it at
    16:29:34 reflection abandoned it — 20s into the fire. The work
    survived only because dispatch had already won the race; a slower
    tick would have lost the continuation silently. `_apply_agent_concern_
    closures` already refuses seeds and system-spawned claim audits on
    exactly this reasoning, so both yield spawn paths now carry the flag.
    """
    _mute_indexer(loop)

    for nid, label in (
            (loop._spawn_concern_from_user_yield('the ask', 'the remainder'),
             'user-turn yield'),
            (loop._create_successor_concern(make_concern(loop), 'more work'),
             'autonomous successor')):
        assert nid, label
        props = loop.resource_manager.get_resource(nid)['properties']
        assert props['system_spawned'] is True, label

        text = props['content']
        shown = [(nid, text, props['activation'], props)]
        assert loop._apply_agent_concern_closures([text], shown) == [], label
        assert loop.resource_manager.get_resource(
            nid)['properties']['status'] == 'active', label


def test_user_yield_empty_next_spawns_nothing(loop):
    _mute_indexer(loop)
    assert loop._spawn_concern_from_user_yield("some ask", "   ") is None
    assert loop._spawn_concern_from_user_yield("some ask", None) is None


def test_synthesize_remainder_complete_verdict(loop):
    loop.backend = StubBackend(['{"complete": true}'])
    assert loop._synthesize_remainder(
        "build the line", [("ACTION 1", "x")]) == ('complete', None)
    assert loop.backend.calls == 1


def test_user_max_iters_synthesizes_then_spawns(loop):
    # The user-turn max_iters route: synthesizer names the remainder,
    # which spawns a fresh concern exactly like a verbatim yield.
    _mute_indexer(loop)
    loop.backend = StubBackend(
        ['{"complete": false, "next_slice": "Place the belt at the drop tile."}'])
    verdict, nxt = loop._synthesize_remainder(
        "proceed with the smelting line", [("ACTION 12", "cut off")])
    assert verdict == 'remainder'
    assert nxt == "Place the belt at the drop tile."
    nid = loop._spawn_concern_from_user_yield("proceed with the smelting line", nxt)
    assert nid
    props = loop.resource_manager.get_resource(nid)["properties"]
    assert props["instruction"] == nxt


# ── one-shot lifecycle: completion is terminal ─────────────────────────

def test_one_shot_satisfied_on_respond(loop):
    nid = make_concern(loop, extra={"category": "one_shot"})
    loop._service_agent_concern(nid, "respond")
    assert loop.resource_manager.get_resource(nid)["properties"]["status"] == "satisfied"


def test_durable_stays_active_on_respond(loop):
    nid = make_concern(loop)
    loop._service_agent_concern(nid, "respond")
    assert loop.resource_manager.get_resource(nid)["properties"]["status"] == "active"


def test_legacy_urgency_spawn_counts_as_one_shot(loop):
    # Pre-category continuation spawns carried rhythm_source='urgency';
    # they must satisfy on completion like stamped one-shots.
    nid = make_concern(loop, extra={"rhythm_source": "urgency"})
    loop._service_agent_concern(nid, "respond")
    assert loop.resource_manager.get_resource(nid)["properties"]["status"] == "satisfied"


def test_seed_never_satisfied_even_as_one_shot(loop):
    nid = make_concern(loop, extra={"seed": True, "category": "one_shot"})
    loop._service_agent_concern(nid, "respond")
    assert loop.resource_manager.get_resource(nid)["properties"]["status"] == "active"


def test_successor_spawn_supersedes_one_shot_parent(loop):
    _mute_indexer(loop)
    parent = make_concern(loop, extra={"category": "one_shot"})
    succ = loop._spawn_successor_from_yield(parent, "finish the belt run")
    assert succ
    assert loop.resource_manager.get_resource(parent)["properties"]["status"] == "satisfied"
    sprops = loop.resource_manager.get_resource(succ)["properties"]
    assert sprops["status"] == "active"
    assert sprops["category"] == "one_shot"


def test_successor_spawn_keeps_durable_parent(loop):
    _mute_indexer(loop)
    parent = make_concern(loop)
    succ = loop._spawn_successor_from_yield(parent, "finish the belt run")
    assert succ
    assert loop.resource_manager.get_resource(parent)["properties"]["status"] == "active"


def test_depth_cap_satisfies_one_shot_parent(loop):
    _mute_indexer(loop)
    parent = make_concern(loop, extra={
        "category": "one_shot",
        "successor_depth": _CONCERN_SUCCESSOR_MAX_DEPTH})
    assert loop._spawn_successor_from_yield(parent, "keep going") is None
    assert loop.resource_manager.get_resource(parent)["properties"]["status"] == "satisfied"


def test_synth_complete_satisfies_one_shot_parent(loop):
    _mute_indexer(loop)
    parent = make_concern(loop, extra={"category": "one_shot"})
    loop.backend = StubBackend(['{"complete": true}'])
    assert loop._maybe_spawn_successor_concern(parent, "instr", [("A", "x")]) is None
    assert loop.resource_manager.get_resource(parent)["properties"]["status"] == "satisfied"


def test_synth_error_leaves_one_shot_active(loop):
    _mute_indexer(loop)
    parent = make_concern(loop, extra={"category": "one_shot"})
    loop.backend = StubBackend(['no json here'])
    assert loop._maybe_spawn_successor_concern(parent, "instr", [("A", "x")]) is None
    assert loop.resource_manager.get_resource(parent)["properties"]["status"] == "active"


# ── stale sweep ────────────────────────────────────────────────────────

def _age_note(loop, nid, days):
    old = (datetime.now(timezone.utc) - timedelta(days=days)).isoformat()
    props = loop.resource_manager.get_resource(nid)["properties"]
    props["created_at"] = old
    props["last_fired_at"] = old
    props.pop("last_bumped_at", None)


def test_sweep_satisfies_stale_one_shot(loop):
    # rhythm 1h (the continuation-spawn value) keeps the 2×rhythm
    # lifetime floor below the 0.5d one_shot default.
    nid = make_concern(loop, extra={"category": "one_shot",
                                    "rhythm_hours": 1})
    _age_note(loop, nid, days=1)  # one_shot lifetime is 0.5d
    _inject_agent_collection(loop)
    _add_to_agent_collection(loop, nid)
    loop._sweep_stale_agent_concerns()
    assert loop.resource_manager.get_resource(nid)["properties"]["status"] == "satisfied"


def test_sweep_keeps_fresh_one_shot(loop):
    nid = make_concern(loop, extra={"category": "one_shot"})
    _inject_agent_collection(loop)
    _add_to_agent_collection(loop, nid)
    loop._sweep_stale_agent_concerns()
    assert loop.resource_manager.get_resource(nid)["properties"]["status"] == "active"


def test_sweep_keeps_durable_within_lifetime(loop):
    nid = make_concern(loop)
    _age_note(loop, nid, days=30)  # durable lifetime is 120d
    _inject_agent_collection(loop)
    _add_to_agent_collection(loop, nid)
    loop._sweep_stale_agent_concerns()
    assert loop.resource_manager.get_resource(nid)["properties"]["status"] == "active"


def test_sweep_never_touches_seeds(loop):
    nid = make_concern(loop, extra={"seed": True})
    _age_note(loop, nid, days=3650)
    _inject_agent_collection(loop)
    _add_to_agent_collection(loop, nid)
    loop._sweep_stale_agent_concerns()
    assert loop.resource_manager.get_resource(nid)["properties"]["status"] == "active"


def test_sweep_infers_one_shot_for_legacy_urgency(loop):
    # Legacy continuation (no category stamp): swept on the one_shot
    # lifetime, not the durable one. rhythm 1h as the spawn paths write.
    nid = make_concern(loop, extra={"rhythm_source": "urgency",
                                    "rhythm_hours": 1})
    _age_note(loop, nid, days=1)
    _inject_agent_collection(loop)
    _add_to_agent_collection(loop, nid)
    loop._sweep_stale_agent_concerns()
    assert loop.resource_manager.get_resource(nid)["properties"]["status"] == "satisfied"


# ── name-keyed seeding + legacy slot adoption ──────────────────────────

def test_seed_name_keyed_creation(loop):
    _mute_indexer(loop)
    _inject_agent_collection(loop)
    loop._seed_concerns_from_config(
        {'concerns': [{'text': 'watch the feed', 'name': 'x-feed'}]})
    nid = loop.resource_manager.named_notes.get('chat:agent_concern:seed:x-feed')
    assert nid
    props = loop.resource_manager.get_resource(nid)["properties"]
    assert props["seed"] is True
    assert props["content"] == 'watch the feed'


def test_seed_legacy_slot_adopted_and_repaired(loop):
    # The live 2026-07-17 corruption: an index slot carrying stale text
    # and a stale designation flag. Naming the entry adopts the slot and
    # the authoritative sync repairs text, flags, and rhythm in place.
    _mute_indexer(loop)
    _inject_agent_collection(loop)
    loop._seed_concerns_from_config(
        {'concerns': [{'text': 'review the WIP', 'wip_reviewer': True}]})
    old_id = loop.resource_manager.named_notes['chat:agent_concern:seed:0']

    loop._seed_concerns_from_config(
        {'concerns': [{'text': 'watch the X feed', 'name': 'x-feed',
                       'rhythm_hours': 1}]})
    assert 'chat:agent_concern:seed:0' not in loop.resource_manager.named_notes
    new_id = loop.resource_manager.named_notes['chat:agent_concern:seed:x-feed']
    assert new_id == old_id  # adopted, not recreated
    props = loop.resource_manager.get_resource(new_id)["properties"]
    assert props["content"] == 'watch the X feed'
    assert not props.get("wip_reviewer")
    assert props["rhythm_hours"] == 1


def test_seed_unnamed_keeps_legacy_no_text_sync(loop):
    _mute_indexer(loop)
    _inject_agent_collection(loop)
    loop._seed_concerns_from_config({'concerns': [{'text': 'original'}]})
    nid = loop.resource_manager.named_notes['chat:agent_concern:seed:0']
    loop._seed_concerns_from_config({'concerns': [{'text': 'edited'}]})
    props = loop.resource_manager.get_resource(nid)["properties"]
    assert props["content"] == 'original'  # index slots never text-sync


def test_seed_duplicate_name_skipped(loop):
    _mute_indexer(loop)
    _inject_agent_collection(loop)
    loop._seed_concerns_from_config(
        {'concerns': [{'text': 'first', 'name': 'dup'},
                      {'text': 'second', 'name': 'dup'}]})
    nid = loop.resource_manager.named_notes['chat:agent_concern:seed:dup']
    assert loop.resource_manager.get_resource(nid)["properties"]["content"] == 'first'


# ── reflection category + one-shot lifetime floor ──────────────────────

def test_parse_reflection_payload_agent_category():
    frame, _, _, _, _, acs = ChatLoop._parse_reflection_payload({
        "frame": "none", "memories": [], "user_concerns": [],
        "agent_concerns": [
            {"text": "remind about dentist", "instruction": "remind",
             "rhythm_hours": 12, "category": "one_shot"},
            {"text": "track topic", "instruction": "check"},
            {"text": "bad category", "instruction": "x",
             "category": "derived"},
        ]})
    assert frame == "none"
    assert acs[0]["category"] == "one_shot"
    assert acs[1]["category"] == "durable"   # missing → durable
    assert acs[2]["category"] == "durable"   # invalid → durable


def test_add_agent_concern_stores_category(loop):
    _mute_indexer(loop)
    _inject_agent_collection(loop)
    nid = loop._add_agent_concern(
        "one-time thing", instruction="do it", rhythm_hours=12,
        skip_recurrence=True, category="one_shot")
    props = loop.resource_manager.get_resource(nid)["properties"]
    assert props["category"] == "one_shot"


def test_stale_sweep_lifetime_floored_by_rhythm(loop):
    _inject_agent_collection(loop)
    now = datetime.now(timezone.utc)

    def make(created_days_ago, rhythm_hours):
        nid = make_concern(loop, extra={
            "category": "one_shot",
            "rhythm_hours": rhythm_hours,
        })
        # create_note stamps its own created_at — backdate post-creation.
        loop.resource_manager.get_resource(nid)["properties"]["created_at"] = (
            now - timedelta(days=created_days_ago)).isoformat()
        loop.resource_manager.resource_registry[
            "Collection_ac"]["properties"]["content"].append(nid)
        return nid

    # 0.6d old with a 24h rhythm: past the 0.5d one_shot default but
    # inside the 2×rhythm floor — must NOT be swept before first fire.
    waiting = make(0.6, 24)
    # 3d old with a 24h rhythm: past the floor — swept.
    expired = make(3.0, 24)

    loop._sweep_stale_agent_concerns()

    assert loop.resource_manager.get_resource(
        waiting)["properties"]["status"] == "active"
    assert loop.resource_manager.get_resource(
        expired)["properties"]["status"] == "satisfied"


# ── dead-concern cleanup (graveyard + delete) ──────────────────────────

def _make_dead(loop, status, days_dead, extra=None):
    old = (datetime.now(timezone.utc) - timedelta(days=days_dead)).isoformat()
    props = {"status": status, "status_changed_at": old}
    props.update(extra or {})
    nid = make_concern(loop, extra=props)
    # create_note stamps created_at=now; backdate so the latest-of grace
    # anchor sees the intended age (in production created_at always
    # precedes status_changed_at).
    loop.resource_manager.get_resource(nid)["properties"]["created_at"] = old
    loop.resource_manager.resource_registry[
        "Collection_ac"]["properties"]["content"].append(nid)
    return nid


def test_set_concern_status_stamps_time(loop):
    nid = make_concern(loop)
    ok, err = loop._set_concern_status(nid, "abandoned")
    assert ok, err
    props = loop.resource_manager.get_resource(nid)["properties"]
    assert props["status"] == "abandoned"
    assert props["status_changed_at"]


def test_dead_concern_disposition(loop):
    assert loop._dead_concern_disposition(
        {"status": "abandoned"}) == "abandoned"
    assert loop._dead_concern_disposition(
        {"status": "satisfied", "category": "one_shot"}) == "completed_one_shot"
    assert loop._dead_concern_disposition(
        {"status": "satisfied", "category": "durable"}) is None
    assert loop._dead_concern_disposition(
        {"status": "abandoned", "seed": True}) is None
    assert loop._dead_concern_disposition({"status": "active"}) is None


def test_delete_dead_concerns_tiers(loop, tmp_path):
    _inject_agent_collection(loop)
    loop._memory_dir = lambda: tmp_path / "memory"
    old_one_shot = _make_dead(loop, "satisfied", 10,
                              extra={"category": "one_shot"})
    fresh_one_shot = _make_dead(loop, "satisfied", 1,
                                extra={"category": "one_shot"})
    old_durable = _make_dead(loop, "satisfied", 10)
    old_abandoned = _make_dead(loop, "abandoned", 10)
    active = _make_dead(loop, "active", 10)

    deleted = loop._delete_dead_agent_concerns()

    assert {nid for nid, _, _ in deleted} == {old_one_shot, old_abandoned}
    mgr = loop.resource_manager
    assert mgr.get_resource(old_one_shot) is None
    assert mgr.get_resource(old_abandoned) is None
    assert mgr.get_resource(fresh_one_shot) is not None   # within grace
    assert mgr.get_resource(old_durable) is not None      # revival pool
    assert mgr.get_resource(active) is not None
    # collection membership cleaned by delete_resource
    content = mgr.resource_registry["Collection_ac"]["properties"]["content"]
    assert old_one_shot not in content and old_abandoned not in content
    # tombstones written, one line each, note carried verbatim
    gy = tmp_path / "memory" / "concerns_graveyard.jsonl"
    lines = [json.loads(l) for l in gy.read_text().splitlines()]
    assert {l["note_id"] for l in lines} == {old_one_shot, old_abandoned}
    assert all(l["note"]["properties"]["status"] in ("satisfied", "abandoned")
               for l in lines)
    # autonomy events logged
    events = [json.loads(l) for l in
              (tmp_path / "autonomy.jsonl").read_text().splitlines()]
    assert sum(1 for e in events if e["event"] == "concern_deleted") == 2


def test_delete_dead_concerns_dry_run(loop, tmp_path):
    _inject_agent_collection(loop)
    loop._memory_dir = lambda: tmp_path / "memory"
    nid = _make_dead(loop, "abandoned", 10)

    doomed = loop._delete_dead_agent_concerns(dry_run=True)

    assert [d[0] for d in doomed] == [nid]
    assert loop.resource_manager.get_resource(nid) is not None
    assert not (tmp_path / "memory" / "concerns_graveyard.jsonl").exists()


def test_delete_dead_concerns_no_timestamp_kept(loop, tmp_path):
    _inject_agent_collection(loop)
    loop._memory_dir = lambda: tmp_path / "memory"
    nid = _make_dead(loop, "abandoned", 10)
    props = loop.resource_manager.get_resource(nid)["properties"]
    for k in ("status_changed_at", "last_fired_at", "last_bumped_at",
              "created_at"):
        props.pop(k, None)

    assert loop._delete_dead_agent_concerns() == []
    assert loop.resource_manager.get_resource(nid) is not None
