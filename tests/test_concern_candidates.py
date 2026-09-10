"""The agent's own concerns, in shadow (2026-09-07): the population cap,
near-miss rows and their recurrence, candidate rows and would-promote,
expectation lines and their checks, and the two companion headings.

Same construction as test_concern_dynamics.py: object.__new__(ChatLoop)
over a scratch world, a stubbed backend, nothing live.
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

from chat import concerns as C                                   # noqa: E402
from chat.chat_loop import ChatLoop                              # noqa: E402
from chat.memories import near_miss_recurrent                    # noqa: E402
from infospace_resource_manager import InfospaceResourceManager   # noqa: E402


class StubBackend:
    def __init__(self, responses):
        self.responses = list(responses)
        self.calls = 0

    def chat(self, messages, **kwargs):
        self.calls += 1
        return self.responses.pop(0) if len(self.responses) > 1 else self.responses[0]


@pytest.fixture
def loop(tmp_path):
    world = f"pytest_scratch_{uuid.uuid4().hex[:8]}"
    mgr = InfospaceResourceManager(world, world_config={"world_name": world})
    inst = object.__new__(ChatLoop)
    inst.character_name = "Tester"
    inst.resource_manager = mgr
    inst._faiss_lock = threading.Lock()
    for cid, kind in (("Collection_ac", "agent_concerns"), ("Collection_uc", "user_concerns")):
        mgr.resource_registry[cid] = {
            "name": cid, "type": mgr.resource_types.Collection, "location": (0, 0),
            "description": kind, "remove_on_take": False,
            "properties": {"content": [], "format": "list", "collection_name": kind, "kind": kind}}
    inst._agent_concerns_collection_id = "Collection_ac"
    inst._user_concerns_collection_id = "Collection_uc"
    inst.backend = StubBackend(['{"verdict": "fire"}'])
    inst._autonomy_log_path = lambda: tmp_path / "autonomy.jsonl"
    inst._memory_dir = lambda: tmp_path / "memory"
    # no embedder: recurrence search finds nothing
    inst._find_similar_concern = lambda text, cid: None
    inst._candidate_embedder = lambda: None
    yield inst
    shutil.rmtree(Path(__file__).parent.parent / "scenarios" / world, ignore_errors=True)


def _note(loop, cid, props, text="concern text"):
    props = {"exclude_from_index": True, **props}
    ok, nid, err, _ = loop.resource_manager.create_note("Tester", text, "text", "pytest", "", "", props)
    assert ok, err
    loop.resource_manager.resource_registry[cid]["properties"]["content"].append(nid)
    return nid


def _rows(path):
    return [json.loads(l) for l in Path(path).read_text().splitlines() if l.strip()]


# ── the cap ───────────────────────────────────────────────────────────

def test_cap_refuses_chosen_concerns_and_passes_debts(loop, tmp_path):
    for i in range(C._AGENT_CONCERN_POPULATION_CAP):
        _note(loop, "Collection_ac", {"kind": "agent_concern", "status": "active",
                                      "activation": 0.1, "instruction": None, "rhythm_hours": 24},
              text=f"c{i}")
    _note(loop, "Collection_ac", {"kind": "agent_concern", "status": "active", "seed": True,
                                  "activation": 0.1, "instruction": None, "rhythm_hours": 24}, text="seed")
    assert loop._active_nonseed_agent_count() == C._AGENT_CONCERN_POPULATION_CAP
    # reflection's kind of create: refused, logged
    assert loop._add_agent_concern("one more", entity="User") is None
    ev = _rows(tmp_path / "autonomy.jsonl")
    assert ev[-1]["event"] == "concern_refused_cap" and ev[-1]["cap"] == C._AGENT_CONCERN_POPULATION_CAP
    # a yield's remainder and system-spawned work pass
    assert loop._add_agent_concern("remainder", entity="User", skip_recurrence=True) is not None
    assert loop._add_agent_concern("verify", entity="User",
                                   extra_properties={"system_spawned": True}) is not None


# ── near misses ────────────────────────────────────────────────────────

def test_near_miss_rows_and_recurrence(tmp_path):
    from utils.file_utils import append_jsonl
    p = tmp_path / "near_misses.jsonl"
    old = (datetime.now(timezone.utc) - timedelta(days=10)).isoformat()
    for _ in range(3):
        append_jsonl(p, {"note_id": "Note_1", "final": 0.61, "rank": 4, "of": 5}, character="T")
    append_jsonl(p, {"note_id": "Note_2", "final": 0.7, "rank": 4, "of": 5}, character="T")
    append_jsonl(p, {"note_id": "Note_3", "final": 0.9, "rank": 4, "of": 5, "ts": old}, character="T")
    append_jsonl(p, {"note_id": "Note_3", "final": 0.9, "rank": 4, "of": 5, "ts": old}, character="T")
    append_jsonl(p, {"note_id": "Note_3", "final": 0.9, "rank": 4, "of": 5, "ts": old}, character="T")
    assert near_miss_recurrent(p, days=7, min_count=3) == [("Note_1", 3, 0.61)]
    assert near_miss_recurrent(p, days=30, min_count=3)[0][0] == "Note_3"      # older rows count in a wider window
    assert near_miss_recurrent(tmp_path / "absent.jsonl") == []


def test_recall_records_losers_only_when_asked(loop, tmp_path):
    loop._memories_collection_id = "Collection_mem"
    hits = [{"document": f"m{i}", "score": 0.9 - i * 0.05, "metadata": {"source_note_id": f"Note_{i}"}}
            for i in range(6)]
    loop.resource_manager.search_collection = lambda *a, **k: (True, hits, None)
    loop.resource_manager.get_resource = lambda nid: {"properties": {}, "content": nid}
    loop._recency_adjust = lambda s, ca: s
    out = loop._recall("q", k=3)
    assert len(out) == 3 and not (tmp_path / "memory" / "near_misses.jsonl").exists()
    out = loop._recall("q", k=3, record_near_misses=True)
    rows = _rows(tmp_path / "memory" / "near_misses.jsonl")
    assert [r["note_id"] for r in rows] == ["Note_3", "Note_4", "Note_5"]
    assert rows[0]["rank"] == 4 and rows[0]["of"] == 6
    assert out[0][5]["runner_up"]["note_id"] == "Note_3"


# ── candidates ─────────────────────────────────────────────────────────

def test_candidates_are_logged_and_recur_without_creating(loop, tmp_path, monkeypatch):
    # Both modes are exercised; the flag's module default is not assumed.
    monkeypatch.setattr(C, "_CANDIDATES_LIVE", False)
    created = []
    monkeypatch.setattr(loop, "_add_agent_concern", lambda *a, **k: created.append(a) or "Note_x")
    cand = [{"text": "the async pipeline keeps needing a diagram", "why": "third module today",
             "source": "trace", "sign": "aversive", "affectable": True},
            {"text": "", "why": "empty text is dropped"}]
    for turn in (1, 2):
        assert loop._log_concern_candidates(cand, turn, "User") == 1
    rows = _rows(tmp_path / "memory" / C._CANDIDATES_FILE)
    assert len(rows) == 2 and all("would_promote" not in r for r in rows)
    assert loop._log_concern_candidates(cand, 3, "User") == 1
    rows = _rows(tmp_path / "memory" / C._CANDIDATES_FILE)
    wp = [r for r in rows if "would_promote" in r]
    assert len(wp) == 1 and wp[0]["count"] == 3 and wp[0]["live"] is False
    assert created == []                                            # shadow: nothing created
    assert C.candidate_recurrence(tmp_path / "memory" / C._CANDIDATES_FILE,
                                  "THE async  pipeline keeps needing a diagram") == 3
    # by meaning: a rephrasing counts when the embedder says so
    fake = lambda texts: [[1.0, 0.0] if "diagram" in t else [0.0, 1.0] for t in texts]
    assert C.candidate_recurrence(tmp_path / "memory" / C._CANDIDATES_FILE,
                                  "modules that need a diagram to explain, again", embed=fake) == 3
    assert C.candidate_recurrence(tmp_path / "memory" / C._CANDIDATES_FILE,
                                  "something else entirely", embed=fake) == 0
    # live: promoted through the normal create path
    monkeypatch.setattr(C, "_CANDIDATES_LIVE", True)
    loop._log_concern_candidates(cand, 4, "User")
    assert created and created[0][0] == cand[0]["text"]


# ── expectations ───────────────────────────────────────────────────────

def test_expect_line_reads_the_last_line_while_fresh():
    now = datetime.now(timezone.utc)
    fresh = (now - timedelta(hours=2)).isoformat()
    stale = (now - timedelta(hours=30)).isoformat()
    agent = {"wip": "findings so far\nNEXT: look again\nEXPECT: input voltage stays above 53 V",
             "wip_updated_at": fresh, "rhythm_hours": 24}
    assert ChatLoop.expect_line(agent, "agent", now)[0] == "input voltage stays above 53 V"
    agent["wip_updated_at"] = stale
    assert ChatLoop.expect_line(agent, "agent", now) is None            # older than its rhythm
    user = {"context": "Bruce is deciding.\nEXPECT: still undecided next time",
            "context_updated_at": stale}
    assert ChatLoop.expect_line(user, "user", now)[0] == "still undecided next time"   # a week for users
    assert ChatLoop.expect_line({"wip": "no line here", "wip_updated_at": fresh}, "agent", now) is None
    assert ChatLoop.expect_line({"context": "EXPECT: x"}, "user", now) is None         # no stamp


def test_expectation_checks_log_in_shadow_and_bump_live(loop, tmp_path, monkeypatch):
    now = datetime.now(timezone.utc).isoformat()
    a = _note(loop, "Collection_ac", {"kind": "agent_concern", "status": "active", "activation": 0.4,
                                      "instruction": "x", "rhythm_hours": 24,
                                      "wip": "w\nEXPECT: the port is closed", "wip_updated_at": now}, "agent one")
    u = _note(loop, "Collection_uc", {"kind": "user_concern", "status": "active", "strength": 0.5,
                                      "context": "c\nEXPECT: still waiting", "context_updated_at": now}, "user one")
    shown = {"agent one": (a, "agent", {}, "the port is closed", 0.1),
             "user one": (u, "user", {}, "still waiting", 0.1)}
    fine = [{"concern": "agent one", "verdict": "held", "direction": "neutral", "evidence": "closed"},
            {"concern": "user one", "verdict": "held", "direction": "neutral", "evidence": "waiting"},
            {"concern": "nobody", "verdict": "held"}]
    checks = [{"concern": "agent one", "verdict": "violated", "direction": "aversive", "evidence": "port open"},
              {"concern": "user one", "verdict": "violated", "direction": "aversive", "evidence": "they moved on"}]
    assert loop._log_expectation_checks(fine, shown, 7) == 2
    rows = _rows(tmp_path / "memory" / C._EXPECTATIONS_FILE)
    assert [r["kind"] for r in rows] == ["agent", "user"] and rows[0]["live"] is False
    get = loop.resource_manager.get_resource
    assert loop._log_expectation_checks(checks, shown, 8) == 2            # shadow: logged, no bump
    assert get(a)["properties"]["activation"] == 0.4 and get(u)["properties"]["strength"] == 0.5
    monkeypatch.setattr(C, "_EXPECTATIONS_LIVE", True)
    loop._log_expectation_checks(fine, shown, 9)                          # clear the repeat
    loop._log_expectation_checks(checks + checks, shown, 10)              # duplicates in one turn: one bump each
    assert get(a)["properties"]["activation"] == pytest.approx(0.4 + C._AGENT_CONCERN_BUMP_AMOUNT)
    assert get(u)["properties"]["strength"] == pytest.approx(0.5 + C._USER_CONCERN_BUMP_AMOUNT)
    # within the turn the repeat wrote no rows: same expectation, verdict and direction as the last row
    assert len(_rows(tmp_path / "memory" / C._EXPECTATIONS_FILE)) == 8      # two per turn 7-10


def test_expectation_rows_are_written_on_change_only(loop, tmp_path, monkeypatch):
    now = datetime.now(timezone.utc).isoformat()
    a = _note(loop, "Collection_ac", {"kind": "agent_concern", "status": "active", "activation": 0.4,
                                      "instruction": "x", "rhythm_hours": 24,
                                      "wip": "w\nEXPECT: quiet", "wip_updated_at": now}, "agent one")
    shown = {"agent one": (a, "agent", {}, "quiet", 0.1)}
    unclear = [{"concern": "agent one", "verdict": "unclear", "direction": "neutral", "evidence": "e1"}]
    assert loop._log_expectation_checks(unclear, shown, 1) == 1
    assert loop._log_expectation_checks(unclear, shown, 2) == 0            # same answer, different evidence: skipped
    held = [{"concern": "agent one", "verdict": "held", "direction": "neutral", "evidence": "e3"}]
    assert loop._log_expectation_checks(held, shown, 3) == 1               # verdict changed
    assert loop._log_expectation_checks(held, shown, 4) == 0
    shown2 = {"agent one": (a, "agent", {}, "louder", 0.1)}               # expectation rewritten
    assert loop._log_expectation_checks(held, shown2, 5) == 1
    rows = _rows(tmp_path / "memory" / C._EXPECTATIONS_FILE)
    assert [(r["turn_seq"], r["verdict"], r["expect"]) for r in rows] == \
        [(1, "unclear", "quiet"), (3, "held", "quiet"), (5, "held", "louder")]
    # live: the first violation bumps; the same violation next turn writes
    # no row and, being a repeat, bumps nothing
    monkeypatch.setattr(C, "_EXPECTATIONS_LIVE", True)
    bad = [{"concern": "agent one", "verdict": "violated", "direction": "aversive", "evidence": "e6"}]
    loop._log_expectation_checks(bad, shown2, 6)
    loop._log_expectation_checks(bad, shown2, 7)
    assert loop.resource_manager.get_resource(a)["properties"]["activation"] == \
        pytest.approx(0.4 + C._AGENT_CONCERN_BUMP_AMOUNT)
    assert len(_rows(tmp_path / "memory" / C._EXPECTATIONS_FILE)) == 4


def test_wip_prompt_asks_for_the_expect_line(loop):
    import inspect
    src = inspect.getsource(C.ConcernsMixin._update_concern_wip)
    assert "'EXPECT: '" in src and "last line" in src


# ── fire-side expectation checks (2026-09-08) ─────────────────────────

def test_parse_check_line_splits_and_drops():
    parse = ChatLoop.parse_check_line
    wip, check = parse("seen twice\nNEXT: look again\nCHECK: violated; aversive; 50.1 V read\nEXPECT: back in range")
    assert wip == "seen twice\nNEXT: look again\nEXPECT: back in range"
    assert check == {"verdict": "violated", "direction": "aversive", "evidence": "50.1 V read"}
    wip, check = parse("summary\nCheck: Held; Neutral; fine; with; semicolons\nEXPECT: same")
    assert check["verdict"] == "held" and check["evidence"] == "fine; with; semicolons"
    wip, check = parse("summary\nCHECK: maybe\nEXPECT: same")       # malformed: removed, flagged
    assert wip == "summary\nEXPECT: same" and "malformed" in check
    assert parse("summary\nEXPECT: same") == ("summary\nEXPECT: same", None)


def _fire(loop, monkeypatch, prev_wip, rewrite, activation=0.4, seen=None):
    now = datetime.now(timezone.utc).isoformat()
    nid = _note(loop, "Collection_ac", {"kind": "agent_concern", "status": "active", "activation": activation,
                                        "instruction": "x", "rhythm_hours": 1, "content": "watch the volts",
                                        "wip": prev_wip, "wip_updated_at": now}, "watch the volts")
    loop.backend = StubBackend([rewrite])
    if seen is not None:
        real = loop.backend.chat
        loop.backend.chat = lambda messages, **kw: (seen.append(messages[0]["content"]), real(messages, **kw))[1]
    monkeypatch.setattr(loop, "_persist_to_disk", lambda: None, raising=False)
    loop._update_concern_wip(nid, "check volts", [("ACTION", "read"), ("OBSERVATION", "50.1 V")], "low", "respond")
    return nid


def test_fire_check_is_asked_for_logged_and_stripped(loop, tmp_path, monkeypatch):
    seen = []
    nid = _fire(loop, monkeypatch, "seen once\nEXPECT: volts stay above 51.5",
                "seen twice\nCHECK: violated; aversive; 50.1 V this fire\nEXPECT: back above 51.5 next hour", seen=seen)
    assert "'CHECK: '" in seen[0]                                          # asked because an EXPECT line existed
    props = loop.resource_manager.get_resource(nid)["properties"]
    assert props["wip"] == "seen twice\nEXPECT: back above 51.5 next hour"
    rows = _rows(tmp_path / "memory" / C._EXPECTATIONS_FILE)
    assert len(rows) == 1 and rows[0]["kind"] == "fire" and rows[0]["verdict"] == "violated"
    assert rows[0]["expect"] == "volts stay above 51.5" and rows[0]["concern"] == "watch the volts"
    assert rows[0]["live"] is False and props["activation"] == 0.4          # shadow: no bump


def test_fire_check_bumps_when_live_and_skips_without_prior_expect(loop, tmp_path, monkeypatch):
    monkeypatch.setattr(C, "_EXPECTATIONS_LIVE", True)
    nid = _fire(loop, monkeypatch, "seen once\nEXPECT: volts stay above 51.5",
                "seen twice\nCHECK: violated; aversive; 50.1 V\nEXPECT: back above 51.5")
    props = loop.resource_manager.get_resource(nid)["properties"]
    assert props["activation"] == pytest.approx(0.4 + C._AGENT_CONCERN_BUMP_AMOUNT)
    # held: logged, no bump
    nid2 = _fire(loop, monkeypatch, "w\nEXPECT: fine", "w2\nCHECK: held; neutral; fine\nEXPECT: fine")
    assert loop.resource_manager.get_resource(nid2)["properties"]["activation"] == 0.4
    # no EXPECT on the previous WIP: nothing asked, nothing logged, a stray CHECK line is kept as text
    seen = []
    nid3 = _fire(loop, monkeypatch, "no expectation yet", "w3\nEXPECT: first one", seen=seen)
    assert "'CHECK: '" not in seen[0]
    assert loop.resource_manager.get_resource(nid3)["properties"]["wip"] == "w3\nEXPECT: first one"
    rows = _rows(tmp_path / "memory" / C._EXPECTATIONS_FILE)
    assert [r["verdict"] for r in rows] == ["violated", "held"] and all(r["live"] for r in rows)


def test_fire_check_runs_on_the_root_fire_only(loop, tmp_path, monkeypatch):
    """A yield continuation is a hop of the same fire minutes later; its
    WIP rewrite must not test the EXPECT the previous hop just wrote."""
    now = datetime.now(timezone.utc).isoformat()
    root = _note(loop, "Collection_ac", {"kind": "agent_concern", "status": "active", "activation": 0.4,
                                         "instruction": "x", "rhythm_hours": 1, "content": "watch the volts",
                                         "wip": "seen once\nEXPECT: volts stay above 51.5",
                                         "wip_updated_at": now}, "watch the volts")
    hop = _note(loop, "Collection_ac", {"kind": "agent_concern", "status": "active", "activation": 0.7,
                                        "instruction": "carry on", "successor_of": root},
                "watch the volts — continuation (depth 1)")
    seen = []
    loop.backend = StubBackend(["seen twice\nCHECK: violated; aversive; 50.1 V\nEXPECT: back above 51.5"])
    real = loop.backend.chat
    loop.backend.chat = lambda messages, **kw: (seen.append(messages[0]["content"]), real(messages, **kw))[1]
    monkeypatch.setattr(loop, "_persist_to_disk", lambda: None, raising=False)
    monkeypatch.setattr(C, "_EXPECTATIONS_LIVE", True)
    loop._update_concern_wip(hop, "carry on", [("ACTION", "read")], "same", "respond")
    assert "'CHECK: '" not in seen[0]                                      # not asked on a hop
    props = loop.resource_manager.get_resource(root)["properties"]
    assert props["wip"].startswith("seen twice")                            # the root's WIP still moves
    assert "CHECK:" in props["wip"]                                         # a stray CHECK line is kept as text
    assert props["activation"] == 0.4                                       # no bump
    assert not (tmp_path / "memory" / C._EXPECTATIONS_FILE).exists()


def test_repeat_violation_does_not_bump(loop, tmp_path, monkeypatch):
    """Two violations in a row mean the expectation is wrong: the second
    is logged but does not bump. Fire-side and post-turn paths alike."""
    monkeypatch.setattr(C, "_EXPECTATIONS_LIVE", True)
    nid = _fire(loop, monkeypatch, "w\nEXPECT: a saying on enquiry",
                "w2\nCHECK: violated; aversive; it was on grace\nEXPECT: a saying on enquiry")
    get = loop.resource_manager.get_resource
    assert get(nid)["properties"]["activation"] == pytest.approx(0.4 + C._AGENT_CONCERN_BUMP_AMOUNT)
    props = get(nid)["properties"]
    loop.backend = StubBackend(["w3\nCHECK: violated; aversive; on surrender this time\nEXPECT: a saying on enquiry"])
    loop._update_concern_wip(nid, "deliver", [("ACTION", "fetch")], "Talk 244", "respond")
    assert props["activation"] == pytest.approx(0.4 + C._AGENT_CONCERN_BUMP_AMOUNT)   # logged, not bumped
    rows = _rows(tmp_path / "memory" / C._EXPECTATIONS_FILE)
    assert [r["verdict"] for r in rows] == ["violated", "violated"]
    # a held check in between clears the repeat
    loop.backend = StubBackend(["w4\nCHECK: held; neutral; on enquiry\nEXPECT: a saying on enquiry"])
    loop._update_concern_wip(nid, "deliver", [("ACTION", "fetch")], "Talk 9", "respond")
    loop.backend = StubBackend(["w5\nCHECK: violated; aversive; on grace again\nEXPECT: a saying on enquiry"])
    loop._update_concern_wip(nid, "deliver", [("ACTION", "fetch")], "Talk 10", "respond")
    assert props["activation"] == pytest.approx(0.4 + 2 * C._AGENT_CONCERN_BUMP_AMOUNT)
    # post-turn path: the concern's last row is a violation, so no bump
    shown = {"watch the volts": (nid, "agent", {}, "a saying on enquiry", 0.5)}
    checks = [{"concern": "watch the volts", "verdict": "violated", "direction": "aversive", "evidence": "grace"}]
    loop._log_expectation_checks(checks, shown, 9)
    assert props["activation"] == pytest.approx(0.4 + 2 * C._AGENT_CONCERN_BUMP_AMOUNT)


def test_fire_check_malformed_or_missing_is_dropped_and_wip_kept(loop, tmp_path, monkeypatch):
    nid = _fire(loop, monkeypatch, "w\nEXPECT: fine", "w2\nCHECK: dunno\nEXPECT: fine")
    assert loop.resource_manager.get_resource(nid)["properties"]["wip"] == "w2\nEXPECT: fine"
    nid2 = _fire(loop, monkeypatch, "w\nEXPECT: fine", "w3\nEXPECT: fine")   # model ignored the ask
    assert loop.resource_manager.get_resource(nid2)["properties"]["wip"] == "w3\nEXPECT: fine"
    assert not (tmp_path / "memory" / C._EXPECTATIONS_FILE).exists()


# ── companion headings ─────────────────────────────────────────────────

def test_companion_sections_carry_the_two_new_headings(loop):
    text = ("COMPANION MODEL: User\n\nHOW THEY THINK & WORK:\nfast\n\n"
            "RELIABILITY:\nstates confidence plainly; corrected once\n\n"
            "SHARED GROUND:\nknows the three channels cold\n\nON THEIR MIND:\nthe cat\n")
    out = loop._companion_sections(text, ("RELIABILITY", "SHARED GROUND"))
    assert out == "RELIABILITY: states confidence plainly; corrected once\n\nSHARED GROUND: knows the three channels cold"
    assert "RELIABILITY" in ChatLoop._COMPANION_HEADINGS and "SHARED GROUND" in ChatLoop._COMPANION_HEADINGS
    import discourse
    t = discourse.COMPANION_UPDATE_TEMPLATE
    assert t.index("RELIABILITY:") < t.index("SHARED GROUND:") < t.index("ON THEIR MIND:")


# ── the whole reflection path, stubbed model ───────────────────────────

def test_reflection_shows_the_sections_and_logs_the_shadow_rows(loop, tmp_path, monkeypatch):
    """Drives _reflect_and_remember with a fake dialog, a reasoning record
    carrying two thoughts, one agent concern with a fresh EXPECT line, and a
    backend that answers with candidates and a check. Asserts what the model
    was shown and what landed in the two shadow logs — and that nothing was
    created or bumped."""
    now = datetime.now(timezone.utc).isoformat()
    a = _note(loop, "Collection_ac", {"kind": "agent_concern", "status": "active", "activation": 0.3,
                                      "instruction": "look", "rhythm_hours": 24, "content": "watch the port",
                                      "wip": "seen once\nEXPECT: the port stays closed", "wip_updated_at": now},
              "watch the port")
    loop._memories_collection_id = "Collection_mem"
    loop._companion_state = {}
    loop._build_dialog = lambda entity, limit: [{"source": "User", "text": "hello"},
                                                {"source": "Tester", "text": "hi"}]
    loop._recall = lambda *a, **k: []
    loop._load_pending_fire_outcomes = lambda: []
    loop._load_reasoning_records = lambda: [
        {"turn_seq": 9, "source": "User", "autonomous": False,
         "working_log": '--- iter 1 ---\nACTION: {"thought": "that decay clock is the same shape as WHALE", "tool": "respond", "text": "x"}\n'
                        'ACTION: {"thought": "the port was open in the trace", "tool": "respond", "text": "y"}'}]
    loop._near_miss_recurrent = lambda **k: [("Note_m", 3, "a memory that keeps coming close")]
    loop._remember = lambda *a, **k: True
    loop._record_capability_gap = lambda g: None
    created = []
    monkeypatch.setattr(loop, "_add_agent_concern", lambda *a, **k: created.append(a) or None)
    seen = {}

    def gen(messages, **kw):
        seen["sys"] = messages[0]["content"]; seen["user"] = messages[1]["content"]
        from types import SimpleNamespace
        return SimpleNamespace(success=True, error=None, text={
            "frame": "none", "memories": [], "user_concerns": [], "user_concerns_updated": [],
            "user_concerns_closed": [], "agent_concerns": [], "agent_concerns_closed": [],
            "capability_gap": None,
            "candidates": [{"text": "the decay clock shares a shape with WHALE", "why": "noticed mid-reasoning",
                            "source": "trace", "sign": "appetitive", "affectable": True}],
            "expectation_checks": [{"concern": "watch the port", "verdict": "violated",
                                    "direction": "aversive", "evidence": "the port was open"}]})
    loop._llm_generate = gen
    loop._reflect_and_remember("User")
    assert "STAGE 7" in seen["sys"] and "STAGE 8" in seen["sys"]
    u = seen["user"]
    assert "## What Tester thought while reasoning this turn" in u and "same shape as WHALE" in u
    assert "## Memories that came close this week and were not used" in u and "(3×)" in u
    assert "EXPECT: the port stays closed" in u and f"0 of {C._AGENT_CONCERN_POPULATION_CAP} active" not in u
    assert "1 of " in u and "candidates" in u.split("Return the JSON")[-1] and "expectation_checks" in u.split("Return the JSON")[-1]
    cands = _rows(tmp_path / "memory" / C._CANDIDATES_FILE)
    assert len(cands) == 1 and cands[0]["turn_seq"] == 9 and cands[0]["sign"] == "appetitive"
    checks = _rows(tmp_path / "memory" / C._EXPECTATIONS_FILE)
    assert len(checks) == 1 and checks[0]["verdict"] == "violated" and checks[0]["concern_id"] == a
    assert created == []
    assert loop.resource_manager.get_resource(a)["properties"]["activation"] == 0.3


# ── self-minted concerns (the `mint` action) ───────────────────────────

def _user_turn(loop):
    loop._current_turn = {"kind": "user", "source": "User"}
    loop._minted_this_turn = False
    loop._last_turn_seq = 41


def test_mint_is_refused_outside_a_user_turn_and_twice_in_one(loop, tmp_path, monkeypatch):
    loop._current_turn = {"kind": "autonomous", "source": "Tester"}
    loop._minted_this_turn = False
    assert loop._run_mint("the retry path swallows one error", "check whether it still does", 24).startswith("ERROR")
    _user_turn(loop)
    assert loop._run_mint("", "check", 24).startswith("ERROR")           # both fields needed
    assert loop._run_mint("x", "", 24).startswith("ERROR")
    assert loop._run_mint("the retry path swallows one error", "check whether it still does", 24).startswith("OK")
    assert loop._run_mint("another thing", "check it", 24).startswith("ERROR: one mint per turn")
    assert len(_rows(tmp_path / "memory" / C._MINTS_FILE)) == 1


def test_mint_in_shadow_records_and_creates_nothing(loop, tmp_path, monkeypatch):
    monkeypatch.setattr(C, "_MINT_LIVE", False)
    _user_turn(loop)
    obs = loop._run_mint("the sign error I half-saw in disposition.py", "read the update step and say whether the sign is wrong", 2)
    assert obs.startswith("OK: recorded") and "shadow" in obs and "outside 8-168h" in obs
    rows = _rows(tmp_path / "memory" / C._MINTS_FILE)
    assert rows[0]["text"].startswith("the sign error") and rows[0]["live"] is False
    assert rows[0]["rhythm_requested"] == 2 and rows[0]["rhythm_hours"] == 8   # clamped to the floor
    assert rows[0]["turn_seq"] == 41 and rows[0]["entity"] == "User"
    assert loop.resource_manager.resource_registry["Collection_ac"]["properties"]["content"] == []
    ev = _rows(tmp_path / "autonomy.jsonl")
    assert ev[-1]["event"] == "concern_minted" and ev[-1]["live"] is False


def test_mint_live_creates_a_durable_capped_concern(loop, tmp_path, monkeypatch):
    monkeypatch.setattr(C, "_MINT_LIVE", True)
    _user_turn(loop)
    obs = loop._run_mint("the sign error I half-saw in disposition.py", "read the update step and say whether the sign is wrong", 500)
    assert obs.startswith("OK: concern Note_") and "recorded as 168h" in obs
    ids = loop.resource_manager.resource_registry["Collection_ac"]["properties"]["content"]
    assert len(ids) == 1
    note = loop.resource_manager.get_resource(ids[0]); p = note["properties"]
    assert p["content"] == "the sign error I half-saw in disposition.py"        # no prefix
    assert p["category"] == "durable" and p["provenance"] == "inferred" and p["seed"] is False
    assert p["rhythm_hours"] == 168 and p["rhythm_source"] == "self" and p["self_minted"] is True
    assert p["activation"] == 0.0 and p["instruction"].startswith("read the update step")
    assert "system_spawned" not in p
    # at the cap: refused, said so, still recorded
    _user_turn(loop)
    monkeypatch.setattr(loop, "_active_nonseed_agent_count", lambda: C._AGENT_CONCERN_POPULATION_CAP)
    obs = loop._run_mint("something else entirely", "check it", 24)
    assert obs.startswith("EMPTY") and "cap" in obs
    assert len(loop.resource_manager.resource_registry["Collection_ac"]["properties"]["content"]) == 1
    assert len(_rows(tmp_path / "memory" / C._MINTS_FILE)) == 2
    # a near-twin revives the existing one rather than duplicating
    _user_turn(loop)
    monkeypatch.setattr(loop, "_active_nonseed_agent_count", lambda: 1)
    loop._find_similar_concern = lambda text, cid: ids[0]
    assert loop._run_mint("that sign error in disposition.py", "check again", 24).startswith("OK: concern " + ids[0])
    assert len(loop.resource_manager.resource_registry["Collection_ac"]["properties"]["content"]) == 1


def test_mint_is_offered_on_user_turns_only(loop):
    loop._discovered_tools = {}
    loop._omitted_tools = []
    loop._peers = []
    loop._get_external_repo = lambda: None
    loop._current_turn = {"kind": "user", "source": "User"}
    cat = loop._build_react_tool_catalog()
    assert '"tool": "mint"' in cat and cat.index('"mint"') < cat.index('"display"')
    loop._current_turn = {"kind": "autonomous", "source": "Tester"}
    assert '"tool": "mint"' not in loop._build_react_tool_catalog()
