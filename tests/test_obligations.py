"""Unit tests for obligation concerns — the agent_concern subtype whose
satisfaction condition is "I have reported to X", not "I have thought
about this".

Covers the three things that distinguish an obligation from an ordinary
concern: it cannot be retired by housekeeping, it goes overdue instead of
being swept, and once reported it stops firing and waits for the
principal. The shape under test is the 2026-08-20 failure — a directive
answered with four options and a "tell me which pulls" that never came
back — which had no representation at all before this.

Uses object.__new__(ChatLoop) with only the attributes the methods under
test touch and a throwaway scratch world — never live world state.
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

from chat.chat_loop import ChatLoop, _AGENT_CONCERN_FIRE_THRESHOLD
from infospace_resource_manager import InfospaceResourceManager


def _collection(mgr, cid, kind):
    mgr.resource_registry[cid] = {
        "name": cid,
        "type": mgr.resource_types.Collection,
        "location": (0, 0),
        "description": f"test {kind}",
        "remove_on_take": False,
        "properties": {"content": [], "format": "list",
                       "collection_name": kind, "kind": kind},
    }
    return cid


@pytest.fixture
def loop(tmp_path):
    """Minimal ChatLoop instance over a scratch world. Real (registry-
    injected) collections so the sweep has something to walk, but no
    auto-indexing, so the embedder never loads."""
    world = f"pytest_scratch_{uuid.uuid4().hex[:8]}"
    mgr = InfospaceResourceManager(world, world_config={"world_name": world})
    inst = object.__new__(ChatLoop)
    inst.character_name = "Tester"
    inst.resource_manager = mgr
    inst._faiss_lock = threading.Lock()
    inst._agent_concerns_collection_id = _collection(
        mgr, "Collection_ac", "agent_concerns")
    inst._user_concerns_collection_id = _collection(
        mgr, "Collection_uc", "user_concerns")
    inst._turn_seq = 2887
    inst._autonomy_log_path = lambda: tmp_path / "autonomy.jsonl"
    yield inst
    scenario_dir = Path(__file__).parent.parent / "scenarios" / world
    shutil.rmtree(scenario_dir, ignore_errors=True)


def props_of(loop, nid):
    return (loop.resource_manager.get_resource(nid) or {}).get("properties") or {}


def read_events(tmp_path):
    p = tmp_path / "autonomy.jsonl"
    if not p.exists() or not p.read_text().strip():
        return []
    return [json.loads(l) for l in p.read_text().strip().splitlines()]


def make_obligation(loop, text="Bruce owes a pick among four revenue options.",
                    instruction=None, owed_to="Bruce", report_by_hours=None):
    return loop._add_agent_concern(
        text, entity=owed_to, instruction=instruction, owed_to=owed_to,
        report_by_hours=report_by_hours)


def backdate(loop, nid, hours, field="owed_at"):
    """Move a timestamp back so the sweep sees an aged obligation."""
    props = props_of(loop, nid)
    props[field] = (datetime.now(timezone.utc)
                    - timedelta(hours=hours)).isoformat()


def set_due(loop, nid, hours_from_now):
    props_of(loop, nid)["report_by"] = (
        datetime.now(timezone.utc) + timedelta(hours=hours_from_now)).isoformat()


# ── creation ───────────────────────────────────────────────────────────

def test_owed_to_creates_obligation_and_fields_persist(loop):
    """The allowlist trap: extra properties absent from create_note's
    allowed_fields are dropped silently, which is how `system_spawned`
    was inert for its whole life. Every obligation field must survive a
    round trip through the resource manager."""
    nid = make_obligation(loop)
    props = props_of(loop, nid)
    assert props["category"] == "obligation"
    assert props["owed_to"] == "Bruce"
    assert props["awaiting"] == "me"
    assert props["owed_turn"] == 2887
    assert props["reported_at"] is None
    assert props["overdue_since"] is None
    assert props["report_by"] is None      # this ask named no horizon
    datetime.fromisoformat(props["owed_at"])


def test_report_by_hours_resolves_to_an_absolute_deadline(loop):
    """Reflection emits a duration because that is what the ask contains;
    storage keeps an absolute time so it reads the same whenever it is
    next looked at."""
    nid = make_obligation(loop, report_by_hours=4)
    due = datetime.fromisoformat(props_of(loop, nid)["report_by"])
    delta = (due - datetime.now(timezone.utc)).total_seconds() / 3600.0
    assert 3.9 < delta < 4.1


def test_sub_hour_deadlines_are_expressible(loop):
    """The rhythm bucket floors at 1h; a five-minute errand must still be
    recordable, and the per-tick sweep can see it come due."""
    nid = make_obligation(loop, report_by_hours=5 / 60)
    due = datetime.fromisoformat(props_of(loop, nid)["report_by"])
    assert 0 < (due - datetime.now(timezone.utc)).total_seconds() < 320


@pytest.mark.parametrize("bad", [None, 0, -3, "soon"])
def test_unusable_horizon_means_no_deadline(loop, bad):
    nid = make_obligation(loop, report_by_hours=bad)
    assert props_of(loop, nid)["report_by"] is None


def test_obligation_category_without_principal_is_demoted(loop):
    """An obligation nobody owes could never be discharged."""
    nid = loop._add_agent_concern(
        "vague sense of duty", category="obligation", owed_to=None)
    props = props_of(loop, nid)
    assert props["category"] == "one_shot"
    assert "owed_to" not in props


def test_obligations_skip_recurrence_merge(loop, monkeypatch):
    """Two requests from the same person about the same topic are two
    debts; merging them forgives the older one."""
    called = []
    monkeypatch.setattr(
        ChatLoop, "_find_similar_concern",
        lambda self, *a, **k: called.append(a) or "Note_existing")
    first = make_obligation(loop, "send me the revenue numbers")
    second = make_obligation(loop, "send me the revenue numbers")
    assert called == []
    assert first != second


# ── the debt cannot be retired by housekeeping ─────────────────────────

@pytest.mark.parametrize(
    "via", ["completed", "superseded", "synth_complete", "depth_cap",
            "stale_sweep"])
def test_satisfy_refused_for_every_via_but_discharge(loop, via):
    nid = make_obligation(loop)
    assert loop._satisfy_agent_concern(nid, via=via) is False
    assert props_of(loop, nid)["status"] == "active"


def test_discharge_is_the_one_route_to_satisfied(loop):
    nid = make_obligation(loop)
    assert loop._satisfy_agent_concern(nid, via="discharged") is True
    assert props_of(loop, nid)["status"] == "satisfied"


def test_stale_sweep_marks_overdue_instead_of_satisfying(loop, tmp_path):
    """The whole of gap 1: an obligation must be able to become overdue.
    The ordinary path here is active → satisfied, which turns a debt back
    into an absence nobody can notice."""
    nid = make_obligation(loop)
    set_due(loop, nid, -1)

    loop._sweep_stale_agent_concerns()

    props = props_of(loop, nid)
    assert props["status"] == "active"
    assert props["overdue_since"]
    ev = [e for e in read_events(tmp_path) if e["event"] == "obligation_overdue"]
    assert len(ev) == 1
    assert ev[0]["owed_to"] == "Bruce"
    assert ev[0]["owed_turn"] == 2887


def test_overdue_stamp_is_idempotent(loop):
    """The rendered age is how long it has been overdue, not how long
    since the last sweep."""
    nid = make_obligation(loop)
    set_due(loop, nid, -1)
    loop._sweep_stale_agent_concerns()
    first = props_of(loop, nid)["overdue_since"]
    loop._sweep_stale_agent_concerns()
    assert props_of(loop, nid)["overdue_since"] == first


def test_deadline_not_yet_reached_is_not_overdue(loop):
    nid = make_obligation(loop, report_by_hours=4)
    loop._sweep_stale_agent_concerns()
    assert props_of(loop, nid)["overdue_since"] is None


def test_an_ask_with_no_horizon_never_goes_overdue(loop):
    """No default clock: an obligation whose request named no deadline
    stays visible and aging rather than alarming on a schedule nobody
    agreed to. Ages far past any plausible default."""
    nid = make_obligation(loop)
    backdate(loop, nid, 24 * 365)

    loop._sweep_stale_agent_concerns()

    props = props_of(loop, nid)
    assert props["overdue_since"] is None
    assert props["status"] == "active"


# ── reporting flips the side; it does not close the debt ───────────────

def test_report_to_principal_flips_awaiting_and_stops_firing(loop, tmp_path):
    nid = make_obligation(loop, instruction="draft the four options")
    props = props_of(loop, nid)
    props["activation"] = 1.0

    assert [f[0] for f in loop._check_and_fire_agent_concerns()] == [nid]

    loop._record_surfaced(nid, "Bruce", "Here are four costed options...")

    props = props_of(loop, nid)
    assert props["awaiting"] == "them"
    assert props["reported_at"]
    assert props["status"] == "active"        # reported ≠ settled
    # Awaiting them, it must not act: proceeding as though the answer
    # arrived is what produced 130 unattended turns.
    assert loop._check_and_fire_agent_concerns() == []
    ev = [e for e in read_events(tmp_path)
          if e["event"] == "obligation_reported"]
    assert len(ev) == 1


def test_report_to_someone_else_does_not_flip(loop):
    nid = make_obligation(loop, instruction="draft the four options")
    loop._record_surfaced(nid, "Jack", "here's what I'm thinking")
    assert props_of(loop, nid)["awaiting"] == "me"


def test_report_clears_the_deadline_and_the_overdue_stamp(loop):
    """The horizon the request named was a horizon on the REPORT — "get
    me the analysis by five" is answered at five whether or not the reply
    comes back. Keeping it would re-alarm a delivered obligation on a
    clock it already answered."""
    nid = make_obligation(loop, report_by_hours=4)
    set_due(loop, nid, -1)
    loop._sweep_stale_agent_concerns()
    assert props_of(loop, nid)["overdue_since"]

    loop._record_surfaced(nid, "Bruce", "here are the options")

    props = props_of(loop, nid)
    assert props["overdue_since"] is None
    assert props["report_by"] is None
    loop._sweep_stale_agent_concerns()
    assert props_of(loop, nid)["overdue_since"] is None


# ── discharge via reflection ───────────────────────────────────────────

def shown(loop, nid):
    props = props_of(loop, nid)
    return [(nid, str(props.get("content", "")), 0.0, props)]


def test_bare_string_settles(loop):
    """The simplest case stays one word."""
    nid = make_obligation(loop)
    text = props_of(loop, nid)["content"]

    assert loop._apply_obligation_updates([text], shown(loop, nid)) == [text]
    assert props_of(loop, nid)["status"] == "satisfied"


def test_state_settled_is_equivalent(loop):
    nid = make_obligation(loop)
    text = props_of(loop, nid)["content"]

    loop._apply_obligation_updates(
        [{"text": text, "state": "settled"}], shown(loop, nid))
    assert props_of(loop, nid)["status"] == "satisfied"


def test_state_reported_flips_without_closing(loop):
    """The user-turn path: a reply that reported but did not settle."""
    nid = make_obligation(loop)
    text = props_of(loop, nid)["content"]

    loop._apply_obligation_updates(
        [{"text": text, "state": "reported"}], shown(loop, nid))

    props = props_of(loop, nid)
    assert props["awaiting"] == "them"
    assert props["status"] == "active"


def test_timeline_agreed_sets_the_deadline_on_the_existing_debt(loop):
    """Obligations skip recurrence merge, so without this path an
    answered "when do you need this?" would mint a second debt beside
    the first instead of filling in the first one's deadline."""
    nid = make_obligation(loop)
    text = props_of(loop, nid)["content"]
    assert props_of(loop, nid)["report_by"] is None

    loop._apply_obligation_updates(
        [{"text": text, "report_by_hours": 6}], shown(loop, nid))

    props = props_of(loop, nid)
    assert props["report_by"]
    assert props["status"] == "active"     # a deadline is not an ending
    loop._sweep_stale_agent_concerns()
    assert props_of(loop, nid)["overdue_since"] is None


def test_state_and_deadline_in_one_entry(loop):
    nid = make_obligation(loop)
    text = props_of(loop, nid)["content"]

    loop._apply_obligation_updates(
        [{"text": text, "state": "reported", "report_by_hours": 6}],
        shown(loop, nid))

    props = props_of(loop, nid)
    assert props["awaiting"] == "them"
    # Reporting clears the deadline it was just given: the report is the
    # thing that horizon was on.
    assert props["report_by"] is None


def test_update_refuses_a_non_obligation(loop):
    """Reflection already has a path for ordinary concerns; a drop and a
    debt paid are different endings."""
    nid = loop._add_agent_concern("track the S&P close", category="durable",
                                  skip_recurrence=True)
    text = props_of(loop, nid)["content"]

    assert loop._apply_obligation_updates([text], shown(loop, nid)) == []
    assert props_of(loop, nid)["status"] == "active"


def test_discharged_obligation_is_not_kept_for_revival(loop):
    nid = make_obligation(loop)
    loop._satisfy_agent_concern(nid, via="discharged")
    assert loop._dead_concern_disposition(
        props_of(loop, nid)) == "discharged_obligation"


# ── surfacing ──────────────────────────────────────────────────────────

def test_obligations_are_exempt_from_the_prompt_budget(loop):
    """An obligation stops gaining activation the moment it stops firing,
    so ranking it by activation drops it off the prompt exactly when it
    has been ignored longest."""
    for i in range(8):
        nid = loop._add_agent_concern(f"loud concern {i}", category="durable",
                                      skip_recurrence=True)
        props_of(loop, nid)["activation"] = 0.9
    owed = make_obligation(loop)

    top = loop._top_active_agent_concerns(n=5)

    assert top[0][0] == owed
    assert len(top) == 6


def test_render_awaiting_them_with_age_and_turn(loop):
    nid = make_obligation(loop)
    loop._record_surfaced(nid, "Bruce", "here are the options")
    backdate(loop, nid, 50, field="reported_at")

    line = ChatLoop._render_obligation_line(props_of(loop, nid))

    assert "Bruce has not come back on it" in line
    assert "2 days" in line
    assert "asked at turn 2887" in line
    assert "no agreed timeline" in line
    assert "OVERDUE" not in line


def test_render_awaiting_me_no_timeline_prompts_asking(loop):
    line = ChatLoop._render_obligation_line(props_of(loop, make_obligation(loop)))
    assert "I owe Bruce a report on this" in line
    assert "no agreed timeline — worth asking what they need" in line
    assert "OVERDUE" not in line


def test_render_due_soon(loop):
    nid = make_obligation(loop, report_by_hours=3)
    line = ChatLoop._render_obligation_line(props_of(loop, nid))
    assert "due in 3h" in line
    assert "OVERDUE" not in line


def test_render_overdue_reports_by_how_much(loop):
    nid = make_obligation(loop, report_by_hours=1)
    set_due(loop, nid, -5)
    loop._sweep_stale_agent_concerns()

    line = ChatLoop._render_obligation_line(props_of(loop, nid))
    assert "OVERDUE by 5h" in line


def test_render_uses_minutes_for_short_horizons(loop):
    nid = make_obligation(loop, report_by_hours=5 / 60)
    line = ChatLoop._render_obligation_line(props_of(loop, nid))
    assert "due in 5 min" in line


def test_render_empty_for_ordinary_concern(loop):
    nid = loop._add_agent_concern("track the S&P close", category="durable",
                                  skip_recurrence=True)
    assert ChatLoop._render_obligation_line(props_of(loop, nid)) == ""
