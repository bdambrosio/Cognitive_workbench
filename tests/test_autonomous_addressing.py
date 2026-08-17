"""Where an autonomous reply is filed, and what does not accelerate a
polled concern.

Both regressions from 2026-08-17, same incident: Jill re-sent the same
Hugging Face paper three times in one day, twice inside an hour.

  1. Her autonomous replies were recorded under her OWN name while every
     fire read the counterpart's thread, so she could not see a word she
     had said unprompted. 1632 auto_says filed under "Jill", zero under
     "User".
  2. Discussing the topic evidence-bumped the papers concern, which then
     surfaced whatever best matched the conversation — the paper already
     under discussion. The bump landed 89 seconds before the fire.
"""

import os
import shutil
import sys
import threading
import uuid
from pathlib import Path

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from chat.chat_loop import ChatLoop
from infospace_resource_manager import InfospaceResourceManager


@pytest.fixture
def loop():
    world = f"pytest_scratch_{uuid.uuid4().hex[:8]}"
    mgr = InfospaceResourceManager(world, world_config={"world_name": world})
    inst = object.__new__(ChatLoop)
    inst.character_name = "Jill"
    inst.resource_manager = mgr
    inst._faiss_lock = threading.Lock()
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
    inst._agent_concerns_collection_id = "Collection_ac"
    yield inst
    shutil.rmtree(Path(__file__).parent.parent / "scenarios" / world,
                  ignore_errors=True)


def make_concern(loop, entity="User", extra=None):
    props = {"kind": "agent_concern", "status": "active", "activation": 0.30,
             "rhythm_hours": 24, "entity": entity, "instruction": "check",
             "exclude_from_index": True}
    props.update(extra or {})
    ok, nid, err, _ = loop.resource_manager.create_note(
        "Jill", "track new papers", "text", "pytest", entity, "", props)
    assert ok, err
    return nid


# ── where an autonomous reply is filed ─────────────────────────────────

def test_a_fire_addresses_the_concerns_counterpart(loop):
    """The reply belongs in the thread it was addressed to, so the NEXT
    fire reading that thread can see it."""
    nid = make_concern(loop, entity="User")
    loop._current_turn = {"autonomous_concern_id": nid}

    assert loop._firing_concern_entity() == "User"


def test_a_fire_for_a_peer_concern_addresses_the_peer(loop):
    """Work raised while talking with Jack resumes against Jack, not the
    user — the reason this is the concern's entity and not a constant."""
    nid = make_concern(loop, entity="Jack")
    loop._current_turn = {"autonomous_concern_id": nid}

    assert loop._firing_concern_entity() == "Jack"


def test_a_fire_with_no_concern_falls_back_to_the_user(loop):
    loop._current_turn = {}
    assert loop._firing_concern_entity() == "User"


# ── what accelerates a concern ─────────────────────────────────────────

def bump_over(loop, nid, text="agent papers and cognitive architecture"):
    loop.resource_manager.search_collection = (
        lambda *a, **k: (True, [{"metadata": {"source_note_id": nid}}], None))
    loop._bump_agent_concerns_on_input(text)
    return loop.resource_manager.get_resource(nid)["properties"]


def test_talking_about_a_polled_topic_does_not_accelerate_it(loop):
    """Hugging Face does not publish a paper because we discussed papers."""
    nid = make_concern(loop, extra={
        "polled": True,
        "triage_verdict": "defer", "triage_reason": "nothing new today"})

    props = bump_over(loop, nid)

    assert props["activation"] == pytest.approx(0.30), "must not be bumped"
    assert props["triage_verdict"] == "defer", \
        "and must not have its defer cache cleared either"


def test_an_unpolled_concern_still_bumps(loop):
    """The user-model reviewer's heat coupling and the capability-gap
    recurrence signal both depend on this; the guard must be narrow."""
    nid = make_concern(loop, extra={
        "user_model_reviewer": True,
        "triage_verdict": "defer", "triage_reason": "nothing new"})

    props = bump_over(loop, nid)

    assert props["activation"] > 0.30
    assert "triage_verdict" not in props, "new evidence reopens the question"


def test_polled_survives_a_note_round_trip(loop):
    """create_note drops extra_props missing from its allowlist, silently.
    That is how `system_spawned` was inert for its whole life — so assert
    the flag is actually on disk, not merely passed in."""
    nid = make_concern(loop, extra={"polled": True})
    assert loop.resource_manager.get_resource(nid)["properties"].get("polled") \
        is True


# ── one resolver, one answer ───────────────────────────────────────────

def test_every_per_counterpart_lookup_agrees(loop):
    """History moved to the concern's entity on 2026-08-16; the companion
    model, the discourse state and the reply filing each kept keying on
    `source`, which on a fire is the character's own name. Three wrong
    answers to one question. They now share a resolver."""
    nid = make_concern(loop, entity="Jack")
    loop._current_turn = {"autonomous_concern_id": nid}
    loop._companion_state = {"Jack": "COMPANION MODEL: Jack", "User": "…"}
    loop._discourse_state = {"Jack": "premises with Jack", "User": "…"}

    assert loop._counterpart_for_turn("Jill") == "Jack"
    assert loop._companion_for_turn("Jill") == ("Jack", "COMPANION MODEL: Jack")
    # and an ordinary turn is still keyed by who is speaking
    assert loop._counterpart_for_turn("User") == "User"
    assert loop._companion_for_turn("User")[0] == "User"


def test_a_fire_with_no_companion_model_gets_nothing(loop):
    nid = make_concern(loop, entity="Nobody")
    loop._current_turn = {"autonomous_concern_id": nid}
    loop._companion_state = {"User": "…"}

    assert loop._companion_for_turn("Jill") == ("Nobody", "")


# ── a silent fire is not a message ─────────────────────────────────────

def test_a_spoken_fire_is_filed_under_the_counterpart(loop):
    nid = make_concern(loop, entity="User")
    loop._current_turn = {"autonomous_concern_id": nid}

    assert loop._reply_recipient("Jill", silent=False) == "User"


def test_a_silent_fire_stays_out_of_the_counterparts_thread(loop):
    """Most fires say nothing — 899 of them on the day this was written,
    against a 20-turn history window. Filing those under the counterpart
    would evict every word the user actually said, which is the failure
    the addressing fix exists to prevent."""
    nid = make_concern(loop, entity="User")
    loop._current_turn = {"autonomous_concern_id": nid}

    assert loop._reply_recipient("Jill", silent=True) == "Jill"


def test_an_ordinary_turn_is_unaffected_either_way(loop):
    loop._current_turn = {}
    assert loop._reply_recipient("User", silent=False) == "User"
    assert loop._reply_recipient("User", silent=True) == "User"


# ── the delivery record ────────────────────────────────────────────────

def test_a_spoken_fire_is_recorded_verbatim(loop):
    nid = make_concern(loop)
    loop._record_surfaced(nid, "User", "I found a paper: HarnessX. It argues …")

    entries = loop.resource_manager.get_resource(nid)["properties"]["surfaced"]
    assert len(entries) == 1
    assert entries[0]["to"] == "User"
    assert entries[0]["text"].startswith("I found a paper: HarnessX")
    assert entries[0]["at"]


def test_the_record_is_bounded_and_keeps_the_recent_end(loop):
    nid = make_concern(loop)
    for i in range(loop._SURFACED_KEEP + 5):
        loop._record_surfaced(nid, "User", f"paper number {i}")

    entries = loop.resource_manager.get_resource(nid)["properties"]["surfaced"]
    assert len(entries) == loop._SURFACED_KEEP
    assert entries[-1]["text"] == f"paper number {loop._SURFACED_KEEP + 4}"


def test_long_replies_are_head_capped(loop):
    nid = make_concern(loop)
    loop._record_surfaced(nid, "User", "TITLE. " + "x" * 5000)

    entry = loop.resource_manager.get_resource(nid)["properties"]["surfaced"][0]
    assert len(entry["text"]) == loop._SURFACED_CHARS
    assert entry["text"].startswith("TITLE."), "the title must survive the cap"


def test_the_block_renders_as_fact_not_prohibition(loop):
    """A status concern should keep reporting while a fault holds, so the
    record states what happened and leaves the policy to the instruction."""
    nid = make_concern(loop)
    loop._record_surfaced(nid, "User", "PV controller is offline.")
    block = loop._surfaced_block(nid)

    assert "PV controller is offline." in block
    assert "not a rule" in block


def test_no_record_no_block(loop):
    assert loop._surfaced_block(make_concern(loop)) == ""


def test_surfaced_survives_a_note_round_trip(loop):
    """The allowlist trap again — assert it is really on disk."""
    ok, nid, err, _ = loop.resource_manager.create_note(
        "Jill", "c", "text", "pytest", "User", "",
        {"kind": "agent_concern", "status": "active",
         "surfaced": [{"at": "2026-08-17T00:00:00+00:00", "to": "User",
                       "text": "hello"}], "exclude_from_index": True})
    assert ok, err
    assert loop.resource_manager.get_resource(nid)["properties"]["surfaced"]
