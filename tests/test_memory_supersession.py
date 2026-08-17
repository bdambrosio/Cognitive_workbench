"""Unit tests for write-time memory supersession.

Reflection cannot dedup against memories it was never shown, so the gate
sits at the write: a near-identical memory is classified restatement /
revision / distinct, and only the last two are stored. Revisions retire
what they replace via a `superseded_by` property, which _recall honours —
there is no removal path from the collection's chunk store, so the
dereference _recall already does is the only place retirement can bite.

Same conventions as test_concern_dynamics: object.__new__(ChatLoop), a
stubbed backend and semantic search, a throwaway scratch world — never
live world state.
"""

import json
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


class StubBackend:
    """Returns a queue of canned verdicts (last one sticks); counts calls."""

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
    world = f"pytest_scratch_{uuid.uuid4().hex[:8]}"
    mgr = InfospaceResourceManager(world, world_config={"world_name": world})
    inst = object.__new__(ChatLoop)
    inst.character_name = "Tester"
    inst.resource_manager = mgr
    inst._faiss_lock = threading.Lock()
    inst._turn_seq = 7
    # Collection injected directly so create_collection's auto-indexing
    # never loads the embedder; semantic search is stubbed per test.
    mgr.resource_registry["Collection_mem"] = {
        "name": "Collection_mem",
        "type": mgr.resource_types.Collection,
        "location": (0, 0),
        "description": "test memories",
        "remove_on_take": False,
        "properties": {"content": [], "format": "list",
                       "collection_name": "memories", "kind": "memories"},
    }
    inst._memories_collection_id = "Collection_mem"
    inst.backend = StubBackend(["distinct"])
    inst._memories_log_path = lambda: tmp_path / "memories.jsonl"
    yield inst
    shutil.rmtree(Path(__file__).parent.parent / "scenarios" / world,
                  ignore_errors=True)


def seed_memory(loop, text, entity=""):
    """A memory already in the store, as _remember would have left it."""
    ok, nid, err, _ = loop.resource_manager.create_note(
        "Tester", text, "text", "pytest", entity, "",
        {"kind": "memory", "category": "fact", "polarity": "positive",
         "entity": entity, "exclude_from_index": True})
    assert ok, err
    return nid


def hit(note_id, score=0.9):
    """Shape search_collection returns: a chunk with source-note metadata."""
    return (True, [{"document": "chunk", "score": score,
                    "metadata": {"source_note_id": note_id}}], None)


def events(loop):
    path = loop._memories_log_path()
    if not path.exists():
        return []
    return [json.loads(l) for l in path.read_text().splitlines() if l.strip()]


def props(loop, note_id):
    return loop.resource_manager.get_resource(note_id).get("properties", {})


# ── the gate ───────────────────────────────────────────────────────────

def test_restatement_is_not_stored(loop):
    prior = seed_memory(loop, "User's name is Bruce.")
    loop.resource_manager.search_collection = lambda *a, **k: hit(prior)
    loop.backend = StubBackend(["restatement"])

    got = loop._remember("The user's name is Bruce.")

    assert got == prior, "should hand back the memory we already hold"
    assert len(loop.resource_manager.get_resource(
        "Collection_mem")["properties"]["content"]) == 0
    assert [e["event"] for e in events(loop)] == ["restatement_skipped"]


def test_revision_is_stored_and_retires_what_it_replaces(loop):
    prior = seed_memory(loop, "User is 79 years old")
    loop.resource_manager.search_collection = lambda *a, **k: hit(prior)
    loop.backend = StubBackend(["revision"])

    new_id = loop._remember("User is 80 years old")

    assert new_id and new_id != prior
    assert props(loop, prior)["superseded_by"] == new_id
    assert props(loop, prior)["superseded_at"]
    assert "superseded_by" not in props(loop, new_id)
    ev = events(loop)[-1]
    assert ev["event"] == "write" and ev["supersedes"] == prior


def test_distinct_keeps_both(loop):
    prior = seed_memory(loop, "User plays Satisfactory with his grandson.")
    loop.resource_manager.search_collection = lambda *a, **k: hit(prior)
    loop.backend = StubBackend(["distinct"])

    new_id = loop._remember("User's grandson is named Owen.")

    assert new_id and new_id != prior
    assert "superseded_by" not in props(loop, prior)
    assert events(loop)[-1].get("supersedes") is None


def test_a_novel_memory_costs_no_llm_call(loop):
    """The common path is a miss; the probe must not run on every write."""
    loop.resource_manager.search_collection = lambda *a, **k: (True, [], None)
    loop.backend = StubBackend(["restatement"])  # would skip, if consulted

    new_id = loop._remember("User just bought a Tesla Model Y.")

    assert new_id
    assert loop.backend.calls == 0


def test_an_already_superseded_memory_is_not_matched_against(loop):
    """A chain points at the live memory, not a retired link in it."""
    stale = seed_memory(loop, "User is 79 years old")
    live = seed_memory(loop, "User is 80 years old")
    props(loop, stale)["superseded_by"] = live
    loop.resource_manager.search_collection = lambda *a, **k: hit(stale)
    loop.backend = StubBackend(["restatement"])

    new_id = loop._remember("User is 81 years old")

    assert new_id not in (stale, live), "should write rather than match a tombstone"
    assert loop.backend.calls == 0


def test_a_memory_about_another_subject_is_never_a_revision(loop):
    """Regression, 2026-08-17. "The user's name is Bruce" (learned with
    Sentinel) and "The user's name is Jack" (learned with Jack) are one
    word apart and score as the same fact. A cleanup pass asked the judge
    which superseded which, it picked the newer, and the agent was left
    believing the user had been renamed. Entity partitions before any
    judgement is sought."""
    bruce = seed_memory(loop, "The user's name is Bruce.", entity="Sentinel")
    loop.resource_manager.search_collection = lambda *a, **k: hit(bruce)
    loop.backend = StubBackend(["revision"])   # what the judge actually said

    new_id = loop._remember("The user's name is Jack.", entity="Jack")

    assert new_id and new_id != bruce
    assert "superseded_by" not in props(loop, bruce), \
        "a fact about one person must not retire the same fact about another"
    assert loop.backend.calls == 0, "the judge should never have been asked"


def test_same_entity_still_supersedes(loop):
    prior = seed_memory(loop, "User is 79 years old", entity="User")
    loop.resource_manager.search_collection = lambda *a, **k: hit(prior)
    loop.backend = StubBackend(["revision"])

    new_id = loop._remember("User is 80 years old", entity="User")

    assert props(loop, prior)["superseded_by"] == new_id


# ── failing open ───────────────────────────────────────────────────────

def test_probe_failure_keeps_both(loop):
    prior = seed_memory(loop, "User is 79 years old")
    loop.resource_manager.search_collection = lambda *a, **k: hit(prior)

    class Boom:
        calls = 0

        def chat(self, *a, **k):
            raise RuntimeError("backend down")

    loop.backend = Boom()
    new_id = loop._remember("User is 80 years old")

    assert new_id, "a probe outage must not swallow the memory"
    assert "superseded_by" not in props(loop, prior)


def test_off_format_verdict_keeps_both(loop):
    prior = seed_memory(loop, "User is 79 years old")
    loop.resource_manager.search_collection = lambda *a, **k: hit(prior)
    loop.backend = StubBackend(["I think these are much the same, really"])

    new_id = loop._remember("User is 80 years old")

    assert new_id
    assert "superseded_by" not in props(loop, prior)


def test_search_failure_falls_through_to_a_normal_write(loop):
    def boom(*a, **k):
        raise RuntimeError("faiss unavailable")

    loop.resource_manager.search_collection = boom
    new_id = loop._remember("User uses Linux.")

    assert new_id
    assert events(loop)[-1]["event"] == "write"


# ── the retirement has to bite at recall ───────────────────────────────

def test_recall_skips_superseded_memories(loop):
    stale = seed_memory(loop, "User is 79 years old")
    live = seed_memory(loop, "User is 80 years old")
    props(loop, stale)["superseded_by"] = live

    # Both chunks still sit in the collection store — nothing removes
    # them — so recall must filter on the note it dereferences anyway.
    loop.resource_manager.search_collection = lambda *a, **k: (True, [
        {"document": "User is 79 years old", "score": 0.95,
         "metadata": {"source_note_id": stale}},
        {"document": "User is 80 years old", "score": 0.90,
         "metadata": {"source_note_id": live}},
    ], None)

    out = loop._recall("how old is the user", k=3)

    assert [t for t, *_ in out] == ["User is 80 years old"]


def test_recall_skips_memories_retired_as_false(loop):
    """A memory withdrawn with nothing to replace it — the peer-agent
    misattribution "The user's name is Jack" — has no supersessor to point
    at, so it carries `retired` instead. Recall must honour both."""
    wrong = seed_memory(loop, "The user's name is Jack.", entity="Jack")
    props(loop, wrong)["retired"] = True
    loop.resource_manager.search_collection = lambda *a, **k: (True, [
        {"document": "The user's name is Jack.", "score": 0.99,
         "metadata": {"source_note_id": wrong}},
    ], None)

    assert loop._recall("what is the user called", k=3) == []


def test_a_retired_memory_is_not_matched_against_at_write(loop):
    retired = seed_memory(loop, "The user's name is Jack.", entity="Jack")
    props(loop, retired)["retired"] = True
    loop.resource_manager.search_collection = lambda *a, **k: hit(retired)
    loop.backend = StubBackend(["restatement"])

    new_id = loop._remember("The user's name is Jack.", entity="Jack")

    assert new_id != retired, "a withdrawn memory must not absorb a new write"
    assert loop.backend.calls == 0
