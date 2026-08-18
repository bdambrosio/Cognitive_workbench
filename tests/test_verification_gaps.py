"""Two gaps exposed by turns 2787-2789 on 2026-08-17.

Jill answered a question about SGLang from one search-web call. The
synthesis invented a version number ("v0.5.17"; the release page said
v0.5.16) inside an otherwise accurate answer. All nine claims graded
`probable` — correctly, they were quoted accurately — so the suspect gate
stood down and nothing was audited. The justify trail, asked later by
hand, had known all along: "Nothing here read the underlying source."

Asked to justify, she then read three GitHub issues directly, ran out of
iterations at 7, and handed on "synthesize the verified bug reports and
provide the final justification". The successor answered "everything is
now grounded in primary source documentation". Three of nine were.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import shutil
import threading
import uuid
from pathlib import Path

import pytest

from chat.chat_loop import ChatLoop
from chat.claims import grade_claim, synthesis_only_claims
from chat.concerns import ConcernsMixin
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
        "name": "Collection_ac", "type": mgr.resource_types.Collection,
        "location": (0, 0), "description": "test", "remove_on_take": False,
        "properties": {"content": [], "format": "list",
                       "collection_name": "agent_concerns",
                       "kind": "agent_concerns"},
    }
    inst._agent_concerns_collection_id = "Collection_ac"
    inst._current_turn = {}
    inst._peers = []
    yield inst
    shutil.rmtree(Path(__file__).parent.parent / "scenarios" / world,
                  ignore_errors=True)


def make_parent(loop):
    ok, nid, err, _ = loop.resource_manager.create_note(
        "Jill", "justify your response", "text", "pytest", "User", "",
        {"kind": "agent_concern", "status": "active", "activation": 0.4,
         "instruction": "audit the claims", "exclude_from_index": True})
    assert ok, err
    return nid


def props(loop, nid):
    return loop.resource_manager.get_resource(nid)["properties"]


def rec(tool_meta):
    return {"tool_meta": tool_meta}


SEARCH = {"$step1": {"tool": "search-web"}}
FETCH = {"$step1": {"tool": "fetch-text"}}


# ── the synthesis-only trigger ─────────────────────────────────────────

def test_a_claim_quoted_only_from_a_summary_is_flagged():
    c = {"claim": "SGLang is currently at v0.5.17.", "grounding": "retrieved",
         "refs": ["$step1"], "quote": "the latest SGLang release is v0.5.17"}
    assert grade_claim(c) == "probable", "the grade is honest; it is not suspect"
    assert synthesis_only_claims(rec(SEARCH), [c]) == [c]


def test_a_claim_read_from_the_document_is_not_flagged():
    c = {"claim": "Issue 33286 is a TypeError.", "grounding": "retrieved",
         "refs": ["$step1"], "quote": "TypeError: 'NoneType' object"}
    assert synthesis_only_claims(rec(FETCH), [c]) == []


def test_one_direct_source_among_several_is_enough():
    c = {"claim": "x", "grounding": "retrieved",
         "refs": ["$step1", "$step2"], "quote": "q"}
    meta = {"$step1": {"tool": "search-web"}, "$step2": {"tool": "fetch-text"}}
    assert synthesis_only_claims(rec(meta), [c]) == []


def test_an_unquoted_claim_is_left_to_the_suspect_gate():
    """No quote means nothing was mediated INTO the claim; that is the
    other gate's business and double-flagging would spawn two audits."""
    c = {"claim": "x", "grounding": "model_prior", "volatility": "volatile",
         "refs": ["$step1"]}
    assert synthesis_only_claims(rec(SEARCH), [c]) == []


def test_a_turn_with_no_tools_flags_nothing():
    c = {"claim": "x", "grounding": "retrieved", "refs": [], "quote": "q"}
    assert synthesis_only_claims(rec({}), [c]) == []


# ── the truncated handoff ──────────────────────────────────────────────

def test_the_truncation_preamble_forbids_claiming_completeness():
    p = ConcernsMixin._TRUNCATED_PARENT_PREAMBLE.lower()
    assert "iteration cap" in p
    assert "partly done" in p
    for word in ("complete", "verified", "grounded"):
        assert word in p, f"the preamble must name {word!r} as a claim to avoid"
    assert "unconfirmed" in p


def test_a_truncated_parent_leads_its_successor_with_the_caveat(loop):
    """It has to lead. A successor that reads its task first and the
    caveat last writes the task's summary, not the caveat's."""
    parent = make_parent(loop)
    new_id = loop._create_successor_concern(
        parent, "Synthesize the verified bug reports and finalize.",
        truncated=True)

    instr = props(loop, new_id)["instruction"]
    assert instr.startswith("NOTE —"), "the caveat must come first"
    assert "Synthesize the verified bug reports" in instr


def test_a_chosen_yield_carries_no_caveat(loop):
    """A `yield` is a boundary the agent picked, not a cut-off, and must
    not inherit a warning about work it deliberately parked."""
    parent = make_parent(loop)
    new_id = loop._spawn_successor_from_yield(parent, "Do the next part.")

    assert props(loop, new_id)["instruction"] == "Do the next part."
