"""The `concern-state` action: a read-only view of the live concern
collections from inside a turn (Jill's proposal, 2026-09-15). No model."""
import shutil
import sys
import threading
import uuid
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from chat.chat_loop import ChatLoop                              # noqa: E402
from chat import concerns as C                                   # noqa: E402
from chat.react import _REACT_TOOLS                              # noqa: E402
from infospace_resource_manager import InfospaceResourceManager  # noqa: E402


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
    yield inst
    shutil.rmtree(Path(__file__).parent.parent / "scenarios" / world, ignore_errors=True)


def _note(loop, cid, props, text):
    props = {"exclude_from_index": True, **props}
    ok, nid, err, _ = loop.resource_manager.create_note("Tester", text, "text", "pytest", "", "", props)
    assert ok, err
    loop.resource_manager.resource_registry[cid]["properties"]["content"].append(nid)
    return nid


def _populate(loop):
    a = _note(loop, "Collection_ac", {"kind": "agent_concern", "status": "active", "activation": 1.0,
                                       "seed": True, "instruction": None, "content": "What is Bruce working toward"},
              "What is Bruce working toward")
    b = _note(loop, "Collection_ac", {"kind": "agent_concern", "status": "active", "activation": 0.4,
                                       "seed": False, "instruction": "Check the S&P close", "rhythm_hours": 24,
                                       "wip": "close was 1pm PDT", "content": "Track S&P 500 closing price daily"},
              "Track S&P 500 closing price daily")
    c = _note(loop, "Collection_ac", {"kind": "agent_concern", "status": "satisfied", "activation": 0.0,
                                       "seed": False, "instruction": "x", "content": "Old finished concern"},
              "Old finished concern")
    u = _note(loop, "Collection_uc", {"kind": "user_concern", "status": "active", "strength": 0.8,
                                       "context": "asked about scaling", "content": "Space management"},
              "Space management")
    return a, b, c, u


def test_default_is_active_agent_concerns_highest_first_with_the_count_line(loop):
    a, b, c, u = _populate(loop)
    out = loop._run_concern_state()
    assert out.startswith("OK:")
    head = out.splitlines()[1]
    assert "2 active of 3 recorded" in head and "1 seed(s), 1 non-seed against a cap of" in head
    assert f"{C._AGENT_CONCERN_POPULATION_CAP}" in head and "1 of the active carry an instruction" in head
    rows = [l for l in out.splitlines() if l.startswith("  Note")]
    assert [r.split()[0] for r in rows] == [a, b]                 # activation 1.0 before 0.4
    assert "no instruction" in rows[0] and "seed" in rows[0]
    assert "instruction" in rows[1] and "rhythm=24h" in rows[1] and "last_fired=never" in rows[1]
    assert c not in out and u not in out                           # satisfied and user rows excluded
    assert "wip:" not in out                                       # detail only with by_id


def test_filters_status_instruction_seed_and_floor(loop):
    a, b, c, u = _populate(loop)
    assert c in loop._run_concern_state(status="satisfied") and a not in loop._run_concern_state(status="satisfied")
    assert [l for l in loop._run_concern_state(status="any").splitlines() if l.startswith("  Note")].__len__() == 3
    only_shape = loop._run_concern_state(has_instruction=False)
    assert a in only_shape and b not in only_shape
    assert b in loop._run_concern_state(seed=False) and a not in loop._run_concern_state(seed=False)
    assert b not in loop._run_concern_state(min_activation=0.5) and a in loop._run_concern_state(min_activation=0.5)
    assert "(no concern matches)" in loop._run_concern_state(min_activation=2)
    assert loop._run_concern_state(min_activation="lots").startswith("ERROR")
    assert loop._run_concern_state(collection="elsewhere").startswith("ERROR")


def test_by_id_gives_the_full_record_and_both_covers_user_concerns(loop):
    a, b, c, u = _populate(loop)
    out = loop._run_concern_state(by_id=b)
    assert b in out and a not in out
    assert "instruction: Check the S&P close" in out and "wip: close was 1pm PDT" in out
    both = loop._run_concern_state(collection="both")
    assert "user concerns: 1 active of 1 recorded" in both and f"{u}  strength=0.80" in both
    user_detail = loop._run_concern_state(collection="user", by_id=u)
    assert "context: asked about scaling" in user_detail


def test_row_cap_and_the_action_is_known_to_the_loop(loop):
    for i in range(C.ConcernsMixin._CONCERN_STATE_ROW_CAP + 3):
        _note(loop, "Collection_ac", {"kind": "agent_concern", "status": "active", "activation": i / 100,
                                       "seed": False, "instruction": "x", "content": f"c{i}"}, f"c{i}")
    out = loop._run_concern_state()
    assert len([l for l in out.splitlines() if l.startswith("  Note")]) == C.ConcernsMixin._CONCERN_STATE_ROW_CAP
    assert "... 3 more; narrow with a filter" in out
    assert "concern-state" in _REACT_TOOLS


# ── the prompt block: ranked rows are fire-capable, lenses stand aside ──

def _block_stub(active, autonomy=True):
    inst = object.__new__(ChatLoop)
    inst._iter_active_agent_concerns = lambda: active
    inst._autonomy_enabled = autonomy
    return inst


def _n(nid, text, activation, **props):
    return (nid, {"text": text, "properties": {"content": text, "status": "active", **props}}, activation)


def test_block_ranks_fire_capable_only_and_lists_the_rest_as_standing():
    active = [_n("Note_1", "What is Bruce working toward", 1.0, seed=True),
              _n("Note_2", "Did what I said land", 1.0, seed=True),
              _n("Note_3", "Track the S&P close", 0.5, instruction="check the close", rhythm_hours=24),
              _n("Note_4", "Monitor the PV controller", 0.2, instruction="read the PV log", rhythm_hours=1)]
    inst = _block_stub(active)
    ranked = ChatLoop._top_active_agent_concerns(inst, fire_capable=True)
    assert [r[0] for r in ranked] == ["Note_3", "Note_4"]                  # the 1.00 lenses are out
    assert [r[0] for r in ChatLoop._top_active_agent_concerns(inst)][:2] == ["Note_1", "Note_2"]
    block = inst._render_agent_concerns_block(ranked)
    assert "- [0.50, rank 1/2] Track the S&P close" in block
    assert "- [0.20, rank 2/2] Monitor the PV controller" in block
    assert "fires every ~24h" in block
    assert "Standing, not ranked" in block and "What is Bruce working toward · Did what I said land" in block
    assert "Non-seed 2 / cap" in block                                       # the cap counts every active non-seed
    assert "rank 1/4" not in block and "Not shown" not in block


def test_block_shows_the_unshown_fire_capable_and_survives_no_ranked_rows():
    active = [_n(f"Note_{i}", f"c{i}", i / 10, instruction="x", rhythm_hours=24) for i in range(1, 8)]
    inst = _block_stub(active)
    ranked = ChatLoop._top_active_agent_concerns(inst, fire_capable=True)
    assert len(ranked) == 5
    block = inst._render_agent_concerns_block(ranked)
    assert "Showing 5 of 7 fire-capable. Not shown: 0.20, 0.10." in block
    lenses_only = _block_stub([_n("Note_9", "a lens", 1.0, seed=True)])
    block2 = lenses_only._render_agent_concerns_block([])
    assert "(no fire-capable concern is active)" in block2 and "a lens" in block2
    assert _block_stub([])._render_agent_concerns_block([]) is None
