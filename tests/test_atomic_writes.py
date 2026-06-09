"""Tests for utils.file_utils atomic writes and their persistence call sites.

Uses a throwaway world name + tmp_path only — never touches live world state
(data/resources, scenarios/jill_chat, ...).
"""

import json
import os
import shutil
import sys
import uuid
from pathlib import Path

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from utils.file_utils import atomic_write_bytes, atomic_write_json, atomic_write_text


# ── atomic_write_* primitives ──────────────────────────────────────────

def test_atomic_write_text_round_trip(tmp_path):
    p = tmp_path / "out.txt"
    atomic_write_text(p, "hello ☂")
    assert p.read_text(encoding="utf-8") == "hello ☂"
    assert not list(tmp_path.glob("*.tmp"))


def test_atomic_write_json_round_trip(tmp_path):
    p = tmp_path / "out.json"
    data = {"a": 1, "b": ["x", "y"]}
    atomic_write_json(p, data)
    assert json.loads(p.read_text()) == data
    assert not list(tmp_path.glob("*.tmp"))


def test_failed_write_preserves_original(tmp_path):
    p = tmp_path / "out.json"
    atomic_write_json(p, {"good": True})
    with pytest.raises(TypeError):
        atomic_write_json(p, {"bad": object()})  # not JSON-serializable
    assert json.loads(p.read_text()) == {"good": True}
    assert not list(tmp_path.glob("*.tmp"))


def test_atomic_write_bytes(tmp_path):
    p = tmp_path / "blob.bin"
    atomic_write_bytes(p, b"\x00\x01\x02")
    assert p.read_bytes() == b"\x00\x01\x02"
    assert not list(tmp_path.glob("*.tmp"))


# ── FAISSStore.save/load ───────────────────────────────────────────────

def test_faiss_store_save_load_round_trip(tmp_path):
    from infospace_resource_manager import FAISSStore

    store = FAISSStore(dimension=8)
    store.add("doc one", [1.0, 0, 0, 0, 0, 0, 0, 0], metadata={"id": 1})
    store.add("doc two", [0, 1.0, 0, 0, 0, 0, 0, 0], metadata={"id": 2})

    path = str(tmp_path / "test.index")
    store.save(path)
    assert not list(tmp_path.glob("*.tmp"))

    loaded = FAISSStore(dimension=8)
    loaded.load(path)
    assert loaded.documents == ["doc one", "doc two"]
    assert loaded.metadata == [{"id": 1}, {"id": 2}]
    assert loaded.index.ntotal == 2


# ── InfospaceResourceManager.save_to_file/load_from_file ───────────────

@pytest.fixture
def scratch_manager():
    from infospace_resource_manager import InfospaceResourceManager

    world = f"pytest_scratch_{uuid.uuid4().hex[:8]}"
    mgr = InfospaceResourceManager(world, world_config={"world_name": world})
    yield mgr
    # The constructor mkdirs scenarios/<world>/resources — remove it.
    scenario_dir = Path(__file__).parent.parent / "scenarios" / world
    shutil.rmtree(scenario_dir, ignore_errors=True)


def test_resource_manager_save_load_round_trip(scratch_manager, tmp_path):
    mgr = scratch_manager
    ok, nid, err, _ = mgr.create_note(
        "Tester", "a fact to remember", "text", "pytest", "", "",
        {"kind": "memory", "exclude_from_index": True},
    )
    assert ok, err
    ok, err = mgr.mark_persistent(nid, "Tester")
    assert ok, err

    target = tmp_path / "resources.json"
    assert mgr.save_to_file(target)
    assert not list(tmp_path.glob("*.tmp"))

    data = json.loads(target.read_text())
    assert nid in data["note_instances"]
    assert data["note_instances"][nid]["properties"]["content"] == "a fact to remember"
