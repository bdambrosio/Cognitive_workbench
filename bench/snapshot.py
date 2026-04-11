#!/usr/bin/env python3
"""Stage 3a: Snapshot and restore agent state for benchmark experiments.

The cross-validation experiment harness needs to save the agent's persistent
state to a labeled snapshot, run trials that mutate it, then restore back
to the snapshot. This module provides those primitives.

Snapshot scope (for character "Jill" in world "infolab"):

  scenarios/infolab/resources/Jill/
    world_model.json          ← persistent facts and tool contracts
    world_model.json.bak      ← backup file
    resources.json            ← all named notes incl. _situation, scheduled goals
    resources.json.bak        ← backup file (only if present)
    resources.json~           ← backup file (only if present)
    cognitive_graph.json      ← cognitive graph nodes/edges
    cognitive_graph.faiss     ← graph embedding index
    entity_index.json         ← entity index
    infolab_notes.index.faiss
    infolab_notes.index.meta
    infolab_collections.index.faiss
    infolab_collections.index.meta
    planner_history/          ← compressed_trace.jsonl + reflection_frame.jsonl

This is the complete persistent state for one character in one world.
Anything else (logs, generated goal_plan_*.py / goal_review_*.md / goal_trace_*.txt
files at the repo root) is considered ephemeral and is NOT snapshotted —
the experiment harness can clean those separately if needed.

Snapshot operations refuse to run while the launcher is up. The check is a
best-effort Zenoh ping at the launcher/ready key; if any character's
queryable answers, snapshot/restore is blocked. To bypass (e.g. for testing
the snapshot module itself), use --force.

Usage:

  python bench/snapshot.py snapshot baseline-clean
  python bench/snapshot.py restore  baseline-clean
  python bench/snapshot.py list
  python bench/snapshot.py describe baseline-clean
  python bench/snapshot.py delete   baseline-clean
"""
from __future__ import annotations

import argparse
import json
import shutil
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

# Path setup so we can import zenoh utils for the launcher liveness check.
_REPO_ROOT = Path(__file__).resolve().parent.parent
_SRC_DIR = _REPO_ROOT / "src"
sys.path.insert(0, str(_SRC_DIR))

DEFAULT_CHARACTER = "Jill"
DEFAULT_WORLD = "infolab"
SNAPSHOTS_DIR = _REPO_ROOT / "bench" / "snapshots"

# Files to snapshot, relative to the character's resource directory.
# Files marked optional=True are skipped if they don't exist (e.g.
# resources.json~ which is created lazily by the resource manager).
_SNAPSHOT_FILES = [
    ("world_model.json", False),
    ("world_model.json.bak", True),
    ("resources.json", False),
    ("resources.json.bak", True),
    ("resources.json~", True),
    ("cognitive_graph.json", True),
    ("cognitive_graph.faiss", True),
    ("entity_index.json", True),
    ("infolab_notes.index.faiss", True),
    ("infolab_notes.index.meta", True),
    ("infolab_collections.index.faiss", True),
    ("infolab_collections.index.meta", True),
]
_SNAPSHOT_DIRS = [
    "planner_history",  # compressed_trace.jsonl, reflection_frame.jsonl
]


def _resource_dir(world: str, character: str) -> Path:
    return _REPO_ROOT / "scenarios" / world / "resources" / character


def _snapshot_dir(label: str) -> Path:
    return SNAPSHOTS_DIR / label


# ──────────────────────────────────────────────────────────────────────────
# Launcher liveness check
# ──────────────────────────────────────────────────────────────────────────

def is_launcher_running(character: str = DEFAULT_CHARACTER, timeout: float = 1.0) -> bool:
    """Best-effort check: query the character's scheduled_goals queryable.
    If anything answers, the launcher is up. If not, assume it's down.

    Returns True if a reply was received, False otherwise.
    """
    try:
        import zenoh  # noqa: WPS433
        from utils.zenoh_utils import make_localhost_config  # noqa: WPS433
    except ImportError:
        # Without zenoh, we can't check — be conservative and assume down.
        return False
    try:
        session = zenoh.open(make_localhost_config())
    except Exception:
        return False
    try:
        key = f"cognitive/{character}/scheduled_goals"
        for reply in session.get(key, timeout=timeout):
            if hasattr(reply, "ok") and reply.ok is not None:
                return True
    except Exception:
        pass
    finally:
        try:
            session.close()
        except Exception:
            pass
    return False


# ──────────────────────────────────────────────────────────────────────────
# Snapshot
# ──────────────────────────────────────────────────────────────────────────

def snapshot(
    label: str,
    *,
    character: str = DEFAULT_CHARACTER,
    world: str = DEFAULT_WORLD,
    force: bool = False,
) -> Path:
    """Save the character's persistent state to bench/snapshots/<label>/.

    Refuses if the launcher is running (unless force=True). Refuses if the
    snapshot label already exists (delete it first or use a different label).

    Returns the snapshot directory path on success.
    Raises RuntimeError on failure.
    """
    if not force and is_launcher_running(character):
        raise RuntimeError(
            f"Launcher appears to be running for character '{character}'. "
            f"Stop it before snapshotting (or use --force to override)."
        )

    snap_dir = _snapshot_dir(label)
    if snap_dir.exists():
        raise RuntimeError(
            f"Snapshot label '{label}' already exists at {snap_dir}. "
            f"Delete it first: python bench/snapshot.py delete {label}"
        )

    src = _resource_dir(world, character)
    if not src.exists():
        raise RuntimeError(
            f"Character resource directory not found: {src}. "
            f"Has the launcher ever run for character '{character}' in world '{world}'?"
        )

    snap_dir.mkdir(parents=True, exist_ok=False)

    captured: List[str] = []
    skipped: List[str] = []

    for fname, optional in _SNAPSHOT_FILES:
        srcfile = src / fname
        if not srcfile.exists():
            if optional:
                skipped.append(fname)
                continue
            raise RuntimeError(f"Required snapshot file missing: {srcfile}")
        shutil.copy2(srcfile, snap_dir / fname)
        captured.append(fname)

    for dname in _SNAPSHOT_DIRS:
        srcdir = src / dname
        if not srcdir.exists():
            skipped.append(f"{dname}/")
            continue
        shutil.copytree(srcdir, snap_dir / dname, dirs_exist_ok=True)
        captured.append(f"{dname}/")

    # Write a manifest with timestamp and the source paths so future
    # restore operations can sanity-check (and human readers can see what
    # this snapshot is).
    manifest = {
        "label": label,
        "character": character,
        "world": world,
        "created": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "source_dir": str(src),
        "captured": captured,
        "skipped": skipped,
    }
    with open(snap_dir / "_snapshot_manifest.json", "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2)

    print(
        f"Snapshot '{label}': {len(captured)} item(s) captured, "
        f"{len(skipped)} optional skipped → {snap_dir}",
        file=sys.stderr,
    )
    return snap_dir


# ──────────────────────────────────────────────────────────────────────────
# Restore
# ──────────────────────────────────────────────────────────────────────────

def restore(
    label: str,
    *,
    character: str = DEFAULT_CHARACTER,
    world: str = DEFAULT_WORLD,
    force: bool = False,
) -> Path:
    """Copy the snapshot back to the character's resource directory,
    overwriting any current state. Refuses if the launcher is running
    (unless force=True).

    The previous `preserve=` kwarg was removed when the bench harness
    switched to the overlay model — partial restores are now expressed
    as `restore(label) + apply_*_overlay(...)` instead of
    `restore(label, preserve=[...])`. The overlay model is strictly
    clearer because each slice of per-run state (world_model,
    cached_plans) is an explicit operation.

    Returns the destination directory on success.
    Raises RuntimeError on launcher-still-running or missing snapshot.
    """
    if not force and is_launcher_running(character):
        raise RuntimeError(
            f"Launcher appears to be running for character '{character}'. "
            f"Stop it before restoring (or use --force to override)."
        )

    snap_dir = _snapshot_dir(label)
    if not snap_dir.exists():
        raise RuntimeError(f"Snapshot label '{label}' not found at {snap_dir}")

    manifest_path = snap_dir / "_snapshot_manifest.json"
    if manifest_path.exists():
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            if manifest.get("character") != character:
                print(
                    f"WARN: snapshot character mismatch "
                    f"(snapshot={manifest.get('character')}, restore={character})",
                    file=sys.stderr,
                )
            if manifest.get("world") != world:
                print(
                    f"WARN: snapshot world mismatch "
                    f"(snapshot={manifest.get('world')}, restore={world})",
                    file=sys.stderr,
                )
        except Exception as e:
            print(f"WARN: snapshot manifest unreadable: {e}", file=sys.stderr)

    dest = _resource_dir(world, character)
    dest.mkdir(parents=True, exist_ok=True)

    restored: List[str] = []

    # Files: copy each snapshot file back, overwriting whatever's there.
    for fname, _optional in _SNAPSHOT_FILES:
        snapfile = snap_dir / fname
        if not snapfile.exists():
            continue
        shutil.copy2(snapfile, dest / fname)
        restored.append(fname)

    # Directories: wipe + recreate to drop any per-experiment additions.
    for dname in _SNAPSHOT_DIRS:
        snapsub = snap_dir / dname
        destsub = dest / dname
        if snapsub.exists():
            if destsub.exists():
                shutil.rmtree(destsub)
            shutil.copytree(snapsub, destsub)
            restored.append(f"{dname}/")
        else:
            # Snapshot didn't have this dir; remove it from the destination
            # too so the restored state matches the snapshot exactly.
            if destsub.exists():
                shutil.rmtree(destsub)

    print(
        f"Restored snapshot '{label}': {len(restored)} item(s) → {dest}",
        file=sys.stderr,
    )
    return dest


# ──────────────────────────────────────────────────────────────────────────
# Overlay primitives
# ──────────────────────────────────────────────────────────────────────────
#
# After restoring a baseline snapshot, the batch runner can overlay
# specific slices of state before starting the launcher again. Two
# slices are supported today:
#
# - world_model overlay: copy a saved world_model.json over the
#   baseline's empty one. Simple file copy.
#
# - cached_plan overlay: inject a saved cached_plan_actions list into
#   the scheduled-goal Note for a given goal_id (by note_name, which
#   is stable across reseeds). Offline JSON surgery on resources.json.
#
# Symmetric capture functions read the same slices back out of disk
# so they can be saved as sidecar artifacts for later overlay use.
#
# Both pairs require the launcher to be stopped (same constraint as
# snapshot/restore — the agent's in-memory state would diverge from
# disk if it were live). is_launcher_running() is checked with force=
# override for tests.


def _scheduled_goal_note_name(goal_id: str) -> str:
    """Compute the note_name used by executive_node._scheduled_goal_note_name.

    Mirrors the agent-side logic: 'goal_17' → '_scheduled_goal_17'.
    """
    return "_scheduled_goal_" + goal_id.replace("goal_", "")


def _find_scheduled_goal_note(
    resources_data: Dict[str, Any], goal_id: str,
) -> Optional[Dict[str, Any]]:
    """Locate the scheduled-goal Note within a loaded resources.json
    dict by its stable note_name. Returns the Note instance dict (not a
    copy — mutations propagate) or None if not found.
    """
    target_name = _scheduled_goal_note_name(goal_id)
    notes = resources_data.get("note_instances") or {}
    for note in notes.values():
        props = note.get("properties") or {}
        if props.get("note_name") == target_name:
            return note
    return None


def apply_world_model_overlay(
    source: Path,
    *,
    character: str = DEFAULT_CHARACTER,
    world: str = DEFAULT_WORLD,
    force: bool = False,
) -> Path:
    """Overlay a saved world_model.json file onto the character's
    current resource directory. Copies `source` to
    `scenarios/<world>/resources/<character>/world_model.json`,
    replacing whatever's there.

    Called after `restore(label)` to carry accumulated world_model
    learnings forward into a fresh baseline.
    """
    if not force and is_launcher_running(character):
        raise RuntimeError(
            f"Launcher appears to be running for character '{character}'. "
            f"Stop it before applying an overlay (or use force=True)."
        )
    source = Path(source)
    if not source.exists():
        raise RuntimeError(f"world_model overlay source not found: {source}")
    dest = _resource_dir(world, character) / "world_model.json"
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, dest)
    print(
        f"Overlaid world_model.json from {source} → {dest}",
        file=sys.stderr,
    )
    return dest


def apply_cached_plan_overlay(
    goal_id: str,
    source: Path,
    *,
    character: str = DEFAULT_CHARACTER,
    world: str = DEFAULT_WORLD,
    force: bool = False,
) -> None:
    """Inject a saved cached_plan_actions list into the scheduled-goal
    Note for `goal_id`, by offline JSON surgery on resources.json.

    `source` is a JSON file containing the cached_plan_actions list
    (produced by `capture_cached_plan`). The target scheduled-goal Note
    is located by its note_name (`_scheduled_goal_<N>`), and its
    `properties.content` is re-serialized with the new field.

    Raises RuntimeError if the goal Note can't be found (i.e. the
    goal_id doesn't exist in the current resources.json — probably a
    baseline/seed mismatch).
    """
    if not force and is_launcher_running(character):
        raise RuntimeError(
            f"Launcher appears to be running for character '{character}'. "
            f"Stop it before applying an overlay (or use force=True)."
        )
    source = Path(source)
    if not source.exists():
        raise RuntimeError(f"cached_plan overlay source not found: {source}")
    with open(source, "r", encoding="utf-8") as f:
        new_plan_actions = json.load(f)
    if not isinstance(new_plan_actions, list):
        raise RuntimeError(
            f"cached_plan source must contain a JSON list, got "
            f"{type(new_plan_actions).__name__} in {source}"
        )

    resources_path = _resource_dir(world, character) / "resources.json"
    if not resources_path.exists():
        raise RuntimeError(
            f"resources.json not found at {resources_path}. Restore a "
            f"baseline before applying cached_plan overlays."
        )
    with open(resources_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    note = _find_scheduled_goal_note(data, goal_id)
    if note is None:
        raise RuntimeError(
            f"scheduled-goal Note for '{goal_id}' not found in "
            f"{resources_path}. Check that the baseline was seeded with "
            f"this goal_id."
        )

    props = note.setdefault("properties", {})
    content_str = props.get("content") or "{}"
    try:
        content_obj = json.loads(content_str) if isinstance(content_str, str) else content_str
    except Exception as e:
        raise RuntimeError(
            f"scheduled-goal Note for '{goal_id}' has unparseable "
            f"content: {e}"
        )
    if not isinstance(content_obj, dict):
        raise RuntimeError(
            f"scheduled-goal Note for '{goal_id}' content is not a dict"
        )

    content_obj["cached_plan_actions"] = new_plan_actions
    props["content"] = json.dumps(content_obj)

    with open(resources_path, "w", encoding="utf-8") as f:
        json.dump(data, f)

    print(
        f"Overlaid cached_plan for {goal_id} ({len(new_plan_actions)} "
        f"actions) from {source}",
        file=sys.stderr,
    )


def capture_world_model(
    dest: Path,
    *,
    character: str = DEFAULT_CHARACTER,
    world: str = DEFAULT_WORLD,
) -> Path:
    """Copy the current world_model.json out of the character's
    resource dir to a named destination file. Used by the batch runner
    to snapshot the accumulating world_model between Trial B phase 1
    goals so it can be overlaid on the next run.

    Does not require the launcher to be stopped — it's a read-only file
    copy. (The launcher persists world_model.json at goal completion,
    so capturing it between goals catches it in a consistent state.)
    """
    src = _resource_dir(world, character) / "world_model.json"
    if not src.exists():
        raise RuntimeError(f"world_model.json not found at {src}")
    dest = Path(dest)
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dest)
    return dest


def capture_cached_plan(
    goal_id: str,
    dest: Path,
    *,
    character: str = DEFAULT_CHARACTER,
    world: str = DEFAULT_WORLD,
) -> Path:
    """Read the cached_plan_actions list for `goal_id` out of the
    current resources.json and write it as a JSON list to `dest`.

    Returns `dest`. Raises RuntimeError if the goal Note or the
    cached_plan field is missing.
    """
    resources_path = _resource_dir(world, character) / "resources.json"
    if not resources_path.exists():
        raise RuntimeError(f"resources.json not found at {resources_path}")
    with open(resources_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    note = _find_scheduled_goal_note(data, goal_id)
    if note is None:
        raise RuntimeError(
            f"scheduled-goal Note for '{goal_id}' not found in "
            f"{resources_path}"
        )

    props = note.get("properties") or {}
    content_str = props.get("content") or "{}"
    try:
        content_obj = json.loads(content_str) if isinstance(content_str, str) else content_str
    except Exception as e:
        raise RuntimeError(
            f"scheduled-goal Note for '{goal_id}' has unparseable content: {e}"
        )
    plan_actions = content_obj.get("cached_plan_actions") if isinstance(content_obj, dict) else None
    if plan_actions is None:
        raise RuntimeError(
            f"scheduled-goal Note for '{goal_id}' has no cached_plan_actions "
            f"to capture (goal has not been reviewed/committed yet)"
        )
    if not isinstance(plan_actions, list):
        raise RuntimeError(
            f"cached_plan_actions for '{goal_id}' is not a list"
        )

    dest = Path(dest)
    dest.parent.mkdir(parents=True, exist_ok=True)
    with open(dest, "w", encoding="utf-8") as f:
        json.dump(plan_actions, f)
    return dest


# ──────────────────────────────────────────────────────────────────────────
# List / describe / delete
# ──────────────────────────────────────────────────────────────────────────

def list_snapshots() -> List[str]:
    """Return all snapshot labels in bench/snapshots/, sorted by name."""
    if not SNAPSHOTS_DIR.exists():
        return []
    labels = sorted(p.name for p in SNAPSHOTS_DIR.iterdir() if p.is_dir())
    return labels


def describe_snapshot(label: str) -> dict:
    """Read and return the snapshot's manifest dict."""
    snap_dir = _snapshot_dir(label)
    manifest_path = snap_dir / "_snapshot_manifest.json"
    if not manifest_path.exists():
        raise RuntimeError(f"Snapshot '{label}' has no manifest")
    return json.loads(manifest_path.read_text(encoding="utf-8"))


def delete_snapshot(label: str) -> None:
    """Remove a snapshot directory and all its contents."""
    snap_dir = _snapshot_dir(label)
    if not snap_dir.exists():
        raise RuntimeError(f"Snapshot '{label}' not found at {snap_dir}")
    shutil.rmtree(snap_dir)
    print(f"Deleted snapshot '{label}'", file=sys.stderr)


# ──────────────────────────────────────────────────────────────────────────
# CLI
# ──────────────────────────────────────────────────────────────────────────

def main(argv: Optional[List[str]] = None) -> int:
    p = argparse.ArgumentParser(
        description="Snapshot and restore agent state for benchmark experiments",
    )
    p.add_argument(
        "--character",
        default=DEFAULT_CHARACTER,
        help=f"Character name (default: {DEFAULT_CHARACTER})",
    )
    p.add_argument(
        "--world",
        default=DEFAULT_WORLD,
        help=f"World name (default: {DEFAULT_WORLD})",
    )
    p.add_argument(
        "--force",
        action="store_true",
        help="Bypass the launcher-liveness safety check",
    )

    sub = p.add_subparsers(dest="cmd", required=True)

    sp_snap = sub.add_parser("snapshot", help="Save current state to a snapshot label")
    sp_snap.add_argument("label", help="Snapshot label (e.g. 'baseline-clean')")

    sp_rest = sub.add_parser("restore", help="Restore state from a snapshot label")
    sp_rest.add_argument("label", help="Snapshot label to restore")

    sub.add_parser("list", help="List all snapshot labels")

    sp_desc = sub.add_parser("describe", help="Show a snapshot's manifest")
    sp_desc.add_argument("label", help="Snapshot label to describe")

    sp_del = sub.add_parser("delete", help="Delete a snapshot")
    sp_del.add_argument("label", help="Snapshot label to delete")

    args = p.parse_args(argv)

    try:
        if args.cmd == "snapshot":
            snapshot(args.label, character=args.character, world=args.world, force=args.force)
        elif args.cmd == "restore":
            restore(
                args.label,
                character=args.character,
                world=args.world,
                force=args.force,
            )
        elif args.cmd == "list":
            labels = list_snapshots()
            if not labels:
                print("(no snapshots)", file=sys.stderr)
            else:
                for label in labels:
                    try:
                        m = describe_snapshot(label)
                        ts = m.get("created", "?")
                        ch = m.get("character", "?")
                        wd = m.get("world", "?")
                        print(f"  {label}  ({ch}/{wd}, {ts})")
                    except Exception:
                        print(f"  {label}  (no manifest)")
        elif args.cmd == "describe":
            m = describe_snapshot(args.label)
            print(json.dumps(m, indent=2))
        elif args.cmd == "delete":
            delete_snapshot(args.label)
    except RuntimeError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 1
    except Exception as e:
        print(f"ERROR: unexpected: {e}", file=sys.stderr)
        return 2

    return 0


if __name__ == "__main__":
    sys.exit(main())
