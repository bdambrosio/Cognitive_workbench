#!/usr/bin/env python3
"""Build a truly-empty baseline snapshot for the bench harness.

The old `baseline-clean` approach was to delete world_model.json from an
existing Jill and snapshot the character directory. That preserved
`resources.json` (for the framework scaffold + seeded goal records) but
also preserved accumulated session history: prior `_user_concerns`,
`_situation`, `_ooda_state`, `conversation_history` items, FAISS-indexed
persistent notes from old runs, and so on. Inspection of a recent trace
showed that history leaking into planner behavior — closed concerns
from prior sessions were priming the planner toward failure modes that
matched the prior runs, and a cross-goal ToM paragraph built from a
different session's debugging was biasing the planner's tool-parameter
experimentation.

This module builds the baseline differently: start from an empty
`scenarios/<world>/resources/<character>/` dir, boot the launcher once
(which writes a ~5KB framework scaffold — two persistent resources:
`_derived_concerns` seeded from the scenario YAML, and an empty
`_scheduled_goals` Collection), send `/goal add` + `/goal rename` for
each entry in a goals YAML file to seed G01..Gnn without running them
(new `/goal add` semantic: create-only unless `run=True`), stop the
launcher, and snapshot.

Usage:

    python bench/baseline.py create \\
        --label baseline-empty \\
        --goals-file bench/goals-benchlab.yaml

Requires the launcher to be stopped before invocation. Refuses to
proceed if a launcher is already running (to avoid clobbering live
state). The snapshot is written by bench/snapshot.py and refuses to
overwrite an existing label — delete first or use a different label.
"""
from __future__ import annotations

import argparse
import json
import shutil
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

_REPO_ROOT = Path(__file__).resolve().parent.parent
_SRC_DIR = _REPO_ROOT / "src"
sys.path.insert(0, str(_SRC_DIR))
sys.path.insert(0, str(Path(__file__).resolve().parent))

# Import order matters: bench/launcher.py must precede anything that
# triggers sys.path insert of src/, otherwise `import launcher` shadows
# bench/launcher.py with src/launcher.py. See experiment.py for the same
# note.
import launcher   # noqa: E402
import snapshot   # noqa: E402
from experiment import clear_bench_logs  # noqa: E402

DEFAULT_CHARACTER = "Jill"
DEFAULT_WORLD = "benchlab"
DEFAULT_SCENARIO = "jill-benchlab.yaml"
DEFAULT_READY_TIMEOUT_S = 120.0
DEFAULT_GOAL_ADD_TIMEOUT_S = 5.0


# ──────────────────────────────────────────────────────────────────────
# Goals YAML loader
# ──────────────────────────────────────────────────────────────────────

def load_goals_file(
    path: Path,
) -> Dict[str, List[Dict[str, str]]]:
    """Load a goals YAML and return a dict with:
      - 'goals': list of {name, text} dicts (required, non-empty)
      - 'seed_notes': list of {name, content} dicts (optional, can be
         empty). Entries may supply either `content` (inline string) or
         `content_file` (repo-relative path); this function resolves
         `content_file` and returns the loaded text under `content` so
         the caller sees a uniform shape.

    Validates that goal and seed-note names are unique within their
    respective sections.
    """
    try:
        import yaml  # noqa: WPS433
    except ImportError:
        raise RuntimeError(
            "PyYAML is required to parse the goals YAML. Install with "
            "`pip install pyyaml`."
        )
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or "goals" not in data:
        raise ValueError(f"{path}: expected top-level key 'goals'")
    goals_raw = data["goals"]
    if not isinstance(goals_raw, list) or not goals_raw:
        raise ValueError(f"{path}: 'goals' must be a non-empty list")

    seen_goal_names: set = set()
    goals: List[Dict[str, str]] = []
    for i, entry in enumerate(goals_raw):
        if not isinstance(entry, dict):
            raise ValueError(f"{path}: goals[{i}] is not a dict")
        name = str(entry.get("name", "")).strip()
        text = str(entry.get("text", "")).strip()
        if not name:
            raise ValueError(f"{path}: goals[{i}] missing 'name'")
        if not text:
            raise ValueError(f"{path}: goals[{i}] ({name}) missing 'text'")
        if name in seen_goal_names:
            raise ValueError(f"{path}: duplicate goal name '{name}'")
        seen_goal_names.add(name)
        goals.append({"name": name, "text": text})

    seed_notes_raw = data.get("seed_notes", []) or []
    if not isinstance(seed_notes_raw, list):
        raise ValueError(f"{path}: 'seed_notes' must be a list if present")

    seen_note_names: set = set()
    seed_notes: List[Dict[str, str]] = []
    for i, entry in enumerate(seed_notes_raw):
        if not isinstance(entry, dict):
            raise ValueError(f"{path}: seed_notes[{i}] is not a dict")
        name = str(entry.get("name", "")).strip()
        if not name:
            raise ValueError(f"{path}: seed_notes[{i}] missing 'name'")
        if name in seen_note_names:
            raise ValueError(f"{path}: duplicate seed-note name '{name}'")
        seen_note_names.add(name)

        inline = entry.get("content")
        file_ref = entry.get("content_file")
        if inline is not None and file_ref is not None:
            raise ValueError(
                f"{path}: seed_notes[{i}] ({name}) sets both 'content' "
                f"and 'content_file' — pick one"
            )
        if inline is not None:
            content = str(inline)
        elif file_ref is not None:
            content_path = (_REPO_ROOT / str(file_ref)).resolve()
            if not content_path.exists():
                raise ValueError(
                    f"{path}: seed_notes[{i}] ({name}) content_file not "
                    f"found: {content_path}"
                )
            content = content_path.read_text(encoding="utf-8")
        else:
            raise ValueError(
                f"{path}: seed_notes[{i}] ({name}) must supply "
                f"'content' or 'content_file'"
            )
        seed_notes.append({"name": name, "content": content})

    return {"goals": goals, "seed_notes": seed_notes}


# ──────────────────────────────────────────────────────────────────────
# Live Zenoh command helpers
# ──────────────────────────────────────────────────────────────────────
#
# The agent's command channel is a one-way PUT subscriber at
# `cognitive/<character>/command`; handlers return strings but those
# strings are not plumbed back to the sender. So to learn the goal_id
# of a goal we just created, we PUT the command and then diff the
# `scheduled_goals` queryable against the pre-PUT snapshot to discover
# the new entry. Polling overhead is ~200ms per goal and there are 10
# goals max.

def _open_session():
    import zenoh  # noqa: WPS433
    from utils.zenoh_utils import make_localhost_config  # noqa: WPS433
    return zenoh.open(make_localhost_config())


def _put_command(session, character: str, payload: Dict[str, Any]) -> None:
    """Fire a command to the agent's one-way command channel. No reply."""
    key = f"cognitive/{character}/command"
    session.put(key, json.dumps(payload).encode("utf-8"))


def _list_scheduled_goal_ids(session, character: str, timeout: float = 3.0) -> set:
    """Query scheduled_goals and return the set of current goal_ids.
    Used as the before/after diff set when seeding.
    """
    key = f"cognitive/{character}/scheduled_goals"
    ids: set = set()
    try:
        for reply in session.get(key, timeout=timeout):
            if not (hasattr(reply, "ok") and reply.ok is not None):
                continue
            try:
                payload = json.loads(reply.ok.payload.to_bytes().decode("utf-8"))
            except Exception:
                continue
            for g in payload.get("goals", []) or []:
                gid = g.get("goal_id")
                if gid:
                    ids.add(gid)
    except Exception as e:
        print(f"WARN: scheduled_goals query failed: {e}", file=sys.stderr)
    return ids


def _wait_for_new_goal_id(
    session,
    character: str,
    prior_ids: set,
    *,
    timeout: float = 5.0,
    poll_interval: float = 0.2,
) -> Optional[str]:
    """Poll scheduled_goals until a new goal_id appears that isn't in
    `prior_ids`. Returns the new goal_id if exactly one was added, or
    None on timeout or ambiguous multi-add.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        current = _list_scheduled_goal_ids(session, character)
        new = current - prior_ids
        if len(new) == 1:
            return next(iter(new))
        if len(new) > 1:
            # Shouldn't happen in the baseline flow (we seed one at a
            # time), but fail loudly instead of silently picking wrong.
            return None
        time.sleep(poll_interval)
    return None


def _create_seed_note(
    session,
    character: str,
    name: str,
    content: str,
    *,
    timeout: float = 10.0,
) -> Optional[str]:
    """Call the agent's `resource/create_note` queryable to create a
    persistent named Note. Returns the new note_id on success, or None
    on failure. The `persistent=True` flag instructs the agent to mark
    the note persistent immediately, so it survives the next goal
    cleanup AND is serialized to resources.json on shutdown.
    """
    key = f"cognitive/{character}/resource/create_note"
    payload = {
        "content": content,
        "format": "text",
        "name": name,
        "persistent": True,
    }
    try:
        for reply in session.get(
            key,
            payload=json.dumps(payload).encode("utf-8"),
            timeout=timeout,
        ):
            if not (hasattr(reply, "ok") and reply.ok is not None):
                continue
            try:
                body = reply.ok.payload.to_bytes().decode("utf-8")
                parsed = json.loads(body)
            except Exception as e:
                print(
                    f"WARN: create-note reply parse failed: {e}",
                    file=sys.stderr,
                )
                continue
            if parsed.get("success"):
                return parsed.get("note_id")
            else:
                err = parsed.get("error", "unknown")
                print(
                    f"ERROR: create-note '{name}' failed: {err}",
                    file=sys.stderr,
                )
                return None
    except Exception as e:
        print(
            f"WARN: create-note query for '{name}' failed: {e}",
            file=sys.stderr,
        )
    return None


def _wait_for_goal_name(
    session,
    character: str,
    goal_id: str,
    expected_name: str,
    *,
    timeout: float = 3.0,
    poll_interval: float = 0.2,
) -> bool:
    """Poll scheduled_goals until the named goal shows the expected name
    (confirms /goal rename landed). Returns True on match, False on timeout.
    """
    key = f"cognitive/{character}/scheduled_goals"
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            for reply in session.get(key, timeout=1.0):
                if not (hasattr(reply, "ok") and reply.ok is not None):
                    continue
                try:
                    payload = json.loads(reply.ok.payload.to_bytes().decode("utf-8"))
                except Exception:
                    continue
                for g in payload.get("goals", []) or []:
                    if g.get("goal_id") == goal_id and g.get("name") == expected_name:
                        return True
        except Exception:
            pass
        time.sleep(poll_interval)
    return False


# ──────────────────────────────────────────────────────────────────────
# The create-baseline flow
# ──────────────────────────────────────────────────────────────────────

def _resource_dir(world: str, character: str) -> Path:
    return _REPO_ROOT / "scenarios" / world / "resources" / character


def create_empty_baseline(
    *,
    label: str,
    goals: List[Dict[str, str]],
    seed_notes: Optional[List[Dict[str, str]]] = None,
    character: str = DEFAULT_CHARACTER,
    world: str = DEFAULT_WORLD,
    scenario: str = DEFAULT_SCENARIO,
    ready_timeout: float = DEFAULT_READY_TIMEOUT_S,
) -> Path:
    """Run the full baseline-creation flow end-to-end.

    Returns the snapshot directory on success. Raises RuntimeError on
    any step failure (launcher won't start, goal seeding failed, etc.)
    so the caller (or CLI main) can surface it.
    """
    # ── Preflight: nothing running, no existing snapshot label,
    #     scenario file exists (we're about to wipe state — fail fast
    #     before that if the launcher won't be able to start) ──────────
    if snapshot.is_launcher_running(character, timeout=1.0):
        raise RuntimeError(
            f"A launcher is already running for '{character}'. Stop it "
            f"before creating a baseline."
        )
    existing = snapshot.SNAPSHOTS_DIR / label
    if existing.exists():
        raise RuntimeError(
            f"Snapshot label '{label}' already exists at {existing}. "
            f"Delete it first or choose a different label."
        )
    scenario_path = _REPO_ROOT / "scenarios" / scenario
    if not scenario_path.exists():
        raise RuntimeError(
            f"Scenario file not found: {scenario_path}. Aborting before "
            f"touching resource state."
        )

    # ── Wipe the character's resource dir ─────────────────────────────
    res_dir = _resource_dir(world, character)
    if res_dir.exists():
        shutil.rmtree(res_dir)
    res_dir.mkdir(parents=True, exist_ok=True)
    print(f"baseline: wiped {res_dir}", file=sys.stderr)

    # ── Boot launcher once so framework scaffold gets written ─────────
    proc = launcher.LauncherProcess(scenario=scenario, character=character)
    proc.start()
    try:
        proc.wait_ready(timeout=ready_timeout)
    except Exception as e:
        try:
            proc.stop()
        except Exception:
            pass
        raise RuntimeError(f"Launcher failed to become ready: {e}") from e
    print(f"baseline: launcher ready ({scenario})", file=sys.stderr)

    # ── Seed each goal via /goal add + /goal rename ───────────────────
    session = None
    seeded: List[Dict[str, str]] = []
    try:
        session = _open_session()
        tracked_ids: set = _list_scheduled_goal_ids(session, character)
        if tracked_ids:
            print(
                f"baseline: WARN — found {len(tracked_ids)} pre-existing "
                f"goal_ids on a supposedly-empty launcher: "
                f"{sorted(tracked_ids)}",
                file=sys.stderr,
            )

        for i, g in enumerate(goals):
            name = g["name"]
            text = g["text"]

            # /goal add (create-only: we rely on the new default semantic,
            # no run=True flag). The handler's reply string goes nowhere
            # on the PUT channel, so we diff scheduled_goals to find the
            # new goal_id.
            _put_command(session, character, {
                "cmd": "/goal add",
                "goal_text": text,
                "source": "baseline-script",
            })

            new_goal_id = _wait_for_new_goal_id(
                session, character, tracked_ids,
                timeout=DEFAULT_GOAL_ADD_TIMEOUT_S,
            )
            if not new_goal_id:
                raise RuntimeError(
                    f"baseline: timed out (or ambiguous multi-add) "
                    f"waiting for {name} to appear in scheduled_goals. "
                    f"Prior ids: {sorted(tracked_ids)}. Agent-side "
                    f"/goal add may have failed — check the launcher log."
                )

            # /goal rename — no reply needed, just verify by polling.
            _put_command(session, character, {
                "cmd": "/goal rename",
                "goal_id": new_goal_id,
                "name": name,
                "source": "baseline-script",
            })
            if not _wait_for_goal_name(session, character, new_goal_id, name):
                raise RuntimeError(
                    f"baseline: /goal rename {new_goal_id} → {name} "
                    f"did not land within the timeout"
                )

            tracked_ids.add(new_goal_id)
            seeded.append({"name": name, "goal_id": new_goal_id})
            print(
                f"baseline: seeded {name} as {new_goal_id} "
                f"({i + 1}/{len(goals)})",
                file=sys.stderr,
            )

        # ── Seed persistent notes via resource/create_note queryable ─
        # These are created AFTER goals so failures don't prevent the
        # goal set from being in the snapshot. Each note is flagged
        # persistent=True so it survives transient cleanup AND gets
        # serialized to resources.json on the upcoming shutdown.
        seeded_notes_out: List[Dict[str, str]] = []
        for i, n in enumerate(seed_notes or []):
            note_name = n["name"]
            content = n["content"]
            new_note_id = _create_seed_note(
                session, character, note_name, content,
            )
            if not new_note_id:
                raise RuntimeError(
                    f"baseline: seed note '{note_name}' creation failed"
                )
            seeded_notes_out.append({
                "name": note_name,
                "note_id": new_note_id,
                "content_len": len(content),
            })
            print(
                f"baseline: seeded note '{note_name}' as {new_note_id} "
                f"({len(content)} chars) "
                f"({i + 1}/{len(seed_notes or [])})",
                file=sys.stderr,
            )
    finally:
        try:
            if session is not None:
                session.close()
        except Exception:
            pass

    # ── Stop launcher (required before snapshot) ──────────────────────
    proc.stop(timeout=30.0)
    # Give the filesystem a moment to settle after the launcher writes
    # its final state on shutdown.
    time.sleep(0.5)

    # ── Take the snapshot ─────────────────────────────────────────────
    snap_dir = snapshot.snapshot(label, character=character, world=world)

    # ── Write a seed manifest next to the snapshot ────────────────────
    manifest_extras = {
        "seeded_goals": seeded,
        "seeded_notes": seeded_notes_out,
        "goals_file_used": [{"name": g["name"]} for g in goals],
        "scenario": scenario,
    }
    extras_path = snap_dir / "_baseline_seed_manifest.json"
    extras_path.write_text(json.dumps(manifest_extras, indent=2), encoding="utf-8")

    print(
        f"baseline: created '{label}' at {snap_dir} with "
        f"{len(seeded)} goals and {len(seeded_notes_out)} seed notes",
        file=sys.stderr,
    )
    return snap_dir


# ──────────────────────────────────────────────────────────────────────
# CLI
# ──────────────────────────────────────────────────────────────────────

def main(argv: Optional[List[str]] = None) -> int:
    p = argparse.ArgumentParser(
        description="Build a truly-empty baseline snapshot for the bench "
        "experiment harness. Wipes the character resource dir, boots the "
        "launcher once to write the framework scaffold, seeds goals from "
        "a YAML file via /goal add (create-only) + /goal rename, stops "
        "the launcher, and takes a snapshot.",
    )
    sub = p.add_subparsers(dest="cmd", required=True)

    sp_create = sub.add_parser(
        "create", help="Build and save a new baseline snapshot",
    )
    sp_create.add_argument("--label", required=True,
                           help="Snapshot label (e.g. baseline-empty)")
    sp_create.add_argument("--goals-file", type=Path, required=True,
                           help="Path to the goals YAML file")
    sp_create.add_argument("--character", default=DEFAULT_CHARACTER)
    sp_create.add_argument("--world", default=DEFAULT_WORLD)
    sp_create.add_argument("--scenario", default=DEFAULT_SCENARIO)
    sp_create.add_argument("--ready-timeout", type=float,
                           default=DEFAULT_READY_TIMEOUT_S)

    args = p.parse_args(argv)

    # Clear bench log files at top-level entry so baseline creation
    # has a clean slate in the logs (same rule as experiment.py).
    clear_bench_logs()

    if args.cmd == "create":
        try:
            parsed = load_goals_file(args.goals_file)
        except Exception as e:
            print(f"ERROR: failed to load goals file: {e}", file=sys.stderr)
            return 2
        try:
            create_empty_baseline(
                label=args.label,
                goals=parsed["goals"],
                seed_notes=parsed.get("seed_notes") or [],
                character=args.character,
                world=args.world,
                scenario=args.scenario,
                ready_timeout=args.ready_timeout,
            )
        except RuntimeError as e:
            print(f"ERROR: {e}", file=sys.stderr)
            return 3
        return 0

    return 2


if __name__ == "__main__":
    sys.exit(main())
