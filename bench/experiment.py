#!/usr/bin/env python3
"""Stage 4: Batch experiment runner for the Cognitive Workbench benchmark.

Composes `trial.run_trial` calls across a list of goals with a reset
strategy determined by the trial type. Plan builders produce a
self-contained trial-plan dict that `run_experiment` executes
mechanically.

Three trial types (per docs/plan-review-eval-taskset.md):

- **Trial A** (baseline): every goal runs `[replan]` from a full restore
  of `baseline-clean`. Measures unaided-planner behavior.

- **Trial B** (accumulating review): goals are shuffled and split 50/50.
  Phase 1 (first half) runs `[replan, review commit, replay]` sequentially
  **without per-goal reset** — world model, committed cached plans, and
  persistent Notes all accumulate. A phase-boundary snapshot
  `post-B-firsthalf-seed-<seed>` captures the end-of-phase-1 state. Phase 2
  (second half) runs `[replan]` with a full restore of that snapshot before
  each goal.

- **Trial C** (replay): every goal runs from a full restore of the
  `post-B-firsthalf-seed-<seed>` snapshot. Goals that were in Trial B's
  first half get `[replay]` (their committed cached plan is in the
  snapshot); others get `[replan]`.

The `preserve=` kwarg on `snapshot.restore` is not used by these trials —
every reset is a full restore from some label — but the primitive is
kept as a general capability.

Known confound: Trial B phase 1 has no per-goal reset, so persistent
Notes from earlier first-half goals are visible to later first-half
goals and to all second-half goals (which start from the phase-1 snapshot).
This is documented in bench/README.md.

See bench/README.md for the manual baseline-creation procedure this
module assumes has already been performed.
"""
from __future__ import annotations

import argparse
import json
import random
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

_REPO_ROOT = Path(__file__).resolve().parent.parent
_SRC_DIR = _REPO_ROOT / "src"
sys.path.insert(0, str(_SRC_DIR))
sys.path.insert(0, str(Path(__file__).resolve().parent))

# Import order matters: `import launcher` must precede `import harvester`
# because harvester.py does its own `sys.path.insert(0, src_dir)` at module
# load, which shadows bench/launcher.py with src/launcher.py (the main
# agent launcher script, which has no LauncherProcess class). Loading
# bench/launcher first pins it into sys.modules before harvester runs.
import launcher   # noqa: E402
import harvester  # noqa: E402
import snapshot   # noqa: E402
from trial import run_trial  # noqa: E402

DEFAULT_CHARACTER = "Jill"
DEFAULT_WORLD = "benchlab"
DEFAULT_BASELINE_LABEL = "baseline-empty"
DEFAULT_SCENARIO = "jill-benchlab.yaml"
DEFAULT_RUN_TIMEOUT_S = 30 * 60

TRIAL_A = "A"
TRIAL_B = "B"
TRIAL_C = "C"


# ──────────────────────────────────────────────────────────────────────
# Plan builders — pure functions, unit-testable in isolation
#
# All plans are composed against a single "empty" baseline snapshot
# (created by bench/baseline.py). Per-goal reset is ALWAYS a full
# restore of the baseline; per-goal carryover (accumulated world_model,
# committed cached plans) is expressed as an explicit `overlays` list
# applied AFTER the restore and BEFORE the launcher starts.
#
# Overlay operations (interpreted by run_experiment against snapshot.py):
#   {"op": "world_model",  "source": <path>}
#   {"op": "cached_plan",  "goal_id": <str>, "source": <path>}
#
# Capture operations (what to pull out of disk after a goal run, for
# later overlay on subsequent goals):
#   {"op": "world_model",  "dest": <path>}
#   {"op": "cached_plan",  "goal_ref": <name>, "dest": <path>}
#
# This is strictly cleaner than the prior "preserve= on a big snapshot"
# model because every slice of carryover is an explicit, named file and
# every reset is a single atomic restore. See bench/README.md for the
# baseline creation procedure and the overlay architecture.
# ──────────────────────────────────────────────────────────────────────

def build_trial_A(
    goals: List[str],
    seed: int,
    baseline: str = DEFAULT_BASELINE_LABEL,
) -> Dict[str, Any]:
    """Trial A: every goal runs replan from an empty baseline.
    Order is seeded-random. No overlays, no captures.
    """
    rng = random.Random(seed)
    ordered = list(goals)
    rng.shuffle(ordered)
    per_goal_reset = {
        g: {"label": baseline, "overlays": []} for g in ordered
    }
    per_goal_protocol = {
        g: [{"kind": "run", "mode": "replan"}] for g in ordered
    }
    per_goal_capture = {g: [] for g in ordered}
    return {
        "name": f"trial-A-seed-{seed}",
        "trial_type": TRIAL_A,
        "seed": seed,
        "goals": ordered,
        "per_goal_protocol": per_goal_protocol,
        "per_goal_reset": per_goal_reset,
        "per_goal_capture": per_goal_capture,
        "metadata": {},
    }


def build_trial_B(
    goals: List[str],
    seed: int,
    baseline: str = DEFAULT_BASELINE_LABEL,
    split_ratio: float = 0.5,
    experiment_dir: str = "exp/trial-B",
) -> Dict[str, Any]:
    """Trial B: accumulating review. Seeded 50/50 split into two phases.

    Phase 1 first-half goals run `[replan, review commit, replay]` with
    full empty-baseline reset between every goal. Cross-goal carryover
    happens through overlays: before goal N, we overlay the
    world_model.json captured after goal N-1 and the cached_plan_actions
    captured for every goal in 0..N-1 that had review-commit success.

    Phase 2 second-half goals run `[replan]` with empty-baseline reset
    + the world_model from end-of-phase-1 overlaid. This isolates the
    effect of accumulated world_model learnings from any cached-plan
    effect (which phase 2 goals don't get).

    `experiment_dir` is a per-run directory where captures live.
    The runner fills in absolute paths at execution time.
    """
    rng = random.Random(seed)
    shuffled = list(goals)
    rng.shuffle(shuffled)
    n = len(shuffled)
    split = max(1, round(n * split_ratio))
    first_half = shuffled[:split]
    second_half = shuffled[split:]

    def _wm_path(step: int) -> str:
        return f"{experiment_dir}/world_model_after_{step}.json"

    def _plan_path(goal_ref: str) -> str:
        return f"{experiment_dir}/cached_plan_{goal_ref}.json"

    per_goal_protocol: Dict[str, List[Dict[str, Any]]] = {}
    per_goal_reset: Dict[str, Dict[str, Any]] = {}
    per_goal_capture: Dict[str, List[Dict[str, Any]]] = {}

    # ── Phase 1 ─────────────────────────────────────────────────────
    # Goal 0: empty baseline, no overlays.
    # Goal N (N>0): empty baseline + world_model from after goal N-1 +
    # cached_plans from all 0..N-1 that produced plans.
    for i, g in enumerate(first_half):
        per_goal_protocol[g] = [
            {"kind": "run",    "mode": "replan"},
            {"kind": "review", "commit": True},
            {"kind": "run",    "mode": "replay"},
        ]
        overlays: List[Dict[str, Any]] = []
        if i > 0:
            overlays.append({"op": "world_model", "source": _wm_path(i - 1)})
            for j in range(i):
                overlays.append({
                    "op": "cached_plan",
                    "goal_ref": first_half[j],
                    "source": _plan_path(first_half[j]),
                })
        per_goal_reset[g] = {"label": baseline, "overlays": overlays}
        # After each phase-1 goal, capture world_model + this goal's
        # cached plan so the next phase-1 goal (and phase 2) can use them.
        per_goal_capture[g] = [
            {"op": "world_model", "dest": _wm_path(i)},
            {"op": "cached_plan", "goal_ref": g, "dest": _plan_path(g)},
        ]

    # ── Phase 2 ─────────────────────────────────────────────────────
    # Each goal: empty baseline + final world_model from phase 1.
    # No cached_plan overlays (these goals were not reviewed).
    final_wm_path = _wm_path(len(first_half) - 1)
    for g in second_half:
        per_goal_protocol[g] = [{"kind": "run", "mode": "replan"}]
        per_goal_reset[g] = {
            "label": baseline,
            "overlays": [{"op": "world_model", "source": final_wm_path}],
        }
        per_goal_capture[g] = []  # nothing to carry forward from phase 2

    return {
        "name": f"trial-B-seed-{seed}",
        "trial_type": TRIAL_B,
        "seed": seed,
        "goals": first_half + second_half,
        "per_goal_protocol": per_goal_protocol,
        "per_goal_reset": per_goal_reset,
        "per_goal_capture": per_goal_capture,
        "metadata": {
            "first_half": list(first_half),
            "second_half": list(second_half),
            "experiment_dir": experiment_dir,
            "final_world_model_path": final_wm_path,
        },
    }


def build_trial_C(
    goals: List[str],
    seed: int,
    reviewed_in_b: List[str],
    final_world_model_path: str,
    cached_plan_paths: Dict[str, str],
    baseline: str = DEFAULT_BASELINE_LABEL,
) -> Dict[str, Any]:
    """Trial C: replay where possible. Every goal starts from an empty
    baseline reset + overlay of the final world_model from Trial B +
    (for reviewed goals) overlay of the corresponding cached plan.

    `reviewed_in_b` is the list of goal refs that had committed plans
    in Trial B phase 1. `final_world_model_path` is the path to the
    saved world_model.json from end-of-phase-1. `cached_plan_paths` is
    a dict mapping goal_ref → saved cached_plan.json path.

    Goals not in `reviewed_in_b` get only the world_model overlay and
    run replan (they have no cached plan to replay).
    """
    rng = random.Random(seed)
    shuffled = list(goals)
    rng.shuffle(shuffled)
    reviewed_set = set(reviewed_in_b)

    per_goal_protocol: Dict[str, List[Dict[str, Any]]] = {}
    per_goal_reset: Dict[str, Dict[str, Any]] = {}
    per_goal_capture: Dict[str, List[Dict[str, Any]]] = {}

    for g in shuffled:
        overlays: List[Dict[str, Any]] = [
            {"op": "world_model", "source": final_world_model_path},
        ]
        if g in reviewed_set:
            plan_src = cached_plan_paths.get(g)
            if not plan_src:
                raise ValueError(
                    f"build_trial_C: {g} is in reviewed_in_b but no "
                    f"cached_plan_paths entry was provided"
                )
            overlays.append({
                "op": "cached_plan",
                "goal_ref": g,
                "source": plan_src,
            })
            mode = "replay"
        else:
            mode = "replan"
        per_goal_protocol[g] = [{"kind": "run", "mode": mode}]
        per_goal_reset[g] = {"label": baseline, "overlays": overlays}
        per_goal_capture[g] = []

    return {
        "name": f"trial-C-seed-{seed}",
        "trial_type": TRIAL_C,
        "seed": seed,
        "goals": shuffled,
        "per_goal_protocol": per_goal_protocol,
        "per_goal_reset": per_goal_reset,
        "per_goal_capture": per_goal_capture,
        "metadata": {
            "reviewed_in_b": list(reviewed_in_b),
            "final_world_model_path": final_world_model_path,
            "cached_plan_paths": dict(cached_plan_paths),
        },
    }


# ──────────────────────────────────────────────────────────────────────
# Runtime helpers — small enough to monkeypatch in tests
# ──────────────────────────────────────────────────────────────────────

def _make_launcher(
    scenario: str = DEFAULT_SCENARIO,
    character: str = DEFAULT_CHARACTER,
) -> launcher.LauncherProcess:
    """Factory for a LauncherProcess. Tests monkeypatch this to return a
    fake launcher object with start/stop/wait_ready/is_running methods.
    """
    return launcher.LauncherProcess(scenario=scenario, character=character)


def _open_session():
    """Open a Zenoh session against the localhost router. Tests
    monkeypatch this to return a sentinel.
    """
    import zenoh  # noqa: WPS433
    from utils.zenoh_utils import make_localhost_config  # noqa: WPS433
    return zenoh.open(make_localhost_config())


def _trigger_agent_save_and_wait(
    session,
    character: str,
    world: str,
    *,
    timeout: float = 10.0,
    poll_interval: float = 0.1,
) -> bool:
    """Ask the running agent to flush its in-memory state to disk, and
    wait for the on-disk `resources.json` mtime to advance.

    Cached-plan and world-model captures read files from disk, but the
    agent only writes those files on shutdown / save_all / a couple of
    other handler paths — NOT after every goal completion. So the
    captured files would otherwise be stale (the pre-run state from
    the last baseline restore, not the post-run state we want).

    Publishes an empty payload to `cognitive/save_all` (a global
    subscriber that triggers `save_to_file` + `world_model.save()` in
    every running character's handler) and polls the resources.json
    mtime until it advances past the pre-publish value.

    Returns True if the file mtime advanced (save observed), False on
    timeout. Tests monkeypatch this to a no-op.
    """
    resources_path = snapshot._resource_dir(world, character) / "resources.json"
    try:
        old_mtime = (
            resources_path.stat().st_mtime
            if resources_path.exists() else 0.0
        )
    except Exception:
        old_mtime = 0.0
    try:
        session.put("cognitive/save_all", b"{}")
    except Exception as e:
        print(
            f"WARN: save_all publish failed for {character}: {e}",
            file=sys.stderr,
        )
        return False
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            cur_mtime = resources_path.stat().st_mtime
            if cur_mtime > old_mtime + 0.05:
                return True
        except Exception:
            pass
        time.sleep(poll_interval)
    return False


# Log files that individual launcher instances append to within one
# bench top-level invocation. Cleared once at the start of each
# top-level entry (experiment.py / baseline.py main()) so the logs
# accumulate across launcher bounces WITHIN one run but don't carry
# over BETWEEN separate invocations. The underlying log handlers use
# mode='a' per src/executive_node.py etc., so bench bounces naturally
# accumulate after the initial clear.
_BENCH_LOG_FILES = [
    _SRC_DIR / "logs" / "executive_node.log",
    _SRC_DIR / "logs" / "character_launcher.log",
    _REPO_ROOT / "logs" / "incremental_planner.log",
    _REPO_ROOT / "logs" / "blackboard_planner.log",
]


def clear_bench_logs() -> None:
    """Remove the agent-side log files that a bench invocation will
    write to. Best-effort: missing files are silently ignored, errors
    are logged but non-fatal.
    """
    for path in _BENCH_LOG_FILES:
        try:
            if path.exists():
                path.unlink()
        except Exception as e:
            print(
                f"WARN: could not clear log file {path}: {e}",
                file=sys.stderr,
            )


def _preflight_shutdown_external_launcher(
    character: str,
    *,
    shutdown_timeout: float = 30.0,
    poll_interval: float = 1.0,
) -> bool:
    """If a launcher for `character` is already running (started by
    someone other than this runner), send the standard Zenoh shutdown
    signal and wait for it to go down.

    Returns True if no external launcher was running OR one was running
    and came down cleanly. Returns False if a launcher was running and
    did not come down within the timeout. Tests monkeypatch this.
    """
    if not snapshot.is_launcher_running(character, timeout=1.0):
        return True

    print(
        f"preflight: external launcher detected for '{character}'; "
        f"sending Zenoh shutdown signal",
        file=sys.stderr,
    )
    try:
        import zenoh  # noqa: WPS433
        from utils.zenoh_utils import make_localhost_config  # noqa: WPS433
    except ImportError:
        print("preflight: zenoh import failed; cannot signal shutdown",
              file=sys.stderr)
        return False

    try:
        session = zenoh.open(make_localhost_config())
    except Exception as e:
        print(f"preflight: zenoh open failed: {e}", file=sys.stderr)
        return False

    try:
        payload = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "source": "bench-experiment-preflight",
            "mode": "save_and_shutdown",
        }
        session.put(
            "cognitive/launcher/shutdown",
            json.dumps(payload).encode("utf-8"),
        )
    except Exception as e:
        print(f"preflight: shutdown put failed: {e}", file=sys.stderr)
        try:
            session.close()
        except Exception:
            pass
        return False

    try:
        session.close()
    except Exception:
        pass

    deadline = time.monotonic() + shutdown_timeout
    while time.monotonic() < deadline:
        if not snapshot.is_launcher_running(character, timeout=1.0):
            print(
                f"preflight: external launcher for '{character}' is down",
                file=sys.stderr,
            )
            return True
        time.sleep(poll_interval)

    print(
        f"preflight: external launcher for '{character}' did NOT stop "
        f"within {shutdown_timeout}s",
        file=sys.stderr,
    )
    return False


# ──────────────────────────────────────────────────────────────────────
# Runner
# ──────────────────────────────────────────────────────────────────────

def _apply_overlays(
    overlays: List[Dict[str, Any]],
    *,
    character: str,
    world: str,
    goal_ref_to_id: Dict[str, str],
) -> List[str]:
    """Apply a list of overlay operations against the currently-restored
    baseline. Returns a human-readable list describing what was applied
    (for the batch_runner metadata).

    Cached-plan overlays target a specific goal_id. The overlay spec
    references a goal_ref (e.g. "G01") which must be resolvable to the
    current goal_id on the just-restored baseline. We use a
    caller-supplied goal_ref_to_id dict built by querying the launcher
    once per bounce.
    """
    applied: List[str] = []
    for ov in overlays:
        op = ov.get("op")
        if op == "world_model":
            snapshot.apply_world_model_overlay(
                Path(ov["source"]),
                character=character,
                world=world,
                force=True,  # launcher is stopped at this point by the runner
            )
            applied.append(f"world_model:{ov['source']}")
        elif op == "cached_plan":
            goal_ref = ov["goal_ref"]
            gid = goal_ref_to_id.get(goal_ref)
            if not gid:
                raise RuntimeError(
                    f"cached_plan overlay references goal_ref '{goal_ref}' "
                    f"which is not in the current baseline. Available: "
                    f"{sorted(goal_ref_to_id.keys())}"
                )
            snapshot.apply_cached_plan_overlay(
                gid,
                Path(ov["source"]),
                character=character,
                world=world,
                force=True,
            )
            applied.append(f"cached_plan:{goal_ref}={ov['source']}")
        else:
            raise RuntimeError(f"unknown overlay op: {op!r}")
    return applied


def _apply_captures(
    captures: List[Dict[str, Any]],
    *,
    character: str,
    world: str,
    goal_ref_to_id: Dict[str, str],
) -> List[str]:
    """Apply capture operations AFTER a goal run completes, saving
    named files that subsequent goals can overlay. Returns a
    human-readable list for the batch_runner metadata.
    """
    captured: List[str] = []
    for cap in captures:
        op = cap.get("op")
        if op == "world_model":
            dest = Path(cap["dest"])
            snapshot.capture_world_model(dest, character=character, world=world)
            captured.append(f"world_model:{dest}")
        elif op == "cached_plan":
            goal_ref = cap["goal_ref"]
            gid = goal_ref_to_id.get(goal_ref)
            if not gid:
                raise RuntimeError(
                    f"cached_plan capture references goal_ref '{goal_ref}' "
                    f"not in current baseline"
                )
            dest = Path(cap["dest"])
            snapshot.capture_cached_plan(
                gid, dest, character=character, world=world,
            )
            captured.append(f"cached_plan:{goal_ref}={dest}")
        else:
            raise RuntimeError(f"unknown capture op: {op!r}")
    return captured


def _resolve_goal_ref_map(session, character: str) -> Dict[str, str]:
    """Query scheduled_goals once and build a name→goal_id map for
    every goal currently known to the launcher. Used by overlay/capture
    to translate user-facing goal_refs into goal_ids.
    """
    out: Dict[str, str] = {}
    # We reuse resolve_goal_handle by querying scheduled_goals directly.
    # For the batch runner we need the full map, not one lookup at a time.
    key = f"cognitive/{character}/scheduled_goals"
    try:
        for reply in session.get(key, timeout=3.0):
            if not (hasattr(reply, "ok") and reply.ok is not None):
                continue
            try:
                payload = json.loads(reply.ok.payload.to_bytes().decode("utf-8"))
            except Exception:
                continue
            for g in payload.get("goals", []) or []:
                name = g.get("name")
                gid = g.get("goal_id")
                if name and gid:
                    out[name] = gid
                if gid:
                    out[gid] = gid  # allow goal_id passthrough too
    except Exception as e:
        print(f"WARN: scheduled_goals query failed: {e}", file=sys.stderr)
    return out


def run_experiment(
    plan: Dict[str, Any],
    *,
    character: str = DEFAULT_CHARACTER,
    world: str = DEFAULT_WORLD,
    out_path: Optional[Path] = None,
    scenario: str = DEFAULT_SCENARIO,
    run_timeout: float = DEFAULT_RUN_TIMEOUT_S,
) -> List[Dict[str, Any]]:
    """Execute a trial plan end-to-end.

    For each goal in plan["goals"] in order:
      1. Stop launcher, restore baseline snapshot, apply per-goal overlays,
         start launcher, wait_ready.
      2. Open a Zenoh session, resolve the goal ref, call run_trial.
      3. Apply per-goal captures (save world_model / cached_plan to disk
         for subsequent goals to overlay).
      4. Append the trial record (plus batch_runner metadata) to out_path.

    Per-goal failures (session open, goal resolve, trial error) produce
    a synthetic failure record and the loop continues. Fatal errors
    (snapshot failed, launcher refused to come ready, overlay surgery
    failed) abort the experiment with fatal_error set in the summary.
    Whatever records were already written stay on disk.

    Returns the list of trial records (same ones appended to out_path).
    """
    records: List[Dict[str, Any]] = []
    launcher_proc = _make_launcher(scenario=scenario, character=character)
    experiment_started = datetime.now(timezone.utc).isoformat(timespec="seconds")
    fatal_error: Optional[str] = None

    def _append(rec: Dict[str, Any]) -> None:
        records.append(rec)
        if out_path is not None:
            with open(out_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(rec, default=str) + "\n")

    # Preflight: if an externally-started launcher is already running,
    # ask it to shut down via the standard Zenoh signal before we
    # restore. Our LauncherProcess instance can't stop a process it
    # didn't start, so without this step the first snapshot.restore()
    # would fail its liveness check.
    if not _preflight_shutdown_external_launcher(character):
        fatal_error = (
            f"preflight: external launcher for '{character}' did not "
            f"shut down when signaled — aborting before first reset"
        )

    try:
        if fatal_error:
            plan_goals_iter: List[str] = []
        else:
            plan_goals_iter = plan["goals"]

        for idx, goal_ref in enumerate(plan_goals_iter):
            reset = plan["per_goal_reset"][goal_ref]
            protocol = plan["per_goal_protocol"][goal_ref]
            captures = plan.get("per_goal_capture", {}).get(goal_ref, [])
            bounce_start = time.monotonic()
            overlays_applied: List[str] = []
            captures_applied: List[str] = []

            # ── Reset + overlays + launcher start ──
            try:
                if launcher_proc.is_running():
                    launcher_proc.stop()
                snapshot.restore(reset["label"], character=character, world=world)
                # Overlays apply while launcher is stopped so disk writes
                # are not racing with the agent.
                if reset.get("overlays"):
                    # Overlays only target world_model.json and specific
                    # Note content — none of which need goal_ref→id
                    # resolution until we do cached_plan. For cached_plan,
                    # the overlay references a goal_ref (e.g. G01) which
                    # resolves to a goal_id that lives in the restored
                    # baseline's resources.json. We need to know that
                    # mapping without a running launcher. Trick: parse
                    # resources.json ourselves, build a name→id map by
                    # reading scheduled-goal Notes directly.
                    baseline_id_map = _resolve_goal_ref_map_from_resources(
                        character=character, world=world,
                    )
                    overlays_applied = _apply_overlays(
                        reset["overlays"],
                        character=character,
                        world=world,
                        goal_ref_to_id=baseline_id_map,
                    )
                launcher_proc.start()
                launcher_proc.wait_ready()
            except Exception as e:
                fatal_error = (
                    f"goal {idx} ({goal_ref}): reset/overlay/start failed: "
                    f"{type(e).__name__}: {e}"
                )
                break

            bounce_ms = int((time.monotonic() - bounce_start) * 1000)
            wait_ready_ms = bounce_ms  # bounce == whole reset+start cycle

            # ── Open a session, resolve goal, run the trial ──
            session = None
            try:
                session = _open_session()
            except Exception as e:
                _append(_failure_record(
                    plan, idx, goal_ref, reset, bounce_ms, wait_ready_ms,
                    f"zenoh session open failed: {type(e).__name__}: {e}",
                    overlays_applied=overlays_applied,
                ))
                continue

            try:
                pre_record = harvester.resolve_goal_handle(
                    session, character, goal_ref
                )
                if pre_record is None:
                    _append(_failure_record(
                        plan, idx, goal_ref, reset, bounce_ms, wait_ready_ms,
                        f"could not resolve goal ref '{goal_ref}'",
                        overlays_applied=overlays_applied,
                    ))
                    continue
                goal_id = pre_record["goal_id"]

                trial_record = run_trial(
                    goal_id,
                    steps=protocol,
                    session=session,
                    character=character,
                    trial_context=_trial_context(plan, idx, goal_ref),
                    run_timeout=run_timeout,
                )
                trial_record["batch_runner"] = {
                    "reset_label": reset["label"],
                    "overlays_applied": overlays_applied,
                    "launcher_bounce_ms": bounce_ms,
                    "wait_ready_ms": wait_ready_ms,
                    "goal_index": idx,
                    "goals_in_plan": len(plan["goals"]),
                }

                # ── Captures (run AFTER the trial completes). The agent
                #     only writes resources.json / world_model.json on
                #     shutdown or save_all — NOT after every goal — so
                #     we trigger save_all and wait for the mtime to
                #     advance before reading. Without this the capture
                #     reads stale pre-run state (e.g. empty
                #     cached_plan_actions) and silently captures
                #     garbage.
                if captures:
                    try:
                        saved = _trigger_agent_save_and_wait(
                            session, character, world,
                        )
                        if not saved:
                            print(
                                f"WARN: agent save_all did not flush "
                                f"resources.json within timeout — capture "
                                f"for {goal_ref} may read stale data",
                                file=sys.stderr,
                            )
                        live_id_map = _resolve_goal_ref_map(session, character)
                        captures_applied = _apply_captures(
                            captures,
                            character=character,
                            world=world,
                            goal_ref_to_id=live_id_map,
                        )
                        trial_record["batch_runner"]["captures_applied"] = captures_applied
                    except Exception as e:
                        # Capture failure is recorded but doesn't fail
                        # the run step — the row is still useful.
                        trial_record["batch_runner"]["capture_error"] = (
                            f"{type(e).__name__}: {e}"
                        )

                _append(trial_record)
            finally:
                try:
                    if session is not None:
                        session.close()
                except Exception:
                    pass

    finally:
        try:
            if launcher_proc.is_running():
                launcher_proc.stop()
        except Exception:
            pass

    # ── Write summary sidecar ──
    if out_path is not None:
        summary = {
            "plan_name": plan["name"],
            "trial_type": plan["trial_type"],
            "seed": plan["seed"],
            "character": character,
            "scenario": scenario,
            "started_at": experiment_started,
            "finished_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "goals_attempted": len(records),
            "goals_in_plan": len(plan["goals"]),
            "errored_records": sum(1 for r in records if r.get("error")),
            "fatal_error": fatal_error,
        }
        summary_path = Path(str(out_path) + ".summary.json")
        with open(summary_path, "w", encoding="utf-8") as f:
            json.dump(
                {"summary": summary, "metadata": plan.get("metadata", {})},
                f,
                indent=2,
            )

    return records


def _resolve_goal_ref_map_from_resources(
    *, character: str, world: str,
) -> Dict[str, str]:
    """Build a name→goal_id map by reading resources.json directly.
    Used during overlay application when the launcher is stopped (so
    we can't query scheduled_goals via Zenoh).
    """
    resources_path = (
        snapshot._resource_dir(world, character) / "resources.json"
    )
    out: Dict[str, str] = {}
    if not resources_path.exists():
        return out
    try:
        with open(resources_path, "r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception:
        return out
    notes = data.get("note_instances") or {}
    for note in notes.values():
        props = note.get("properties") or {}
        note_name = props.get("note_name") or ""
        if not note_name.startswith("_scheduled_goal_"):
            continue
        content_str = props.get("content") or "{}"
        try:
            content = json.loads(content_str) if isinstance(content_str, str) else content_str
        except Exception:
            continue
        if not isinstance(content, dict):
            continue
        gid = content.get("goal_id")
        name = content.get("name")
        if gid and name:
            out[name] = gid
        if gid:
            out[gid] = gid  # passthrough for callers that already have the id
    return out


def _trial_context(plan: Dict[str, Any], idx: int, goal_ref: str) -> Dict[str, Any]:
    """Build the trial_context dict stamped on every trial record."""
    ctx: Dict[str, Any] = {
        "plan_name": plan["name"],
        "trial_type": plan["trial_type"],
        "seed": plan["seed"],
        "goal_ref": goal_ref,
        "goal_index": idx,
    }
    if plan["trial_type"] == TRIAL_B:
        first_half = plan.get("metadata", {}).get("first_half", [])
        ctx["first_half"] = goal_ref in first_half
    return ctx


def _failure_record(
    plan: Dict[str, Any],
    idx: int,
    goal_ref: str,
    reset: Optional[Dict[str, Any]],
    bounce_ms: int,
    wait_ready_ms: int,
    error: str,
    *,
    overlays_applied: Optional[List[str]] = None,
) -> Dict[str, Any]:
    """Build a trial-shaped record for a failure that happened before
    run_trial could be invoked (session open, goal resolve).
    """
    return {
        "trial_context": _trial_context(plan, idx, goal_ref),
        "goal_id": None,
        "steps": [],
        "error": error,
        "started_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "elapsed_ms": 0,
        "batch_runner": {
            "reset_label": reset["label"] if reset else None,
            "overlays_applied": list(overlays_applied or []),
            "launcher_bounce_ms": bounce_ms,
            "wait_ready_ms": wait_ready_ms,
            "goal_index": idx,
            "goals_in_plan": len(plan["goals"]),
        },
    }


# ──────────────────────────────────────────────────────────────────────
# Small helpers for Trial C chaining from a Trial B JSONL
# ──────────────────────────────────────────────────────────────────────

def read_reviewed_from_jsonl(path: Path) -> List[str]:
    """Extract first-half goal refs from a Trial B JSONL file. Used to
    build Trial C without hand-typing the reviewed list.
    """
    reviewed: List[str] = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                row = json.loads(line)
            except Exception:
                continue
            ctx = row.get("trial_context") or {}
            if ctx.get("trial_type") == TRIAL_B and ctx.get("first_half") is True:
                goal_ref = ctx.get("goal_ref")
                if goal_ref and goal_ref not in reviewed:
                    reviewed.append(goal_ref)
    return reviewed


# ──────────────────────────────────────────────────────────────────────
# CLI
# ──────────────────────────────────────────────────────────────────────

def _flatten_goals(goals_arg: List[str]) -> List[str]:
    """argparse nargs='+' gives us a list; also accept whitespace-separated
    refs within a single element so --goals "G01 G02 G03" works.
    """
    out: List[str] = []
    for item in goals_arg:
        out.extend(item.split())
    return [g.strip() for g in out if g.strip()]


def main(argv: Optional[List[str]] = None) -> int:
    p = argparse.ArgumentParser(
        description="Stage 4 batch experiment runner for the Cognitive "
        "Workbench benchmark. See bench/README.md for setup procedure.",
    )
    sub = p.add_subparsers(dest="cmd", required=True)

    def _add_common(sp: argparse.ArgumentParser) -> None:
        sp.add_argument(
            "--goals", nargs="+", required=True,
            help="Goal refs (names or goal_ids), e.g. --goals G01 G02 G03",
        )
        sp.add_argument(
            "--seed", type=int, required=True,
            help="Random seed for goal ordering (and Trial B split)",
        )
        sp.add_argument("--character", default=DEFAULT_CHARACTER)
        sp.add_argument("--baseline", default=DEFAULT_BASELINE_LABEL)
        sp.add_argument(
            "--out", type=Path, default=None,
            help="Append per-goal trial records to this JSONL file. A "
                 "<out>.summary.json sidecar is also written.",
        )
        sp.add_argument("--scenario", default=DEFAULT_SCENARIO)
        sp.add_argument("--run-timeout", type=float, default=DEFAULT_RUN_TIMEOUT_S)

    sp_a = sub.add_parser("trial-A", help="Run Trial A (baseline)")
    _add_common(sp_a)

    sp_b = sub.add_parser("trial-B", help="Run Trial B (accumulating review)")
    _add_common(sp_b)
    sp_b.add_argument(
        "--split-ratio", type=float, default=0.5,
        help="Fraction of goals in phase 1 (default 0.5)",
    )
    sp_b.add_argument(
        "--experiment-dir", type=Path, default=None,
        help="Directory to write world_model / cached_plan capture "
             "sidecar files. Default: exp/trial-B-seed-<seed>/",
    )

    sp_c = sub.add_parser("trial-C", help="Run Trial C (replay where available)")
    _add_common(sp_c)
    sp_c.add_argument(
        "--trial-b-dir", type=Path, required=True,
        help="Directory containing the capture sidecar files produced "
             "by a prior trial-B run (world_model_after_*.json, "
             "cached_plan_<GOAL>.json). Trial C reads these as overlay "
             "sources.",
    )
    group = sp_c.add_mutually_exclusive_group(required=True)
    group.add_argument(
        "--reviewed-in-b-from", type=Path, default=None,
        help="Extract first-half goal refs from a Trial B JSONL file",
    )
    group.add_argument(
        "--reviewed-in-b", nargs="+", default=None,
        help="Explicit list of goal refs that were reviewed in Trial B",
    )

    args = p.parse_args(argv)
    goals = _flatten_goals(args.goals)
    if not goals:
        print("ERROR: --goals is empty after parsing", file=sys.stderr)
        return 2

    # Clear bench log files at the top-level entry. Within one
    # experiment.py invocation, all launcher bounces append to these
    # files, so debugging a mid-run failure has full context.
    clear_bench_logs()

    if args.cmd == "trial-A":
        plan = build_trial_A(goals, args.seed, baseline=args.baseline)
    elif args.cmd == "trial-B":
        exp_dir = args.experiment_dir or Path(f"exp/trial-B-seed-{args.seed}")
        exp_dir.mkdir(parents=True, exist_ok=True)
        plan = build_trial_B(
            goals, args.seed,
            baseline=args.baseline,
            split_ratio=args.split_ratio,
            experiment_dir=str(exp_dir),
        )
    elif args.cmd == "trial-C":
        if args.reviewed_in_b_from:
            reviewed = read_reviewed_from_jsonl(args.reviewed_in_b_from)
            if not reviewed:
                print(
                    f"WARN: no first-half goal refs found in "
                    f"{args.reviewed_in_b_from}; Trial C will replan all goals",
                    file=sys.stderr,
                )
        else:
            reviewed = _flatten_goals(args.reviewed_in_b)

        # Derive the final world_model path from the trial-B dir by
        # finding the highest-numbered world_model_after_N.json file.
        trial_b_dir = args.trial_b_dir
        wm_files = sorted(trial_b_dir.glob("world_model_after_*.json"))
        if not wm_files:
            print(
                f"ERROR: no world_model_after_*.json files found in "
                f"{trial_b_dir}. Did Trial B complete successfully?",
                file=sys.stderr,
            )
            return 2
        final_wm = str(wm_files[-1])
        cached_plan_paths: Dict[str, str] = {}
        for g in reviewed:
            p_path = trial_b_dir / f"cached_plan_{g}.json"
            if not p_path.exists():
                print(
                    f"ERROR: reviewed goal {g} has no cached_plan file at "
                    f"{p_path}. Trial B capture may have failed for this goal.",
                    file=sys.stderr,
                )
                return 2
            cached_plan_paths[g] = str(p_path)

        plan = build_trial_C(
            goals, args.seed,
            reviewed_in_b=reviewed,
            final_world_model_path=final_wm,
            cached_plan_paths=cached_plan_paths,
            baseline=args.baseline,
        )
    else:
        return 2

    print(
        f"Experiment plan: {plan['name']} "
        f"({len(plan['goals'])} goals, protocol={args.cmd})",
        file=sys.stderr,
    )
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)

    records = run_experiment(
        plan,
        character=args.character,
        out_path=args.out,
        scenario=args.scenario,
        run_timeout=args.run_timeout,
    )

    n_errored = sum(1 for r in records if r.get("error"))
    print(
        f"Experiment complete: {len(records)}/{len(plan['goals'])} goals "
        f"attempted, {n_errored} errored",
        file=sys.stderr,
    )
    return 0 if n_errored == 0 and len(records) == len(plan["goals"]) else 4


if __name__ == "__main__":
    sys.exit(main())
