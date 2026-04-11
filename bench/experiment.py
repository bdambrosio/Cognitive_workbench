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
DEFAULT_BASELINE_LABEL = "baseline-clean"
DEFAULT_SCENARIO = "jill-infospace-bench.yaml"
DEFAULT_RUN_TIMEOUT_S = 30 * 60

TRIAL_A = "A"
TRIAL_B = "B"
TRIAL_C = "C"


# ──────────────────────────────────────────────────────────────────────
# Plan builders — pure functions, unit-testable in isolation
# ──────────────────────────────────────────────────────────────────────

def build_trial_A(
    goals: List[str],
    seed: int,
    baseline: str = DEFAULT_BASELINE_LABEL,
) -> Dict[str, Any]:
    """Trial A: every goal runs replan from a full baseline restore.
    Order is seeded-random. Returns a trial-plan dict.
    """
    rng = random.Random(seed)
    ordered = list(goals)
    rng.shuffle(ordered)
    return {
        "name": f"trial-A-seed-{seed}",
        "trial_type": TRIAL_A,
        "seed": seed,
        "goals": ordered,
        "per_goal_protocol": {
            g: [{"kind": "run", "mode": "replan"}] for g in ordered
        },
        "per_goal_reset": {
            g: {"label": baseline, "preserve": []} for g in ordered
        },
        "phase_boundaries": {},
        "post_trial_snapshot": None,
        "metadata": {},
    }


def build_trial_B(
    goals: List[str],
    seed: int,
    baseline: str = DEFAULT_BASELINE_LABEL,
    split_ratio: float = 0.5,
) -> Dict[str, Any]:
    """Trial B: accumulating review. Seeded split into two phases.

    Phase 1 runs `[replan, review commit, replay]` on each first-half goal
    with NO per-goal reset so cached plans survive. A phase-boundary
    snapshot is taken after phase 1. Phase 2 runs `[replan]` on each
    second-half goal, restoring the phase-1 snapshot before each run.
    """
    rng = random.Random(seed)
    shuffled = list(goals)
    rng.shuffle(shuffled)
    n = len(shuffled)
    split = max(1, round(n * split_ratio))
    first_half = shuffled[:split]
    second_half = shuffled[split:]

    snapshot_label = f"post-B-firsthalf-seed-{seed}"

    per_goal_protocol: Dict[str, List[Dict[str, Any]]] = {}
    per_goal_reset: Dict[str, Optional[Dict[str, Any]]] = {}

    for i, g in enumerate(first_half):
        per_goal_protocol[g] = [
            {"kind": "run",    "mode": "replan"},
            {"kind": "review", "commit": True},
            {"kind": "run",    "mode": "replay"},
        ]
        # Only the very first goal of the whole trial gets a reset.
        # Subsequent first-half goals share the launcher so cached plans
        # and world-model accumulation are preserved.
        per_goal_reset[g] = (
            {"label": baseline, "preserve": []} if i == 0 else None
        )

    for g in second_half:
        per_goal_protocol[g] = [{"kind": "run", "mode": "replan"}]
        per_goal_reset[g] = {"label": snapshot_label, "preserve": []}

    return {
        "name": f"trial-B-seed-{seed}",
        "trial_type": TRIAL_B,
        "seed": seed,
        "goals": first_half + second_half,
        "per_goal_protocol": per_goal_protocol,
        "per_goal_reset": per_goal_reset,
        "phase_boundaries": {
            # After the last first-half index, snapshot post-firsthalf.
            len(first_half) - 1: {
                "action": "snapshot",
                "label": snapshot_label,
            },
        },
        "post_trial_snapshot": None,
        "metadata": {
            "first_half": list(first_half),
            "second_half": list(second_half),
            "post_firsthalf_snapshot": snapshot_label,
        },
    }


def build_trial_C(
    goals: List[str],
    seed: int,
    post_firsthalf_label: str,
    reviewed_in_b: List[str],
) -> Dict[str, Any]:
    """Trial C: replay where possible. Every goal starts from a full
    restore of `post_firsthalf_label`. Goals in `reviewed_in_b` run
    `[replay]`; the rest run `[replan]`.
    """
    rng = random.Random(seed)
    shuffled = list(goals)
    rng.shuffle(shuffled)
    reviewed_set = set(reviewed_in_b)

    per_goal_protocol = {}
    per_goal_reset = {}
    for g in shuffled:
        mode = "replay" if g in reviewed_set else "replan"
        per_goal_protocol[g] = [{"kind": "run", "mode": mode}]
        per_goal_reset[g] = {"label": post_firsthalf_label, "preserve": []}

    return {
        "name": f"trial-C-seed-{seed}",
        "trial_type": TRIAL_C,
        "seed": seed,
        "goals": shuffled,
        "per_goal_protocol": per_goal_protocol,
        "per_goal_reset": per_goal_reset,
        "phase_boundaries": {},
        "post_trial_snapshot": None,
        "metadata": {
            "source_snapshot": post_firsthalf_label,
            "reviewed_in_b": list(reviewed_in_b),
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

def run_experiment(
    plan: Dict[str, Any],
    *,
    character: str = DEFAULT_CHARACTER,
    out_path: Optional[Path] = None,
    scenario: str = DEFAULT_SCENARIO,
    run_timeout: float = DEFAULT_RUN_TIMEOUT_S,
) -> List[Dict[str, Any]]:
    """Execute a trial plan end-to-end.

    For each goal in plan["goals"] in order:
      1. If per_goal_reset[goal] is not None, stop launcher, restore
         snapshot (with optional preserve=), start launcher, wait_ready.
      2. Open a Zenoh session, resolve the goal ref, call run_trial.
      3. Append the trial record (plus batch_runner metadata) to out_path.
      4. If this goal index is a phase boundary, stop launcher, snapshot,
         start launcher, wait_ready.

    Per-goal failures (session open, goal resolve, trial error) produce a
    synthetic failure record and the loop continues. Fatal errors (snapshot
    failed, launcher refused to come ready) abort the experiment with
    fatal_error set in the summary; whatever records were already written
    stay on disk.

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

    # Preflight: if an externally-started launcher is already running for
    # this character, ask it to shut down via the standard Zenoh signal
    # before we try to snapshot/restore. Our LauncherProcess instance can't
    # stop a process it didn't start, so without this step the first
    # snapshot.restore() would fail its liveness check.
    if not _preflight_shutdown_external_launcher(character):
        fatal_error = (
            f"preflight: external launcher for '{character}' did not "
            f"shut down when signaled — aborting before first reset"
        )

    try:
        if fatal_error:
            # Skip the main loop but let the finally+summary block run.
            plan_goals_iter: List[str] = []
        else:
            plan_goals_iter = plan["goals"]
        for idx, goal_ref in enumerate(plan_goals_iter):
            reset = plan["per_goal_reset"][goal_ref]
            protocol = plan["per_goal_protocol"][goal_ref]
            bounce_start = time.monotonic()
            wait_ready_ms = 0

            # ── Reset if requested ──
            if reset is not None:
                try:
                    if launcher_proc.is_running():
                        launcher_proc.stop()
                    snapshot.restore(
                        reset["label"],
                        character=character,
                        preserve=reset.get("preserve") or None,
                    )
                    launcher_proc.start()
                    launcher_proc.wait_ready()
                except Exception as e:
                    fatal_error = (
                        f"goal {idx} ({goal_ref}): reset/start failed: "
                        f"{type(e).__name__}: {e}"
                    )
                    break
                wait_ready_ms = int((time.monotonic() - bounce_start) * 1000)
            else:
                # No reset requested — launcher must already be running
                # from a prior goal in this phase.
                if not launcher_proc.is_running():
                    fatal_error = (
                        f"goal {idx} ({goal_ref}): no reset requested but "
                        "launcher is not running. First goal of a phase "
                        "must specify a reset."
                    )
                    break

            bounce_ms = int((time.monotonic() - bounce_start) * 1000)

            # ── Open a session, resolve goal, run the trial ──
            try:
                session = _open_session()
            except Exception as e:
                _append(_failure_record(
                    plan, idx, goal_ref, reset, bounce_ms, wait_ready_ms,
                    f"zenoh session open failed: {type(e).__name__}: {e}",
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
                    "reset_label": reset["label"] if reset else None,
                    "reset_preserve": list(reset.get("preserve") or []) if reset else [],
                    "launcher_bounce_ms": bounce_ms,
                    "wait_ready_ms": wait_ready_ms,
                    "goal_index": idx,
                    "goals_in_plan": len(plan["goals"]),
                }
                _append(trial_record)
            finally:
                try:
                    session.close()
                except Exception:
                    pass

            # ── Phase boundary snapshot (e.g. end of Trial B first half) ──
            boundary = plan.get("phase_boundaries", {}).get(idx)
            if boundary and boundary.get("action") == "snapshot":
                label = boundary["label"]
                try:
                    if launcher_proc.is_running():
                        launcher_proc.stop()
                    snapshot.snapshot(label, character=character)
                    launcher_proc.start()
                    launcher_proc.wait_ready()
                except Exception as e:
                    fatal_error = (
                        f"phase boundary snapshot '{label}' failed after "
                        f"goal {idx}: {type(e).__name__}: {e}"
                    )
                    break

        # ── Optional end-of-trial snapshot ──
        if fatal_error is None and plan.get("post_trial_snapshot"):
            label = plan["post_trial_snapshot"]
            try:
                if launcher_proc.is_running():
                    launcher_proc.stop()
                snapshot.snapshot(label, character=character)
            except Exception as e:
                fatal_error = (
                    f"post-trial snapshot '{label}' failed: "
                    f"{type(e).__name__}: {e}"
                )
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
            "reset_preserve": list(reset.get("preserve") or []) if reset else [],
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

    sp_c = sub.add_parser("trial-C", help="Run Trial C (replay where available)")
    _add_common(sp_c)
    sp_c.add_argument(
        "--post-firsthalf-label", required=True,
        help="Snapshot label produced by Trial B, "
             "e.g. post-B-firsthalf-seed-42",
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

    if args.cmd == "trial-A":
        plan = build_trial_A(goals, args.seed, baseline=args.baseline)
    elif args.cmd == "trial-B":
        plan = build_trial_B(
            goals, args.seed,
            baseline=args.baseline,
            split_ratio=args.split_ratio,
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
        plan = build_trial_C(
            goals, args.seed,
            post_firsthalf_label=args.post_firsthalf_label,
            reviewed_in_b=reviewed,
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
