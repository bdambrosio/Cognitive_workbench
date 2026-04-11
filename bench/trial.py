#!/usr/bin/env python3
"""Stage 3e: Trial driver for the benchmark harness.

A **trial** is one self-contained experimental record: "execute this
sequence of steps against the running agent for this goal, and return
everything that happened." Trials are composed of primitive steps from
a small vocabulary:

  - {"kind": "run", "mode": "replan" | "replay"}
      Submit the goal via the harvester, wait for completion, compute
      the harvester row, run the verdict grader. Produces a step dict
      with `row` and `verdict` sub-fields.

  - {"kind": "review", "commit": bool}
      Invoke the automated reviewer. If commit=True, the rewritten plan
      is committed (goal switches to replay mode, learnings injected
      into the world model). If commit=False, the reviewer runs in
      dry-run mode — the LLM call still happens (so we capture the
      rewrite + learnings as observational data), but nothing is
      mutated and no files are left behind. Produces a step dict with
      a `review` sub-field.

Trials are ordered sequences of these steps. The step list is the
trial's protocol — there is no trial_kind enum. Callers pass whatever
step sequence they need for their experiment.

The caller also supplies an opaque `trial_context` (any JSON-serialisable
value: dict, string, int, whatever) that gets embedded verbatim in the
trial record. trial.py never inspects it. Orchestrators use it to label
trials with phase/batch/split/seed metadata for post-processing.

Failure handling: fail fast. If any step errors, the error is recorded
on that step AND on the trial's top-level `error` field, subsequent
steps are skipped, and the trial returns. This keeps the data clean —
you never see a "post-review run" that happened without a prior review.

Usage as a library:

    from trial import run_trial

    result = run_trial(
        "goal_15",
        steps=[
            {"kind": "run",    "mode": "replan"},
            {"kind": "review", "commit": True},
            {"kind": "run",    "mode": "replay"},
        ],
        trial_context={"phase": "A", "split_seed": 42},
    )

CLI:

    python bench/trial.py run goal_15 --steps '[
        {"kind": "run", "mode": "replan"},
        {"kind": "review", "commit": false},
        {"kind": "run", "mode": "replay"}
    ]' --context '{"phase": "A"}'

Requires the launcher to be running for the target character.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

# Path setup so sibling bench/ modules import cleanly.
_REPO_ROOT = Path(__file__).resolve().parent.parent
_SRC_DIR = _REPO_ROOT / "src"
sys.path.insert(0, str(_SRC_DIR))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import harvester  # noqa: E402
from verdict import (  # noqa: E402
    DEFAULT_MODEL as VERDICT_DEFAULT_MODEL,
    compute_verdict,
    _open_zenoh_session,
)
from reviewer import (  # noqa: E402
    DEFAULT_MODEL as REVIEWER_DEFAULT_MODEL,
    review_goal,
)

DEFAULT_CHARACTER = "Jill"
DEFAULT_RUN_TIMEOUT_S = 30 * 60  # 30 minutes; harvester-compatible
DEFAULT_POLL_INTERVAL_S = 3.0


# ──────────────────────────────────────────────────────────────────────────
# Step validation
# ──────────────────────────────────────────────────────────────────────────

_VALID_RUN_MODES = {"replan", "replay"}
_VALID_KINDS = {"run", "review"}


def _validate_steps(steps: List[Dict[str, Any]]) -> Optional[str]:
    """Return a human-readable error string if the step list is malformed,
    None if it's valid. Called before execution so we don't half-run a
    trial with a typo.
    """
    if not isinstance(steps, list) or not steps:
        return "steps must be a non-empty list"
    for i, step in enumerate(steps):
        if not isinstance(step, dict):
            return f"step {i}: not a dict"
        kind = step.get("kind")
        if kind not in _VALID_KINDS:
            return f"step {i}: kind must be one of {_VALID_KINDS}, got {kind!r}"
        if kind == "run":
            mode = step.get("mode")
            if mode not in _VALID_RUN_MODES:
                return (
                    f"step {i}: run mode must be one of {_VALID_RUN_MODES}, "
                    f"got {mode!r}"
                )
        elif kind == "review":
            if "commit" not in step:
                return f"step {i}: review step must specify 'commit' (bool)"
            if not isinstance(step["commit"], bool):
                return f"step {i}: review commit must be bool"
    return None


# ──────────────────────────────────────────────────────────────────────────
# Step executors
# ──────────────────────────────────────────────────────────────────────────

def _execute_run_step(
    session: Any,
    character: str,
    goal_id: str,
    mode: str,
    *,
    verdict_model: str,
    verdict_api_key: Optional[str],
    poll_interval: float,
    run_timeout: float,
    log_path: Path,
) -> Dict[str, Any]:
    """Submit a goal run, wait for completion, build harvester row, call
    verdict grader. Returns a step dict:

        {
          "kind": "run",
          "mode": "...",
          "started_at": "...",
          "elapsed_ms": ...,
          "row": <harvester row>,
          "verdict": <verdict sub-object or None>,
          "error": str or None,
        }

    Never raises — all errors are captured as the step's `error` field
    so the trial can decide whether to continue.
    """
    t0 = time.monotonic()
    step: Dict[str, Any] = {
        "kind": "run",
        "mode": mode,
        "started_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "elapsed_ms": 0,
        "row": None,
        "verdict": None,
        "error": None,
    }

    # Snapshot the pre-submit goal record so wait_for_completion can
    # distinguish a stale record from a fresh completion. Same dance the
    # harvester CLI does.
    pre_record = harvester.query_goal_record(session, character, goal_id)
    if pre_record is None:
        step["error"] = f"Goal {goal_id} not found in agent state"
        step["elapsed_ms"] = int((time.monotonic() - t0) * 1000)
        return step
    pre_submit_updated = pre_record.get("updated", "")

    if not harvester.submit_goal_run(session, character, goal_id, mode):
        step["error"] = "failed to publish /goal run command"
        step["elapsed_ms"] = int((time.monotonic() - t0) * 1000)
        return step

    try:
        record = harvester.wait_for_completion(
            session=session,
            character=character,
            goal_id=goal_id,
            poll_interval=poll_interval,
            timeout=run_timeout,
            pre_submit_updated=pre_submit_updated,
        )
    except Exception as e:
        step["error"] = f"wait_for_completion raised: {type(e).__name__}: {e}"
        step["elapsed_ms"] = int((time.monotonic() - t0) * 1000)
        return step

    harvest_error = record.get("_harvest_error")
    log_lines = harvester.slice_log_for_goal(log_path, goal_id)
    try:
        row = harvester.compute_metrics(record, log_lines, goal_id, mode)
    except Exception as e:
        step["error"] = f"compute_metrics raised: {type(e).__name__}: {e}"
        step["elapsed_ms"] = int((time.monotonic() - t0) * 1000)
        return step
    step["row"] = row

    if harvest_error:
        step["error"] = f"harvester error: {harvest_error}"
        step["elapsed_ms"] = int((time.monotonic() - t0) * 1000)
        return step

    # Verdict grading. Failures here are non-fatal for the run itself —
    # the harvester row is still useful data — but they get recorded so
    # the caller knows the grade is missing.
    try:
        verdict_row = compute_verdict(
            row,
            session=session,
            character=character,
            model=verdict_model,
            api_key=verdict_api_key,
        )
        step["verdict"] = verdict_row.get("opus_verdict")
    except Exception as e:
        step["verdict"] = None
        # Don't overwrite an earlier error, and don't mark the whole
        # step as errored just because grading failed.
        step.setdefault("verdict_error",
                        f"{type(e).__name__}: {e}")

    step["elapsed_ms"] = int((time.monotonic() - t0) * 1000)
    return step


def _execute_review_step(
    session: Any,
    character: str,
    goal_id: str,
    commit: bool,
    *,
    review_model: str,
    review_api_key: Optional[str],
) -> Dict[str, Any]:
    """Invoke review_goal with dry_run=not commit. Returns a step dict:

        {
          "kind": "review",
          "commit": bool,
          "started_at": "...",
          "elapsed_ms": ...,
          "review": <full reviewer result dict, minus plan_text>,
          "plan_text": <the rewritten plan text, kept separate for size>,
          "error": str or None,
        }
    """
    t0 = time.monotonic()
    step: Dict[str, Any] = {
        "kind": "review",
        "commit": commit,
        "started_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "elapsed_ms": 0,
        "review": None,
        "plan_text": None,
        "error": None,
    }

    try:
        result = review_goal(
            goal_id,
            character=character,
            session=session,
            model=review_model,
            api_key=review_api_key,
            dry_run=not commit,
        )
    except Exception as e:
        step["error"] = f"review_goal raised: {type(e).__name__}: {e}"
        step["elapsed_ms"] = int((time.monotonic() - t0) * 1000)
        return step

    # Split plan_text out so the main review sub-dict stays small for
    # JSON dumps. Callers who need the rewritten plan can still get it.
    plan_text = result.pop("plan_text", None)
    step["review"] = result
    step["plan_text"] = plan_text

    if result.get("error"):
        step["error"] = result["error"]
    elif commit and not result.get("committed"):
        step["error"] = "review completed but commit did not succeed"
    elif not commit and not result.get("llm_called"):
        step["error"] = "dry-run review did not complete the LLM call"

    step["elapsed_ms"] = int((time.monotonic() - t0) * 1000)
    return step


# ──────────────────────────────────────────────────────────────────────────
# Main entry: run_trial
# ──────────────────────────────────────────────────────────────────────────

def run_trial(
    goal_id: str,
    steps: List[Dict[str, Any]],
    *,
    trial_context: Any = None,
    character: str = DEFAULT_CHARACTER,
    session: Any = None,
    # Verdict config
    verdict_model: str = VERDICT_DEFAULT_MODEL,
    verdict_api_key: Optional[str] = None,
    # Reviewer config
    review_model: str = REVIEWER_DEFAULT_MODEL,
    review_api_key: Optional[str] = None,
    # Harvester config
    poll_interval: float = DEFAULT_POLL_INTERVAL_S,
    run_timeout: float = DEFAULT_RUN_TIMEOUT_S,
    log_path: Optional[Path] = None,
) -> Dict[str, Any]:
    """Execute a trial: run each step in sequence, stop on first error,
    return the full trial record.

    Args:
        goal_id: The goal to exercise (must already exist in the agent).
        steps: Ordered list of step dicts. See module docstring for the
            step vocabulary. Validated before execution — a malformed
            list is an immediate error with no steps executed.
        trial_context: Opaque caller-supplied metadata (any JSON-ish
            value) embedded verbatim in the trial record under the
            `trial_context` key. trial.py never inspects it.
        character: Character name (for Zenoh topics).
        session: Optional pre-opened Zenoh session. If None, opens one
            and closes it before returning.
        verdict_model / verdict_api_key: Config for the independent
            quality grader in the run step.
        review_model / review_api_key: Config for the automated
            reviewer in the review step.
        poll_interval / run_timeout: Harvester polling config.
        log_path: Override the executive_node log location (defaults to
            harvester.DEFAULT_LOG_PATH).

    Returns a dict of shape:

        {
          "trial_context": <whatever caller passed, or None>,
          "goal_id": "goal_15",
          "character": "Jill",
          "started_at": "...",
          "elapsed_ms": ...,
          "steps": [<step dict>, ...],
          "error": None or <first error encountered>,
        }

    On validation error, `steps` is empty and `error` carries the
    validation message.
    """
    started_at = datetime.now(timezone.utc).isoformat(timespec="seconds")
    t0 = time.monotonic()
    trial: Dict[str, Any] = {
        "trial_context": trial_context,
        "goal_id": goal_id,
        "character": character,
        "started_at": started_at,
        "elapsed_ms": 0,
        "steps": [],
        "error": None,
    }

    validation_error = _validate_steps(steps)
    if validation_error:
        trial["error"] = f"step validation failed: {validation_error}"
        trial["elapsed_ms"] = int((time.monotonic() - t0) * 1000)
        return trial

    resolved_log_path = log_path or harvester.DEFAULT_LOG_PATH

    own_session = False
    if session is None:
        session = _open_zenoh_session()
        own_session = True
        if session is None:
            trial["error"] = (
                "could not open Zenoh session — is the launcher running?"
            )
            trial["elapsed_ms"] = int((time.monotonic() - t0) * 1000)
            return trial

    try:
        for step_spec in steps:
            kind = step_spec["kind"]
            if kind == "run":
                step_result = _execute_run_step(
                    session=session,
                    character=character,
                    goal_id=goal_id,
                    mode=step_spec["mode"],
                    verdict_model=verdict_model,
                    verdict_api_key=verdict_api_key,
                    poll_interval=poll_interval,
                    run_timeout=run_timeout,
                    log_path=resolved_log_path,
                )
            elif kind == "review":
                step_result = _execute_review_step(
                    session=session,
                    character=character,
                    goal_id=goal_id,
                    commit=step_spec["commit"],
                    review_model=review_model,
                    review_api_key=review_api_key,
                )
            else:
                # _validate_steps should have caught this.
                step_result = {
                    "kind": kind,
                    "error": f"internal: unknown step kind {kind!r}",
                }

            trial["steps"].append(step_result)

            if step_result.get("error"):
                trial["error"] = (
                    f"step {len(trial['steps']) - 1} ({kind}): "
                    f"{step_result['error']}"
                )
                break
    finally:
        if own_session:
            try:
                session.close()
            except Exception:
                pass
        trial["elapsed_ms"] = int((time.monotonic() - t0) * 1000)

    return trial


# ──────────────────────────────────────────────────────────────────────────
# CLI
# ──────────────────────────────────────────────────────────────────────────

def _parse_json_arg(raw: str, field_name: str) -> Any:
    try:
        return json.loads(raw)
    except json.JSONDecodeError as e:
        raise SystemExit(f"ERROR: --{field_name} is not valid JSON: {e}")


def main(argv: Optional[List[str]] = None) -> int:
    p = argparse.ArgumentParser(
        description="Execute a benchmark trial (a sequence of run/review "
        "steps) against a running agent. See module docstring for the "
        "step vocabulary."
    )
    sub = p.add_subparsers(dest="cmd", required=True)

    sp = sub.add_parser("run", help="Run a single trial from a step list")
    sp.add_argument("goal_id", help="Goal ID to exercise")
    sp.add_argument("--steps", required=True,
                    help="JSON array of step dicts (see module docstring)")
    sp.add_argument("--context", default=None,
                    help="Opaque JSON value to embed as trial_context")
    sp.add_argument("--character", default=DEFAULT_CHARACTER)
    sp.add_argument("--verdict-model", default=VERDICT_DEFAULT_MODEL)
    sp.add_argument("--review-model", default=REVIEWER_DEFAULT_MODEL)
    sp.add_argument("--api-key", default=None,
                    help="Anthropic API key for both verdict and reviewer "
                         "(or set CLAUDE_API_KEY)")
    sp.add_argument("--poll-interval", type=float, default=DEFAULT_POLL_INTERVAL_S)
    sp.add_argument("--run-timeout", type=float, default=DEFAULT_RUN_TIMEOUT_S)
    sp.add_argument("--log-path", type=Path, default=None,
                    help="Override executive_node log path")
    sp.add_argument("--output", type=Path, default=None,
                    help="Write the full trial record JSON to this file")

    args = p.parse_args(argv)

    if args.cmd == "run":
        steps = _parse_json_arg(args.steps, "steps")
        context = _parse_json_arg(args.context, "context") if args.context else None

        trial = run_trial(
            args.goal_id,
            steps=steps,
            trial_context=context,
            character=args.character,
            verdict_model=args.verdict_model,
            verdict_api_key=args.api_key,
            review_model=args.review_model,
            review_api_key=args.api_key,
            poll_interval=args.poll_interval,
            run_timeout=args.run_timeout,
            log_path=args.log_path,
        )

        # Strip large fields from the stdout summary. The full record
        # (including plan_text and per-step rows) goes to --output.
        summary = _summarize_for_stdout(trial)
        print(json.dumps(summary, indent=2))

        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            with open(args.output, "w", encoding="utf-8") as f:
                json.dump(trial, f, indent=2, default=str)
            print(f"(full trial record written to {args.output})", file=sys.stderr)

        return 0 if not trial.get("error") else 1

    return 2


def _summarize_for_stdout(trial: Dict[str, Any]) -> Dict[str, Any]:
    """Build a compact version of the trial record for stdout display.
    Trims the verbose fields (full harvester rows, plan_text) but keeps
    enough to see at a glance whether the trial succeeded.
    """
    compact = {
        "trial_context": trial.get("trial_context"),
        "goal_id": trial.get("goal_id"),
        "character": trial.get("character"),
        "started_at": trial.get("started_at"),
        "elapsed_ms": trial.get("elapsed_ms"),
        "error": trial.get("error"),
        "steps": [],
    }
    for step in trial.get("steps", []):
        s = {
            "kind": step.get("kind"),
            "elapsed_ms": step.get("elapsed_ms"),
            "error": step.get("error"),
        }
        if step.get("kind") == "run":
            s["mode"] = step.get("mode")
            row = step.get("row") or {}
            s["status"] = row.get("status")
            s["quality_status"] = row.get("quality_status")
            s["planner_tool_calls"] = row.get("planner_tool_call_count")
            s["cached_plan_step_count"] = row.get("cached_plan_step_count")
            s["duration_ms"] = row.get("duration_ms")
            verdict = step.get("verdict") or {}
            s["verdict"] = verdict.get("verdict") if isinstance(verdict, dict) else None
            s["score_percent"] = verdict.get("score_percent") if isinstance(verdict, dict) else None
        elif step.get("kind") == "review":
            s["commit"] = step.get("commit")
            review = step.get("review") or {}
            s["committed"] = review.get("committed")
            s["learnings_count"] = review.get("learnings_count")
            s["input_tokens"] = review.get("input_tokens")
            s["output_tokens"] = review.get("output_tokens")
        compact["steps"].append(s)
    return compact


if __name__ == "__main__":
    sys.exit(main())
