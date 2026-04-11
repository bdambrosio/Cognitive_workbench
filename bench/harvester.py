#!/usr/bin/env python3
"""Stage 1 benchmark harvester for the Cognitive Workbench.

Submits a single goal by ID to a running launcher via Zenoh, polls the
scheduled_goals queryable until the run terminates, then harvests metrics
from the goal record + the bracketed slice of executive_node.log and
writes a single JSONL row.

Stage 1 scope (deliberately minimal):
  - One goal per invocation. Multi-goal batching is stage 4.
  - No Opus quality verdict. That's stage 2.
  - No snapshot/restore between runs. That's stage 3.
  - No A/B comparison. That's stage 3.
  - No randomization. Stage 4.

Requires a running launcher (the agent thread for the named character must
be alive on the same Zenoh session).

Usage:
    python bench/harvester.py --goal goal_15
    python bench/harvester.py --goal G01            # resolved by name
    python bench/harvester.py --goal goal_15 --mode replay --out runs.jsonl
    python bench/harvester.py --goal goal_15 --no-submit  # harvest only

The --goal argument accepts either the internal goal_id (goal_15) or the
display name (G01). Name resolution is handled by resolve_goal_handle();
goal_id matches take precedence, ambiguous name matches error out.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

# The launcher's Zenoh helpers live in src/. Add it to sys.path so we can
# reuse make_localhost_config() instead of duplicating it.
_REPO_ROOT = Path(__file__).resolve().parent.parent
_SRC_DIR = _REPO_ROOT / "src"
sys.path.insert(0, str(_SRC_DIR))

import zenoh  # noqa: E402
from utils.zenoh_utils import make_localhost_config  # noqa: E402

DEFAULT_CHARACTER = "Jill"
DEFAULT_POLL_INTERVAL_S = 3.0
DEFAULT_TIMEOUT_S = 30 * 60  # 30 minutes
DEFAULT_LOG_PATH = _SRC_DIR / "logs" / "executive_node.log"

TERMINAL_STATUSES = {"completed", "failed", "ready"}


# ──────────────────────────────────────────────────────────────────────────
# Zenoh I/O
# ──────────────────────────────────────────────────────────────────────────

def open_zenoh() -> zenoh.Session:
    """Open a Zenoh session against the localhost router (same as launcher)."""
    return zenoh.open(make_localhost_config())


def query_goal_record(
    session: zenoh.Session, character: str, goal_id: str, timeout: float = 3.0
) -> Optional[Dict[str, Any]]:
    """Query the agent's scheduled_goals queryable and return the matching
    goal dict, or None if the goal isn't present or the queryable is silent.

    Strict goal_id lookup. Does NOT match on display name — use
    resolve_goal_handle for that. This function stays goal_id-only so hot
    polling paths (wait_for_completion) never pay the name-match cost.

    The queryable returns a payload of shape:
        {"success": True, "goals": [ {goal_id: ..., ...}, ... ]}
    """
    key = f"cognitive/{character}/scheduled_goals"
    try:
        for reply in session.get(key, timeout=timeout):
            if not (hasattr(reply, "ok") and reply.ok is not None):
                continue
            try:
                payload = json.loads(reply.ok.payload.to_bytes().decode("utf-8"))
            except Exception:
                continue
            for g in payload.get("goals", []) or []:
                if g.get("goal_id") == goal_id:
                    return g
    except Exception as e:
        print(f"WARN: scheduled_goals query failed: {e}", file=sys.stderr)
    return None


def resolve_goal_handle(
    session: zenoh.Session,
    character: str,
    goal_ref: str,
    timeout: float = 3.0,
) -> Optional[Dict[str, Any]]:
    """Resolve a goal reference to a full goal record.

    `goal_ref` may be the internal goal_id (`goal_17`) or the display
    name (`G01`). goal_id matches take precedence so an unambiguous ID
    always wins — this matters because a goal named "goal_99" that
    doesn't actually have goal_id goal_99 should still resolve.

    Returns the full matched goal record (so callers can also use it as
    the pre-submit snapshot without a second query), or None if nothing
    matched. Raises ValueError if the name matches more than one goal,
    so callers can surface the ambiguity instead of silently running
    against the wrong one.
    """
    key = f"cognitive/{character}/scheduled_goals"
    try:
        for reply in session.get(key, timeout=timeout):
            if not (hasattr(reply, "ok") and reply.ok is not None):
                continue
            try:
                payload = json.loads(reply.ok.payload.to_bytes().decode("utf-8"))
            except Exception:
                continue
            goals = payload.get("goals", []) or []
            for g in goals:
                if g.get("goal_id") == goal_ref:
                    return g
            name_matches = [g for g in goals if g.get("name") == goal_ref]
            if len(name_matches) == 1:
                return name_matches[0]
            if len(name_matches) > 1:
                candidate_ids = [g.get("goal_id", "?") for g in name_matches]
                raise ValueError(
                    f"Goal name '{goal_ref}' is ambiguous — matches "
                    f"{len(name_matches)} goals: {candidate_ids}. "
                    f"Pass an explicit goal_id."
                )
    except ValueError:
        raise
    except Exception as e:
        print(f"WARN: scheduled_goals query failed: {e}", file=sys.stderr)
    return None


def submit_goal_run(
    session: zenoh.Session, character: str, goal_id: str, mode: Optional[str] = None
) -> bool:
    """Send a /goal run command to the agent's command channel."""
    key = f"cognitive/{character}/command"
    payload: Dict[str, Any] = {"cmd": "/goal run", "goal_id": goal_id}
    if mode:
        payload["execution_mode"] = mode
    try:
        session.put(key, json.dumps(payload).encode("utf-8"))
        return True
    except Exception as e:
        print(f"ERROR: command put failed: {e}", file=sys.stderr)
        return False


# ──────────────────────────────────────────────────────────────────────────
# Polling for completion
# ──────────────────────────────────────────────────────────────────────────

NO_PROGRESS_TIMEOUT_S = 100.0


def wait_for_completion(
    session: zenoh.Session,
    character: str,
    goal_id: str,
    poll_interval: float,
    timeout: float,
    pre_submit_updated: Optional[str],
) -> Dict[str, Any]:
    """Poll until the goal record reflects a real completion.

    Two watchdogs run in parallel with the outer `timeout`:

    1. **outer timeout** (`timeout`, default 30 min): absolute ceiling
       on a single goal's wall time. Long by design so legit slow runs
       aren't cut off.
    2. **no-progress watchdog** (NO_PROGRESS_TIMEOUT_S = 100s): if the
       goal record never shows forward progress — `status` leaving
       "ready" OR `last_run_mode` getting populated — we declare the
       run non-started and return with `_harvest_error =
       "no_progress_non_started"`. Protects against the case where the
       launcher's scheduled_goals queryable responds (so `wait_ready`
       succeeds) but the agent can't actually execute goals (e.g.,
       lingering sglang workers from an incomplete prior teardown
       holding GPU resources). Without this, the outer 30-min timeout
       would be wasted on runs that were never going to happen.

    Acceptance criteria for "completed":
    - `status ∈ {"completed", "failed"}` — return immediately.
    - `status` terminal AND we observed `status == "running"` earlier
      — return (legit post-run state, including "ready" from a
      user-interrupted run).
    - `status` terminal AND `last_run_mode != ""` AND
      `updated > pre_submit_updated` — fast-completion escape hatch,
      for runs that finished before our first poll caught them in
      "running". The `last_run_mode` requirement is the fix for a
      harvester bug that let `_cmd_goal_run`'s execution_mode-touch
      trip this path on non-started runs. Only `_set_scheduled_goal_result`
      sets `last_run_mode`, so its presence is the ground truth that
      the goal thread actually ran to a result.

    Returns the most recent goal record. On timeout (outer), carries
    `_harvest_error="timeout"`. On no-progress, carries
    `_harvest_error="no_progress_non_started"`.
    """
    start_time = time.monotonic()
    deadline = start_time + timeout
    no_progress_deadline = start_time + NO_PROGRESS_TIMEOUT_S
    saw_running = False
    saw_progress = False
    last_record: Optional[Dict[str, Any]] = None

    while time.monotonic() < deadline:
        record = query_goal_record(session, character, goal_id)
        if record is not None:
            last_record = record
            status = record.get("status", "")
            updated = record.get("updated", "")
            last_run_mode = record.get("last_run_mode", "")

            # Progress tracking: either the status has left "ready" or
            # the run-completion path has stamped last_run_mode. Either
            # is definitive evidence that /goal run was picked up.
            if status != "ready" or last_run_mode:
                saw_progress = True

            if status == "running":
                saw_running = True
            elif status in TERMINAL_STATUSES:
                if saw_running:
                    return record
                # Fast-completion escape hatch, gated on last_run_mode
                # so a non-started run (record touched only by
                # _cmd_goal_run's execution_mode update) can't trip it.
                if (
                    pre_submit_updated
                    and updated
                    and updated > pre_submit_updated
                    and last_run_mode
                ):
                    return record

        # No-progress watchdog: if nothing has happened within the
        # grace window, the launcher is up but can't run goals. Return
        # error so the batch runner can move on instead of burning the
        # outer timeout.
        if not saw_progress and time.monotonic() > no_progress_deadline:
            print(
                f"WARN: goal '{goal_id}' showed no progress after "
                f"{NO_PROGRESS_TIMEOUT_S}s — treating as non-started",
                file=sys.stderr,
            )
            if last_record is None:
                return {
                    "_harvest_error": "no_progress_non_started",
                    "goal_id": goal_id,
                }
            out = dict(last_record)
            out["_harvest_error"] = "no_progress_non_started"
            return out

        time.sleep(poll_interval)

    print(f"WARN: timeout after {timeout}s waiting for goal completion", file=sys.stderr)
    if last_record is None:
        return {"_harvest_error": "no_record_found", "goal_id": goal_id}
    last_record = dict(last_record)
    last_record["_harvest_error"] = "timeout"
    return last_record


# ──────────────────────────────────────────────────────────────────────────
# Log slice + parse
# ──────────────────────────────────────────────────────────────────────────

def slice_log_for_goal(log_path: Path, goal_id: str) -> List[str]:
    """Extract log lines bracketed by `[GOAL goal_X mode=... START]` and
    the matching `[GOAL goal_X mode=... END ...]` line. Returns the LAST
    such bracket pair so the harness sees the most recent run.

    If only START is present (END never fired — crash, abort, etc.) we
    return everything from START to end-of-file.
    """
    if not log_path.exists():
        return []
    try:
        all_lines = log_path.read_text(encoding="utf-8", errors="replace").splitlines()
    except Exception as e:
        print(f"WARN: log read failed: {e}", file=sys.stderr)
        return []

    marker = f"[GOAL {goal_id} mode="
    start_idx: Optional[int] = None
    end_idx: Optional[int] = None
    for i, line in enumerate(all_lines):
        if marker not in line:
            continue
        if "START]" in line:
            start_idx = i
            end_idx = None
        elif "END " in line and start_idx is not None:
            end_idx = i
    if start_idx is None:
        return []
    if end_idx is None:
        end_idx = len(all_lines) - 1
    return all_lines[start_idx : end_idx + 1]


# Pre-compiled regexes for parsing the log slice.
_END_BRACKET_RE = re.compile(
    r"\[GOAL\s+(?P<gid>\S+)\s+mode=(?P<mode>\w+)\s+END\s+"
    r"status=(?P<status>\w+)\s+steps=(?P<steps>-?\d+)\s+"
    r"duration_ms=(?P<dur>-?\d+)\s+quality=(?P<quality>\S+?)\]"
)

# Matches `Executing action: {"type": "<tool>", ...}` lines emitted by
# infospace_executor when it dispatches a tool call. Captures both the
# tool type and the optional `target` field so the harvester can classify
# infrastructure (executor housekeeping like loading _ooda_state) vs
# planner-issued calls.
_EXEC_ACTION_RE = re.compile(
    r'Executing action:\s*\{"type":\s*"([\w-]+)"'
    r'(?:[^}]*?"target":\s*"([^"]*)")?'
)

# Tool types that are always infrastructure regardless of target. `init`
# is the world-init pre-step the executor runs before each goal.
_INFRA_TOOL_TYPES = {"init"}


def parse_end_bracket(log_lines: List[str]) -> Dict[str, Any]:
    """Find the last END bracket in the slice and parse its fields."""
    for line in reversed(log_lines):
        m = _END_BRACKET_RE.search(line)
        if m:
            quality = m.group("quality")
            return {
                "bracket_mode": m.group("mode"),
                "bracket_status": m.group("status"),
                "bracket_steps": int(m.group("steps")),
                "bracket_duration_ms": int(m.group("dur")),
                "bracket_quality": None if quality == "none" else quality,
            }
    return {}


def count_tool_calls(log_lines: List[str]) -> Dict[str, Any]:
    """Count tool calls and per-type histogram from the log slice, splitting
    infrastructure calls (executor housekeeping like init and loads of
    `_*` system notes) from planner-issued calls.

    Classification heuristic:
      - `init` → always infrastructure
      - `load` with target starting with `_` → infrastructure
        (loads of _ooda_state, _scheduled_goals, _situation, etc.)
      - any other call → planner-issued

    The heuristic is conservative — sensor-written named Notes also start
    with `_` (e.g. `_rss_pending_titles`), so a `load` of one of those is
    technically planner-issued but will be counted as infrastructure here.
    Acceptable for stage 1; the cross-validation harness can re-classify
    from the cached_plan_actions source if it needs precision.

    Tool failures are inferred from `Code block at step N failed` lines.
    The executor usually wraps tool failures in a returned dict rather
    than logging them as failures, so this count is a lower bound.
    """
    total = 0
    infra = 0
    by_type: Dict[str, int] = {}
    by_type_infra: Dict[str, int] = {}
    by_type_planner: Dict[str, int] = {}
    code_block_failures = 0

    for line in log_lines:
        m = _EXEC_ACTION_RE.search(line)
        if m:
            total += 1
            t = m.group(1)
            target = m.group(2) or ""
            by_type[t] = by_type.get(t, 0) + 1
            is_infra = (t in _INFRA_TOOL_TYPES) or (
                t == "load" and target.startswith("_")
            )
            if is_infra:
                infra += 1
                by_type_infra[t] = by_type_infra.get(t, 0) + 1
            else:
                by_type_planner[t] = by_type_planner.get(t, 0) + 1
        if "Code block at step" in line and "failed" in line.lower():
            code_block_failures += 1

    return {
        "tool_call_count": total,
        "tool_call_types": by_type,
        "infrastructure_call_count": infra,
        "infrastructure_call_types": by_type_infra,
        "planner_tool_call_count": total - infra,
        "planner_tool_call_types": by_type_planner,
        "code_block_failures": code_block_failures,
    }


# ──────────────────────────────────────────────────────────────────────────
# Metric row
# ──────────────────────────────────────────────────────────────────────────

def compute_metrics(
    record: Dict[str, Any],
    log_lines: List[str],
    goal_id: str,
    submitted_mode: Optional[str],
) -> Dict[str, Any]:
    """Build the JSONL row for this run from the goal record + log slice."""
    end = parse_end_bracket(log_lines)
    counts = count_tool_calls(log_lines)

    step_results = record.get("step_results", []) or []
    cached_plan = record.get("cached_plan_actions", []) or []
    vc_meta = record.get("vision_criteria_meta", {}) or {}

    # Step-level outcomes (replay only — planning runs leave step_results empty)
    step_failures = sum(1 for sr in step_results if isinstance(sr, dict) and sr.get("status") == "failed")
    step_exceptions = sum(1 for sr in step_results if isinstance(sr, dict) and sr.get("exception_type"))
    step_durations_ms = [
        sr.get("duration_ms", 0)
        for sr in step_results
        if isinstance(sr, dict) and isinstance(sr.get("duration_ms"), (int, float))
    ]

    return {
        "harvest_ts": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "harvester_version": 1,
        # Goal identity
        "goal_id": goal_id,
        "goal_name": record.get("name", ""),
        "goal_text": record.get("goal_text", ""),
        # Run identity
        "submitted_mode": submitted_mode,
        "last_run_mode": record.get("last_run_mode", ""),
        "execution_mode_setting": record.get("execution_mode", ""),
        # Outcome
        "status": record.get("status", ""),
        "quality_status": record.get("quality_status", ""),
        "last_result": (record.get("last_result", "") or "")[:1000],
        "primary_product": record.get("primary_product", ""),
        # Plan structure
        "cached_plan_step_count": len(cached_plan),
        # Per-step instrumentation (replay only at the moment)
        "step_results_count": len(step_results),
        "step_failures": step_failures,
        "step_exceptions": step_exceptions,
        "step_durations_ms_total": sum(step_durations_ms),
        # Tool calls (from log slice). The total includes both planner-
        # issued calls and executor housekeeping; the planner_* fields
        # isolate what the planner actually decided to call.
        "tool_call_count": counts["tool_call_count"],
        "tool_call_types": counts["tool_call_types"],
        "planner_tool_call_count": counts["planner_tool_call_count"],
        "planner_tool_call_types": counts["planner_tool_call_types"],
        "infrastructure_call_count": counts["infrastructure_call_count"],
        "infrastructure_call_types": counts["infrastructure_call_types"],
        "code_block_failures": counts["code_block_failures"],
        # Vision criteria provenance
        "vision_criteria": record.get("vision_criteria", ""),
        "vision_criteria_source": record.get("vision_criteria_source", ""),
        "vision_criteria_version": vc_meta.get("generator_version"),
        "vision_criteria_model": vc_meta.get("model", ""),
        "vision_criteria_generated_at": vc_meta.get("generated_at", ""),
        "vision_criteria_text_hash": vc_meta.get("goal_text_hash", ""),
        "last_quality_eval": (record.get("last_quality_eval", "") or "")[:2000],
        # Bracket-derived (sanity check + duration_ms)
        **end,
        # Goal record timestamps (for elapsed wall-time)
        "created": record.get("created", ""),
        "updated": record.get("updated", ""),
        # Harvest meta
        "log_slice_lines": len(log_lines),
        "_harvest_error": record.get("_harvest_error"),
    }


# ──────────────────────────────────────────────────────────────────────────
# Human-readable summary
# ──────────────────────────────────────────────────────────────────────────

def _truncate(text: str, n: int = 240) -> str:
    """Truncate text for the summary block, replacing newlines with spaces."""
    if not text:
        return ""
    text = " ".join(text.split())
    if len(text) <= n:
        return text
    return text[: n - 1] + "…"


def _format_duration(ms: int) -> str:
    if ms is None or ms < 0:
        return "?"
    if ms < 1000:
        return f"{ms}ms"
    if ms < 60_000:
        return f"{ms / 1000:.1f}s"
    return f"{ms // 60_000}m{(ms % 60_000) // 1000:02d}s"


def format_summary(row: Dict[str, Any]) -> str:
    """Render a multi-line human-readable summary of the harvested row.

    Goes to stderr alongside (not instead of) the JSON row on stdout, so
    a human watching the harness output can see what just happened
    without parsing JSON. Suppress with --no-summary if scripting.
    """
    lines = []
    lines.append("─" * 70)
    lines.append(f"Goal:        {row.get('goal_id', '?')} — {row.get('goal_name', '')}")
    text = _truncate(row.get("goal_text", ""), 200)
    if text:
        lines.append(f"Goal text:   {text}")
    lines.append("")

    # Run identity
    submitted = row.get("submitted_mode") or "(default)"
    actual = row.get("last_run_mode") or "?"
    bracket = row.get("bracket_mode") or "?"
    mode_str = f"submitted={submitted}  actual={actual}  bracket={bracket}"
    lines.append(f"Run mode:    {mode_str}")
    lines.append(
        f"Outcome:     status={row.get('status', '?')}  "
        f"quality={row.get('quality_status', '?')}  "
        f"duration={_format_duration(row.get('bracket_duration_ms', -1))}"
    )
    err = row.get("_harvest_error")
    if err:
        lines.append(f"Harvest:     ⚠ {err}")
    lines.append("")

    # Plan structure + tool counts
    lines.append(
        f"Plan:        {row.get('cached_plan_step_count', 0)} cached step(s)  "
        f"({row.get('step_results_count', 0)} step_result entries)"
    )
    lines.append(
        f"Tool calls:  {row.get('tool_call_count', 0)} total = "
        f"{row.get('planner_tool_call_count', 0)} planner + "
        f"{row.get('infrastructure_call_count', 0)} infrastructure"
    )
    planner_types = row.get("planner_tool_call_types", {}) or {}
    if planner_types:
        type_summary = ", ".join(f"{k}:{v}" for k, v in sorted(planner_types.items()))
        lines.append(f"  planner:   {type_summary}")
    infra_types = row.get("infrastructure_call_types", {}) or {}
    if infra_types:
        type_summary = ", ".join(f"{k}:{v}" for k, v in sorted(infra_types.items()))
        lines.append(f"  infra:     {type_summary}")
    cb_fails = row.get("code_block_failures", 0)
    step_fails = row.get("step_failures", 0)
    step_excs = row.get("step_exceptions", 0)
    if cb_fails or step_fails or step_excs:
        lines.append(
            f"Failures:    code_block={cb_fails}  "
            f"step_failed={step_fails}  step_exception={step_excs}"
        )
    lines.append("")

    # Vision criteria provenance
    vc_version = row.get("vision_criteria_version")
    vc_source = row.get("vision_criteria_source") or "(none)"
    vc_model = row.get("vision_criteria_model") or "(none)"
    if vc_version is not None:
        lines.append(
            f"Vision:      v{vc_version}  source={vc_source}  model={vc_model}"
        )
        vc_text = (row.get("vision_criteria") or "").strip()
        if vc_text:
            for vline in vc_text.splitlines()[:6]:
                lines.append(f"  {vline}")
            extra = len(vc_text.splitlines()) - 6
            if extra > 0:
                lines.append(f"  …({extra} more lines)")
        eval_text = (row.get("last_quality_eval") or "").strip()
        if eval_text:
            lines.append("Quality eval:")
            for eline in eval_text.splitlines()[:6]:
                lines.append(f"  {eline}")
    else:
        lines.append("Vision:      (no criteria recorded)")
    lines.append("")

    # Last result excerpt
    last_result = (row.get("last_result") or "").strip()
    if last_result:
        lines.append(f"Last result: {_truncate(last_result, 320)}")
    pp = row.get("primary_product") or ""
    if pp:
        lines.append(f"Primary product: {pp}")
    lines.append("─" * 70)
    return "\n".join(lines)


# ──────────────────────────────────────────────────────────────────────────
# CLI
# ──────────────────────────────────────────────────────────────────────────

def main(argv: Optional[List[str]] = None) -> int:
    p = argparse.ArgumentParser(
        description="Stage 1 benchmark harvester for Cognitive Workbench",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument(
        "--goal",
        required=True,
        help="Goal ID or display name (e.g. goal_15 or G01). goal_id matches "
        "take precedence; ambiguous name matches error out.",
    )
    p.add_argument("--character", default=DEFAULT_CHARACTER, help="Character name")
    p.add_argument(
        "--mode",
        choices=["replan", "replay"],
        default=None,
        help="Override goal's execution_mode for this run (default: use stored mode)",
    )
    p.add_argument("--poll-interval", type=float, default=DEFAULT_POLL_INTERVAL_S)
    p.add_argument(
        "--timeout",
        type=float,
        default=DEFAULT_TIMEOUT_S,
        help="Hard ceiling in seconds before giving up on the run",
    )
    p.add_argument("--log-path", type=Path, default=DEFAULT_LOG_PATH)
    p.add_argument(
        "--out",
        type=Path,
        default=None,
        help="Append the JSONL row to this file. Default: write to stdout.",
    )
    p.add_argument(
        "--no-submit",
        action="store_true",
        help="Skip submitting the run; harvest the current goal-record state. "
        "Useful for collecting a row from a goal that already ran.",
    )
    p.add_argument(
        "--no-summary",
        action="store_true",
        help="Suppress the human-readable summary block on stderr. JSON row "
        "still goes to stdout (or --out file). Useful for scripts that want "
        "clean stderr.",
    )
    p.add_argument(
        "--opus",
        action="store_true",
        help="After harvesting, call bench/verdict.py to grade the row with "
        "an Anthropic model (default claude-sonnet-4-6). Reuses this "
        "harvester's Zenoh session for primary-product lookup. Requires "
        "CLAUDE_API_KEY in the environment. Adds an `opus_verdict` "
        "sub-object to the row.",
    )
    p.add_argument(
        "--opus-model",
        default=None,
        help="Override the default Anthropic model for --opus grading. "
        "Defaults to verdict.py's DEFAULT_MODEL (claude-sonnet-4-6).",
    )
    args = p.parse_args(argv)

    session = open_zenoh()
    try:
        # Resolve --goal (which may be a name like "G01") to a canonical
        # goal_id and the pre-submit goal record in one Zenoh round-trip.
        try:
            pre_record = resolve_goal_handle(session, args.character, args.goal)
        except ValueError as e:
            print(f"ERROR: {e}", file=sys.stderr)
            return 2
        if pre_record is None:
            print(
                f"ERROR: goal '{args.goal}' not found via "
                f"cognitive/{args.character}/scheduled_goals — "
                f"is the launcher running for character {args.character}?",
                file=sys.stderr,
            )
            return 2
        goal_id = pre_record.get("goal_id") or args.goal
        if goal_id != args.goal:
            print(
                f"Resolved goal name '{args.goal}' → {goal_id}",
                file=sys.stderr,
            )
        pre_submit_updated = pre_record.get("updated", "")

        if not args.no_submit:
            print(
                f"Submitting {goal_id} (mode={args.mode or 'default'}) "
                f"to {args.character}...",
                file=sys.stderr,
            )
            if not submit_goal_run(session, args.character, goal_id, args.mode):
                return 3
            # Give the agent a beat to update the goal record to running.
            time.sleep(1.0)

        if args.no_submit:
            record = pre_record
            print("--no-submit: harvesting current goal-record state", file=sys.stderr)
        else:
            print(
                f"Polling for completion (interval={args.poll_interval}s, "
                f"timeout={args.timeout}s)...",
                file=sys.stderr,
            )
            record = wait_for_completion(
                session,
                args.character,
                goal_id,
                args.poll_interval,
                args.timeout,
                pre_submit_updated,
            )

        log_lines = slice_log_for_goal(args.log_path, goal_id)
        print(
            f"Harvested goal record (status={record.get('status', '?')}) "
            f"and {len(log_lines)}-line log slice",
            file=sys.stderr,
        )

        row = compute_metrics(record, log_lines, goal_id, args.mode)

        # Optional stage 2: Opus quality verdict.
        if args.opus:
            try:
                from verdict import compute_verdict, format_verdict_summary, DEFAULT_MODEL
            except ImportError as e:
                print(
                    f"WARN: --opus requested but verdict.py import failed: {e}",
                    file=sys.stderr,
                )
            else:
                opus_model = args.opus_model or DEFAULT_MODEL
                print(
                    f"Calling Opus verdict (model={opus_model})...",
                    file=sys.stderr,
                )
                row = compute_verdict(
                    row,
                    session=session,
                    character=args.character,
                    model=opus_model,
                )

        row_text = json.dumps(row, ensure_ascii=False)

        if args.out:
            args.out.parent.mkdir(parents=True, exist_ok=True)
            with open(args.out, "a", encoding="utf-8") as f:
                f.write(row_text + "\n")
            print(f"Appended row to {args.out}", file=sys.stderr)
        else:
            print(row_text)

        # Human-readable summary to stderr (after the JSON row, so the
        # row is the first thing visible in scripted contexts).
        if not args.no_summary:
            print(format_summary(row), file=sys.stderr)
            if args.opus and row.get("opus_verdict"):
                from verdict import format_verdict_summary  # local import OK
                print(format_verdict_summary(row), file=sys.stderr)

        return 0 if not row.get("_harvest_error") else 4
    finally:
        try:
            session.close()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
