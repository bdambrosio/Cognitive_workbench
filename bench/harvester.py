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
    python bench/harvester.py --goal goal_15 --mode replay --out runs.jsonl
    python bench/harvester.py --goal goal_15 --no-submit  # harvest only
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

def wait_for_completion(
    session: zenoh.Session,
    character: str,
    goal_id: str,
    poll_interval: float,
    timeout: float,
    pre_submit_updated: Optional[str],
) -> Dict[str, Any]:
    """Poll until the goal record's status leaves 'running' AND its updated
    timestamp is newer than the pre-submit value (so we don't return the
    pre-run state and call it done).

    Returns the most recent goal record. If the timeout fires before
    completion, the returned record carries `_harvest_error="timeout"`.
    """
    deadline = time.monotonic() + timeout
    saw_running = False
    last_record: Optional[Dict[str, Any]] = None
    while time.monotonic() < deadline:
        record = query_goal_record(session, character, goal_id)
        if record is not None:
            last_record = record
            status = record.get("status", "")
            updated = record.get("updated", "")
            if status == "running":
                saw_running = True
            elif status in TERMINAL_STATUSES:
                # Two acceptance conditions: (a) we observed running at
                # some earlier poll (so we know this is post-run state),
                # OR (b) the updated timestamp moved past the pre-submit
                # value (handles fast runs that finished before our first
                # poll caught them in 'running' state).
                if saw_running:
                    return record
                if pre_submit_updated and updated and updated > pre_submit_updated:
                    return record
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

# Matches `Executing action: {"type": "<tool>"...}` lines emitted by
# infospace_executor when it dispatches a tool call.
_EXEC_ACTION_RE = re.compile(r'Executing action:\s*\{"type":\s*"([\w-]+)"')


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
    """Count tool calls and per-type histogram from the log slice.

    Tool failures are inferred from `Code block at step N failed` lines and
    `Tool '<name>' executed` followed by status=failed (rare; the executor
    usually wraps failure in a returned dict, not a log line).
    """
    total = 0
    by_type: Dict[str, int] = {}
    code_block_failures = 0
    for line in log_lines:
        m = _EXEC_ACTION_RE.search(line)
        if m:
            total += 1
            t = m.group(1)
            by_type[t] = by_type.get(t, 0) + 1
        if "Code block at step" in line and "failed" in line.lower():
            code_block_failures += 1
    return {
        "tool_call_count": total,
        "tool_call_types": by_type,
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
        # Tool calls (from log slice)
        "tool_call_count": counts["tool_call_count"],
        "tool_call_types": counts["tool_call_types"],
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
# CLI
# ──────────────────────────────────────────────────────────────────────────

def main(argv: Optional[List[str]] = None) -> int:
    p = argparse.ArgumentParser(
        description="Stage 1 benchmark harvester for Cognitive Workbench",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--goal", required=True, help="Goal ID to run (e.g. goal_15)")
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
    args = p.parse_args(argv)

    session = open_zenoh()
    try:
        # Snapshot pre-submit state so wait_for_completion can detect a
        # finished run via the updated timestamp even if we miss 'running'.
        pre_record = query_goal_record(session, args.character, args.goal)
        if pre_record is None:
            print(
                f"ERROR: goal '{args.goal}' not found via "
                f"cognitive/{args.character}/scheduled_goals — "
                f"is the launcher running for character {args.character}?",
                file=sys.stderr,
            )
            return 2
        pre_submit_updated = pre_record.get("updated", "")

        if not args.no_submit:
            print(
                f"Submitting {args.goal} (mode={args.mode or 'default'}) "
                f"to {args.character}...",
                file=sys.stderr,
            )
            if not submit_goal_run(session, args.character, args.goal, args.mode):
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
                args.goal,
                args.poll_interval,
                args.timeout,
                pre_submit_updated,
            )

        log_lines = slice_log_for_goal(args.log_path, args.goal)
        print(
            f"Harvested goal record (status={record.get('status', '?')}) "
            f"and {len(log_lines)}-line log slice",
            file=sys.stderr,
        )

        row = compute_metrics(record, log_lines, args.goal, args.mode)
        row_text = json.dumps(row, ensure_ascii=False)

        if args.out:
            args.out.parent.mkdir(parents=True, exist_ok=True)
            with open(args.out, "a", encoding="utf-8") as f:
                f.write(row_text + "\n")
            print(f"Appended row to {args.out}", file=sys.stderr)
        else:
            print(row_text)

        return 0 if not row.get("_harvest_error") else 4
    finally:
        try:
            session.close()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
