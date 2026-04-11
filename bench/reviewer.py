#!/usr/bin/env python3
"""Stage 3d: Automated R-Opus reviewer for the benchmark harness.

Wraps the manual `/goal plan review` → human review → `/goal plan commit`
loop into a single function that can be called from the cross-validation
trial driver. The flow:

  1. Publish `/goal plan review <goal_id>` to the agent's command channel
     so the executive_node writes the review bundle, plan file, and (in
     planning runs) trace file to the repo root.
  2. Wait for the bundle to land on disk (poll mtime > our pre-trigger
     baseline so we know we're not reading a stale bundle).
  3. Read the bundle + plan + trace + plan_review_prompt.md.
  4. Send all of that to an Anthropic model with a system prompt that
     mirrors the human reviewer's instructions and demands the rewritten
     plan as the literal Python file content with a ### LEARNINGS block
     at the end.
  5. Strip code fences from the model output, write it to
     goal_plan_<goal_id>.py.
  6. Publish `/goal plan commit <goal_id>` so the executive_node loads
     the rewritten plan, switches the goal to replay mode, and injects
     the LEARNINGS block into the world model.
  7. Wait for goal_plan_<goal_id>.py to be unlinked (the commit handler
     deletes it on success) — that's our completion signal.

Requires the launcher to be running for the target character. Reviewer
is a no-op if the agent's command channel doesn't respond.

Usage as a library:

    from reviewer import review_goal

    result = review_goal("goal_15", model="claude-opus-4-6")
    # result["committed"] is True if commit succeeded
    # result["learnings_count"] is how many LEARNINGS the model wrote
    # result["elapsed_ms"] is total wallclock

CLI:

    python bench/reviewer.py review goal_15 --model claude-sonnet-4-6
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional

# Path setup so we can import the zenoh utils.
_REPO_ROOT = Path(__file__).resolve().parent.parent
_SRC_DIR = _REPO_ROOT / "src"
sys.path.insert(0, str(_SRC_DIR))

# Reuse the helpers from verdict.py — same Anthropic client conventions,
# same </end> stop marker, same env-var lookup.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from verdict import (  # noqa: E402
    STOP_MARKER,
    _resolve_api_key,
    _open_zenoh_session,
)

DEFAULT_MODEL = "claude-sonnet-4-6"
DEFAULT_CHARACTER = "Jill"
DEFAULT_MAX_TOKENS = 8192
DEFAULT_TEMPERATURE = 0.0
# /goal plan review writes 3 files synchronously inside the command
# handler, but the handler runs on the executive's command thread and may
# be queued behind other work. 30s is generous; in practice the bundle
# lands within 2-3 seconds.
DEFAULT_BUNDLE_TIMEOUT_S = 30.0
# /goal plan commit is much faster — just a file read + JSON write +
# world_model save. 15s catches even a slow consolidation pass.
DEFAULT_COMMIT_TIMEOUT_S = 15.0


# ──────────────────────────────────────────────────────────────────────────
# File path helpers (must match executive_node._goal_plan_path et al.)
# ──────────────────────────────────────────────────────────────────────────

def _bundle_path(goal_id: str) -> Path:
    return _REPO_ROOT / f"goal_review_{goal_id}.md"


def _plan_path(goal_id: str) -> Path:
    return _REPO_ROOT / f"goal_plan_{goal_id}.py"


def _trace_path(goal_id: str) -> Path:
    return _REPO_ROOT / f"goal_trace_{goal_id}.txt"


def _prompt_guidelines_path() -> Path:
    return _REPO_ROOT / "plan_review_prompt.md"


# ──────────────────────────────────────────────────────────────────────────
# Command channel
# ──────────────────────────────────────────────────────────────────────────

def _publish_command(session: Any, character: str, cmd: str, **fields) -> None:
    """Send a structured /command to the executive_node's command channel.
    Fields are merged into the payload dict.
    """
    payload = {"cmd": cmd, "source": "bench-reviewer"}
    payload.update(fields)
    session.put(
        f"cognitive/{character}/command",
        json.dumps(payload).encode("utf-8"),
    )


def _wait_for_file_newer_than(
    path: Path, baseline_mtime: float, timeout: float, interval: float = 0.5
) -> bool:
    """Poll until path exists with mtime strictly greater than baseline,
    or until timeout elapses. Returns True if the file appeared in time.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            st = path.stat()
            if st.st_mtime > baseline_mtime and st.st_size > 0:
                return True
        except FileNotFoundError:
            pass
        time.sleep(interval)
    return False


def _wait_for_file_gone(path: Path, timeout: float, interval: float = 0.5) -> bool:
    """Poll until path no longer exists, or timeout. Returns True if gone."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not path.exists():
            return True
        time.sleep(interval)
    return False


# ──────────────────────────────────────────────────────────────────────────
# Anthropic call
# ──────────────────────────────────────────────────────────────────────────

_REVIEWER_SYSTEM_PROMPT = """You are an automated plan reviewer for the Cognitive Workbench autonomous agent.

Your job: read a plan review bundle for a single goal that has already run, decide whether the cached plan is worth keeping or needs to be rewritten, and produce a corrected plan with a LEARNINGS block at the end.

You will receive:
- The review bundle (goal_review_<id>.md): goal text, run provenance, cached plan, step outcomes, vision criteria, last quality eval, scheduler events, execution log, structural checks, available tools.
- The current cached plan file (goal_plan_<id>.py).
- The planner trace (goal_trace_<id>.txt) if a planning run produced one.
- The plan review guidelines (plan_review_prompt.md) — apply its checklist and antipatterns rigorously.

Your output is the literal contents of a new goal_plan_<id>.py file. The format must match the format of the input plan file exactly so the executive_node can parse it back. After the final step, append a `### LEARNINGS` section with `- ` bulleted items capturing actionable insights for the agent's world model and tool insights. Each learning should be a single declarative sentence the planner can use as a fact.

Rules:
- Verify every tool call in the plan against the Available Tools section in the bundle and the tool's parameter contract. Fix mismatches.
- Apply the antipatterns listed in plan_review_prompt.md.
- If the goal succeeded with a clean run, you MAY return the existing plan unchanged plus a LEARNINGS block summarising what worked. Do not invent rewrites for goals that didn't fail.
- Do NOT add steps the goal didn't ask for. Do NOT second-guess primary product binding if it already worked.
- The $eval_target binding MUST be set correctly — check vision criteria expectations.
- LEARNINGS must be specific. "Use the right tool" is useless. "When extracting references from a PDF, use extract-references with mode='structured' rather than feeding raw text to the LLM" is useful.
- Tool-specific learnings should start with the tool name followed by a colon ("tool-name: insight..."). Other learnings become world model facts.

OUTPUT FORMAT (CRITICAL):
- Respond with the literal Python file content ONLY. No prose before, no prose after.
- Do NOT wrap the output in markdown code fences (no ```python, no ```).
- Start immediately with the file's first line (typically the metadata comment block at the top of a plan file).
- End with the ### LEARNINGS block, then output \\n</end> and stop.

end your output with
</end>"""


def _call_anthropic(
    system_prompt: str,
    user_prompt: str,
    *,
    model: str,
    max_tokens: int,
    temperature: float,
    api_key: str,
) -> Dict[str, Any]:
    """Call the Anthropic Messages API and return the rewritten plan text
    plus call metadata. Mirrors verdict._call_anthropic but is local so the
    reviewer doesn't pollute verdict's namespace.
    """
    try:
        from anthropic import Anthropic  # noqa: WPS433
    except ImportError as e:
        raise ImportError(
            "The 'anthropic' Python SDK is not installed. "
            "Install it with: pip install anthropic"
        ) from e

    client = Anthropic(api_key=api_key)
    t0 = time.monotonic()
    response = client.messages.create(
        model=model,
        max_tokens=max_tokens,
        temperature=temperature,
        system=system_prompt,
        messages=[{"role": "user", "content": user_prompt}],
        stop_sequences=[STOP_MARKER],
    )
    elapsed_ms = int((time.monotonic() - t0) * 1000)

    text = ""
    for block in response.content:
        if getattr(block, "type", None) == "text":
            text += getattr(block, "text", "") or ""
    text = text.strip()
    if text.endswith(STOP_MARKER):
        text = text[: -len(STOP_MARKER)].rstrip()

    return {
        "text": text,
        "elapsed_ms": elapsed_ms,
        "stop_reason": getattr(response, "stop_reason", None),
        "input_tokens": getattr(response.usage, "input_tokens", None),
        "output_tokens": getattr(response.usage, "output_tokens", None),
    }


def _strip_code_fences(text: str) -> str:
    """Remove leading/trailing markdown code fences if the model added them.
    Handles ```python ... ``` and ``` ... ```. Idempotent.
    """
    s = (text or "").strip()
    if not s:
        return s
    lines = s.splitlines()
    # Leading fence
    if lines and lines[0].startswith("```"):
        lines = lines[1:]
    # Trailing fence
    if lines and lines[-1].strip() == "```":
        lines = lines[:-1]
    return "\n".join(lines)


def _count_learnings(plan_text: str) -> int:
    """Count the bulleted entries in the trailing ### LEARNINGS block.
    Mirrors the parsing logic in executive_node._cmd_goal_plan_commit so
    the caller knows how many world model updates to expect.
    """
    import re as _re
    match = _re.search(r'^###\s+LEARNINGS\s*$(.+)', plan_text,
                       _re.MULTILINE | _re.DOTALL)
    if not match:
        return 0
    count = 0
    for line in match.group(1).strip().splitlines():
        line = line.strip()
        if line.startswith('- '):
            count += 1
        elif line and not line.startswith('#'):
            count += 1
    return count


# ──────────────────────────────────────────────────────────────────────────
# Main entry: review_goal
# ──────────────────────────────────────────────────────────────────────────

def review_goal(
    goal_id: str,
    *,
    character: str = DEFAULT_CHARACTER,
    session: Any = None,
    model: str = DEFAULT_MODEL,
    api_key: Optional[str] = None,
    max_tokens: int = DEFAULT_MAX_TOKENS,
    temperature: float = DEFAULT_TEMPERATURE,
    bundle_timeout: float = DEFAULT_BUNDLE_TIMEOUT_S,
    commit_timeout: float = DEFAULT_COMMIT_TIMEOUT_S,
    dry_run: bool = False,
) -> Dict[str, Any]:
    """Run the full /goal plan review → R-Opus → /goal plan commit cycle.

    Args:
        goal_id: The goal to review. Must already exist in the agent.
        character: Character name (used for the command channel topic).
        session: Optional pre-opened Zenoh session. If None, opens a new
            one and closes it on exit.
        model: Anthropic model identifier.
        api_key: Explicit API key, or None to use CLAUDE_API_KEY env.
        max_tokens: Max output tokens for the rewrite.
        temperature: Sampling temperature.
        bundle_timeout: How long to wait for the review bundle file.
        commit_timeout: How long to wait for the commit handler to unlink
            the plan file.
        dry_run: If True, still trigger /goal plan review and do the LLM
            call so we get the rewritten plan text and learnings count,
            but DO NOT write the rewritten plan to disk and DO NOT
            trigger /goal plan commit. No agent state is mutated. The
            cached plan file that /goal plan review dropped at
            goal_plan_<id>.py is removed at the end so nothing leaks.
            Used by trial.py to capture observational review data on
            runs that should not be altered.

    Returns a result dict with:
        goal_id, character, model, started_at, elapsed_ms, dry_run,
        bundle_received (bool), llm_called (bool), plan_written (bool),
        committed (bool), learnings_count (int), error (str or None),
        plan_text (str, the rewritten plan)
    """
    started_at = datetime.now(timezone.utc).isoformat(timespec="seconds")
    t0 = time.monotonic()
    result: Dict[str, Any] = {
        "goal_id": goal_id,
        "character": character,
        "model": model,
        "started_at": started_at,
        "dry_run": dry_run,
        "bundle_received": False,
        "llm_called": False,
        "plan_written": False,
        "committed": False,
        "learnings_count": 0,
        "error": None,
        "plan_text": None,
        "elapsed_ms": 0,
    }

    api_key_resolved = _resolve_api_key(api_key)
    if not api_key_resolved:
        result["error"] = (
            "No Anthropic API key found. Set CLAUDE_API_KEY or pass api_key."
        )
        result["elapsed_ms"] = int((time.monotonic() - t0) * 1000)
        return result

    own_session = False
    if session is None:
        session = _open_zenoh_session()
        own_session = True
        if session is None:
            result["error"] = "Could not open Zenoh session — is the launcher running?"
            result["elapsed_ms"] = int((time.monotonic() - t0) * 1000)
            return result

    try:
        bundle_path = _bundle_path(goal_id)
        plan_path = _plan_path(goal_id)
        trace_path = _trace_path(goal_id)

        # Capture pre-trigger mtime so we know when a new bundle lands.
        # The /goal plan review handler always rewrites both files, so a
        # newer mtime is reliable signal we have fresh content (not a
        # leftover bundle from a previous run).
        baseline_mtime = 0.0
        if bundle_path.exists():
            baseline_mtime = bundle_path.stat().st_mtime

        # Step 1: trigger /goal plan review
        _publish_command(session, character, "/goal plan review", goal_id=goal_id)
        print(
            f"reviewer: triggered /goal plan review {goal_id}, "
            f"waiting for bundle...",
            file=sys.stderr,
        )

        # Step 2: wait for the bundle to appear
        if not _wait_for_file_newer_than(bundle_path, baseline_mtime, bundle_timeout):
            result["error"] = (
                f"Review bundle never appeared at {bundle_path} "
                f"(timeout {bundle_timeout}s). Is the launcher up and "
                f"the goal_id correct?"
            )
            result["elapsed_ms"] = int((time.monotonic() - t0) * 1000)
            return result
        result["bundle_received"] = True

        # Step 3: read all the inputs
        bundle_text = bundle_path.read_text(encoding="utf-8")
        plan_text_in = plan_path.read_text(encoding="utf-8") if plan_path.exists() else ""
        trace_text = ""
        if trace_path.exists():
            try:
                trace_text = trace_path.read_text(encoding="utf-8")
            except Exception:
                trace_text = ""
        guidelines_text = ""
        gp = _prompt_guidelines_path()
        if gp.exists():
            guidelines_text = gp.read_text(encoding="utf-8")

        # Cap trace size — planner traces can be huge and the bundle
        # already contains the relevant per-step instrumentation. Tail
        # is more useful than head because failures usually surface late.
        TRACE_TAIL_CHARS = 40_000
        if len(trace_text) > TRACE_TAIL_CHARS:
            trace_text = (
                f"(trace truncated to last {TRACE_TAIL_CHARS} chars)\n"
                + trace_text[-TRACE_TAIL_CHARS:]
            )

        # Step 4: build the user prompt
        user_prompt = _build_user_prompt(
            goal_id=goal_id,
            bundle_text=bundle_text,
            plan_text=plan_text_in,
            trace_text=trace_text,
            guidelines_text=guidelines_text,
        )

        # Step 5: call Anthropic
        try:
            api_result = _call_anthropic(
                _REVIEWER_SYSTEM_PROMPT,
                user_prompt,
                model=model,
                max_tokens=max_tokens,
                temperature=temperature,
                api_key=api_key_resolved,
            )
        except Exception as e:
            result["error"] = f"Anthropic call failed: {type(e).__name__}: {e}"
            result["elapsed_ms"] = int((time.monotonic() - t0) * 1000)
            return result
        result["llm_called"] = True
        result["api_elapsed_ms"] = api_result["elapsed_ms"]
        result["input_tokens"] = api_result.get("input_tokens")
        result["output_tokens"] = api_result.get("output_tokens")

        rewritten = _strip_code_fences(api_result["text"])
        if not rewritten.strip():
            result["error"] = "Model returned empty plan text after fence stripping"
            result["elapsed_ms"] = int((time.monotonic() - t0) * 1000)
            return result

        result["plan_text"] = rewritten
        result["learnings_count"] = _count_learnings(rewritten)

        if dry_run:
            # Dry-run: clean up the plan file the /goal plan review
            # handler dumped to disk so nothing leaks into the repo, and
            # skip the commit publish entirely. The caller gets the
            # rewritten plan text and learnings count via result["plan_text"]
            # / result["learnings_count"] without mutating agent state.
            try:
                if plan_path.exists():
                    plan_path.unlink()
            except Exception:
                pass
            print(
                f"reviewer: dry-run complete for {goal_id} "
                f"(no plan written, no commit triggered)",
                file=sys.stderr,
            )
        else:
            # Step 6: write rewritten plan back to disk (overwrite the
            # current cached plan that the review handler just dumped there)
            try:
                plan_path.write_text(rewritten, encoding="utf-8")
            except Exception as e:
                result["error"] = f"Failed to write rewritten plan: {e}"
                result["elapsed_ms"] = int((time.monotonic() - t0) * 1000)
                return result
            result["plan_written"] = True

            # Step 7: trigger /goal plan commit
            _publish_command(session, character, "/goal plan commit", goal_id=goal_id)
            print(
                f"reviewer: triggered /goal plan commit {goal_id}, "
                f"waiting for plan file removal...",
                file=sys.stderr,
            )

            # Step 8: wait for the plan file to be unlinked (commit handler
            # deletes it on success). If the file persists, the commit failed
            # and the user needs to inspect the launcher log.
            if not _wait_for_file_gone(plan_path, commit_timeout):
                result["error"] = (
                    f"Commit didn't unlink {plan_path} within {commit_timeout}s. "
                    f"Check launcher log; the rewritten plan may still have "
                    f"been loaded."
                )
                result["elapsed_ms"] = int((time.monotonic() - t0) * 1000)
                return result
            result["committed"] = True

    finally:
        if own_session:
            try:
                session.close()
            except Exception:
                pass
        result["elapsed_ms"] = int((time.monotonic() - t0) * 1000)

    return result


def _build_user_prompt(
    *,
    goal_id: str,
    bundle_text: str,
    plan_text: str,
    trace_text: str,
    guidelines_text: str,
) -> str:
    """Construct the single user message containing all review inputs."""
    parts = [
        f"# Plan Review Task: {goal_id}",
        "",
        "Below are the four inputs you need to review and rewrite this plan.",
        "After reading them all, output the literal contents of the new "
        f"goal_plan_{goal_id}.py file with a ### LEARNINGS block appended.",
        "",
        "============================================================",
        "## INPUT 1: Plan Review Guidelines (plan_review_prompt.md)",
        "============================================================",
        guidelines_text or "(plan_review_prompt.md not found)",
        "",
        "============================================================",
        f"## INPUT 2: Review Bundle (goal_review_{goal_id}.md)",
        "============================================================",
        bundle_text,
        "",
        "============================================================",
        f"## INPUT 3: Current Plan File (goal_plan_{goal_id}.py)",
        "============================================================",
        plan_text or "(no plan file present — goal has no cached plan yet)",
        "",
        "============================================================",
        f"## INPUT 4: Planner Trace (goal_trace_{goal_id}.txt)",
        "============================================================",
        trace_text or "(no trace — last run was replay mode or goal has not run)",
        "",
        "============================================================",
        f"## YOUR TASK",
        "============================================================",
        "Apply the review checklist from INPUT 1 to the plan in INPUT 3, "
        "informed by the bundle (INPUT 2) and trace (INPUT 4). Output the "
        "literal Python file content for the rewritten plan, ending with a "
        "### LEARNINGS block. No prose, no fences, no commentary. End with "
        "\\n</end>.",
    ]
    return "\n".join(parts)


# ──────────────────────────────────────────────────────────────────────────
# CLI
# ──────────────────────────────────────────────────────────────────────────

def main(argv: Optional[list] = None) -> int:
    p = argparse.ArgumentParser(
        description="Automated R-Opus reviewer for the bench harness. "
        "Triggers /goal plan review, sends the bundle to an Anthropic "
        "model, writes the rewritten plan, and triggers /goal plan commit."
    )
    sub = p.add_subparsers(dest="cmd", required=True)

    sp = sub.add_parser("review", help="Review a single goal")
    sp.add_argument("goal_id", help="Goal ID to review (e.g. goal_15)")
    sp.add_argument("--character", default=DEFAULT_CHARACTER)
    sp.add_argument("--model", default=DEFAULT_MODEL,
                    help=f"Anthropic model (default: {DEFAULT_MODEL}). "
                         f"Use claude-opus-4-6 for production runs.")
    sp.add_argument("--api-key", default=None,
                    help="Anthropic API key (or set CLAUDE_API_KEY)")
    sp.add_argument("--max-tokens", type=int, default=DEFAULT_MAX_TOKENS)
    sp.add_argument("--temperature", type=float, default=DEFAULT_TEMPERATURE)
    sp.add_argument("--bundle-timeout", type=float, default=DEFAULT_BUNDLE_TIMEOUT_S)
    sp.add_argument("--commit-timeout", type=float, default=DEFAULT_COMMIT_TIMEOUT_S)
    sp.add_argument("--dry-run", action="store_true",
                    help="Run the LLM review but do not commit — leaves no "
                         "files on disk and does not mutate agent state. "
                         "The rewritten plan is only returned in the result dict.")
    sp.add_argument("--output", type=Path, default=None,
                    help="Optional path to write the result JSON")

    args = p.parse_args(argv)

    if args.cmd == "review":
        result = review_goal(
            args.goal_id,
            character=args.character,
            model=args.model,
            api_key=args.api_key,
            max_tokens=args.max_tokens,
            temperature=args.temperature,
            bundle_timeout=args.bundle_timeout,
            commit_timeout=args.commit_timeout,
            dry_run=args.dry_run,
        )
        # Don't dump the full plan_text to stdout — too noisy. Print a
        # summary line and write the full result to --output if requested.
        summary = {k: v for k, v in result.items() if k != "plan_text"}
        print(json.dumps(summary, indent=2))
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            with open(args.output, "w", encoding="utf-8") as f:
                json.dump(result, f, indent=2)
            print(f"(full result written to {args.output})", file=sys.stderr)
        # Exit success: dry-run is fine as long as the LLM call succeeded;
        # non-dry-run needs committed=True.
        if args.dry_run:
            return 0 if result.get("llm_called") and not result.get("error") else 1
        return 0 if result.get("committed") else 1

    return 2


if __name__ == "__main__":
    sys.exit(main())
