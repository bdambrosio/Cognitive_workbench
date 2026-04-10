#!/usr/bin/env python3
"""Stage 2: Opus quality verdict for a Cognitive Workbench harvester row.

Takes a row produced by bench/harvester.py and asks an Anthropic model
(default: claude-sonnet-4-6) to independently grade the run against the
goal text. The verdict is structured: per-objective achievement judgments,
overall score, alignment with the system's vision criteria, and notable
issues. The result decorates the row in-place under `opus_verdict`.

Two ways to invoke:

  1. Standalone, on a row JSON string:
       echo '{"goal_id": "goal_18", ...}' | python bench/verdict.py --stdin

  2. Standalone, on a JSONL file (one decorated row per input row):
       python bench/verdict.py --jsonl-file runs.jsonl --out runs_graded.jsonl

  3. Imported by harvester.py via the --opus flag (the harvester reuses
     its open Zenoh session for primary-product lookup).

Requirements:
  - CLAUDE_API_KEY (or ANTHROPIC_API_KEY) in the environment
  - anthropic Python SDK installed in the venv
  - For primary-product content lookup: a running launcher with the
    target character available on Zenoh (optional — verdict still runs
    against last_result alone if the resource can't be loaded)
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

# Path setup so we can import from src/ if needed (Zenoh helpers).
_REPO_ROOT = Path(__file__).resolve().parent.parent
_SRC_DIR = _REPO_ROOT / "src"
sys.path.insert(0, str(_SRC_DIR))

DEFAULT_MODEL = "claude-sonnet-4-6"
DEFAULT_MAX_TOKENS = 4096
DEFAULT_TEMPERATURE = 0.0
DEFAULT_CHARACTER = "Jill"
STOP_MARKER = "</end>"


# ──────────────────────────────────────────────────────────────────────────
# Anthropic API key resolution
# ──────────────────────────────────────────────────────────────────────────

def _resolve_api_key(explicit: Optional[str] = None) -> Optional[str]:
    """Find the Anthropic API key from explicit arg, CLAUDE_API_KEY env, or
    ANTHROPIC_API_KEY env (in that order of precedence)."""
    if explicit:
        return explicit
    return os.environ.get("CLAUDE_API_KEY") or os.environ.get("ANTHROPIC_API_KEY")


# ──────────────────────────────────────────────────────────────────────────
# Zenoh primary-product lookup
# ──────────────────────────────────────────────────────────────────────────

def _open_zenoh_session():
    """Open a Zenoh session against the localhost router. Returns None if
    Zenoh is unavailable so callers can degrade gracefully."""
    try:
        import zenoh  # noqa: WPS433
        from utils.zenoh_utils import make_localhost_config  # noqa: WPS433
        return zenoh.open(make_localhost_config())
    except Exception as e:
        print(f"WARN: could not open Zenoh session: {e}", file=sys.stderr)
        return None


def load_resource_content(
    session: Any, character: str, resource_id: str, timeout: float = 3.0
) -> Optional[str]:
    """Query the agent's resource_by_id queryable and return the Note's
    content string. Returns None if the resource isn't found, the queryable
    is silent, or the response is malformed.

    The queryable lives at `cognitive/<character>/resource/<resource_id>`
    and replies with `{"success": True, "resource": {properties: {content: ...}}}`.
    """
    if session is None or not resource_id:
        return None
    key = f"cognitive/{character}/resource/{resource_id}"
    try:
        for reply in session.get(key, timeout=timeout):
            if not (hasattr(reply, "ok") and reply.ok is not None):
                continue
            try:
                payload = json.loads(reply.ok.payload.to_bytes().decode("utf-8"))
            except Exception:
                continue
            if not payload.get("success"):
                return None
            res = payload.get("resource") or {}
            props = res.get("properties") or {}
            content = props.get("content")
            if content is None:
                return None
            if isinstance(content, str):
                return content
            return json.dumps(content, ensure_ascii=False)
    except Exception as e:
        print(f"WARN: resource lookup failed for {resource_id}: {e}", file=sys.stderr)
    return None


# ──────────────────────────────────────────────────────────────────────────
# Prompt construction
# ──────────────────────────────────────────────────────────────────────────

_VERDICT_SYSTEM_PROMPT = """You are an independent quality reviewer for an autonomous agent's run on a goal.

Your job: decompose the goal into atomic objectives, then judge whether each objective was achieved by the agent's actual output. Be specific (cite evidence from the actual run artifacts) and independent (do NOT trust the agent's self-assessment — grade against the goal text itself).

Rules:
- An objective is a concrete, verifiable thing the goal asks for. Decompose into 1-5 objectives.
- "achieved" is true only if the run produced explicit evidence (in LAST RESULT or PRIMARY PRODUCT) that the objective was met. The agent claiming success is not evidence.
- Cite quoted evidence from LAST RESULT or PRIMARY PRODUCT for each PASS verdict. For FAIL, briefly state what was missing.
- "score_percent" is achieved_count / total_count * 100, integer.
- "verdict" is "satisfied" if all objectives PASS, "partial" if some PASS, "failed" if none PASS.
- "notable_issues" lists work the planner did that wasn't asked for (redundant retries, second-guessing, off-topic detours, missing $eval_target binding, etc.). Empty list is fine.
- "criteria_alignment" maps each system criterion label to PASS / FAIL / N/A. N/A means the criterion's condition didn't apply (e.g. "if matches exist" when there were no matches).
- If PRIMARY PRODUCT is "(no primary product bound)", grade against LAST RESULT alone, lower confidence accordingly.

OUTPUT FORMAT (CRITICAL):
- Respond with the raw JSON object ONLY. No prose before. No prose after.
- Do NOT wrap the JSON in markdown code fences (no ```json, no ```).
- Do NOT add any explanation or commentary outside the JSON.
- Start your response with the opening { character. End the JSON with }.
- After the closing }, output \\n</end> and stop.

end your output with
</end>"""


def _extract_json_object(text: str) -> str:
    """Extract a JSON object from a model response that may include markdown
    code fences, leading/trailing prose, or the </end> stop marker.

    Strategy: find the first '{' and the last '}' and slice between them.
    This is robust against ```json ... ``` wrapping, "Here's the verdict:"
    prefixes, and trailing prose. It assumes the JSON object is the only
    object in the response (no nested objects outside the primary one),
    which is enforced by the system prompt schema.

    If no braces are found, returns the original text so json.loads fails
    with a meaningful error.
    """
    text = (text or "").strip()
    if text.endswith(STOP_MARKER):
        text = text[: -len(STOP_MARKER)].rstrip()
    start = text.find("{")
    end = text.rfind("}")
    if start == -1 or end == -1 or end < start:
        return text
    return text[start : end + 1]


def _build_user_prompt(row: Dict[str, Any], primary_product_content: Optional[str]) -> str:
    """Construct the user message containing the goal and run artifacts."""
    goal_text = row.get("goal_text") or "(no goal text)"
    criteria = (row.get("vision_criteria") or "").strip()
    cached_plan = row.get("cached_plan", "") or row.get("cached_plan_source", "")
    last_result = (row.get("last_result") or "").strip()

    pp_id = row.get("primary_product") or ""
    if primary_product_content:
        pp_section = (
            f"PRIMARY PRODUCT ({pp_id}):\n{primary_product_content[:8000]}"
        )
    elif pp_id:
        pp_section = (
            f"PRIMARY PRODUCT ({pp_id}): (content unavailable — resource lookup failed)"
        )
    else:
        pp_section = (
            "PRIMARY PRODUCT: (no primary product bound — grade against LAST RESULT alone)"
        )

    if criteria:
        criteria_section = f"SYSTEM CRITERIA (the planner's own quality criteria, may be incomplete):\n{criteria}"
    else:
        criteria_section = "SYSTEM CRITERIA: (none recorded)"

    if cached_plan:
        plan_section = f"CACHED PLAN (the code blocks the planner produced):\n{cached_plan[:6000]}"
    else:
        plan_section = "CACHED PLAN: (not available)"

    schema = """{
  "objectives": [
    {"id": "obj1", "text": "<atomic objective from the goal>",
     "achieved": true|false, "evidence": "<quoted from LAST RESULT or PRIMARY PRODUCT>",
     "confidence": 0.0-1.0}
  ],
  "achieved_count": <int>,
  "total_count": <int>,
  "score_percent": <int 0-100>,
  "verdict": "satisfied" | "partial" | "failed",
  "notable_issues": ["<unrequested or wasted work>", ...],
  "criteria_alignment": {"<criterion_label>": "PASS" | "FAIL" | "N/A", ...}
}"""

    return f"""GOAL:
{goal_text}

{criteria_section}

{plan_section}

LAST RESULT (planner's own narrative summary):
{last_result[:4000] if last_result else "(empty)"}

{pp_section}

Schema (output JSON only, then </end>):
{schema}"""


# ──────────────────────────────────────────────────────────────────────────
# Anthropic call
# ──────────────────────────────────────────────────────────────────────────

def _call_anthropic(
    system_prompt: str,
    user_prompt: str,
    model: str,
    max_tokens: int,
    temperature: float,
    api_key: str,
) -> Dict[str, Any]:
    """Call the Anthropic Messages API and return the parsed verdict dict.

    Raises ImportError if the anthropic SDK isn't installed.
    Raises RuntimeError on API errors. Returns the raw text on JSON parse
    failure (caller should attach as error to the row).
    """
    try:
        from anthropic import Anthropic  # noqa: WPS433
    except ImportError as e:
        raise ImportError(
            "The 'anthropic' Python SDK is not installed in this venv. "
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

    # Extract text from the response. Modern SDK returns content as a list
    # of blocks; for a text-only response we want the first text block.
    text = ""
    for block in response.content:
        if getattr(block, "type", None) == "text":
            text += getattr(block, "text", "") or ""
    text = text.strip()

    # Strip the </end> marker if the model included it (it shouldn't, since
    # we set it as a stop sequence, but be defensive).
    if text.endswith(STOP_MARKER):
        text = text[: -len(STOP_MARKER)].rstrip()

    return {
        "text": text,
        "elapsed_ms": elapsed_ms,
        "stop_reason": getattr(response, "stop_reason", None),
        "input_tokens": getattr(response.usage, "input_tokens", None),
        "output_tokens": getattr(response.usage, "output_tokens", None),
    }


# ──────────────────────────────────────────────────────────────────────────
# Main entry: compute_verdict
# ──────────────────────────────────────────────────────────────────────────

def compute_verdict(
    row: Dict[str, Any],
    *,
    session: Any = None,
    character: str = DEFAULT_CHARACTER,
    model: str = DEFAULT_MODEL,
    api_key: Optional[str] = None,
    max_tokens: int = DEFAULT_MAX_TOKENS,
    temperature: float = DEFAULT_TEMPERATURE,
    plan_path: Optional[Path] = None,
) -> Dict[str, Any]:
    """Decorate a stage 1 row with an Opus quality verdict.

    Returns a NEW dict with the row's fields plus an `opus_verdict`
    sub-object. The original row is not mutated.

    Args:
        row: A stage 1 row dict (as produced by harvester.py).
        session: Optional open Zenoh session for primary-product lookup.
            If None, a session is opened temporarily and closed at the end.
            Pass an existing session to share with the harvester.
        character: Character name for the resource queryable.
        model: Anthropic model identifier.
        api_key: Optional explicit API key. Defaults to CLAUDE_API_KEY env.
        max_tokens: Max output tokens.
        temperature: Sampling temperature.
        plan_path: Optional path to the cached plan source file. If not
            provided and the row has a `goal_id`, looks for
            `goal_plan_<id>.py` at the repo root.
    """
    out = dict(row)
    verdict_meta: Dict[str, Any] = {
        "model": model,
        "verdict_ts": datetime.now(timezone.utc).isoformat(timespec="seconds"),
    }

    resolved_key = _resolve_api_key(api_key)
    if not resolved_key:
        verdict_meta["error"] = (
            "no API key found (set CLAUDE_API_KEY or ANTHROPIC_API_KEY)"
        )
        out["opus_verdict"] = verdict_meta
        return out

    # Load primary product content if a Note ID is bound and we can reach it.
    pp_id = (out.get("primary_product") or "").strip()
    pp_content: Optional[str] = None
    opened_session_locally = False
    if pp_id and pp_id.startswith("Note_"):
        if session is None:
            session = _open_zenoh_session()
            opened_session_locally = True
        pp_content = load_resource_content(session, character, pp_id)
    try:
        # Load the cached plan source from disk if available — gives Opus
        # the actual code the planner ran rather than just a step count.
        if plan_path is None:
            goal_id = out.get("goal_id") or ""
            if goal_id:
                candidate = _REPO_ROOT / f"goal_plan_{goal_id}.py"
                if candidate.exists():
                    plan_path = candidate
        if plan_path and plan_path.exists():
            try:
                out_local = dict(out)
                out_local["cached_plan"] = plan_path.read_text(encoding="utf-8")
                user_prompt = _build_user_prompt(out_local, pp_content)
            except Exception:
                user_prompt = _build_user_prompt(out, pp_content)
        else:
            user_prompt = _build_user_prompt(out, pp_content)

        try:
            api_result = _call_anthropic(
                system_prompt=_VERDICT_SYSTEM_PROMPT,
                user_prompt=user_prompt,
                model=model,
                max_tokens=max_tokens,
                temperature=temperature,
                api_key=resolved_key,
            )
        except ImportError as e:
            verdict_meta["error"] = str(e)
            out["opus_verdict"] = verdict_meta
            return out
        except Exception as e:
            verdict_meta["error"] = f"API call failed: {e}"
            out["opus_verdict"] = verdict_meta
            return out

        verdict_meta["elapsed_ms"] = api_result.get("elapsed_ms")
        verdict_meta["input_tokens"] = api_result.get("input_tokens")
        verdict_meta["output_tokens"] = api_result.get("output_tokens")
        verdict_meta["stop_reason"] = api_result.get("stop_reason")
        verdict_meta["primary_product_loaded"] = pp_content is not None

        # Parse the JSON verdict. Use _extract_json_object to defensively
        # strip markdown code fences and any stray prose around the JSON —
        # the system prompt forbids fences but models do it anyway.
        text = api_result.get("text", "")
        json_text = _extract_json_object(text)
        try:
            parsed = json.loads(json_text)
            verdict_meta.update(parsed)
        except json.JSONDecodeError as e:
            verdict_meta["error"] = f"verdict JSON parse failed: {e}"
            verdict_meta["raw_text"] = text[:2000]

        out["opus_verdict"] = verdict_meta
        return out
    finally:
        if opened_session_locally and session is not None:
            try:
                session.close()
            except Exception:
                pass


# ──────────────────────────────────────────────────────────────────────────
# Human-readable summary of the verdict
# ──────────────────────────────────────────────────────────────────────────

def format_verdict_summary(row: Dict[str, Any]) -> str:
    """Render the opus_verdict subdict as a multi-line human-readable block."""
    v = row.get("opus_verdict") or {}
    if not v:
        return "(no opus_verdict on row)"
    if v.get("error"):
        return f"OPUS VERDICT ERROR: {v['error']}"
    lines = []
    lines.append("─" * 70)
    lines.append(f"OPUS VERDICT  ({v.get('model', '?')})")
    score = v.get("score_percent")
    verdict = v.get("verdict", "?")
    achieved = v.get("achieved_count", "?")
    total = v.get("total_count", "?")
    lines.append(
        f"  verdict={verdict}  score={score}%  ({achieved}/{total} objectives)"
    )
    in_tok = v.get("input_tokens")
    out_tok = v.get("output_tokens")
    elapsed = v.get("elapsed_ms")
    if in_tok is not None or out_tok is not None or elapsed is not None:
        parts = []
        if in_tok is not None:
            parts.append(f"in={in_tok}")
        if out_tok is not None:
            parts.append(f"out={out_tok}")
        if elapsed is not None:
            parts.append(f"{elapsed}ms")
        lines.append(f"  cost  " + " ".join(parts))

    objectives = v.get("objectives") or []
    if objectives:
        lines.append("")
        lines.append("Objectives:")
        for o in objectives:
            mark = "✓" if o.get("achieved") else "✗"
            oid = o.get("id", "?")
            text = (o.get("text") or "")[:90]
            conf = o.get("confidence")
            conf_str = f" [conf={conf}]" if conf is not None else ""
            lines.append(f"  {mark} {oid}: {text}{conf_str}")
            evidence = (o.get("evidence") or "").strip()
            if evidence:
                lines.append(f"    └ {evidence[:200]}")

    crit = v.get("criteria_alignment") or {}
    if crit:
        lines.append("")
        lines.append("Criteria alignment:")
        for label, status in crit.items():
            mark = {"PASS": "✓", "FAIL": "✗", "N/A": "·"}.get(status, "?")
            lines.append(f"  {mark} {label}: {status}")

    issues = v.get("notable_issues") or []
    if issues:
        lines.append("")
        lines.append("Notable issues:")
        for issue in issues:
            lines.append(f"  - {issue}")

    lines.append("─" * 70)
    return "\n".join(lines)


# ──────────────────────────────────────────────────────────────────────────
# CLI
# ──────────────────────────────────────────────────────────────────────────

def _process_one_row(
    row: Dict[str, Any],
    args: argparse.Namespace,
    session: Any,
) -> Dict[str, Any]:
    return compute_verdict(
        row,
        session=session,
        character=args.character,
        model=args.model,
        api_key=None,
        max_tokens=args.max_tokens,
        temperature=args.temperature,
    )


def main(argv: Optional[List[str]] = None) -> int:
    p = argparse.ArgumentParser(
        description="Stage 2 Opus quality verdict for harvester rows",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--stdin", action="store_true",
                     help="Read one JSON row from stdin")
    src.add_argument("--row", help="JSON row as a single argument")
    src.add_argument("--jsonl-file", type=Path,
                     help="Read multiple rows from a JSONL file")

    p.add_argument("--out", type=Path, default=None,
                   help="Write decorated rows to this file. Default: stdout.")
    p.add_argument("--model", default=DEFAULT_MODEL,
                   help="Anthropic model identifier")
    p.add_argument("--character", default=DEFAULT_CHARACTER,
                   help="Character name for resource queryable lookup")
    p.add_argument("--max-tokens", type=int, default=DEFAULT_MAX_TOKENS)
    p.add_argument("--temperature", type=float, default=DEFAULT_TEMPERATURE)
    p.add_argument("--no-summary", action="store_true",
                   help="Suppress the human-readable summary on stderr")

    args = p.parse_args(argv)

    # Resolve API key up front so we can fail fast.
    if not _resolve_api_key():
        print(
            "ERROR: no API key found. Set CLAUDE_API_KEY or ANTHROPIC_API_KEY.",
            file=sys.stderr,
        )
        return 5

    # Collect input rows.
    rows: List[Dict[str, Any]] = []
    if args.stdin:
        try:
            rows.append(json.loads(sys.stdin.read()))
        except json.JSONDecodeError as e:
            print(f"ERROR: stdin is not valid JSON: {e}", file=sys.stderr)
            return 6
    elif args.row:
        try:
            rows.append(json.loads(args.row))
        except json.JSONDecodeError as e:
            print(f"ERROR: --row is not valid JSON: {e}", file=sys.stderr)
            return 6
    elif args.jsonl_file:
        if not args.jsonl_file.exists():
            print(f"ERROR: jsonl file not found: {args.jsonl_file}", file=sys.stderr)
            return 6
        for line in args.jsonl_file.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                print(f"WARN: skipping invalid JSONL line: {line[:80]}", file=sys.stderr)

    if not rows:
        print("ERROR: no input rows", file=sys.stderr)
        return 6

    # Open Zenoh once and reuse for primary-product lookups across all rows.
    # If it fails, that's OK — verdict still works against last_result alone.
    session = _open_zenoh_session()

    out_rows: List[Dict[str, Any]] = []
    try:
        for i, row in enumerate(rows, 1):
            print(
                f"Grading row {i}/{len(rows)}: {row.get('goal_id', '?')}",
                file=sys.stderr,
            )
            decorated = _process_one_row(row, args, session)
            out_rows.append(decorated)
            if not args.no_summary:
                print(format_verdict_summary(decorated), file=sys.stderr)
    finally:
        if session is not None:
            try:
                session.close()
            except Exception:
                pass

    # Emit output.
    out_text = "\n".join(json.dumps(r, ensure_ascii=False) for r in out_rows)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        with open(args.out, "a", encoding="utf-8") as f:
            f.write(out_text + "\n")
        print(f"Appended {len(out_rows)} row(s) to {args.out}", file=sys.stderr)
    else:
        print(out_text)

    # Exit code: 0 if all rows graded cleanly, 7 if any row had an error.
    any_error = any(
        (r.get("opus_verdict") or {}).get("error") for r in out_rows
    )
    return 7 if any_error else 0


if __name__ == "__main__":
    sys.exit(main())
