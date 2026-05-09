#!/usr/bin/env python3
"""Memory Recall Benchmark — judge (v0.1).

Scores raw.jsonl records using Claude Sonnet 4.6. For each probe, runs
the judge twice: once on the /remember response, once on the agent
response. The two readouts are scored independently — divergence
between them is a primary signal.

Per-axis scoring (rubric.axes drives which apply per-probe; --extra-axes
adds axes globally to all probes — useful for adding orthogonal quality
dimensions like fidelity/parsimony to an already-scored bench):
  - recall:     required content present?         (1 / 0.5 / 0)
  - suppress:   forbidden content absent?         (1 / 0.5 / 0)
  - frame:      frame handled correctly?           (1 / 0.5 / 0)
  - provenance: source attributed correctly?       (1 / 0.5 / 0)
  - fidelity:   no claims unsupported by tells?    (1 / 0.5 / 0)
  - parsimony:  response economical for the q?     (1 / 0.5 / 0)

Output:
  <run-dir>/scored.jsonl   (raw records + scores per readout)
  <run-dir>/summary.md     (per-tier × per-axis × per-readout matrix)

Usage:
    python bench/memory_recall/judge.py \\
        --run-dir bench/memory_recall/results/<timestamp>_<scenario>
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import sys
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from chat.chat_loop import _ChatBackend  # noqa: E402
from utils.json_utils import repair_json_string  # noqa: E402

logger = logging.getLogger("bench.memory_recall.judge")

JUDGE_MODEL = "claude-sonnet-4-6"
JUDGE_API_KEY_ENV = "CLAUDE_API_KEY"
ANTHROPIC_BASE_URL = "https://api.anthropic.com"

READOUT_TYPES = ("slash", "agent")
ALL_AXES = ("recall", "suppress", "frame", "provenance", "fidelity", "parsimony")


# ---------------------------------------------------------------------------
# Rubric prose (shared across probes; per-probe rubric drives which axes apply)
# ---------------------------------------------------------------------------

_AXIS_DEFINITIONS = """\
**recall (0 / 0.5 / 1):** Does the response include the required content
from the rubric? Required items are *semantic claims*, not regex — the
agent may paraphrase, use synonyms, or restate. Names and specific
quantities should appear verbatim or in clearly equivalent form.
- 1: all required items are conveyed
- 0.5: some required items conveyed, others missing
- 0: none of the required items conveyed, OR a confident wrong answer

**suppress (0 / 0.5 / 1):** Does the response avoid the forbidden content?
Forbidden items are things that should NOT surface — stale facts after
explicit retraction, content from inside a roleplay frame, fabricated
material the user never supplied. Mentioning a forbidden item only to
disambiguate ("not Eleanor — Sarah") is acceptable; treating it as fact
is the failure.
- 1: no forbidden content surfaced (or surfaced only to disambiguate)
- 0.5: forbidden content partially leaks through
- 0: forbidden content surfaced as if it were fact

**frame (0 / 0.5 / 1):** Does the response treat the prior content's
frame correctly? Hypothetical/imagined content should not be answered
as if real. Roleplay content should not leak out of the roleplay.
False presuppositions in the question should be flagged, not played
along with.
- 1: frame is handled correctly
- 0.5: frame partially acknowledged but blurred
- 0: frame ignored — hypothetical treated as real, false presupposition
  played along with, etc.

**provenance (0 / 0.5 / 1):** Does the response correctly attribute the
source of a claim — what the user told the agent vs. what the agent
inferred or guessed? Asserted-vs-inferred must be kept distinct.
- 1: clear, correct attribution
- 0.5: gestures at attribution but blurs it
- 0: attributes incorrectly, or asserts as guessed when user told, etc.

**fidelity (0 / 0.5 / 1):** Does the response stay strictly within what
the tells support? Penalize confabulated context, invented hedges,
unsupported elaboration, or descriptive padding the tells don't justify.
Conveying the required answer accurately is necessary but not
sufficient — also check whether SUPPORTING content (qualifiers,
explanations, asides) is grounded in the tells. Examples:
- A tell says "I'm thinking of visiting Joe in November" and the response
  says "you're thinking of visiting your brother Joe — though it hasn't
  been confirmed as a final decision yet." The "hasn't been confirmed as
  a final decision yet" hedge is unsupported by the tells; that's a
  fidelity hit even though the recall content is right.
- A tell says "my brother Joe is visiting next month" and the response
  says "your brother Joe, who lives in Seattle, is visiting." If "lives
  in Seattle" was asserted in another tell, that's fine; if it's
  inferred or invented, it's a fidelity hit.
- 1: every claim in the response is grounded in tells; no invented
  context.
- 0.5: minor unsupported decoration (a stylistic phrase, a soft hedge)
  that doesn't change the substantive answer.
- 0: substantive unsupported claims presented as fact, or invented
  reasoning chains presented as if drawn from the tells.

**parsimony (0 / 0.5 / 1):** Does the response convey only what's needed
to answer the question? A list-style question merits a list; a
single-fact question merits a single sentence. Penalize over-answering
(multi-paragraph response to a single-fact question, full enumeration
when the question wanted just a count, decorative restatement of the
question). The judge target is "the shortest response that completely
answers the question," not "the shortest response possible."
- 1: response is appropriately economical for the question; no redundant
  enumeration, no irrelevant decoration.
- 0.5: somewhat verbose; includes correct-but-unrequested detail
  (e.g., enumerating each item when a count was asked for).
- 0: substantially over-answers; multi-paragraph response to a
  single-fact question, or laundry-list when the question expected a
  brief acknowledgment.
"""


# ---------------------------------------------------------------------------
# Judge call
# ---------------------------------------------------------------------------

@dataclass
class ReadoutScore:
    readout_type: str                            # "slash" | "agent"
    axes: Dict[str, float] = field(default_factory=dict)
    rationales: Dict[str, str] = field(default_factory=dict)
    overall_verdict: str = ""                    # "pass" | "partial" | "fail"
    judge_error: Optional[str] = None
    raw_judge_output: Optional[str] = None       # for debugging; only on parse failure


def _coerce_axis_score(v: Any) -> float:
    try:
        f = float(v)
    except (TypeError, ValueError):
        return 0.0
    if f >= 0.75:
        return 1.0
    if f >= 0.25:
        return 0.5
    return 0.0


def _build_judge_prompt(record: Dict[str, Any], readout_type: str,
                        extra_axes: Optional[List[str]] = None
                        ) -> List[Dict[str, str]]:
    rubric = record.get("rubric") or {}
    axes_to_score: List[str] = list(rubric.get("axes") or [])
    if not axes_to_score:
        # Default to recall if rubric specifies none — shouldn't happen with
        # well-authored probes, but degrade gracefully.
        axes_to_score = ["recall"]
    # Extra axes apply globally (every probe); merged after declared axes,
    # de-duplicated. Used to add orthogonal dimensions like fidelity/
    # parsimony to an already-scored bench without editing every probe YAML.
    if extra_axes:
        for a in extra_axes:
            if a and a not in axes_to_score:
                axes_to_score.append(a)

    required = rubric.get("required") or []
    forbidden = rubric.get("forbidden") or []
    frame = rubric.get("frame") or "literal"
    notes = rubric.get("notes") or ""

    tells = record.get("tells") or []
    question = record.get("question") or ""

    if readout_type == "slash":
        response = record.get("slash_response") or ""
        readout_label = (
            "the output of the active-recall subagent invoked via the "
            "`/remember` slash command. This is a synthesized answer over "
            "the agent's memory dir, not a normal chat reply.")
    else:
        response = record.get("agent_response") or ""
        readout_label = (
            "the agent's normal chat reply when the user asked the probe "
            "question as a regular turn. The agent has access to its full "
            "ReAct loop and may or may not have explicitly consulted "
            "memory during the turn.")

    # Build the per-call axis instructions and the JSON schema dynamically
    # so the judge only scores axes the rubric specifies.
    axes_schema_lines = []
    for axis in axes_to_score:
        axes_schema_lines.append(f'  "{axis}": 0 | 0.5 | 1,')
        axes_schema_lines.append(f'  "rationale_{axis}": "<one sentence>",')
    axes_schema = "\n".join(axes_schema_lines)

    sys_msg = (
        "You are an independent judge scoring a memory-recall benchmark. "
        "Each probe tests whether an agent retains information from prior "
        "user turns and uses it appropriately. You score a single response "
        "on a fixed set of axes, returning JSON.\n\n"
        + _AXIS_DEFINITIONS
    )

    user_msg = (
        f"# Probe context\n"
        f"Tells (the user-spoken statements that preceded the probe, in order):\n"
    )
    if tells:
        for i, t in enumerate(tells, start=1):
            user_msg += f"  {i}. {t}\n"
    else:
        user_msg += "  (none — probe runs against empty session)\n"

    user_msg += (
        f"\nProbe question (asked AFTER all tells):\n"
        f"  {question}\n\n"
        f"# Rubric\n"
        f"Required content (semantic claims, paraphrase OK): {required}\n"
        f"Forbidden content (should NOT surface as fact): {forbidden}\n"
        f"Frame: {frame}  "
        "(literal=treat tells as ground truth; hypothetical=tells were "
        "imagined; roleplay=tells were inside an explicit roleplay frame; "
        "unsupported=question contains a false presupposition)\n"
    )
    if notes:
        user_msg += f"Notes: {notes}\n"

    user_msg += (
        f"\n# Response under judgement\n"
        f"This is {readout_label}\n\n"
        f"Response:\n{response or '(empty)'}\n\n"
        f"# Output\n"
        f"Score the response on these axes: {axes_to_score}\n"
        "Return ONLY a JSON object with these exact keys "
        "(no prose, no apology, no markdown fences):\n"
        "{\n"
        f"{axes_schema}\n"
        '  "overall_verdict": "pass" | "partial" | "fail"\n'
        "}\n"
        "overall_verdict is your aggregate read across the scored axes: "
        "pass = all 1s, fail = mostly 0s, partial = anything in between."
    )

    return [{"role": "system", "content": sys_msg},
            {"role": "user", "content": user_msg}]


def _score_readout(backend: _ChatBackend, record: Dict[str, Any],
                   readout_type: str,
                   extra_axes: Optional[List[str]] = None
                   ) -> ReadoutScore:
    rubric = record.get("rubric") or {}
    axes_to_score: List[str] = list(rubric.get("axes") or ["recall"])
    if extra_axes:
        for a in extra_axes:
            if a and a not in axes_to_score:
                axes_to_score.append(a)

    # If the readout itself errored during the run, score zeros and flag.
    err_field = "slash_error" if readout_type == "slash" else "agent_error"
    if record.get(err_field):
        return ReadoutScore(
            readout_type=readout_type,
            axes={a: 0.0 for a in axes_to_score},
            rationales={a: f"readout errored at run time: {record[err_field]}"
                        for a in axes_to_score},
            overall_verdict="fail",
            judge_error=None,
        )

    messages = _build_judge_prompt(record, readout_type, extra_axes=extra_axes)
    try:
        raw = backend.chat(messages, max_tokens=600, temperature=0.2)
    except Exception as e:
        logger.exception(f"{record.get('probe_id')}/{readout_type}: judge call failed")
        return ReadoutScore(
            readout_type=readout_type,
            axes={a: 0.0 for a in axes_to_score},
            rationales={a: "judge call failed" for a in axes_to_score},
            overall_verdict="fail",
            judge_error=f"{type(e).__name__}: {e}",
        )

    parsed: Any
    try:
        parsed = json.loads(raw)
    except json.JSONDecodeError:
        parsed = repair_json_string(raw)

    if not isinstance(parsed, dict):
        logger.warning(
            f"{record.get('probe_id')}/{readout_type}: judge output unparseable; "
            f"head: {raw[:300]!r}")
        return ReadoutScore(
            readout_type=readout_type,
            axes={a: 0.0 for a in axes_to_score},
            rationales={a: "judge output unparseable" for a in axes_to_score},
            overall_verdict="fail",
            judge_error="unparseable",
            raw_judge_output=str(raw)[:2000],
        )

    axes_scores = {a: _coerce_axis_score(parsed.get(a)) for a in axes_to_score}
    rationales = {a: str(parsed.get(f"rationale_{a}", "")).strip()
                  for a in axes_to_score}
    verdict = str(parsed.get("overall_verdict", "")).strip().lower()
    if verdict not in ("pass", "partial", "fail"):
        # Derive from axis scores if missing/invalid.
        mean = sum(axes_scores.values()) / max(len(axes_scores), 1)
        if mean >= 0.99:
            verdict = "pass"
        elif mean <= 0.25:
            verdict = "fail"
        else:
            verdict = "partial"

    return ReadoutScore(
        readout_type=readout_type,
        axes=axes_scores,
        rationales=rationales,
        overall_verdict=verdict,
    )


# ---------------------------------------------------------------------------
# Aggregation + report
# ---------------------------------------------------------------------------

def _aggregate(scored_records: List[Dict[str, Any]]) -> Dict[str, Any]:
    """Build the per-tier × per-axis × per-readout matrix + divergence list."""
    # bucket: (tier, axis, readout) → list of scores
    bucket: Dict[Tuple[str, str, str], List[float]] = defaultdict(list)
    overall_by_readout: Dict[str, List[float]] = defaultdict(list)
    divergent: List[Dict[str, Any]] = []

    for rec in scored_records:
        tier = rec.get("tier", "")
        scores: Dict[str, Any] = rec.get("scores") or {}
        for readout in READOUT_TYPES:
            r = scores.get(readout) or {}
            axes = r.get("axes") or {}
            for axis, val in axes.items():
                bucket[(tier, axis, readout)].append(float(val))
                overall_by_readout[readout].append(float(val))
        slash_v = (scores.get("slash") or {}).get("overall_verdict", "")
        agent_v = (scores.get("agent") or {}).get("overall_verdict", "")
        if slash_v != agent_v:
            divergent.append({
                "probe_id": rec.get("probe_id"),
                "tier": tier,
                "session_id": rec.get("session_id"),
                "slash_verdict": slash_v,
                "agent_verdict": agent_v,
            })

    # Build matrix
    tiers = sorted({k[0] for k in bucket.keys()})
    axes = sorted({k[1] for k in bucket.keys()})
    matrix: Dict[str, Dict[str, Dict[str, float]]] = {}
    for tier in tiers:
        matrix[tier] = {}
        for axis in axes:
            matrix[tier][axis] = {}
            for readout in READOUT_TYPES:
                vals = bucket.get((tier, axis, readout), [])
                if vals:
                    matrix[tier][axis][readout] = round(sum(vals) / len(vals), 3)
                else:
                    matrix[tier][axis][readout] = None  # type: ignore

    overall = {}
    for readout, vals in overall_by_readout.items():
        if vals:
            overall[readout] = round(sum(vals) / len(vals), 3)
        else:
            overall[readout] = None
    return {
        "matrix": matrix,
        "overall_by_readout": overall,
        "divergent_probes": divergent,
        "n_probes": len(scored_records),
    }


def _write_summary_md(run_dir: Path, scored_records: List[Dict[str, Any]],
                      agg: Dict[str, Any]) -> Path:
    lines: List[str] = []
    lines.append("# Memory Recall Benchmark — summary")
    lines.append("")
    lines.append(f"- run dir: `{run_dir}`")
    lines.append(f"- judge: `{JUDGE_MODEL}`")
    lines.append(f"- probes scored: {agg['n_probes']}")
    lines.append("")

    # Overall
    overall = agg["overall_by_readout"]
    lines.append("## Overall mean (across all axes, all probes)")
    lines.append("")
    lines.append("| Readout | Mean |")
    lines.append("|---------|------|")
    for r in READOUT_TYPES:
        lines.append(f"| {r} | {overall.get(r, '—')} |")
    lines.append("")

    # Matrix
    lines.append("## Per-tier × per-axis × per-readout mean")
    lines.append("")
    matrix = agg["matrix"]
    if not matrix:
        lines.append("(no scored axes)")
    else:
        all_axes = sorted({a for t in matrix for a in matrix[t]})
        header = "| Tier | " + " | ".join(
            f"{a} (slash)" for a in all_axes) + " | " + " | ".join(
            f"{a} (agent)" for a in all_axes) + " |"
        lines.append(header)
        sep = "|------|" + "|".join(["----"] * (2 * len(all_axes))) + "|"
        lines.append(sep)
        for tier in sorted(matrix.keys()):
            row = [tier or "—"]
            for axis in all_axes:
                v = matrix[tier].get(axis, {}).get("slash")
                row.append("—" if v is None else f"{v}")
            for axis in all_axes:
                v = matrix[tier].get(axis, {}).get("agent")
                row.append("—" if v is None else f"{v}")
            lines.append("| " + " | ".join(row) + " |")
    lines.append("")

    # Divergent probes
    div = agg["divergent_probes"]
    lines.append("## Probes where /remember and agent diverge on verdict")
    lines.append("")
    if not div:
        lines.append("(none — all probes had matching slash/agent verdicts)")
    else:
        lines.append("| Probe | Tier | /remember | agent |")
        lines.append("|-------|------|-----------|-------|")
        for d in div:
            lines.append(
                f"| {d['probe_id']} | {d.get('tier','')} | "
                f"{d['slash_verdict']} | {d['agent_verdict']} |")
    lines.append("")

    # Per-probe detail
    lines.append("## Per-probe scores")
    lines.append("")
    for rec in scored_records:
        pid = rec.get("probe_id", "?")
        tier = rec.get("tier", "")
        sid = rec.get("session_id", "")
        scores = rec.get("scores") or {}
        lines.append(f"### {pid}  (tier {tier or '—'}, session `{sid}`)")
        lines.append("")
        lines.append(f"**Question:** {rec.get('question','')}")
        lines.append("")
        for readout in READOUT_TYPES:
            r = scores.get(readout) or {}
            axes = r.get("axes") or {}
            verdict = r.get("overall_verdict", "")
            rats = r.get("rationales") or {}
            err = r.get("judge_error")
            axis_str = ", ".join(f"{a}={v}" for a, v in axes.items())
            lines.append(f"- **{readout}** [{verdict}] {axis_str}"
                         + (f"  _(judge_error: {err})_" if err else ""))
            for a, rat in rats.items():
                lines.append(f"  - {a}: {rat}")
        lines.append("")

    out = run_dir / "summary.md"
    out.write_text("\n".join(lines))
    return out


# ---------------------------------------------------------------------------
# Run-dir loading
# ---------------------------------------------------------------------------

def _load_raw(run_dir: Path) -> List[Dict[str, Any]]:
    raw_path = run_dir / "raw.jsonl"
    if not raw_path.is_file():
        raise RuntimeError(f"no raw.jsonl in {run_dir}")
    out: List[Dict[str, Any]] = []
    with open(raw_path, "r") as f:
        for line in f:
            s = line.strip()
            if not s:
                continue
            try:
                out.append(json.loads(s))
            except json.JSONDecodeError as e:
                logger.warning(f"skipping malformed jsonl line: {e}")
    return out


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    parser = argparse.ArgumentParser(
        description="Score a finished memory_recall benchmark run with "
                    "Sonnet 4.6 as judge.")
    parser.add_argument("--run-dir", type=Path, required=True,
                        help="Path to the run directory (contains raw.jsonl).")
    parser.add_argument("--judge-model", default=JUDGE_MODEL,
                        help=f"Judge model name (default {JUDGE_MODEL}).")
    parser.add_argument(
        "--extra-axes", nargs="+", default=None,
        help="Extra axis names to score on EVERY probe (in addition to "
             "the per-probe rubric.axes). Useful for adding orthogonal "
             "quality dimensions without editing every probe YAML. "
             f"Recognized: {ALL_AXES}.")
    args = parser.parse_args()

    run_dir = args.run_dir.resolve()
    if not run_dir.is_dir():
        parser.error(f"run dir not found: {run_dir}")
    if not os.environ.get(JUDGE_API_KEY_ENV):
        parser.error(f"{JUDGE_API_KEY_ENV} not set — required for the judge.")

    backend = _ChatBackend(
        server="anthropic",
        model=args.judge_model,
        base_url=ANTHROPIC_BASE_URL,
        is_reasoning=False,
        api_key=JUDGE_API_KEY_ENV,
    )

    raw_records = _load_raw(run_dir)
    # Filter out any subset-error rows (no probe_id, no rubric).
    probe_records = [r for r in raw_records
                     if r.get("probe_id") and r.get("rubric")]
    skipped_subset_errors = [r for r in raw_records if r.get("subset_error")]
    if skipped_subset_errors:
        logger.warning(
            f"skipping {len(skipped_subset_errors)} subset-error row(s); "
            f"see raw.jsonl for details")

    scored_records: List[Dict[str, Any]] = []
    for rec in probe_records:
        pid = rec.get("probe_id")
        logger.info(f"scoring {pid}…")
        per_readout: Dict[str, Any] = {}
        for readout in READOUT_TYPES:
            score = _score_readout(backend, rec, readout,
                                   extra_axes=args.extra_axes)
            per_readout[readout] = {
                "axes": score.axes,
                "rationales": score.rationales,
                "overall_verdict": score.overall_verdict,
                "judge_error": score.judge_error,
                "raw_judge_output": score.raw_judge_output,
            }
            ax = ", ".join(f"{a}={v}" for a, v in score.axes.items())
            logger.info(f"  {pid}/{readout}: [{score.overall_verdict}] {ax}")

        scored = dict(rec)  # shallow copy
        scored["scores"] = per_readout
        scored_records.append(scored)

    # Write scored.jsonl
    scored_path = run_dir / "scored.jsonl"
    with open(scored_path, "w") as f:
        for rec in scored_records:
            f.write(json.dumps(rec, default=str) + "\n")

    # Aggregate + summary
    agg = _aggregate(scored_records)
    summary_path = _write_summary_md(run_dir, scored_records, agg)

    # Aggregate JSON for downstream tooling
    (run_dir / "summary.json").write_text(
        json.dumps(agg, indent=2, default=str))

    print(f"\nscored.jsonl: {scored_path}")
    print(f"summary.md:   {summary_path}")
    print(f"summary.json: {run_dir / 'summary.json'}")
    overall = agg["overall_by_readout"]
    print(f"\nOverall mean: slash={overall.get('slash')}  "
          f"agent={overall.get('agent')}")
    print(f"Divergent probes: {len(agg['divergent_probes'])}")


if __name__ == "__main__":
    main()
