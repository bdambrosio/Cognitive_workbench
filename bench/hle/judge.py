#!/usr/bin/env python3
"""HLE benchmark — judge.

Scores raw.jsonl from bench/hle/runner.py with Claude Sonnet 4.6. One
verdict per question: did the model's reply correctly answer the
question, given the gold answer? Single axis (correctness), three-way
verdict (correct / partial / incorrect) plus a one-sentence rationale.

Output:
    <run-dir>/scored.jsonl   — raw row + judge verdict per question
    <run-dir>/summary.md     — overall accuracy + per-category breakdown
    <run-dir>/summary.json   — same numbers, machine-readable

Usage:
    python bench/hle/judge.py --run-dir bench/hle/results/<timestamp>_<scenario>

Requires CLAUDE_API_KEY env var.
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import sys
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from chat.chat_loop import _ChatBackend  # noqa: E402
from utils.json_utils import repair_json_string  # noqa: E402

logger = logging.getLogger("bench.hle.judge")

JUDGE_MODEL = "claude-sonnet-4-6"
JUDGE_API_KEY_ENV = "CLAUDE_API_KEY"
ANTHROPIC_BASE_URL = "https://api.anthropic.com"
# Extra samples on judge API failure or unparseable output
# (availability retry; rubric unchanged).
_JUDGE_RETRIES = 2

VERDICTS = ("correct", "partial", "incorrect")


@dataclass
class Verdict:
    verdict: str           # "correct" | "partial" | "incorrect"
    score: float           # 1.0 | 0.5 | 0.0
    rationale: str
    raw_judge_output: Optional[str] = None
    judge_error: Optional[str] = None


_SYS_MSG = (
    "You are an independent judge scoring answers on the HLE benchmark "
    "(Humanity's Last Exam — open-ended research questions across many "
    "fields). For each item you receive the question, the gold answer, "
    "and a model's reply. Return a JSON verdict.\n\n"
    "Scoring rules:\n"
    "- correct: the reply gives the right answer. Paraphrase, extra "
    "context, or different formatting is fine if the core answer is "
    "right and unambiguous.\n"
    "- partial: the reply gets something substantive right but is "
    "incomplete, hedged into ambiguity, or wrong in a load-bearing "
    "subcomponent.\n"
    "- incorrect: the reply gives a wrong answer, refuses, or is "
    "unrelated to the question.\n\n"
    "Be strict about correctness — for technical/scientific questions, "
    "a near-miss is incorrect, not partial. Reserve 'partial' for cases "
    "where the model demonstrably did some of the work but missed part "
    "of the answer.\n\n"
    "CRITICAL: Do NOT attempt to solve, re-derive, or verify the problem "
    "yourself. You are given the gold answer — treat it as ground truth "
    "and judge ONLY whether the reply matches it. Show no working; emit "
    "the JSON verdict and nothing else."
)


def _build_judge_messages(question: str, gold: str,
                          reply: str) -> List[Dict[str, str]]:
    user = (
        f"# Question\n{question}\n\n"
        f"# Gold answer\n{gold}\n\n"
        f"# Model reply\n{reply or '(empty)'}\n\n"
        f"# Output\n"
        "Respond with ONLY the JSON object below — the very first character "
        "of your response must be '{', with no analysis, prose, or markdown "
        "fences before or after it:\n"
        "{\n"
        '  "verdict": "correct" | "partial" | "incorrect",\n'
        '  "rationale": "<one sentence, no derivation>"\n'
        "}\n"
    )
    return [{"role": "system", "content": _SYS_MSG},
            {"role": "user", "content": user}]


def _coerce_verdict(raw: Any) -> str:
    v = str(raw or "").strip().lower()
    if v in VERDICTS:
        return v
    if v.startswith("correct"):
        return "correct"
    if v.startswith("partial"):
        return "partial"
    return "incorrect"


def _verdict_to_score(v: str) -> float:
    return {"correct": 1.0, "partial": 0.5, "incorrect": 0.0}.get(v, 0.0)


def _score_one(backend: _ChatBackend, record: Dict[str, Any]) -> Verdict:
    # Run-time errors short-circuit to incorrect with the error in the rationale.
    if record.get("error"):
        return Verdict(
            verdict="incorrect",
            score=0.0,
            rationale=f"runner errored: {record['error']}",
            judge_error=None,
        )
    question = record.get("question") or ""
    gold = record.get("gold") or ""
    reply = record.get("reply") or ""

    messages = _build_judge_messages(question, gold, reply)
    # Availability retry (rubric unchanged): transient API failures
    # (ReadTimeout at 120s observed live) and unparseable output get a
    # bounded resample before force-scoring — a single flake otherwise
    # shrinks the denominator and breaks run-to-run comparability.
    raw: Any = None
    last_err: Optional[str] = None
    parsed: Any = None
    for attempt in range(1 + _JUDGE_RETRIES):
        try:
            # Generous cap + the "don't re-derive" system rule keep the
            # judge from spending its whole budget reasoning out the
            # problem and never reaching the JSON verdict — the failure
            # mode that force-scored hard math/physics items as
            # unparseable=incorrect.
            raw = backend.chat(messages, max_tokens=1024, temperature=0.0)
        except Exception as e:
            logger.exception(
                f"{record.get('question_id')}: judge call failed "
                f"(attempt {attempt + 1})")
            last_err = f"{type(e).__name__}: {e}"
            continue
        try:
            parsed = json.loads(raw)
        except json.JSONDecodeError:
            parsed = repair_json_string(raw)
        # Fallback: if the judge prefaced the verdict with prose despite
        # the instructions, extract the outermost {...} and retry the parse.
        if not isinstance(parsed, dict) and isinstance(raw, str):
            start, end = raw.find("{"), raw.rfind("}")
            if 0 <= start < end:
                snippet = raw[start:end + 1]
                try:
                    parsed = json.loads(snippet)
                except json.JSONDecodeError:
                    parsed = repair_json_string(snippet)
        if isinstance(parsed, dict):
            break
        logger.warning(
            f"{record.get('question_id')}: judge output unparseable "
            f"(attempt {attempt + 1}); head: {str(raw)[:300]!r}")
        last_err = "unparseable"

    if not isinstance(parsed, dict):
        if raw is None:
            return Verdict(
                verdict="incorrect", score=0.0,
                rationale="judge call failed",
                judge_error=last_err,
            )
        return Verdict(
            verdict="incorrect", score=0.0,
            rationale="judge output unparseable",
            judge_error="unparseable",
            raw_judge_output=str(raw)[:2000],
        )

    verdict = _coerce_verdict(parsed.get("verdict"))
    rationale = str(parsed.get("rationale", "")).strip()
    return Verdict(
        verdict=verdict,
        score=_verdict_to_score(verdict),
        rationale=rationale,
    )


# ---------------------------------------------------------------------------
# Aggregation + report
# ---------------------------------------------------------------------------

def _aggregate(scored: List[Dict[str, Any]]) -> Dict[str, Any]:
    by_cat: Dict[str, Dict[str, Any]] = defaultdict(
        lambda: {"correct": 0, "partial": 0, "incorrect": 0,
                 "score_sum": 0.0, "n": 0})
    overall = {"correct": 0, "partial": 0, "incorrect": 0,
               "score_sum": 0.0, "n": 0}
    judge_errors = 0
    for r in scored:
        j = r.get("judge") or {}
        # A judge-side failure (unparseable output / API error) is NOT a
        # model wrong-answer — exclude it from the score so it can't
        # masquerade as model-incorrect (and so judge flakiness doesn't
        # differ silently between a baseline and an FP8 run). Runner errors
        # (judge_error is None, verdict incorrect) are real model failures
        # and stay counted.
        if j.get("judge_error"):
            judge_errors += 1
            continue
        v = j.get("verdict", "incorrect")
        s = float(j.get("score", 0.0))
        cat = r.get("category") or "(uncategorized)"
        for bucket in (by_cat[cat], overall):
            bucket[v] = bucket.get(v, 0) + 1
            bucket["score_sum"] += s
            bucket["n"] += 1
    per_cat = {}
    for cat, b in sorted(by_cat.items()):
        n = b["n"] or 1
        per_cat[cat] = {
            "n": b["n"],
            "correct": b["correct"],
            "partial": b["partial"],
            "incorrect": b["incorrect"],
            "score_mean": round(b["score_sum"] / n, 4),
            "exact_correct_rate": round(b["correct"] / n, 4),
        }
    n = overall["n"] or 1
    return {
        "overall": {
            "n": overall["n"],
            "correct": overall["correct"],
            "partial": overall["partial"],
            "incorrect": overall["incorrect"],
            "score_mean": round(overall["score_sum"] / n, 4),
            "exact_correct_rate": round(overall["correct"] / n, 4),
        },
        "judge_errors": judge_errors,
        "per_category": per_cat,
    }


def _write_summary_md(run_dir: Path, scored: List[Dict[str, Any]],
                      agg: Dict[str, Any]) -> Path:
    lines: List[str] = []
    lines.append("# HLE benchmark — summary")
    lines.append("")
    o = agg["overall"]
    lines.append(f"- judge: `{JUDGE_MODEL}`")
    lines.append(f"- n scored: {o['n']}")
    lines.append(
        f"- **score_mean** (correct=1, partial=0.5, incorrect=0): "
        f"{o['score_mean']:.4f}")
    lines.append(
        f"- **exact_correct_rate**: {o['exact_correct_rate']:.4f} "
        f"({o['correct']}/{o['n']})")
    lines.append(
        f"- partial: {o['partial']}  incorrect: {o['incorrect']}")
    je = agg.get("judge_errors", 0)
    if je:
        lines.append(
            f"- judge_errors (excluded from score, NOT model-incorrect): {je}")
    lines.append("")
    if agg["per_category"]:
        lines.append("## Per-category")
        lines.append("")
        lines.append("| category | n | correct | partial | incorrect | "
                     "score_mean | exact_rate |")
        lines.append("|---|---:|---:|---:|---:|---:|---:|")
        for cat, c in agg["per_category"].items():
            lines.append(
                f"| {cat} | {c['n']} | {c['correct']} | {c['partial']} | "
                f"{c['incorrect']} | {c['score_mean']:.4f} | "
                f"{c['exact_correct_rate']:.4f} |")
        lines.append("")
    lines.append("## Sample incorrect (first 10)")
    lines.append("")
    shown = 0
    for r in scored:
        if shown >= 10:
            break
        v = (r.get("judge") or {})
        if v.get("verdict") != "incorrect":
            continue
        lines.append(f"### {r.get('question_id')} ({r.get('category', '')})")
        lines.append("")
        lines.append(f"**Q:** {r.get('question', '')[:300]}")
        lines.append("")
        lines.append(f"**Gold:** {r.get('gold', '')[:300]}")
        lines.append("")
        lines.append(f"**Reply:** {(r.get('reply') or '')[:500]}")
        lines.append("")
        lines.append(f"_rationale: {v.get('rationale', '')}_")
        lines.append("")
        shown += 1
    out = run_dir / "summary.md"
    out.write_text("\n".join(lines))
    return out


# ---------------------------------------------------------------------------
# I/O
# ---------------------------------------------------------------------------

def _load_raw(run_dir: Path) -> List[Dict[str, Any]]:
    raw_path = run_dir / "raw.jsonl"
    if not raw_path.is_file():
        raise RuntimeError(f"raw.jsonl not found in {run_dir}")
    out: List[Dict[str, Any]] = []
    with open(raw_path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                out.append(json.loads(line))
            except json.JSONDecodeError as e:
                logger.warning(f"skipping malformed jsonl line: {e}")
    return out


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    parser = argparse.ArgumentParser(
        description="Score a finished HLE benchmark run with Sonnet 4.6 "
                    "as judge.")
    parser.add_argument("--run-dir", type=Path, required=True,
                        help="Path to the run directory (contains raw.jsonl).")
    parser.add_argument("--judge-model", default=JUDGE_MODEL,
                        help=f"Judge model name (default {JUDGE_MODEL}).")
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
    scored: List[Dict[str, Any]] = []
    for rec in raw_records:
        qid = rec.get("question_id") or rec.get("question_index")
        logger.info(f"scoring {qid}…")
        v = _score_one(backend, rec)
        out = dict(rec)
        out["judge"] = {
            "verdict": v.verdict,
            "score": v.score,
            "rationale": v.rationale,
            "judge_error": v.judge_error,
            "raw_judge_output": v.raw_judge_output,
        }
        scored.append(out)
        logger.info(f"  {qid}: [{v.verdict}] {v.rationale[:120]}")

    scored_path = run_dir / "scored.jsonl"
    with open(scored_path, "w") as f:
        for r in scored:
            f.write(json.dumps(r, default=str) + "\n")

    agg = _aggregate(scored)
    summary_path = _write_summary_md(run_dir, scored, agg)
    (run_dir / "summary.json").write_text(
        json.dumps(agg, indent=2, default=str))

    o = agg["overall"]
    je = agg.get("judge_errors", 0)
    print(f"\nscored.jsonl: {scored_path}")
    print(f"summary.md:   {summary_path}")
    print(f"summary.json: {run_dir / 'summary.json'}")
    print(
        f"\nOverall: score_mean={o['score_mean']:.4f}  "
        f"exact_correct={o['exact_correct_rate']:.4f} "
        f"({o['correct']}/{o['n']})"
        + (f"  |  judge_errors (excluded): {je}" if je else ""))


if __name__ == "__main__":
    main()
