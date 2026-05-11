#!/usr/bin/env python3
"""Judge: score discourse-reflection production outputs against gold.

For each pair_NN.json, run a Sonnet judge that compares
`production_new_discourse_state` to the gold annotations on six axes:

  add-recall        — of gold should_add_*, what fraction appear in production?
  add-precision     — of new-vs-prior items in production, what fraction are dialog-grounded?
  update-correctness — did gold should_modify items get the right modification?
  carryover-preservation — did gold should_carry_over_unchanged items survive intact?
  prune-correctness — did gold should_remove items get removed?
  wording           — are written items concise, faithful, self-contained?

Output: scores/judgement_NN.json per pair plus an aggregate summary
printed at end of run.

Caveat: the same model (Sonnet) wrote the gold AND scores against it.
Some self-anchoring is unavoidable in v0.1; calibration against human
ratings on a small subset is the next step. Judge here is asked to be
mechanical (does production contain X? not "should production contain X?")
to minimize that bias.

Defaults score against the *historical* production_new_discourse_state
already captured in each pair file. To score a re-run baseline (e.g.,
current prompt + current backend), pass --output-field naming the
field where the alternative output was written.

Usage:
    CLAUDE_API_KEY=... python bench/discourse_reflect/judge.py \\
        --pairs-dir bench/discourse_reflect/pairs \\
        --scores-dir bench/discourse_reflect/scores \\
        --model claude-sonnet-4-6
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import re
import sys
from datetime import datetime, timezone
from pathlib import Path
from statistics import mean
from typing import Any, Dict, List, Optional

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent.parent
SRC_DIR = REPO_ROOT / "src"
sys.path.insert(0, str(SRC_DIR))

from chat.chat_loop import _ChatBackend  # noqa: E402

logger = logging.getLogger("bench.discourse_reflect.judge")

DEFAULT_MODEL = "claude-sonnet-4-6"
ANTHROPIC_BASE_URL = "https://api.anthropic.com"
API_KEY_ENV = "CLAUDE_API_KEY"
PROMPT_VERSION = "v1"

AXES = (
    "add_recall",
    "add_precision",
    "update_correctness",
    "carryover_preservation",
    "prune_correctness",
    "wording",
)

JUDGE_SYSTEM = """\
You are scoring the output of a discourse-extraction system against a \
gold-annotated test case. The system was given a prior discourse state \
and a dialog window; it produced an updated discourse state. A separate \
labelling pass produced gold annotations describing what the correct \
update should contain.

You will compare the system's output (PRODUCTION NEW STATE) against the \
gold and score it on six axes. BE MECHANICAL. Your job is "does \
production contain X?", not "should production contain X?". The gold \
defines what should be present; you check whether it is.

Semantic matching: an item counts as present if production contains a \
sentence (or bullet) carrying the same load-bearing semantic content. \
Wording can differ, qualifiers can be re-ordered. But missing key \
qualifiers, narrowed scope, or a substantively different claim is NOT \
a match.

OUTPUT FORMAT — emit ONLY this JSON object, nothing else:

{
  "add_recall": {
    "items": [
      {"gold_item": "<paraphrase from gold>",
       "match": "yes|partial|no",
       "production_quote": "<verbatim or close paraphrase from production, or empty if no>",
       "note": "<short rationale, e.g. 'missing the key-qualifier X'>"}
    ],
    "score": <float in [0,1] or null if gold has no should_add_* items>,
    "score_basis": "<short explanation, e.g. '3 yes + 1 partial(0.5) / 5 gold = 0.70'>"
  },
  "add_precision": {
    "items": [
      {"production_item": "<paraphrase of an item in production new but not in prior>",
       "grounded": "yes|partial|no",
       "dialog_quote": "<quote or close paraphrase from dialog, or empty>",
       "note": "<short rationale>"}
    ],
    "score": <float in [0,1] or null if production has no new-vs-prior items>,
    "score_basis": "<short explanation>"
  },
  "update_correctness": {
    "items": [
      {"gold_modify_target": "<paraphrase of the prior item gold says should be modified>",
       "gold_modification": "<the modification gold prescribes>",
       "production_handling": "modified_correctly|partially_modified|unchanged|dropped",
       "note": "<short rationale>"}
    ],
    "score": <float in [0,1] or null if gold has no should_modify items>,
    "score_basis": "<short explanation>"
  },
  "carryover_preservation": {
    "items": [
      {"gold_carryover_item": "<paraphrase of the prior item gold says should be preserved>",
       "production_handling": "preserved|re_worded_acceptably|dropped|mangled",
       "note": "<short rationale>"}
    ],
    "score": <float in [0,1] or null if gold has no should_carry_over_unchanged items>,
    "score_basis": "<short explanation>"
  },
  "prune_correctness": {
    "items": [
      {"gold_remove_item": "<paraphrase of the prior item gold says should be removed>",
       "production_handling": "removed|still_present",
       "note": "<short rationale>"}
    ],
    "score": <float in [0,1] or null if gold has no should_remove items>,
    "score_basis": "<short explanation>"
  },
  "wording": {
    "sample_items": [
      {"production_item": "<verbatim item from production new state>",
       "score_0_3": <integer 0-3>,
       "note": "<short rationale: terseness, faithfulness to dialog, self-containment>"}
    ],
    "score": <float in [0,1] (mean of sample_items / 3) or null if no items to sample>,
    "score_basis": "<short explanation>"
  },
  "overall_note": "<2-4 sentences summarizing the production output's update quality>"
}

Scoring conventions:
- match=yes scores 1.0; match=partial scores 0.5; match=no scores 0.0.
- production_handling=modified_correctly or preserved or removed scores 1.0.
- production_handling=partially_modified or re_worded_acceptably scores 0.5.
- production_handling=unchanged, dropped, mangled, still_present scores 0.0.
- For wording, sample 3-5 representative items from production new state \
(not all of them); score each 0-3 (3=ideal, 0=incoherent or fabricated); \
report the mean / 3 as the axis score.
- score=null when the axis is N/A for this pair (gold list empty for the \
axis it depends on, or no production items to evaluate). DO NOT score 1.0 \
in that case.
"""

JUDGE_USER_TEMPLATE = """\
PRIOR DISCOURSE STATE (what the system saw as input):
=========================
{prior_state}
=========================

DIALOG WINDOW (the conversation context the system saw):
=========================
{dialog_block}
=========================

GOLD ANNOTATIONS (what the correct update should contain):
=========================
{gold_block}
=========================

PRODUCTION NEW STATE (what the system actually produced — score this):
=========================
{production_new_state}
=========================

Score the PRODUCTION NEW STATE against the GOLD on all six axes. Emit \
the JSON object now.\
"""


def _format_dialog(dialog_window: List[Dict[str, str]]) -> str:
    if not dialog_window:
        return "(empty)"
    lines: List[str] = []
    for entry in dialog_window:
        src = entry.get("source", "?")
        text = (entry.get("text") or "").strip()
        if text:
            lines.append(f"[{src}] {text}")
    return "\n\n".join(lines)


def _format_gold(gold: Dict[str, Any]) -> str:
    """Render gold annotations as a compact human-readable block for
    the judge prompt. JSON-style dump is fine and keeps structure
    explicit — gold dicts are small."""
    return json.dumps(gold, indent=2, ensure_ascii=False)


def _strip_code_fences(s: str) -> str:
    s = s.strip()
    if s.startswith("```"):
        s = re.sub(r"^```(?:json)?\s*", "", s)
        s = re.sub(r"\s*```\s*$", "", s)
    return s.strip()


def _parse_judgement_json(raw: str) -> Optional[Dict[str, Any]]:
    raw = _strip_code_fences(raw)
    try:
        return json.loads(raw)
    except json.JSONDecodeError as e:
        logger.warning(f"json parse failed: {e}; raw head: {raw[:200]!r}")
        return None


def _validate_axes_present(j: Dict[str, Any]) -> List[str]:
    """Return list of missing axis keys; empty list if all present."""
    return [a for a in AXES if a not in j]


def _judge_one(backend: _ChatBackend, pair: Dict[str, Any],
               output_field: str) -> Optional[Dict[str, Any]]:
    """Run one judge call. Returns the judgement dict on success,
    None on failure."""
    production = pair.get(output_field) or ""
    if not production.strip():
        logger.warning(f"  pair has empty {output_field!r}; skipping")
        return None
    user_msg = JUDGE_USER_TEMPLATE.format(
        prior_state=pair["prior_discourse_state"],
        dialog_block=_format_dialog(pair["dialog_window"]),
        gold_block=_format_gold(pair["gold"]),
        production_new_state=production,
    )
    messages = [
        {"role": "system", "content": JUDGE_SYSTEM},
        {"role": "user", "content": user_msg},
    ]
    try:
        raw = backend.chat(messages, max_tokens=8000, temperature=0.2)
    except Exception as e:
        logger.warning(f"backend.chat raised: {e}")
        return None
    j = _parse_judgement_json(raw)
    if j is None:
        return None
    missing = _validate_axes_present(j)
    if missing:
        logger.warning(f"judgement missing axes: {missing}")
        return None
    return j


def _aggregate(judgements: List[Dict[str, Any]]) -> Dict[str, Any]:
    """Per-axis: mean of non-null per-pair scores + count of non-null
    pairs (denominator visible)."""
    out: Dict[str, Any] = {}
    for axis in AXES:
        per_pair: List[float] = []
        for j in judgements:
            block = j.get(axis) or {}
            s = block.get("score")
            if s is None:
                continue
            try:
                per_pair.append(float(s))
            except (TypeError, ValueError):
                continue
        out[axis] = {
            "mean": round(mean(per_pair), 3) if per_pair else None,
            "n_scored_pairs": len(per_pair),
            "n_total_pairs": len(judgements),
        }
    return out


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    parser = argparse.ArgumentParser(
        description="Score discourse-reflection production outputs against gold.")
    parser.add_argument("--pairs-dir", type=Path, required=True)
    parser.add_argument("--scores-dir", type=Path, required=True)
    parser.add_argument("--model", type=str, default=DEFAULT_MODEL)
    parser.add_argument("--output-field", type=str,
                        default="production_new_discourse_state",
                        help="Pair-file field naming the production "
                             "output to judge. Default scores the "
                             "historical capture; use a different "
                             "field name to score a re-run baseline.")
    parser.add_argument("--limit", type=int, default=0,
                        help="Process at most N pairs (0 = all).")
    parser.add_argument("--skip-existing", action="store_true",
                        help="Skip pairs whose judgement file already exists.")
    args = parser.parse_args()

    pairs_dir = args.pairs_dir.resolve()
    scores_dir = args.scores_dir.resolve()
    if not pairs_dir.is_dir():
        parser.error(f"pairs dir not found: {pairs_dir}")
    if not os.environ.get(API_KEY_ENV):
        parser.error(f"{API_KEY_ENV} not set — required for the judge.")
    scores_dir.mkdir(parents=True, exist_ok=True)

    pair_paths = sorted(pairs_dir.glob("pair_*.json"))
    if args.limit > 0:
        pair_paths = pair_paths[:args.limit]
    logger.info(f"judging {len(pair_paths)} pairs against field "
                f"{args.output_field!r} with {args.model}")

    backend = _ChatBackend(
        server="anthropic",
        model=args.model,
        base_url=ANTHROPIC_BASE_URL,
        is_reasoning=False,
        api_key=API_KEY_ENV,
    )

    n_done = 0
    n_skipped = 0
    n_failed = 0
    failed_ids: List[str] = []
    judgements: List[Dict[str, Any]] = []
    for path in pair_paths:
        pair = json.loads(path.read_text(encoding="utf-8"))
        pid = pair.get("id", path.stem)
        out_path = scores_dir / f"judgement_{pid.split('_')[-1]}.json"

        if args.skip_existing and out_path.exists():
            n_skipped += 1
            try:
                judgements.append(json.loads(out_path.read_text(encoding="utf-8"))["judgement"])
            except Exception as e:
                logger.warning(f"  {pid}: skip but failed to load existing: {e}")
            logger.info(f"  {pid}: skip (existing judgement)")
            continue

        if not pair.get("gold"):
            logger.warning(f"  {pid}: no gold; skipping")
            n_skipped += 1
            continue

        logger.info(f"  {pid}: judging...")
        judgement = _judge_one(backend, pair, args.output_field)
        if judgement is None:
            n_failed += 1
            failed_ids.append(pid)
            logger.warning(f"  {pid}: FAILED (no judgement written)")
            continue

        record = {
            "pair_id": pid,
            "judgement": judgement,
            "judge_provenance": {
                "model": args.model,
                "prompt_version": PROMPT_VERSION,
                "scored_field": args.output_field,
                "ts": datetime.now(timezone.utc).isoformat(),
            },
        }
        out_path.write_text(
            json.dumps(record, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )
        judgements.append(judgement)
        n_done += 1

        # One-line per-pair summary.
        scores = []
        for axis in AXES:
            s = (judgement.get(axis) or {}).get("score")
            scores.append(f"{axis[:6]}={s if s is None else f'{s:.2f}'}")
        logger.info(f"  {pid}: " + "  ".join(scores))

    # Aggregate.
    print()
    print(f"=== Judging summary ===")
    print(f"judged:   {n_done}")
    print(f"skipped:  {n_skipped}")
    print(f"failed:   {n_failed}  {failed_ids if failed_ids else ''}")
    print()
    if judgements:
        agg = _aggregate(judgements)
        print(f"=== Per-axis aggregate ({len(judgements)} pair(s) in pool) ===")
        print(f"{'axis':<26}  {'mean':>7}  {'scored':>7}/{'total':<5}")
        print("-" * 52)
        for axis in AXES:
            row = agg[axis]
            mn = row["mean"]
            mn_s = f"{mn:.3f}" if mn is not None else "  N/A"
            print(f"{axis:<26}  {mn_s:>7}  {row['n_scored_pairs']:>7}/{row['n_total_pairs']:<5}")
        agg_path = scores_dir / "aggregate.json"
        agg_path.write_text(
            json.dumps({
                "model": args.model,
                "prompt_version": PROMPT_VERSION,
                "scored_field": args.output_field,
                "ts": datetime.now(timezone.utc).isoformat(),
                "n_pairs_in_pool": len(judgements),
                "axes": agg,
            }, indent=2),
            encoding="utf-8",
        )
        print(f"\naggregate written to {agg_path}")


if __name__ == "__main__":
    main()
