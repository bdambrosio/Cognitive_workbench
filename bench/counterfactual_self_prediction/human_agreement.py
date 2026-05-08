#!/usr/bin/env python3
"""Counterfactual Self-Prediction Benchmark — human-agreement spot-check.

v0.2 Task 4. Generates a fillable YAML form sampling N pairs from a
scored run-dir. Human fills in `human_predict_bucket` and
`human_enact_bucket` per pair. Re-run with --score to compare human's
bucket assignments against Sonnet's and print agreement metrics.

This calibrates how much we should trust Sonnet's closed-bucket
classification. If human and Sonnet agree most of the time, the
closed-bucket scoring is sound; if they disagree often, the judge
prompt or option_set design needs work.

Usage:
    # Step 1: generate the fillable form (after judge.py has produced scores.json)
    python bench/counterfactual_self_prediction/human_agreement.py \\
        --run-dir bench/counterfactual_self_prediction/results/<stamp>_<scenario>

    # Step 2: edit <run-dir>/human_check.yaml — replace `null` in each
    # `human_predict_bucket` and `human_enact_bucket` field with one
    # option_set entry verbatim, or the literal string "other".

    # Step 3: re-run to score
    python bench/counterfactual_self_prediction/human_agreement.py \\
        --run-dir bench/counterfactual_self_prediction/results/<stamp>_<scenario>

Mode is auto-detected: if human_check.yaml doesn't exist or has no
filled fields, --generate runs; otherwise --score runs. Use --force to
re-generate (overwrites). Multi-run parents are not supported in v0.2 —
score each run-dir individually.
"""

from __future__ import annotations

import argparse
import json
import logging
import random
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml

logger = logging.getLogger("bench.counterfactual_self_prediction.human_agreement")

HUMAN_CHECK_FILE = "human_check.yaml"
HUMAN_AGREEMENT_FILE = "human_agreement.md"


# ---------------------------------------------------------------------------
# Run-dir validation
# ---------------------------------------------------------------------------

def _is_multi_run_parent(d: Path) -> bool:
    idx = d / "run_index.json"
    if not idx.is_file():
        return False
    try:
        data = json.loads(idx.read_text())
    except Exception:
        return False
    return int(data.get("n_runs") or 0) >= 2


def _load_scores_json(run_dir: Path) -> Dict[str, Any]:
    path = run_dir / "scores.json"
    if not path.is_file():
        raise RuntimeError(
            f"{path} not found — run judge.py on this run-dir first."
        )
    with open(path, "r") as f:
        return json.load(f)


def _load_pair_summary(run_dir: Path, pair_id: str) -> Dict[str, Any]:
    path = run_dir / pair_id / "summary.json"
    if not path.is_file():
        raise RuntimeError(f"missing pair summary: {path}")
    with open(path, "r") as f:
        return json.load(f)


# ---------------------------------------------------------------------------
# Generate mode
# ---------------------------------------------------------------------------

def _build_form(run_dir: Path, n_sample: Optional[int],
                seed: Optional[int]) -> Dict[str, Any]:
    """Construct the fillable form dict, sampling n_sample pairs (or all
    if n_sample is None)."""
    scores = _load_scores_json(run_dir)
    per_pair_scores = scores.get("per_pair") or []
    if not per_pair_scores:
        raise RuntimeError(f"no per-pair scores in {run_dir / 'scores.json'}")

    pair_ids = [p["pair_id"] for p in per_pair_scores]
    if n_sample is None or n_sample >= len(pair_ids):
        chosen = pair_ids
    else:
        rng = random.Random(seed)
        chosen = rng.sample(pair_ids, n_sample)

    sonnet_by_pair = {p["pair_id"]: p for p in per_pair_scores}

    form_pairs: List[Dict[str, Any]] = []
    for pid in chosen:
        summary = _load_pair_summary(run_dir, pid)
        sonnet = sonnet_by_pair.get(pid, {})
        form_pairs.append({
            "pair_id": pid,
            "option_set": summary.get("option_set") or [],
            "predict_user_text": summary.get("predict", {}).get("user_text", ""),
            "predict_agent_reply": summary.get("predict", {}).get("agent_reply", ""),
            "enact_user_text": summary.get("enact", {}).get("user_text", ""),
            "enact_agent_reply": summary.get("enact", {}).get("agent_reply", ""),
            "sonnet_predict_bucket": sonnet.get("predict_bucket", ""),
            "sonnet_enact_bucket": sonnet.get("enact_bucket", ""),
            "human_predict_bucket": None,   # ← edit me
            "human_enact_bucket": None,     # ← edit me
            "human_notes": "",
        })

    return {
        "_instructions": (
            "For each pair, set `human_predict_bucket` and "
            "`human_enact_bucket` to one of the option_set entries "
            "(verbatim) or the literal string \"other\". Then re-run "
            "human_agreement.py with the same --run-dir to score."
        ),
        "run_dir": str(run_dir),
        "n_pairs_in_form": len(form_pairs),
        "pairs": form_pairs,
    }


def _generate_form(run_dir: Path, n_sample: Optional[int],
                   seed: Optional[int], force: bool) -> Path:
    out = run_dir / HUMAN_CHECK_FILE
    if out.exists() and not force:
        raise RuntimeError(
            f"{out} already exists — use --force to overwrite, or run "
            "without --generate to score the existing form."
        )
    form = _build_form(run_dir, n_sample, seed)
    with open(out, "w") as f:
        yaml.safe_dump(form, f, sort_keys=False, allow_unicode=True,
                       default_flow_style=False, width=100)
    logger.info(f"wrote fillable form: {out}")
    n = form.get("n_pairs_in_form", 0)
    print(f"\nGenerated form for {n} pair(s).")
    print(f"  → edit {out}")
    print( "    set human_predict_bucket and human_enact_bucket per pair, "
           "then re-run this command with the same --run-dir.")
    return out


# ---------------------------------------------------------------------------
# Score mode
# ---------------------------------------------------------------------------

def _form_has_any_filled(form: Dict[str, Any]) -> bool:
    for p in (form.get("pairs") or []):
        if p.get("human_predict_bucket") is not None:
            return True
        if p.get("human_enact_bucket") is not None:
            return True
    return False


def _bucket_eq(a: str, b: str) -> bool:
    """Compare buckets after normalizing whitespace. Sonnet sometimes
    drops trailing punctuation or normalizes spacing; humans copying
    from option_set may add/strip whitespace too."""
    return " ".join((a or "").split()).strip() == " ".join((b or "").split()).strip()


def _score_form(run_dir: Path) -> Path:
    path = run_dir / HUMAN_CHECK_FILE
    if not path.is_file():
        raise RuntimeError(
            f"{path} not found — generate it first by running with no flags "
            "(or --generate)."
        )
    with open(path, "r") as f:
        form = yaml.safe_load(f)

    pairs = form.get("pairs") or []
    if not pairs:
        raise RuntimeError(f"{path} has no pairs section")

    rows: List[Dict[str, Any]] = []
    n_predict_filled = 0
    n_enact_filled = 0
    n_predict_agree = 0
    n_enact_agree = 0
    n_both_agree = 0
    n_both_filled = 0

    for p in pairs:
        pid = p.get("pair_id", "?")
        sonnet_pred = p.get("sonnet_predict_bucket", "") or ""
        sonnet_enact = p.get("sonnet_enact_bucket", "") or ""
        human_pred = p.get("human_predict_bucket")
        human_enact = p.get("human_enact_bucket")

        pred_filled = human_pred is not None and str(human_pred).strip() != ""
        enact_filled = human_enact is not None and str(human_enact).strip() != ""
        if pred_filled:
            n_predict_filled += 1
        if enact_filled:
            n_enact_filled += 1

        pred_agree = pred_filled and _bucket_eq(human_pred, sonnet_pred)
        enact_agree = enact_filled and _bucket_eq(human_enact, sonnet_enact)
        if pred_agree:
            n_predict_agree += 1
        if enact_agree:
            n_enact_agree += 1
        if pred_filled and enact_filled:
            n_both_filled += 1
            if pred_agree and enact_agree:
                n_both_agree += 1

        rows.append({
            "pair_id": pid,
            "sonnet_predict": sonnet_pred,
            "sonnet_enact": sonnet_enact,
            "human_predict": human_pred,
            "human_enact": human_enact,
            "predict_agree": pred_agree if pred_filled else None,
            "enact_agree": enact_agree if enact_filled else None,
            "human_notes": p.get("human_notes", ""),
        })

    # Build report
    lines: List[str] = []
    lines.append("# Counterfactual Self-Prediction — human / Sonnet bucket-assignment agreement")
    lines.append("")
    lines.append(f"- run dir: `{run_dir}`")
    lines.append(f"- form: `{path.name}`")
    lines.append(f"- pairs in form: **{len(pairs)}**")
    lines.append(f"- predict-bucket fields filled: {n_predict_filled}")
    lines.append(f"- enact-bucket fields filled: {n_enact_filled}")
    if n_predict_filled:
        pct = 100.0 * n_predict_agree / n_predict_filled
        lines.append(f"- **predict-bucket agreement**: {n_predict_agree}/{n_predict_filled} ({pct:.1f}%)")
    if n_enact_filled:
        pct = 100.0 * n_enact_agree / n_enact_filled
        lines.append(f"- **enact-bucket agreement**: {n_enact_agree}/{n_enact_filled} ({pct:.1f}%)")
    if n_both_filled:
        pct = 100.0 * n_both_agree / n_both_filled
        lines.append(f"- both-axes agreement (pair-level): {n_both_agree}/{n_both_filled} ({pct:.1f}%)")
    lines.append("")

    lines.append("## Per-pair")
    lines.append("")
    lines.append("| Pair | Predict (human / Sonnet) | Enact (human / Sonnet) | Pred agree | Enact agree |")
    lines.append("|------|--------------------------|-------------------------|------------|-------------|")
    for r in rows:
        def cap(s: Any, n: int) -> str:
            s = str(s) if s is not None else "—"
            return s[:n] + ("…" if len(s) > n else "")
        pa = ("✓" if r["predict_agree"] else
              ("✗" if r["predict_agree"] is False else "—"))
        ea = ("✓" if r["enact_agree"] else
              ("✗" if r["enact_agree"] is False else "—"))
        lines.append(
            f"| {r['pair_id']} "
            f"| {cap(r['human_predict'], 35)} / {cap(r['sonnet_predict'], 35)} "
            f"| {cap(r['human_enact'], 35)}  / {cap(r['sonnet_enact'], 35)} "
            f"| {pa} | {ea} |"
        )
    lines.append("")

    disagreements = [r for r in rows
                     if r["predict_agree"] is False or r["enact_agree"] is False]
    if disagreements:
        lines.append("## Disagreements (worth a closer look)")
        lines.append("")
        for r in disagreements:
            lines.append(f"### {r['pair_id']}")
            if r["predict_agree"] is False:
                lines.append("- **Predict bucket disagreement.**")
                lines.append(f"  - human: {r['human_predict']}")
                lines.append(f"  - sonnet: {r['sonnet_predict']}")
            if r["enact_agree"] is False:
                lines.append("- **Enact bucket disagreement.**")
                lines.append(f"  - human: {r['human_enact']}")
                lines.append(f"  - sonnet: {r['sonnet_enact']}")
            if r.get("human_notes"):
                lines.append(f"- human notes: {r['human_notes']}")
            lines.append("")

    out = run_dir / HUMAN_AGREEMENT_FILE
    out.write_text("\n".join(lines))
    logger.info(f"wrote agreement report: {out}")

    # Stdout summary
    print()
    print(f"Pairs scored: {len(pairs)}")
    if n_predict_filled:
        print(f"  predict-bucket agreement: {n_predict_agree}/{n_predict_filled} "
              f"({100.0 * n_predict_agree / n_predict_filled:.1f}%)")
    if n_enact_filled:
        print(f"  enact-bucket agreement:   {n_enact_agree}/{n_enact_filled} "
              f"({100.0 * n_enact_agree / n_enact_filled:.1f}%)")
    print(f"\nReport: {out}")
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
        description="Generate or score a human-agreement spot-check form "
                    "for a counterfactual-self-prediction run.")
    parser.add_argument("--run-dir", type=Path, required=True,
                        help="Path to a single run dir (must contain scores.json). "
                             "Multi-run parents not supported in v0.2 — score "
                             "each run-dir individually.")
    parser.add_argument("--generate", action="store_true",
                        help="Force generate-mode (overwrites only with --force).")
    parser.add_argument("--score", action="store_true",
                        help="Force score-mode.")
    parser.add_argument("--n", type=int, default=2,
                        help="Number of pairs to sample for the form "
                             "(default 2). Ignored with --all.")
    parser.add_argument("--all", action="store_true",
                        help="Include all pairs in the form (default is to "
                             "sample --n at random).")
    parser.add_argument("--seed", type=int, default=None,
                        help="Random seed for sampling (reproducibility).")
    parser.add_argument("--force", action="store_true",
                        help="Allow overwriting existing human_check.yaml on generate.")
    args = parser.parse_args()

    run_dir = args.run_dir.resolve()
    if not run_dir.is_dir():
        parser.error(f"run dir not found: {run_dir}")

    if _is_multi_run_parent(run_dir):
        parser.error(
            "multi-run parent dirs are not supported in v0.2. Pass a "
            "single run-dir (e.g. <parent>/run-01)."
        )

    if args.generate and args.score:
        parser.error("--generate and --score are mutually exclusive")

    n_sample: Optional[int] = None if args.all else args.n

    # Auto-detect mode if neither flag given.
    form_path = run_dir / HUMAN_CHECK_FILE
    if args.generate:
        _generate_form(run_dir, n_sample, args.seed, args.force)
        return
    if args.score:
        _score_form(run_dir)
        return

    if not form_path.exists():
        _generate_form(run_dir, n_sample, args.seed, args.force)
        return

    # Form exists. If it has any filled human fields, score; else suggest filling.
    try:
        with open(form_path, "r") as f:
            existing = yaml.safe_load(f)
    except Exception as e:
        parser.error(f"failed to load existing {form_path}: {e}")

    if _form_has_any_filled(existing):
        _score_form(run_dir)
    else:
        print(f"{form_path} exists but has no filled human_*_bucket fields.")
        print("Edit the file (set human_predict_bucket and human_enact_bucket "
              "per pair), then re-run.")


if __name__ == "__main__":
    main()
