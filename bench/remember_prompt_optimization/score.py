#!/usr/bin/env python3
"""Score every variant subdir under a remember_prompt_optimization run-dir.

Runs `bench/memory_recall/judge.py` per variant (the variant subdirs each
contain a raw.jsonl in the format that judge expects), then aggregates the
per-variant summary.json files into a single compare.md ranking variants
by slash mean and showing per-tier breakdown + per-probe win/loss vs the
baseline.

Usage:
    cd src
    python ../bench/remember_prompt_optimization/score.py \\
        --run-dir ../bench/remember_prompt_optimization/results/<stamp>_<scenario>
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent.parent

logger = logging.getLogger("bench.remember_prompt_optimization.score")

JUDGE_SCRIPT = REPO_ROOT / "bench" / "memory_recall" / "judge.py"
BASELINE_VARIANT = "v0_baseline"


def _variant_subdirs(run_dir: Path) -> List[Path]:
    return sorted(d for d in run_dir.iterdir()
                  if d.is_dir() and (d / "raw.jsonl").is_file())


def _run_judge_for_variant(variant_dir: Path, rescore: bool,
                           extra_axes: Optional[List[str]] = None) -> None:
    summary_path = variant_dir / "summary.json"
    if summary_path.is_file() and not rescore:
        logger.info(f"{variant_dir.name}: summary.json exists, skipping "
                    f"(pass --rescore to force)")
        return
    cmd = [sys.executable, str(JUDGE_SCRIPT), "--run-dir", str(variant_dir)]
    if extra_axes:
        cmd.extend(["--extra-axes", *extra_axes])
    logger.info(f"{variant_dir.name}: running judge…")
    result = subprocess.run(cmd, env=os.environ.copy())
    if result.returncode != 0:
        raise RuntimeError(
            f"judge failed for {variant_dir.name} (exit {result.returncode})")


def _load_summary(variant_dir: Path) -> Dict[str, Any]:
    path = variant_dir / "summary.json"
    if not path.is_file():
        return {}
    try:
        return json.loads(path.read_text())
    except Exception as e:
        logger.warning(f"{variant_dir.name}: bad summary.json ({e})")
        return {}


def _load_scored(variant_dir: Path) -> List[Dict[str, Any]]:
    path = variant_dir / "scored.jsonl"
    if not path.is_file():
        return []
    out: List[Dict[str, Any]] = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                out.append(json.loads(line))
            except Exception:
                continue
    return out


def _per_probe_slash(scored: List[Dict[str, Any]]) -> Dict[str, str]:
    """Map probe_id → slash verdict ('pass' / 'fail' / '?')."""
    out: Dict[str, str] = {}
    for rec in scored:
        pid = rec.get("probe_id") or ""
        verdict = (rec.get("scores", {}).get("slash") or {}).get(
            "overall_verdict") or rec.get("slash_verdict") or "?"
        out[pid] = str(verdict)
    return out


def _write_compare_md(run_dir: Path, summaries: Dict[str, Dict[str, Any]],
                      per_probe: Dict[str, Dict[str, str]]) -> Path:
    """Write compare.md aggregating all variants' results, ranked by overall
    slash mean. Highlights per-probe wins/losses vs v0_baseline."""
    variants = sorted(summaries.keys())
    if BASELINE_VARIANT not in summaries:
        logger.warning(
            f"{BASELINE_VARIANT} not present; per-probe diffs will be omitted")

    overall_rows = []
    for v in variants:
        s = summaries[v]
        ov = s.get("overall_by_readout") or {}
        overall_rows.append((v, ov.get("slash"), ov.get("agent"),
                             s.get("n_probes")))

    # Rank by slash mean (None → -1 so it sorts last but is still listed).
    overall_rows.sort(key=lambda r: (r[1] if r[1] is not None else -1),
                      reverse=True)

    lines: List[str] = []
    lines.append("# Remember-prompt optimization — variant comparison")
    lines.append("")
    lines.append(f"- run dir: `{run_dir}`")
    lines.append(f"- variants: {variants}")
    lines.append("")

    lines.append("## Overall slash mean (ranked)")
    lines.append("")
    lines.append("| Rank | Variant | slash mean | agent mean | n probes |")
    lines.append("|------|---------|------------|------------|----------|")
    baseline_slash = (summaries.get(BASELINE_VARIANT, {}).get(
        "overall_by_readout") or {}).get("slash")
    for i, (v, slash, agent, n) in enumerate(overall_rows, start=1):
        slash_s = f"{slash:.3f}" if slash is not None else "n/a"
        agent_s = f"{agent:.3f}" if agent is not None else "n/a"
        diff_s = ""
        if baseline_slash is not None and slash is not None:
            d = slash - baseline_slash
            diff_s = f" ({d:+.3f} vs baseline)" if v != BASELINE_VARIANT else ""
        lines.append(f"| {i} | `{v}` | {slash_s}{diff_s} | {agent_s} | {n} |")
    lines.append("")

    # Per-tier × per-readout = slash. Show a column per variant.
    lines.append("## Per-tier slash mean (per variant)")
    lines.append("")
    all_tiers = sorted(
        {tier for v in variants for tier in (summaries[v].get("matrix") or {})}
    )
    if all_tiers:
        header = "| Tier | " + " | ".join(f"`{v}`" for v in variants) + " |"
        sep = "|------|" + "|".join(["-----"] * len(variants)) + "|"
        lines.append(header)
        lines.append(sep)
        for tier in all_tiers:
            row = [f"| {tier}"]
            for v in variants:
                matrix = summaries[v].get("matrix") or {}
                tier_data = matrix.get(tier) or {}
                # Average non-None slash entries across axes.
                vals = []
                for axis_name, axis in tier_data.items():
                    val = (axis or {}).get("slash")
                    if val is not None:
                        vals.append(val)
                cell = f"{sum(vals)/len(vals):.2f}" if vals else "—"
                row.append(cell)
            lines.append(" | ".join(row) + " |")
        lines.append("")

    # Per-probe verdict swing vs baseline.
    if BASELINE_VARIANT in per_probe:
        lines.append("## Per-probe verdicts (slash) — gain/loss vs baseline")
        lines.append("")
        baseline_pp = per_probe[BASELINE_VARIANT]
        all_pids = sorted({pid for v in per_probe.values() for pid in v})
        # Filter to probes where any non-baseline variant differs from baseline.
        interesting = []
        for pid in all_pids:
            base_v = baseline_pp.get(pid, "?")
            if any(per_probe[v].get(pid, "?") != base_v
                   for v in variants if v != BASELINE_VARIANT):
                interesting.append(pid)
        if not interesting:
            lines.append("(no per-probe verdict differences across variants)")
            lines.append("")
        else:
            header = "| Probe | " + " | ".join(f"`{v}`" for v in variants) + " |"
            sep = "|-------|" + "|".join(["-----"] * len(variants)) + "|"
            lines.append(header)
            lines.append(sep)
            for pid in interesting:
                row = [f"| {pid}"]
                base_v = baseline_pp.get(pid, "?")
                for v in variants:
                    cell = per_probe[v].get(pid, "?")
                    marker = ""
                    if v != BASELINE_VARIANT:
                        if cell == "pass" and base_v != "pass":
                            marker = " ✓"  # gain
                        elif cell != "pass" and base_v == "pass":
                            marker = " ✗"  # loss
                    row.append(cell + marker)
                lines.append(" | ".join(row) + " |")
            lines.append("")

    out = run_dir / "compare.md"
    out.write_text("\n".join(lines))
    return out


def _write_compare_json(run_dir: Path, summaries: Dict[str, Dict[str, Any]]
                        ) -> Path:
    payload = {
        "run_dir": str(run_dir),
        "variants": sorted(summaries.keys()),
        "per_variant_summary": summaries,
    }
    out = run_dir / "compare.json"
    out.write_text(json.dumps(payload, indent=2, default=str))
    return out


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    parser = argparse.ArgumentParser(
        description="Run memory_recall judge per variant subdir and "
                    "aggregate into compare.md.")
    parser.add_argument("--run-dir", type=Path, required=True,
                        help="Path to the multi-variant run dir (contains "
                             "<variant>/raw.jsonl files).")
    parser.add_argument("--rescore", action="store_true",
                        help="Force rescoring even if summary.json exists.")
    parser.add_argument(
        "--extra-axes", nargs="+", default=None,
        help="Forwarded to memory_recall/judge.py — adds these axes to "
             "every probe's scoring on top of the rubric's declared axes. "
             "Use to add fidelity/parsimony dimensions without editing "
             "probe YAMLs. (Pass --rescore alongside if summary.json "
             "already exists from a prior axis-narrower run.)")
    args = parser.parse_args()

    run_dir = args.run_dir.resolve()
    if not run_dir.is_dir():
        parser.error(f"run dir not found: {run_dir}")

    variants = _variant_subdirs(run_dir)
    if not variants:
        parser.error(f"no variant subdirs with raw.jsonl found in {run_dir}")

    print(f"Variants to score: {[v.name for v in variants]}", flush=True)
    for vd in variants:
        _run_judge_for_variant(vd, rescore=args.rescore,
                               extra_axes=args.extra_axes)

    summaries = {vd.name: _load_summary(vd) for vd in variants}
    per_probe = {vd.name: _per_probe_slash(_load_scored(vd)) for vd in variants}

    md_path = _write_compare_md(run_dir, summaries, per_probe)
    json_path = _write_compare_json(run_dir, summaries)

    print(f"\nCompare report:")
    print(f"  {md_path}")
    print(f"  {json_path}")


if __name__ == "__main__":
    main()
