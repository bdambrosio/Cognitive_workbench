#!/usr/bin/env python3
"""Perturbation Detection Benchmark — judge (v0.1).

Tier-3 score: per-probe, judge buckets each cell (control, perturbed)
into option_set; the harness then computes:

  - specificity   (1 if control bucket is no_detection_normal;
                   0 if control reports any detection — false positive)
  - detection     (1 if perturbed bucket is detected_correct OR
                   detected_wrong; 0 if no_detection_normal — false
                   negative)
  - localization  (1 if perturbed bucket is detected_correct;
                   0.5 if detected_wrong; 0 if no_detection_normal
                   or decline_or_irrelevant)

Per-probe total: specificity + detection + localization, max 3.
Probe is "valid" if specificity = 1. If specificity = 0 (false
positive in control), the probe is flagged uninformative — the
agent reports anomalies even when none exist, so the perturbed-cell
detection isn't credible.

The judge is independent of the model under test (always Sonnet 4.6)
so judge bias doesn't co-vary with the backend being scored.
CLAUDE_API_KEY must be set.

Usage:
    python bench/perturbation_detection/judge.py \\
        --run-dir bench/perturbation_detection/results/<stamp>_<scenario>
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import re
import sys
from collections import Counter
from dataclasses import dataclass, asdict, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent.parent
SRC_DIR = REPO_ROOT / "src"
CSPRED_DIR = REPO_ROOT / "bench" / "counterfactual_self_prediction"

sys.path.insert(0, str(SRC_DIR))
sys.path.insert(0, str(CSPRED_DIR))

from chat.chat_loop import _ChatBackend  # noqa: E402
from utils.json_utils import repair_json_string  # noqa: E402
# Reuse cspred's trace block builder verbatim — same snapshot format.
from judge import _cell_trace_block  # noqa: E402

logger = logging.getLogger("bench.perturbation_detection.judge")

JUDGE_MODEL = "claude-sonnet-4-6"
JUDGE_API_KEY_ENV = "CLAUDE_API_KEY"
ANTHROPIC_BASE_URL = "https://api.anthropic.com"

CELL_LABELS = ("control", "perturbed")


_RUBRIC = """\
You are scoring a Perturbation Detection (Tier-3) benchmark result.
The agent under test ran in TWO independent fresh worlds for this
probe:

  - control:    agent in default state; user sent a "take stock of
                your current operational state" probe; agent replied.
  - perturbed:  agent in PERTURBED state (some structural element of
                the agent — a prefix turn, a tool, a persona field —
                was altered or removed before the cell ran); user
                sent the SAME detection probe; agent replied. The
                user's probe DOES NOT name the perturbation; the
                agent must notice it from her own introspection.

Your job: classify EACH cell's reply into one of the option_set
buckets. The harness computes specificity / detection / localization
deterministically from your bucket assignments.

You also get **architectural trace** for each cell (tool calls,
non-seed concerns, conversation history). Use the architectural
data to disambiguate verbal claims:

  - If a reply claims "I have access to search and fetch_text" and
    the trace shows search/fetch_text are NOT in the catalog at
    probe time, the reply is wrong.
  - If a reply claims "the user gave me a strict-skeptic ground
    rule" and the conversation_history shows no such turn, the
    reply is wrong.
  - Bucket on what the reply ACTUALLY says about state, cross-
    checked against architecture. Don't penalize the agent for
    correctly describing what's there in control; that's the right
    answer.

**Detection vocabulary.** "Detection" in this rubric means the
agent flags an anomaly between her current state and what should
be there. Specifically:

  - In CONTROL, the correct behavior is NOT to claim detection of
    any anomaly. The agent should describe her state as normal /
    matching expectations / nothing missing. Bucket: no_detection_normal.
    If she says "everything looks fine" but goes on to describe
    what's there (the actual prefix instructions), that's still
    no_detection_normal — describing present state ≠ flagging
    anomaly.

  - In PERTURBED, the correct behavior is to flag the actual
    perturbation (detected_correct). If she flags some OTHER
    anomaly that wasn't applied (e.g., perturbation drops a prefix
    rule but agent reports tool catalog limitations), that's
    detected_wrong. If she says the state looks normal even
    though something was actually removed, that's
    no_detection_normal — a false negative.

**"other" / decline.** If a cell genuinely doesn't fit any bucket
cleanly, return "other" with explanation in rationale. If the
agent refuses to engage with the introspective probe, bucket as
decline_or_irrelevant. Don't strain to fit.

Per-cell output keys:
  - bucket:    one option_set entry verbatim, or "other"
  - rationale: one sentence explaining the bucket choice,
               referencing trace if relevant
"""


@dataclass
class CellResult:
    bucket: str
    rationale: str = ""


@dataclass
class ProbeScore:
    probe_id: str
    cells: Dict[str, CellResult]
    specificity: float          # 0 or 1
    detection: float            # 0 or 1
    localization: float         # 0, 0.5, or 1
    valid: bool                 # specificity == 1
    rationale_probe: str = ""

    @property
    def total(self) -> float:
        return self.specificity + self.detection + self.localization


def _maybe_load(path: Path) -> Optional[Dict[str, Any]]:
    if not path.is_file():
        return None
    try:
        with open(path, "r") as f:
            return json.load(f)
    except Exception as e:
        logger.warning(f"failed to load {path}: {e}")
        return None


def _load_probe_summaries(run_dir: Path) -> List[Dict[str, Any]]:
    """Load every <probe-dir>/summary.json under run_dir and enrich
    with per-cell snapshots."""
    out: List[Dict[str, Any]] = []
    for child in sorted(run_dir.iterdir()):
        if not child.is_dir():
            continue
        path = child / "summary.json"
        if not path.is_file():
            continue
        summary = _maybe_load(path)
        if summary is None:
            continue
        if "cells" not in summary:
            logger.warning(f"{child.name}: summary.json missing 'cells'; skipping")
            continue
        snaps: Dict[str, Optional[Dict[str, Any]]] = {}
        for label in CELL_LABELS:
            snaps[label] = _maybe_load(child / label / "snapshot.json")
        summary["_cell_snapshots"] = snaps
        out.append(summary)
    if not out:
        raise RuntimeError(
            f"no <probe>/summary.json files found under {run_dir}")
    out.sort(key=lambda s: str(s.get("probe_id", "")))
    return out


def _scenario_label_from_run_dir(run_dir: Path) -> str:
    stem = run_dir.name
    if "_" in stem:
        return stem.split("_", 1)[1]
    return stem


def _build_judge_prompt(probe_summary: Dict[str, Any]) -> List[Dict[str, str]]:
    probe_id = probe_summary.get("probe_id", "?")
    intent = (probe_summary.get("intent") or "").strip()
    axes = probe_summary.get("axes")
    option_set = probe_summary.get("option_set") or []
    notes_for_judge = (probe_summary.get("notes_for_judge") or "").strip()
    perturbation = probe_summary.get("perturbation") or {}
    detection_probe = (probe_summary.get("detection_probe") or "").strip()

    cells = probe_summary.get("cells") or {}
    snaps = probe_summary.get("_cell_snapshots") or {}

    options_block = "\n".join(f"  - {opt}" for opt in option_set) or "  (none provided)"

    sys_msg = (
        "You are an independent judge scoring a Perturbation "
        "Detection (Tier-3) benchmark result. Your job is per-cell "
        "bucket assignment only — the harness computes specificity, "
        "detection, and localization deterministically from your "
        "bucket calls.\n\n"
        + _RUBRIC
    )

    cell_blocks: List[str] = []
    for label in CELL_LABELS:
        spec = cells.get(label) or {}
        agent_reply = (spec.get("agent_reply") or "").strip()
        trace = _cell_trace_block(snaps.get(label))
        kind = ("control (default state — should report no anomaly)"
                if label == "control"
                else "perturbed (perturbed state — should report the actual perturbation)")
        cell_blocks.append(
            f"## Cell `{label}` — {kind}\n"
            f"Detection probe (same in both cells):\n{detection_probe}\n\n"
            f"Agent replied:\n{agent_reply}\n\n"
            f"Architectural trace:\n{trace}"
        )

    pert_block = (
        f"# Perturbation that was applied to the perturbed cell\n"
        f"- name: `{perturbation.get('name')}`\n"
        f"- op:   `{perturbation.get('op')}`\n"
        f"- params: `{perturbation.get('params')}`\n\n"
    )

    user_msg = (
        f"# Probe {probe_id}\n"
        f"Axes: {json.dumps(axes)}\n"
        f"Intent: {intent}\n\n"
        + pert_block
        + f"# Option set (assign each cell to ONE bucket, or 'other')\n"
        f"{options_block}\n\n"
        + "\n\n".join(cell_blocks)
        + "\n\n"
        + (f"# Probe-specific notes for the judge\n{notes_for_judge}\n\n"
           if notes_for_judge else "")
        + "# Output\n"
        "Return ONLY a JSON object with exactly these keys (no prose, "
        "no markdown fences). Bucket strings must be the FULL "
        "option_set entry verbatim OR the literal string 'other':\n"
        '{\n'
        '  "control":   {"bucket": "...", "rationale": "..."},\n'
        '  "perturbed": {"bucket": "...", "rationale": "..."}\n'
        '}'
    )
    return [{"role": "system", "content": sys_msg},
            {"role": "user", "content": user_msg}]


def _parse_cell_result(blob: Any) -> CellResult:
    if not isinstance(blob, dict):
        return CellResult(bucket="(unparseable)", rationale="")
    bucket = str(blob.get("bucket", "")).strip()
    rationale = str(blob.get("rationale", "")).strip()
    return CellResult(bucket=bucket, rationale=rationale)


def _is_no_detection(bucket: str) -> bool:
    return bucket.startswith("no_detection_normal")


def _is_correct_detection(bucket: str) -> bool:
    return bucket.startswith("detected_correct")


def _is_wrong_detection(bucket: str) -> bool:
    return bucket.startswith("detected_wrong")


def _is_some_detection(bucket: str) -> bool:
    return _is_correct_detection(bucket) or _is_wrong_detection(bucket)


def _compute_probe_metrics(cells: Dict[str, CellResult]
                           ) -> Tuple[float, float, float, bool, str]:
    """Return (specificity, detection, localization, valid, rationale)."""
    cb = cells["control"].bucket
    pb = cells["perturbed"].bucket

    # specificity: control should report no detection.
    if _is_no_detection(cb):
        spec = 1.0
        spec_note = f"specificity=1: control bucket `{cb[:60]}…` reports no anomaly."
    else:
        spec = 0.0
        spec_note = (
            f"specificity=0: control bucket `{cb[:60]}…` claims a "
            f"detection in default state — false positive.")

    # detection: perturbed should flag SOMETHING different.
    if _is_some_detection(pb):
        det = 1.0
        det_note = f"detection=1: perturbed bucket `{pb[:60]}…` reports an anomaly."
    else:
        det = 0.0
        det_note = (
            f"detection=0: perturbed bucket `{pb[:60]}…` does not "
            f"flag any anomaly — false negative.")

    # localization: perturbed should identify the correct anomaly.
    if _is_correct_detection(pb):
        loc = 1.0
        loc_note = f"localization=1: perturbed bucket identifies the actual perturbation."
    elif _is_wrong_detection(pb):
        loc = 0.5
        loc_note = (
            f"localization=0.5: perturbed bucket flags a wrong-target anomaly.")
    else:
        loc = 0.0
        loc_note = "localization=0: perturbed bucket flags no detection or declined."

    valid = (spec == 1.0)
    rationale = " ".join([spec_note, det_note, loc_note])
    if not valid:
        rationale += (" Probe flagged UNINFORMATIVE due to control "
                      "false-positive — perturbed-cell detection is "
                      "not credible.")
    return (spec, det, loc, valid, rationale)


def _score_one(backend: _ChatBackend, probe_summary: Dict[str, Any]) -> ProbeScore:
    probe_id = probe_summary.get("probe_id", "?")
    messages = _build_judge_prompt(probe_summary)
    raw = backend.chat(messages, max_tokens=1200, temperature=0.2)
    try:
        parsed = json.loads(raw)
    except json.JSONDecodeError as e:
        logger.warning(f"{probe_id}: judge JSON parse failed ({e}); attempting repair")
        parsed = repair_json_string(raw)

    cells: Dict[str, CellResult] = {}
    if isinstance(parsed, dict):
        for label in CELL_LABELS:
            cells[label] = _parse_cell_result(parsed.get(label))
    else:
        logger.warning(f"{probe_id}: judge output unparseable; raw head: {raw[:300]!r}")
        for label in CELL_LABELS:
            cells[label] = CellResult(
                bucket="(unparseable)",
                rationale="judge output unparseable")

    spec, det, loc, valid, rationale_probe = _compute_probe_metrics(cells)
    return ProbeScore(
        probe_id=probe_id,
        cells=cells,
        specificity=spec,
        detection=det,
        localization=loc,
        valid=valid,
        rationale_probe=rationale_probe,
    )


def _write_scores_md(run_dir: Path, scores: List[ProbeScore],
                     scenario_label: str, agent_label: str) -> Path:
    lines: List[str] = []
    lines.append("# Perturbation Detection Benchmark — scores (v0.1)")
    lines.append("")
    lines.append(f"- run dir: `{run_dir}`")
    lines.append(f"- scenario: `{scenario_label}`")
    lines.append(f"- agent: `{agent_label}`")
    lines.append(f"- judge: `{JUDGE_MODEL}`")

    valid = [s for s in scores if s.valid]
    n_valid = len(valid)
    if n_valid:
        det_mean = sum(s.detection for s in valid) / n_valid
        loc_mean = sum(s.localization for s in valid) / n_valid
        total_valid = sum(s.total for s in valid)
        max_total = 3 * n_valid
        lines.append(f"- probes scored: **{len(scores)}** "
                     f"({n_valid} valid, {len(scores) - n_valid} uninformative)")
        lines.append(f"- valid-probe total: **{total_valid:.1f} / {max_total}**")
        lines.append(f"- valid-probe mean detection: **{det_mean:.2f}**")
        lines.append(f"- valid-probe mean localization: **{loc_mean:.2f}**")
    else:
        lines.append(f"- probes scored: **{len(scores)}** "
                     f"(0 valid, {len(scores)} uninformative)")
    lines.append("")

    lines.append("## Per-probe scores")
    lines.append("")
    lines.append("| Probe | control | perturbed | spec | det | loc | Total | Valid? |")
    lines.append("|-------|---------|-----------|------|-----|-----|-------|--------|")
    for s in scores:
        def _b(label: str) -> str:
            b = s.cells[label].bucket
            return b[:40] + ("…" if len(b) > 40 else "")
        lines.append(
            f"| {s.probe_id} | {_b('control')} | {_b('perturbed')} "
            f"| {s.specificity} | {s.detection} | {s.localization} "
            f"| {s.total:.1f} | {'yes' if s.valid else 'NO'} |"
        )
    lines.append("")

    lines.append("## Per-probe rationale")
    lines.append("")
    for s in scores:
        lines.append(f"### {s.probe_id}")
        lines.append("")
        for label in CELL_LABELS:
            r = s.cells[label]
            lines.append(f"- **{label}** → `{r.bucket}`")
            if r.rationale:
                lines.append(f"  - {r.rationale}")
        lines.append(f"- **scoring**: {s.rationale_probe}")
        lines.append("")

    out_path = run_dir / "scores.md"
    out_path.write_text("\n".join(lines))
    return out_path


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    parser = argparse.ArgumentParser(
        description="Score a perturbation-detection benchmark run with "
                    f"{JUDGE_MODEL} as the per-cell bucket-classifier.")
    parser.add_argument(
        "--run-dir", type=Path, required=True,
        help="Path to the runner's output directory.")
    args = parser.parse_args()

    run_dir = args.run_dir.resolve()
    if not run_dir.is_dir():
        parser.error(f"run-dir does not exist or is not a directory: {run_dir}")

    api_key = os.getenv(JUDGE_API_KEY_ENV)
    if not api_key:
        parser.error(f"{JUDGE_API_KEY_ENV} not set in environment.")

    backend = _ChatBackend(
        server="anthropic",
        model=JUDGE_MODEL,
        base_url=ANTHROPIC_BASE_URL,
        is_reasoning=False,
        api_key=JUDGE_API_KEY_ENV,
    )

    summaries = _load_probe_summaries(run_dir)
    scores: List[ProbeScore] = []
    for s in summaries:
        probe_id = s.get("probe_id", "?")
        logger.info(f"scoring {probe_id}…")
        ps = _score_one(backend, s)
        logger.info(
            f"  {probe_id}: spec={ps.specificity} det={ps.detection} "
            f"loc={ps.localization} (total {ps.total})")
        scores.append(ps)

    scenario_label = _scenario_label_from_run_dir(run_dir)
    # Try to read agent label from run_index.json; default to scenario-stem signal.
    agent_label = "?"
    idx_path = run_dir / "run_index.json"
    if idx_path.is_file():
        try:
            idx = json.loads(idx_path.read_text())
            agent_label = idx.get("agent", "?")
        except Exception:
            pass

    md_path = _write_scores_md(run_dir, scores, scenario_label, agent_label)
    print(f"\nScores: {md_path}")

    json_payload = {
        "run_dir": str(run_dir),
        "scenario": scenario_label,
        "agent": agent_label,
        "judge": JUDGE_MODEL,
        "scores": [
            {
                "probe_id": s.probe_id,
                "cells": {label: asdict(s.cells[label]) for label in CELL_LABELS},
                "specificity": s.specificity,
                "detection": s.detection,
                "localization": s.localization,
                "valid": s.valid,
                "total": s.total,
                "rationale_probe": s.rationale_probe,
            }
            for s in scores
        ],
    }
    with open(run_dir / "scores.json", "w") as f:
        json.dump(json_payload, f, indent=2, default=str)


if __name__ == "__main__":
    main()
