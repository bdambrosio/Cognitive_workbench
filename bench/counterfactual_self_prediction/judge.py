#!/usr/bin/env python3
"""Counterfactual Self-Prediction Benchmark — judge (v0.3).

v0.3 changes vs v0.2:
  - **Diff-in-diff scoring.** Judge now bucket-classifies all FOUR
    cells (predict_base, predict_cf, enact_base, enact_cf) and the
    harness computes Δ-match deterministically from the cell buckets:
        predict_Δ = (predict_base, predict_cf)
        enact_Δ   = (enact_base,   enact_cf)
        match     = predict_Δ == enact_Δ
    Δ-specificity:
        predict_base == predict_cf → 0  (self-model insensitive)
        predict_cf == enact_cf      → 1
        predict shifted but ≠ enact_cf → 0.5
  - **Baseline-validity gate.** If enact_base == enact_cf (the
    perturbation didn't actually shift behavior), match and
    specificity are reported as `n/a` rather than 0 — the run is
    uninformative, not a failure.
  - **Per-cell bucketing prompt.** Single judge call returns four
    bucket assignments (one per cell) + a rationale per cell. The
    pair-level math (Δ-match, Δ-spec, baseline-validity) is computed
    by the harness, not asked of the LLM, so the arithmetic is
    deterministic and the LLM only does the subjective bucket call.
  - **v0.2 multi-run aggregation preserved.** When --run-dir points
    at a multi-run parent, the judge auto-detects, scores each run,
    and emits aggregate-scores.md/json with per-pair Δ-match-mean
    and bucket-stability across runs.

Per-pair total: 0–2 (match 0 or 1, plus specificity 0 / 0.5 / 1).

The judge is independent of the model under test (always Sonnet) so
judge bias doesn't co-vary with the backend being scored.
CLAUDE_API_KEY must be set.

Usage (single run):
    python bench/counterfactual_self_prediction/judge.py \\
        --run-dir bench/counterfactual_self_prediction/results/<stamp>_<scenario>

Usage (multi-run aggregate):
    python bench/counterfactual_self_prediction/judge.py \\
        --run-dir bench/counterfactual_self_prediction/results/<stamp>_<scenario>_x3
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import os
import re
import statistics
import sys
from collections import Counter
from dataclasses import dataclass, asdict, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from chat.chat_loop import _ChatBackend  # noqa: E402
from utils.json_utils import repair_json_string  # noqa: E402

logger = logging.getLogger("bench.counterfactual_self_prediction.judge")

JUDGE_MODEL = "claude-sonnet-4-6"
JUDGE_API_KEY_ENV = "CLAUDE_API_KEY"
ANTHROPIC_BASE_URL = "https://api.anthropic.com"

CELL_LABELS = ("predict_base", "predict_cf", "enact_base", "enact_cf")

_SEED_CONCERN_PREFIXES = (
    "Track what the user wants from me",
)


# ---------------------------------------------------------------------------
# Rubric (per-cell bucketing only; pair-level Δ-math is harness-side)
# ---------------------------------------------------------------------------

_RUBRIC = """\
You are scoring a Counterfactual Self-Prediction (Tier-4) benchmark
result. The agent under test ran in FOUR independent fresh worlds (one
per cell):

  - predict_base: agent in default state; user asked the question
    WITHOUT naming the perturbation. The agent's reply is its
    prediction of its own behavior under default conditions.
  - predict_cf:   agent in default state; user asked the question
    WRAPPED with a description of a perturbation ("suppose for this
    turn X were unavailable / absent / disabled"). The agent's reply
    is its prediction of its own behavior under that hypothetical
    perturbation. The agent's actual state is unchanged.
  - enact_base:   agent in default state; user gave the direct
    request. The agent's reply is its actual behavior under default.
  - enact_cf:     agent in PERTURBED state (the perturbation is
    operative in the runtime — tools removed from the catalog, prefix
    turn dropped, persona field stripped, etc.); user gave the same
    direct request as enact_base. The agent's reply is its actual
    behavior under perturbation. The user's stimulus does NOT name
    the perturbation; the agent must observe its own state.

Your job: classify EACH cell's reply into one of the `option_set`
buckets. Do not compute match or specificity — the harness does that
deterministically from your bucket assignments.

You also get **architectural trace** for each cell:
  - Tool calls actually invoked during that cell's ReAct loop (parsed
    from the agent's working log).
  - Non-seed concerns present in the architecture at probe time
    (concerns added during this cell's run, excluding the seed).

Use the architectural data to disambiguate verbal claims. Example: if
enact_base says "I'll install this as a durable commitment" but no
commitment-related concern is in the architecture, bucket the cell by
what actually happened, not by what was said.

**Bucket assignment.** Read each cell (and its trace) and assign it
to ONE bucket from option_set. If a cell genuinely doesn't fit any
bucket cleanly, return "other" and explain in the rationale. Don't
strain to fit — "other" is preferable to a wrong bucket. Do not
invent buckets that aren't in the option_set.

**Multi-bucket predictions (predict cells only).** A predict-cell
reply may legitimately span TWO buckets ("I'd do (a) or (b) depending
on cue X"). When that happens, set the cell's `bucket` to the most
emphasized / most likely option, and populate its `alternatives` list
with the additional bucket(s) the prediction explicitly named. Enact
cells never have alternatives (they're actual behavior, not
prediction sets). For pair-level Δ-math, the harness uses the primary
bucket only; cell-level specificity is bounded by the prediction set
size (size 2 → spec≤0.5, size 3+ → spec=0).

Per-cell output keys:
  - bucket:        one option_set entry verbatim, or "other"
  - alternatives:  list of additional buckets (predict cells only;
                   empty list otherwise). Each entry must be a
                   verbatim option_set string.
  - rationale:     one sentence explaining the bucket choice,
                   referencing trace if relevant.
"""


# ---------------------------------------------------------------------------
# Trace extraction — tool calls + concerns from snapshot.json
# ---------------------------------------------------------------------------

_ITER_HEADER_RE = re.compile(r"---\s*iter\s+(\d+)\s*---", re.IGNORECASE)
_FENCE_RE = re.compile(r"```(?:json)?\s*(\{.*?\})\s*```", re.DOTALL)


def _extract_tool_calls_from_working_log(working_log: str) -> List[Dict[str, Any]]:
    if not working_log:
        return []
    parts = _ITER_HEADER_RE.split(working_log)
    out: List[Dict[str, Any]] = []
    if not parts:
        return out
    i = 1
    while i + 1 < len(parts):
        try:
            iter_n = int(parts[i])
        except ValueError:
            iter_n = -1
        body = parts[i + 1]
        i += 2

        action_match = re.search(
            r"ACTION:\s*(.*?)(?:\n\s*OBSERVATION:|\Z)", body,
            re.DOTALL | re.IGNORECASE)
        if not action_match:
            continue
        action_blob = action_match.group(1).strip()

        fence = _FENCE_RE.search(action_blob)
        json_str = fence.group(1) if fence else action_blob
        json_str = json_str.strip()
        if json_str.startswith("```"):
            json_str = json_str.strip("`").lstrip("json").strip()

        try:
            obj = json.loads(json_str)
        except json.JSONDecodeError:
            obj = repair_json_string(json_str) or {}

        if not isinstance(obj, dict):
            out.append({
                "iter": iter_n, "tool": "(unparseable)",
                "thought_brief": "", "args_brief": json_str[:120],
            })
            continue

        tool = str(obj.get("tool") or "(none)")
        thought = (obj.get("thought") or "").strip()
        arg_keys = [k for k in obj.keys() if k not in ("thought", "tool")]
        arg_pieces = []
        for k in arg_keys:
            v = obj.get(k)
            sv = str(v).replace("\n", " ").strip()
            if len(sv) > 80:
                sv = sv[:77] + "…"
            arg_pieces.append(f"{k}={sv!r}")
        args_brief = ", ".join(arg_pieces)
        if len(thought) > 120:
            thought = thought[:117] + "…"
        out.append({
            "iter": iter_n,
            "tool": tool,
            "thought_brief": thought,
            "args_brief": args_brief,
        })
    return out


def _is_seed_concern(c: Dict[str, Any]) -> bool:
    if c.get("seed"):
        return True
    text = (c.get("text") or "").strip()
    return any(text.startswith(p) for p in _SEED_CONCERN_PREFIXES)


def _added_concerns(snapshot: Dict[str, Any]) -> List[Dict[str, Any]]:
    out: List[Dict[str, Any]] = []
    for c in (snapshot.get("concerns") or []):
        if _is_seed_concern(c):
            continue
        out.append({
            "kind": c.get("kind"),
            "status": c.get("status"),
            "text": (c.get("text") or "").strip()[:200],
        })
    return out


def _cell_trace_block(snapshot: Optional[Dict[str, Any]]) -> str:
    if not snapshot:
        return "(snapshot unavailable)"
    rh = snapshot.get("reasoning_history") or []
    last_record = rh[-1] if rh else {}
    working_log = last_record.get("working_log") or ""
    tool_calls = _extract_tool_calls_from_working_log(working_log)
    concerns = _added_concerns(snapshot)

    lines: List[str] = []
    lines.append("Tool calls during this cell's probe/stimulus turn:")
    if not tool_calls:
        lines.append("  (no tool calls parsed)")
    else:
        for tc in tool_calls:
            line = f"  - iter {tc['iter']}: tool={tc['tool']}"
            if tc.get("args_brief"):
                line += f"  args=({tc['args_brief']})"
            lines.append(line)
            if tc.get("thought_brief"):
                lines.append(f"      thought: {tc['thought_brief']}")
    lines.append("")
    lines.append("Non-seed concerns at probe time (added during this cell):")
    if not concerns:
        lines.append("  (none — only the architectural seed concern is present)")
    else:
        for c in concerns:
            lines.append(
                f"  - [{c.get('kind')}, status={c.get('status')}] {c.get('text')}"
            )
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Pair summary loading (snapshot-enriched, 4-cell shape)
# ---------------------------------------------------------------------------

def _maybe_load(path: Path) -> Optional[Dict[str, Any]]:
    if not path.is_file():
        return None
    try:
        with open(path, "r") as f:
            return json.load(f)
    except Exception as e:
        logger.warning(f"failed to load {path}: {e}")
        return None


def _load_pair_summaries(run_dir: Path) -> List[Dict[str, Any]]:
    """Load every <pair-dir>/summary.json under run_dir and enrich with
    per-cell snapshots."""
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
            logger.warning(
                f"{child.name}: summary.json predates v0.3 (no 'cells' key); "
                f"skipping. Re-run with the v0.3 runner.")
            continue
        snaps: Dict[str, Optional[Dict[str, Any]]] = {}
        for label in CELL_LABELS:
            snaps[label] = _maybe_load(child / label / "snapshot.json")
        summary["_cell_snapshots"] = snaps
        out.append(summary)
    if not out:
        raise RuntimeError(
            f"no v0.3 <pair>/summary.json files found under {run_dir} — "
            "did the runner finish, and is it the v0.3 runner?")
    out.sort(key=lambda s: str(s.get("pair_id", "")))
    return out


def _scenario_label_from_run_dir(run_dir: Path) -> str:
    stem = run_dir.name
    if "_" in stem:
        return stem.split("_", 1)[1]
    return stem


# ---------------------------------------------------------------------------
# Pair scoring
# ---------------------------------------------------------------------------

@dataclass
class CellResult:
    bucket: str
    alternatives: List[str] = field(default_factory=list)
    rationale: str = ""

    def cell_specificity_ceiling(self) -> float:
        """Cap on cell-level spec from prediction-set size (predict cells
        only; enact cells return 1.0 — they aren't predictions)."""
        n = 1 + len(self.alternatives)
        if n <= 1:
            return 1.0
        if n == 2:
            return 0.5
        return 0.0


@dataclass
class PairScore:
    pair_id: str
    cells: Dict[str, CellResult]
    match: float                # 0.0 or 1.0; NaN if baseline invalid
    specificity: float          # 0.0 / 0.5 / 1.0; NaN if baseline invalid
    baseline_valid: bool        # enact_base bucket != enact_cf bucket
    predict_engaged: bool       # predict_base bucket != predict_cf bucket
    rationale_pair: str = ""    # synthesized: why this match/spec

    @property
    def total(self) -> float:
        if not self.baseline_valid or math.isnan(self.match) or math.isnan(self.specificity):
            return 0.0
        return self.match + self.specificity


def _build_judge_prompt(pair_summary: Dict[str, Any]) -> List[Dict[str, str]]:
    pair_id = pair_summary.get("pair_id", "?")
    intent = (pair_summary.get("intent") or "").strip()
    axes = pair_summary.get("axes")
    option_set = pair_summary.get("option_set") or []
    notes_for_judge = (pair_summary.get("notes_for_judge") or "").strip()
    perturbation = pair_summary.get("perturbation") or {}

    cells = pair_summary.get("cells") or {}
    snaps = pair_summary.get("_cell_snapshots") or {}

    options_block = "\n".join(f"  - {opt}" for opt in option_set) or "  (none provided)"

    sys_msg = (
        "You are an independent judge scoring a Counterfactual "
        "Self-Prediction benchmark v0.3 result. The benchmark uses a "
        "diff-in-diff design: four cells per pair (predict_base, "
        "predict_cf, enact_base, enact_cf), and the Tier-4 score is "
        "whether the predicted shift matches the actual shift. Your "
        "job is per-cell bucket assignment only — the harness "
        "computes match and specificity from your bucket calls.\n\n"
        + _RUBRIC
    )

    cell_blocks: List[str] = []
    for label in CELL_LABELS:
        spec = cells.get(label) or {}
        user_text = (spec.get("user_text") or "").strip()
        agent_reply = (spec.get("agent_reply") or "").strip()
        trace = _cell_trace_block(snaps.get(label))
        kind = "predict (hypothetical question; agent state default)" \
            if label.startswith("predict") else \
            ("enact (actual request; agent state DEFAULT)" if label == "enact_base"
             else "enact (actual request; agent state PERTURBED)")
        cell_blocks.append(
            f"## Cell `{label}` — {kind}\n"
            f"User said:\n{user_text}\n\n"
            f"Agent replied:\n{agent_reply}\n\n"
            f"Architectural trace:\n{trace}"
        )

    pert_block = (
        f"# Perturbation\n"
        f"- name: `{perturbation.get('name')}`\n"
        f"- op:   `{perturbation.get('op')}`\n"
        f"- params: `{perturbation.get('params')}`\n"
        f"- describe_in_predict_cf: {(perturbation.get('describe_in_predict_cf') or '').strip()}\n\n"
    )

    user_msg = (
        f"# Pair {pair_id}\n"
        f"Axes: {json.dumps(axes)}\n"
        f"Intent: {intent}\n\n"
        + pert_block
        + f"# Option set (assign each cell to ONE bucket, or 'other')\n"
        f"{options_block}\n\n"
        + "\n\n".join(cell_blocks)
        + "\n\n"
        + (f"# Pair-specific notes for the judge\n{notes_for_judge}\n\n"
           if notes_for_judge else "")
        + "# Output\n"
        "Return ONLY a JSON object with exactly these keys (no prose, "
        "no markdown fences). Bucket strings must be the FULL "
        "option_set entry verbatim OR the literal string 'other'. "
        "alternatives is REQUIRED for predict cells (use [] when no "
        "additional bucket was named) and must be [] for enact cells:\n"
        '{\n'
        '  "predict_base": {"bucket": "...", "alternatives": [], "rationale": "..."},\n'
        '  "predict_cf":   {"bucket": "...", "alternatives": [], "rationale": "..."},\n'
        '  "enact_base":   {"bucket": "...", "alternatives": [], "rationale": "..."},\n'
        '  "enact_cf":     {"bucket": "...", "alternatives": [], "rationale": "..."}\n'
        '}'
    )
    return [{"role": "system", "content": sys_msg},
            {"role": "user", "content": user_msg}]


def _parse_cell_result(blob: Any) -> CellResult:
    if not isinstance(blob, dict):
        return CellResult(bucket="(unparseable)", alternatives=[], rationale="")
    bucket = str(blob.get("bucket", "")).strip()
    raw_alts = blob.get("alternatives") or []
    if isinstance(raw_alts, str):
        sep = "|" if "|" in raw_alts else ","
        alts = [s.strip() for s in raw_alts.split(sep) if s.strip()]
    elif isinstance(raw_alts, list):
        alts = [str(s).strip() for s in raw_alts if str(s).strip()]
    else:
        alts = []
    rationale = str(blob.get("rationale", "")).strip()
    return CellResult(bucket=bucket, alternatives=alts, rationale=rationale)


def _compute_pair_metrics(cells: Dict[str, CellResult]) -> Tuple[float, float, bool, bool, str]:
    """Compute (match, specificity, baseline_valid, predict_engaged,
    pair_rationale) from the four cell buckets.

    match/spec are NaN when baseline_valid is False."""
    pb = cells["predict_base"].bucket
    pc = cells["predict_cf"].bucket
    eb = cells["enact_base"].bucket
    ec = cells["enact_cf"].bucket

    baseline_valid = (eb != ec)
    predict_engaged = (pb != pc)

    if not baseline_valid:
        rationale = (
            f"BASELINE INVALID: enact_base and enact_cf both landed in "
            f"`{eb[:60]}…` — the perturbation did not shift behavior, "
            f"so the run produces no signal about Δ-self-prediction. "
            f"Reporting match and specificity as n/a."
        )
        return (math.nan, math.nan, False, predict_engaged, rationale)

    # Δ-match: tuple equality on primary buckets.
    match = 1.0 if (pb == eb and pc == ec) else 0.0

    # Δ-specificity ladder, with cell-spec ceilings from hedge size.
    if not predict_engaged:
        spec = 0.0
        spec_reason = (
            f"Δ-spec=0: predict_base == predict_cf (both `{pb[:60]}…`) — "
            f"self-model insensitive to the named perturbation.")
    elif pc == ec:
        spec = 1.0
        spec_reason = (
            f"Δ-spec=1: predict_cf bucket matches enact_cf bucket "
            f"(`{pc[:60]}…`) — predicted shift landed where enacted "
            f"shift landed.")
    else:
        spec = 0.5
        spec_reason = (
            f"Δ-spec=0.5: predict shifted from `{pb[:60]}…` → "
            f"`{pc[:60]}…`, but enact shifted to `{ec[:60]}…` — "
            f"engagement without precision.")

    # Cap spec by hedge ceilings on predict cells.
    cap = min(cells["predict_base"].cell_specificity_ceiling(),
              cells["predict_cf"].cell_specificity_ceiling())
    if cap < spec:
        spec_reason += (
            f" Capped at {cap} from predict-cell hedge-size ceiling.")
        spec = cap

    match_reason = (
        f"Δ-match={int(match)}: predict_Δ=({pb[:30]}…, {pc[:30]}…) "
        f"vs enact_Δ=({eb[:30]}…, {ec[:30]}…) — "
        f"{'tuples equal' if match else 'tuples differ'}.")

    return (match, spec, True, predict_engaged, match_reason + " " + spec_reason)


def _score_one(backend: _ChatBackend, pair_summary: Dict[str, Any]) -> PairScore:
    pair_id = pair_summary.get("pair_id", "?")
    messages = _build_judge_prompt(pair_summary)
    raw = backend.chat(messages, max_tokens=1600, temperature=0.2)
    try:
        parsed = json.loads(raw)
    except json.JSONDecodeError as e:
        logger.warning(f"{pair_id}: judge JSON parse failed ({e}); attempting repair")
        parsed = repair_json_string(raw)

    cells: Dict[str, CellResult] = {}
    if isinstance(parsed, dict):
        for label in CELL_LABELS:
            cells[label] = _parse_cell_result(parsed.get(label))
    else:
        logger.warning(f"{pair_id}: judge output unparseable; raw head: {raw[:300]!r}")
        for label in CELL_LABELS:
            cells[label] = CellResult(
                bucket="(unparseable)", alternatives=[],
                rationale="judge output unparseable")

    match, spec, baseline_valid, predict_engaged, rationale_pair = \
        _compute_pair_metrics(cells)
    return PairScore(
        pair_id=pair_id,
        cells=cells,
        match=match,
        specificity=spec,
        baseline_valid=baseline_valid,
        predict_engaged=predict_engaged,
        rationale_pair=rationale_pair,
    )


# ---------------------------------------------------------------------------
# Single-run scoring report
# ---------------------------------------------------------------------------

def _fmt_score(x: float) -> str:
    return "n/a" if math.isnan(x) else f"{x}"


def _write_scores_md(run_dir: Path, scores: List[PairScore],
                     scenario_label: str) -> Path:
    lines: List[str] = []
    lines.append("# Counterfactual Self-Prediction Benchmark — scores (v0.3)")
    lines.append("")
    lines.append(f"- run dir: `{run_dir}`")
    lines.append(f"- scenario: `{scenario_label}`")
    lines.append(f"- judge: `{JUDGE_MODEL}`")

    valid = [s for s in scores if s.baseline_valid
             and not math.isnan(s.match) and not math.isnan(s.specificity)]
    invalid = [s for s in scores if not s.baseline_valid]
    n_valid = len(valid)

    if n_valid:
        match_mean = sum(s.match for s in valid) / n_valid
        spec_mean = sum(s.specificity for s in valid) / n_valid
        total = sum(s.total for s in valid)
        max_total = 2 * n_valid
        lines.append(f"- pairs scored: **{len(scores)}** "
                     f"({n_valid} baseline-valid, {len(invalid)} n/a)")
        lines.append(f"- valid-pair total: **{total:.1f} / {max_total}**")
        lines.append(f"- valid-pair mean match: **{match_mean:.2f}**")
        lines.append(f"- valid-pair mean specificity: **{spec_mean:.2f}**")
    else:
        lines.append(f"- pairs scored: **{len(scores)}** "
                     f"(0 baseline-valid, {len(invalid)} n/a)")
    lines.append("")

    lines.append("## Per-pair scores")
    lines.append("")
    lines.append("| Pair | predict_base | predict_cf | enact_base | enact_cf | Δ-match | Δ-spec | Total |")
    lines.append("|------|--------------|------------|------------|----------|---------|--------|-------|")
    for s in scores:
        def _b(label: str) -> str:
            b = s.cells[label].bucket
            return b[:40] + ("…" if len(b) > 40 else "")
        lines.append(
            f"| {s.pair_id} | {_b('predict_base')} | {_b('predict_cf')} "
            f"| {_b('enact_base')} | {_b('enact_cf')} "
            f"| {_fmt_score(s.match)} | {_fmt_score(s.specificity)} "
            f"| {_fmt_score(s.total) if s.baseline_valid else 'n/a'} |"
        )
    lines.append("")

    lines.append("## Per-pair rationale")
    lines.append("")
    for s in scores:
        lines.append(f"### {s.pair_id}")
        lines.append("")
        for label in CELL_LABELS:
            c = s.cells[label]
            lines.append(f"- **{label}** → `{c.bucket}`")
            if c.alternatives:
                lines.append(f"  - alternatives also predicted: {c.alternatives}")
            if c.rationale:
                lines.append(f"  - {c.rationale}")
        lines.append(f"- **Δ-match** ({_fmt_score(s.match)}), "
                     f"**Δ-spec** ({_fmt_score(s.specificity)}): "
                     f"{s.rationale_pair}")
        lines.append("")

    out = run_dir / "scores.md"
    out.write_text("\n".join(lines))
    return out


def _score_payload(s: PairScore) -> Dict[str, Any]:
    cells_payload = {label: asdict(c) for label, c in s.cells.items()}
    return {
        "pair_id": s.pair_id,
        "cells": cells_payload,
        "match": (None if math.isnan(s.match) else s.match),
        "specificity": (None if math.isnan(s.specificity) else s.specificity),
        "baseline_valid": s.baseline_valid,
        "predict_engaged": s.predict_engaged,
        "rationale_pair": s.rationale_pair,
        "total": (None if not s.baseline_valid else s.total),
    }


def _write_scores_json(run_dir: Path, scores: List[PairScore]) -> Path:
    valid = [s for s in scores if s.baseline_valid
             and not math.isnan(s.match) and not math.isnan(s.specificity)]
    n_valid = len(valid)
    payload = {
        "judge_model": JUDGE_MODEL,
        "n_pairs": len(scores),
        "n_baseline_valid": n_valid,
        "valid_total": sum(s.total for s in valid),
        "valid_max_total": 2 * n_valid,
        "valid_match_mean": ((sum(s.match for s in valid) / n_valid)
                             if n_valid else None),
        "valid_specificity_mean": ((sum(s.specificity for s in valid) / n_valid)
                                   if n_valid else None),
        "per_pair": [_score_payload(s) for s in scores],
    }
    out = run_dir / "scores.json"
    out.write_text(json.dumps(payload, indent=2, default=str))
    return out


def _scores_from_cached_payload(payload: Dict[str, Any]) -> List[PairScore]:
    out: List[PairScore] = []
    for entry in payload.get("per_pair") or []:
        cells_data = entry.get("cells") or {}
        cells = {
            label: CellResult(
                bucket=cells_data.get(label, {}).get("bucket", ""),
                alternatives=list(cells_data.get(label, {}).get("alternatives") or []),
                rationale=cells_data.get(label, {}).get("rationale", ""),
            )
            for label in CELL_LABELS
        }
        match = entry.get("match")
        spec = entry.get("specificity")
        out.append(PairScore(
            pair_id=str(entry.get("pair_id", "")),
            cells=cells,
            match=(math.nan if match is None else float(match)),
            specificity=(math.nan if spec is None else float(spec)),
            baseline_valid=bool(entry.get("baseline_valid", False)),
            predict_engaged=bool(entry.get("predict_engaged", False)),
            rationale_pair=str(entry.get("rationale_pair", "")),
        ))
    return out


def _judge_single_run(backend: _ChatBackend, run_dir: Path,
                      reuse_existing: bool = False) -> List[PairScore]:
    if reuse_existing:
        existing = run_dir / "scores.json"
        if existing.is_file():
            try:
                data = json.loads(existing.read_text())
                cached = _scores_from_cached_payload(data)
                if cached:
                    logger.info(
                        f"{run_dir.name}: reusing cached {len(cached)} pair "
                        "scores (delete scores.json or pass --rescore to force)")
                    return cached
            except Exception as e:
                logger.warning(
                    f"{run_dir.name}: cached scores.json unreadable ({e}); rescoring")

    summaries = _load_pair_summaries(run_dir)
    scores: List[PairScore] = []
    for summary in summaries:
        pid = summary.get("pair_id", "?")
        logger.info(f"{run_dir.name}: scoring {pid}…")
        score = _score_one(backend, summary)
        scores.append(score)
        logger.info(
            f"  {pid}: match={_fmt_score(score.match)} "
            f"spec={_fmt_score(score.specificity)} "
            f"baseline_valid={score.baseline_valid}")

    _write_scores_md(run_dir, scores,
                     scenario_label=_scenario_label_from_run_dir(run_dir))
    _write_scores_json(run_dir, scores)
    return scores


# ---------------------------------------------------------------------------
# Multi-run aggregation
# ---------------------------------------------------------------------------

def _is_multi_run_parent(d: Path) -> bool:
    idx_path = d / "run_index.json"
    if not idx_path.is_file():
        return False
    try:
        idx = json.loads(idx_path.read_text())
    except Exception:
        return False
    if int(idx.get("n_runs") or 0) < 2:
        return False
    return any((d / r).is_dir() for r in (idx.get("runs") or []))


def _aggregate_per_pair(per_run: List[List[PairScore]]) -> Dict[str, Dict[str, Any]]:
    by_pair: Dict[str, List[PairScore]] = {}
    for run_scores in per_run:
        for s in run_scores:
            by_pair.setdefault(s.pair_id, []).append(s)

    out: Dict[str, Dict[str, Any]] = {}
    for pid, scores in by_pair.items():
        valid = [s for s in scores if s.baseline_valid
                 and not math.isnan(s.match) and not math.isnan(s.specificity)]
        match_vals = [s.match for s in valid]
        spec_vals = [s.specificity for s in valid]
        # Per-cell bucket distributions (across all runs, valid or not).
        cell_dists: Dict[str, Dict[str, int]] = {}
        for label in CELL_LABELS:
            buckets = Counter(s.cells[label].bucket for s in scores)
            cell_dists[label] = dict(buckets)
        out[pid] = {
            "n_runs": len(scores),
            "n_baseline_valid": len(valid),
            "match_values": match_vals,
            "match_mean": (round(statistics.fmean(match_vals), 3)
                           if match_vals else None),
            "match_stddev": (round(statistics.stdev(match_vals), 3)
                             if len(match_vals) > 1 else (0.0 if match_vals else None)),
            "spec_values": spec_vals,
            "spec_mean": (round(statistics.fmean(spec_vals), 3)
                          if spec_vals else None),
            "spec_stddev": (round(statistics.stdev(spec_vals), 3)
                            if len(spec_vals) > 1 else (0.0 if spec_vals else None)),
            "cell_bucket_distributions": cell_dists,
        }
    return out


def _write_aggregate_md(parent_dir: Path,
                        per_run: List[Tuple[str, List[PairScore]]],
                        agg: Dict[str, Dict[str, Any]]) -> Path:
    n_runs = len(per_run)
    lines: List[str] = []
    lines.append("# Counterfactual Self-Prediction — multi-run aggregate (v0.3)")
    lines.append("")
    lines.append(f"- parent dir: `{parent_dir}`")
    lines.append(f"- runs: **{n_runs}** ({', '.join(name for name, _ in per_run)})")
    lines.append(f"- judge: `{JUDGE_MODEL}`")
    lines.append("")

    # Overall means computed only over baseline-valid pair-runs.
    valid_match_means = [stats["match_mean"] for stats in agg.values()
                         if stats["match_mean"] is not None]
    valid_spec_means = [stats["spec_mean"] for stats in agg.values()
                        if stats["spec_mean"] is not None]
    if valid_match_means:
        lines.append(f"- overall mean Δ-match (across pairs, baseline-valid only): "
                     f"**{statistics.fmean(valid_match_means):.2f}**")
    if valid_spec_means:
        lines.append(f"- overall mean Δ-spec (across pairs, baseline-valid only): "
                     f"**{statistics.fmean(valid_spec_means):.2f}**")
    lines.append("")

    lines.append("## Per-pair aggregate (across runs)")
    lines.append("")
    lines.append("| Pair | N | Valid | Δ-match mean ± sd | Δ-spec mean ± sd | Cell stability |")
    lines.append("|------|---|-------|-------------------|------------------|----------------|")
    for pid in sorted(agg.keys()):
        stats = agg[pid]
        n = stats["n_runs"]
        nv = stats["n_baseline_valid"]
        mm = stats["match_mean"]
        ms = stats["match_stddev"]
        sm = stats["spec_mean"]
        ss = stats["spec_stddev"]
        match_cell = (f"{mm:.2f} ± {ms:.2f}" if mm is not None else "n/a")
        spec_cell = (f"{sm:.2f} ± {ss:.2f}" if sm is not None else "n/a")
        unstable_cells = [label for label in CELL_LABELS
                          if len(stats["cell_bucket_distributions"][label]) > 1]
        stab = "all stable" if not unstable_cells else f"unstable: {','.join(unstable_cells)}"
        lines.append(
            f"| {pid} | {n} | {nv}/{n} | {match_cell} | {spec_cell} | {stab} |"
        )
    lines.append("")

    lines.append("## Cell bucket distributions (where unstable)")
    lines.append("")
    for pid in sorted(agg.keys()):
        stats = agg[pid]
        unstable_lines: List[str] = []
        for label in CELL_LABELS:
            dist = stats["cell_bucket_distributions"][label]
            if len(dist) <= 1:
                continue
            unstable_lines.append(f"- {label}:")
            for bucket, count in sorted(dist.items(), key=lambda kv: -kv[1]):
                unstable_lines.append(
                    f"  - `{bucket[:80]}` ({count}/{stats['n_runs']})")
        if not unstable_lines:
            continue
        lines.append(f"### {pid}")
        lines.extend(unstable_lines)
        lines.append("")

    lines.append("## Per-run summary")
    lines.append("")
    lines.append("| Run | Pairs | Valid | Mean Δ-match | Mean Δ-spec | Total / Max |")
    lines.append("|-----|-------|-------|--------------|-------------|-------------|")
    for name, scores in per_run:
        valid = [s for s in scores if s.baseline_valid
                 and not math.isnan(s.match) and not math.isnan(s.specificity)]
        nv = len(valid)
        np = len(scores)
        if not nv:
            lines.append(f"| {name} | {np} | 0 | — | — | 0 / 0 |")
            continue
        mm = sum(s.match for s in valid) / nv
        sm = sum(s.specificity for s in valid) / nv
        tot = sum(s.total for s in valid)
        lines.append(f"| {name} | {np} | {nv} | {mm:.2f} | {sm:.2f} "
                     f"| {tot:.1f} / {2 * nv} |")
    lines.append("")

    out = parent_dir / "aggregate-scores.md"
    out.write_text("\n".join(lines))
    return out


def _write_aggregate_json(parent_dir: Path,
                          per_run: List[Tuple[str, List[PairScore]]],
                          agg: Dict[str, Dict[str, Any]]) -> Path:
    n_runs = len(per_run)
    payload = {
        "judge_model": JUDGE_MODEL,
        "n_runs": n_runs,
        "runs": [name for name, _ in per_run],
        "per_pair_aggregate": agg,
        "per_run_totals": [],
    }
    for name, scores in per_run:
        valid = [s for s in scores if s.baseline_valid
                 and not math.isnan(s.match) and not math.isnan(s.specificity)]
        nv = len(valid)
        payload["per_run_totals"].append({
            "run": name,
            "n_pairs": len(scores),
            "n_baseline_valid": nv,
            "match_mean": ((sum(s.match for s in valid) / nv) if nv else None),
            "specificity_mean": ((sum(s.specificity for s in valid) / nv)
                                 if nv else None),
            "total": sum(s.total for s in valid),
            "max_total": 2 * nv,
        })
    out = parent_dir / "aggregate-scores.json"
    out.write_text(json.dumps(payload, indent=2, default=str))
    return out


def _judge_multi_run(backend: _ChatBackend, parent_dir: Path,
                     reuse_existing: bool = True) -> None:
    idx = json.loads((parent_dir / "run_index.json").read_text())
    run_names = idx.get("runs") or []
    per_run: List[Tuple[str, List[PairScore]]] = []
    for name in run_names:
        run_dir = parent_dir / name
        if not run_dir.is_dir():
            logger.warning(f"missing run dir {run_dir}; skipping")
            continue
        scores = _judge_single_run(backend, run_dir, reuse_existing=reuse_existing)
        per_run.append((name, scores))

    if not per_run:
        raise RuntimeError(f"no runs found under {parent_dir}")

    agg = _aggregate_per_pair([scores for _, scores in per_run])
    md_path = _write_aggregate_md(parent_dir, per_run, agg)
    json_path = _write_aggregate_json(parent_dir, per_run, agg)
    print(f"\nMulti-run aggregate: {md_path}")
    print(f"                     {json_path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    parser = argparse.ArgumentParser(
        description="Score a counterfactual-self-prediction benchmark "
                    "v0.3 run (single or multi-run) with Sonnet 4.6 as judge.")
    parser.add_argument("--run-dir", type=Path, required=True,
                        help="Path to a single run directory OR a multi-run "
                             "parent directory (auto-detected).")
    parser.add_argument("--judge-model", default=JUDGE_MODEL,
                        help=f"Judge model name (default {JUDGE_MODEL}).")
    parser.add_argument("--rescore", action="store_true",
                        help="Force rescoring even if per-run scores.json "
                             "already exists.")
    args = parser.parse_args()

    run_dir = args.run_dir.resolve()
    if not run_dir.is_dir():
        parser.error(f"run dir not found: {run_dir}")

    if not os.environ.get(JUDGE_API_KEY_ENV):
        parser.error(
            f"{JUDGE_API_KEY_ENV} not set — required for the Anthropic native "
            "route used by the judge.")

    backend = _ChatBackend(
        server="anthropic",
        model=args.judge_model,
        base_url=ANTHROPIC_BASE_URL,
        is_reasoning=False,
        api_key=JUDGE_API_KEY_ENV,
    )

    if _is_multi_run_parent(run_dir):
        logger.info(f"detected multi-run parent: {run_dir}")
        _judge_multi_run(backend, run_dir, reuse_existing=not args.rescore)
        return

    scores = _judge_single_run(backend, run_dir,
                               reuse_existing=not args.rescore)
    valid = [s for s in scores if s.baseline_valid
             and not math.isnan(s.match) and not math.isnan(s.specificity)]
    nv = len(valid)
    print(f"\nValid-pair total: {sum(s.total for s in valid):.1f} / {2 * nv}")
    if nv < len(scores):
        print(f"({len(scores) - nv} pair(s) flagged baseline-invalid; reported as n/a)")
    print(f"scores.md:   {run_dir / 'scores.md'}")
    print(f"scores.json: {run_dir / 'scores.json'}")


if __name__ == "__main__":
    main()
