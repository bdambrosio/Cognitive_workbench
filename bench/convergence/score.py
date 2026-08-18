#!/usr/bin/env python3
"""Score probe 1 — convergence.

    python bench/convergence/score.py --run-dir bench/convergence/results/<ts>_<arm>

DIVISION OF LABOUR. An LLM EXTRACTS structure from the reply's prose; the
comparison against ground truth is arithmetic. The instrument therefore cannot
move the score — it can only fail to find a claim, which shows up as
`not_stated` rather than as `wrong`. Grading whether prose asserts a path is
primed is meaning, not string presence, so regexing it would be exactly the
keyword matching the house rules forbid.

The extractor is pinned to ONE arm regardless of which arm is under test
(bench/common.EXTRACTOR_ARM). Scoring arm A with a Gemma extraction and arm B
with a Luna extraction would let extraction quality masquerade as the metric.
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
sys.path.insert(0, str(REPO / "src"))
sys.path.insert(0, str(REPO))

from bench.common import build_extractor, extract_json  # noqa: E402

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s %(levelname)s %(name)s: %(message)s")
logger = logging.getLogger("bench.convergence.score")

GROUND_TRUTH = HERE / "ground_truth.json"

PATH_DESCRIPTIONS = {
    "seed": "the path that creates concerns from scenario YAML seed entries at "
            "startup (seed=True, rhythm supplied per-seed from config)",
    "hop_budget_carrier": "the path that creates a concern to resume an "
                          "agent-to-agent exchange whose hop budget was spent, "
                          "so an undelivered message can be re-sent",
    "user_yield": "the path that creates a continuation concern when the agent "
                  "ends a turn with the yield action mid-work",
    "successor": "the path that creates a successor/continuation concern when "
                 "an autonomous fire runs out of room, carrying a depth counter",
}

EXTRACT_SYS = """You extract structured facts from a technical report. You do
not evaluate, correct, or supplement it — if the report does not state a fact,
you return null for that fact. Inventing a plausible value corrupts the
measurement.

You are given a report describing code paths that create "agent_concern"
records, and a list of the paths we care about with a description of each.
Match the report's paths to ours by MEANING, not by wording — the report may
name them differently, or describe them without naming them at all.

For each of our paths, return what the report claims:

  "activation": the numeric starting activation the report attributes to this
      path. If the report says it starts at the constructor default, or "0", or
      "not primed", return 0.0. If the report says it is primed at the fire
      threshold, return the number it gives (e.g. 0.7). If the report does not
      say, return null.
  "rhythm_hours": the numeric rhythm in hours. If the report says it comes from
      the scenario YAML / is configured per-seed / is external rather than a
      fixed number, return the string "from_yaml". If not stated, return null.
  "rhythm_source": the report's value for the rhythm source field, lowercased
      (typically "external" or "urgency"). If not stated, return null.
  "fires_without_autonomy": true if the report claims this path CAN fire when
      the --autonomy flag is off; false if it claims it CANNOT; null if the
      report does not address it. Note: a report may distinguish CREATION from
      FIRING — this field is about FIRING only.
  "path_described": true if the report describes this path at all, else false.

Return ONLY JSON of the form:
{"paths": {"<our_key>": {"activation": ..., "rhythm_hours": ...,
 "rhythm_source": ..., "fires_without_autonomy": ..., "path_described": ...}}}
"""

CELLS = ("activation", "rhythm_hours", "rhythm_source", "fires_without_autonomy")


def _truth_cell(path: Dict[str, Any], cell: str) -> Any:
    if cell == "rhythm_hours":
        return "from_yaml" if path.get("rhythm_hours") is None else path["rhythm_hours"]
    return path.get(cell)


def _cells_match(cell: str, claimed: Any, truth: Any) -> bool:
    if claimed is None:
        return False
    if cell == "activation":
        try:
            return abs(float(claimed) - float(truth)) < 0.01
        except (TypeError, ValueError):
            return False
    if cell == "rhythm_hours":
        if isinstance(truth, str):
            return isinstance(claimed, str) and claimed.strip().lower() == truth
        try:
            return abs(float(claimed) - float(truth)) < 0.01
        except (TypeError, ValueError):
            return False
    if cell == "rhythm_source":
        return str(claimed).strip().lower() == str(truth).strip().lower()
    if cell == "fires_without_autonomy":
        return bool(claimed) == bool(truth)
    return False


def score_convergence(reply: str, truth: Dict[str, Any]) -> Dict[str, Any]:
    """16 cells: 4 paths x 4 facts."""
    backend = build_extractor()
    body = (f"OUR PATHS:\n" +
            "\n".join(f"- {k}: {v}" for k, v in PATH_DESCRIPTIONS.items()) +
            f"\n\n=== REPORT ===\n{reply}")
    parsed = extract_json(backend, EXTRACT_SYS, body, max_tokens=2000)
    claimed = ((parsed or {}).get("paths") or {}) if isinstance(parsed, dict) else {}
    if not claimed:
        logger.warning("extraction produced no paths — scoring all cells "
                       "not_stated. This is an INSTRUMENT failure, not "
                       "necessarily a backend failure; check raw extraction.")

    grid: Dict[str, Dict[str, str]] = {}
    correct = wrong = not_stated = 0
    for p in truth["paths"]:
        key = p["key"]
        got = claimed.get(key) or {}
        grid[key] = {}
        for cell in CELLS:
            c = got.get(cell)
            t = _truth_cell(p, cell)
            if c is None:
                grid[key][cell] = "not_stated"
                not_stated += 1
            elif _cells_match(cell, c, t):
                grid[key][cell] = "correct"
                correct += 1
            else:
                grid[key][cell] = f"wrong (said {c!r}, truth {t!r})"
                wrong += 1
    return {
        "cells_correct": correct,
        "cells_wrong": wrong,
        "cells_not_stated": not_stated,
        "cells_total": len(truth["paths"]) * len(CELLS),
        "score": round(correct / (len(truth["paths"]) * len(CELLS)), 4),
        "paths_described": sorted(k for k, v in claimed.items()
                                  if (v or {}).get("path_described")),
        "grid": grid,
        "extraction_ok": bool(claimed),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", type=Path, required=True)
    args = ap.parse_args()

    raw = json.loads((args.run_dir / "raw.json").read_text(encoding="utf-8"))
    meta = json.loads((args.run_dir / "run_meta.json").read_text(encoding="utf-8"))
    truth = json.loads(GROUND_TRUTH.read_text(encoding="utf-8"))

    conv = score_convergence(raw.get("reply") or "", truth)

    from bench.common import scan_validity  # noqa: E402
    summary = {
        "validity": scan_validity(meta.get("captured_at"),
                                  meta.get("wall_clock_s")),
        "backend_arm": meta.get("backend_arm"),
        "backend_label": meta.get("backend_label"),
        "served_model_check": meta.get("served_model_check"),
        "ground_truth_commit": truth.get("commit"),
        "wall_clock_s": meta.get("wall_clock_s"),
        "costs": meta.get("costs"),
        "probe1_convergence": conv,
    }
    (args.run_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, default=str) + "\n", encoding="utf-8")

    print(f"\n=== {meta.get('backend_label')} ===")
    print(f"  wall clock          {meta.get('wall_clock_s')}s")
    print(f"  probe1 convergence  {conv['cells_correct']}/{conv['cells_total']} "
          f"(wrong={conv['cells_wrong']} not_stated={conv['cells_not_stated']})")
    for pk, cells in conv["grid"].items():
        bad = {c: v for c, v in cells.items() if v != "correct"}
        print(f"    {pk:<20} {'ALL CORRECT' if not bad else bad}")
    costs = meta.get("costs") or {}
    print(f"  exits               {costs.get('exit_reasons')} "
          f"(iters_total={costs.get('iters_total')})")
    print(f"  wrote {args.run_dir / 'summary.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
