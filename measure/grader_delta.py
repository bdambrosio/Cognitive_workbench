#!/usr/bin/env python3
"""Compare the live (self-graded) claims against a pinned re-grade.

    python3 measure/grader_delta.py --worlds-matching 'coord_search.*'

Answers one question: how much of a provenance difference between two runs
is the agent, and how much is the grader?

It matters because the live grader calls `self.backend.chat` — each run
graded itself — so any cross-arm comparison built on `claims.jsonl` alone
confounds the model under test with the instrument measuring it. The
`4% -> 97% model_prior` spread across the archived coord_search arms is
exactly such a comparison.

Reads `claims.jsonl` (self-graded) and `measure/regraded/<world>.<agent>.jsonl`
(pinned). Reports both, and the shift. Nothing is written.
"""

from __future__ import annotations

import argparse
import fnmatch
import json
import sys
from collections import Counter
from pathlib import Path
from typing import Dict, List, Optional, Tuple

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from measure.provenance import load_claim_rows                 # noqa: E402

REGRADED = REPO / "measure" / "regraded"

# Ordered so the table reads the same way every time.
GROUNDINGS = ("retrieved", "context", "inferred", "user_asserted",
              "memory", "model_prior")


def load_regraded(world: str, agent: str) -> List[Dict]:
    p = REGRADED / f"{world}.{agent}.jsonl"
    if not p.exists():
        return []
    out = []
    for line in p.open(errors="replace"):
        line = line.strip()
        if not line:
            continue
        try:
            out.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return out


def mix(rows: List[Dict]) -> Tuple[Counter, int]:
    g = Counter()
    for r in rows:
        for c in (r.get("claims") or []):
            g[c.get("grounding")] += 1
    return g, sum(g.values())


def pct(n: int, d: int) -> Optional[float]:
    return round(100.0 * n / d, 1) if d else None


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--worlds-matching", required=True)
    args = ap.parse_args()

    worlds = sorted(d.name for d in (REPO / "scenarios").iterdir()
                    if d.is_dir() and fnmatch.fnmatch(d.name, args.worlds_matching))

    rows_out = []
    for w in worlds:
        for agent_dir in sorted((REPO / "scenarios" / w).iterdir()):
            a = agent_dir.name
            regr = load_regraded(w, a)
            if not regr:
                continue
            self_g, self_n = mix(load_claim_rows(w, a))
            pin_g, pin_n = mix(regr)
            rows_out.append((f"{w}/{a}", self_g, self_n, pin_g, pin_n))

    if not rows_out:
        print("no re-graded output found — run measure/regrade.py first")
        return 1

    print(f"\n{'run':34} {'claims':>13}   {'model_prior':>13}   "
          f"{'retrieved':>13}")
    print(f"{'':34} {'self / pin':>13}   {'self / pin':>13}   "
          f"{'self / pin':>13}")
    print("-" * 82)
    for label, sg, sn, pg, pn in rows_out:
        mp = f"{pct(sg['model_prior'], sn)} / {pct(pg['model_prior'], pn)}"
        rt = f"{pct(sg['retrieved'], sn)} / {pct(pg['retrieved'], pn)}"
        print(f"{label:34} {f'{sn} / {pn}':>13}   {mp:>13}   {rt:>13}")

    # Range across arms, computed both ways. If the pinned range is much
    # narrower than the self-graded one, the spread was the instrument.
    def rng(idx_g, idx_n, key):
        vals = [pct(r[idx_g][key], r[idx_n]) for r in rows_out
                if r[idx_n]]
        vals = [v for v in vals if v is not None]
        return (min(vals), max(vals)) if vals else (None, None)

    s_lo, s_hi = rng(1, 2, "model_prior")
    p_lo, p_hi = rng(3, 4, "model_prior")
    print("-" * 82)
    print(f"model_prior range across arms:")
    print(f"    self-graded (each arm graded itself) : {s_lo}% .. {s_hi}%"
          f"   spread {round(s_hi - s_lo, 1) if s_lo is not None else '?'}pp")
    print(f"    pinned grader (one instrument)       : {p_lo}% .. {p_hi}%"
          f"   spread {round(p_hi - p_lo, 1) if p_lo is not None else '?'}pp")
    print("\nA pinned spread much narrower than the self-graded one means the "
          "range\nwas the instrument, not the agents. n per arm is small — "
          "read the counts.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
