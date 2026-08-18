#!/usr/bin/env python3
"""One table across every arm and probe that has been scored.

    python3 bench/report.py

Reads the summary.json each scorer wrote. Prints nothing it did not measure —
an arm that has not run is absent, not zero, and every row names the model the
server actually reported so a number can never drift loose from its backend.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List

HERE = Path(__file__).resolve().parent
PROBE_DIRS = ("convergence", "yield_probe", "tictactoe")


def _rows() -> List[Dict[str, Any]]:
    out = []
    for probe in PROBE_DIRS:
        for run in sorted((HERE / probe / "results").glob("*")):
            f = run / "summary.json"
            if not f.is_file():
                continue
            try:
                s = json.loads(f.read_text(encoding="utf-8"))
            except json.JSONDecodeError as e:
                print(f"  unreadable {f}: {e}")
                continue
            s["_probe_dir"] = probe
            s["_run"] = run.name
            out.append(s)
    return out


def main() -> int:
    rows = _rows()
    if not rows:
        print("no scored runs yet — run bench/run_probes.py first")
        return 0

    print("\nBENCHMARKS V2 — probes 1-3\n")
    hdr = (f"{'arm':<8} {'probe':<13} {'score':>7} {'wall_s':>8}  detail")
    print(hdr)
    print("-" * len(hdr) + "-" * 30)
    for s in sorted(rows, key=lambda r: (r.get("backend_arm") or "", r["_probe_dir"])):
        arm = s.get("backend_arm") or "?"
        wall = s.get("wall_clock_s")
        if "probe1_convergence" in s:
            c = s["probe1_convergence"]
            exits = (s.get("costs") or {}).get("exit_reasons")
            print(f"{arm:<8} {'convergence':<13} {c['score']:>7} {wall:>8}  "
                  f"{c['cells_correct']}/{c['cells_total']} cells "
                  f"(wrong={c['cells_wrong']} unstated={c['cells_not_stated']}) "
                  f"exits={exits}"
                  f"{'' if c.get('extraction_ok') else '  [EXTRACTION FAILED]'}")
        if "probe3_yield" in s:
            y = s["probe3_yield"]
            print(f"{arm:<8} {'yield':<13} {y['score']:>7} {wall:>8}  "
                  f"exit={y['exit_reasons']} continuation={y['continuation_spawned']} "
                  f"primed={y['continuation_primed_at_threshold']}")
        if "probe2_tictactoe" in s:
            t = s["probe2_tictactoe"]
            print(f"{arm:<8} {'tictactoe':<13} {t['score']:>7} {wall:>8}  "
                  f"legal={t['legal_move_rate']} "
                  f"blocks={t['forced_blocks_made']}/{t['forced_blocks_faced']} "
                  f"wins={t['wins_taken']}/{t['wins_available']} "
                  f"terminal={t['terminal_agreement']}")

    print("\nserved-model check (a row that cannot name its backend is not evidence):")
    for s in sorted(rows, key=lambda r: (r.get("backend_arm") or "")):
        chk = s.get("served_model_check") or {}
        print(f"  {s.get('backend_arm'):<8} {s['_probe_dir']:<13} "
              f"{chk.get('served')}")

    print("\nn=1 per cell. Gemma's three no-reasoning runs on 2026-08-16 disagreed")
    print("with each other more than the backends did on most measures — treat a")
    print("single run as anecdote, not a result.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
