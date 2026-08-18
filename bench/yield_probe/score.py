#!/usr/bin/env python3
"""Score probe 3 — yield. Entirely mechanical, no instrument involved.

    python bench/yield_probe/score.py --run-dir bench/yield_probe/results/<ts>_<arm>

Three components, equally weighted:

  yielded                            did the turn hand off rather than stop
  continuation_spawned               did a one-shot urgency concern get created
  continuation_primed_at_threshold   at 0.70, not 0.0

The third is the one that bites. Note_19 was created at activation 0.000
against a 1h rhythm and never fired — a conversational beat scheduled on the
same clock as a daily security patrol. A yield whose continuation is unprimed
has lost the work just as surely as one that spawns nothing.

Creation is asserted, not firing: creation happens in-turn and needs no
autonomy, while firing is gated unconditionally at chat_loop.py:2211. So this
scores correctly with the bench's autonomy off, which is also the only safe
setting — the bench must never mutate live resource-manager state.

NOTHING HERE GRADES THE REPLY'S CONTENT. That is probe 1's job. Asking one
turn to both yield and finish is what made the merged version incoherent and
scored Gemma 0/16 for correctly doing as it was told.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, List

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
sys.path.insert(0, str(REPO))


def score_yield(trace: List[Dict[str, Any]],
                concerns: List[Dict[str, Any]]) -> Dict[str, Any]:
    exits = [t.get("exit_reason") for t in trace]
    yielded = "yield" in exits
    spawned = [c for c in concerns
               if c.get("category") == "one_shot"
               and not c.get("seed")
               and c.get("rhythm_source") == "urgency"]
    primed = [c for c in spawned
              if abs(float(c.get("activation") or 0.0) - 0.70) < 0.01]
    return {
        "exit_reasons": exits,
        "yielded": yielded,
        "iters_at_exit": [t.get("iters") for t in trace],
        "hit_max_iters": "max_iters" in exits,
        "continuation_spawned": bool(spawned),
        "continuation_primed_at_threshold": bool(primed),
        "continuation_count": len(spawned),
        "continuations": [{k: c.get(k) for k in
                           ("note_id", "activation", "rhythm_hours",
                            "rhythm_source", "category", "system_spawned")}
                          for c in spawned],
        "score": round(sum([yielded, bool(spawned), bool(primed)]) / 3.0, 4),
        "_score_note": "yielded + continuation spawned + primed at threshold",
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", type=Path, required=True)
    args = ap.parse_args()

    raw = json.loads((args.run_dir / "raw.json").read_text(encoding="utf-8"))
    meta = json.loads((args.run_dir / "run_meta.json").read_text(encoding="utf-8"))
    y = score_yield(raw.get("trace") or [], raw.get("agent_concerns") or [])

    summary = {
        "backend_arm": meta.get("backend_arm"),
        "backend_label": meta.get("backend_label"),
        "served_model_check": meta.get("served_model_check"),
        "wall_clock_s": meta.get("wall_clock_s"),
        "costs": meta.get("costs"),
        "probe3_yield": y,
    }
    (args.run_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, default=str) + "\n", encoding="utf-8")

    print(f"\n=== {meta.get('backend_label')} ===")
    print(f"  exit reasons        {y['exit_reasons']} (iters {y['iters_at_exit']})")
    print(f"  yielded             {y['yielded']}")
    print(f"  continuation        spawned={y['continuation_spawned']} "
          f"primed={y['continuation_primed_at_threshold']}")
    print(f"  score               {y['score']}   wall={meta.get('wall_clock_s')}s")
    print(f"  wrote {args.run_dir / 'summary.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
