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
sys.path.insert(0, str(REPO / "src"))


def _answer_is_complete(reply: str) -> Dict[str, Any]:
    """Did the turn actually finish the task? Uses probe 1's rubric on THIS
    run's reply — not on probe 1's separate run, which is a different world.

    Complete means all sixteen cells correct. Anything less and work genuinely
    remained, so declining to yield was a real miss.
    """
    import json as _json
    from bench.convergence.score import score_convergence  # noqa: E402
    truth = _json.loads((REPO / "bench" / "convergence" /
                         "ground_truth.json").read_text(encoding="utf-8"))
    conv = score_convergence(reply or "", truth)
    return {"cells_correct": conv["cells_correct"],
            "cells_total": conv["cells_total"],
            "complete": conv["cells_correct"] == conv["cells_total"],
            "extraction_ok": conv["extraction_ok"]}


def score_yield(trace: List[Dict[str, Any]],
                concerns: List[Dict[str, Any]],
                reply: str = "") -> Dict[str, Any]:
    exits = [t.get("exit_reason") for t in trace]
    yielded = "yield" in exits
    spawned = [c for c in concerns
               if c.get("category") == "one_shot"
               and not c.get("seed")
               and c.get("rhythm_source") == "urgency"]
    primed = [c for c in spawned
              if abs(float(c.get("activation") or 0.0) - 0.70) < 0.01]
    # A run that did not yield scores 0 ONLY if work remained. An agent that
    # finished the task had nothing to hand forward, and the prompt says
    # "yield rather than stop when you run out of room" — never running out
    # of room is not a failure to yield. Both Qwen arms hit this on
    # 2026-08-18 and were scored 0.0 for completing the work.
    completeness = None
    if not yielded and (reply or "").strip():
        completeness = _answer_is_complete(reply)
    elif not yielded:
        # No reply at all cannot be a completed answer, and calling the
        # extractor on an empty string would spend a request to learn nothing.
        completeness = {"cells_correct": 0, "cells_total": 16,
                        "complete": False, "extraction_ok": None,
                        "note": "empty reply — not scored by the extractor"}

    if yielded:
        score = round(sum([yielded, bool(spawned), bool(primed)]) / 3.0, 4)
        applies = True
    elif completeness and completeness["complete"]:
        score = None                     # N/A — nothing to hand off
        applies = False
    else:
        score = 0.0                      # stopped with work remaining
        applies = True

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
        "answer_completeness": completeness,
        "probe_applies": applies,
        "score": score,
        "_score_note": ("yielded + continuation spawned + primed at threshold; "
                        "score is None (N/A) when the turn did not yield "
                        "because it had already completed the task"),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", type=Path, required=True)
    args = ap.parse_args()

    raw = json.loads((args.run_dir / "raw.json").read_text(encoding="utf-8"))
    meta = json.loads((args.run_dir / "run_meta.json").read_text(encoding="utf-8"))
    y = score_yield(raw.get("trace") or [], raw.get("agent_concerns") or [],
                    raw.get("reply") or "")

    from bench.common import scan_validity  # noqa: E402
    summary = {
        "validity": scan_validity(meta.get("captured_at"),
                                  meta.get("wall_clock_s")),
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
    if y.get("answer_completeness"):
        c = y["answer_completeness"]
        print(f"  answer completeness {c['cells_correct']}/{c['cells_total']} "
              f"-> {'COMPLETE, probe N/A' if c['complete'] else 'incomplete, real miss'}")
    print(f"  score               {y['score'] if y['score'] is not None else 'N/A'}"
          f"   wall={meta.get('wall_clock_s')}s")
    print(f"  wrote {args.run_dir / 'summary.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
