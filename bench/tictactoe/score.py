#!/usr/bin/env python3
"""Score probe 2. Fully mechanical — no model, no judge, no network.

    python bench/tictactoe/score.py --run-dir bench/tictactoe/results/<ts>_<arm>

Every metric is a fact about a recorded position, so this is re-runnable and
deterministic. Re-scoring an old run after changing a metric costs nothing.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "src"))

from bench.tictactoe import game  # noqa: E402


def score(raw: Dict[str, Any]) -> Dict[str, Any]:
    plies = [p for p in raw.get("plies") or [] if p.get("mover") == "X"]

    fmt_violations = sum(1 for p in plies if p.get("format_violation"))
    illegal = sum(1 for p in plies if p.get("illegal_move"))

    # Precedence: a win on offer outranks a block. Scoring a missed block on a
    # ply where taking the win was correct would penalise the right move.
    wins_faced = wins_taken = 0
    blocks_faced = blocks_made = 0
    for p in plies:
        played = p.get("played")
        avail = p.get("wins_available") or []
        req = p.get("blocks_required") or []
        if avail:
            wins_faced += 1
            if played in avail:
                wins_taken += 1
        elif req:
            blocks_faced += 1
            if played in req:
                blocks_made += 1

    true_result = raw.get("true_result")
    claimed = (raw.get("claimed_result") or "").strip().lower()
    terminal_ok = bool(claimed) and claimed == str(true_result).strip().lower()

    n = len(plies) or 1
    legal_rate = round((n - fmt_violations - illegal) / n, 4)
    block_rate = round(blocks_made / blocks_faced, 4) if blocks_faced else None
    win_rate = round(wins_taken / wins_faced, 4) if wins_faced else None

    # Composite: the three things that matter, each in [0,1], equally weighted.
    # A None component (situation never arose) drops out rather than counting
    # as zero — a game with no forced block did not fail to block.
    parts = [legal_rate, block_rate, win_rate, 1.0 if terminal_ok else 0.0]
    parts = [p for p in parts if p is not None]
    return {
        "plies_by_agent": len(plies),
        "format_violations": fmt_violations,
        "illegal_moves": illegal,
        "legal_move_rate": legal_rate,
        "forced_blocks_faced": blocks_faced,
        "forced_blocks_made": blocks_made,
        "forced_block_rate": block_rate,
        "wins_available": wins_faced,
        "wins_taken": wins_taken,
        "win_take_rate": win_rate,
        "true_result": true_result,
        "claimed_result": raw.get("claimed_result"),
        "terminal_agreement": terminal_ok,
        "score": round(sum(parts) / len(parts), 4),
        "_score_note": ("mean of legal_move_rate, forced_block_rate, "
                        "win_take_rate, terminal_agreement; components whose "
                        "situation never arose are omitted, not zeroed"),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", type=Path, required=True)
    args = ap.parse_args()

    raw = json.loads((args.run_dir / "raw.json").read_text(encoding="utf-8"))
    meta = json.loads((args.run_dir / "run_meta.json").read_text(encoding="utf-8"))
    s = score(raw)

    from bench.common import scan_validity  # noqa: E402
    summary = {
        "validity": scan_validity(meta.get("captured_at"),
                                  meta.get("wall_clock_s")),
        "backend_arm": meta.get("backend_arm"),
        "backend_label": meta.get("backend_label"),
        "served_model_check": meta.get("served_model_check"),
        "wall_clock_s": meta.get("wall_clock_s"),
        "costs": meta.get("costs"),
        "probe2_tictactoe": s,
    }
    (args.run_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, default=str) + "\n", encoding="utf-8")

    print(f"\n=== {meta.get('backend_label')} ===")
    print(game.render({int(k): v for k, v in raw["final_board"].items()}))
    print(f"  legal moves        {s['legal_move_rate']} "
          f"(format={s['format_violations']} illegal={s['illegal_moves']})")
    print(f"  forced blocks      {s['forced_blocks_made']}/{s['forced_blocks_faced']}")
    print(f"  wins taken         {s['wins_taken']}/{s['wins_available']}")
    print(f"  terminal agreement {s['terminal_agreement']} "
          f"(true={s['true_result']} claimed={s['claimed_result']})")
    print(f"  score              {s['score']}   wall={meta.get('wall_clock_s')}s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
