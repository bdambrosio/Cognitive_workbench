#!/usr/bin/env python3
"""Render a form_grid JSONL as the K x N table, one line per cell.

    python3 measure/form_grid/read.py                 # newest grid
    python3 measure/form_grid/read.py grid_....jsonl

`full/asked` is the count of findings carrying all three §5 fields.
`collapse` is where the terminal run of broken findings starts — the point the
form stopped and did not come back. `chars` is output length up to that point,
because a finding is not a fixed amount of writing: on the field run Qwen spent
~930 characters on one and grok ~350, so an equal K is not equal output.
"""
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent


def main() -> int:
    if len(sys.argv) > 1:
        path = Path(sys.argv[1])
    else:
        grids = sorted(HERE.glob("grid_*.jsonl"))
        if not grids:
            raise SystemExit(f"no grid_*.jsonl in {HERE}")
        path = grids[-1]
    rows = [json.loads(l) for l in path.open(encoding="utf-8") if l.strip()]
    print(f"{path.name}  {len(rows)} cells\n")
    print(f"{'model':16} {'K':>3} {'N':>6} {'full/asked':>11} {'collapse':>9} "
          f"{'chars':>7} {'finish':>7} {'secs':>5}")
    for r in rows:
        if r.get("status") != "ok":
            print(f"{r['config'][:16]:16} {r['k']:>3} {r['n']:>6} "
                  f"{'—':>11} {str(r.get('status')):>9}")
            continue
        col = r.get("collapse_at")
        print(f"{r['config'][:16]:16} {r['k']:>3} {r['n']:>6} "
              f"{str(r.get('fully_formed')) + '/' + str(r['k']):>11} "
              f"{(str(col) if col else '—'):>9} "
              f"{r.get('chars_before_collapse', len(r.get('content') or '')):>7,} "
              f"{str(r.get('finish')):>7} {round(r.get('secs', 0)):>5}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
