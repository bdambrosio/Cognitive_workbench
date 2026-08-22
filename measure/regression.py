#!/usr/bin/env python3
"""Did a harness change move the agent's behaviour?

    python3 measure/regression.py --world jill_chat --agent Jill \
            --since 2026-08-01 --bucket 40

Splits a trace into consecutive buckets of N turns, prints the process
metrics for each, and annotates which harness commits landed inside each
bucket. Process metrics only — they are computed from the trace and need no
grader, so this costs nothing and has no recall ceiling.

WHY BUCKETS AND NOT BEFORE/AFTER-ONE-COMMIT. 52 commits touching src/ landed
in the 2026-08-14..22 jill_chat window alone. At that density a before/after
split around a single commit cannot attribute a change to it — everything
else moved too. So this does NOT claim causation. It shows the series and
names the candidates, and a step change is a place to go and look, not a
finding.

Read a difference against run-to-run variance, which is still unmeasured for
this agent. Two conclusions have already been retracted in this project for
treating one run as a result.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import List

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from measure import process                                    # noqa: E402
from measure.harness_rev import RevisionIndex                   # noqa: E402
from measure.trace import Turn, load_turns, window              # noqa: E402


def buckets(turns: List[Turn], n: int) -> List[List[Turn]]:
    return [turns[i:i + n] for i in range(0, len(turns), n)]


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", required=True)
    ap.add_argument("--agent", required=True)
    ap.add_argument("--since")
    ap.add_argument("--until")
    ap.add_argument("--bucket", type=int, default=40,
                    help="turns per bucket (default 40)")
    ap.add_argument("--commits", action="store_true",
                    help="name the harness commits landing in each bucket")
    args = ap.parse_args()

    turns = window(load_turns(args.world, args.agent), args.since, args.until)
    if not turns:
        print("no turns in window")
        return 1
    revs = RevisionIndex()

    print(f"\n{args.world}/{args.agent} — {len(turns)} turns, "
          f"buckets of {args.bucket}\n")
    hdr = (f"{'from':>10} {'turns':>5} {'auton':>5} {'toolless':>8} "
           f"{'reauth':>6} {'maxit':>5} {'errs':>5} {'yield':>5} "
           f"{'iters/t':>7} {'commits':>7}")
    print(hdr)
    print("-" * len(hdr))

    for b in buckets(turns, args.bucket):
        rep = process.build(b)
        lo = min(t.ts for t in b if t.ts)
        hi = max(t.ts for t in b if t.ts)
        landed = revs.span(lo, hi)
        ipt = round(rep.total_iterations / max(len(b), 1), 1)
        print(f"{lo:%m-%d %H:%M} {len(b):>5} {rep.autonomous:>5} "
              f"{rep.toolless_replies:>8} "
              f"{(rep.re_authoring if rep.re_authoring is not None else 0):>6} "
              f"{rep.exit_reasons.get('max_iters', 0):>5} "
              f"{rep.exit_reasons.get('llm_error', 0):>5} "
              f"{rep.exit_reasons.get('yield', 0):>5} "
              f"{ipt:>7} {len(landed):>7}")
        if args.commits and landed:
            for when, sha, subject in landed:
                print(f"           . {when:%m-%d} {sha} {subject[:58]}")

    print("\nNo causation is claimed. Commit density in this window is high "
          "enough that\na step change names candidates, not a cause — go and "
          "look at what landed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
