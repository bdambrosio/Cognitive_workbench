#!/usr/bin/env python3
"""Report the metric vector for one or more runs.

    python3 measure/report.py --world venture_solo
    python3 measure/report.py --world jill_chat --agent Jill \
            --since 2026-08-02 --until 2026-08-23
    python3 measure/report.py --worlds-matching 'coord_search.*'

Deliberately prints a VECTOR, never a single score. The v2 suite collapsed
`turn_taking` to 1.0 for an model whose trace recorded `premature_reply:
True` — the one case in the whole campaign where a probe separated the
models, discarded by the mean. Nothing here averages a discriminating fact
away.

There is no ship gate and no composite. Run it as a reporting instrument
until the numbers demonstrably separate.
"""

from __future__ import annotations

import argparse
import fnmatch
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from measure import process, provenance                        # noqa: E402
from measure.harness_rev import RevisionIndex                   # noqa: E402
from measure.trace import iter_agents, load_turns, window       # noqa: E402


def worlds_matching(pattern: str) -> list:
    root = REPO / "scenarios"
    return sorted(d.name for d in root.iterdir()
                  if d.is_dir() and fnmatch.fnmatch(d.name, pattern))


def report_one(world: str, agent: str, since, until, revs: RevisionIndex,
               show_commits: bool) -> None:
    turns = window(load_turns(world, agent), since, until)
    label = f"{world}/{agent}"
    if not turns:
        print(f"\n=== {label} === no turns in window")
        return

    print(process.render(process.build(turns), label))

    rows = provenance.load_claim_rows(world, agent)
    if rows:
        print(provenance.render(provenance.build(turns, rows), label))
    else:
        print(f"\n=== provenance — {label} ===\n  no claims.jsonl "
              f"(claim grading began 2026-08-02; earlier runs have none)")

    lo = min(t.ts for t in turns if t.ts)
    hi = max(t.ts for t in turns if t.ts)
    at_start, at_end = revs.at(lo), revs.at(hi)
    print(f"\n  harness at start  {at_start[0] if at_start else '?'} "
          f"{at_start[1][:60] if at_start else ''}")
    if at_end and at_start and at_end[0] != at_start[0]:
        landed = revs.span(lo, hi)
        print(f"  harness at end    {at_end[0]} {at_end[1][:60]}")
        print(f"  !! {len(landed)} harness commits landed DURING this window "
              f"— it is not a single revision")
        if show_commits:
            for when, sha, subject in landed:
                print(f"       {when:%Y-%m-%d} {sha} {subject[:64]}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world")
    ap.add_argument("--worlds-matching")
    ap.add_argument("--agent", help="default: every agent with a trace")
    ap.add_argument("--since")
    ap.add_argument("--until")
    ap.add_argument("--commits", action="store_true",
                    help="list harness commits that landed during the window")
    args = ap.parse_args()

    if not args.world and not args.worlds_matching:
        ap.error("give --world or --worlds-matching")

    worlds = ([args.world] if args.world
              else worlds_matching(args.worlds_matching))
    if not worlds:
        print("no matching worlds")
        return 1

    revs = RevisionIndex()
    for w in worlds:
        agents = [args.agent] if args.agent else list(iter_agents(w))
        if not agents:
            print(f"\n=== {w} === no agent traces")
            continue
        for a in agents:
            report_one(w, a, args.since, args.until, revs, args.commits)

    print("\nn=1 is anecdote. Two conclusions have already been retracted "
          "for treating a single run as a result.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
