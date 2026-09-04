#!/usr/bin/env python3
"""Delete the worlds under scenarios/ that a finished workflow stage left behind.

    python3 workflowsv2/sweep_worlds.py            # list what would go
    python3 workflowsv2/sweep_worlds.py --delete   # delete it
    python3 workflowsv2/sweep_worlds.py --days 14 --delete

THE RULE (Bruce, 2026-09-04). A run's record is its run directory under
engagements/<e>/runs/ and its merged directory: the runner copies the
working record there when the run ends, so the world the stage ran in adds
nothing once the run directory exists. Such a world is scratch and is deleted
when it is older than `--days` (default 30). Run and merged directories are
never touched here; they live as long as the engagement's retention term and
go when the engagement goes.

WHAT IS KEPT, by name, whatever its age:

  jill_*            the companion worlds (jill_chat and its archives, jill_factorio),
                    not workflow artefacts
  post_*            post-delivery conversations, resumed by design
  intake_*          intake conversations, same
  demo_*            the public demo's visitor worlds; src/demo/app.py sweeps
                    these on its own schedule (demo.yaml keep_days)

Everything else under scenarios/ is a stage world or a bench world. Age is the
directory's modification time, which the world's own writes keep current, so a
world still in use is never old.
"""
from __future__ import annotations

import argparse
import shutil
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SCENARIOS = REPO / "scenarios"
KEEP_PREFIXES = ("jill_", "post_", "intake_", "demo_")


def candidates(days: int) -> list[tuple[Path, float]]:
    cutoff = time.time() - days * 86400
    out = []
    for d in sorted(SCENARIOS.iterdir()) if SCENARIOS.is_dir() else []:
        if not d.is_dir() or d.name.startswith(KEEP_PREFIXES):
            continue
        # The newest write anywhere inside, not the directory entry alone:
        # a world's own files change while it is in use.
        newest = max((p.stat().st_mtime for p in d.rglob("*")), default=d.stat().st_mtime)
        newest = max(newest, d.stat().st_mtime)
        if newest < cutoff:
            out.append((d, newest))
    return out


def size_of(d: Path) -> int:
    return sum(p.stat().st_size for p in d.rglob("*") if p.is_file())


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--days", type=int, default=30,
                    help="delete worlds whose newest file is older than this (default 30)")
    ap.add_argument("--delete", action="store_true",
                    help="delete; without it, list only")
    args = ap.parse_args(argv)
    found = candidates(args.days)
    total = 0
    for d, newest in found:
        n = size_of(d)
        total += n
        age = int((time.time() - newest) / 86400)
        print(f"{'delete' if args.delete else 'would delete'}  {d.name:50s} {age:4d} d  {n / 1e6:7.1f} MB")
        if args.delete:
            shutil.rmtree(d)
    print(f"{len(found)} world(s), {total / 1e6:.1f} MB"
          + ("" if args.delete else " — pass --delete to remove them"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
