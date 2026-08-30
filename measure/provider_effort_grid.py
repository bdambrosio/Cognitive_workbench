#!/usr/bin/env python3
"""Two providers serving one model id, across the reasoning-effort dial.

WHAT THIS ANSWERS. When two endpoints serve the SAME artifact, a difference in
tokens per call is the serving configuration and not the model. Fireworks
launched GLM-5.3-Flash on 2026-08-30 saying they had "unplugged peculiar
behaviors of over-thinking" before launch; both their endpoint and Modal's
report the artifact `z-ai/glm-5.3-flash-20260826`. That makes the claim
testable rather than a matter of trusting a launch post.

THE PROBE IS `prescreen.probe`, UNCHANGED. Same real REACT_ACTION_SCHEMA, same
max_tokens a real call uses, same omitted temperature, fallbacks off. This file
only decides what to call and how to read the rows back. Anything about the
request body belongs there, not here.

WHY n=3 AND NOT n=1. docs/model-prescreen.md is explicit that gate 2 is an
order of magnitude: one route sampled twice returned 727 and 1,877 completion
tokens on byte-identical requests. A single sample cannot separate a route from
a bad draw, and the effect being looked for here is not guaranteed to be 10x.

WHY THE ORDER INTERLEAVES. Providers are sampled inside each repeat rather than
in blocks, so an upstream condition that drifts over the minutes the grid runs
lands on both arms instead of on whichever went second.

SEQUENTIAL, ONE CALL AT A TIME. The 429 on this model names the model, not the
endpoint — the throttle is upstream of the provider split — so parallel calls
here contend with each other and with any run in flight on the same id.

Usage:
    export OPENROUTER_API_KEY=...
    python3 measure/provider_effort_grid.py                       # GLM default
    python3 measure/provider_effort_grid.py <model> <tagA,tagB> [reps]

Writes one JSON row per call to measure/provider_effort_grid.jsonl and prints a
summary. Rows are appended, never rewritten.
"""
from __future__ import annotations

import json
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from prescreen import probe                                   # noqa: E402

OUT = Path(__file__).resolve().parent / "provider_effort_grid.jsonl"

# None sends no `reasoning_effort` at all, which is the row where the
# over-thinking shows: measured 1623/1838/2367 completion tokens against
# 60/117/216 at `low` on modal/fp8 (or_glm53flash_high.yaml, 2026-08-28).
EFFORTS = (None, "low", "medium", "high")


def run(model: str, tags: list[str], reps: int) -> list[dict]:
    rows = []
    with OUT.open("a") as fh:
        for rep in range(1, reps + 1):
            for effort in EFFORTS:
                for tag in tags:
                    row = probe(model, tag, effort=effort)
                    row["rep"], row["effort"] = rep, effort or "none-sent"
                    rows.append(row)
                    fh.write(json.dumps(row) + "\n")
                    fh.flush()
                    label = f"{tag:14} {row['effort']:10} rep{rep}"
                    if row.get("status") != "ok":
                        print(f"  {label}  {row['status']}: "
                              f"{row.get('err', '')[:90]}")
                        continue
                    print(f"  {label}  ctok={row['completion_tok']:<6} "
                          f"rchars={row['reasoning_chars']:<6} "
                          f"finish={row['finish']:<10} "
                          f"valid={row['schema_valid']} "
                          f"served={row.get('provider')}")
    return rows


def summarise(rows: list[dict], tags: list[str]) -> None:
    ok = [r for r in rows if r.get("status") == "ok"]
    if not ok:
        print("\nNo successful calls — nothing to summarise.")
        return
    print(f"\n{'effort':12} {'provider':14} {'n':>2} "
          f"{'completion tokens':>26} {'reasoning chars':>22}  schema")
    for effort in [e or "none-sent" for e in EFFORTS]:
        for tag in tags:
            g = [r for r in ok if r["effort"] == effort and r["tag"] == tag]
            if not g:
                continue
            ctok = [r["completion_tok"] for r in g if r["completion_tok"]]
            rch = [r["reasoning_chars"] for r in g]
            valid = sum(1 for r in g if r["schema_valid"])
            med = f"med {statistics.median(ctok):.0f}" if ctok else "—"
            print(f"{effort:12} {tag:14} {len(g):>2} "
                  f"{'/'.join(str(c) for c in ctok):>16} {med:>9} "
                  f"{'/'.join(str(c) for c in rch):>22}  {valid}/{len(g)}")
    print("\nRead the spread, not the midpoint: per-call variance on this "
          "workload is at least 2.6x, so a gap under ~3x is not a finding.")
    print(f"Rows appended to {OUT}")


def main() -> int:
    model = sys.argv[1] if len(sys.argv) > 1 else "z-ai/glm-5.3-flash"
    # A FLAG IS NOT A MODEL ID. `--help` was accepted as one and fired all 24
    # calls before anything checked it. They 400'd, so that cost nothing — but
    # a real id in that position starts a grid against whatever is in flight on
    # the same model, which is the one thing this file must not do by accident.
    if model.startswith("-") or "/" not in model:
        print(f"{model!r} is not a model id (expected `publisher/name`).\n")
        print(__doc__)
        return 2
    tags = (sys.argv[2].split(",") if len(sys.argv) > 2
            else ["modal/fp8", "fireworks"])
    reps = int(sys.argv[3]) if len(sys.argv) > 3 else 3
    total = reps * len(EFFORTS) * len(tags)
    print(f"{model}: {len(tags)} providers x {len(EFFORTS)} efforts x "
          f"{reps} reps = {total} calls, sequential\n")
    summarise(run(model, tags, reps), tags)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
