#!/usr/bin/env python3
"""Score a duplicates.json against the hand-marked reference for chhoto.

    python3 measure/duplicates_reference/score.py <engagement>/surface/duplicates.json

THE REFERENCE. On 2026-09-17 Claude read the 173 claims enumerated from five
chhoto sources (claims/*.json, copied from the enumeration runs named in
reference.json) with no model output in view and marked 15 pairs that make
the same assertion, 19 pairs where a narrower claim is entailed by a wider
one, and 4 pairs where the seller's documents contradict each other. The
every-pair model pass it replaced had marked 25 pairs, 11 of them wrong by
that reading, so agreement with a model pass is not a measure of anything.

WHAT A SCORE MEANS. A duplicates.json is scored only when its pairs name the
enumeration runs in reference.json; ids differ between enumerations. "Found"
is a reference pair marked by the pass, in either direction. "Wrong" is a
mark that is in neither list, which the reader of this script judges by eye;
the two borderline reference calls are noted in reference.json. Lean towards
missing: a missed pair is audited twice, a wrong mark drops a claim from the
audit unless the practice keeps it on the surface page.
"""
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent


def main(path: str) -> int:
    ref = json.loads((HERE / "reference.json").read_text(encoding="utf-8"))
    got = json.loads(Path(path).read_text(encoding="utf-8"))
    by_source = {v["claim_source"]: k for k, v in ref["sources"].items()}
    stale = [s for s, v in ref["sources"].items()
             if got.get("runs", {}).get(v["claim_source"]) != v["enumeration_run"]]
    if stale:
        print(f"not scored: {path} was made on other enumeration runs than the "
              f"reference for {', '.join(stale)}")
        return 2
    claims = {}
    for k in ref["order"]:
        for c in json.loads((HERE / "claims" / f"{k}.json").read_text(encoding="utf-8"))["claims"]:
            claims[f"{k}{c['id']}"] = c["statement"]
    marks = {frozenset((f"{by_source[p['source']]}{p['id']}",
                        f"{by_source[p['same_as']['source']]}{p['same_as']['id']}")): p
             for p in got["pairs"]}
    same = {frozenset(p) for p in ref["same"]}
    ent = {frozenset(p) for p in ref["entailed_by"]}
    print(f"{len(marks)} marks: {len(same & set(marks))} of {len(same)} same pairs found, "
          f"{len(ent & set(marks))} of {len(ent)} entailed pairs marked, "
          f"{len(set(marks) - same - ent)} marks in neither list")
    for k in sorted(set(marks) - same - ent, key=str):
        a, b = sorted(k)
        print(f"  ? {a} ~ {b}: {claims[a][:70]} | {claims[b][:70]}")
    for k in sorted(same - set(marks), key=str):
        a, b = sorted(k)
        print(f"  missed same: {a} ~ {b}")
    return 0


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit(__doc__)
    raise SystemExit(main(sys.argv[1]))
