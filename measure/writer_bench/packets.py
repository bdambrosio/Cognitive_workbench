#!/usr/bin/env python3
"""Pack each case's replies, under neutral labels, for blind fact-checking.

    python3 measure/writer_bench/packets.py <cases.jsonl> <out_dir> <label>=<results.jsonl> [...]

Each packet holds the case's context (what Bruce asked and what Jill read
that turn) and every reply for the case: the original live reply plus one
per (label, option), shuffled under letters. The letter -> source key is
written to key.json beside the packets and is not given to the checkers.
Packets are split into files of --per cases.
"""
from __future__ import annotations

import argparse
import json
import random
import string
from pathlib import Path


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("cases")
    ap.add_argument("out")
    ap.add_argument("results", nargs="+")
    ap.add_argument("--per", type=int, default=8)
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    cases = [json.loads(l) for l in Path(args.cases).open()]
    replies = {}
    for spec in args.results:
        label, path = spec.split("=", 1)
        for l in Path(path).open():
            r = json.loads(l)
            if r.get("reply"):
                replies.setdefault(r["turn_seq"], []).append((f"{label}:{r['option']}", r["reply"]))
    rng = random.Random(20261020)
    key, packets = {}, []
    for c in cases:
        items = [("original", c["original_reply"])] + replies.get(c["turn_seq"], [])
        rng.shuffle(items)
        letters = list(string.ascii_uppercase[:len(items)])
        key[str(c["turn_seq"])] = {L: src for L, (src, _) in zip(letters, items)}
        rec = c["trace_record"]
        packets.append({
            "turn_seq": c["turn_seq"], "ts": c["ts"],
            "user_input": rec.get("user_input"),
            "observations": (rec.get("working_log") or "")[:40000],
            "observations_full": rec.get("observations_full") or {},
            "replies": {L: text for L, (_, text) in zip(letters, items)}})
    for i in range(0, len(packets), args.per):
        with (out / f"packet{i // args.per + 1}.jsonl").open("w", encoding="utf-8") as fh:
            for p in packets[i:i + args.per]:
                fh.write(json.dumps(p) + "\n")
    (out / "key.json").write_text(json.dumps(key, indent=1))
    print(f"{len(packets)} cases in {(len(packets) + args.per - 1) // args.per} packets; key in {out / 'key.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
