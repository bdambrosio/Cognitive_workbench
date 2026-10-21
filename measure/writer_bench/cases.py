#!/usr/bin/env python3
"""Build the writer-bench cases: the exact prompt of the call that wrote each
selected reply, with the census findings for that reply.

    python3 measure/writer_bench/cases.py <census_dir> <out.jsonl>

<census_dir> holds the 2026-10-20 census: batch{1,2,3}.jsonl (the sampled
replies) and result{1,2,3}.jsonl (the fact-check of each). Selection, fixed
by seed: every FP4-Local turn with a false claim, old-quant turns with a
false claim up to 30 in all, and 10 turns with no false claim (5 per model).
The prompt is the last iteration of the turn's block in chat_trace.txt, the
one whose output was the `respond` action.
"""
from __future__ import annotations

import json
import random
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import chat_trace as T                                            # noqa: E402

SEED = 20261020


def main() -> int:
    census, out = Path(sys.argv[1]), Path(sys.argv[2])
    batch = {json.loads(l)["turn_seq"]: json.loads(l)
             for i in (1, 2, 3) for l in (census / f"batch{i}.jsonl").open()}
    result = {json.loads(l)["turn_seq"]: json.loads(l)
              for i in (1, 2, 3) for l in (census / f"result{i}.jsonl").open()}
    rng = random.Random(SEED)
    false = [s for s, r in result.items() if int(r.get("n_false") or 0) > 0]
    clean = [s for s, r in result.items() if int(r.get("n_false") or 0) == 0
             and int(r.get("n_claims") or 0) >= 2]
    fp4 = sorted(s for s in false if batch[s]["model"] == "fp4local")
    old = sorted(s for s in false if batch[s]["model"] == "nvfp4old")
    pick = fp4 + rng.sample(old, 30 - len(fp4))
    for m in ("fp4local", "nvfp4old"):
        pick += rng.sample(sorted(s for s in clean if batch[s]["model"] == m), 5)
    # The trace record's timestamp can trail the block header by a second
    # (6495: 20:17:03 against 20:17:02), so match within two seconds.
    from datetime import datetime, timedelta
    want = {}
    for s in pick:
        t = datetime.fromisoformat(batch[s]["ts"][:19])
        for d in (0, -1, 1, -2, 2):
            want.setdefault((t + timedelta(seconds=d)).strftime("%Y-%m-%d %H:%M:%S"), s)
    found = {}
    for b in T.blocks():
        s = want.get(b["ts"])
        if s is not None and s not in found and b["source"] == "User":
            calls = T.turn_calls(b["body"])
            found[s] = calls[-1]
    # The turn's reasoning-trace record: the pre-send check attributes a
    # draft against it exactly as the post-turn pass attributes the reply.
    records = {}
    trace = T.CHAT_TRACE.parent / "reasoning_trace.jsonl"
    with trace.open(encoding="utf-8") as fh:
        for line in fh:
            m = re.match(r'\{"turn_seq": (\d+),', line)
            if m and int(m.group(1)) in found:
                records[int(m.group(1))] = json.loads(line)
    with out.open("w", encoding="utf-8") as fh:
        for s in pick:
            if s not in found:
                print(f"turn {s}: no chat_trace block at {batch[s]['ts'][:19]}", file=sys.stderr)
                continue
            c = found[s]
            fh.write(json.dumps({
                "turn_seq": s, "ts": batch[s]["ts"], "model": batch[s]["model"],
                "tools": batch[s]["tools"], "system": c["system"], "user": c["user"],
                "original_action": c["assistant"], "original_reply": batch[s]["reply"],
                "census": result[s], "trace_record": records.get(s) or {}}) + "\n")
    print(f"{len(found)} of {len(pick)} cases written to {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
