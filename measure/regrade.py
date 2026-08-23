#!/usr/bin/env python3
"""Re-grade a trace window's claims with a PINNED grader.

    python3 measure/regrade.py --world venture_solo --dry-run
    python3 measure/regrade.py --world venture_solo
    python3 measure/regrade.py --world jill_chat --agent Jill \
            --since 2026-08-14 --limit 50

WHY THIS EXISTS. The live grader calls `self.backend.chat`, so the backend
under test grades its own output. Grounding numbers are therefore not
comparable across arms — the measuring instrument changes with the thing
measured, which is the one confound a comparison bench exists to remove.

WHY IT IS A SEPARATE PASS, not a change to the live path. Production
grading feeds the agent's own background verification concern, and that
works — it caught a mis-stated paper summary at turn 3007. It should keep
running on whatever backend the agent runs on. Measurement is a different
job with a different requirement, so it re-reads the trace offline.
`attribute_claims()` is pure and offline-runnable, which is what makes this
possible without touching the live loop.

Output goes to `measure/regraded/<world>.<agent>.jsonl`, NEVER to
`claims.jsonl`. The agent's own record is not overwritten by a measurement.

COSTS MONEY. The pinned grader is a cloud model, one call per turn with a
reply. `--dry-run` prints what would be graded and stops.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "src"))

from chat.backend import _ChatBackend                          # noqa: E402
from chat.claims import attribute_claims                       # noqa: E402

from measure.trace import load_turns, window                   # noqa: E402

OUT_DIR = REPO / "measure" / "regraded"

# THE PINNED INSTRUMENT. Changing any of this invalidates comparison with
# every number already produced — it is the instrument, not a parameter.
# Matches the cloud arm in scenarios/coord_search_luna.yaml.
GRADER = {
    "server": "local",          # _ChatBackend route 2: api_key set -> bearer
    "model": "gpt-5.6-luna",
    "base_url": "https://api.openai.com/v1",
    "api_key": "OPENAI_API_KEY",   # NAME of the env var, not the key
    "reasoning_effort": "low",
}

# Same floor the live path uses. A literal 1600 cut the JSON mid-object on
# replies with many quoted claims and dropped the whole pass.
_TOKEN_BUDGET = 8000


def make_grader():
    key_var = GRADER["api_key"]
    if not os.environ.get(key_var):
        raise SystemExit(
            f"regrade: ${key_var} is not set. The pinned grader is a cloud "
            f"model; export it or run with --dry-run.")
    backend = _ChatBackend(
        server=GRADER["server"], model=GRADER["model"],
        base_url=GRADER["base_url"], api_key=GRADER["api_key"],
        reasoning_effort=GRADER["reasoning_effort"],
    )

    def llm_chat(messages: List[Dict[str, str]], _attempt=[0]) -> str:
        # Retry once at double budget when the backend ran out of room,
        # mirroring the live caller. A partial beats nothing; a complete
        # pass beats a partial.
        _attempt[0] += 1
        cap = _TOKEN_BUDGET * 2 if _attempt[0] > 1 else _TOKEN_BUDGET
        # 0.0, NOT 0.2. An instrument that returns a different reading on
        # the same input is not an instrument. Observed 2026-08-23: one run
        # scored Tier 3 = 2 on four consecutive passes and Tier 3 = 3 on a
        # fifth, and failed the unsupported check on a sixth. Small compared
        # with the variance of the thing being measured — four runs of one
        # arm produced four different tool strategies — but it is noise in
        # the ruler rather than in the object, and it compounds every
        # comparison built on it.
        #
        # IT IS NOT SUFFICIENT. At 0.0 the same run still scored unsupported
        # = 0 twice and 1 twice across four consecutive passes. A cloud model
        # at temperature zero is not deterministic — no seed is sent, and
        # batching and reasoning-mode sampling both survive it. So this
        # narrows the grader's noise without removing it, and any threshold
        # that gates on a count this grader produces needs more than one
        # pass to mean anything.
        #
        # This changes the pinned instrument, and the header above says what
        # that costs. It is still worth it: the numbers it invalidates were
        # not reproducible either, which is a stronger objection than the one
        # the pinning rule exists to raise.
        return backend.chat(messages, max_tokens=cap, temperature=0.0,
                            cot_profile='none')

    return backend, llm_chat


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", required=True)
    ap.add_argument("--agent", required=True)
    ap.add_argument("--since")
    ap.add_argument("--until")
    ap.add_argument("--limit", type=int,
                    help="grade at most N turns (cost control)")
    ap.add_argument("--dry-run", action="store_true",
                    help="report what would be graded, call nothing")
    args = ap.parse_args()

    turns = window(load_turns(args.world, args.agent), args.since, args.until)
    gradable = [t for t in turns
                if t.produced_chars > 0
                and str(t.raw.get("raw_response") or "").strip() != "(no reply)"]
    if args.limit:
        gradable = gradable[:args.limit]

    print(f"{args.world}/{args.agent}: {len(turns)} turns in window, "
          f"{len(gradable)} with a reply to grade")
    print(f"pinned grader: {GRADER['model']} "
          f"(reasoning_effort={GRADER['reasoning_effort']})")
    if args.dry_run:
        chars = sum(t.produced_chars for t in gradable)
        print(f"dry run — would make {len(gradable)} cloud calls over "
              f"{chars:,} chars of reply. Nothing called.")
        return 0

    backend, llm_chat = make_grader()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    out_path = OUT_DIR / f"{args.world}.{args.agent}.jsonl"

    written = failed = 0
    with out_path.open("w", encoding="utf-8") as fh:
        for i, t in enumerate(gradable, 1):
            status: Dict[str, Any] = {}
            try:
                claims = attribute_claims(t.raw, llm_chat,
                                          character_name=args.agent,
                                          status=status)
            except Exception as e:                       # noqa: BLE001
                print(f"  [{i}/{len(gradable)}] turn {t.seq}: FAILED "
                      f"{type(e).__name__}: {e}")
                failed += 1
                continue
            if claims is None:
                # None is a pass that did not run; [] is a finding. The
                # distinction has to survive into the output or a failed
                # grade reads as "this reply made no claims".
                print(f"  [{i}/{len(gradable)}] turn {t.seq}: no result "
                      f"({status.get('error') or 'unknown'})")
                failed += 1
                continue
            fh.write(json.dumps({
                "turn_seq": t.seq,
                "ts": t.ts.isoformat() if t.ts else None,
                "grader": GRADER["model"],
                "claims": claims,
                "status": status,
            }, default=str) + "\n")
            written += 1
            if i % 10 == 0 or i == len(gradable):
                print(f"  [{i}/{len(gradable)}] graded")

    print(f"\nwrote {written} rows to {out_path}"
          + (f"   ({failed} failed)" if failed else ""))
    print("claims.jsonl was not touched.")
    return 0 if written else 1


if __name__ == "__main__":
    raise SystemExit(main())
