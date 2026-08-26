#!/usr/bin/env python3
"""Enumerate the claim surface and stop. One leg, no verification.

    python3 measure/fixtures/dataroom/claims.py --world c1_grok --model measure/models/grok_4p6.yaml

WHAT THIS IS FOR. The claim count is the denominator every coverage figure in
a report divides by, and it has not been stable: given the same nine documents
and the same instructions, three backends returned 62, 67 and 273 (2026-08-25).
Two attempts to fix that by defining a claim more precisely made it worse, then
different. Naming the claim sources per engagement (§12 step 1) is the third
approach, and this script measures it directly instead of inferring it from a
full audit.

It runs one leg, captures what the agent enumerated, and stops. About a tenth
of the wall clock of a scored run, and it isolates the one variable.

NOT A SCORED RUN. There is no report, no Gap Map and nothing for score.py to
grade. The output is the count, the claim text, and where the agent said each
claim came from.
"""

from __future__ import annotations

import argparse
import json
import logging
import re
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
for p in (str(REPO), str(REPO / "src")):
    if p not in sys.path:
        sys.path.insert(0, p)

# Reused rather than reimplemented: the config build, the reply reader and the
# exit-reason reader are the same machinery a scored run uses, and a second
# copy would drift from it.
from workflows.claims_audit.runner import (                    # noqa: E402
    build_config, latest_reply, last_exit_reason, SOURCE, CORPUS,
)

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s")
logger = logging.getLogger("dataroom.claims")

SURFACE_MARK = "=== CLAIM SURFACE ==="

# The claim sources are named here for the same reason they are named in the
# scored run's brief: they are an engagement fact, not something to infer.
BRIEF = """The target is the data room bound to inspect_external: nine
documents for a small SaaS business called flowmetrics, offered for sale by a
seller named Dave. I am the buyer's side.

The claim sources are doc1 (the listing), doc2 (the tech stack description)
and doc9 (the technical claims). Those are the documents Dave asserts things
in. The other six are evidence.

Enumerate the claim surface and close it as the method says. Do not verify
anything: I want the claim surface itself, not an audit.

After the count, list every claim you counted, one per line, each with the
document and line it came from. Then stop and end your turn.
"""


def extract(world: str, agent: str) -> dict:
    """The count the agent committed to, and the text it enumerated.

    Reads the trace rather than the reply because the marker's meaning is
    positional — it records where enumeration ended — and only the trace knows
    which leg and iteration that was.
    """
    from measure.trace import load_turns
    turns = load_turns(world, agent)
    for i, t in enumerate(turns, 1):
        body = f"{t.working_log or ''}\n{t.raw.get('raw_response') or ''}"
        if SURFACE_MARK not in body:
            continue
        tail = body.split(SURFACE_MARK, 1)[1]
        m = re.search(r"(\d[\d,]*)\s+\S*claims\b", tail, re.I)
        # A MARKER WITHOUT A COUNT IS NOT A CLOSURE, and it is often not even
        # an emission: an model that quotes the instruction back into a tool
        # query puts the literal marker in its own working log. Declaring on
        # the marker alone reported declared=True / count=None for a run whose
        # five inspection attempts all returned nothing. Same rule as
        # score.py's claim_surface — keep looking, and declare only on a count.
        if not m:
            continue
        return {
            "declared": True,
            "leg": i,
            "count": int(m.group(1).replace(",", "")),
            # NOT TRUNCATED AT 20k. That cap cut a 244-claim enumeration
            # mid-listing, losing one of three documents entirely and
            # under-reporting the per-document breakdown by 81 lines. The
            # listing is the artifact this script exists to capture.
            "text": tail[:400000],
        }
    return {"declared": False, "leg": None, "count": None, "text": ""}


# Reused from overlap.py rather than kept as a second copy: this one matched
# only the fixture's `docN` form and returned {} for every real target, whose
# models cite `README.md:9`. One parser, one set of fixes.
from measure.fixtures.dataroom.overlap import per_document   # noqa: E402,F401


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", required=True, help="fresh world name; never reuse one")
    ap.add_argument("--model", type=Path, default=None,
                    help="YAML with an llm_config block; replaces the scenario's")
    ap.add_argument("--brief-file", type=Path, default=None,
                    help="brief for a target other than the fixture")
    ap.add_argument("--external-repo", type=Path, default=None,
                    help="enumerate a real target instead of the fixture corpus")
    args = ap.parse_args()

    if (REPO / "scenarios" / args.world).exists():
        raise SystemExit(f"world '{args.world}' already exists — use a fresh name")
    if args.external_repo and not args.brief_file:
        raise SystemExit("--external-repo needs --brief-file: the built-in "
                         "brief names flowmetrics and its nine documents")

    ts = time.strftime("%Y-%m-%dT%H-%M-%SZ", time.gmtime())
    out = HERE / "claims" / f"{ts}_{args.world}"
    out.mkdir(parents=True, exist_ok=True)

    name, cfg = build_config(args.world, args.model, None, None, None,
                             args.external_repo)

    from chat.chat_loop import ChatLoop                        # noqa: E402
    from chat.model_params import TOP_P                        # noqa: E402
    loop = ChatLoop(character_name=name, character_config=cfg)
    resolved_model = loop.backend.resolved_model()
    resolved_temperature = ((cfg.get("chat") or {}).get("react_temperature")
                            or loop.backend.temperature_for_model())
    logger.info("world=%s model=%s temperature=%s top_p=%s",
                args.world, resolved_model, resolved_temperature, TOP_P)

    t0, error = time.time(), None
    try:
        text = (args.brief_file.read_text(encoding="utf-8")
                if args.brief_file else BRIEF)
        loop._process_user_turn(source=SOURCE, text=text, close=False)
        reply = latest_reply(loop, SOURCE)
        exit_reason = last_exit_reason(args.world, name)
        # ONE LEG, WHATEVER HAPPENS. A scored run continues until the Gap Map
        # appears; this one wants the surface and nothing after it, so a
        # `yield` here is a finished job rather than a reason to continue.
        if exit_reason in ("llm_error", "crashed"):
            error = f"turn ended {exit_reason} — enumeration is not valid"
    except Exception as e:                                     # noqa: BLE001
        error, reply, exit_reason = f"{type(e).__name__}: {e}", "", None
        logger.exception("run failed")
    finally:
        try:
            loop._post_turn_executor.shutdown(wait=True)
        except Exception as e:                                 # noqa: BLE001
            logger.warning("executor shutdown failed: %s", e)

    surface = extract(args.world, name)
    meta = {
        "world": args.world,
        "model_config": str(args.model) if args.model else None,
        "resolved_model": resolved_model,
        "resolved_temperature": resolved_temperature,
        "top_p": TOP_P,
        "external_repo": cfg.get("external_repo") or str(CORPUS),
        "exit_reason": exit_reason,
        "wall_clock_s": round(time.time() - t0, 1),
        "error": error,
        "declared": surface["declared"],
        "count": surface["count"],
        "leg": surface["leg"],
        "per_document": per_document(surface["text"]),
    }
    (out / "meta.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
    (out / "surface.md").write_text(
        f"# claim surface — {args.world} ({resolved_model})\n\n"
        f"count: {surface['count']}\n\n```\n{surface['text']}\n```\n",
        encoding="utf-8")
    (out / "reply.md").write_text(reply or "", encoding="utf-8")

    print(f"\n  {resolved_model}: count={surface['count']} "
          f"declared={surface['declared']} exit={exit_reason} "
          f"{meta['wall_clock_s']}s error={error}")
    print(f"  per document: {meta['per_document']}")
    print(f"  {out}")
    return 1 if error else 0


if __name__ == "__main__":
    raise SystemExit(main())
