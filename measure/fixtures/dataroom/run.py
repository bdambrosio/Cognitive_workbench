#!/usr/bin/env python3
"""Drive one dataroom audit run, unattended.

    python3 measure/fixtures/dataroom/run.py --world dataroom_1
    python3 measure/fixtures/dataroom/run.py --world dataroom_2 \
            --arm measure/arms/nemotron_fp8.yaml --max-turns 20

Loads `scenarios/audit.yaml`, points `inspect_external` at the fixture
corpus, gives the brief as the opening turn, then drives "continue" for as
long as the agent keeps yielding. Writes the reply and the trace next to the
run so `score.py` can read them.

WHY A RUNNER AND NOT --cli. The interactive path needs a human to paste the
brief and press return between legs. A benchmark needs the same input every
time, which a human cannot supply; three runs typed by hand are three
different runs.

WHY autonomy IS OFF. Continuations are driven here rather than by the tick
sensor, so a run is deterministic in its leg count and a stall is visible as
a stall rather than as a missing tick. Concern CREATION happens in-turn and
needs no autonomy; only FIRING is gated.

WHY THE ARM REPLACES llm_config RATHER THAN MERGING. A merge lets a stale
field from the scenario — an api_key, a reasoning_effort — survive into an
arm that never declared one, which is the silent second variable this exists
to avoid. Shape and rationale recovered from the retired bench/common.py.
"""

from __future__ import annotations

import argparse
import datetime
import json
import logging
import sys
import time
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "src"))

import yaml                                                    # noqa: E402

SCENARIO = REPO / "scenarios" / "audit.yaml"
CORPUS = HERE / "corpus"
SOURCE = "User"

logging.basicConfig(level=logging.WARNING)
logger = logging.getLogger("dataroom.run")
logger.setLevel(logging.INFO)

BRIEF = f"""Read audit/METHOD.md — that is your method for this engagement.

The target is the data room bound to inspect_external: nine documents for a
small SaaS business called flowmetrics, offered for sale by a seller named
Dave. I am the buyer's side. Audit it.

Enumerate the claim surface first and tell me roughly how big it is, then
work the priority order. Recap as the method says. Stop and tell me if you
find a delta.

Where a claim turns on a number, a date, or a threshold, quote the source
line verbatim rather than working from a summary of the document. Summaries
carry the qualitative facts and drop the figures, and nothing marks the
omission.

When you are done, give me the report: a recommendation, the findings worst
first with a document citation on each, and what I should ask Dave before
closing. Keep it under 900 words — what earns the page is your call.
"""

CONTINUE = "continue"


def build_config(world: str, arm_path: Optional[Path]
                 ) -> Tuple[str, Dict[str, Any]]:
    from launcher import parse_characters                      # noqa: E402

    scenario = yaml.safe_load(SCENARIO.read_text(encoding="utf-8")) or {}
    scen_llm = dict(scenario.get("llm_config") or {})

    arm_llm = None
    if arm_path:
        arm = yaml.safe_load(Path(arm_path).read_text(encoding="utf-8")) or {}
        arm_llm = dict(arm.get("llm_config") or {})
        if not arm_llm:
            raise SystemExit(f"{arm_path}: no llm_config block")
        for char in (scenario.get("characters") or {}).values():
            if isinstance(char, dict) and char.get("mode") == "chat":
                char["llm_config"] = dict(arm_llm)   # REPLACE, never merge
        scen_llm.update(arm_llm)

    world_cfg = dict(scenario.get("world_config") or {})
    world_cfg["world_name"] = world

    chars = parse_characters(scenario, scen_llm, world_cfg,
                             scenario.get("setting", ""),
                             scenario.get("alt_llm_config") or {})
    chat_chars = [(n, c) for n, c in chars if c.get("mode") == "chat"]
    if len(chat_chars) != 1:
        raise SystemExit(f"expected 1 chat character, found {len(chat_chars)}")
    name, cfg = chat_chars[0]
    cfg["autonomy_enabled"] = False
    # The two per-target values, set here rather than by editing the committed
    # scenario, so a run leaves no diff behind.
    cfg["external_repo"] = str(CORPUS)
    return name, cfg


def last_exit_reason(world: str, agent: str) -> Optional[str]:
    """Read it off the trace, not the loop. `exit_reason` is a local inside
    _process_user_turn and is never stored on the object — getattr for it
    returns None, which reads as "not a yield" and would end every run after
    one leg."""
    p = (REPO / "scenarios" / world / agent / "memory" /
         "reasoning_trace.jsonl")
    if not p.exists():
        return None
    last = None
    for line in p.open(errors="replace"):
        line = line.strip()
        if line:
            last = line
    if not last:
        return None
    try:
        return json.loads(last).get("exit_reason")
    except json.JSONDecodeError:
        return None


def latest_reply(loop, source: str) -> str:
    turns = loop.store.get_recent_turns(source, limit=4, scope="all")
    for t in reversed(turns):
        if str(t.get("name", "")) != source:
            return str(t.get("text", ""))
    return ""


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", required=True,
                    help="fresh world name; never reuse one")
    ap.add_argument("--arm", type=Path, default=None,
                    help="YAML with an llm_config block; replaces the scenario's")
    ap.add_argument("--max-turns", type=int, default=25,
                    help="hard cap on legs (default 25)")
    args = ap.parse_args()

    if (REPO / "scenarios" / args.world).exists():
        raise SystemExit(
            f"world '{args.world}' already exists — a second run in one world "
            f"inherits the first's conclusions as memories. Pick a fresh name.")

    ts = datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H-%M-%SZ")
    out = HERE / "results" / f"{ts}_{args.world}"
    out.mkdir(parents=True, exist_ok=True)

    name, cfg = build_config(args.world, args.arm)
    logger.info("world=%s arm=%s model=%s", args.world, args.arm,
                (cfg.get("llm_config") or {}).get("model") or "(scenario default)")

    from chat.chat_loop import ChatLoop                        # noqa: E402
    loop = ChatLoop(character_name=name, character_config=cfg)

    t0 = time.time()
    legs, error = [], None
    try:
        text = BRIEF
        for i in range(args.max_turns):
            loop._process_user_turn(source=SOURCE, text=text, close=False)
            reply = latest_reply(loop, SOURCE)
            exit_reason = last_exit_reason(args.world, name)
            legs.append({"leg": i + 1, "sent": text[:80],
                         "exit_reason": exit_reason,
                         "reply_chars": len(reply)})
            logger.info("leg %d: exit=%s chars=%d", i + 1, exit_reason,
                        len(reply))
            if exit_reason != "yield":
                break
            text = CONTINUE
    except Exception as e:                                     # noqa: BLE001
        error = f"{type(e).__name__}: {e}"
        logger.exception("run failed")
    finally:
        try:
            loop._post_turn_executor.shutdown(wait=True)
        except Exception as e:                                 # noqa: BLE001
            logger.warning("executor shutdown failed: %s", e)
        try:
            loop._persist_to_disk()
        except Exception as e:                                 # noqa: BLE001
            logger.warning("final persist failed: %s", e)

    wall = round(time.time() - t0, 1)
    (out / "run_meta.json").write_text(json.dumps({
        "world": args.world,
        "arm": str(args.arm) if args.arm else "(scenario default)",
        "llm_config": cfg.get("llm_config"),
        "external_repo": cfg.get("external_repo"),
        "legs": legs,
        "wall_clock_s": wall,
        "error": error,
        "captured_at_utc": ts,
    }, indent=2, default=str) + "\n", encoding="utf-8")

    print(f"\n{len(legs)} legs, {wall}s, error={error}")
    print(f"meta: {out / 'run_meta.json'}")
    print(f"score with:  python3 measure/fixtures/dataroom/score.py "
          f"--world {args.world}")
    return 1 if error else 0


if __name__ == "__main__":
    raise SystemExit(main())
