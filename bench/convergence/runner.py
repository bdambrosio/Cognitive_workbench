#!/usr/bin/env python3
"""Probe 1 — convergence.

Can the backend produce the sixteen facts at all? One turn, no yielding.

SPLIT FROM PROBE 3 on 2026-08-18 after the merged version scored Gemma 0/16.
It had not failed: it found all four paths and yielded as instructed, leaving
a 243-char handoff where the answer should have been, because the continuation
only fires with autonomy on. Probe 3 REWARDS yielding and probe 1 needs the
completed answer — one turn cannot satisfy both, and the merged run measured
the contradiction rather than the backend. See bench/yield_probe/ for probe 3.

    cd src
    python ../bench/convergence/runner.py --backend gemma

Writes bench/convergence/results/<ts>_<backend>/ with raw.json + run_meta.json,
then score with:

    python bench/convergence/score.py --run-dir <that dir>
"""

from __future__ import annotations

import argparse
import datetime
import json
import logging
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
sys.path.insert(0, str(REPO / "src"))
sys.path.insert(0, str(REPO))

from bench.common import (build_loop_config, load_arm,  # noqa: E402
                          read_agent_concerns, read_reasoning_trace,
                          turn_costs, verify_served_model)

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s %(levelname)s %(name)s: %(message)s")
logger = logging.getLogger("bench.convergence.runner")

SOURCE = "User"
SCENARIO = REPO / "scenarios" / "yield_test.yaml"

# The 2026-08-16 task with TWO deliberate departures, so this is not
# byte-identical to the archived yield_test.* runs and they are not a baseline
# for probe 1 (they were launcher-driven with autonomy on, so their
# continuations fired and finished work a single turn cannot):
#
#   1. The yield instruction is replaced by a completion instruction — probe 3
#      owns yielding, and one turn cannot both yield and finish.
#   2. rhythm_source is asked for BY NAME. The rubric has always scored it, but
#      the old prompt asked only for "activation and rhythm", so every backend
#      lost a cell per path for not volunteering a field nobody requested.
#      Gemma scored 12/16 on 2026-08-18 with zero wrong and all four misses in
#      that one column. The ask now matches what is scored.
PROMPT = (
    "Using inspect, map every code path in src/chat/concerns.py that creates "
    "an agent_concern. For each path report: what triggers it, its starting "
    "activation, its rhythm — BOTH the rhythm_hours value and the "
    "rhythm_source value — and whether it can fire without --autonomy. Work "
    "through all of them and give your complete findings in this turn, with "
    "the specific values you found. Do not yield; if you run short of room, "
    "report the concrete values you have rather than promising them."
)


def _latest_reply(loop, source: str) -> str:
    turns = loop.store.get_recent_turns(source, limit=4, scope="all")
    for t in reversed(turns):
        if t.get("direction") == "out":
            return str(t.get("text", ""))
    return ""


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--backend", required=True,
                    help="arm name from bench/backends.yaml")
    ap.add_argument("--output-dir", type=Path, default=None)
    ap.add_argument("--world-name", default=None,
                    help="override; defaults to a unique per-run world")
    args = ap.parse_args()

    arm = load_arm(args.backend)
    # Fail here rather than mid-run. A connection-refused on iteration 1 reads
    # as a yield-mechanism failure in the trace and is not one.
    served = verify_served_model(arm)
    logger.info("arm=%s served=%s", arm["name"], served.get("served"))

    ts = datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H-%M-%SZ")
    world = args.world_name or f"bench-conv-{ts}-{args.backend}"
    out = args.output_dir or (HERE / "results" / f"{ts}_{args.backend}")
    out.mkdir(parents=True, exist_ok=True)

    from chat.chat_loop import ChatLoop  # noqa: E402

    char_name, char_cfg = build_loop_config(SCENARIO, arm, world, autonomy=False)
    loop = ChatLoop(character_name=char_name, character_config=char_cfg)

    t0 = time.time()
    reply, error = "", None
    try:
        loop._process_user_turn(source=SOURCE, text=PROMPT, close=False)
        reply = _latest_reply(loop, SOURCE)
    except Exception as e:
        error = f"{type(e).__name__}: {e}"
        logger.exception("turn failed: %s", e)
    finally:
        try:
            loop._post_turn_executor.shutdown(wait=True)
        except Exception as e:
            logger.warning("executor shutdown failed: %s", e)
        try:
            loop._persist_to_disk()
        except Exception as e:
            logger.warning("final persist failed: %s", e)
    wall = round(time.time() - t0, 1)

    trace = read_reasoning_trace(world, char_name)
    concerns = read_agent_concerns(world, char_name)

    (out / "raw.json").write_text(json.dumps({
        "prompt": PROMPT,
        "reply": reply,
        "error": error,
        "wall_clock_s": wall,
        "trace": trace,
        "agent_concerns": concerns,
    }, indent=2, default=str) + "\n", encoding="utf-8")

    (out / "run_meta.json").write_text(json.dumps({
        "probe": "convergence (probe 1)",
        "backend_arm": arm["name"],
        "backend_label": arm.get("label"),
        "served_model_check": served,
        "llm_config": arm.get("llm_config"),
        "scenario": str(SCENARIO.relative_to(REPO)),
        "world_name": world,
        "character": char_name,
        "autonomy_enabled": False,
        "wall_clock_s": wall,
        "costs": turn_costs(trace),
        "captured_at": ts,
        "error": error,
    }, indent=2, default=str) + "\n", encoding="utf-8")

    print(f"\nwrote {out}")
    print(f"  wall={wall}s turns={len(trace)} reply={len(reply)} chars")
    for t in trace:
        print(f"  turn exit={t.get('exit_reason')} iters={t.get('iters')}")
    return 1 if error else 0


if __name__ == "__main__":
    raise SystemExit(main())
