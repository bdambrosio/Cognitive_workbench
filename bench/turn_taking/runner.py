#!/usr/bin/env python3
"""Probe 5 — turn-taking under simultaneous dispatch.

    cd src
    python ../bench/turn_taking/runner.py --backend gemma

Two agents, one message, delivered to BOTH AT ONCE — which is what the CLI's
`→ sent to Jill, Jack` actually does. The message names an order. The question
is whether the second agent waits, or answers immediately and invents what the
first was going to say.

Observed 2026-08-15: Jill answered 2s after Jack and wrote Jack's opening line
herself, puppeting her scene partner. "Jack, you go first" is unenforced by the
loop; it is a request the prose makes and nothing checks.

Both characters run as threads in ONE process, which is exactly what
launcher.py does (`threading.Thread(target=run_agent, ...)`), so this reuses
the real dispatch path rather than simulating it. Messages go in over Zenoh the
same way the CLI sends them.
"""

from __future__ import annotations

import argparse
import datetime
import json
import logging
import sys
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
sys.path.insert(0, str(REPO / "src"))
sys.path.insert(0, str(REPO))

import yaml  # noqa: E402

from bench.common import load_arm, verify_served_model  # noqa: E402

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s %(levelname)s %(name)s: %(message)s")
logger = logging.getLogger("bench.turn_taking.runner")

SCENARIO = REPO / "scenarios" / "coord_search.yaml"
FIRST, SECOND = "Jack", "Jill"

# Named order, one addressee explicitly told to wait. Deliberately a task where
# the second agent CANNOT correctly answer without the first — so answering
# early necessarily means inventing the other's half.
SEED = (
    f"Both of you please read this. {FIRST}: you go FIRST — pick a single "
    f"integer between 1 and 100 and say it, nothing else. "
    f"{SECOND}: WAIT until you have actually seen {FIRST}'s number, then reply "
    f"with that number doubled. Do not guess or assume what {FIRST} will "
    f"choose. If you have not seen a number from {FIRST} yet, say exactly "
    f"'waiting for {FIRST}' and nothing else."
)


def _envelope(text: str, source: str = "User") -> str:
    return json.dumps({
        "timestamp": datetime.datetime.now().isoformat(),
        "sequence_id": 0,
        "mode": "text",
        "content": json.dumps({"source": source, "text": text, "close": False}),
    })


def _build_configs(arm: Dict[str, Any], world: str):
    from launcher import parse_characters  # noqa: E402
    scen = yaml.safe_load(SCENARIO.read_text(encoding="utf-8")) or {}
    arm_llm = dict(arm.get("llm_config") or {})
    scen_llm = dict(scen.get("llm_config") or {})
    scen_llm.update(arm_llm)
    for c in (scen.get("characters") or {}).values():
        if isinstance(c, dict) and c.get("mode") == "chat":
            c["llm_config"] = dict(arm_llm)
    world_cfg = dict(scen.get("world_config") or {})
    world_cfg["world_name"] = world
    chars = parse_characters(scen, scen_llm, world_cfg,
                             scen.get("setting", ""),
                             scen.get("alt_llm_config") or {})
    out = {n: c for n, c in chars if c.get("mode") == "chat"}
    for c in out.values():
        c["autonomy_enabled"] = False
    missing = {FIRST, SECOND} - set(out)
    if missing:
        raise RuntimeError(f"{SCENARIO} lacks chat characters {missing}")
    return out


def _replies(world: str, who: str) -> List[Dict[str, Any]]:
    """Outbound turns this agent has produced, with timestamps."""
    p = (REPO / "scenarios" / world / who / "memory" / "reasoning_trace.jsonl")
    if not p.is_file():
        return []
    out = []
    for line in p.read_text(errors="replace").splitlines():
        if not line.strip():
            continue
        try:
            out.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--backend", required=True)
    ap.add_argument("--deadline", type=float, default=420.0)
    ap.add_argument("--output-dir", type=Path, default=None)
    args = ap.parse_args()

    arm = load_arm(args.backend)
    served = verify_served_model(arm)
    logger.info("arm=%s served=%s", arm["name"], served.get("served"))

    ts = datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H-%M-%SZ")
    world = f"bench-turns-{ts}-{args.backend}"
    out = args.output_dir or (HERE / "results" / f"{ts}_{args.backend}")
    out.mkdir(parents=True, exist_ok=True)

    from launcher import run_agent  # noqa: E402
    import zenoh  # noqa: E402
    from utils.zenoh_utils import make_localhost_config  # noqa: E402

    configs = _build_configs(arm, world)
    shutdown = threading.Event()
    threads = []
    for name in (FIRST, SECOND):
        t = threading.Thread(target=run_agent,
                             args=(name, configs[name], None, None, shutdown),
                             name=f"agent-{name}", daemon=True)
        t.start()
        threads.append(t)
        logger.info("started %s", name)

    error = None
    t0 = time.time()
    try:
        # Let both open their Zenoh sessions and subscribe before publishing,
        # or the seed lands before anyone is listening.
        time.sleep(20)
        session = zenoh.open(make_localhost_config())
        pubs = {n: session.declare_publisher(f"cognitive/{n}/sense_data")
                for n in (FIRST, SECOND)}
        payload = _envelope(SEED)
        dispatch_at = time.time()
        # SIMULTANEOUS — this is the condition under test, not a shortcut.
        for n in (FIRST, SECOND):
            pubs[n].put(payload)
        logger.info("dispatched to both at once")

        deadline = time.time() + args.deadline
        while time.time() < deadline:
            if all(_replies(world, n) for n in (FIRST, SECOND)):
                logger.info("both agents produced a turn")
                break
            time.sleep(3)
        else:
            logger.warning("deadline reached before both replied")
        # Give the later agent room to finish its post-turn work.
        time.sleep(20)
    except Exception as e:
        error = f"{type(e).__name__}: {e}"
        logger.exception("run failed: %s", e)
        dispatch_at = t0
    finally:
        shutdown.set()
        for t in threads:
            t.join(timeout=45)

    wall = round(time.time() - t0, 1)
    traces = {n: _replies(world, n) for n in (FIRST, SECOND)}

    (out / "raw.json").write_text(json.dumps({
        "seed": SEED,
        "first": FIRST, "second": SECOND,
        "dispatch_at": dispatch_at,
        "traces": traces,
        "error": error,
        "wall_clock_s": wall,
    }, indent=2, default=str) + "\n", encoding="utf-8")

    (out / "run_meta.json").write_text(json.dumps({
        "probe": "turn_taking (probe 5)",
        "backend_arm": arm["name"],
        "backend_label": arm.get("label"),
        "served_model_check": served,
        "llm_config": arm.get("llm_config"),
        "scenario": str(SCENARIO.relative_to(REPO)),
        "world_name": world,
        "characters": [FIRST, SECOND],
        "wall_clock_s": wall,
        "captured_at": ts,
        "error": error,
    }, indent=2, default=str) + "\n", encoding="utf-8")

    print(f"\nwrote {out}  wall={wall}s "
          f"turns={ {n: len(v) for n, v in traces.items()} }")
    return 1 if error else 0


if __name__ == "__main__":
    raise SystemExit(main())
