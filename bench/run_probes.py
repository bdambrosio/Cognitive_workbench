#!/usr/bin/env python3
"""Drive probes 1-3 across backend arms, resumably.

    python3 bench/run_probes.py --arms gemma,qwen,luna

Runs everything it can without help. The one thing it cannot do is restart the
vLLM server, so when it reaches a local arm whose model is not the one being
served it stops, prints the exact command, and exits 2. Re-run the SAME command
afterwards — completed (arm, probe) pairs are recorded in campaign.json and
skipped, so it picks up where it left off.

Arm order is computed, not given: whichever local model is already loaded runs
first, cloud arms run whenever, and the arm needing a restart is left for last.
That way one restart covers the whole campaign instead of two.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "src"))

from bench.common import load_arm  # noqa: E402

STATE = HERE / "campaign.json"

PROBES = {
    "convergence": (HERE / "convergence" / "runner.py",
                    HERE / "convergence" / "score.py"),
    "yield_probe": (HERE / "yield_probe" / "runner.py",
                    HERE / "yield_probe" / "score.py"),
    "tictactoe": (HERE / "tictactoe" / "runner.py",
                  HERE / "tictactoe" / "score.py"),
}

RESTART_HINT = {
    "gemma": (
        "vllm serve google/gemma-4-31B-it --max-model-len 65336 --port 5000 \\\n"
        "  --host 127.0.0.1 --max-num-seqs 4 --max-num-batched-tokens 8192 \\\n"
        "  --limit-mm-per-prompt '{\"image\": 1, \"audio\": 0}' \\\n"
        "  --speculative-config '{\"method\": \"mtp\", \"model\": "
        "\"google/gemma-4-31B-it-assistant\", \"num_speculative_tokens\": 4}' \\\n"
        "  --async-scheduling --gpu-memory-utilization 0.96 \\\n"
        "  --reasoning-parser gemma4"),
    "qwen": (
        "vllm serve Qwen/Qwen3.8-27B --port 5000 --host 127.0.0.1 \\\n"
        "  --reasoning-parser qwen3\n"
        "  # plus whatever FP8 / MTP flags the 2026-08-14 round-2 config used"),
}


def _load_state() -> Dict[str, Any]:
    if STATE.is_file():
        try:
            return json.loads(STATE.read_text(encoding="utf-8"))
        except json.JSONDecodeError as e:
            print(f"  campaign.json unreadable ({e}); starting fresh")
    return {"done": [], "runs": {}}


def _save_state(state: Dict[str, Any]) -> None:
    STATE.write_text(json.dumps(state, indent=2) + "\n", encoding="utf-8")


def _served_model(url: str = "http://127.0.0.1:5000") -> Optional[List[str]]:
    """What the local server is serving right now, or None if it is down."""
    import urllib.error
    import urllib.request
    try:
        with urllib.request.urlopen(f"{url}/v1/models", timeout=5) as fh:
            payload = json.loads(fh.read().decode("utf-8"))
        return [m.get("id", "") for m in (payload.get("data") or [])]
    except (urllib.error.URLError, OSError, ValueError):
        return None


def _arm_ready(arm: Dict[str, Any], served: Optional[List[str]]) -> bool:
    expect = arm.get("expects_served_model")
    if not expect:
        return True                      # cloud
    if served is None:
        return False
    return any(expect.lower() in s.lower() for s in served)


def _order_arms(names: List[str], served: Optional[List[str]]) -> List[str]:
    """Loaded-local first, then cloud, then the arm needing a restart."""
    arms = {n: load_arm(n) for n in names}
    ready_local = [n for n in names
                   if arms[n].get("expects_served_model") and _arm_ready(arms[n], served)]
    cloud = [n for n in names if not arms[n].get("expects_served_model")]
    rest = [n for n in names if n not in ready_local and n not in cloud]
    return ready_local + cloud + rest


def _run(cmd: List[str], cwd: Path) -> int:
    print(f"    $ {' '.join(str(c) for c in cmd)}")
    return subprocess.run([str(c) for c in cmd], cwd=str(cwd)).returncode


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--arms", default="gemma,qwen,luna")
    ap.add_argument("--probes", default="convergence,yield_probe,tictactoe")
    ap.add_argument("--reset", action="store_true",
                    help="forget prior progress and run everything again")
    args = ap.parse_args()

    names = [a.strip() for a in args.arms.split(",") if a.strip()]
    probes = [p.strip() for p in args.probes.split(",") if p.strip()]
    for p in probes:
        if p not in PROBES:
            print(f"unknown probe {p!r}; have {sorted(PROBES)}")
            return 1

    if args.reset and STATE.is_file():
        STATE.unlink()
    state = _load_state()
    done = set(tuple(d) for d in state["done"])

    served = _served_model()
    print(f"local server serving: {served if served else '<down>'}")
    ordered = _order_arms(names, served)
    print(f"arm order: {' -> '.join(ordered)}\n")

    for arm_name in ordered:
        arm = load_arm(arm_name)
        pending = [p for p in probes if (arm_name, p) not in done]
        if not pending:
            print(f"[{arm_name}] all probes already done — skipping")
            continue

        if not _arm_ready(arm, _served_model()):
            print(f"\n{'=' * 68}")
            print(f"PAUSED — arm {arm_name!r} needs a different model served.")
            print(f"Expected a model matching {arm['expects_served_model']!r}; "
                  f"server has {_served_model() or '<down>'}.")
            print(f"\nPlease restart the vLLM server:\n")
            # A *_reasoning arm serves the SAME model as its base arm — the
            # dial is a request field, not a deployment. Without this strip,
            # gemma_reasoning printed "(see backends.yaml notes)" instead of a
            # command, which is a worse thing to hand someone mid-campaign.
            hint_key = arm_name.replace("_reasoning", "")
            print(RESTART_HINT.get(hint_key, "  (see backends.yaml notes)"))
            print(f"\nThen re-run this exact command; finished work is skipped:")
            print(f"  python3 bench/run_probes.py --arms {args.arms} "
                  f"--probes {args.probes}")
            print(f"{'=' * 68}")
            _save_state(state)
            return 2

        for probe in pending:
            runner, scorer = PROBES[probe]
            print(f"\n[{arm_name}] {probe} — running")
            t0 = time.time()
            # Runners import `launcher`, so they must start from src/.
            rc = _run([sys.executable, runner, "--backend", arm_name],
                      cwd=REPO / "src")
            if rc != 0:
                print(f"[{arm_name}] {probe} runner exited {rc} — NOT scoring, "
                      f"NOT marking done. Fix and re-run.")
                _save_state(state)
                return 1
            results = sorted((HERE / probe / "results").glob(f"*_{arm_name}"))
            if not results:
                print(f"[{arm_name}] {probe}: runner left no results dir")
                _save_state(state)
                return 1
            run_dir = results[-1]
            print(f"[{arm_name}] {probe} — scoring {run_dir.name}")
            rc = _run([sys.executable, scorer, "--run-dir", run_dir], cwd=REPO)
            if rc != 0:
                print(f"[{arm_name}] {probe} scorer exited {rc}")
                _save_state(state)
                return 1
            done.add((arm_name, probe))
            state["done"] = [list(d) for d in sorted(done)]
            state["runs"][f"{arm_name}/{probe}"] = str(run_dir.relative_to(REPO))
            _save_state(state)
            print(f"[{arm_name}] {probe} done in {round(time.time() - t0, 1)}s")

    print(f"\n{'=' * 68}\nCAMPAIGN COMPLETE")
    for k, v in sorted(state["runs"].items()):
        print(f"  {k:<28} {v}")
    print(f"\nCompare with:  python3 bench/report.py")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
