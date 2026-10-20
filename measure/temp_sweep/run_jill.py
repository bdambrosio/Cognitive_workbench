#!/usr/bin/env python3
"""Run the Jill arm of the temperature sweep: the real chat agent, one turn
per run, with no memory carried from one run to the next.

    python3 measure/temp_sweep/run_jill.py --temperatures 0.2,0.5,0.8,1.0,1.3 \
        --repeats 3 [--only j3_compound_interest]

Each run (one prompt, one temperature, one repeat):
  1. deletes scenarios/jill_tempsweep/, the world this arm uses, so Jill
     starts with no memory, notes, concerns state or traces;
  2. writes scenarios/jill-tempsweep.yaml from scenarios/jill-chat.yaml's
     Jill, changed only as WORLD and _sweep_character() say, with
     chat.react_temperature set to the run's temperature;
  3. launches `launcher.py jill-tempsweep.yaml --reasoning` with its console
     log captured, waits for her zenoh subscriber, and sends the prompt on
     cognitive/Jill/sense_data in the envelope cli.py sends;
  4. waits for her reply in memory/conversation.txt, then for the post-turn
     work to go quiet (no new <llm-usage> line for --quiet seconds);
  5. stops the launcher and copies memory/, subagent_traces/,
     inspect_traces/ and the log into results/jill_<started>/<run>/;
  6. deletes the world again.

react_temperature reaches the ReAct action call only (src/chat/react.py:593);
her subagents, reflection, memories and concerns keep the model's configured
temperature. That is the knob Jill has, and it is what this arm varies.

The live world `jill_chat` is never read or written. The character is still
named Jill, so her zenoh topics are the real ones: the runner refuses to start
while a launcher for jill-chat.yaml is running.
"""
from __future__ import annotations

import argparse
import datetime
import json
import os
import random
import re
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
SRC = REPO / "src"
sys.path.insert(0, str(SRC))

WORLD = "jill_tempsweep"
SCENARIO = REPO / "scenarios" / "jill-tempsweep.yaml"
WORLD_DIR = REPO / "scenarios" / WORLD
LIVE = REPO / "scenarios" / "jill-chat.yaml"
PYTHON = REPO / "zenoh_venv" / "bin" / "python3"
# The launcher's console shows WARNING and above; its INFO lines, the
# "agent threads started" line and <llm-usage> among them, go only to this
# file. A run reads the part written since it started.
LAUNCHER_LOG = REPO / "logs" / "character_launcher.log"

# Tools with effects outside this machine's scratch state, or that need
# hardware or a running world: mail, the vault, scripts, images, the head,
# X, and the shared world.
OMIT = ["check-email", "obsidian", "exec-script", "generate-image",
        "camera-capture", "head-move", "look-at-target", "check-x-feed",
        "world-look", "world-mark", "world-move"]

USAGE = re.compile(r"^(\S+ \S+) .*<llm-usage> elapsed=(\S+)s prompt=(\S+) cached=(\S+) "
                   r"completion=(\S+) reasoning=(\S+) finish=(\S+) model=(\S*) temp=(\S+)")


def _launchers() -> list:
    """Pids of running launcher.py processes and the scenario each was given.
    Reads /proc argv as separate arguments, so this script's own command
    line, which names the scenario too, is never mistaken for one."""
    found = []
    for d in Path("/proc").iterdir():
        if not d.name.isdigit() or int(d.name) == os.getpid():
            continue
        try:
            argv = (d / "cmdline").read_bytes().split(b"\0")
        except OSError:
            continue
        argv = [a.decode(errors="replace") for a in argv if a]
        for i, a in enumerate(argv):
            if a.endswith("launcher.py") and i + 1 < len(argv):
                found.append((int(d.name), argv[i + 1]))
    return found


def _sweep_character(temperature: float) -> dict:
    live = yaml.safe_load(LIVE.read_text())
    jill = live["characters"]["Jill"]
    jill["sensors"] = []
    chat = jill.setdefault("chat", {})
    chat["react_temperature"] = temperature
    chat["omitted_tools"] = sorted(set(chat.get("omitted_tools") or []) | set(OMIT))
    chat["head"] = False
    return {"world_config": {"world_name": WORLD},
            "llm_config": live.get("llm_config"),
            "setting": live.get("setting"),
            "characters": {"Jill": jill}}


def _wipe() -> None:
    if WORLD_DIR.exists():
        shutil.rmtree(WORLD_DIR)


def _send(text: str) -> None:
    import zenoh
    from utils.zenoh_utils import make_localhost_config
    s = zenoh.open(make_localhost_config())
    try:
        pub = s.declare_publisher("cognitive/Jill/sense_data")
        time.sleep(1.0)          # let the publisher match the subscriber
        pub.put(json.dumps({"timestamp": datetime.datetime.now().isoformat(),
                            "sequence_id": 0, "mode": "text",
                            "content": json.dumps({"source": "User", "text": text})}))
        time.sleep(1.0)
    finally:
        s.close()


def _reply(conv: Path) -> str | None:
    """Her first reply in conversation.txt, or None while there is none."""
    if not conv.exists():
        return None
    blocks = conv.read_text(encoding="utf-8").split("=" * 80)
    for i, b in enumerate(blocks):
        if re.search(r"\] \S+ <- Jill", b) and i + 1 < len(blocks):
            return blocks[i + 1].strip()
    return None


def _stop(proc: subprocess.Popen) -> None:
    for sig, wait in ((signal.SIGINT, 45), (signal.SIGTERM, 20), (signal.SIGKILL, 10)):
        if proc.poll() is not None:
            return
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            return
        try:
            proc.wait(timeout=wait)
        except subprocess.TimeoutExpired:
            continue


def _since(offset: int) -> str:
    """The launcher log written after `offset`; all of it if it rotated."""
    with LAUNCHER_LOG.open("rb") as fh:
        if LAUNCHER_LOG.stat().st_size >= offset:
            fh.seek(offset)
        return fh.read().decode("utf-8", errors="replace")


def usage_lines(text: str) -> list:
    rows = []
    for line in text.splitlines():
        m = USAGE.match(line)
        if m:
            def num(x):
                return None if x == "None" else float(x)
            rows.append({"at": m.group(1), "secs": num(m.group(2)),
                         "prompt": num(m.group(3)), "completion": num(m.group(5)),
                         "reasoning": num(m.group(6)), "finish": m.group(7),
                         "temp": num(m.group(9))})
    return rows


def one_run(p: dict, t: float, rep: int, out_root: Path, args) -> dict:
    run = out_root / f"{p['id']}_t{t}_r{rep}"
    run.mkdir(parents=True)
    log = run / "launcher.log"
    _wipe()
    SCENARIO.write_text(yaml.safe_dump(_sweep_character(t), sort_keys=False,
                                       allow_unicode=True), encoding="utf-8")
    meta = {"prompt": p["id"], "text": p["text"], "temperature": t, "repeat": rep}
    t0 = time.time()
    offset = LAUNCHER_LOG.stat().st_size if LAUNCHER_LOG.exists() else 0
    with (run / "console.log").open("w") as fh:
        proc = subprocess.Popen([str(PYTHON), "launcher.py", SCENARIO.name, "--reasoning"],
                                cwd=SRC, stdout=fh, stderr=subprocess.STDOUT,
                                stdin=subprocess.DEVNULL, start_new_session=True)
    try:
        # Her subscriber is up only at "chat zenoh ready", which can come
        # well after the thread starts (53 s on 2026-10-20 with the old quant
        # holding 108 GB of host RAM); a message sent before it is lost.
        while "chat zenoh ready" not in _since(offset):
            if proc.poll() is not None or time.time() - t0 > args.startup_timeout:
                meta["error"] = "launcher did not start"
                return meta
            time.sleep(2)
        time.sleep(args.settle)
        meta["startup_secs"] = round(time.time() - t0, 1)
        sent = time.time()
        _send(p["text"])
        conv = WORLD_DIR / "Jill" / "memory" / "conversation.txt"
        while (reply := _reply(conv)) is None:
            if proc.poll() is not None or time.time() - sent > args.turn_timeout:
                meta["error"] = "no reply"
                break
            time.sleep(3)
        meta["reply"] = reply
        meta["reply_secs"] = round(time.time() - sent, 1)
        n, quiet_since = len(usage_lines(_since(offset))), time.time()
        while time.time() - quiet_since < args.quiet and time.time() - sent < args.turn_timeout:
            time.sleep(5)
            m = len(usage_lines(_since(offset)))
            if m != n:
                n, quiet_since = m, time.time()
        meta["done_secs"] = round(time.time() - sent, 1)
    finally:
        _stop(proc)
        log.write_text(_since(offset), encoding="utf-8")
        for sub in ("memory", "subagent_traces", "inspect_traces"):
            src = WORLD_DIR / "Jill" / sub
            if src.exists():
                shutil.copytree(src, run / sub)
        _wipe()
        (run / "meta.json").write_text(json.dumps(meta, indent=1), encoding="utf-8")
    return meta


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--temperatures", required=True)
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--only", default=None)
    ap.add_argument("--prompts", default="jill_prompts.yaml",
                    help="comma-separated prompt files in this directory")
    ap.add_argument("--label", default="",
                    help="appended to the results directory name, e.g. the model")
    ap.add_argument("--seed", type=int, default=20261017)
    ap.add_argument("--startup-timeout", type=float, default=300)
    ap.add_argument("--settle", type=float, default=15,
                    help="seconds between her subscriber being ready and the send")
    ap.add_argument("--turn-timeout", type=float, default=1800)
    ap.add_argument("--quiet", type=float, default=90,
                    help="post-turn work is done after this long with no LLM call")
    args = ap.parse_args()

    running = _launchers()
    if running:
        raise SystemExit(f"a launcher is running ({running}); stop it first — "
                         f"this arm uses Jill's topics")
    prompts = [p for name in args.prompts.split(",")
               for p in yaml.safe_load((HERE / name).read_text())["prompts"]]
    if args.only:
        keep = set(args.only.split(","))
        prompts = [p for p in prompts if p["id"] in keep]
    temps = [float(x) for x in args.temperatures.split(",")]
    order = [(rep, t, p["id"]) for rep in range(1, args.repeats + 1)
             for t in temps for p in prompts]
    random.Random(args.seed).shuffle(order)
    by_id = {p["id"]: p for p in prompts}
    started = datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H-%M-%SZ")
    out_root = HERE / "results" / f"jill_{started}{'_' + args.label if args.label else ''}"
    print(f"{len(order)} runs -> {out_root}", flush=True)
    for n, (rep, t, pid) in enumerate(order, 1):
        meta = one_run(by_id[pid], t, rep, out_root, args)
        print(f"[{n}/{len(order)}] {pid} t={t} rep={rep} startup={meta.get('startup_secs')}s "
              f"reply={meta.get('reply_secs')}s done={meta.get('done_secs')}s "
              f"{meta.get('error') or ''}", flush=True)
    SCENARIO.unlink(missing_ok=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
