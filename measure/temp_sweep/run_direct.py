#!/usr/bin/env python3
"""Run the audit arm of the temperature sweep: single calls to one server.

    python3 measure/temp_sweep/run_direct.py \
        --model-yaml measure/models/local_qwen38flashnext.yaml \
        --temperatures 0.2,0.5,0.8,1.0,1.3 --repeats 3

Sends each prompt in prompts.jsonl (see build.py) at each temperature, the
given number of times, one call at a time. The order is shuffled with a fixed
seed so that drift in the server over the run falls on every temperature
rather than one. Rows are appended to results/direct_<started>.jsonl as each
call returns, so a stopped run keeps what it had.

THE TEMPERATURE IS SENT AS A LITERAL, deliberately: it is the variable under
measurement. The call goes through measure/prescreen.py:probe(), whose opt-in
`temperature` exists for this question, not through _ChatBackend, which
resolves temperature from src/chat/model_params.py. Every row records the
temperature it was sent at.
"""
from __future__ import annotations

import argparse
import datetime
import json
import random
import sys
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))
from measure.prescreen import probe                              # noqa: E402

HERE = Path(__file__).resolve().parent


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model-yaml", required=True, type=Path)
    ap.add_argument("--temperatures", required=True,
                    help="comma-separated, e.g. 0.2,0.5,0.8,1.0,1.3")
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--only", default=None, help="comma-separated prompt ids")
    ap.add_argument("--max-tokens", type=int, default=32768)
    ap.add_argument("--seed", type=int, default=20261017)
    args = ap.parse_args()

    cfg = yaml.safe_load(args.model_yaml.read_text())["llm_config"]
    url = cfg["vllm_url"].rstrip("/") + "/v1/chat/completions"
    temps = [float(t) for t in args.temperatures.split(",")]
    prompts = [json.loads(l) for l in (HERE / "prompts.jsonl").read_text().splitlines()]
    if args.only:
        keep = set(args.only.split(","))
        prompts = [p for p in prompts if p["id"] in keep]

    order = [(rep, t, p["id"]) for rep in range(1, args.repeats + 1)
             for t in temps for p in prompts]
    random.Random(args.seed).shuffle(order)
    by_id = {p["id"]: p for p in prompts}

    started = datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H-%M-%SZ")
    out = HERE / "results" / f"direct_{started}.jsonl"
    out.parent.mkdir(exist_ok=True)
    print(f"{len(order)} calls -> {out}", flush=True)
    for n, (rep, t, pid) in enumerate(order, 1):
        p = by_id[pid]
        row = probe(cfg["model"], "local", effort=cfg.get("reasoning_effort"),
                    max_tokens=args.max_tokens, timeout=1800, url=url,
                    key_env=None, temperature=t, system=p["system"],
                    user=p["user"], schema=p["schema"], keep_content=True)
        row.update({"prompt": pid, "repeat": rep, "sent_temperature": t,
                    "model_yaml": str(args.model_yaml), "started": started})
        with out.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(row) + "\n")
        print(f"[{n}/{len(order)}] {pid} t={t} rep={rep} {row.get('status')} "
              f"finish={row.get('finish')} reasoning={row.get('reasoning_tok')} "
              f"completion={row.get('completion_tok')} {row.get('secs')}s", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
