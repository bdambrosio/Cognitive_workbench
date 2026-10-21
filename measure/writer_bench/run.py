#!/usr/bin/env python3
"""Run the writing-step options on the bench cases.

    python3 measure/writer_bench/run.py <cases.jsonl> <out.jsonl>
        [--model measure/models/<name>.yaml] [--options base,cheap,...]
        [--workers 2] [--only 6495,6511]

Without --model the backend is Jill's own (scenarios/jill-chat.yaml
llm_config: the local server on port 5000). Sampling settings resolve from
src/chat/model_params.py, as on every live call. One line per (case,
option): the reply, the tool a first call chose instead of responding (the
bench cannot run tools), the details a check flagged, and every call with
its wall time. `added_secs` is the option's time beyond the base action
call, which is the cost a live turn would add.

Cases are appended as they finish; a rerun skips (case, option) pairs
already in <out.jsonl>.
"""
from __future__ import annotations

import argparse
import json
import sys
import threading
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(REPO / "src"))
sys.path.insert(0, str(HERE))

import yaml                                                    # noqa: E402
import variants as V                                           # noqa: E402

ALL = ["base", "cheap", "points", "cheap_points", "writer", "precheck", "agree"]
_lock = threading.Lock()


def backend_factory(model_yaml):
    from chat.backend import _ChatBackend
    if model_yaml:
        llm = (yaml.safe_load(Path(model_yaml).read_text()) or {}).get("llm_config") or {}
    else:
        live = yaml.safe_load((REPO / "scenarios/jill-chat.yaml").read_text())
        llm = (live["characters"]["Jill"].get("llm_config") or live.get("llm_config") or {})

    def make():
        return _ChatBackend(
            server=llm.get("server", "local"), model=llm.get("model", ""),
            base_url=llm.get("vllm_url") or llm.get("base_url") or "http://127.0.0.1:5000",
            is_reasoning=llm.get("is_reasoning_model"), api_key=llm.get("api_key"),
            reasoning_effort=llm.get("reasoning_effort"), extra_body=llm.get("extra_body"),
            session_affinity="writer_bench/Jill")
    return make


def run_case(case, options, make, out, done):
    seq = case["turn_seq"]
    # A stored base is reused, so the options built on its draft stay
    # consistent with the base row already written.
    base = done.get((seq, "base"))
    for opt in options:
        if (seq, opt) in done:
            continue
        c = V.Caller(make)
        try:
            if opt == "base":
                r = V.opt_base(c, case)
                base = {**r, "calls": list(c.calls)}
            elif opt in ("cheap", "points", "cheap_points"):
                r = getattr(V, f"opt_{opt}")(c, case)
            else:
                if base is None:
                    raise RuntimeError("base must run first")
                if opt == "agree":
                    r = V.opt_agree(c, case, base, make)
                else:
                    r = getattr(V, f"opt_{opt}")(c, case, base)
        except Exception as e:                                 # noqa: BLE001
            r = {"reply": "", "error": f"{type(e).__name__}: {e}"}
        base_secs = sum(x["secs"] for x in base["calls"]) if base else None
        own = sum(x["secs"] for x in c.calls)
        added = own if opt in ("writer", "precheck", "agree") else (own - base_secs if base_secs is not None and opt != "base" else 0.0)
        row = {"turn_seq": seq, "option": opt, **r, "calls": c.calls,
               "secs": round(own, 2), "base_secs": base_secs, "added_secs": round(added, 2)}
        with _lock, out.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(row) + "\n")
        print(f"{seq} {opt:8s} {own:6.1f}s added {added:6.1f}s chose={r.get('chose_tool')} "
              f"flagged={len(r.get('flagged') or [])} {r.get('error') or ''}", flush=True)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cases")
    ap.add_argument("out")
    ap.add_argument("--model", default=None)
    ap.add_argument("--options", default=",".join(ALL))
    ap.add_argument("--workers", type=int, default=2)
    ap.add_argument("--only", default=None)
    args = ap.parse_args()
    options = [o for o in ALL if o in args.options.split(",")]
    if any(o in ("writer", "precheck", "agree") for o in options) and "base" not in options:
        options.insert(0, "base")
    cases = [json.loads(l) for l in Path(args.cases).open()]
    if args.only:
        keep = {int(x) for x in args.only.split(",")}
        cases = [c for c in cases if c["turn_seq"] in keep]
    out = Path(args.out)
    done = {}
    if out.exists():
        for l in out.open():
            r = json.loads(l)
            if not r.get("error"):
                done[(r["turn_seq"], r["option"])] = r
    make = backend_factory(args.model)
    with ThreadPoolExecutor(args.workers) as ex:
        list(ex.map(lambda c: run_case(c, options, make, out, done), cases))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
