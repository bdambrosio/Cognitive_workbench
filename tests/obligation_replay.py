"""Replay real Jill exchanges through the real reflection call and count
what obligations come out.

The question this answers is precision, not recall: a missed obligation
leaves things exactly as they are today, while a spurious one is a debt
nobody owes that nags until someone notices. So the sample is mostly
ordinary turns, and the number to look at is how many obligations exist
at the end of a run over N real turns.

Fidelity: everything is the live path — the configured backend, the real
system prompt, the real existing-concern and memory-neighbour lookups
against real state. The ONE substitution is `_build_dialog`, which
normally reads the last few turns from the dialog store and here returns
the historical exchange being replayed. That is the variable under test.

Safety: runs against a COPY of the jill_chat world (see
feedback_resource_manager_safe_testing — live state was corrupted this
way once). Reflection writes, so it must not write to the real world.

Usage:
    python3 obligation_replay.py [--n 40] [--keep] [--world jill_chat]
"""

import argparse
import json
import os
import shutil
import sys
import time
import uuid
from pathlib import Path

REPO = Path(__file__).resolve()
# Run from the repo root; src/ must be importable the same way launcher does it.
REPO = Path(os.environ.get("CW_REPO", "/home/bruce/Downloads/Cognitive_workbench"))
sys.path.insert(0, str(REPO / "src"))

import yaml  # noqa: E402


def load_character_config(scenario: Path, world_name: str) -> dict:
    """Jill's character config exactly as launcher.parse_characters builds
    it, with the world redirected at the replay copy."""
    data = yaml.safe_load(scenario.read_text())
    cfg = dict(data["characters"]["Jill"])
    cfg["setting"] = data.get("setting", "")
    cfg["world_config"] = {"world_name": world_name}
    cfg.setdefault("llm_config", data.get("llm_config") or {})
    # Never fire concerns during a replay: we are measuring what reflection
    # writes, not running an agent.
    cfg["autonomy_enabled"] = False
    return cfg


def load_turns(trace: Path, n: int, min_chars: int, contains: str = "",
               must_include=(2887,)):
    """Sample non-autonomous turns, always including the ground-truth
    positives.

    Weighted toward SUBSTANTIVE turns on purpose. turn_seq restarted at 1
    every session before the seeding fix, so the file is dominated by
    short session openers ("yes please", "morning") — 853 of 1247
    eligible rows carry seq<100. Those are trivially negative, and a
    denominator full of them flatters the precision number. `min_chars`
    keeps the sample on turns where a request could plausibly have been
    made, which is the case that matters.
    """
    rows = []
    with trace.open() as f:
        for i, line in enumerate(f):
            line = line.strip()
            if not line:
                continue
            try:
                d = json.loads(line)
            except json.JSONDecodeError:
                continue                      # truncated tail row
            if d.get("autonomous"):
                continue                      # autonomous turns don't reflect
            if not (d.get("user_input") or "").strip():
                continue
            d["_row"] = i                     # turn_seq is not unique; this is
            rows.append(d)
    forced = [r for r in rows if int(r.get("turn_seq") or 0) in must_include]
    forced_rows = {r["_row"] for r in forced}
    pool = [r for r in rows
            if r["_row"] not in forced_rows
            and len(r.get("user_input") or "") >= min_chars]
    if contains:
        # SAMPLE SELECTION ONLY — this pattern never reaches the system
        # under test, which classifies semantically. It is here to find
        # turns where the user stated a timeline, so the deadline path
        # gets exercised at all rather than being reported as untested.
        import re
        pat = re.compile(contains, re.I)
        pool = [r for r in pool if pat.search(r.get("user_input") or "")]
        forced = []
    step = max(1, len(pool) // max(1, n - len(forced)))
    sampled = pool[::step][: max(0, n - len(forced))]
    out = forced + sampled
    out.sort(key=lambda r: r["_row"])
    print(f"pool: {len(rows)} eligible, {len(pool)} substantive "
          f"(>={min_chars} chars), sampling {len(out)}", flush=True)
    return out


def obligations(loop):
    """Every active obligation in the world, as {id: props}."""
    out = {}
    for nid, note, _a in loop._iter_active_agent_concerns():
        props = note.get("properties") or {}
        if props.get("category") == "obligation":
            out[nid] = props
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=40)
    ap.add_argument("--world", default="jill_chat")
    ap.add_argument("--min-chars", type=int, default=120,
                    help="minimum user_input length for a sampled turn")
    ap.add_argument("--contains", default="",
                    help="regex over user_input for SAMPLE SELECTION only")
    ap.add_argument("--keep", action="store_true",
                    help="keep the replay world copy for inspection")
    args = ap.parse_args()

    src_world = REPO / "scenarios" / args.world
    replay_world = f"oblreplay_{uuid.uuid4().hex[:8]}"
    dst_world = REPO / "scenarios" / replay_world
    print(f"copying {src_world} → {dst_world} ...", flush=True)
    shutil.copytree(src_world, dst_world)

    try:
        from chat.chat_loop import ChatLoop

        cfg = load_character_config(REPO / "scenarios" / "jill-chat.yaml",
                                    replay_world)
        print(f"backend: {cfg['llm_config']}", flush=True)
        loop = ChatLoop(character_name="Jill", character_config=cfg)

        turns = load_turns(
            dst_world / "Jill" / "memory" / "reasoning_trace.jsonl",
            args.n, args.min_chars, args.contains)
        print(f"replaying {len(turns)} turns\n", flush=True)

        before = obligations(loop)
        print(f"obligations already present: {len(before)}\n", flush=True)

        minted = []
        t0 = time.time()
        for i, row in enumerate(turns, 1):
            seq = int(row.get("turn_seq") or 0)
            dialog = [
                {"source": row.get("source") or "User",
                 "text": row.get("user_input") or ""},
                {"source": "Jill", "text": row.get("raw_response") or ""},
            ]
            # The one substitution — see module docstring.
            loop._build_dialog = lambda source, limit=4, _d=dialog: _d
            seen = set(obligations(loop))
            try:
                loop._reflect_and_remember("User")
            except Exception as e:
                print(f"  turn {seq}: reflection raised {e}", flush=True)
                continue
            now = obligations(loop)
            fresh = {k: v for k, v in now.items() if k not in seen}
            flag = "  <-- OBLIGATION" if fresh else ""
            print(f"[{i}/{len(turns)}] row {row['_row']} seq {seq} "
                  f"({time.time() - t0:.0f}s){flag}", flush=True)
            for nid, props in fresh.items():
                rec = {
                    "turn_seq": seq,
                    "user_input": (row.get("user_input") or "")[:300],
                    "text": str(props.get("content", ""))[:200],
                    "owed_to": props.get("owed_to"),
                    "report_by": props.get("report_by"),
                    "instruction": (props.get("instruction") or "")[:120],
                }
                minted.append(rec)
                print(json.dumps(rec, indent=2), flush=True)

        print("\n" + "=" * 70)
        print(f"turns replayed:      {len(turns)}")
        print(f"obligations minted:  {len(minted)}")
        print(f"rate:                {len(minted) / max(1, len(turns)):.1%} of turns")
        print(f"turn 2887 minted:    "
              f"{any(m['turn_seq'] == 2887 for m in minted)}")
        with_deadline = sum(1 for m in minted if m["report_by"])
        print(f"with a deadline:     {with_deadline}/{len(minted)}")
        out = Path(__file__).with_name("obligation_replay_results.json")
        out.write_text(json.dumps(
            {"turns": len(turns), "minted": minted}, indent=2))
        print(f"\nwrote {out}")
    finally:
        if args.keep:
            print(f"replay world kept at {dst_world}")
        else:
            shutil.rmtree(dst_world, ignore_errors=True)


if __name__ == "__main__":
    main()
