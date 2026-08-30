#!/usr/bin/env python3
"""Does the instruction stack change whether a turn yields or responds?

    python3 measure/yield_probe.py --cell bare --n 20
    python3 measure/yield_probe.py --cell full --n 20 --model measure/models/local_qwen38flashnext.yaml

WHY. On five ChatterMate runs, coverage was close to a step function of one
decision in the first turn: the four that yielded resolved 83-96% of their claim
surface, the one that responded resolved 41%. The runner asks for the REPORT as
soon as a turn responds with blocks outstanding, so an early `respond` turns one
leg of evidence into the whole report.

**It is not budget pressure.** cm_glm_2 yielded at 6 iterations and cm_glm_3
responded at 5, against a cap of 16, and react.py's nudge — which fires at 14 —
appears in no trace. The model chooses with three-quarters of its budget unused.

WHAT THIS ISOLATES. One turn, no runner, no delivery blocks, no review, no
scoring. The only thing that varies between cells is instruction text:

  bare  the core only — tools.py's yield entry and react.py's nudge
  full  plus the claims-audit stack: the persona's "never end with a
        stated-but-unexecuted plan", the scenario's "works it in legs", and
        METHOD §12's "work the priority order straight through"

A difference implicates the workflow text. NO difference implicates the core,
where tools.py's yield entry carries two triggers that disagree when the budget
is ample and work remains — "when the task genuinely needs more than the
remaining budget" and, four sentences later, "`yield` only when real work
remains". That is the case under test.

THE TASK NAMES NO ENDING. It says what to produce and never says when to stop,
report, yield, or work in legs. Any such word here would be a sixth instruction
channel competing with the five under study.
"""
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "src"))

import yaml                                                    # noqa: E402

CELLS = REPO / "measure" / "fixtures" / "yield_probe"
SOURCE = "User"

# Deliberately open-ended and deliberately silent about how to finish. Nine
# documents is more than one turn's worth of reading at a comfortable pace and
# fewer than the 16-action cap can exhaust, which is the region the observed
# runs sat in (5-6 iterations).
TASK = (
    "The document set bound to `inspect_external` holds nine documents. "
    "Catalogue the quantitative claims they make about the business: for each, "
    "the number, what it measures, and the document and line it appears on."
)


def build_config(cell: str, world: str,
                 model_path: Optional[Path]) -> Tuple[str, Dict[str, Any]]:
    """Mirror of workflows/claims_audit/runner.py's build_config.

    Same shape on purpose — a model block REPLACES llm_config rather than
    merging into it, so a stale field from the scenario cannot survive into a
    model that never declared one.
    """
    from launcher import parse_characters                      # noqa: E402

    path = CELLS / f"{cell}.yaml"
    if not path.is_file():
        raise SystemExit(f"no such cell: {path}")
    scenario = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    scen_llm = dict(scenario.get("llm_config") or {})

    if model_path:
        model = yaml.safe_load(Path(model_path).read_text(encoding="utf-8")) or {}
        model_llm = dict(model.get("llm_config") or {})
        if not model_llm:
            raise SystemExit(f"{model_path}: no llm_config block")
        for char in (scenario.get("characters") or {}).values():
            if isinstance(char, dict) and char.get("mode") == "chat":
                char["llm_config"] = dict(model_llm)
        scen_llm.update(model_llm)

    world_cfg = dict(scenario.get("world_config") or {})
    world_cfg["world_name"] = world
    chars = parse_characters(scenario, scen_llm, world_cfg,
                             scenario.get("setting", ""),
                             scenario.get("alt_llm_config") or {})
    chat = [(n, c) for n, c in chars if c.get("mode") == "chat"]
    if len(chat) != 1:
        raise SystemExit(f"expected 1 chat character, found {len(chat)}")
    name, cfg = chat[0]
    # AUTONOMY OFF. What is under test is the DECISION to yield, not the
    # continuation it spawns: with autonomy off the concern a yield creates
    # never fires, which is the cleaner question and matches how a workflow
    # run is driven.
    cfg["autonomy_enabled"] = False
    ext = cfg.get("external_repo")
    if ext and not Path(ext).is_absolute():
        cfg["external_repo"] = str(REPO / ext)
    return name, cfg


def one_sample(cell: str, world: str, model_path: Optional[Path]
               ) -> Dict[str, Any]:
    """One turn in a fresh world. Returns the row, never raises."""
    from chat.chat_loop import ChatLoop                        # noqa: E402
    from measure.trace import load_turns                       # noqa: E402

    row: Dict[str, Any] = {"cell": cell, "world": world}
    t0 = time.time()
    loop = None
    try:
        name, cfg = build_config(cell, world, model_path)
        loop = ChatLoop(character_name=name, character_config=cfg)
        row["model"] = loop.backend.resolved_model()
        loop._process_user_turn(source=SOURCE, text=TASK, close=False)
    except Exception as e:                                     # noqa: BLE001
        row["error"] = f"{type(e).__name__}: {e}"
    finally:
        if loop is not None:
            try:
                loop._post_turn_executor.shutdown(wait=True)
            except Exception:                                  # noqa: BLE001
                pass
            try:
                loop._persist_to_disk()
            except Exception:                                  # noqa: BLE001
                pass
    row["wall_s"] = round(time.time() - t0, 1)
    # THE TRACE IS THE RECORD, not the loop object. measure/trace.py already
    # parses exit_reason and iters off the same rows every other metric reads;
    # re-deriving them here would be a second parser to keep in step.
    try:
        turns = load_turns(world, "Jill")
        if turns:
            row["exit_reason"] = turns[-1].exit_reason
            # `iterations`, not `iters`. The trace ROW carries `iters` and the
            # Turn dataclass renames it — reading the row's spelling off a grep
            # and assuming the object matched cost the first sanity run.
            row["iters"] = turns[-1].iterations
    except Exception as e:                                     # noqa: BLE001
        row.setdefault("error", f"trace unreadable: {e}")
    return row


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cell", required=True, choices=("bare", "full", "both"),
                    help="`both` INTERLEAVES the cells, one sample each per "
                         "round. Anything that drifts over an hour — server "
                         "load, thermal, cache state — otherwise lands in "
                         "whichever cell was collected first. Same reason "
                         "model-qualification.md interleaves models by round.")
    ap.add_argument("--n", type=int, default=20)
    ap.add_argument("--model", type=Path,
                    default=REPO / "measure/models/local_qwen38flashnext.yaml")
    ap.add_argument("--tag", default="",
                    help="suffix for the world names and the output file")
    ap.add_argument("--keep-worlds", action="store_true",
                    help="do not delete each sample's world after reading it")
    args = ap.parse_args()

    rev = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=REPO,
                         capture_output=True, text=True).stdout.strip()
    dirty = bool(subprocess.run(["git", "status", "--porcelain"], cwd=REPO,
                                capture_output=True, text=True).stdout.strip())
    if dirty:
        # Not fatal — but a set collected across an edit is not one set, which
        # is the failure that emptied the qualification board on 2026-08-27.
        print("WARNING: working tree is dirty; samples may not share an "
              "instrument.", file=sys.stderr)

    cells = ("bare", "full") if args.cell == "both" else (args.cell,)
    out = REPO / "measure" / f"yield_probe_{args.cell}{args.tag}.jsonl"
    rows = []
    plan = [(i, c) for i in range(1, args.n + 1) for c in cells]
    for i, cell in plan:
        world = f"yp_{cell}{args.tag}_{i}"
        shutil.rmtree(REPO / "scenarios" / world, ignore_errors=True)
        row = one_sample(cell, world, args.model)
        row.update(sample=i, harness_rev=rev, tree_dirty=dirty)
        rows.append(row)
        with out.open("a", encoding="utf-8") as f:
            f.write(json.dumps(row, default=str) + "\n")
        print(f"  {i:>3}/{args.n} {cell:5} {row.get('exit_reason','?'):9} "
              f"iters={row.get('iters','?'):<3} {row['wall_s']:>6}s"
              f"{'  ' + row['error'][:60] if row.get('error') else ''}")
        if not args.keep_worlds:
            shutil.rmtree(REPO / "scenarios" / world, ignore_errors=True)

    print(f"\n  rev={rev}")
    for cell in cells:
        ok = [r for r in rows if r["cell"] == cell and r.get("exit_reason")]
        y = sum(1 for r in ok if r["exit_reason"] == "yield")
        r_ = sum(1 for r in ok if r["exit_reason"] == "respond")
        its = sorted(r["iters"] for r in ok if isinstance(r.get("iters"), int))
        rate = f"{r_ / len(ok):.0%}" if ok else "-"
        band = (f"iters {its[0]}/{its[len(its)//2]}/{its[-1]}" if its else "")
        print(f"  {cell:5} usable {len(ok):>3}/{args.n}   yield {y:>3}  "
              f"respond {r_:>3}   respond-rate {rate:>4}   {band}")
    print("  (cap 16, nudge at 14)")
    print(f"  rows: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
