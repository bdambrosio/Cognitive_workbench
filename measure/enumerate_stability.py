#!/usr/bin/env python3
"""Enumerate one claim source several times per model and compare the surfaces.

Runs the real phase one (`claims_audit/runner.py --enumerate-only`), so what
is measured is the runner's own enumeration: the schema-constrained call, then
`assemble_surface`, which folds claims that repeat one quote within a section.
The doc9 fixture froze 14 claims on 2026-09-06 and 7 on 2026-09-12 with the
same model and settings; both emissions had 15 claims. The difference was how
the model quoted: fragments the first time, whole sentences the second, and
whole sentences fold. This harness separates the two: what the model emitted,
and what survived the fold.

  run      python3 measure/enumerate_stability.py run --model measure/models/fw_glm53flash.yaml \
               --n 3 --label sept12 [--engagement dataroom-fixture-doc9]
           Runs sequentially (one run at a time per hosted route) and prints
           the run directories.
  compare  python3 measure/enumerate_stability.py compare <run dir>...
           One row per run, then within-model pairwise overlap.

Overlap is Jaccard on two sets per run: the normalised (quote, lines) pairs the
fold keys on, and the set of source lines any claim quotes. Statements are
printed, not compared: their wording varies with every run and a similarity
score over them would need its own design.
"""
import argparse
import itertools
import json
import re
import subprocess
import sys

import yaml
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RUNNER = ROOT / "workflowsv2" / "claims_audit" / "runner.py"
ENGAGEMENTS = ROOT / "workflowsv2" / "claims_audit" / "engagements"


def _norm(s: str) -> str:
    return re.sub(r"\s+", " ", (s or "").strip().strip('"').strip()).lower()


def run(args) -> int:
    runs_dir = ENGAGEMENTS / args.engagement / "runs"
    stem = Path(args.model).stem
    dirs = []
    for i in range(1, args.n + 1):
        world = f"enum_{args.label}_{stem}_{i}"
        cmd = [sys.executable, str(RUNNER), "--engagement", args.engagement,
               "--world", world, "--model", args.model, "--enumerate-only"]
        print("+", " ".join(cmd), flush=True)
        proc = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
        found = sorted(runs_dir.glob(f"*_{world}"))
        if proc.returncode != 0 or not found:
            print(f"run {i} failed (exit {proc.returncode}):\n{proc.stderr[-2000:]}")
            return 1
        dirs.append(found[-1])
        print(f"  -> {found[-1]}", flush=True)
    print("\n".join(str(d) for d in dirs))
    return 0


def figures(run_dir: Path) -> dict:
    meta = json.loads((run_dir / "run_meta.json").read_text())
    emitted = json.loads((run_dir / "surface_emission.txt").read_text())["claims"]
    frozen = json.loads((run_dir / "claims.json").read_text())["claims"]
    eng = ENGAGEMENTS / run_dir.parent.parent.name / "engagement.yaml"
    src_name = (yaml.safe_load(eng.read_text()).get("claim_sources") or [None])[0]
    src_lines = []
    if src_name:
        p = Path(meta["external_repo"]) / src_name
        if p.is_file():
            src_lines = p.read_text().splitlines()
    whole = 0
    for c in emitted:
        ln = c.get("lines") or [0, 0]
        if ln[0] == ln[1] and 1 <= ln[0] <= len(src_lines):
            line_text = _norm(src_lines[ln[0] - 1].lstrip("*- ").strip())
            if _norm(c.get("quote")) == line_text:
                whole += 1
    keys = {(_norm(c.get("quote")), tuple(c.get("lines") or [])) for c in emitted}
    lines = set()
    for c in emitted:
        ln = c.get("lines") or []
        if len(ln) == 2:
            lines.update(range(ln[0], ln[1] + 1))
    calls = meta["surface"].get("calls") or []
    reasoning = sum(a.get("reasoning_chars", 0) for c in calls for a in c.get("attempts", []))
    return {
        "run": run_dir.name, "model": meta["resolved_model"],
        "temperature": meta["resolved_temperature"],
        "effort": (meta.get("llm_config") or {}).get("reasoning_effort"),
        "emitted": len(emitted), "frozen": len(frozen),
        "folded": len(emitted) - len(frozen),
        "whole_line_quotes": whole, "distinct_quotes": len(keys),
        "lines_quoted": len(lines), "reasoning_chars": reasoning,
        "wall_s": meta.get("wall_clock_s"),
        "_keys": keys, "_lines": lines,
        "_statements": [c.get("statement") for c in frozen],
    }


def jaccard(a: set, b: set) -> float:
    return len(a & b) / len(a | b) if (a | b) else 1.0


def compare(args) -> int:
    rows = [figures(Path(d)) for d in args.runs]
    cols = ["run", "model", "temperature", "effort", "emitted", "frozen", "folded",
            "whole_line_quotes", "distinct_quotes", "lines_quoted",
            "reasoning_chars", "wall_s"]
    print("| " + " | ".join(cols) + " |")
    print("|" + "---|" * len(cols))
    for r in rows:
        print("| " + " | ".join(str(r[c])[:44] for c in cols) + " |")
    by_model = {}
    for r in rows:
        by_model.setdefault(r["model"], []).append(r)
    print("\nWithin-model pairwise overlap (Jaccard): quote+lines keys / source lines quoted")
    for model, rs in by_model.items():
        if len(rs) < 2:
            print(f"  {model}: one run, nothing to compare")
            continue
        for a, b in itertools.combinations(rs, 2):
            print(f"  {model}: {a['run'][-14:]} vs {b['run'][-14:]}  "
                  f"keys {jaccard(a['_keys'], b['_keys']):.2f}  "
                  f"lines {jaccard(a['_lines'], b['_lines']):.2f}")
    if args.statements:
        for r in rows:
            print(f"\n{r['run']} ({r['frozen']} frozen):")
            for i, s in enumerate(r["_statements"], 1):
                print(f"  {i}. {s}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run")
    r.add_argument("--engagement", default="dataroom-fixture-doc9")
    r.add_argument("--model", required=True)
    r.add_argument("--n", type=int, default=3)
    r.add_argument("--label", required=True)
    r.set_defaults(fn=run)
    c = sub.add_parser("compare")
    c.add_argument("runs", nargs="+")
    c.add_argument("--statements", action="store_true", help="print each run's frozen statements")
    c.set_defaults(fn=compare)
    args = ap.parse_args()
    return args.fn(args)


if __name__ == "__main__":
    sys.exit(main())
