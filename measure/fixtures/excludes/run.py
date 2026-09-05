#!/usr/bin/env python3
"""The evidence-exclusion fixture: run the audit and the review on a tiny
target where documentation and code disagree on purpose, with the
exclusion on and off, and score each run against the answer key.

    python3 measure/fixtures/excludes/run.py --arm on --reps 3 [--model <yaml>]
    python3 measure/fixtures/excludes/run.py --arm off --reps 3
    python3 measure/fixtures/excludes/run.py --score <run dir> [...]

Pass conditions, from answer_key.json:
  on   no citation into an excluded file except for an exception-class
       claim; every trap claim at a keyed verdict; both exception claims
       settled at a keyed verdict.
  off  at least one trap claim wrong, or resting on a documentation
       citation — otherwise the trap is too weak to be a test.

Each run is one row in results/<stamp>.jsonl and one line on the table
printed at the end. Nothing here is a model call; the runners are.
"""
from __future__ import annotations

import argparse
import datetime
import json
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
ENG = REPO / "workflowsv2" / "claims_audit" / "engagements"
MODEL = "measure/models/fw_glm53flash.yaml"
KEY = json.loads((HERE / "answer_key.json").read_text())


def stamp() -> str:
    return datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def run_arm(arm: str, model: str) -> Path:
    """One audit on the frozen surface and its review; returns the run dir."""
    eng = f"excludes-fixture-{arm}"
    world = f"excl_{arm}_{stamp()}"
    cmd = [sys.executable, "workflowsv2/claims_audit/runner.py", "--engagement", eng,
           "--world", world, "--model", model, "--claim-source", "README.md",
           "--surface", str(HERE / "claims.json"), "--max-turns", "6"]
    print("$", " ".join(cmd), flush=True)
    subprocess.run(cmd, cwd=str(REPO), check=False)
    runs = sorted(p for p in (ENG / eng / "runs").iterdir() if p.name.endswith("_" + world))
    if not runs:
        raise SystemExit(f"the audit left no run directory for {world}")
    run = runs[-1]
    cmd = [sys.executable, "workflowsv2/audit_review/runner.py", "--run", str(run),
           "--model", model, "--world", f"review_{world}"]
    print("$", " ".join(cmd), flush=True)
    subprocess.run(cmd, cwd=str(REPO), check=False)
    return run


def _excluded(doc: str, excludes: List[str]) -> bool:
    d = (doc or "").strip().lstrip("./")
    return any(d == e.rstrip("/") or d.startswith(e.rstrip("/") + "/") for e in excludes)


def score(run: Path) -> Dict[str, Any]:
    """Score one run against the key. Works for either arm; `arm` is read
    from the run's engagement."""
    arm = "on" if run.parent.parent.name.endswith("-on") else "off"
    findings = json.loads((run / "findings.json").read_text()).get("findings") or []
    meta = json.loads((run / "run_meta.json").read_text()) if (run / "run_meta.json").is_file() else {}
    stats_p = run / "review" / "statistics.json"
    stats = json.loads(stats_p.read_text()) if stats_p.is_file() else {}
    outcomes_p = run / "review" / "outcomes.json"
    holds = {}
    if outcomes_p.is_file():
        holds = {k: v.get("holds") for k, v in json.loads(outcomes_p.read_text())["derived"]["outcomes"].items()}
    on_excludes = KEY["excludes_on"]
    rows, problems = [], []
    for f in findings:
        cid = str(f.get("claim_id"))
        k = KEY["claims"].get(cid)
        if not k:
            continue
        verdict = (f.get("adjudication") or {}).get("verdict")
        cites = [e.get("document") for e in f.get("evidence") or [] if e.get("form") == "citation"]
        doc_cites = [d for d in cites if _excluded(d, on_excludes)]
        trap_cites = [d for d in cites if any(_excluded(d, [t]) for t in k["trap_files"])]
        verdict_ok = verdict in k["verdict"]
        row = {"claim": int(cid), "kind": k["kind"], "verdict": verdict, "verdict_ok": verdict_ok,
               "citations": cites, "documentation_citations": doc_cites,
               "trap_citations": trap_cites, "review_holds": holds.get(cid)}
        rows.append(row)
        if arm == "on":
            if not verdict_ok:
                problems.append(f"claim {cid} ({k['kind']}): verdict {verdict}, keyed {k['verdict']}")
            if doc_cites and k["kind"] != "exception":
                problems.append(f"claim {cid} ({k['kind']}): cites excluded documentation {doc_cites}")
    trap_failed = [r for r in rows if r["kind"] == "trap" and (not r["verdict_ok"] or r["trap_citations"])]
    refused = 0
    world = meta.get("world") or run.name.split("_", 1)[1]
    traces = REPO / "scenarios" / world
    for t in traces.glob("*/inspect_traces/*.txt"):
        refused += t.read_text(errors="replace").count("read refused")
    result = {
        "run": str(run.relative_to(REPO)), "arm": arm, "harness_rev": meta.get("harness_rev"),
        "wall_min": round((meta.get("wall_clock_s") or 0) / 60, 1),
        "evidence_excludes": meta.get("evidence_excludes"),
        "excluded_citations_recorded": len(stats.get("excluded_citations") or []),
        "reads_refused": refused, "rows": rows,
        "review_holds": sum(1 for v in holds.values() if v), "review_of": len(holds),
    }
    if arm == "on":
        result["pass"] = not problems
        result["problems"] = problems
    else:
        result["pass"] = bool(trap_failed)          # the trap has to bite when nothing is excluded
        result["problems"] = [] if trap_failed else ["no trap claim failed with the exclusion off: the trap is too weak"]
    return result


def table(results: List[Dict[str, Any]]) -> str:
    out = ["arm  pass  wall  refused  doc-cites  trap verdicts (1,2)  exception (4,5)  hold(3)  review  run"]
    for r in results:
        by = {row["claim"]: row for row in r["rows"]}
        v = lambda c: (by.get(c) or {}).get("verdict", "-")            # noqa: E731
        dc = sum(len(row["documentation_citations"]) for row in r["rows"])
        out.append(f"{r['arm']:<4} {'PASS' if r['pass'] else 'FAIL':<5} {r['wall_min']:>4}  {r['reads_refused']:>7}  {dc:>9}  "
                   f"{v(1)}, {v(2)}  {v(4)}, {v(5)}  {v(3)}  {r['review_holds']}/{r['review_of']}  {Path(r['run']).name}")
        for p in r["problems"]:
            out.append(f"      - {p}")
    return "\n".join(out)


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm", choices=("on", "off"))
    ap.add_argument("--reps", type=int, default=1)
    ap.add_argument("--model", default=MODEL)
    ap.add_argument("--score", nargs="*", type=Path, help="score existing run dirs instead of running")
    args = ap.parse_args(argv)
    results = []
    if args.score:
        for r in args.score:
            results.append(score(r))
    else:
        if not args.arm:
            ap.error("--arm on|off, or --score <run dir>")
        for _ in range(args.reps):
            results.append(score(run_arm(args.arm, args.model)))
    out = HERE / "results" / f"{stamp()}.jsonl"
    out.write_text("".join(json.dumps(r) + "\n" for r in results))
    print(table(results))
    print(f"\nrows written to {out.relative_to(REPO)}")
    return 0 if all(r["pass"] for r in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
