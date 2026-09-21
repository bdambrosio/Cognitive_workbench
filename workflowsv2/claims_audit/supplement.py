"""A reviewed test of ONE claim that the delivered review listed and did not
test. The practice runs it, after delivery, when a client asks for it.

    python3 workflowsv2/claims_audit/supplement.py run     --engagement E --claim-source SRC --claim N --by NAME [--model yaml]
    python3 workflowsv2/claims_audit/supplement.py approve --engagement E --supplement <directory name> --by NAME
    python3 workflowsv2/claims_audit/supplement.py list    --engagement E

`run` writes a surface holding the one claim, runs the audit runner on it and
then the review runner on that audit, both unchanged and with the arguments
the chain job gives them, and writes what they found as a record:

    <engagement>/supplements/<stamp>_<claim source>_<N>/
        surface.json      the one claim (and its parent, when it has one)
        run.log           the two runners' output
        supplement.json   the record
        supplement.md     the record, for a person to read

The audit run itself lands in <engagement>/runs/ like any other. Nothing is
written under <engagement>/merged/, so the delivered run stays the current
run and the delivered report does not change. There is no materiality rating.

The model is the one the delivered audit of that claim source ran on, so the
result can be set beside the report's; `--model` names another, and the
record says so. No temperature is passed: it resolves per model, as in the
chain (src/chat/model_params.py).

`approve` records who read the result and when. A supplement that is not
approved is never shown to a client (record.py reads only approved ones).
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
for p in (str(REPO), str(REPO / "src")):
    if p not in sys.path:
        sys.path.insert(0, p)

from workflowsv2 import engagement_state as state                        # noqa: E402
from client_ui import jobs                                               # noqa: E402
from utils.file_utils import atomic_write_text, read_jsonl               # noqa: E402

SUPPLEMENTS = "supplements"
RECORD = "supplement.json"


class Refused(Exception):
    """The test cannot be run as asked; the message says why."""


def _read(path: Path) -> Dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _write(path: Path, obj: Any) -> None:
    atomic_write_text(path, json.dumps(obj, indent=1, ensure_ascii=False) + "\n")


def records(eng_dir: Path) -> List[Dict[str, Any]]:
    """Every supplement record of the engagement, oldest first, each with
    `dir`, the name of its directory."""
    out = []
    for f in sorted((eng_dir / SUPPLEMENTS).glob(f"*/{RECORD}")):
        out.append({**_read(f), "dir": f.parent.name})
    return out


def delivered_run(eng_dir: Path) -> Path:
    cur = state.current_run(eng_dir, state.load(eng_dir).get("current_intake"))
    if cur is None or not (cur / "report.md").is_file():
        raise Refused("the engagement has no current run with a report; there is nothing to supplement")
    return cur


def delivered_model(merged_dir: Path, claim_source: str) -> str:
    """The model file the delivered audit of this claim source ran on."""
    for r in _read(merged_dir / "meta.json").get("runs") or []:
        if r.get("claim_source") == claim_source:
            model = _read(Path(r["dir"]) / "run_meta.json").get("model_config")
            if model:
                return str(model)
    raise Refused(f"the delivered run records no audit of {claim_source!r}; name the model with --model")


def one_claim_surface(surface: Dict[str, Any], claim_source: str, cid: int) -> Dict[str, Any]:
    """The surface the audit runner is given: the one claim with its tier
    removed, because the runner tests a claim that has no tier 2 or 3
    (schemas.split_by_tier). A claim read from another (`implied_by`) needs
    that parent in the surface (schemas.check_surface); the parent is carried
    as a listed claim, so it is not tested here."""
    by_id = {c.get("id"): c for c in surface.get("claims") or []}
    claim = by_id.get(cid)
    if claim is None:
        raise Refused(f"{claim_source} has no claim {cid} in its frozen surface")
    if claim.get("tier") not in (2, 3):
        raise Refused(f"{claim_source} #{cid} is tier {claim.get('tier')}: the delivered review tested it")
    claims = [{k: v for k, v in claim.items() if k not in ("tier", "tier_basis")}]
    parent = by_id.get(claim.get("implied_by"))
    if parent is not None:
        listed = parent.get("tier") in (2, 3)
        claims.insert(0, parent if listed else {
            **parent, "tier": 2,
            "tier_basis": "carried so that the claim read from it can be tested alone; "
                          "its own finding is in the delivered report"})
    return {"claim_source": claim_source, "claims": claims}


def audit_command(eng: str, claim_source: str, surface: Path, model: str, world: str) -> List[str]:
    """The audit step, and below it the review step, with the chain job's
    argument shapes (client_ui/jobs.py, Chain.steps)."""
    return jobs._py("workflowsv2/claims_audit/runner.py", "--engagement", eng, "--world", world,
                    "--claim-source", claim_source, "--surface", str(surface), "--model", model)


def review_command(run_dir: Path, model: str, world: str) -> List[str]:
    return jobs._py("workflowsv2/audit_review/runner.py", "--run", str(run_dir), "--model", model,
                    "--world", f"review_{world}")


def _step(argv: List[str], log: Path) -> int:
    with open(log, "a", encoding="utf-8") as out:
        out.write(f"\n=== {state.stamp()}\n$ {' '.join(argv)}\n")
        out.flush()
        return subprocess.run(argv, cwd=str(REPO), stdout=out, stderr=subprocess.STDOUT).returncode


def result(run_dir: Path, cid: int) -> Dict[str, Any]:
    """What the two runners wrote about the claim: the finding as the audit
    wrote it, and the review's outcome for it."""
    meta = _read(run_dir / "run_meta.json")
    found = [f for f in _read(run_dir / "findings.json").get("findings") or [] if f.get("claim_id") == cid]
    out: Dict[str, Any] = {"resolved_model": meta.get("resolved_model"),
                           "resolved_temperature": meta.get("resolved_temperature"), "top_p": meta.get("top_p"),
                           "finding": found[0] if found else None, "review": None}
    outcomes = run_dir / "review" / "outcomes.json"
    if outcomes.is_file():
        o = _read(outcomes)
        mine = (o.get("derived", {}).get("outcomes") or {}).get(str(cid))
        if mine is not None:
            st = o.get("standings") or {}
            standing = (st.get("per_finding") or {}).get(str(cid)) or {}
            # The review retests only some exceptions, as it does in the chain.
            out["review"] = {"holds": bool(mine.get("holds")),
                             "adverse_observations": mine.get("adverse_observations") or [],
                             "standing": standing.get("standing")
                             or (f"not retested ({st.get('reason')})" if st.get("ran") is False else "not retested")}
    out["run_problems"] = [i for d in (run_dir, run_dir / "review") for i in read_jsonl(d / "issues.jsonl")
                           if i.get("severity") == "blocking"]
    return out


def markdown(rec: Dict[str, Any]) -> str:
    c, f, r = rec["claim"], rec.get("finding"), rec.get("review")
    rows = [f"# Tested after delivery: {rec['claim_source']} #{c['id']}", "",
            f"Engagement `{rec['engagement']}`. Delivered run `{rec['delivered_run']}`. "
            f"Run on {rec['at']} by {rec['by']}, on {rec.get('resolved_model') or rec['model']}"
            + ("" if rec["model_is_delivered"] else " (NOT the model the delivered audit ran on)") + ".", "",
            f"> {' '.join(str(c.get('quote') or '').split())}", "",
            f"**Statement:** {c.get('statement')}", "",
            f"**As delivered:** listed, not tested, tier {rec['tier']}. {rec.get('tier_basis') or ''}", ""]
    if rec.get("error"):
        rows += [f"**The test did not finish:** {rec['error']}", ""]
    if f:
        adj = f.get("adjudication") or {}
        rows += ["**Verdict:** " + str(adj.get("verdict"))
                 + "".join(f"  **{k}:** {v}" for k, v in adj.items() if k != "verdict"), ""]
        for e in f.get("evidence") or []:
            if e.get("form") == "citation":
                lo, hi = (e.get("lines") or [None, None])[:2]
                rows += [f"- `{e.get('document')}:{lo}-{hi}` {e.get('shows') or ''}",
                         "", "  ```", *("  " + x for x in str(e.get("quote") or "").splitlines()), "  ```"]
            else:
                rows += [f"- Search ({e.get('kind')}): {e.get('performed')} Result: {e.get('result')}"]
        rows.append("")
    if r:
        rows += ["**Independent check:** " + ("holds" if r["holds"] else "does not hold")
                 + (f" ({', '.join(r['adverse_observations'])})" if r["adverse_observations"] else "")
                 + (f"; retest: {r['standing']}" if r.get("standing") else ""), ""]
    elif not rec.get("error"):
        rows += ["**Independent check:** the review recorded no outcome for this claim.", ""]
    for p in rec.get("run_problems") or []:
        rows.append(f"- PROBLEM ({p.get('stage')}, {p.get('code')}): {p.get('text')}")
    rows += ["", f"Audit run: `{rec.get('audit_run')}`", "",
             "Not rated for materiality. The delivered report is unchanged. "
             + (f"Approved by {rec['approved']['by']} on {rec['approved']['at']}." if rec.get("approved")
                else "NOT APPROVED: not shown to the client until `supplement.py approve`.")]
    return "\n".join(rows) + "\n"


def run(eng_dir: Path, claim_source: str, cid: int, by: str, model: Optional[str] = None) -> Path:
    """Test one listed claim. Returns the supplement's directory."""
    job = state.running_job(eng_dir)
    if job:
        raise Refused(f"a {job.get('kind')} job is running for this engagement; wait for it to end")
    merged = delivered_run(eng_dir)
    for rec in records(eng_dir):
        if (rec["claim_source"], rec["claim"]["id"], rec["delivered_run"]) == (claim_source, cid, merged.name) \
                and not rec.get("error"):
            raise Refused(f"this claim was already tested after delivery: {SUPPLEMENTS}/{rec['dir']}")
    frozen = jobs.surface_file(eng_dir, claim_source)
    if not frozen.is_file():
        raise Refused(f"no frozen surface for {claim_source!r}")
    surface = _read(frozen)
    one = one_claim_surface(surface, claim_source, cid)
    claim = next(c for c in surface["claims"] if c.get("id") == cid)
    own = delivered_model(merged, claim_source) if model is None else None
    model = model or own

    ts = state.stamp()
    out = eng_dir / SUPPLEMENTS / f"{ts}_{jobs.slug(claim_source)}_{cid}"
    out.mkdir(parents=True)
    _write(out / "surface.json", one)
    world = f"supp_{eng_dir.name}_{jobs.slug(claim_source)}_{cid}_{ts}"
    rec: Dict[str, Any] = {"engagement": eng_dir.name, "claim_source": claim_source,
                           "claim": {k: claim.get(k) for k in ("id", "quote", "lines", "statement", "about")},
                           "tier": claim.get("tier"), "tier_basis": claim.get("tier_basis"),
                           "delivered_run": merged.name, "model": model, "model_is_delivered": own is not None,
                           "world": world, "at": ts, "by": by, "audit_run": None, "error": None,
                           "approved": None}
    log = out / "run.log"
    code = _step(audit_command(eng_dir.name, claim_source, out / "surface.json", model, world), log)
    run_dir = jobs._newest(eng_dir / "runs", world)
    if code != 0 or run_dir is None:
        rec["error"] = f"the audit exited {code}" if code else "the audit left no run directory"
    else:
        rec["audit_run"] = str(run_dir)
        code = _step(review_command(run_dir, model, world), log)
        if code != 0:
            rec["error"] = f"the review exited {code}"
        rec.update(result(run_dir, cid))
        if not rec["error"] and rec["finding"] is None:
            rec["error"] = "the audit wrote no finding for the claim"
    _write(out / RECORD, rec)
    atomic_write_text(out / "supplement.md", markdown(rec))
    return out


def approve(eng_dir: Path, name: str, by: str) -> Dict[str, Any]:
    f = eng_dir / SUPPLEMENTS / name / RECORD
    if not f.is_file():
        raise Refused(f"no supplement {name!r} in this engagement")
    rec = _read(f)
    if rec.get("error") or not rec.get("finding") or not rec.get("review"):
        raise Refused("this test did not finish with a finding and a check; it cannot be approved")
    rec["approved"] = {"by": by, "at": state.stamp()}
    _write(f, rec)
    atomic_write_text(f.parent / "supplement.md", markdown(rec))
    return rec


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("what", choices=("run", "approve", "list"))
    ap.add_argument("--engagement", required=True)
    ap.add_argument("--claim-source")
    ap.add_argument("--claim", type=int)
    ap.add_argument("--supplement", help="for `approve`: the supplement's directory name")
    ap.add_argument("--by", help="who runs or approves it")
    ap.add_argument("--model", default=None, help="default: the model the delivered audit of the claim source ran on")
    args = ap.parse_args()
    eng_dir = state.ENGAGEMENTS / args.engagement
    if not eng_dir.is_dir():
        raise SystemExit(f"no engagement {args.engagement!r}")
    try:
        if args.what == "list":
            for rec in records(eng_dir):
                v = ((rec.get("finding") or {}).get("adjudication") or {}).get("verdict")
                print(f"{rec['dir']}  {rec['claim_source']} #{rec['claim']['id']}  "
                      f"{rec.get('error') or v}  {'approved' if rec.get('approved') else 'not approved'}")
            return 0
        if not args.by:
            raise SystemExit("--by is needed: the record says who ran or approved the test")
        if args.what == "approve":
            if not args.supplement:
                raise SystemExit("approve needs --supplement")
            approve(eng_dir, args.supplement, args.by)
            print(f"approved: {args.supplement}")
            return 0
        if not args.claim_source or args.claim is None:
            raise SystemExit("run needs --claim-source and --claim")
        out = run(eng_dir, args.claim_source, args.claim, args.by, args.model)
    except Refused as e:
        raise SystemExit(f"refused: {e}")
    rec = _read(out / RECORD)
    print((out / "supplement.md").read_text(encoding="utf-8"))
    print(f"Record: {out}")
    return 1 if rec.get("error") else 0


if __name__ == "__main__":
    raise SystemExit(main())
