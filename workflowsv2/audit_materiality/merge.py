"""Merge the reviewed runs of one engagement into one findings file. Mechanical.

NO DEDUPLICATION AND NO JUDGEMENT, by decision (2026-09-02): the claim surface
is human-owned in production, so the same assertion appearing in two claim
sources is the human's to resolve before adjudication, not the merge's. The
one cross-run comparison made here is a string equality — two claims whose
normalised quote is identical across claim sources — and it is reported, never
resolved.

RUNS ARE NAMED, NEVER FOUND. An engagement's `runs/` holds void runs (a 429 on
the first call), superseded runs and good ones side by side, and nothing in a
run directory says which it is. The caller lists the runs to merge.
"""
from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

from workflowsv2.claims_audit.schemas import _norm

REQUIRED = ("claims.json", "findings.json", "run_meta.json")

#: The output check writes "finding N ..." for a problem with the Nth finding
#: (1-based, list position) and "claim N ..." for a frozen claim no finding
#: adjudicates. Only the first kind belongs to a finding.
_FINDING_PROBLEM = re.compile(r"^finding (\d+)\b")


def _read(p: Path) -> Any:
    return json.loads(p.read_text(encoding="utf-8"))


def load_run(run: Path) -> Dict[str, Any]:
    """One run directory, as the merge reads it. Raises on a missing input."""
    run = Path(run)
    missing = [f for f in REQUIRED if not (run / f).is_file()]
    if missing:
        raise FileNotFoundError(f"{run}: not a finished audit run — missing "
                                f"{', '.join(missing)}")
    claims = _read(run / "claims.json")
    findings = _read(run / "findings.json")
    meta = _read(run / "run_meta.json")
    # A review that ended in error leaves outcomes.json with nothing in it.
    # Such a run is unreviewed, not reviewed-and-clean.
    outcomes = None
    if (run / "review" / "outcomes.json").is_file():
        meta_r = (_read(run / "review" / "review_meta.json")
                  if (run / "review" / "review_meta.json").is_file() else {})
        if not meta_r.get("error"):
            outcomes = _read(run / "review" / "outcomes.json")
    return {"dir": run, "claims": claims, "findings": findings, "meta": meta,
            "outcomes": outcomes}


def _review_of(cid: Any, outcomes: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    """What the review concluded about the finding on this claim.

    `outcomes.json` keys claim ids as strings (JSON object keys); the audit's
    `claim_id` is an integer. Both are tried.
    """
    if not outcomes:
        return {"outcome": "unreviewed"}
    derived = (outcomes.get("derived") or {}).get("outcomes") or {}
    row = derived.get(str(cid)) if str(cid) in derived else derived.get(cid)
    if row is None:
        return {"outcome": "unreviewed"}
    out = {"outcome": "holds" if row.get("holds") else "does_not_hold",
           "adverse_observations": list(row.get("adverse_observations") or [])}
    per = ((outcomes.get("standings") or {}).get("per_finding") or {})
    stand = per.get(str(cid)) if str(cid) in per else per.get(cid)
    if stand:
        out["retest"] = stand
    return out


def merge(run_dirs: Sequence[Path]) -> Dict[str, Any]:
    """The merged findings of several runs, one claim source each."""
    runs = [load_run(Path(r)) for r in run_dirs]
    out_runs, findings, unclaimed, questions = [], [], [], []
    verdicts: Dict[str, int] = {}
    reviewed = unreviewed = 0
    run_problems: Dict[str, List[str]] = {}
    by_quote: Dict[str, List[Dict[str, Any]]] = {}

    for r in runs:
        src = r["claims"].get("claim_source") or r["findings"].get("claim_source") or ""
        meta = r["meta"]
        claims_by_id = {c.get("id"): c for c in r["claims"].get("claims") or []}
        # Problems the output check recorded, by finding position.
        per_finding: Dict[int, List[str]] = {}
        others: List[str] = []
        for p in ((meta.get("output_check") or {}).get("problems") or []):
            m = _FINDING_PROBLEM.match(p)
            if m:
                per_finding.setdefault(int(m.group(1)), []).append(p)
            else:
                others.append(p)
        if others:
            run_problems[src] = others
        out_runs.append({
            "dir": str(r["dir"]), "claim_source": src,
            "world": meta.get("world"),
            "resolved_model": meta.get("resolved_model"),
            "harness_rev": meta.get("harness_rev"),
            "target_rev": meta.get("target_rev"),
            "reviewed": r["outcomes"] is not None,
            "claims": len(claims_by_id),
            "findings": len(r["findings"].get("findings") or []),
            # Coverage figures the report stage places: what the run read,
            # how long it gathered, and the files its searches named that it
            # never opened (METHOD §8). All from run_meta; nothing judged.
            "files_read": len(meta.get("files_read") or {}),
            "gathering_legs": meta.get("gathering_legs"),
            "unopened_candidates": list(
                ((meta.get("output_check") or {}).get("figures") or {})
                .get("unopened_candidates") or [])})
        for i, f in enumerate(r["findings"].get("findings") or [], 1):
            cid = f.get("claim_id")
            c = claims_by_id.get(cid) or {}
            adj = f.get("adjudication") or {}
            v = adj.get("verdict")
            verdicts[v] = verdicts.get(v, 0) + 1
            review = _review_of(cid, r["outcomes"])
            if review["outcome"] == "unreviewed":
                unreviewed += 1
            else:
                reviewed += 1
            rec = {"claim_source": src, "claim_id": cid,
                   "quote": c.get("quote"), "lines": c.get("lines"),
                   "statement": c.get("statement"),
                   "about": c.get("about"),
                   "locations": list(c.get("locations") or []),
                   "adjudication": adj, "evidence": f.get("evidence") or [],
                   "review": review,
                   "citation_problems": per_finding.get(i, [])}
            if f.get("correction"):
                rec["correction"] = f["correction"]
            findings.append(rec)
            key = _norm(c.get("quote") or "")
            if key:
                by_quote.setdefault(key, []).append(rec)
        for u in r["findings"].get("unclaimed") or []:
            unclaimed.append({"claim_source": src, **(u or {})})
        for q in r["findings"].get("questions") or []:
            questions.append({"claim_source": src, "question": q})

    restated = []
    for key, recs in by_quote.items():
        sources = {x["claim_source"] for x in recs}
        if len(sources) > 1:
            restated.append({
                "quote": recs[0]["quote"],
                "in": [{"claim_source": x["claim_source"],
                        "claim_id": x["claim_id"],
                        "verdict": x["adjudication"].get("verdict")}
                       for x in recs]})

    return {"runs": out_runs, "findings": findings, "unclaimed": unclaimed,
            "questions": questions,
            "figures": {"runs": len(runs), "claims": sum(x["claims"] for x in out_runs),
                        "findings": len(findings), "verdicts": verdicts,
                        "reviewed": reviewed, "unreviewed": unreviewed,
                        "restated_quotes": restated,
                        "run_problems": run_problems}}
