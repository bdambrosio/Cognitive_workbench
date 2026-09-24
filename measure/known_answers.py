#!/usr/bin/env python3
"""Score one engagement's run against a file of known answers.

    python3 measure/known_answers.py --answers measure/known_answers/chhoto.yaml \\
        --engagement cmp-chhoto-glm-med [--merged <merged run directory>]

For each answer, in this order:

  enumerated   a frozen claim whose own quote overlaps the answer's lines. A
               claim that reaches those lines only through `locations` does not
               count: that is how the README's "only the hit is recorded" was
               lost on 2026-09-24. Among several candidates the one whose
               statement is closest to the answer's is taken (the duplicates
               pass's embedder), and the similarity is printed so a wrong pick
               can be seen.
  tested       the claim has a finding in the merged run (tier 1, or no tier).
  verdict      the finding's verdict is one the answer lists.
  reason       the finding cites at least one of the answer's `cites_any` line
               ranges, as a citation or in a derivation's basis. A right verdict
               that cites none of them is right for a reason the answer does not
               accept (cmp-chhoto-glm-med INSTALLATION #101).

An answer whose claim source is not one of the engagement's is out of scope and
not scored. Nothing here reads a finding's prose.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml

REPO = Path(__file__).resolve().parent.parent
for p in (str(REPO), str(REPO / "src")):
    if p not in sys.path:
        sys.path.insert(0, p)

from workflowsv2.claims_audit.duplicates import embed      # noqa: E402

ENGAGEMENTS = REPO / "workflowsv2" / "claims_audit" / "engagements"


def _overlaps(a: List[int], b: List[int]) -> bool:
    return len(a) == 2 and len(b) == 2 and a[0] <= b[1] and b[0] <= a[1]


def _cited(finding: Dict[str, Any]) -> List[Dict[str, Any]]:
    """Every (document, lines) the finding cites, directly or as a basis."""
    out = []
    for e in finding.get("evidence") or []:
        if e.get("document"):
            out.append({"document": e["document"], "lines": e.get("lines")})
        for b in e.get("basis") or []:
            out.append({"document": b.get("document"), "lines": b.get("lines")})
    return out


def score(answers: List[Dict[str, Any]], eng: Path, merged: Path) -> List[Dict[str, Any]]:
    surfaces = {}
    for f in sorted((eng / "surface").glob("*.surface.json")):
        s = json.loads(f.read_text(encoding="utf-8"))
        surfaces[s["claim_source"]] = s["claims"]
    findings = {(f["claim_source"], f["claim_id"]): f
                for f in json.loads((merged / "merged.json").read_text(encoding="utf-8"))["findings"]}
    rows = []
    for a in answers:
        row: Dict[str, Any] = {"id": a["id"], "source": a["source"], "lines": a["lines"]}
        rows.append(row)
        claims = surfaces.get(a["source"])
        if claims is None:
            row["result"] = "out of scope"
            continue
        own = [c for c in claims if _overlaps(c.get("lines") or [], a["lines"])]
        if not own:
            via = [c["id"] for c in claims
                   if any(_overlaps(loc.get("lines") or [], a["lines"]) for loc in c.get("locations") or [])]
            row["result"] = "NOT ENUMERATED" + (f" (only as a location of #{', #'.join(map(str, via))})" if via else "")
            continue
        vecs = embed([a["statement"]] + [c.get("statement") or "" for c in own])
        sims = [float(vecs[0] @ v) for v in vecs[1:]]
        best = max(range(len(own)), key=lambda i: sims[i])
        c = own[best]
        row.update(claim=c["id"], similarity=round(sims[best], 2), tier=c.get("tier"),
                   statement=c.get("statement"))
        f = findings.get((a["source"], c["id"]))
        if f is None:
            row["result"] = "enumerated, not tested"
            continue
        verdict = (f.get("adjudication") or {}).get("verdict")
        row.update(verdict=verdict, review=(f.get("review") or {}).get("outcome"))
        ok_verdict = verdict in (a.get("verdict") or [])
        need = a.get("cites_any") or []
        cited = _cited(f)
        ok_reason = (not need) or any(n["document"] == x["document"] and _overlaps(x.get("lines") or [], n["lines"])
                                      for n in need for x in cited)
        row["result"] = ("right" if ok_verdict and ok_reason
                         else "right verdict, reason not cited" if ok_verdict
                         else f"WRONG VERDICT (expected {' or '.join(a['verdict'])})")
    return rows


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--answers", required=True, type=Path)
    ap.add_argument("--engagement", required=True)
    ap.add_argument("--merged", type=Path, default=None, help="default: the engagement's latest merged run")
    args = ap.parse_args()
    eng = ENGAGEMENTS / args.engagement
    merged: Optional[Path] = args.merged or max((eng / "merged").iterdir(), default=None)
    if merged is None:
        raise SystemExit(f"{eng}: no merged run")
    answers = yaml.safe_load(args.answers.read_text(encoding="utf-8"))["answers"]
    rows = score(answers, eng, merged)
    print(f"{args.engagement}  ({merged.name})")
    for r in rows:
        where = f"{r['source']}:{r['lines'][0]}-{r['lines'][1]}"
        detail = ""
        if "claim" in r:
            detail = f"  #{r['claim']} tier {r.get('tier')} sim {r['similarity']}"
            if "verdict" in r:
                detail += f"  {r['verdict']} / review {r['review']}"
        print(f"  {r['id']:26s} {where:28s} {r['result']}{detail}")
    scored = [r for r in rows if r["result"] != "out of scope"]
    enum = [r for r in scored if "claim" in r]
    tested = [r for r in enum if "verdict" in r]
    right_v = [r for r in tested if not r["result"].startswith("WRONG")]
    right = [r for r in tested if r["result"] == "right"]
    print(f"  enumerated {len(enum)}/{len(scored)}; tested {len(tested)}; "
          f"right verdict {len(right_v)}/{len(tested)}; right verdict and reason {len(right)}/{len(tested)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
