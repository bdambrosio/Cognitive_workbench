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
  tier         where the answer carries `tier`, the surface's tier for the
               claim is one the answer lists; a miss says whether the run
               rated the claim lower (a higher number) or higher. An answer
               with `tier` and no `verdict` is scored on the tier alone.
  tested       the claim has a finding in the merged run (tier 1, or no tier).
  verdict      the finding's verdict is one the answer lists.
  distance     for a wrong settled verdict, how many steps it lies from the
               nearest accepted one on the order real, real_with_caveat,
               partial, contradicted; `lenient` when it lies toward real,
               `harsh` when toward contradicted. `unverifiable` is not on the
               order: where the answer settles the claim it is counted as
               unsettled, not given a distance.
  materiality  where the answer carries `materiality` (for a gap) or
               `exposure` (for an unsettled claim), the run's rating in
               materiality.json is one the answer lists; a miss says whether
               the run rated lower or higher on the scale not_material,
               material, decisive. A run that settled a claim the answer leaves
               unsettled, or the reverse, has no rating of that kind and is
               reported as "not rated as such".
  in report    the finding reaches report.md whole: its section (a shown or
               unsettled claim) carries the record's gap text and every
               citation, or its row in the "Claims that hold" table carries
               every citation. This is what the buyer sees; a finding right in
               merged.json and cut or dropped in the report is not delivered.
  reason       the finding cites at least one of the answer's `cites_any` line
               ranges, as a citation or in a derivation's basis. A right verdict
               that cites none of them is right for a reason the answer does not
               accept (cmp-chhoto-glm-med INSTALLATION #101).

An answer whose claim source is not one of the engagement's is out of scope and
not scored. Answers are summed separately by `selected`: `failure` answers were
chosen because runs got them wrong, so their count says whether a known failure
came back and is not a rate; `sample` answers are a random draw from the
reviewed claim surface, and only their counts read as a rate; `tier-sample`
answers are a random draw across tiers, scored on the tier alone. Nothing here reads a finding's prose.
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml

REPO = Path(__file__).resolve().parent.parent
for p in (str(REPO), str(REPO / "src")):
    if p not in sys.path:
        sys.path.insert(0, p)

from workflowsv2.claims_audit.duplicates import embed      # noqa: E402
from workflowsv2.audit_report.render import _md_safe, _lines  # noqa: E402  (the report's own formatting)

ENGAGEMENTS = REPO / "workflowsv2" / "claims_audit" / "engagements"


ORDER = ("real", "real_with_caveat", "partial", "contradicted")
SCALE = ("not_material", "material", "decisive")


def _rating(ratings: Dict[str, Any], array: str, field: str, src: str, cid: Any) -> Optional[str]:
    for r in ratings.get(array) or []:
        if r.get("claim_source") == src and str(r.get("claim_id")) == str(cid):
            return r.get(field)
    return None


def _scale_off(got: str, accepted: List[str]) -> Optional[int]:
    if got not in SCALE or not any(a in SCALE for a in accepted):
        return None
    return min((SCALE.index(got) - SCALE.index(a) for a in accepted if a in SCALE), key=abs)


def _distance(verdict: str, accepted: List[str]) -> Optional[int]:
    """Signed steps from the nearest accepted verdict: negative is lenient, positive harsh.

    None when either side is off the order (`unverifiable`)."""
    on = [ORDER.index(v) for v in accepted if v in ORDER]
    if verdict not in ORDER or not on:
        return None
    return min((ORDER.index(verdict) - i for i in on), key=abs)


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


def _section(report: str, heading: str) -> str:
    m = re.search(rf"^{re.escape(heading)}\n(.*?)(?=^## |\Z)", report, re.S | re.M)
    return m.group(1) if m else ""


def in_report(report: str, f: Dict[str, Any]) -> Dict[str, Any]:
    """Where the finding appears in report.md and whether it appears whole."""
    src, cid = f["claim_source"], f["claim_id"]
    cites = [e for e in f.get("evidence") or [] if isinstance(e, dict) and e.get("form") == "citation"]
    m = re.search(rf"^### {re.escape(src)}, claim {cid} — .*?(?=^### |^## |\Z)", report, re.S | re.M)
    if m:
        sec = m.group(0)
        gap = (f.get("adjudication") or {}).get("gap")
        found = sum(1 for e in cites if f"`{e.get('document')}`, {_lines(e.get('lines'))}" in sec)
        return {"where": "section", "gap": (not gap) or (_md_safe(gap) in sec),
                "cites": (found, len(cites))}
    hold = _section(report, "## Claims that hold")
    row = re.search(rf"^\| {re.escape(src)} \| {cid} \| .*$", hold, re.M)
    if row:
        found = sum(1 for e in cites if f"`{e.get('document')}` {_lines(e.get('lines'))}" in row.group(0))
        return {"where": "hold row", "gap": True, "cites": (found, len(cites))}
    return {"where": "MISSING", "gap": False, "cites": (0, len(cites))}


def score(answers: List[Dict[str, Any]], eng: Path, merged: Path) -> List[Dict[str, Any]]:
    surfaces = {}
    for f in sorted((eng / "surface").glob("*.surface.json")):
        s = json.loads(f.read_text(encoding="utf-8"))
        surfaces[s["claim_source"]] = s["claims"]
    findings = {(f["claim_source"], f["claim_id"]): f
                for f in json.loads((merged / "merged.json").read_text(encoding="utf-8"))["findings"]}
    report_file = merged / "report.md"
    report = report_file.read_text(encoding="utf-8") if report_file.is_file() else None
    rating_file = merged / "materiality.json"
    ratings = json.loads(rating_file.read_text(encoding="utf-8")) if rating_file.is_file() else {}
    rows = []
    for a in answers:
        row: Dict[str, Any] = {"id": a["id"], "source": a["source"], "lines": a["lines"],
                               "selected": a.get("selected", "failure")}
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
        if a.get("tier"):
            exp = [int(t) for t in a["tier"]]
            got = c.get("tier")
            row.update(tier_expected=exp, tier_ok=got in exp,
                       tier_off=None if got is None else min((got - t for t in exp), key=abs))
        if not a.get("verdict"):
            row["result"] = "tier only"
            continue
        f = findings.get((a["source"], c["id"]))
        if f is None:
            row["result"] = "enumerated, not tested"
            continue
        verdict = (f.get("adjudication") or {}).get("verdict")
        row.update(verdict=verdict, review=(f.get("review") or {}).get("outcome"))
        if report is not None:
            row["report"] = in_report(report, f)
        for kind, array in (("materiality", "ratings"), ("exposure", "exposures")):
            if a.get(kind):
                got = _rating(ratings, array, kind, a["source"], c["id"])
                row[kind] = {"expected": a[kind], "got": got,
                             "off": None if got is None else _scale_off(got, a[kind])}
        ok_verdict = verdict in (a.get("verdict") or [])
        need = a.get("cites_any") or []
        cited = _cited(f)
        ok_reason = (not need) or any(n["document"] == x["document"] and _overlaps(x.get("lines") or [], n["lines"])
                                      for n in need for x in cited)
        if ok_verdict:
            row["result"] = "right" if ok_reason else "right verdict, reason not cited"
        elif verdict == "unverifiable":
            row["result"] = f"UNSETTLED (expected {' or '.join(a['verdict'])})"
        else:
            d = _distance(verdict, a.get("verdict") or [])
            row["distance"] = d
            how = "" if d is None else f", {abs(d)} step{'s' if abs(d) > 1 else ''} {'harsh' if d > 0 else 'lenient'}"
            row["result"] = f"WRONG VERDICT (expected {' or '.join(a['verdict'])}{how})"
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
            for kind in ("materiality", "exposure"):
                if r.get(kind):
                    k = r[kind]
                    if k["got"] is None:
                        detail += f"  {kind}: not rated as such"
                    elif k["off"] == 0:
                        detail += f"  {kind} ok"
                    else:
                        detail += (f"  {kind.upper()} {k['got']} (expected {' or '.join(k['expected'])}"
                                   + ("" if k["off"] is None else f", rated {'higher' if k['off'] > 0 else 'lower'}") + ")")
            if r.get("report"):
                rp = r["report"]; n, m = rp["cites"]
                whole = rp["where"] != "MISSING" and rp["gap"] and n == m
                detail += ("  report: whole" if whole else
                           f"  REPORT: {rp['where']}" + ("" if rp["gap"] else ", gap missing")
                           + (f", citations {n}/{m}" if n != m else ""))
            if "tier_ok" in r:
                detail += ("  tier ok" if r["tier_ok"]
                           else f"  TIER {r.get('tier')} (expected {' or '.join(map(str, r['tier_expected']))})")
        print(f"  {r['id']:26s} {where:28s} {r['result']}{detail}")
    for group in ("failure", "sample", "tier-sample"):
        scored = [r for r in rows if r["selected"] == group and r["result"] != "out of scope"]
        if not scored:
            continue
        enum = [r for r in scored if "claim" in r]
        tested = [r for r in enum if "verdict" in r]
        right_v = [r for r in tested if r["result"].startswith("right")]
        right = [r for r in tested if r["result"] == "right"]
        unsettled = [r for r in tested if r["result"].startswith("UNSETTLED")]
        dist = [r["distance"] for r in tested if r.get("distance") is not None]
        tiered = [r for r in enum if "tier_ok" in r]
        if tiered:
            off = [r["tier_off"] for r in tiered if not r["tier_ok"] and r["tier_off"] is not None]
            print(f"  {group} tiers: right {sum(r['tier_ok'] for r in tiered)}/{len(tiered)}; "
                  f"rated lower than expected {sum(d > 0 for d in off)}, higher {sum(d < 0 for d in off)}")
        if not tested and all("tier_ok" in r for r in enum):
            continue
        for kind in ("materiality", "exposure"):
            rated = [r for r in tested if r.get(kind)]
            if rated:
                have = [r for r in rated if r[kind]["got"] is not None]
                offs = [r[kind]["off"] for r in have if r[kind]["off"]]
                print(f"  {group} {kind}: right {sum(r[kind]['off'] == 0 for r in have)}/{len(have)}; "
                      f"rated lower {sum(o < 0 for o in offs)}, higher {sum(o > 0 for o in offs)}; "
                      f"not rated as such {len(rated) - len(have)}")
        delivered = [r for r in tested if r.get("report")]
        whole = [r for r in delivered if r["report"]["where"] != "MISSING" and r["report"]["gap"]
                 and r["report"]["cites"][0] == r["report"]["cites"][1]]
        if delivered:
            print(f"  {group} in report: whole {len(whole)}/{len(delivered)}; "
                  f"missing {sum(r['report']['where'] == 'MISSING' for r in delivered)}")
        print(f"  {group}: enumerated {len(enum)}/{len(scored)}; tested {len(tested)}; "
              f"right verdict {len(right_v)}/{len(tested)}; right verdict and reason {len(right)}/{len(tested)}; "
              f"unsettled {len(unsettled)}; wrong lenient {sum(d < 0 for d in dist)}, harsh {sum(d > 0 for d in dist)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
