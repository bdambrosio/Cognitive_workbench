#!/usr/bin/env python3
"""Compare one audit run over three claim sources against three runs over one.

    python3 measure/fixtures/dataroom/split_compare.py \
        --baseline glm_base_0901 --split glm_doc1_0901 glm_doc2_0901 glm_doc9_0901 \
        --reference fixture_grok_0831

THE PROPOSAL UNDER TEST. The brief names three documents in which the seller
asserts things. A single run enumerates all three at once and the count is
unstable across models — 62, 67 and 273 from three backends on these same nine
documents. Running the audit once per claim source bounds each enumeration to
one document. Citation finding is unaffected either way: the target stays the
whole data room, because the evidence for a claim in one document usually sits
in another.

WHY COUNTS ARE NOT THE MEASURE. Two runs that both enumerate 28 claims may have
enumerated different sets of 28, so `overlap.py` compares the document lines a
surface cites instead. That matters more here than usual, for a reason specific
to splitting: a single run can merge one claim asserted in two sources, and
grok did — its Finding 30 cites doc1:19 and doc9:10 as one claim. Three
separate runs cannot make that merge, so the union of three surfaces is
expected to run high against a merged one. That offset is a property of the
design, not a finding about the model, and reporting a count without the line
sets would hide it.

REUSED, NOT REIMPLEMENTED. The ledger parser is `workflows.coverage`, the
finding parser is `workflows.citations`, the citation parser is `overlap.py`
beside this file, and the §5 field check is `measure/form_grid/run.py`.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
for p in (str(REPO), str(REPO / "src"), str(REPO / "measure" / "form_grid"), str(HERE)):
    if p not in sys.path:
        sys.path.insert(0, p)

from workflows import coverage, citations                      # noqa: E402
from overlap import cited_lines                                # noqa: E402
import run as form_grid                                        # noqa: E402

ENGAGEMENTS = REPO / "workflows" / "claims_audit" / "engagements"


def find_run(world: str) -> Path:
    """The newest run directory for a world, across every engagement.

    A world names a run, not an engagement, and the split runs live under three
    different engagements — so this searches all of them rather than requiring
    the caller to say which.
    """
    hits = sorted(ENGAGEMENTS.glob(f"*/runs/*_{world}"))
    if not hits:
        raise SystemExit(f"no run for world '{world}' under {ENGAGEMENTS}")
    return hits[-1]


def read(world: str) -> dict:
    d = find_run(world)
    report = (d / "report.md").read_text(errors="replace")
    ledger = coverage.parse_ledger(report)
    findings = citations.finding_claims(report)
    # The claim surface's own citations: the `Claim (...)` field of each
    # finding, not the whole report, whose Evidence lines cite the six evidence
    # documents and would swamp the comparison.
    claim_lines = [ln for ln in report.splitlines()
                   if ln.lstrip().lstrip("*").strip().lower().startswith("claim")]
    form = form_grid.form(report, len(ledger) or len(findings))
    return {"world": world, "dir": d, "ledger": len(ledger),
            "findings": len(findings), "cites": cited_lines("\n".join(claim_lines)),
            "full": form["fully_formed"], "collapse": form["collapse_at"],
            "emitted": form["findings_emitted"]}


def jaccard(a: set, b: set) -> float:
    return len(a & b) / len(a | b) if (a or b) else 0.0


def line(r: dict) -> str:
    return (f"{r['world']:20} ledger={r['ledger']:>3} findings={r['findings']:>3} "
            f"whole={r['full']:>3}/{r['emitted']:<3} "
            f"collapse={str(r['collapse'] or '—'):>4} "
            f"cited_lines={len(r['cites']):>4}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--baseline", required=True, help="world of the single run")
    ap.add_argument("--split", nargs="+", required=True,
                    help="worlds of the per-claim-source runs")
    ap.add_argument("--reference", default=None,
                    help="world to compare both against, e.g. a grok run")
    args = ap.parse_args()

    base = read(args.baseline)
    parts = [read(w) for w in args.split]
    ref = read(args.reference) if args.reference else None

    print("PER RUN\n")
    print("  " + line(base) + "   <- single run, all claim sources")
    for p in parts:
        print("  " + line(p))
    if ref:
        print("  " + line(ref) + "   <- reference")

    union = set().union(*(p["cites"] for p in parts))
    tot_ledger = sum(p["ledger"] for p in parts)
    tot_find = sum(p["findings"] for p in parts)
    print(f"\nSPLIT COMBINED\n\n  ledger={tot_ledger}  findings={tot_find}  "
          f"cited_lines={len(union)}")
    print(f"  overlap with single run: J={jaccard(union, base['cites']):.3f}  "
          f"(shared {len(union & base['cites'])}, "
          f"split-only {len(union - base['cites'])}, "
          f"single-only {len(base['cites'] - union)})")
    if ref:
        print(f"\nAGAINST REFERENCE ({ref['world']}, ledger={ref['ledger']}, "
              f"cited_lines={len(ref['cites'])})\n")
        for lbl, cites, n in (("single run", base["cites"], base["ledger"]),
                              ("split union", union, tot_ledger)):
            print(f"  {lbl:12} claims {n:>3} vs {ref['ledger']:<3} "
                  f"(gap {n - ref['ledger']:+d})   "
                  f"J={jaccard(cites, ref['cites']):.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
