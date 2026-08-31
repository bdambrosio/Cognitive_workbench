#!/usr/bin/env python3
"""Coverage figures, computed from the verdict ledger rather than stated.

WHY THIS EXISTS, and it is not that models are bad at arithmetic. On
2026-08-30 the coverage figures failed to reconcile on most runs on the board,
usually by exactly one, across two models and three effort settings. The cause
was structural. METHOD's coverage statement counts CLAIMS; the report is
organised by FINDINGS; and nothing required those to be in bijection.

Claim 31 on the ChatterMate medium run reads "knowledge worker polls every 60s
and fails jobs stuck in `processing` on startup; ticket investigator polls…" —
three seller assertions in one numbered claim, so no single truth value. The
report gave it `[partial]` as Finding 5, which was correct, and then listed the
true half among the supported claims as `31-poll`. One claim, one right
verdict, counted twice.

A calculator would not have helped. Counting the verdicts a report carries
counts findings, and would have produced the same wrong number with more
confidence.

SO THE LEDGER IS THE INPUT, NOT THE REPORT. METHOD §16's `=== COVERAGE ===`
block carries one line per claim — `31. [partial]` — and every figure here is
arithmetic over that. The auditor decides each verdict, which only it can do,
and states it once. Nothing downstream re-derives a judgement, and nothing asks
a model to compute a figure it will be quoted on.
"""
from __future__ import annotations

import re
from typing import Any, Dict, List, Optional, Tuple

# `31. [partial]` — a claim number, a verdict, nothing else on the line.
# Tolerant of decoration a model adds unbidden, strict about the shape.
_LEDGER_LINE = re.compile(r"(?m)^\s*\**\s*(\d+)\s*\.\s*\**\s*\[([^\]]+)\]\s*\**\s*$")

# METHOD §6: five verdicts a resolved claim can carry, plus the two statuses
# for a claim that reached no verdict — attempted and unsettled, or never
# reached at all. `[unattempted]` is a ledger status because §1a and §12 step
# 5 both permit the work to stop with claims unexamined, while §16 requires
# every frozen claim on exactly one ledger line: without it, a run that
# stopped early could not write a conforming ledger at all.
SUPPORTED = ("real", "real, minor caveat", "real, operational caveat")
UNSUPPORTED = ("partial", "delta")
RESOLVED = SUPPORTED + UNSUPPORTED
UNRESOLVED = ("unverifiable", "unattempted")


def parse_ledger(text: str) -> List[Tuple[int, str]]:
    """Every `<n>. [verdict]` line, in the order written."""
    return [(int(n), v.strip().lower()) for n, v in _LEDGER_LINE.findall(text or "")]


def surface_count(claim_surface: str) -> Optional[int]:
    """The claim count the surface froze, from its own count line (§12 step 2)."""
    m = re.search(r"(?m)^\s*(\d+)\s+claims\b", claim_surface or "")
    return int(m.group(1)) if m else None


def figures(ledger: List[Tuple[int, str]], identified: Optional[int]) -> Dict[str, Any]:
    """Every coverage figure §1a defines, and nothing a model was asked to add."""
    verdicts = [v for _, v in ledger]
    supported = sum(1 for v in verdicts if v in SUPPORTED)
    unsupported = sum(1 for v in verdicts if v in UNSUPPORTED)
    unverifiable = verdicts.count("unverifiable")
    resolved = supported + unsupported
    # A ledger written before `[unattempted]` existed omits those claims
    # rather than naming them, which `check` reports as missing; the residual
    # keeps their count visible in the figures either way.
    unattempted = (verdicts.count("unattempted")
                   or max(0, (identified or len(ledger)) - resolved - unverifiable))
    ident = identified if identified is not None else len(ledger)
    return {
        "identified": ident,
        "resolved": resolved,
        "supported": supported,
        "unsupported": unsupported,
        "unverifiable": unverifiable,
        # §1a: coverage = resolved / identified, consistency = supported / resolved
        "coverage_pct": round(100 * resolved / ident, 1) if ident else None,
        "consistency_pct": round(100 * supported / resolved, 1) if resolved else None,
        "unattempted": unattempted,
    }


def check(claim_surface: str, coverage_block: str) -> Dict[str, Any]:
    """Does the ledger account for the frozen surface exactly once each?

    NAMES THE DEFECT, NOT THE DISCREPANCY. "The numbers disagree" sends a
    person back through a 17,000-character report. "Claim 31 appears twice" and
    "claims 47, 48 are in the surface and not in the ledger" are the two things
    that can actually be wrong, and either is repaired in one line.
    """
    ledger = parse_ledger(coverage_block)
    ident = surface_count(claim_surface)
    seen: Dict[int, int] = {}
    for n, _ in ledger:
        seen[n] = seen.get(n, 0) + 1

    duplicated = sorted(n for n, c in seen.items() if c > 1)
    expected = set(range(1, ident + 1)) if ident else set()
    missing = sorted(expected - set(seen)) if ident else []
    unknown = sorted(set(seen) - expected) if ident else []
    bad_verdicts = sorted({v for _, v in ledger
                           if v not in RESOLVED + UNRESOLVED})

    problems: List[str] = []
    if ident is None:
        problems.append("the claim surface states no count, so the ledger "
                        "cannot be checked against it")
    if not ledger:
        problems.append("the coverage block carries no verdict ledger")
    if duplicated:
        problems.append("claims appear more than once in the ledger: "
                        + ", ".join(map(str, duplicated)))
    if missing:
        problems.append(f"{len(missing)} claim(s) in the surface are absent "
                        "from the ledger: " + ", ".join(map(str, missing[:12]))
                        + (" …" if len(missing) > 12 else ""))
    if unknown:
        problems.append("the ledger names claims that are not in the surface: "
                        + ", ".join(map(str, unknown[:12])))
    if bad_verdicts:
        problems.append("verdicts outside METHOD §6: "
                        + ", ".join(f"[{v}]" for v in bad_verdicts))

    return {"ok": not problems, "problems": problems,
            "identified": ident, "ledger_lines": len(ledger),
            "duplicated": duplicated, "missing": missing,
            "unknown": unknown, "bad_verdicts": bad_verdicts,
            "figures": figures(ledger, ident)}


def findings_check(report: str, coverage_block: str) -> Dict[str, Any]:
    """Is every claim the ledger gives a verdict backed by a finding that cites it?

    TRACEABILITY, NOT CARDINALITY. A finding is a relevant subset of the
    source; a claim is a seller assertion; the mapping between them is
    many-to-many and a verdict falls out of the match. Findings 8 covering the
    seven SQL-guardrail claims from one body of evidence, and claims 13 and 15
    appearing under two findings each, are both correct — an earlier version of
    this function reported both as defects, having mistaken a report's
    numbering for a rule about ratios.

    What is left when the ratios go is the one thing that survives any reading:
    the audit must not assert a verdict it never backed. Claim 29 on
    `cm_ledger_glm_1` carried `[real, minor caveat]` in the ledger and was named
    by no finding, so nothing cited it and nothing noticed.

    NAMES THE DEFECT, like `check` above. "Claim 29 has no finding" is repaired
    in one line; "the findings and the ledger disagree" is not.
    """
    from workflows.citations import finding_claims

    ledger = dict(parse_ledger(coverage_block))
    resolved = {n for n, v in ledger.items() if v in RESOLVED}
    unresolved = {n for n, v in ledger.items() if v in UNRESOLVED}

    # A `[derived]` finding resolves no seller claim (§6) and names none.
    claim_findings = [f for f in finding_claims(report) if f["verdict"] != "derived"]
    seen = {c for f in claim_findings for c in f["claims"]}

    missing = sorted(resolved - seen)
    unknown = sorted(seen - set(ledger))
    untraceable = [f["n"] for f in claim_findings if not f["claims"]]
    unresolved_as_finding = sorted(seen & unresolved)

    def _ids(ns, cap: int = 12) -> str:
        ns = list(ns)
        return ", ".join(map(str, ns[:cap])) + (" …" if len(ns) > cap else "")

    problems: List[str] = []
    if not ledger:
        problems.append("the coverage block carries no verdict ledger, so the "
                        "findings cannot be checked against it")
    if missing:
        problems.append(f"{len(missing)} resolved claim(s) are named by no "
                        "finding: " + _ids(missing))
    if unknown:
        problems.append("findings name claims that are not in the ledger: "
                        + _ids(unknown))
    if untraceable:
        problems.append("finding(s) name no claim, so nothing they establish "
                        "reaches the surface: " + _ids(untraceable))
    if unresolved_as_finding:
        problems.append("claims carrying an examination status are stated as "
                        "findings; METHOD \u00a76 reports them in the coverage "
                        "block: " + _ids(unresolved_as_finding))

    return {"ok": not problems, "problems": problems,
            "resolved": len(resolved), "claim_findings": len(claim_findings),
            "claims_in_findings": len(seen), "missing": missing,
            "unknown": unknown, "untraceable": untraceable,
            "unresolved_as_finding": unresolved_as_finding}


def statement(fig: Dict[str, Any]) -> str:
    """The canonical coverage sentence §1a specifies, from computed figures."""
    return (f"Coverage: {fig['resolved']} of {fig['identified']} identified "
            f"claims resolved; {fig['supported']} of {fig['resolved']} "
            f"resolved claims supported.")
