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

# METHOD §6: five verdicts a resolved claim can carry, plus the one status for
# a claim that was attempted and did not resolve.
SUPPORTED = ("real", "real, minor caveat", "real, operational caveat")
UNSUPPORTED = ("partial", "delta")
RESOLVED = SUPPORTED + UNSUPPORTED
UNRESOLVED = ("unverifiable",)


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
    unverifiable = sum(1 for v in verdicts if v in UNRESOLVED)
    resolved = supported + unsupported
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
        "unattempted": max(0, ident - resolved - unverifiable),
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


def statement(fig: Dict[str, Any]) -> str:
    """The canonical coverage sentence §1a specifies, from computed figures."""
    return (f"Coverage: {fig['resolved']} of {fig['identified']} identified "
            f"claims resolved; {fig['supported']} of {fig['resolved']} "
            f"resolved claims supported.")
