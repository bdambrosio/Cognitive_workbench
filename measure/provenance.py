#!/usr/bin/env python3
"""Provenance metrics — where a reply's claims came from, and whether the
account it gives of itself survives contact with the trace.

Reads `claims.jsonl` (produced by the live grader) joined to the trace.
Reuses the production helpers in `src/chat/claims.py` rather than
reimplementing them: `valid_refs_for` for ref validity and
`_restore_observations` for un-capping the working log.

EVERY METRIC REPORTS ITS OWN DENOMINATOR. Coverage varies enormously —
`observations_full` is present on 150 of 3,010 jill_chat rows, and
`claims.jsonl` starts 2026-08-02 while the trace goes back to 05-03. A
provenance rate quoted without saying what it was computed over is not a
fact about the agent.

THE THREE FAILURE CLASSES, and which of them this file can see
(established 2026-08-22 against two labelled fabrications):

  extraction miss   a checkable assertion never became a claim
                    -> NOT detectable here. Only `extraction_density`
                       hints at it, and it is a proxy, not a measure.
  stale source      the cited source is genuine but no longer true
                    -> NOT detectable here. Needs re-checking the source
                       against current state, and only works for claims
                       about this repo/system.
  ref counterfeit   the cited step does not support the claim
                    -> detectable, and what `quote_gate` / `ref_validity`
                       address.

Do not report a "counterfeit rate" as if it covered all three.
"""

from __future__ import annotations

import json
import sys
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "src"))

from chat.claims import (                                    # noqa: E402
    _restore_observations, valid_refs_for,
)

from measure.trace import Turn, parse_ts                     # noqa: E402


def claims_path(world: str, agent: str) -> Path:
    return REPO / "scenarios" / world / agent / "memory" / "claims.jsonl"


def load_claim_rows(world: str, agent: str) -> List[Dict[str, Any]]:
    p = claims_path(world, agent)
    if not p.exists():
        return []
    rows = []
    for line in p.open(errors="replace"):
        line = line.strip()
        if not line:
            continue
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return rows


@dataclass
class Coverage:
    """What a number was computed over. Never report a rate without one."""
    n: int = 0
    of: int = 0
    note: str = ""

    def __str__(self) -> str:
        if not self.of:
            return f"n={self.n}"
        return f"{self.n}/{self.of}"


@dataclass
class ProvenanceReport:
    grounding: Counter = field(default_factory=Counter)
    total_claims: int = 0
    turns_with_claims: int = 0
    turns_total: int = 0
    reply_chars: int = 0

    quote_checked: int = 0
    quote_located: int = 0
    quote_uncheckable: int = 0        # capped log, no observations_full

    refs_checked: int = 0
    refs_invalid: int = 0

    volatile_model_prior: int = 0
    model_prior_total: int = 0

    def rate(self, num: int, den: int) -> Optional[float]:
        return round(num / den, 4) if den else None

    @property
    def extraction_density(self) -> Optional[float]:
        """Claims per 1,000 characters of reply. A PROXY for extraction
        recall, not a measure of it — a reply can be dense in prose and
        sparse in checkable assertions. Its purpose is to make a
        suspiciously low number visible, not to certify a high one."""
        if not self.reply_chars:
            return None
        return round(1000.0 * self.total_claims / self.reply_chars, 2)


def join_turn(turns_by_seq: Dict[Any, List[Turn]], row: Dict[str, Any]
              ) -> Optional[Turn]:
    """Find the trace turn a claims row describes.

    NEITHER KEY WORKS ALONE, and both failures are live:

    - `turn_seq` is not unique. One value repeats 235 times in jill_chat.
    - `claims.ts` is the GRADING time, not the turn time — grading runs
      after the turn, so the two timestamps never match. Joining on ts
      equality silently matches nothing, which looks like "no claims"
      rather than like a bug.

    So: match on turn_seq, then among the candidates take the latest whose
    turn started at or before grading time.
    """
    cands = turns_by_seq.get(row.get("turn_seq")) or []
    if not cands:
        return None
    if len(cands) == 1:
        return cands[0]
    graded_at = parse_ts(row.get("ts"))
    if graded_at is None:
        return cands[-1]
    earlier = [t for t in cands if t.ts is not None and t.ts <= graded_at]
    return earlier[-1] if earlier else cands[0]


def build(turns: List[Turn], claim_rows: List[Dict[str, Any]]
          ) -> ProvenanceReport:
    """Join claims to turns and compute the report. See `join_turn` for why
    the join is not a one-liner."""
    turns_by_seq: Dict[Any, List[Turn]] = {}
    for t in turns:
        turns_by_seq.setdefault(t.seq, []).append(t)
    for v in turns_by_seq.values():
        v.sort(key=lambda t: (t.ts is None, t.ts))

    rep = ProvenanceReport(turns_total=len(turns))

    for row in claim_rows:
        turn = join_turn(turns_by_seq, row)
        claims = row.get("claims") or []
        if not claims:
            continue
        rep.turns_with_claims += 1
        rep.total_claims += len(claims)
        if turn is not None:
            rep.reply_chars += turn.produced_chars

        allowed = valid_refs_for(turn.raw) if turn is not None else None
        log = ""
        restorable = False
        if turn is not None:
            full = turn.raw.get("observations_full") or {}
            restorable = bool(full)
            log = _restore_observations(turn.working_log, full)

        for c in claims:
            g = c.get("grounding")
            rep.grounding[g] += 1

            if g == "model_prior":
                rep.model_prior_total += 1
                if c.get("volatility") == "volatile":
                    rep.volatile_model_prior += 1

            if allowed is not None:
                for r in (c.get("refs") or []):
                    rep.refs_checked += 1
                    if r not in allowed:
                        rep.refs_invalid += 1

            quote = (c.get("quote") or "").strip()
            if not quote or turn is None:
                continue
            # A capped log without observations_full cannot settle the
            # question either way. Counting those as failures produced a
            # spurious 35% miss rate on first pass; they are uncheckable,
            # and are reported as such.
            if not restorable and "…[observation capped" in turn.working_log:
                rep.quote_uncheckable += 1
                continue
            rep.quote_checked += 1
            if quote[:60] in log:
                rep.quote_located += 1

    return rep


def render(rep: ProvenanceReport, label: str) -> str:
    out = [f"\n=== provenance — {label} ==="]
    tot = rep.total_claims
    out.append(f"  claims            {tot} over {rep.turns_with_claims} turns "
               f"(of {rep.turns_total} turns in window)")
    if rep.extraction_density is not None:
        out.append(f"  extraction density {rep.extraction_density} claims per "
                   f"1k reply chars   [PROXY — see module docstring]")
    if tot:
        out.append("  grounding mix:")
        for k, v in rep.grounding.most_common():
            out.append(f"      {str(k):15} {v:5}  {100.0 * v / tot:5.1f}%")
    if rep.model_prior_total:
        r = rep.rate(rep.volatile_model_prior, rep.model_prior_total)
        out.append(f"  volatile share of model_prior  {r}  "
                   f"({rep.volatile_model_prior}/{rep.model_prior_total})")
    if rep.quote_checked:
        r = rep.rate(rep.quote_located, rep.quote_checked)
        out.append(f"  quote gate        {r}  "
                   f"({rep.quote_located}/{rep.quote_checked} located)")
    if rep.quote_uncheckable:
        out.append(f"  quote uncheckable {rep.quote_uncheckable}  "
                   f"(capped log, no observations_full — NOT a failure)")
    if rep.refs_checked:
        out.append(f"  invalid refs      {rep.refs_invalid}/{rep.refs_checked}")
    return "\n".join(out)
