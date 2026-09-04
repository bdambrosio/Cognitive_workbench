"""The materiality output schema, and the checks that run after it is parsed.

MATERIALITY.md §3, §4 and §7 state the contract; this file is the machine
half, and `lint_workflow.check_materiality_vocab` keeps the two in step. Same
division as the other two schema files: shape and the closed vocabulary in
the schema; everything that looks at the inputs — does this finding exist,
was every rateable finding rated — after parsing.

TWO RATINGS, ONE SCALE, NEVER ONE TOTAL. A finding with a verdict about the
claim gets `materiality`; an `unverifiable` finding gets `exposure`. Until
2026-09-02 unverifiable was rated "as if false" under `materiality`, and on
ChatterMate every `decisive` was an unsettled claim — a mark against the
seller for something the audit never showed. `unverifiable` is deliberately
not adverse (claims_audit.schemas.ADVERSE_VERDICTS); this split keeps it so.
"""
from __future__ import annotations

from typing import Any, Dict, List, Sequence

#: MATERIALITY.md §3, in increasing order of consequence. `exposure` uses the
#: same values.
MATERIALITY = ("not_material", "material", "decisive")

#: MATERIALITY.md §4: the verdicts rated for `materiality`, and the one rated
#: for `exposure`. `real` is neither.
RATED_VERDICTS = ("contradicted", "partial", "real_with_caveat")
EXPOSED_VERDICTS = ("unverifiable",)


def _verdict(finding: Dict[str, Any]) -> Any:
    return (finding.get("adjudication") or {}).get("verdict")


def rateable(finding: Dict[str, Any]) -> bool:
    """Rated for `materiality`."""
    return _verdict(finding) in RATED_VERDICTS


def exposable(finding: Dict[str, Any]) -> bool:
    """Rated for `exposure`."""
    return _verdict(finding) in EXPOSED_VERDICTS


def ratings_schema() -> Dict[str, Any]:
    """One batch, or the whole set: MATERIALITY.md §7. Both arrays are always
    present; a batch fills one and leaves the other empty."""
    def item(field: str) -> Dict[str, Any]:
        return {"type": "object", "properties": {
            "claim_source": {"type": "string"},
            "claim_id": {"type": "integer", "minimum": 1},
            field: {"enum": list(MATERIALITY)},
            "basis": {"type": "string"}},
            "required": ["claim_source", "claim_id", field, "basis"]}
    return {"type": "object", "properties": {
        "ratings": {"type": "array", "items": item("materiality")},
        "exposures": {"type": "array", "items": item("exposure")}},
        "required": ["ratings", "exposures"]}


def merge_parts(parts: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    """Batches concatenate in the order asked. Duplicates are NOT removed: a
    finding rated twice is a defect `check_ratings` reports."""
    out: Dict[str, Any] = {"ratings": [], "exposures": []}
    for part in parts:
        if isinstance(part, dict):
            out["ratings"].extend(part.get("ratings") or [])
            out["exposures"].extend(part.get("exposures") or [])
    return out


def _key(source: Any, cid: Any) -> str:
    return f"{source}#{cid}"


def check_ratings(obj: Dict[str, Any], merged: Dict[str, Any]
                  ) -> Dict[str, Any]:
    """What MATERIALITY.md requires that the schema cannot express."""
    problems: List[str] = []
    findings = merged.get("findings") or []
    known = {_key(f["claim_source"], f["claim_id"]): f for f in findings}
    wanted = {"materiality": {k for k, f in known.items() if rateable(f)},
              "exposure": {k for k, f in known.items() if exposable(f)}}
    seen = {"materiality": set(), "exposure": set()}
    tally = {"materiality": {}, "exposure": {}}
    for field, array in (("materiality", "ratings"), ("exposure", "exposures")):
        other = "exposure" if field == "materiality" else "materiality"
        for r in obj.get(array) or []:
            k = _key(r.get("claim_source"), r.get("claim_id"))
            w = f"{field} {k}"
            if k not in known:
                problems.append(f"{w}: no such finding")
                continue
            if k in wanted[other]:
                problems.append(f"{w}: this finding is rated for {other}, "
                                f"not {field} (MATERIALITY §4)")
            elif k not in wanted[field]:
                problems.append(f"{w}: a `real` finding is not rated "
                                f"(MATERIALITY §4)")
            if k in seen[field]:
                problems.append(f"{w}: rated twice")
            seen[field].add(k)
            m = r.get(field)
            tally[field][m] = tally[field].get(m, 0) + 1
            if m not in MATERIALITY:
                problems.append(f"{w}: {field} {m!r} is not one of §3's")
            if not (r.get("basis") or "").strip():
                problems.append(f"{w}: no basis (MATERIALITY §5)")
        for k in sorted(wanted[field] - seen[field]):
            problems.append(f"finding {k} was not rated for {field}")
    return {"ok": not problems, "problems": problems,
            "figures": {"rateable": len(wanted["materiality"]),
                        "rated": len(seen["materiality"] & wanted["materiality"]),
                        "materiality": tally["materiality"],
                        "exposable": len(wanted["exposure"]),
                        "exposed": len(seen["exposure"] & wanted["exposure"]),
                        "exposure": tally["exposure"]}}


#: Rating replicates (Bruce, 2026-09-03). Two samples; a disagreement
#: escalates that finding to five; four or five agreeing ships with the
#: count; three to two, or no majority, ships the plurality marked
#: borderline with every basis kept, for the practice to decide.
REPLICATES_FIRST = 2
REPLICATES_ESCALATED = 5


def combine(field: str, samples: List[Dict[str, Any]]) -> Dict[str, Any]:
    """One rating from its samples. `samples` are rating rows for one
    finding, each with `field` and `basis`, in sampling order. Returns the
    shipped row: the majority (or plurality) value with the first basis
    given for it, plus `samples`, `agreement` ("4 of 5") and `borderline`."""
    values = [r.get(field) for r in samples]
    counts: Dict[str, int] = {}
    for v in values:
        counts[v] = counts.get(v, 0) + 1
    top = max(counts.values())
    leaders = [v for v, c in counts.items() if c == top]
    # The plurality; on a tie, the value sampled first.
    value = next(v for v in values if v in leaders)
    n = len(samples)
    borderline = n > REPLICATES_FIRST and (top < n - 1 or len(leaders) > 1) or \
        (n == REPLICATES_FIRST and top < n)
    first = next(r for r in samples if r.get(field) == value)
    return {"claim_source": first.get("claim_source"),
            "claim_id": first.get("claim_id"), field: value,
            "basis": first.get("basis"),
            "agreement": f"{top} of {n}", "borderline": bool(borderline),
            "samples": [{field: r.get(field), "basis": r.get("basis")}
                        for r in samples]}


def contested(field: str, a: Sequence[Dict[str, Any]], b: Sequence[Dict[str, Any]]
              ) -> List[str]:
    """The keys (`<source>#<id>`) whose two samples disagree, or that one
    pass rated and the other did not."""
    ka = {f"{r.get('claim_source')}#{r.get('claim_id')}": r.get(field) for r in a}
    kb = {f"{r.get('claim_source')}#{r.get('claim_id')}": r.get(field) for r in b}
    return sorted(k for k in set(ka) | set(kb) if ka.get(k) != kb.get(k))
