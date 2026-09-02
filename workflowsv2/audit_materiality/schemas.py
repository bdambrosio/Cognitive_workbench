"""The materiality output schema, and the checks that run after it is parsed.

MATERIALITY.md §3 and §7 state the contract; this file is the machine half,
and `lint_workflow.check_materiality_vocab` keeps the two in step. Same
division as the other two schema files: shape and the closed vocabulary in
the schema; everything that looks at the inputs — does this finding exist,
was every rateable finding rated — after parsing.
"""
from __future__ import annotations

from typing import Any, Dict, List, Sequence

#: MATERIALITY.md §3, in increasing order of consequence.
MATERIALITY = ("not_material", "material", "decisive")

#: MATERIALITY.md §4: every verdict but `real` is rated.
NOT_RATED_VERDICTS = ("real",)


def rateable(finding: Dict[str, Any]) -> bool:
    return (finding.get("adjudication") or {}).get("verdict") \
        not in NOT_RATED_VERDICTS


def ratings_schema() -> Dict[str, Any]:
    """One batch, or the whole set: MATERIALITY.md §7."""
    rating = {"type": "object", "properties": {
        "claim_source": {"type": "string"},
        "claim_id": {"type": "integer", "minimum": 1},
        "materiality": {"enum": list(MATERIALITY)},
        "basis": {"type": "string"}},
        "required": ["claim_source", "claim_id", "materiality", "basis"]}
    return {"type": "object", "properties": {
        "ratings": {"type": "array", "items": rating}},
        "required": ["ratings"]}


def merge_parts(parts: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    """Batches concatenate in the order asked. Duplicates are NOT removed: a
    finding rated twice is a defect `check_ratings` reports."""
    out: Dict[str, Any] = {"ratings": []}
    for part in parts:
        if isinstance(part, dict):
            out["ratings"].extend(part.get("ratings") or [])
    return out


def _key(source: Any, cid: Any) -> str:
    return f"{source}#{cid}"


def check_ratings(obj: Dict[str, Any], merged: Dict[str, Any]
                  ) -> Dict[str, Any]:
    """What MATERIALITY.md requires that the schema cannot express."""
    problems: List[str] = []
    findings = merged.get("findings") or []
    known = {_key(f["claim_source"], f["claim_id"]): f for f in findings}
    wanted = {k for k, f in known.items() if rateable(f)}
    seen = set()
    tally: Dict[str, int] = {}
    for r in obj.get("ratings") or []:
        k = _key(r.get("claim_source"), r.get("claim_id"))
        w = f"rating {k}"
        if k not in known:
            problems.append(f"{w}: no such finding")
            continue
        if k not in wanted:
            problems.append(f"{w}: a `real` finding is not rated "
                            f"(MATERIALITY §4)")
        if k in seen:
            problems.append(f"{w}: rated twice")
        seen.add(k)
        m = r.get("materiality")
        tally[m] = tally.get(m, 0) + 1
        if m not in MATERIALITY:
            problems.append(f"{w}: materiality {m!r} is not one of §3's")
        if not (r.get("basis") or "").strip():
            problems.append(f"{w}: no basis (MATERIALITY §5)")
    for k in sorted(wanted - seen):
        problems.append(f"finding {k} was not rated")
    return {"ok": not problems, "problems": problems,
            "figures": {"rateable": len(wanted), "rated": len(seen & wanted),
                        "materiality": tally}}
