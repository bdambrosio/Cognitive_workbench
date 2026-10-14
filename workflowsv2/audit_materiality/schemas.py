"""The materiality output schema, and the checks that run after it is parsed.

MATERIALITY.md §3, §4, §5a and §7 state the contract; this file is the machine
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

from typing import Any, Dict, List, Optional, Sequence

#: MATERIALITY.md §3, in increasing order of consequence. `exposure` uses the
#: same values.
MATERIALITY = ("not_material", "material", "decisive")

#: MATERIALITY.md §5a: how much of the target a change that closes a gap
#: would touch, in increasing order of risk. Never a time.
BANDS = ("none", "setting", "one_place", "several_places", "design")

#: MATERIALITY.md §8: the check's verdict on one `in_short`.
CHECK_VERDICTS = ("holds", "adds", "contradicts", "misattributes", "unmarked_knowledge")

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


def ratings_schema(sources: Optional[Sequence[str]] = None) -> Dict[str, Any]:
    """One batch, or the whole set: MATERIALITY.md §7. Both arrays are always
    present; a batch fills one and leaves the other empty.

    `sources` are the claim sources of the findings being rated; when given,
    `claim_source` can only be one of them. On cw-site (2026-09-19) one pass
    wrote `site/privacy.md` for `claim_sources/site/privacy.md` on twelve
    findings; the rows matched no finding and the ratings check failed the
    step after twenty minutes of calls."""
    def item(field: str) -> Dict[str, Any]:
        props = {"claim_source": {"enum": list(sources)} if sources else {"type": "string"},
                 "claim_id": {"type": "integer", "minimum": 1},
                 field: {"enum": list(MATERIALITY)},
                 "basis": {"type": "string"}}
        required = ["claim_source", "claim_id", field, "basis"]
        if field == "materiality":   # a gap the review showed has a remedy and a plain account
            props["remedy"] = remedy_schema()
            props["in_short"] = {"type": "string"}
            required += ["remedy", "in_short"]
        return {"type": "object", "properties": props, "required": required}
    return {"type": "object", "properties": {
        "ratings": {"type": "array", "items": item("materiality")},
        "exposures": {"type": "array", "items": item("exposure")}},
        "required": ["ratings", "exposures"]}


def remedy_schema() -> Dict[str, Any]:
    return {"type": "object", "properties": {"change": {"type": "string"},
                                             "band": {"enum": list(BANDS)}},
            "required": ["change", "band"]}


def observations_schema() -> Dict[str, Any]:
    """MATERIALITY.md §7: the observations, grouped by the fact they record,
    one rating per group."""
    return {"type": "object", "properties": {"observations": {"type": "array", "items": {
        "type": "object", "properties": {
            "members": {"type": "array", "items": {"type": "integer", "minimum": 1}},
            "materiality": {"enum": list(MATERIALITY)},
            "basis": {"type": "string"},
            "in_short": {"type": "string"},
            "remedy": remedy_schema()},
        "required": ["members", "materiality", "basis", "in_short", "remedy"]}}},
        "required": ["observations"]}


def _remedy_problems(w: str, remedy: Any) -> List[str]:
    if not isinstance(remedy, dict):
        return [f"{w}: no remedy (MATERIALITY §5a)"]
    out = []
    if remedy.get("band") not in BANDS:
        out.append(f"{w}: band {remedy.get('band')!r} is not one of §5a's")
    if remedy.get("band") != "none" and not (remedy.get("change") or "").strip():
        out.append(f"{w}: a remedy with band {remedy.get('band')!r} and no change (MATERIALITY §5a)")
    return out


def check_observations(obj: Dict[str, Any], n: int) -> Dict[str, Any]:
    """What MATERIALITY.md requires of the observation groups that the schema
    cannot express: every observation, numbered 1..n, in exactly one group."""
    problems: List[str] = []
    seen: Dict[int, int] = {}
    for g, row in enumerate(obj.get("observations") or [], 1):
        w = f"observation group {g}"
        members = row.get("members") or []
        if not members:
            problems.append(f"{w}: no members")
        for m in members:
            if not isinstance(m, int) or not 1 <= m <= n:
                problems.append(f"{w}: no observation {m!r}")
            elif m in seen:
                problems.append(f"{w}: observation {m} is also in group {seen[m]}")
            else:
                seen[m] = g
        if row.get("materiality") not in MATERIALITY:
            problems.append(f"{w}: materiality {row.get('materiality')!r} is not one of §3's")
        if not (row.get("basis") or "").strip():
            problems.append(f"{w}: no basis (MATERIALITY §5)")
        problems += _remedy_problems(w, row.get("remedy"))
        if not (row.get("in_short") or "").strip():
            problems.append(f"{w}: no in_short (MATERIALITY §5b)")
    problems += [f"observation {m} is in no group" for m in range(1, n + 1) if m not in seen]
    return {"ok": not problems, "problems": problems}


def in_short_checks_schema(keys: Sequence[str]) -> Dict[str, Any]:
    """MATERIALITY.md §8: one verdict per `in_short`, named by its key."""
    return {"type": "object", "properties": {"checks": {"type": "array", "items": {
        "type": "object", "properties": {
            "key": {"enum": list(keys)},
            "verdict": {"enum": list(CHECK_VERDICTS)},
            "words": {"type": "string"},
            "why": {"type": "string"}},
        "required": ["key", "verdict", "words", "why"]}}},
        "required": ["checks"]}


def in_short_rewrites_schema(keys: Sequence[str]) -> Dict[str, Any]:
    """MATERIALITY.md §8: an `in_short` written again after the check."""
    return {"type": "object", "properties": {"rewrites": {"type": "array", "items": {
        "type": "object", "properties": {
            "key": {"enum": list(keys)},
            "in_short": {"type": "string"}},
        "required": ["key", "in_short"]}}},
        "required": ["rewrites"]}


def verdicts_of(obj: Dict[str, Any], keys: Sequence[str]) -> Optional[Dict[str, Dict[str, Any]]]:
    """{key: check row} when the check gave every key exactly one verdict
    from §8, else None: a check that skipped or doubled a sentence did not
    check it."""
    rows = [r for r in (obj or {}).get("checks") or [] if isinstance(r, dict)]
    got = {r.get("key"): r for r in rows}
    if len(rows) != len(got) or set(got) != set(keys) or any(
            r.get("verdict") not in CHECK_VERDICTS for r in rows):
        return None
    return got


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
            if field == "materiality":
                problems += _remedy_problems(w, r.get("remedy"))
                if not (r.get("in_short") or "").strip() and not r.get("in_short_dropped"):
                    problems.append(f"{w}: no in_short (MATERIALITY §5b)")
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
            **{k: first[k] for k in ("remedy", "in_short") if first.get(k) is not None},
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
