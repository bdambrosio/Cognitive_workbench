"""The review output schema, and the checks that run after it is parsed.

REVIEW.md §8 states the contract; this file is the machine half of it, and
`lint_workflow.check_review_vocab` is what keeps the two in step.

THE SAME DIVISION AS THE AUDIT'S SCHEMA. Shape and closed vocabularies are
enforced by the decoder; anything that has to look at the audit's output — does
this claim_id exist, is every finding reviewed — is checked here after parsing.
`claims_audit/schemas.py` carries the argument for that split and for the two
things deliberately left out of both schemas (no `oneOf`, no filename enum).

THE OUTCOME IS DERIVED, NEVER EMITTED. REVIEW.md §6 gives the reviewer four
independent observations and takes the conclusion away from it: a finding holds
when every observation is clean. Asking a model for a conclusion that follows
from its own observations buys nothing and lets it disagree with itself, which
is a defect with no upside. `derive_outcomes` is that arithmetic.
"""
from __future__ import annotations

import re
from typing import Any, Dict, List, Sequence, Tuple

#: REVIEW.md §6. Each answers one of §5's checks and is independent of the rest.
OBSERVATIONS: Dict[str, Tuple[str, ...]] = {
    "evidence_relevant":   ("yes", "no"),
    "evidence_supports":   ("yes", "no"),
    "verdict_calibration": ("correct", "overstated", "understated"),
    "searches_adequate":   ("yes", "no", "not_applicable"),
}

#: The value of each observation that raises nothing. `not_applicable` is clean
#: because it means the question did not arise — REVIEW.md §6 confines
#: `searches_adequate` to `unverifiable` findings.
CLEAN: Dict[str, Tuple[str, ...]] = {
    "evidence_relevant":   ("yes",),
    "evidence_supports":   ("yes",),
    "verdict_calibration": ("correct",),
    "searches_adequate":   ("yes", "not_applicable"),
}

#: REVIEW.md §9. Retested when adverse, because each asserts the audit's
#: evidence does not do a job and a second reader can settle whether that is so.
#: `verdict_calibration` is absent deliberately: a verdict one step off is
#: calibration, and two reviewers disagreeing about one step measures the
#: reviewers.
RETESTED: Tuple[str, ...] = (
    "evidence_relevant", "evidence_supports", "searches_adequate")

#: REVIEW.md §5 check 1, over the claim surface rather than the findings.
FIDELITY: Tuple[str, ...] = ("faithful", "overstates", "understates",
                             "unrelated")


def _claim_check() -> Dict[str, Any]:
    return {"type": "object", "properties": {
        "claim_id": {"type": "integer", "minimum": 1},
        "fidelity": {"enum": list(FIDELITY)},
        "note": {"type": "string"}},
        "required": ["claim_id", "fidelity"]}


def _finding_review() -> Dict[str, Any]:
    props: Dict[str, Any] = {
        "claim_id": {"type": "integer", "minimum": 1},
        "exception": {"type": "string"},
        "finding_says": {"type": "string"},
        "materials_show": {"type": "string"}}
    for name, values in OBSERVATIONS.items():
        props[name] = {"enum": list(values)}
    return {"type": "object", "properties": props,
            "required": ["claim_id", *OBSERVATIONS]}


# ---------------------------------------------------------------------------
# ONE SCHEMA PER PART, AND THE WHOLE AS THEIR UNION.
#
# The review's three parts are independent — claim fidelity is over the claim
# surface, the reviews are over findings, the record check is one statement
# about the audit — so each can be asked for on its own. That is what makes
# BATCHING A RUNNER DECISION rather than a schema change: a batch is a call for
# `finding_reviews_schema` over ten findings instead of forty, and the runner
# concatenates.
#
# It has to be built in from the start. Under constrained decoding a generation
# cut at max_tokens is invalid JSON rather than a short answer, so the loss is
# total; a review of two hundred findings that can only be asked for at once is
# a review that cannot be recovered when it breaks. The audit's adjudication
# call already takes a subset of claims for the same reason — its reprompt path
# passes only the claims that got no finding.
# ---------------------------------------------------------------------------

def claim_checks_schema() -> Dict[str, Any]:
    """Part one, or one batch of it: REVIEW.md §5 check 1."""
    return {"type": "object", "properties": {
        "claim_checks": {"type": "array", "items": _claim_check()}},
        "required": ["claim_checks"]}


def finding_reviews_schema() -> Dict[str, Any]:
    """Part two, or one batch of it: REVIEW.md §5 checks 2 to 5."""
    return {"type": "object", "properties": {
        "finding_reviews": {"type": "array", "items": _finding_review()}},
        "required": ["finding_reviews"]}


def record_check_schema() -> Dict[str, Any]:
    """Part three: REVIEW.md §7's one statement about the audit as a whole."""
    return {"type": "object", "properties": {
        "record_check": {"type": "string"}},
        "required": ["record_check"]}


def review_schema() -> Dict[str, Any]:
    """All three parts in one response, for a review small enough to ask at once."""
    return {"type": "object", "properties": {
        "claim_checks": {"type": "array", "items": _claim_check()},
        "finding_reviews": {"type": "array", "items": _finding_review()},
        "record_check": {"type": "string"}},
        "required": ["claim_checks", "finding_reviews", "record_check"]}


def merge_parts(parts: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    """Assemble batch responses into the one object `check_review` reads.

    Arrays concatenate in the order the batches were asked for; `record_check`
    takes the last non-empty one. Duplicates are NOT removed here — a claim
    reviewed twice is a defect `check_review` reports, and silently collapsing
    it would hide a batch boundary that went wrong.
    """
    out: Dict[str, Any] = {"claim_checks": [], "finding_reviews": [],
                           "record_check": ""}
    for part in parts:
        if not isinstance(part, dict):
            continue
        out["claim_checks"].extend(part.get("claim_checks") or [])
        out["finding_reviews"].extend(part.get("finding_reviews") or [])
        if (part.get("record_check") or "").strip():
            out["record_check"] = part["record_check"]
    return out


def derive_outcomes(obj: Dict[str, Any]) -> Dict[str, Any]:
    """Whether each finding holds, per REVIEW.md §6, and the totals.

    Not a judgement and not the reviewer's to make: a finding holds when every
    observation is clean. Returned separately rather than written back into the
    reviewer's object, so what the model said and what follows from it stay
    distinguishable in the record.
    """
    outcomes, adverse = {}, {}
    for r in obj.get("finding_reviews") or []:
        cid = r.get("claim_id")
        bad = [n for n in OBSERVATIONS
               if r.get(n) not in CLEAN[n]]
        outcomes[cid] = {"holds": not bad, "adverse_observations": bad}
        for n in bad:
            adverse[n] = adverse.get(n, 0) + 1
    holds = sum(1 for o in outcomes.values() if o["holds"])
    return {"outcomes": outcomes,
            "holds": holds,
            "does_not_hold": len(outcomes) - holds,
            "adverse_by_observation": adverse,
            # What §9 sends to a second reviewer: the adverse observations that
            # qualify, plus the sample of clean findings the process draws as a
            # positive control. The sample itself is the runner's to choose.
            "retest_candidates": sorted(
                cid for cid, o in outcomes.items()
                if any(n in RETESTED for n in o["adverse_observations"]))}


def check_review(obj: Dict[str, Any], frozen: Sequence[Dict[str, Any]],
                 findings: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    """What REVIEW.md requires that the schema cannot express."""
    problems: List[str] = []
    claim_ids = {c.get("id") for c in frozen}
    finding_ids = {f.get("claim_id") for f in findings}
    verdicts = {f.get("claim_id"): (f.get("adjudication") or {}).get("verdict")
                for f in findings}

    seen_cc = set()
    for cc in obj.get("claim_checks") or []:
        cid = cc.get("claim_id")
        if cid not in claim_ids:
            problems.append(f"claim_check {cid!r} is not in the frozen surface")
        elif cid in seen_cc:
            problems.append(f"claim {cid} is checked twice")
        seen_cc.add(cid)
        if cc.get("fidelity") != "faithful" and not (cc.get("note") or "").strip():
            problems.append(f"claim {cid}: fidelity {cc.get('fidelity')!r} "
                            f"requires a note (REVIEW §8)")
    for missing in sorted(claim_ids - seen_cc, key=lambda x: (x is None, x)):
        problems.append(f"claim {missing} in the frozen surface was not checked")

    seen_fr = set()
    for r in obj.get("finding_reviews") or []:
        cid = r.get("claim_id")
        w = f"finding_review {cid}"
        if cid not in finding_ids:
            problems.append(f"{w}: no finding adjudicates that claim")
        elif cid in seen_fr:
            problems.append(f"{w}: reviewed twice")
        seen_fr.add(cid)
        # §6 confines searches_adequate to unverifiable findings. A yes/no on a
        # finding that made no searches is an opinion about nothing.
        sa, verdict = r.get("searches_adequate"), verdicts.get(cid)
        if verdict == "unverifiable" and sa == "not_applicable":
            problems.append(f"{w}: the finding is `unverifiable`, so its "
                            f"searches are the evidence and must be judged")
        if verdict is not None and verdict != "unverifiable" \
                and sa != "not_applicable":
            problems.append(f"{w}: `searches_adequate` is {sa!r} on a "
                            f"`{verdict}` finding, which made no searches")
        bad = [n for n in OBSERVATIONS if r.get(n) not in CLEAN[n]]
        if bad and not (r.get("exception") or "").strip():
            problems.append(f"{w}: {', '.join(bad)} adverse and no exception "
                            f"(REVIEW §6)")
        if bad and not (r.get("materials_show") or "").strip():
            problems.append(f"{w}: an exception must quote what the cited "
                            f"material says (REVIEW §8)")
    for missing in sorted(finding_ids - seen_fr, key=lambda x: (x is None, x)):
        problems.append(f"finding on claim {missing} was not reviewed")

    if not (obj.get("record_check") or "").strip():
        problems.append("no `record_check` — REVIEW §7 requires one statement "
                        "about the audit as a whole")

    derived = derive_outcomes(obj)
    return {"ok": not problems, "problems": problems,
            "figures": {"claims_checked": len(seen_cc),
                        "findings_reviewed": len(seen_fr),
                        "holds": derived["holds"],
                        "does_not_hold": derived["does_not_hold"],
                        "adverse_by_observation":
                            derived["adverse_by_observation"],
                        "retest_candidates": len(derived["retest_candidates"]),
                        "fidelity": _tally(obj.get("claim_checks") or [],
                                           "fidelity")}}


def _tally(rows: Sequence[Dict[str, Any]], field: str) -> Dict[str, int]:
    out: Dict[str, int] = {}
    for r in rows:
        out[r.get(field)] = out.get(r.get(field), 0) + 1
    return out
