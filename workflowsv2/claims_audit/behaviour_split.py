"""Split claims about behaviour into a mechanism claim and a behaviour claim,
after enumeration and before the surface is frozen.

    propose(backend, section_body, claims) -> {"splits": [...], "parse": ..., "raw": ...}
    apply(claims, splits, by) -> the record of what changed

WHY A SEPARATE STEP, AND WHY NOT `decompose`. A sentence that says what the
software does when it runs asserts a mechanism, which code settles, and
behaviour in operation, which code does not settle (METHOD §7). One claim
gets one verdict, so a verdict on the mechanism reads as a verdict on the
behaviour. `decompose` unpacks a category claim into the properties a buyer
would take it to assert, on the practice's request, one claim at a time.
This pass does one fixed thing to one kind of sentence, over every claim of
a section in one call, without being asked. The two share plumbing and not
meaning, so this pass has its own method document, BEHAVIOUR_SPLIT.md.

WHAT A SPLIT LOOKS LIKE IN THE SURFACE. The parent claim stays and its
`statement` becomes the mechanism reading. One new claim is appended with
the parent's quote and lines, `implied_by` the parent's id, `property`
"behaviour in operation", and `approved_by` naming this pass rather than a
person. A parent left as it was would still be adjudicated whole, which is
the defect being removed. The statement the parent had before is kept in
the run record, not in the surface.

ACCEPTED UNLESS REMOVED (Bruce, 2026-09-15). The split rows are ordinary
draft claims. The practice sees "implied by N" on the surface page and drops
a split it does not accept. A wrong split is an extra claim in view; a split
that waited for a click and did not get one would be a missing claim nobody
sees. A run nobody scrubs keeps every split.
"""
from __future__ import annotations

import sys
import types
from pathlib import Path
from typing import Any, Dict, List, Sequence

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
for p in (str(REPO), str(REPO / "src")):
    if p not in sys.path:
        sys.path.insert(0, p)

from workflowsv2.emit import emit                               # noqa: E402
from workflowsv2.claims_audit.decompose import append_subclaims  # noqa: E402
from chat.workflow import load_workflow                        # noqa: E402

METHOD_PATH = HERE / "method" / "BEHAVIOUR_SPLIT.md"
PROPERTY = "behaviour in operation"
BY = "behaviour split, automatic"


def schema() -> Dict[str, Any]:
    item = {"type": "object", "properties": {
        "claim_id": {"type": "integer", "minimum": 1},
        "mechanism": {"type": "string"},
        "behaviour": {"type": "string"}},
        "required": ["claim_id", "mechanism", "behaviour"]}
    return {"type": "object", "properties": {
        "splits": {"type": "array", "items": item}},
        "required": ["splits"]}


def eligible(claim: Dict[str, Any]) -> bool:
    """Claims the pass considers: about the target, not a restatement, not
    already read out of another claim."""
    return (claim.get("about") == "target" and not claim.get("restates")
            and claim.get("implied_by") is None)


def propose(backend, section_body: str, claims: Sequence[Dict[str, Any]],
            max_tokens: int = 8192) -> Dict[str, Any]:
    """One schema-constrained emission under BEHAVIOUR_SPLIT.md for the
    claims of one section. `section_body` is the section's text with line
    numbers, as the enumerator saw it. Returns {"splits": [...], "parse": ...,
    "parse_error": ..., "raw": ...}; a split naming an id not in `claims`,
    or with an empty statement, is dropped."""
    method = load_workflow(METHOD_PATH)
    rows = [c for c in claims if eligible(c)]
    if not rows:
        return {"splits": [], "parse": None, "parse_error": None, "raw": ""}
    listed = "\n".join(f"  {c.get('id')}. quote: {c.get('quote')}\n"
                       f"      statement: {c.get('statement')}" for c in rows)
    user = (f"The section of the claim source, with its line numbers:\n\n"
            f"{section_body}\n\n"
            f"The claims enumerated from it:\n\n{listed}\n\n"
            f"Emit the splits per BEHAVIOUR_SPLIT.md §4.")
    out = emit(types.SimpleNamespace(backend=backend), method, user, schema(), max_tokens)
    obj = out.get("obj") if isinstance(out.get("obj"), dict) else {}
    ids = {int(c.get("id")) for c in rows}
    splits, seen = [], set()
    for s in obj.get("splits") or []:
        if not isinstance(s, dict):
            continue
        try:
            cid = int(s.get("claim_id"))
        except (TypeError, ValueError):
            continue
        mech = str(s.get("mechanism") or "").strip()
        beh = str(s.get("behaviour") or "").strip()
        if cid in ids and cid not in seen and mech and beh:
            splits.append({"claim_id": cid, "mechanism": mech, "behaviour": beh})
            seen.add(cid)
    return {"splits": splits, "parse": out.get("parse"),
            "parse_error": out.get("parse_error"), "raw": out.get("raw")}


def apply(claims: List[Dict[str, Any]], splits: Sequence[Dict[str, Any]],
          by: str = BY) -> List[Dict[str, Any]]:
    """Apply proposed splits to `claims` in place: the parent's statement
    becomes the mechanism reading and a behaviour subclaim is appended.
    Returns one record per split, for the run record:
    {claim_id, statement_before, mechanism, behaviour, behaviour_id}."""
    by_id = {int(c.get("id")): c for c in claims if c.get("id") is not None}
    applied = []
    for s in splits:
        parent = by_id.get(int(s["claim_id"]))
        if parent is None or not eligible(parent):
            continue
        before = parent.get("statement")
        parent["statement"] = s["mechanism"]
        rows = append_subclaims(claims, parent,
                                [{"statement": s["behaviour"], "property": PROPERTY}], by)
        applied.append({"claim_id": parent["id"], "statement_before": before,
                        "mechanism": s["mechanism"], "behaviour": s["behaviour"],
                        "behaviour_id": rows[0]["id"] if rows else None})
    return applied
