"""Decompose one broad claim into testable subclaims, on the practice's
request from the surface editor, before the surface is frozen.

    propose(backend, claim, section, others, category_note) -> {"subclaims", "declined"}
    append_subclaims(claims, parent, proposals, by) -> the new claim rows

WHY A SEPARATE STEP AND NOT ENUMERATION. Enumeration writes down what the
seller said; decomposition writes down what a reasonable buyer would take a
broad statement to assert. The second is an interpretation, so it is made
on request, recorded with the parent it came from and the person who
approved it, and never at adjudication time (METHOD §5: the surface is
frozen before any claim is tested). The agent proposes; the practice edits
and approves on the surface page (Bruce, 2026-09-05).

A subclaim is an ordinary claim about the target with `implied_by` naming
its parent. Its `quote` and `lines` are the parent's, because the words are
the parent's; its `statement` is the practice's reading. Evidence gathering,
adjudication and the review work from the statement; the review's fidelity
check knows a subclaim's statement says more than its quote by design.
"""
from __future__ import annotations

import sys
import types
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
for p in (str(REPO), str(REPO / "src")):
    if p not in sys.path:
        sys.path.insert(0, p)

from workflowsv2.emit import emit                               # noqa: E402
from chat.workflow import load_workflow                        # noqa: E402

METHOD_PATH = HERE / "method" / "DECOMPOSE.md"
MAX_SUBCLAIMS = 6
#: Lines of the claim source shown on each side of the parent claim.
CONTEXT_LINES = 12


def schema() -> Dict[str, Any]:
    item = {"type": "object", "properties": {"statement": {"type": "string"},
                                             "property": {"type": "string"}},
            "required": ["statement", "property"]}
    declined = {"type": "object", "properties": {"text": {"type": "string"},
                                                 "why": {"type": "string"}},
                "required": ["text", "why"]}
    return {"type": "object", "properties": {
        "subclaims": {"type": "array", "items": item, "maxItems": MAX_SUBCLAIMS},
        "declined": {"type": "array", "items": declined}},
        "required": ["subclaims", "declined"]}


def backend_from_model(model_yaml: Path):
    """A bare model backend from a model file's llm_config, the way ChatLoop
    builds its own; no world, no persona. For one emission."""
    import yaml
    from chat.backend import _ChatBackend
    doc = yaml.safe_load(Path(model_yaml).read_text(encoding="utf-8")) or {}
    llm = doc.get("llm_config") or {}
    if not llm:
        raise SystemExit(f"{model_yaml}: no llm_config block")
    return _ChatBackend(
        server=llm.get("server", "local"), model=llm.get("model", ""),
        base_url=llm.get("vllm_url") or llm.get("base_url") or "http://127.0.0.1:5000",
        is_reasoning=llm.get("is_reasoning_model"), api_key=llm.get("api_key"),
        reasoning_effort=llm.get("reasoning_effort"), extra_body=llm.get("extra_body"))


def section_text(claim_source: Path, lines: Sequence[int], context: int = CONTEXT_LINES) -> str:
    body = claim_source.read_text(encoding="utf-8", errors="replace").splitlines()
    lo, hi = int(lines[0]), int(lines[1])
    a, b = max(1, lo - context), min(len(body), hi + context)
    return "\n".join(f"{n:>4}{'>' if lo <= n <= hi else ' '}|{body[n - 1]}" for n in range(a, b + 1))


def propose(backend, claim: Dict[str, Any], section: str,
            others: Sequence[Dict[str, Any]], category_note: Optional[str] = None,
            max_tokens: int = 4096) -> Dict[str, Any]:
    """One schema-constrained emission under DECOMPOSE.md. Returns
    {"subclaims": [...], "declined": [...], "parse": ..., "raw": ...}."""
    method = load_workflow(METHOD_PATH)
    listed = "\n".join(f"  {c.get('id')}. {c.get('statement')}" for c in others
                       if c.get("id") != claim.get("id"))
    user = (f"The claim to decompose, claim {claim.get('id')}:\n"
            f"  quote: {claim.get('quote')}\n  statement: {claim.get('statement')}\n"
            f"  at lines {claim.get('lines')} of the claim source\n\n"
            f"The claim source around it (the claim's lines marked with >):\n{section}\n\n"
            f"The other claims already enumerated from this document:\n{listed or '  (none)'}\n"
            + (f"\nThe practice's note on this category:\n{category_note.strip()}\n" if category_note else "")
            + "\nEmit the decomposition per DECOMPOSE.md §4.")
    out = emit(types.SimpleNamespace(backend=backend), method, user, schema(), max_tokens)
    obj = out.get("obj") if isinstance(out.get("obj"), dict) else {}
    subs = [s for s in (obj.get("subclaims") or []) if isinstance(s, dict)
            and str(s.get("statement") or "").strip()][:MAX_SUBCLAIMS]
    declined = [d for d in (obj.get("declined") or []) if isinstance(d, dict)]
    return {"subclaims": subs, "declined": declined, "parse": out.get("parse"),
            "parse_error": out.get("parse_error"), "raw": out.get("raw")}


def append_subclaims(claims: List[Dict[str, Any]], parent: Dict[str, Any],
                     proposals: Sequence[Dict[str, Any]], by: str) -> List[Dict[str, Any]]:
    """New claim rows for the approved proposals: ids after the highest in
    the surface, the parent's quote and lines, `implied_by` the parent's id.
    Appended to `claims`; returned for the page to show."""
    next_id = max([int(c.get("id") or 0) for c in claims] + [0]) + 1
    new = []
    for p in proposals:
        st = str(p.get("statement") or "").strip()
        if not st:
            continue
        row = {"id": next_id, "quote": parent.get("quote"), "lines": list(parent.get("lines") or [0, 0]),
               "statement": st, "about": "target", "implied_by": int(parent.get("id")),
               "property": str(p.get("property") or "").strip(), "approved_by": by}
        claims.append(row)
        new.append(row)
        next_id += 1
    return new
