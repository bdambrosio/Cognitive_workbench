"""The report stage's output schema, and the checks that run after it is
parsed. REPORT.md §7 states the contract; `lint_workflow.check_report_fields`
keeps the two in step.

The agent's whole output is seven passages of prose. Nothing it writes is
checkable against a cited line, which is why the checks here are about the
passages' relation to the record — a claim id that does not exist, a marker
that belongs to the harness — and never about their content.
"""
from __future__ import annotations

import re
from typing import Any, Dict, List, Tuple

#: REPORT.md §7, in document order.
FIELDS: Tuple[str, ...] = ("summary", "conclusion", "scope_note", "shown_note",
                           "unsettled_note", "not_examined_note", "limitations")

#: The fields that are written only when the document calls for them
#: (REPORT.md §6): empty otherwise, and never empty when it does.
CONDITIONAL: Tuple[str, ...] = ("conclusion", "not_examined_note")

_CLAIM_REF = re.compile(r"\bclaim\s+#?(\d+)\b", re.I)


def prose_schema() -> Dict[str, Any]:
    return {"type": "object",
            "properties": {f: {"type": "string"} for f in FIELDS},
            "required": list(FIELDS)}


def check_prose(obj: Dict[str, Any], merged: Dict[str, Any],
                has_not_examined: bool,
                wants_conclusion: bool = False) -> Dict[str, Any]:
    """What REPORT.md requires that the schema cannot express."""
    problems: List[str] = []
    ids = {f.get("claim_id") for f in merged.get("findings") or []}
    wanted = {"conclusion": wants_conclusion,
              "not_examined_note": has_not_examined}
    why = {"conclusion": "the document does not ask for a conclusion",
           "not_examined_note": "the document has no such claims"}
    for f in FIELDS:
        text = obj.get(f)
        if not isinstance(text, str):
            problems.append(f"{f}: missing")
            continue
        if not text.strip():
            if f in CONDITIONAL and not wanted[f]:
                continue
            problems.append(f"{f}: empty (REPORT.md §6)")
            continue
        if f in CONDITIONAL and not wanted[f]:
            problems.append(f"{f}: written, and {why[f]} (REPORT.md §6)")
        if re.search(r"(?m)^\s*===", text):
            problems.append(f"{f}: carries a `===` marker line")
        for m in _CLAIM_REF.finditer(text):
            if int(m.group(1)) not in ids:
                problems.append(f"{f}: names claim {m.group(1)}, which no "
                                f"finding carries")
    return {"ok": not problems, "problems": problems,
            "figures": {f: len((obj.get(f) or "").split()) for f in FIELDS}}
