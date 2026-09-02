"""The intake form schema and its check. INTAKE.md §2 and §4 state the
contract; `lint_workflow.check_intake_fields` keeps the two in step.

THE FORM IS RE-EMITTED WHOLE AFTER EVERY TURN by a schema-constrained call
the runner makes, and the runner writes the file. The agent has no tool that
writes or edits a note, and none is added: this is the pattern every other
stage uses, and a whole re-emission is also how an earlier slot gets revised
when the buyer corrects themselves.
"""
from __future__ import annotations

from typing import Any, Dict, List, Tuple

#: INTAKE.md §2, in order; the next question is for the emptiest slot.
SLOTS: Dict[str, Tuple[str, ...]] = {
    "identify": ("client", "acting", "counterparty", "confidentiality"),
    "situation": ("subject", "price", "structure", "timetable"),
    "background": ("target", "claim_sources", "known"),
    "assessment": ("paying_for", "price_movers", "walk_away"),
    "recommendation": ("scope", "deliverable", "seller_questions"),
}


def empty_form() -> Dict[str, Any]:
    out: Dict[str, Any] = {slot: {f: "" for f in fields}
                           for slot, fields in SLOTS.items()}
    out["open_questions"] = []
    out["notes"] = []
    return out


def intake_schema() -> Dict[str, Any]:
    """Every field a string, empty allowed; every slot and field required so
    the decoder produces the whole form each time."""
    props: Dict[str, Any] = {}
    for slot, fields in SLOTS.items():
        props[slot] = {"type": "object",
                       "properties": {f: {"type": "string"} for f in fields},
                       "required": list(fields)}
    props["open_questions"] = {"type": "array", "items": {"type": "string"}}
    props["notes"] = {"type": "array", "items": {"type": "string"}}
    return {"type": "object", "properties": props,
            "required": list(SLOTS) + ["open_questions", "notes"]}


def check_intake(obj: Dict[str, Any]) -> Dict[str, Any]:
    """Which fields are filled, which slots are complete, and the emptiest
    slot — the one the next question is for."""
    empty: Dict[str, List[str]] = {}
    filled = 0
    total = 0
    for slot, fields in SLOTS.items():
        got = obj.get(slot) or {}
        missing = [f for f in fields if not str(got.get(f) or "").strip()]
        total += len(fields)
        filled += len(fields) - len(missing)
        if missing:
            empty[slot] = missing
    complete = [s for s in SLOTS if s not in empty]
    emptiest = None
    if empty:
        emptiest = max(empty, key=lambda s: (len(empty[s]) / len(SLOTS[s]),
                                              -list(SLOTS).index(s)))
    return {"complete": complete, "empty": empty, "emptiest": emptiest,
            "filled": filled, "total": total,
            "open_questions": len(obj.get("open_questions") or []),
            "notes": len(obj.get("notes") or [])}


def ledger(check: Dict[str, Any]) -> str:
    """The one line appended to each of the buyer's turns."""
    if not check["empty"]:
        return "[form: every slot is filled]"
    parts = [f"{s} ({', '.join(fs)})" for s, fs in check["empty"].items()]
    return (f"[form: {check['filled']} of {check['total']} fields filled; "
            f"still empty — " + "; ".join(parts) + "]")


def engagement_blocks(obj: Dict[str, Any]) -> Dict[str, str]:
    """The `transaction:` and `thresholds:` text the engagement file gets when
    the practice finishes the intake. Prose assembled from the fields the
    buyer filled, nothing added."""
    s, a, i = obj.get("situation") or {}, obj.get("assessment") or {}, obj.get("identify") or {}

    def line(label: str, val: Any) -> str:
        v = str(val or "").strip()
        return f"{label}: {v}" if v else ""

    transaction = "\n".join(x for x in (
        line("Client", i.get("client")),
        line("Acting for the client", i.get("acting")),
        line("Counterparty", i.get("counterparty")),
        line("Subject", s.get("subject")),
        line("Price and basis", s.get("price")),
        line("Structure", s.get("structure")),
        line("Timetable", s.get("timetable"))) if x)
    thresholds = "\n".join(x for x in (
        line("Paying for", a.get("paying_for")),
        line("Would change the price or terms", a.get("price_movers")),
        line("Would end the deal", a.get("walk_away"))) if x)
    return {"transaction": transaction, "thresholds": thresholds}
