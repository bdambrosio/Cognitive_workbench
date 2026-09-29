"""The case form, its check, and the ledger appended to each of the
practitioner's turns. CONSULT.md §6 states the form's contract;
`lint_workflow.check_consult_fields` keeps the two in step.

THE FORM IS RE-EMITTED WHOLE AFTER EVERY TURN by a schema-constrained call
the session makes, as in the intake. The model writes what the practitioner
said; it does not write feature ids. Those are attached by the session
(session.map_findings), and carried across re-emissions by the finding's
words.
"""
from __future__ import annotations

from typing import Any, Dict, List, Tuple

#: CONSULT.md §6, in order.
SLOTS: Dict[str, Tuple[str, ...]] = {
    "client": ("age", "sex", "prakriti", "vikriti"),
    "presenting": ("complaint", "onset", "duration"),
}
LISTS = ("findings", "exposures", "open_questions", "notes")
STATUS = ("present", "absent", "unclear")
SOURCES = ("reported", "examined", "wearable", "lab")
DOSHAS = ("vata", "pitta", "kapha")

_STR = {"type": "string"}


def empty_form() -> Dict[str, Any]:
    out: Dict[str, Any] = {slot: {f: "" for f in fields} for slot, fields in SLOTS.items()}
    out["client"]["vikriti_doshas"] = []
    out["exam"] = ""
    for k in LISTS:
        out[k] = []
    return out


def case_schema() -> Dict[str, Any]:
    props: Dict[str, Any] = {}
    for slot, fields in SLOTS.items():
        p = {f: _STR for f in fields}
        req = list(fields)
        if slot == "client":
            p["vikriti_doshas"] = {"type": "array", "items": {"type": "string", "enum": list(DOSHAS)}}
            req.append("vikriti_doshas")
        props[slot] = {"type": "object", "properties": p, "required": req}
    props["findings"] = {"type": "array", "items": {"type": "object", "properties": {
        "text": _STR, "status": {"type": "string", "enum": list(STATUS)},
        "source": {"type": "string", "enum": list(SOURCES)}, "when": _STR},
        "required": ["text", "status", "source", "when"]}}
    props["exam"] = _STR
    for k in ("exposures", "open_questions", "notes"):
        props[k] = {"type": "array", "items": _STR}
    return {"type": "object", "properties": props,
            "required": list(SLOTS) + ["findings", "exam", "exposures", "open_questions", "notes"]}


def mapping_schema() -> Dict[str, Any]:
    """The answer to 'which feature is each finding': one entry per finding,
    `feature_id` one of that finding's candidate ids or empty for none. The
    reason comes before the choice. With the fields named `text` and
    `feature`, a live run filled `feature` with the finding's own words."""
    return {"type": "object", "properties": {"mappings": {"type": "array", "items": {
        "type": "object", "properties": {"finding": _STR, "why": _STR, "feature_id": _STR},
        "required": ["finding", "why", "feature_id"]}}}, "required": ["mappings"]}


def check_case(form: Dict[str, Any]) -> Dict[str, Any]:
    empty: Dict[str, List[str]] = {}
    for slot, fields in SLOTS.items():
        got = form.get(slot) or {}
        missing = [f for f in fields if not str(got.get(f) or "").strip()]
        if missing:
            empty[slot] = missing
    findings = form.get("findings") or []
    unmatched = [f["text"] for f in findings if not f.get("feature_id")]
    return {"empty": empty, "findings": len(findings),
            "matched": len(findings) - len(unmatched), "unmatched": unmatched}


def ledger(check: Dict[str, Any], ranked: List[Dict[str, Any]],
           questions: List[Dict[str, Any]], kb) -> str:
    """The block appended to each of the practitioner's turns: the state of
    the form, the differential as computed, the next questions, and any red
    flag. CONSULT.md §3 tells the agent how to use it."""
    lines = []
    empty = "; ".join(f"{s} ({', '.join(fs)})" for s, fs in check["empty"].items())
    lines.append(f"[case: {check['findings']} finding(s), {check['matched']} matched to the "
                 f"knowledge base" + (f"; not matched: {'; '.join(check['unmatched'])}"
                                      if check["unmatched"] else "")
                 + (f"; still empty: {empty}" if empty else "") + "]")
    if not ranked:
        lines.append("[differential: no disease in the knowledge base matches the findings so far]")
    else:
        rows = []
        for i, r in enumerate(ranked[:5], 1):
            sup = ", ".join(f"{_name(kb, s['feature'])} ({'/'.join(s['verses'])})"
                            for s in r["supporting"])
            ag = ", ".join(_name(kb, s["feature"]) for s in r["against"])
            rows.append(f"{i}. {r['id']} — {r['label']}; score "
                        f"{r['score']}; found: {sup or 'none'}" + (f"; absent: {ag}" if ag else ""))
        lines.append("[differential, computed:\n" + "\n".join(rows) + "]")
    if questions:
        qs = []
        for q in questions:
            f = kb.feature(q["feature"]) or {}
            if q["not_held_by"]:
                bears = (f"if present, favours {', '.join(q['held_by'])} over "
                         f"{', '.join(q['not_held_by'])}")
            else:
                bears = ("shared by every leading candidate: confirms the disease, "
                         "does not tell its forms apart")
            qs.append(f"- {f.get('lay_question') or f.get('en') or q['feature']} "
                      f"({q['feature']}; {bears}; {'/'.join(q['verses'])})")
        lines.append("[ask next:\n" + "\n".join(qs) + "]")
    red = [(r["id"], x) for r in ranked for x in r["red_flags"]]
    if red:
        lines.append("[RED FLAG — a sign the text gives as fatal or incurable was found: "
                     + "; ".join(f"{_name(kb, x['feature'])} for {rid} ({'/'.join(x['verses'])})"
                                 for rid, x in red) + "]")
    return "\n".join(lines)


def _name(kb, fid: str) -> str:
    f = kb.feature(fid) or {}
    return f.get("en") or fid
