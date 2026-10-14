"""The report stage's output schema, and the checks that run after it is
parsed. REPORT.md §7 states the contract; `lint_workflow.check_report_fields`
keeps the two in step.

The agent's output is seven passages of prose and two arrangements: the
subjects the findings are grouped under, and which questions for the seller
repeat one another. Nothing it writes is checkable against a cited line,
which is why the checks here are about the output's relation to the record —
a claim id that does not exist, a marker that belongs to the harness, a
finding left out of the groups — and never about its content.
"""
from __future__ import annotations

import re
from typing import Any, Dict, List, Optional, Set, Tuple

#: REPORT.md §7, in document order.
FIELDS: Tuple[str, ...] = ("summary", "conclusion", "scope_note", "shown_note",
                           "unsettled_note", "not_examined_note", "limitations")

#: REPORT.md §7: the arrangements, which are not prose.
ARRANGEMENTS: Tuple[str, ...] = ("groups", "duplicate_questions")

#: The sections whose findings may be grouped by subject (REPORT.md §6).
GROUPED_SECTIONS: Tuple[str, ...] = ("shown", "buyer", "practice")

#: The fields that are written only when the document calls for them
#: (REPORT.md §6): empty otherwise, and never empty when it does.
CONDITIONAL: Tuple[str, ...] = ("conclusion", "not_examined_note")

_CLAIM_REF = re.compile(r"\bclaim\s+#?(\d+)\b", re.I)


def prose_schema() -> Dict[str, Any]:
    props: Dict[str, Any] = {f: {"type": "string"} for f in FIELDS}
    props["groups"] = {"type": "array", "items": {"type": "object", "properties": {
        "section": {"enum": list(GROUPED_SECTIONS)},
        "title": {"type": "string"},
        "findings": {"type": "array", "items": {"type": "string"}}},
        "required": ["section", "title", "findings"]}}
    props["duplicate_questions"] = {"type": "array", "items": {
        "type": "array", "items": {"type": "integer", "minimum": 1}}}
    return {"type": "object", "properties": props,
            "required": list(FIELDS) + list(ARRANGEMENTS)}


def valid_groups(obj: Optional[Dict[str, Any]], sections: Dict[str, Set[str]],
                 optional: Set[str] = frozenset()
                 ) -> Tuple[Dict[str, List[Dict[str, Any]]], List[str]]:
    """The groups of each section that the record bears out, and the
    problems. `sections` holds the finding keys (`<claim source>#<claim
    id>`) of each grouped section. A section's groups are used only when
    they hold every one of its findings exactly once; otherwise the section
    is shown ungrouped and the problem is reported. A key in `optional` (a
    finding the practice withdrew) may be in one group or in none."""
    by_section: Dict[str, List[Dict[str, Any]]] = {}
    for g in (obj or {}).get("groups") or []:
        if isinstance(g, dict) and g.get("section") in GROUPED_SECTIONS:
            by_section.setdefault(g["section"], []).append(g)
    good: Dict[str, List[Dict[str, Any]]] = {}
    problems: List[str] = []
    for sec, groups in by_section.items():
        keys = sections.get(sec) or set()
        bad: List[str] = []
        seen: Set[str] = set()
        for g in groups:
            if not str(g.get("title") or "").strip():
                bad.append("a group has no title")
            for k in g.get("findings") or []:
                if k not in keys:
                    bad.append(f"{k!r} is not a finding of this section")
                elif k in seen:
                    bad.append(f"{k} is in two groups")
                seen.add(k)
        bad += [f"{k} is in no group" for k in sorted(keys - seen - set(optional))]
        if bad:
            problems += [f"groups ({sec}): {b}; the section is shown ungrouped" for b in bad]
        else:
            good[sec] = groups
    return good, problems


def dropped_questions(obj: Optional[Dict[str, Any]], n: int) -> Tuple[Set[int], List[str]]:
    """The numbers (1..n) of the questions for the seller that repeat an
    earlier one: every member of a `duplicate_questions` entry but its first.
    Nothing is dropped when an entry names a number twice or one that does
    not exist."""
    entries = (obj or {}).get("duplicate_questions") or []
    problems: List[str] = []
    seen: Set[int] = set()
    for e in entries:
        for q in e or []:
            if not isinstance(q, int) or not 1 <= q <= n:
                problems.append(f"duplicate_questions: there is no question {q!r}")
            elif q in seen:
                problems.append(f"duplicate_questions: question {q} is in two entries")
            seen.add(q)
    if problems:
        return set(), [p + "; no question was dropped" for p in problems]
    return {q for e in entries for q in sorted(e)[1:]}, []


def check_prose(obj: Dict[str, Any], merged: Dict[str, Any],
                has_not_examined: bool,
                wants_conclusion: bool = False,
                sections: Optional[Dict[str, Set[str]]] = None,
                optional: Set[str] = frozenset()) -> Dict[str, Any]:
    """What REPORT.md requires that the schema cannot express. `sections`
    and `optional`: as for `valid_groups`, for the groups' check."""
    problems: List[str] = []
    problems += valid_groups(obj, sections or {}, optional)[1]
    problems += dropped_questions(obj, len(merged.get("questions") or []))[1]
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
