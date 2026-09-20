"""The output schemas of the four outreach calls, and the checks that run
after an answer is parsed. PROSPECT.md §12-§15 say what makes a field correct;
the field names here and there must agree.
"""
from __future__ import annotations

from typing import Any, Dict, List, Sequence, Tuple

from workflowsv2.claims_audit.schemas import quote_at

#: The options of the `category`, `relationship` and problem-recognition selects
#: of the practice's Attio list, by title, so a value is written there as it is.
#: PROSPECT.md §4 and §14 define them; a new option needs a definition there.
TYPES = ("M&A adviser", "Repeat acquirer", "Searcher", "Small PE-family office", "VC / Investor",
         "Lawyer", "Contracted-software buyer", "licensee", "Connector", "Technical feedback", "none")
RELATIONSHIPS = ("Warm", "Cold", "Referral")
RECOGNITION = ("Unknown", "Weak", "Strong", "Immediate need")
CATEGORIES = ("strong", "plausible", "weak", "reject")
ABOUT = ("yes", "no", "unsure")
QUESTIONS = 9
#: PROSPECT.md §11 asks for about 80 words; a draft over this is flagged.
MESSAGE_WORDS = 100

#: Web pages are set with typographic quotes and dashes; a model quoting them
#: writes the keyboard characters. Eight of eight citations dropped in the first
#: benchmark run (2026-09-20) were true quotes that differed only in this.
#: Presentation only: both sides are mapped the same way before `quote_at`.
_TYPOGRAPHY = str.maketrans({"\u2018": "'", "\u2019": "'", "\u201c": '"', "\u201d": '"',
                             "\u2013": "-", "\u2014": "-", "\u00a0": " ", "\u2026": "..."})

_STR = {"type": "string"}
CITATION = {"type": "object", "properties": {
    "file": _STR,
    "lines": {"type": "array", "items": {"type": "integer", "minimum": 1},
              "minItems": 2, "maxItems": 2},
    "quote": _STR},
    "required": ["file", "lines", "quote"]}
_CITATIONS = {"type": "array", "items": CITATION}


def queries_schema() -> Dict[str, Any]:
    return {"type": "object", "properties": {
        "queries": {"type": "array", "items": _STR, "maxItems": 4},
        "reason": _STR},
        "required": ["queries", "reason"]}


def page_schema() -> Dict[str, Any]:
    return {"type": "object", "properties": {
        "about_candidate": {"type": "string", "enum": list(ABOUT)},
        "reason": _STR},
        "required": ["about_candidate", "reason"]}


def scout_schema() -> Dict[str, Any]:
    return {"type": "object", "properties": {
        "queries": {"type": "array", "items": _STR, "maxItems": 3}, "reason": _STR},
        "required": ["queries", "reason"]}


def first_look_schema() -> Dict[str, Any]:
    return {"type": "object", "properties": {
        "fits": {"type": "string", "enum": ["yes", "no"]},
        "prospect_type": {"type": "string", "enum": list(TYPES)},
        "reason": _STR},
        "required": ["fits", "prospect_type", "reason"]}


def qualification_schema() -> Dict[str, Any]:
    answer = {"type": "object", "properties": {
        "question": {"type": "integer", "minimum": 1, "maximum": QUESTIONS},
        "answer": _STR, "citations": _CITATIONS},
        "required": ["question", "answer", "citations"]}
    return {"type": "object", "properties": {
        "answers": {"type": "array", "items": answer},
        "prospect_type": {"type": "string", "enum": list(TYPES)},
        "relationship": {"type": "string", "enum": list(RELATIONSHIPS)},
        "problem_recognition": {"type": "string", "enum": list(RECOGNITION)},
        "category": {"type": "string", "enum": list(CATEGORIES)},
        "why_person": _STR, "why_now": _STR,
        "why_now_citations": _CITATIONS,
        "use_case": _STR, "concerns": _STR, "reject_reason": _STR},
        "required": ["answers", "prospect_type", "relationship", "problem_recognition", "category", "why_person", "why_now",
                     "why_now_citations", "use_case", "concerns", "reject_reason"]}


def draft_schema() -> Dict[str, Any]:
    return {"type": "object", "properties": {
        "angle": _STR, "message": _STR, "rests_on": _CITATIONS, "assumes": _STR},
        "required": ["angle", "message", "rests_on", "assumes"]}


def check_citations(citations: Any, evidence: Dict[str, List[str]]
                    ) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    """(kept, dropped). A citation is kept when its quote is in the named
    evidence file. One whose quote is in the file but not at the stated lines
    is kept with `at_lines: False`: the words are the person's, and only the
    place is wrong. A dropped citation carries `why`."""
    kept: List[Dict[str, Any]] = []
    dropped: List[Dict[str, Any]] = []
    for c in citations if isinstance(citations, list) else []:
        if not isinstance(c, dict):
            continue
        name, quote = str(c.get("file") or ""), str(c.get("quote") or "")
        body = evidence.get(name)
        if body is None:
            dropped.append({**c, "why": "no evidence file of that name"})
            continue
        if not quote.strip():
            dropped.append({**c, "why": "empty quote"})
            continue
        try:
            lo, hi = int(c["lines"][0]), int(c["lines"][1])
        except (KeyError, IndexError, TypeError, ValueError):
            lo, hi = 1, len(body)
        if not 1 <= lo <= hi <= len(body):
            lo, hi, outside = 1, len(body), True
        else:
            outside = False
        status, detail = quote_at([x.translate(_TYPOGRAPHY) for x in body], lo, hi,
                                  quote.translate(_TYPOGRAPHY))
        if status == "missing":
            dropped.append({**c, "why": f"the quote is not in the file: {detail!r}"})
            continue
        if status == "prefixed":
            quote = detail or quote
        kept.append({"file": name, "lines": [lo, hi], "quote": quote,
                     "at_lines": status != "elsewhere" and not outside})
    return kept, dropped


def clean_qualification(obj: Any, evidence: Dict[str, List[str]]
                        ) -> Tuple[Dict[str, Any], List[Dict[str, Any]]]:
    """The qualification with every citation checked, and the citations
    dropped. A category or type outside the enum becomes None; the caller
    treats that as no answer."""
    obj = obj if isinstance(obj, dict) else {}
    dropped: List[Dict[str, Any]] = []
    answers: Dict[int, Dict[str, Any]] = {}
    for a in obj.get("answers") or []:
        if not isinstance(a, dict):
            continue
        try:
            n = int(a.get("question"))
        except (TypeError, ValueError):
            continue
        text = str(a.get("answer") or "").strip()
        if not 1 <= n <= QUESTIONS or n in answers or not text:
            continue
        kept, bad = check_citations(a.get("citations"), evidence)
        dropped += [{**b, "in": f"answer {n}"} for b in bad]
        answers[n] = {"question": n, "answer": text, "citations": kept}
    kept_now, bad_now = check_citations(obj.get("why_now_citations"), evidence)
    dropped += [{**b, "in": "why_now"} for b in bad_now]

    def s(key: str) -> str:
        return str(obj.get(key) or "").strip()

    return {"answers": [answers[n] for n in sorted(answers)],
            "unanswered": [n for n in range(1, QUESTIONS + 1) if n not in answers],
            "prospect_type": obj.get("prospect_type") if obj.get("prospect_type") in TYPES else None,
            "relationship": obj.get("relationship") if obj.get("relationship") in RELATIONSHIPS else None,
            "problem_recognition": (obj.get("problem_recognition")
                                    if obj.get("problem_recognition") in RECOGNITION else None),
            "category": obj.get("category") if obj.get("category") in CATEGORIES else None,
            "why_person": s("why_person"), "why_now": s("why_now"),
            "why_now_citations": kept_now,
            "use_case": s("use_case"), "concerns": s("concerns"),
            "reject_reason": s("reject_reason")}, dropped


def clean_draft(obj: Any, evidence: Dict[str, List[str]]
                ) -> Tuple[Dict[str, Any], List[Dict[str, Any]], List[str]]:
    """(draft, citations dropped, flags). Flags are for the person who edits
    the message; none of them stops the draft from being shown."""
    obj = obj if isinstance(obj, dict) else {}
    kept, bad = check_citations(obj.get("rests_on"), evidence)
    message = str(obj.get("message") or "").strip()
    flags: List[str] = []
    if not kept:
        flags.append("the message's opening rests on no checked citation")
    words = len(message.split())
    if words > MESSAGE_WORDS:
        flags.append(f"the message is {words} words")
    return {"angle": str(obj.get("angle") or "").strip(), "message": message,
            "rests_on": kept, "assumes": str(obj.get("assumes") or "").strip()}, bad, flags


def numbered(lines: Sequence[str], max_words: int) -> Tuple[str, bool]:
    """The text as the model sees it, `  12|text`, cut after `max_words`.
    Returns the text and whether it was cut."""
    shown: List[str] = []
    words = 0
    for n, line in enumerate(lines, 1):
        shown.append(f"{n:>4}|{line}")
        words += len(line.split())
        if words >= max_words and n < len(lines):
            total = sum(len(x.split()) for x in lines)
            shown.append(f"[the file continues: {total} words in all, {max_words} shown]")
            return "\n".join(shown), True
    return "\n".join(shown), False
