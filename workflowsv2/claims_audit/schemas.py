"""The audit output schema, and the checks that run after it is parsed.

METHOD §13 states the contract; this file is the machine half of it. The two
must agree, and `lint_workflow.check_schema_vocab` is what makes them.

WHAT THE SCHEMA ENFORCES AND WHAT PYTHON ENFORCES. The schema carries shape:
which fields exist, which are required, and the two closed vocabularies
(`verdict`, `unresolved_because`). Everything that needs to look at the target
— does this document exist, is this line range real, does this quote appear
where it says — is checked here, after parsing, against the corpus.

That division is deliberate and it is the same one `citations.py` was written
for: "an LLM asked whether a marker appears is slower, dearer and less reliable
than a substring test." Two consequences, both of which look like omissions
until you know the reason:

  - **`document` is a plain string, not an enum of the corpus filenames.** An
    enum would make a nonexistent document unrepresentable at decode time,
    which is genuinely stronger — for nine documents. A real target is 1,141
    files and the enum becomes the schema. `check_output` catches the same
    defect with a better message, at every target size.

  - **Evidence is one flat object with a `form` discriminator, not a `oneOf`
    union.** The union is the faithful model of METHOD §7 and it decodes
    correctly on the local route — probed 2026-09-01, four runs, schema-valid
    every time. It was still rejected. `oneOf` with per-branch
    `additionalProperties: false` is the construct most likely to vary between
    constrained-decoding backends, and requiring it would narrow the candidate
    pool to catch a defect `check_output` already catches. Widening that pool
    is what this refactor is for.

TRUNCATION IS THE FAILURE MODE THAT MATTERS. Under constrained decoding a
generation cut at `max_tokens` yields invalid JSON rather than a short report,
so the loss is total instead of partial. `salvage_findings` recovers the
complete elements of a cut findings array, after `claims._salvage_truncated_claims`,
which is sound for the same reason: array elements here are independent.
"""
from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

# --------------------------------------------------------------------------
# The closed vocabularies. METHOD §6 and §8 are the source; the linter checks
# that these lists and those tables say the same thing.
# --------------------------------------------------------------------------
VERDICTS: Tuple[str, ...] = (
    "real", "real_with_caveat", "partial", "contradicted", "unverifiable")

#: Verdicts that require `adjudication.gap` (METHOD §13). `real` has nothing
#: beside it; `unverifiable` uses `unresolved_because` instead.
GAP_REQUIRED: Tuple[str, ...] = ("real_with_caveat", "partial", "contradicted")

UNRESOLVED_BECAUSE: Tuple[str, ...] = (
    "not_in_the_materials", "present_but_not_readable", "outside_the_materials")

EVIDENCE_FORMS: Tuple[str, ...] = ("citation", "derived", "search")

SEARCH_KINDS: Tuple[str, ...] = ("lexical", "structural")

#: Fields each evidence form requires, per METHOD §7. The schema requires only
#: `form`; this table is enforced after parsing.
FORM_FIELDS: Dict[str, Tuple[str, ...]] = {
    "citation": ("document", "lines", "quote", "shows"),
    "derived":  ("basis", "derivation", "consequence"),
    "search":   ("kind", "performed", "result"),
}

_LINES = {"type": "array", "items": {"type": "integer"},
          "minItems": 2, "maxItems": 2}


def surface_schema() -> Dict[str, Any]:
    """Phase one: the claim surface, per METHOD §5 and §13.

    Emitted before any evidence is gathered and frozen when it lands. The
    freeze is a control, not a convenience — enumerating while adjudicating
    means choosing what counts as a claim with the verdicts already in view.
    """
    claim = {"type": "object", "properties": {
        "id": {"type": "integer", "minimum": 1},
        "quote": {"type": "string"}, "lines": _LINES,
        "statement": {"type": "string"}},
        "required": ["id", "quote", "lines", "statement"]}
    return {"type": "object", "properties": {
        "claim_source": {"type": "string"},
        "claims": {"type": "array", "items": claim},
        "not_completed": {"type": "string"}},
        "required": ["claim_source", "claims"]}


def check_surface(obj: Dict[str, Any], corpus: Path,
                  claim_source: str) -> Dict[str, Any]:
    """The frozen surface, against the document it was read from."""
    problems: List[str] = []
    body: List[str] = []
    src = corpus / claim_source
    if src.is_file():
        body = src.read_text(encoding="utf-8", errors="replace").splitlines()
    else:
        problems.append(f"claim source {claim_source!r} is not in the materials")

    claims = obj.get("claims")
    if not isinstance(claims, list):
        return {"ok": False, "problems": ["`claims` is not a list"],
                "figures": {}}
    incomplete = obj.get("not_completed")
    if incomplete and claims:
        problems.append("`not_completed` is set and claims were emitted; "
                        "METHOD §4 requires an empty list")
    if not incomplete and not claims:
        problems.append("no claims and no `not_completed` — METHOD §4 requires "
                        "one or the other")

    seen_id, seen_q = {}, {}
    for i, c in enumerate(claims, 1):
        w = f"claim {c.get('id', f'#{i}')}"
        cid = c.get("id")
        if cid in seen_id:
            problems.append(f"{w}: id {cid} is used twice (also at position "
                            f"{seen_id[cid]}) — findings refer to claims by id")
        else:
            seen_id[cid] = i
        q = _norm(c.get("quote"))
        if not q:
            problems.append(f"{w}: no quote — METHOD §5 makes the verbatim "
                            f"quote the claim's identity")
        elif q in seen_q:
            problems.append(f"{w}: same quote as claim {seen_q[q]}")
        else:
            seen_q[q] = cid
        lines = c.get("lines")
        if body and isinstance(lines, list) and len(lines) == 2 \
                and all(isinstance(n, int) for n in lines):
            lo, hi = lines
            if lo < 1 or hi < lo or hi > len(body):
                problems.append(f"{w}: lines {lo}-{hi} against a "
                                f"{len(body)}-line claim source")
            elif q and q not in _norm("\n".join(body[lo - 1:hi])):
                where = ("elsewhere in the document"
                         if q in _norm("\n".join(body)) else "nowhere in it")
                problems.append(f"{w}: quote is not at lines {lo}-{hi}; it is "
                                f"{where}")
        else:
            problems.append(f"{w}: lines {lines!r} is not a start and end")

    return {"ok": not problems, "problems": problems,
            "figures": {"claims": len(claims),
                        "not_completed": bool(incomplete)}}


def audit_schema() -> Dict[str, Any]:
    """The response schema for the emission call, per METHOD §13.

    Takes no arguments on purpose. Nothing target-specific belongs in a
    schema that a decoder has to compile — see the module docstring.
    """
    basis = {"type": "object", "properties": {
        "document": {"type": "string"}, "lines": _LINES,
        "quote": {"type": "string"}},
        "required": ["document", "lines", "quote"]}
    evidence = {"type": "object", "properties": {
        "form": {"enum": list(EVIDENCE_FORMS)},
        # citation
        "document": {"type": "string"}, "lines": _LINES,
        "quote": {"type": "string"}, "shows": {"type": "string"},
        # derived
        "basis": {"type": "array", "items": basis},
        "derivation": {"type": "string"}, "consequence": {"type": "string"},
        # search
        "kind": {"enum": list(SEARCH_KINDS)},
        "performed": {"type": "string"}, "result": {"type": "string"}},
        "required": ["form"]}
    adjudication = {"type": "object", "properties": {
        "verdict": {"enum": list(VERDICTS)},
        "gap": {"type": "string"},
        "unresolved_because": {"enum": list(UNRESOLVED_BECAUSE)}},
        "required": ["verdict"]}
    finding = {"type": "object", "properties": {
        "claim_id": {"type": "integer", "minimum": 1},
        "adjudication": adjudication,
        "evidence": {"type": "array", "items": evidence, "minItems": 1},
        "correction": {"type": "string"}},
        "required": ["claim_id", "adjudication", "evidence"]}
    return {"type": "object", "properties": {
        "claim_source": {"type": "string"},
        "findings": {"type": "array", "items": finding},
        "unclaimed": {"type": "array", "items": {"type": "object", "properties": {
            "note": {"type": "string"}, "evidence": evidence},
            "required": ["note", "evidence"]}},
        "questions": {"type": "array", "items": {"type": "string"}},
        "not_completed": {"type": "string"}},
        "required": ["claim_source", "findings"]}


# --------------------------------------------------------------------------
# Parsing
# --------------------------------------------------------------------------

def salvage_findings(text: str) -> Optional[Dict[str, Any]]:
    """Recover the complete findings of a truncated emission, or None.

    Schema-aware, and local to this module for the same reason
    `claims._salvage_truncated_claims` is local to its own: dropping the
    trailing partial element is only sound because the elements are
    independent, which is true of findings and is not true in general.

    Returns the object with whatever findings parsed whole. The caller must
    record that salvage fired — a salvaged run is not a clean one.
    """
    start = text.find("{")
    if start < 0:
        return None
    src = text[start:]
    m = re.search(r'"findings"\s*:\s*\[', src)
    if not m:
        return None
    head, out, depth, in_str, esc, elem = src[:m.end()], [], 0, False, False, ""
    for ch in src[m.end():]:
        if esc:
            elem += ch; esc = False; continue
        if ch == "\\" and in_str:
            elem += ch; esc = True; continue
        if ch == '"':
            in_str = not in_str
        if not in_str:
            if ch == "{":
                depth += 1
            elif ch == "}":
                depth -= 1
                if depth == 0:
                    elem += ch
                    try:
                        out.append(json.loads(elem))
                    except Exception:      # a cut element; stop, keep the rest
                        break
                    elem = ""
                    continue
            elif ch == "]" and depth == 0:
                break
            elif depth == 0 and ch in ", \n\r\t":
                continue
        elem += ch
    if not out:
        return None
    src_head = json.loads(head[:head.rfind('"findings"')].rstrip().rstrip(",") + "}")
    src_head["findings"] = out
    return src_head


# --------------------------------------------------------------------------
# The checks, after parsing, against the target
# --------------------------------------------------------------------------

#: Markdown decoration a quote routinely loses on the way out of a document.
#: NOT a content filter — every one of these is presentation, and a quote that
#: differs only by them is the same text.
_DECORATION = re.compile(r"[*_`]+|^\s*[-*+]\s+|^\s*\d+\.\s+", re.M)


def _norm(s: str) -> str:
    """Compare quotes the way a reader would: ignoring whitespace and markdown.

    WHITESPACE, because a quote copied out of a document routinely differs
    from it by a line wrap and by nothing else.

    MARKDOWN, because this check's first run reported seven of twelve findings
    on doc9 as quoting text that "does not appear in that document at all",
    and every one of them was a faithful quote of `*   **Dyno:** ...` written
    down as `Dyno: ...`. The model dropped the decoration, which is what a
    person quoting that line would also do. `repair_json_string` strips the
    same characters out of structured output for the same reason.

    The risk is the other way — normalising until a wrong quote matches — so
    this removes presentation only: emphasis, code ticks, and list markers.
    Nothing that carries meaning is touched.

    BOTH SIDES MUST BE JOINED THE SAME WAY BEFORE THIS RUNS. The list-marker
    patterns are line-anchored, so a quote that kept its newlines gets them
    stripped while a source span joined with spaces does not — which reported
    a correct citation as quoting text found "nowhere in it". Callers join
    source lines with a newline.
    """
    return re.sub(r"\s+", " ", _DECORATION.sub(" ", s or "")).strip()


def check_output(obj: Dict[str, Any], corpus: Path, claim_source: str,
                 frozen: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    """Everything METHOD requires that the schema cannot express.

    Returns `{"ok": bool, "problems": [...], "figures": {...}}`, the shape the
    runner already records for its other checks. A problem is a sentence a
    person can act on, not a code.
    """
    problems: List[str] = []
    docs: Dict[str, List[str]] = {}
    if corpus.is_dir():
        for f in sorted(corpus.rglob("*")):
            if f.is_file():
                try:
                    docs[f.name] = f.read_text(
                        encoding="utf-8", errors="replace").splitlines()
                except OSError as e:
                    problems.append(f"could not read {f.name}: {e}")

    def _cite(where: str, doc: Any, lines: Any, quote: Any) -> None:
        """One citation, against the corpus. METHOD §7."""
        if not isinstance(doc, str) or doc not in docs:
            problems.append(f"{where}: document {doc!r} is not in the materials")
            return
        body = docs[doc]
        if (not isinstance(lines, list) or len(lines) != 2
                or not all(isinstance(n, int) for n in lines)):
            problems.append(f"{where}: lines {lines!r} is not a start and end")
            return
        lo, hi = lines
        if lo < 1 or hi < lo:
            problems.append(f"{where}: lines {lo}-{hi} is not a range")
            return
        if hi > len(body):
            problems.append(f"{where}: cites {doc}:{lo}-{hi}, which has "
                            f"{len(body)} lines — the integers are not line "
                            f"numbers, or the range is wrong")
            return
        if isinstance(quote, str) and _norm(quote):
            span = _norm("\n".join(body[lo - 1:hi]))
            if _norm(quote) not in span:
                whole = _norm("\n".join(body))
                problems.append(
                    f"{where}: quote is not at {doc}:{lo}-{hi}"
                    + ("; it is elsewhere in that document"
                       if _norm(quote) in whole else
                       "; it does not appear in that document at all"))

    findings = obj.get("findings")
    if not isinstance(findings, list):
        return {"ok": False, "problems": ["`findings` is not a list"],
                "figures": {}}

    incomplete = obj.get("not_completed")
    if incomplete and findings:
        problems.append("`not_completed` is set and findings were emitted; "
                        "METHOD §4 requires an empty findings list")
    if not incomplete and not findings:
        problems.append("no findings and no `not_completed` — METHOD §4 "
                        "requires one or the other")

    # THE FROZEN SURFACE IS THE DENOMINATOR, and this is the check that
    # freezing buys. v1 could not make it: enumeration and adjudication
    # happened in one pass, so a claim quietly dropped left no trace of ever
    # having been enumerated.
    ids = {c.get("id") for c in frozen}
    seen: Dict[Any, int] = {}
    verdicts: Dict[str, int] = {}
    forms: Dict[str, int] = {}
    for i, f in enumerate(findings, 1):
        w = f"finding {i}"
        adj = f.get("adjudication") or {}
        ev = f.get("evidence") or []

        cid = f.get("claim_id")
        if cid not in ids:
            problems.append(f"{w}: claim_id {cid!r} is not in the frozen "
                            f"surface")
        elif cid in seen:
            problems.append(f"{w}: claim {cid} was already adjudicated by "
                            f"finding {seen[cid]} — METHOD §4 gives each claim "
                            f"exactly one finding")
        else:
            seen[cid] = i

        v = adj.get("verdict")
        verdicts[v] = verdicts.get(v, 0) + 1
        if v not in VERDICTS:
            problems.append(f"{w}: verdict {v!r} is not one of METHOD §6's")
        if v in GAP_REQUIRED and not _norm(adj.get("gap")):
            problems.append(f"{w}: verdict {v!r} requires `gap` (METHOD §13)")
        if v == "real" and _norm(adj.get("gap")):
            problems.append(f"{w}: verdict `real` carries a `gap`")
        if v == "unverifiable":
            if adj.get("unresolved_because") not in UNRESOLVED_BECAUSE:
                problems.append(f"{w}: `unverifiable` requires "
                                f"`unresolved_because` (METHOD §8)")
            kinds = {e.get("kind") for e in ev
                     if isinstance(e, dict) and e.get("form") == "search"}
            missing = [k for k in SEARCH_KINDS if k not in kinds]
            if missing:
                problems.append(f"{w}: `unverifiable` needs a lexical and a "
                                f"structural search (METHOD §8); missing "
                                f"{', '.join(missing)}")

        if not ev:
            problems.append(f"{w}: no evidence — METHOD §4 gives every finding "
                            f"the evidence its adjudication rests on")
        for j, e in enumerate(ev, 1):
            ew = f"{w} evidence {j}"
            if not isinstance(e, dict):
                problems.append(f"{ew}: not an object"); continue
            form = e.get("form")
            forms[form] = forms.get(form, 0) + 1
            if form not in FORM_FIELDS:
                problems.append(f"{ew}: form {form!r} is not one of "
                                f"{', '.join(EVIDENCE_FORMS)}")
                continue
            for field in FORM_FIELDS[form]:
                if e.get(field) in (None, "", []):
                    problems.append(f"{ew}: form {form!r} requires {field!r} "
                                    f"(METHOD §7)")
            if form == "citation":
                _cite(ew, e.get("document"), e.get("lines"), e.get("quote"))
            elif form == "derived":
                basis = e.get("basis") or []
                if len(basis) < 2:
                    problems.append(f"{ew}: a derived fact follows from two or "
                                    f"more facts (METHOD §7); {len(basis)} given")
                for k, b in enumerate(basis, 1):
                    if isinstance(b, dict):
                        _cite(f"{ew} basis {k}", b.get("document"),
                              b.get("lines"), b.get("quote"))

    for cid in sorted(ids - set(seen), key=lambda x: (x is None, x)):
        problems.append(f"claim {cid} in the frozen surface has no finding — "
                        f"METHOD §4 requires one for every claim")

    for i, u in enumerate(obj.get("unclaimed") or [], 1):
        e = (u or {}).get("evidence") or {}
        if e.get("form") == "citation":
            _cite(f"unclaimed {i}", e.get("document"), e.get("lines"),
                  e.get("quote"))

    return {"ok": not problems, "problems": problems,
            "figures": {"findings": len(findings),
                        "frozen_claims": len(ids),
                        "adjudicated": len(seen),
                        "verdicts": verdicts,
                        "evidence_forms": forms,
                        "unclaimed": len(obj.get("unclaimed") or []),
                        "questions": len(obj.get("questions") or []),
                        "not_completed": bool(incomplete)}}
