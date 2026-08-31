#!/usr/bin/env python3
"""Resolve a report's citations against the materials it cites.

EXTRACTED FROM `workflows/audit_review/runner.py` 2026-08-30, unchanged, when
audit_postprocess became the second caller. It was written for the reviewer —
"[broken citation] is one of the two verdicts that fail a report, so it should
be decided by a file operation rather than by judgement that was measured
flipping on identical text" — and the delivery stage needs the same fact for a
different reason: on 2026-08-30 a ChatterMate report shipped three citations
naming lines past the end of their files, against a METHOD §12 step 6b that
says a finding with an invalid citation does not ship.

IMPORTING THE RUNNER WAS NOT AN OPTION. `audit_review/runner.py` calls
`logging.basicConfig` at module scope, so importing it from another entry point
reconfigures the host's logging. This module does no such thing and holds no
state.

THE INDEX IS NOT THE MATERIALS. It resolves `docN:NN` references and quoted
spans; a report citing a document by section produces no entry, and an index
that resolves a name to the wrong file marks sound references broken. Callers
must treat an unresolved entry as a question for a reader, never as a settled
defect — REVIEW.md §4.0 states the rule for the review side, and the delivery
stage flags rather than repairs for the same reason.
"""
from __future__ import annotations

import re
from pathlib import Path
from typing import Any, Dict, List

# Report-side citation forms, from what models actually emit across 29 reports.
_CITE = re.compile(
    r"\b([A-Za-z][A-Za-z0-9_.\-/]*)\s*:\s*(?:lines?\s*)?(\d+)(?:\s*[-–]\s*(\d+))?")
_LOOKS_LIKE_A_FILE = re.compile(r"\.[A-Za-z][A-Za-z0-9]{0,4}$")
_DOCN = re.compile(r"^doc(\d)", re.I)
_FINDING = re.compile(r"^\s*\**\s*Finding\s+(\d+)[^\n\[]*\[([^\]]+)\]", re.M | re.I)

# A quote mark. Straight and curly, because a report that has been through a
# model may carry either, and marks ALTERNATE open/close: they are paired in
# document order, first with second, third with fourth. A regex that scans for
# an opening mark followed by a closing one slides past a pair too short to
# keep and matches from that pair's CLOSING mark to the next pair's opening
# one, capturing the prose between two quoted words. `"Streamlined" is not a
# fair description of "not present."` yielded ` is not a fair description of `
# as a quote. Pairing in order cannot do that.
_QUOTE_MARK = re.compile(r"[\"“”]")
# The 12-character floor drops `"active"` and other single-word quoting, which
# is not a citation of anything.
_MIN_QUOTE_CHARS = 12

# The fields of a §5 finding that carry evidence. Quotes are read from these
# and from nowhere else. `Gap:`, `Derivation:` and `Consequence:` are the
# auditor's own prose, where quoting a word back at the seller — `"Streamlined"
# is not a fair description of "not present."` — is legitimate writing and not
# a citation of anything. Scoping to the evidence fields removed every false
# miss across the three reports on disk: five misses became three, and all
# three survivors are real paraphrases. This is a structural parse of the
# labels §5 mandates, in the same class as `=== LIMITATIONS ===`.
#
# A field runs to the next blank line, the next §5 label, or the next finding —
# not to the end of its first line. `Basis:` is two lines by design, and a long
# Evidence quote may wrap.
_EVIDENCE_FIELD = re.compile(
    r"^[ \t]*\**[ \t]*(?:Claim|Evidence|Basis)\b"
    r".*?(?=\n[ \t]*\n|\n[ \t]*\**[ \t]*(?:Gap|Derivation|Consequence|"
    r"Escalates|Claim|Evidence|Basis|Finding)\b|\Z)",
    re.M | re.I | re.S)


_NOT_TEXT = re.compile(r"[^a-z0-9 ]+")
_SEGMENT = re.compile(r"(?<=[.;:])\s+|\n+")
_MIN_SEGMENT_CHARS = 8


def _flatten(s: str) -> str:
    """Lowercased alphanumerics and single spaces, and nothing else.

    Everything discarded here is a difference that is not a citation failure:
    markdown emphasis and bullet markers (`*   **Status:**` against `Status:`),
    the newline where a quote spans two source lines, a re-typed apostrophe,
    and the punctuation a model adds when it joins two bullets into one
    sentence. What survives is the words, in order.
    """
    return " ".join(_NOT_TEXT.sub(" ", s.lower()).split())


def _quoted_spans(text: str) -> List[str]:
    """Quoted spans in one field, pairing quote marks in document order.

    Marks alternate open/close, so the first pairs with the second and the
    third with the fourth. A pair too short to keep is CONSUMED rather than
    skipped: the alternative is matching from its closing mark to the next
    pair's opening one, which reports the prose between two quoted words as a
    quotation of the materials.
    """
    marks = [m.start() for m in _QUOTE_MARK.finditer(text)]
    return [" ".join(text[a + 1:b].split())
            for a, b in zip(marks[::2], marks[1::2])]


def resolve_quotes(report: str, target: Path) -> List[Dict[str, Any]]:
    """Every quote in a finding's evidence fields, looked for in the materials.

    THE SECOND HALF OF THE CITATION SURFACE. `_CITE` sees `docN:NN` and nothing
    else, so a report citing its evidence by section name — "(doc4, Backups
    section)" — resolved to zero entries and the reviewer read that absence as
    absence of evidence. On 2026-08-26 that produced "supported 1 of 15" on a
    report whose evidence quotes were, every one, verbatim correct.

    A quote resolves by file operation exactly as a line reference does, so it
    decides `[broken citation]` without judgement. It also survives the boundary
    a line number does not: the inspect subagent reformats numbered content into
    prose before the auditor ever sees it (code_subagent.py:407 numbers it;
    subagent.py:252 is the model's own answer, unnumbered).

    SEGMENTS, NOT SPANS. Reports quote composites — several bullets of doc4's
    Backups section joined into one sentence, sometimes reordered, sometimes
    with `...` between them. Every fact in such a quote can be verbatim while
    the span as typed appears nowhere, so each quote is split and its segments
    resolved separately.

    ONE QUOTE, ONE DOCUMENT. A quote resolves when SOME SINGLE DOCUMENT holds
    every one of its segments — not when each segment is found somewhere. The
    difference is a fabrication route: fragments drawn from two documents,
    joined into one sentence that reads as continuous evidence, every fragment
    verbatim and the sentence true of nothing. Measured across every quote in
    the three reports on disk, the rule costs nothing — not one needed two
    documents — and it settles attribution without reading the citation label,
    so a mistyped document name cannot defeat it. A quote whose segments are
    all found but never together is `split`, and it does not resolve.

    A quote is exact or it does not resolve. There is no near-match tier: a
    similarity threshold would put a judgement in the layer that exists to keep
    judgement out, and would need calibrating against failures we do not have.
    A paraphrase — "Integration with…" for "we integrate with…", or dropping
    the "all" from "all critical paths" — does not resolve, and the second of
    those is an auditor softening a claim while presenting it as verbatim.

    WHAT THIS CHANNEL IS NOT. It is the second half of the citation surface,
    not a replacement for the first. §5 requires `document:lines` on both
    halves of every finding, and that requirement is what makes a finding
    checkable at all: a quote is content, so a report can satisfy a
    quote-shaped rule by writing prose and pointing nowhere. Measured on
    2026-08-26, a report written to a quote-only contract left 10 of 33
    evidence fields with nothing a reader could search for, against 1 of 45
    under §5 as it stands. A citation that resolves to the wrong line is a
    different problem, and it belongs to the reviewer's judgement — METHOD §12a and
    REVIEW.md §6 — not to the citation format.
    """
    if not target.is_dir():
        return []
    flat = {f.name: _flatten(f.read_text(errors="replace"))
            for f in sorted(target.rglob("*")) if f.is_file()}

    body = _FINDING.sub("", report)     # titles quote the claim; they are headings
    out: List[Dict[str, Any]] = []
    seen = set()
    for field in _EVIDENCE_FIELD.finditer(body):
        for q in _quoted_spans(field.group(0)):
            if len(q) < _MIN_QUOTE_CHARS or q in seen:
                continue
            seen.add(q)
            segs = [s for s in (_flatten(x) for x in _SEGMENT.split(q))
                    if len(s) >= _MIN_SEGMENT_CHARS]
            if not segs:
                continue
            whole = _flatten(q)
            # Documents holding EVERY segment. One quote, one document.
            docs_hit = [n for n, text in flat.items()
                        if all(s in text for s in segs)]
            missing = [s for s in segs
                       if not any(s in text for text in flat.values())]
            contiguous = any(whole in flat[n] for n in docs_hit)
            rec: Dict[str, Any] = {
                "quote": q[:300], "resolved": bool(docs_hit),
                "how": "contiguous" if contiguous
                       else "segments" if docs_hit
                       else "split" if not missing
                       else "partial" if len(missing) < len(segs)
                       else "miss",
                "segments": len(segs),
                "segments_found": len(segs) - len(missing),
                "documents": docs_hit,
            }
            if missing:
                rec["missing"] = [s[:160] for s in missing]
            out.append(rec)
    return out


def scheme(citations: List[Dict[str, Any]],
           quotes: List[Dict[str, Any]]) -> Dict[str, Any]:
    """Whether the report's `docN:NN` references are line numbers at all.

    Decidable by file operation, and only in the negative: **an integer that
    exceeds its document's line count proves the reference is not a line
    number.** `doc2:13` against a five-line doc2 is not a broken line reference;
    it is claim 13 of the 13 that report enumerated for doc2.

    This does not say what the scheme IS. Nothing mechanical can. It says the
    declared one does not hold, which is what REVIEW.md §4.0 needs in order to
    stop rather than report a ratio computed over a misread.
    """
    over = [c for c in citations
            if not c.get("resolved") and " lines" in str(c.get("why", ""))]
    consistent: Optional[bool] = None
    if citations:
        consistent = not over

    # PER DOCUMENT, because the rate is what distinguishes the two cases. One
    # stray reference in forty is a typo. Four of the seven references to one
    # five-line document are a different coordinate system, and only the second
    # is grounds to stop. Measured across 29 reports on 2026-08-26: six carry at
    # least one over-length reference, and they range from 1-in-42 to 4-in-7.
    by_doc: Dict[str, Dict[str, Any]] = {}
    for c in citations:
        key = str(c.get("cited", "")).split(":")[0].strip().lower() or "?"
        d = by_doc.setdefault(key, {"refs": 0, "exceeding": 0, "max_cited": 0})
        d["refs"] += 1
        d["max_cited"] = max(d["max_cited"], int(c.get("line") or 0))
        if c in over:
            d["exceeding"] += 1
            d["file_lines"] = str(c.get("why", ""))

    return {
        "line_refs": len(citations),
        "line_refs_resolving": sum(1 for c in citations if c.get("resolved")),
        "line_refs_exceeding_file_length": len(over),
        "by_document": {k: v for k, v in sorted(by_doc.items())
                        if v["exceeding"]},
        "quotes": len(quotes),
        "quotes_resolving": sum(1 for q in quotes if q.get("resolved")),
        "quotes_contiguous": sum(1 for q in quotes
                                 if q.get("how") == "contiguous"),
        "consistent_with_line_numbering": consistent,
    }


def _locate(tok: str, by_rel: Dict[str, Path],
            by_base: Dict[str, List[Path]]) -> Tuple[Optional[Path], Any]:
    """The file a citation names. Returns (file, note); file is None on failure
    and the note says why. A note beside a file is an ambiguity worth recording,
    not a failure.

    HONOUR THE PATH WHEN THE REPORT GIVES ONE. `backend/app/tools/x.py:355` says
    which x.py it means, and reading only the basename discards the one piece of
    disambiguation the auditor supplied.

    A BARE NAME MATCHING SEVERAL FILES IS STILL RESOLVED, with the ambiguity
    recorded beside it. It was read as the shallowest until 2026-08-31, which
    on ChatterMate is wrong far more often than right — `ticket.py` names four
    files of 287, 390, 556 and 1045 lines, so a citation at line 755 was
    measured against the 287-line one and reported past the end. Every such
    report on that engagement was a false alarm.

    The candidates are returned so the caller can choose on evidence. What it
    must NOT do is refuse. A citation like `channels/slack.py:124-147,129,365`
    is the auditor saying the evidence is spread through a file and must be
    read; answering "unresolvable" reports the checker's limitation as a defect
    in the report, which is the failure this whole module exists to avoid.
    Unresolved is reserved for a citation pointing at nothing: no file of that
    name, or no candidate containing the line.
    """
    if tok in by_rel:
        return by_rel[tok], None
    if "/" in tok:
        hits = [f for rel, f in by_rel.items() if rel.endswith("/" + tok)]
        if len(hits) == 1:
            return hits[0], None
        if len(hits) > 1:
            # Several files end with this path. Same rule as a bare name: the
            # caller chooses on evidence and records the alternatives. Refusing
            # here rejected channels/slack.py:124-147 — a citation whose only
            # fault was that two files share that suffix.
            return None, hits
    cands = by_base.get(tok.rsplit("/", 1)[-1], [])
    if len(cands) == 1:
        return cands[0], None
    if len(cands) > 1:
        return None, cands
    return None, "no such document in the materials"


def _choose(cands: List[Path], lo: int, hi: int,
            report: str, at: int) -> Tuple[Optional[Path], Any]:
    """Pick among same-named files, on evidence, and always pick one.

    Three passes, strongest first.

      QUOTE. A quote written beside the citation is evidence about which file
      was actually read, and it tests content rather than the thing being
      checked. Only quotes near the citation count; further ones belong to
      other citations.

      LINE RANGE. Failing that, the candidates that contain the cited lines. A
      citation is often a range across a file — "read 124-147, then 129, then
      365" — which no short quote can stand for, and refusing those would
      reject the auditor's work for being spread out.

      SHALLOWEST. Failing both, the conventional reading of a bare name, with
      every alternative recorded so a reader can check another.

    Returns None only when no candidate contains the line, which is a citation
    pointing at nothing.
    """
    window = report[max(0, at - 400):at + 400]
    spans = [_flatten(q) for q in _quoted_spans(window)
             if len(q) >= _MIN_QUOTE_CHARS]

    def _body(c: Path) -> Optional[str]:
        try:
            lines = c.read_text(errors="replace").splitlines()
        except OSError:
            return None
        if lo < 1 or lo > len(lines):
            return None
        return _flatten("\n".join(lines[lo - 1:min(hi, len(lines))]))

    holds = [c for c in cands if _body(c) is not None]
    if not holds:
        return None, (f"{len(cands)} files share this name and none has a "
                      f"line {lo}")
    others = lambda pick: ", ".join(str(c) for c in holds if c is not pick)
    if spans:
        hit = [c for c in holds if any(s in (_body(c) or "") for s in spans)]
        if len(hit) == 1:
            return hit[0], (f"{len(cands)} files share this name; chosen by the "
                            f"quote beside the citation")
    if len(holds) == 1:
        return holds[0], (f"{len(cands)} files share this name; only this one "
                          f"has the cited lines")
    best = min(holds, key=lambda p: (len(p.parts), str(p)))
    return best, (f"{len(holds)} files share this name and contain the cited "
                  f"lines; read as the shallowest. Also: {others(best)}")


def resolve_citations(report: str, target: Path) -> Dict[str, Any]:
    """Every citation in the report, fetched from the materials.

    THE REVIEWER DOES NOT FETCH ITS OWN. Two reasons, and the second is the
    one that matters. A model asked to quote a line it has not read will
    sometimes produce a plausible line; handing it the bytes removes that
    possibility entirely. And `[broken citation]` is one of the two verdicts
    that fail a report, so it should be decided by a file operation rather than
    by judgement that was measured flipping on identical text.

    Matching is deliberately the same shape as measure/fixtures/dataroom/
    overlap.py: a document reference is `docN` or something with a file
    extension, so "12:30" and "Active customers: 120" are not citations.
    """
    # INDEXED BY PATH, NOT BY BASENAME. Keying on `f.name` let one file win
    # per basename and threw away the path the report gave, so a citation was
    # resolved against whichever same-named file rglob happened to reach last.
    # On the ChatterMate engagement that was not an edge case: the engagement
    # names TWO claim sources called README.md — the root one at 625 lines and
    # backend/app/knowledge/README.md at 149 — and every reference into the
    # root README past line 149 came back "exceeds file length". Fourteen of
    # cm_glm_2's twenty over-length marks, and three of cm_glm_1's eleven, were
    # this and nothing else. The reviewer then spends its judgement clearing
    # the index's own noise, and a reviewer that does not do that work grades
    # against a corrupted signal.
    by_rel: Dict[str, Path] = {}
    by_base: Dict[str, List[Path]] = {}
    if target.is_dir():
        for f in target.rglob("*"):
            if f.is_file():
                by_rel[f.relative_to(target).as_posix()] = f
                by_base.setdefault(f.name, []).append(f)
    by_docn = {}
    for name, cands in by_base.items():
        m = _DOCN.match(name)
        if m:
            by_docn.setdefault(f"doc{m.group(1)}", cands[0])

    out: List[Dict[str, Any]] = []
    seen = set()
    for m in _CITE.finditer(report):
        tok, lo, hi = m.group(1), int(m.group(2)), m.group(3)
        norm = tok.strip().lstrip("./")
        base = norm.rsplit("/", 1)[-1]
        dm = _DOCN.match(base)
        if not (dm or _LOOKS_LIKE_A_FILE.search(base)):
            continue
        key = (norm, lo, hi)
        if key in seen:
            continue
        seen.add(key)
        note = None
        if dm:
            f = by_docn.get(f"doc{dm.group(1)}")
            if f is None:
                note = "no such document in the materials"
        else:
            f, note = _locate(norm, by_rel, by_base)
            if f is None and isinstance(note, list):
                f, note = _choose(note, lo, int(hi) if hi else lo,
                                  report, m.start())
        rec: Dict[str, Any] = {"cited": m.group(0).strip(), "document": base,
                               "line": lo, "through": int(hi) if hi else lo}
        if f is None:
            rec["resolved"] = False
            rec["why"] = note if isinstance(note, str) else \
                "no such document in the materials"
        else:
            if note:
                rec["ambiguous"] = note
            lines = f.read_text(errors="replace").splitlines()
            hi_i = min(rec["through"], lo + 40)
            if lo < 1 or lo > len(lines):
                rec["resolved"] = False
                rec["why"] = f"{f.name} has {len(lines)} lines"
            else:
                rec["resolved"] = True
                rec["document"] = f.relative_to(target).as_posix()
                rec["text"] = "\n".join(
                    f"{n}: {lines[n-1]}" for n in range(lo, min(hi_i, len(lines)) + 1))
        out.append(rec)
    # AMBIGUITY IS NOT A FAILURE. A name matching several files is resolved by
    # _choose against the one holding the cited lines, and the alternatives are
    # recorded on the citation. Someone verifying opens two files instead of
    # one; that is a wish, not a defect, and reporting it as one dings an audit
    # for the checker's convenience. Unresolved now means what it says: no file
    # of that name, or no candidate containing the line.
    broken = [c for c in out if not c["resolved"]]
    quotes = resolve_quotes(report, target)
    return {"citations": out, "total": len(out), "broken": len(broken),
            "quotes": quotes, "scheme": scheme(out, quotes)}
