#!/usr/bin/env python3
"""Review a finished claims audit against the evidence it cites.

    python3 workflows/audit_review/runner.py --run <audit run directory>
    python3 workflows/audit_review/runner.py --run <dir> --model measure/models/grok_4p6.yaml

WHY. METHOD §12a: "There is no independent review of the report, and that is a
decision... Revisit it before the first engagement where a finding moves real
money." And §12 step 6b, on the audit checking its own citations: "This catches
a citation pointing at nothing. It will not catch one pointing convincingly at
the wrong line." This catches that one.

WHAT IS MECHANICAL HERE AND WHAT IS NOT. Two jobs are done before the reviewer
starts, because they have answers that do not depend on judgement:

  conformance   the method-conformance criteria that need no answer key —
                markers, closed vocabularies, a claim surface that closed
  citations     every cited line, fetched, and every quoted span, looked for.
                Whether doc4:16-19 EXISTS is a file operation, and it decides
                one of the two verdicts that fail a report. Resolving it here
                means the reviewer cannot hallucinate a line it never fetched,
                and spends its judgement on whether the line SUPPORTS the claim.
  scheme        whether those integers can be line numbers at all. A reference
                past the end of its document proves they are not — see §4.0.
                This does not decide admissibility; it is what the reviewer
                decides it on.

An LLM asked whether `=== LIMITATIONS ===` appears is slower, dearer and less
reliable than a substring test. The workflow's value is the checks with no
mechanical form.

ONE REVIEW PER RUN. The runner refuses if <run>/review/ exists: a reviewer whose
`inspect` can see a previous review is not independent of it.

NOT A RE-AUDIT. The subject is the report, not the business — REVIEW.md §2, §10.
"""

from __future__ import annotations

import argparse
import datetime
import json
import logging
import re
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

HERE = Path(__file__).resolve().parent          # workflows/audit_review
REPO = HERE.parent.parent
for p in (str(REPO), str(REPO / "src")):
    if p not in sys.path:
        sys.path.insert(0, p)

import yaml                                                    # noqa: E402

SCENARIO = HERE / "scenario.yaml"
SOURCE = "User"

logging.basicConfig(level=logging.WARNING)
logger = logging.getLogger("audit_review")
logger.setLevel(logging.INFO)

# The second deliverable is ASKED FOR, not detected — the same rule the audit
# runner arrived at. Its opening sentence is a stable sentinel: harness text,
# emitted verbatim, so matching it is safe in a way matching model output is not.
SUMMARY_REQUEST = ("The review is received. Now the review summary, per §8. "
                   "The whole reply is the summary — no preamble, no "
                   "restatement of the review.")
CONTINUE = "continue"

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
    different problem, and it belongs to the reviewer's judgement — §12a and
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
    files = {f.name: f for f in target.rglob("*") if f.is_file()} \
        if target.is_dir() else {}
    by_docn = {}
    for name, f in files.items():
        m = _DOCN.match(name)
        if m:
            by_docn.setdefault(f"doc{m.group(1)}", f)

    out: List[Dict[str, Any]] = []
    seen = set()
    for m in _CITE.finditer(report):
        tok, lo, hi = m.group(1), int(m.group(2)), m.group(3)
        base = tok.rsplit("/", 1)[-1]
        dm = _DOCN.match(base)
        if not (dm or _LOOKS_LIKE_A_FILE.search(base)):
            continue
        key = (base, lo, hi)
        if key in seen:
            continue
        seen.add(key)
        f = by_docn.get(f"doc{dm.group(1)}") if dm else files.get(base)
        rec: Dict[str, Any] = {"cited": m.group(0).strip(), "document": base,
                               "line": lo, "through": int(hi) if hi else lo}
        if f is None:
            rec["resolved"] = False
            rec["why"] = "no such document in the materials"
        else:
            lines = f.read_text(errors="replace").splitlines()
            hi_i = min(rec["through"], lo + 40)
            if lo < 1 or lo > len(lines):
                rec["resolved"] = False
                rec["why"] = f"{f.name} has {len(lines)} lines"
            else:
                rec["resolved"] = True
                rec["document"] = f.name
                rec["text"] = "\n".join(
                    f"{n}: {lines[n-1]}" for n in range(lo, min(hi_i, len(lines)) + 1))
        out.append(rec)
    broken = [c for c in out if not c["resolved"]]
    quotes = resolve_quotes(report, target)
    return {"citations": out, "total": len(out), "broken": len(broken),
            "quotes": quotes, "scheme": scheme(out, quotes)}


def unpointed_fields(report: str) -> List[str]:
    """Evidence fields carrying neither a line reference nor a quote.

    THE DENOMINATOR THAT WAS MISSING. `resolve_citations` and `resolve_quotes`
    score the pointers a report makes. Neither notices a field that makes none,
    so a report can read `16 of 23 quotes resolving` while ten of its evidence
    fields say `Evidence (doc4): Single standard-1x dyno, no read replicas` —
    prose, naming a whole document, with nothing a reader can search for. That
    is §5's requirement failing silently: "without both, a reader cannot check
    the finding and the practice cannot defend it."

    Measured 2026-08-26 across four reports: 1 of 29 and 1 of 45 under §5 as it
    stands, and both of those assert that no evidence exists ("no source code
    provided to verify exact version"), which is the one legitimate case. A
    report written to a quote-only contract left 10 of 33.

    Structure only, no corpus. Whether a pointer RESOLVES is the other two
    functions' job; this one asks whether the auditor pointed at all.
    """
    body = _FINDING.sub("", report)     # titles quote the claim; they are headings
    out: List[str] = []
    for field in _EVIDENCE_FIELD.finditer(body):
        text = field.group(0)
        quoted = [q for q in _quoted_spans(text) if len(q) >= _MIN_QUOTE_CHARS]
        if quoted or _CITE.search(text):
            continue
        out.append(" ".join(text.split())[:160])
    return out


def conformance(run: Path) -> Dict[str, Any]:
    """The method-conformance checks that need no answer key.

    Reused from the fixture scorer rather than reimplemented — these are the
    five criteria that hold for ANY engagement, including a paying client's
    where no answer key exists or ever will. Recall against a key stays in
    measure/, which is where the key is.
    """
    sys.path.insert(0, str(REPO / "measure" / "fixtures" / "dataroom"))
    import score                                               # noqa: E402
    report = (run / "report.md").read_text(errors="replace") \
        if (run / "report.md").is_file() else ""
    gap = (run / "gap_map.md").read_text(errors="replace") \
        if (run / "gap_map.md").is_file() else ""
    vc = score.verdict_conformance(report)
    rec = score.recommendation_of(report)
    el = score.gap_map_elements(gap) if gap else {}
    return {
        "recommendation": rec,
        "§6 verdicts only": not vc["off_vocabulary"],
        "off_vocabulary": vc["off_vocabulary"],
        "limitations statement": bool(score._LIMITS_RE.search(report)),
        "gap map present": bool(gap.strip()),
        "§15 elements missing": [k for k, ok in el.items() if not ok],
        "evidence fields": len(_EVIDENCE_FIELD.findall(report)),
        "evidence fields pointing nowhere": unpointed_fields(report),
        "findings": [{"n": int(n), "verdict": v.strip()}
                     for n, v in _FINDING.findall(report)],
    }


def build_config(run: Path, world: str, model_path: Optional[Path],
                 target: Path) -> Tuple[str, Dict[str, Any]]:
    """Same shape as the audit runner's, and the same reasons: the model config
    REPLACES the llm_config block rather than merging into it, and per-session
    paths are set here rather than by editing the committed scenario."""
    from launcher import parse_characters                      # noqa: E402
    scenario = yaml.safe_load(SCENARIO.read_text(encoding="utf-8")) or {}
    scen_llm = dict(scenario.get("llm_config") or {})
    if model_path:
        doc = yaml.safe_load(Path(model_path).read_text(encoding="utf-8")) or {}
        llm = dict(doc.get("llm_config") or {})
        if not llm:
            raise SystemExit(f"{model_path}: no llm_config block")
        for ch in (scenario.get("characters") or {}).values():
            if isinstance(ch, dict) and ch.get("mode") == "chat":
                ch["llm_config"] = dict(llm)
        scen_llm.update(llm)
    world_cfg = dict(scenario.get("world_config") or {})
    world_cfg["world_name"] = world
    chars = parse_characters(scenario, scen_llm, world_cfg,
                             scenario.get("setting", ""),
                             scenario.get("alt_llm_config") or {})
    chat = [(n, c) for n, c in chars if c.get("mode") == "chat"]
    if len(chat) != 1:
        raise SystemExit(f"expected 1 chat character, found {len(chat)}")
    name, cfg = chat[0]
    cfg["autonomy_enabled"] = False
    cfg["external_repo"] = str(target)
    cfg["inspect_repo"] = str(run)
    return name, cfg


BRIEF = """You are reviewing the claims audit in this run directory, per REVIEW.md.

The report and the Gap Map are under `inspect`, with the auditor's working
record. The materials that audit examined are under `inspect_external`.

Citations have been resolved for you as far as a file operation reaches.
`review/citations.json` holds three things: every `docN:NN` reference with the
text of the line it names; every quote in a Claim, Evidence or Basis field,
looked for in the materials, with the document that holds all of it; and a
`scheme` block saying whether those integers can be line numbers at all.

A quote resolves only when one document holds every part of it. `split` means
each part was found but never together in one document — fragments from two
documents joined into a sentence that reads as continuous evidence. `miss`
means the words are not in the materials as written.

`review/conformance.json` holds the checks that need no answer key, including
`evidence fields` and `evidence fields pointing nowhere` — the fields carrying
neither a reference nor a quote, which §4.0's second question turns on. A field
that names a document and then writes prose gives a reader nothing to search
for, and it is invisible in the citation counts because it makes no citation.

Do not re-fetch what it has already resolved, and do not assess support for a
finding whose citation is broken. **It is not the whole of the materials.** A
reference it does not cover — an evidence document named by section rather than
by line — is absent from it because no regex could fetch it, not because the
document says nothing. Read anything it does not cover under `inspect_external`.

Settle admissibility first, per §4.0. If the report's citation scheme cannot be
established, say so and stop: do not enumerate, do not check findings, do not
report a supported ratio.

Otherwise enumerate the findings and close the review surface as the method
says, then check every one. Work in as many legs as you need — end a leg with
`yield` and I will say continue. End with the review; I will ask for the summary
after it.
"""


# Exceptions in a finished review, as §6 writes them.
_EXCEPTION = re.compile(
    r"^\s*\**\s*Exception\s+\d+\s*:\s*report\s+Finding\s+(\d+)[^\n\[]*\[([^\]]+)\]",
    re.M | re.I)

# The two FAIL verdicts that rest on judgement. `[broken citation]` and
# `[uncited]` are decided by a file operation — citations.json says a
# reference resolves or it does not, conformance.json says a field carries a
# pointer or it does not — so re-asking a model would spend money to
# re-derive a fact it cannot change.
_JUDGEMENT_VERDICTS = ("unsupported", "indeterminate")


def judgement_exceptions(review_text: str) -> List[Dict[str, str]]:
    """Findings a review disputed on judgement rather than on a file fact."""
    out, seen = [], set()
    for finding, verdict in _EXCEPTION.findall(review_text or ""):
        v = verdict.strip().lower()
        if v in _JUDGEMENT_VERDICTS and finding not in seen:
            seen.add(finding)
            out.append({"finding": finding, "verdict": v})
    return out


CONFIRM_BRIEF = """A colleague is reviewing the claims audit in this run directory and has
asked for a second opinion on specific findings, before anything is
reported.

Check these findings, and only these: {findings}.

For each one, read what the report claims and what its citations actually
say — `review/citations.json` has every cited line already fetched — and give
it a verdict from REVIEW.md §6. Judge each finding on the evidence in front
of you. You have not been told what anyone else concluded, and you should not
try to infer it: a second opinion that guesses at the first is not one.

REVIEW.md §2 and §10 bind here as everywhere. The subject is the report, not
the business. A claim the report never made is not an exception, and a
statement about what the audit could NOT verify is a coverage statement
rather than a claim about the target.

Answer with one line per finding and nothing else:

    Finding <N>: [verdict] — <one sentence, citing the line that settles it>
"""


# When the review fails a finding, retest it once. The fail stands only if
# the retest agrees; a single disagreement ignores it. Two reviewers in
# total: the one who wrote the review, plus CONFIRMING_REVIEWERS more.
REVIEWERS_REQUIRED = 2
CONFIRMING_REVIEWERS = REVIEWERS_REQUIRED - 1


def confirm_exceptions(run: Path, world: str, model_path: Optional[Path],
                       target: Path, disputed: List[Dict[str, str]],
                       ) -> Dict[str, Any]:
    """Retest the findings the first reviewer failed.

    THE RULE. When the review fails a finding, that finding is retested
    once, by a reviewer who has not seen the first verdict. The fail stands
    only if the retest reaches the same verdict. A single disagreement
    ignores the fail, and the disagreement is reported.

    WHY IT APPLIES ONLY TO FINDINGS THAT FAIL. §9 fails a whole report on a
    single exception, so one wrong judgement condemns the report. Measured on
    2026-08-26: one clean report was reviewed five times and came back
    supported 12 of 12 four times and 11 of 12 once, and the one dissent had
    rebutted a finding by attacking a claim the audit never made. A wrong
    judgement the other way cannot do the same damage, because passing a bad
    report would need every real exception missed at once. A reviewer
    asserting a defect in finished work carries the higher standard; a
    reviewer finding no defect does not.

    A DISAGREEMENT IS NOT NOISE. One reviewer finding a fault and the other
    not means the finding is genuinely borderline, and saying so is more use
    than rounding it to yes or no. The tally is reported either way.

    BLIND. The retest is launched before `review.md` is written to disk, so
    it cannot reach the first verdict through `inspect`, and it is not told
    it. It confirms; it does not overrule. A finding whose fail does not
    stand stays in the review as recorded, with its tally.
    """
    names = ", ".join(f"Finding {d['finding']}" for d in disputed)
    from chat.chat_loop import ChatLoop                        # noqa: E402
    # Imported here as well as in main(): main()'s import is local to it, and
    # a NameError raised inside the retest reads as "could not obtain the
    # retest", which silently turns every failed finding into one that does
    # not stand. Cost one review on 2026-08-26 before it was noticed.
    from workflows.claims_audit.runner import latest_reply     # noqa: E402
    t0, opinions, replies, models = time.time(), [], [], []

    for n in range(1, CONFIRMING_REVIEWERS + 1):
        # A world of its own: the retest must not see the first review any
        # more than it is told its verdict.
        name, cfg = build_config(run, f"{world}_{n}", model_path, target)
        loop = ChatLoop(character_name=name, character_config=cfg)
        try:
            loop._process_user_turn(source=SOURCE, close=False,
                                    text=CONFIRM_BRIEF.format(findings=names))
            reply = latest_reply(loop, SOURCE)
            models.append(loop.backend.resolved_model())
        except Exception as e:                                 # noqa: BLE001
            logger.warning("retest %d failed: %s", n, e)
            return {"ran": False, "error": f"{type(e).__name__}: {e}",
                    "reviewers_required": REVIEWERS_REQUIRED,
                    "reviewers_obtained": len(opinions) + 1,
                    "disputed": disputed}
        finally:
            try:
                loop._post_turn_executor.shutdown(wait=True)
            except Exception as e:                             # noqa: BLE001
                logger.warning("confirm executor shutdown failed: %s", e)
        replies.append(reply)
        verdicts = {}
        for m in re.finditer(r"Finding\s+(\d+)\s*:\s*\[([^\]]+)\]",
                             reply or "", re.I):
            verdicts[m.group(1)] = m.group(2).strip().lower()
        opinions.append(verdicts)

    results = []
    for d in disputed:
        others = [o.get(d["finding"]) for o in opinions]
        agreeing = 1 + sum(1 for v in others if v == d["verdict"])
        results.append({**d, "other_verdicts": others,
                        "agreeing": agreeing,
                        "of": REVIEWERS_REQUIRED,
                        # The retest must have reached the same verdict. A
                        # retest that returned no verdict for this finding
                        # has not agreed to fail it.
                        "accepted_as_failed": agreeing == REVIEWERS_REQUIRED})
    n_acc = sum(1 for r in results if r["accepted_as_failed"])
    logger.info("retest: %d of %d failed findings stand", n_acc, len(results))
    return {"ran": True, "error": None, "world": world,
            "reviewers_required": REVIEWERS_REQUIRED,
            "resolved_models": models,
            "wall_clock_s": round(time.time() - t0, 1),
            "replies": replies, "results": results,
            "accepted_as_failed": n_acc, "failed_findings": len(results)}


def _confirmation_note(c: Dict[str, Any]) -> str:
    """What the summary turn is told about the retest.

    States what it found and does not argue the verdict: REVIEW.md §9 carries
    the rule, and a runner that also reasoned the conclusion would be writing
    the review.
    """
    if not c.get("ran"):
        return ("\n\n[The client's process could not obtain the retest. Per "
                "§9 the result is INCONCLUSIVE: report ADMISSIBLE or "
                "INADMISSIBLE as you found it, list every finding you failed "
                "as found but not retested, and give neither PASS nor FAIL. "
                "A retest that could not be run is not a finding that did "
                "not hold.]")
    lines = []
    for r in c["results"]:
        others = ", ".join(f"[{v}]" if v else "[no verdict returned]"
                           for v in r["other_verdicts"])
        lines.append(
            f"  Finding {r['finding']}: you found [{r['verdict']}]; the "
            f"retest found {others} — {r['agreeing']} of {r['of']} — "
            + ("STANDS" if r["accepted_as_failed"] else "DOES NOT STAND"))
    return ("\n\n[The retest, obtained by the client's process from a reviewer "
            "who was not told your verdicts and did not see your review:\n"
            + "\n".join(lines) +
            "\n\nApply §9. A finding you failed on judgement stands only where "
            "the retest reached the same verdict on it. Report a finding "
            "whose fail does not stand as found but not upheld, and give the "
            "tally — do not delete it, and do not restate it as agreement.]")


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", type=Path, required=True,
                    help="the audit run directory to review")
    ap.add_argument("--model", type=Path, default=None,
                    help="YAML with an llm_config block. Need not be the model "
                         "that produced the audit — arguably should not be")
    ap.add_argument("--world", default=None)
    ap.add_argument("--max-turns", type=int, default=25)
    args = ap.parse_args()

    run = args.run.resolve()
    meta_p = run / "run_meta.json"
    if not meta_p.is_file():
        raise SystemExit(f"{run}: no run_meta.json — not a run directory")
    if not (run / "report.md").is_file():
        raise SystemExit(f"{run}: no report.md to review")
    out = run / "review"
    if out.exists():
        raise SystemExit(
            f"{out} already exists. One review per run: a reviewer whose "
            f"`inspect` can see a previous review is not independent of it. "
            f"Move it OUT of the run directory, or delete it, to review again.")
    # RENAMING IS NOT MOVING. `inspect` reaches the whole run directory, so a
    # previous review parked alongside as review.8192-era/ or
    # review.pre-admissibility/ is exactly as visible as review/ was. The
    # guard above was written for the name and missed the point of itself.
    stale = sorted(d.name for d in run.glob("review*")
                   if d.is_dir() and (d / "review.md").is_file())
    if stale:
        raise SystemExit(
            f"{run} still holds a previous review: {', '.join(stale)}. "
            f"`inspect` reaches the whole run directory, so renaming one does "
            f"not hide it. Move it outside the run directory to review again.")

    meta = json.loads(meta_p.read_text(encoding="utf-8"))
    target = Path(meta.get("external_repo") or "").resolve()
    if not target.is_dir():
        raise SystemExit(f"target {target} does not exist — citations cannot "
                         f"be resolved, and a review without them is opinion")
    out.mkdir(parents=True)

    report = (run / "report.md").read_text(encoding="utf-8", errors="replace")
    cites = resolve_citations(report, target)
    (out / "citations.json").write_text(json.dumps(cites, indent=2), encoding="utf-8")
    conf = conformance(run)
    (out / "conformance.json").write_text(json.dumps(conf, indent=2), encoding="utf-8")
    logger.info("resolved %d citations, %d broken; %d quotes, %d resolving; "
                "%d findings parsed", cites["total"], cites["broken"],
                cites["scheme"]["quotes"], cites["scheme"]["quotes_resolving"],
                len(conf["findings"]))

    world = args.world or f"review_{run.name.split('_', 1)[-1]}"
    if (REPO / "scenarios" / world).exists():
        raise SystemExit(f"world '{world}' already exists — pass --world")

    name, cfg = build_config(run, world, args.model, target)
    from chat.chat_loop import ChatLoop                        # noqa: E402
    from chat.model_params import TOP_P                        # noqa: E402
    from workflows.claims_audit.runner import (                # noqa: E402
        latest_reply, last_exit_reason, git_rev)
    loop = ChatLoop(character_name=name, character_config=cfg)
    resolved_model = loop.backend.resolved_model()
    logger.info("reviewing %s with %s", run.name, resolved_model)

    ts = datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H-%M-%SZ")
    t0, legs, error, summary_sent, review_text = time.time(), [], None, False, ""
    confirmation: Optional[Dict[str, Any]] = None
    try:
        text = BRIEF
        for i in range(args.max_turns):
            loop._process_user_turn(source=SOURCE, text=text, close=False)
            reply = latest_reply(loop, SOURCE)
            exit_reason = last_exit_reason(world, name)
            legs.append({"leg": i + 1, "exit_reason": exit_reason,
                         "reply_chars": len(reply)})
            logger.info("leg %d: exit=%s chars=%d", i + 1, exit_reason, len(reply))
            if exit_reason in ("llm_error", "crashed"):
                error = f"turn {i + 1} ended {exit_reason} — review is not valid"
                break
            if summary_sent:
                break
            if exit_reason == "yield":
                text = (CONTINUE + f"\n\n[review state, recorded by the client's "
                        f"process — leg {i + 2} of {args.max_turns}, "
                        f"{(time.time() - t0) / 60:.0f} min elapsed.]")
                continue
            review_text, summary_sent = reply, True
            # Blind second opinion on judgement-based disputes, BEFORE
            # review.md is written and before the summary is asked for, so
            # the confirming reviewer cannot reach the first verdict and the
            # summary can state the outcome. See confirm_exceptions.
            disputed = judgement_exceptions(review_text)
            if disputed:
                logger.info("%d finding(s) failed on judgement — retesting",
                            len(disputed))
                confirmation = confirm_exceptions(
                    run, f"{world}_confirm", args.model, target, disputed)
                text = SUMMARY_REQUEST + _confirmation_note(confirmation)
            else:
                text = SUMMARY_REQUEST
    except Exception as e:                                     # noqa: BLE001
        error = f"{type(e).__name__}: {e}"
        logger.exception("review failed")
    finally:
        try:
            loop._post_turn_executor.shutdown(wait=True)
        except Exception as e:                                 # noqa: BLE001
            logger.warning("executor shutdown failed: %s", e)

    final = latest_reply(loop, SOURCE)
    if summary_sent:
        (out / "review.md").write_text((review_text or "").rstrip() + "\n",
                                       encoding="utf-8")
        (out / "summary.md").write_text(final.strip() + "\n", encoding="utf-8")
    elif final:
        (out / "review.md").write_text(final, encoding="utf-8")
        logger.warning("no summary leg — review summary not produced")

    (out / "review_meta.json").write_text(json.dumps({
        "reviewed_run": run.name,
        "world": world,
        "model_config": str(args.model) if args.model else "(scenario default)",
        "resolved_model": resolved_model,
        "top_p": TOP_P,
        "target": str(target),
        "target_rev": meta.get("target_rev"),
        "harness_rev": git_rev(REPO),
        "citations_total": cites["total"],
        "citations_broken": cites["broken"],
        "scheme": cites["scheme"],
        "findings_parsed": len(conf["findings"]),
        "confirmation": confirmation,
        "legs": legs,
        "wall_clock_s": round(time.time() - t0, 1),
        "error": error,
        "captured_at_utc": ts,
    }, indent=2, default=str) + "\n", encoding="utf-8")

    print(f"\n{len(legs)} legs, {round(time.time() - t0, 1)}s, error={error}")
    sch = cites["scheme"]
    print(f"citations: {cites['total']} line refs, {cites['broken']} broken; "
          f"{sch['quotes']} quotes, {sch['quotes_resolving']} resolving")
    if sch["consistent_with_line_numbering"] is False:
        print(f"scheme: NOT line numbers — "
              f"{sch['line_refs_exceeding_file_length']} references exceed "
              f"their document's line count. See REVIEW.md §4.0.")
    print(f"review: {out}/review.md, summary.md")
    return 1 if error else 0


if __name__ == "__main__":
    raise SystemExit(main())
