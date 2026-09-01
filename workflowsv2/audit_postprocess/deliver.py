#!/usr/bin/env python3
"""Turn a finished audit run into a client deliverable and an editor's worklist.

    python3 workflowsv2/audit_postprocess/deliver.py --run <audit run directory>

WHAT THIS IS. The audit produces evidence; this produces a document. It
REORGANISES AND CHECKS. It does not re-judge: no finding is added, removed,
reworded or re-verdicted, and every citation passes through byte-for-byte.

NO MODEL. Bruce's call, 2026-08-30. Everything here is either a file operation
or markdown surgery over a format METHOD §16 already specifies, and the one
judgement anyone wanted — ranking findings by consequence — is going to a human
editor because it is not derivable from the materials at all. The audit cannot
know whether this buyer weighs security posture above feature completeness.

WHY THE CHECKS ARE THE POINT, and the reformatting is the bonus. METHOD §12
step 6b says "a finding with a missing or invalid citation does not ship", and
nothing enforced it before delivery.

WHAT THAT CHECK ACTUALLY FOUND, once it could tell two things apart: no broken
citations at all, and a great many underspecified ones. On ChatterMate, 18 of
79 and 23 of 94 citations name a file that exists several times in the target —
`ticket.py` names four files of 287, 390, 556 and 1045 lines — so a reader
cannot tell which was meant. They were read as "past the end of the file" for a
day, and reported as shipped defects, because the resolver picked a namesake
and the caller conflated the two categories. The independent review that said
none were broken was right.

FLAG, NEVER REPAIR. The citation index resolves `docN:NN` references and quoted
spans and nothing else, and it can resolve a name to the wrong file — which is
why REVIEW.md §4.0 forbids a reviewer from assigning `[broken citation]` on the
index alone. The same caution binds here: an unresolved citation is a question
for the editor, not a settled defect, and this script must not "fix" one.

STRUCTURE, NOT KEYWORDS. Sections are classified by SHAPE — a findings block
carries `**Finding N: … — [verdict]**` headers, an inventory carries bare
`N. text — [verdict]: citations` lines — never by matching words in a heading.
The shapes come from METHOD §5 and are what the audit_review parsers already
key on.
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from workflowsv2 import blocks                                    # noqa: E402
# A §5 finding header, and the claim ids it names, from the module that owns
# both. This file kept its own copy of `_FINDING` and its own `_CLAIM_ID` until
# 2026-08-31, defended by a comment promising they stayed byte-identical to
# patterns nothing checked them against. Two of the three copies then drifted in
# what they KEPT — the claim ids reached this file and were discarded, so the
# delivery brief could say "4 findings" and never "5 claims".
from workflowsv2.citations import (_FINDING, finding_claims,      # noqa: E402
                                 resolve_citations)
from workflowsv2 import coverage as cov, issues as iss             # noqa: E402

# An inventory line: a bare ordinal, a title, a verdict, then citations. This is
# the 35-line block that made up the bulk of the ChatterMate report body.
_INVENTORY = re.compile(r"^\s*(\d+)\.\s+(?!\*)([^\n\[]{3,}?)\s*[—-]\s*\[([^\]]+)\]",
                        re.M)
# The same header, with the title captured. Separate from the shared `_FINDING`
# because an editor ranking findings needs titles and a parser checking verdicts
# does not; the claim ids come from `finding_claims`, not from here.
_FINDING_TITLE = re.compile(
    r"^\s*\**\s*Finding\s+(\d+)\s*(?:\(claims?[^)]*\))?\s*:?\s*"
    r"(.*?)\s*[—-]\s*\[([^\]]+)\]", re.M)
_H2 = re.compile(r"^## .*$", re.M)

# WHAT THE BRACKETS MEAN, for a reader who has never seen METHOD. Every finding
# header ends in a verdict — `— [delta]` — and until 2026-08-30 nothing in the
# deliverable said what one was. The delivery agent cannot supply this: it is
# given the report, the gap map and the check files, not METHOD, so it has no
# definitions to gloss from and would be guessing.
#
# Wording condensed from METHOD §6's verdict table for a buyer rather than an
# auditor. lint_workflow checks the KEYS here against that table, so a verdict
# added or retired there fails the lint rather than drifting silently. The
# wording is ours; the set is METHOD's.
VERDICT_GLOSS = (
    ("[real]", "the materials bear the claim out"),
    ("[real, minor caveat]", "the claim holds, with a discrepancy that does "
                             "not affect the decision"),
    ("[real, operational caveat]", "the claim holds today, but how the system "
                                   "is operated qualifies it"),
    ("[partial]", "the claim is substantially true and has a specific, cited "
                  "gap"),
    ("[delta]", "the claim is not true — the claim source says one thing and "
                "the materials show another"),
    ("[unverifiable]", "the audit tried and the supplied materials could not "
                       "settle it"),
)


def how_to_read() -> str:
    """A short note placed before the findings. Fixed text, not generated."""
    lines = ["## How to read a finding", "",
             "Each finding names the seller's claim, the evidence that settles "
             "it, and the gap between them. The reference after each piece of "
             "evidence is a file and line number in the materials examined, so "
             "any finding can be checked.",
             "",
             "The bracket at the end of a finding's title is the audit's "
             "verdict on that claim:", ""]
    lines += [f"- `{k}` — {v}" for k, v in VERDICT_GLOSS]
    return "\n".join(lines) + "\n"


def sections(body: str) -> List[Tuple[str, str]]:
    """The report split at its `##` headings, in order, heading text kept."""
    marks = list(_H2.finditer(body))
    if not marks:
        return [("", body)]
    out = [("", body[:marks[0].start()])] if marks[0].start() else []
    for i, m in enumerate(marks):
        end = marks[i + 1].start() if i + 1 < len(marks) else len(body)
        out.append((m.group(0), body[m.end():end]))
    return out


def is_inventory(text: str) -> bool:
    """A section of bare numbered verdict lines rather than §5 finding blocks.

    Shape, not words: inventory lines carry a verdict and no `Finding N` header.
    A section qualifies when it holds several such lines and no finding header,
    which is what separates the ChatterMate report's 35-line supported list from
    its twelve narrative findings.
    """
    return len(_INVENTORY.findall(text)) >= 3 and not _FINDING.search(text)


# An Exception block opener, loose. The strict §6 pattern and the meaning of
# each disposition belong to audit_review; delivery only needs to know whether
# a reviewer raised anything and whether it touched a citation we flag, so it
# reads the review as text rather than re-implementing that parser.
_EXCEPTION_ANY = re.compile(r"(?m)^\s*\**\s*Exception\s+\d+\s*:")
_INADMISSIBLE = re.compile(r"(?m)^\s*\**\s*(?:Result\s*:\s*)?INADMISSIBLE\b")


def review_signal(run: Path, unresolved: List[Dict[str, Any]]) -> Dict[str, Any]:
    """What an independent review, if one ran, says about processing this run.

    NOT CONTENT FOR THE CLIENT. A review is internal QA; "independently
    reviewed" in a deliverable is a marketing claim that would have to be true
    and defensible every time. This reads it to decide how to PROCESS.

    The decision it actually changes is the citation worklist. Delivery flags
    every citation the index could not resolve and tells the editor to check
    each against the materials. If a reviewer — who is instructed by REVIEW.md
    §4.0 to do exactly that check, against the documents rather than the index —
    already examined one, the editor's job on it is different. And if the review
    never mentions any of them, that is worth knowing too: on the ChatterMate
    medium run the index found three and the review reported none, without
    saying whether it had looked.
    """
    rev = run / "review"
    text = (rev / "review.md").read_text(errors="replace") \
        if (rev / "review.md").is_file() else ""
    if not text:
        return {"ran": False}
    return {
        "ran": True,
        "inadmissible": bool(_INADMISSIBLE.search(text)),
        "exceptions_raised": len(_EXCEPTION_ANY.findall(text)),
        # Substring presence of the exact citation token we flagged — a factual
        # text search, not a judgement about what the reviewer concluded.
        "flagged_citations_mentioned": [c.get("cited") for c in unresolved
                                        if str(c.get("cited")) in text],
    }


def parked_notes(surface: str) -> List[str]:
    """Prose an auditor left in the claim surface that is not a claim.

    STRUCTURAL, NOT SEMANTIC. A claim surface is a count line and numbered
    claims; anything else in it is something the auditor wrote and had nowhere
    to put. No word matching is involved — the test is "is this line a numbered
    claim or the count", which is the shape §12 step 2 specifies.

    On the ChatterMate medium run this is where a real observation ended up:
    the seller's knowledge README states max_depth 3 / max_links 10 in its
    example and 5 / 25 in its options table, "to be tested, not yet a finding".
    It is the last line of the last appendix, where no client will reach it.
    """
    out = []
    for line in (surface or "").splitlines():
        s = line.strip()
        if (not s or re.match(r"^\d+\.", s) or re.match(r"^\d+ claims\b", s)
                or s.startswith("===") or re.match(r"^[^:]{1,60}\(\d+ claims?\)", s)):
            continue
        # METHOD §12 step 2 mandates a per-document count below the total, so
        # that line belongs to the surface too. Recognised by shape — two or
        # more `<filename> <count>` pairs — not by the words in it.
        if len(re.findall(r"[\w./-]+\.\w{1,5}\s+\d+", s)) >= 2:
            continue
        if len(s) > 80:                      # a sentence, not a list header
            out.append(s)
    return out


def audit(run: Path) -> Dict[str, Any]:
    """Every mechanical check, with no judgement in any of them."""
    report = (run / "report.md").read_text(errors="replace") \
        if (run / "report.md").is_file() else ""
    gap = (run / "gap_map.md").read_text(errors="replace") \
        if (run / "gap_map.md").is_file() else ""
    meta = json.loads((run / "run_meta.json").read_text()) \
        if (run / "run_meta.json").is_file() else {}
    target = Path(meta.get("external_repo") or ".")

    cites = resolve_citations(report, target) if target.is_dir() else {}
    rows = cites.get("citations") or []
    unresolved = [c for c in rows if str(c.get("resolved")).lower() == "false"]

    titles = {int(n): ti.strip() for n, ti, _ in _FINDING_TITLE.findall(report)}
    findings = [{"n": f["n"], "verdict": f["verdict"],
                 "title": titles.get(f["n"], ""), "claims": f["claims"]}
                for f in finding_claims(report)]
    inv = [{"n": int(n), "title": t.strip(), "verdict": v.strip()}
           for n, t, v in _INVENTORY.findall(report)]

    return {
        "report_chars": len(report),
        "gap_map_present": bool(gap.strip()),
        "target": str(target),
        "citations_total": len(rows),
        "citations_unresolved": unresolved,
        "findings": findings,
        "findings_without_claim_id": [f["n"] for f in findings
                                      if not f["claims"]
                                      and f["verdict"] != "derived"],
        "inventory_entries": inv,
        "claim_surface_recovered": bool(claim_surface(run)),
        "parked_notes": parked_notes(claim_surface(run)),
        "review": review_signal(run, unresolved),
        "coverage": {k: v for k, v in coverage_of(run).items()
                     if k in ("legacy", "check", "statement", "figures")},
        "blocks_closed": {n: blocks.closed(report if n != "GAP MAP" else gap, n)
                          for n in ("REPORT", "LIMITATIONS", "GAP MAP")},
    }


def coverage_of(run: Path) -> Dict[str, Any]:
    """The coverage figures, computed from the ledger — never the report's.

    METHOD §16 gained a `=== COVERAGE ===` block on 2026-08-30 carrying one
    verdict per claim, and every figure here is arithmetic over it. Runs that
    predate the block have their coverage statement inside REPORT and no
    ledger; for those, `ledger` is empty and the report's own sentence is left
    to stand, because recomputing figures from findings is the mistake the
    block exists to remove.
    """
    report = (run / "report.md").read_text(errors="replace") \
        if (run / "report.md").is_file() else ""
    block = blocks.content(report, "COVERAGE", blocks.BLOCKS) or ""
    if not block:
        return {"ledger": [], "legacy": True, "check": None, "prose": ""}
    c = cov.check(claim_surface(run), block)
    # The ledger lines are the record; the prose after them is what the numbers
    # do not say. The client reads the prose, and the ledger goes to Appendix B.
    prose = "\n".join(l for l in block.splitlines()
                       if not cov._LEDGER_LINE.match(l)).strip()
    return {"ledger": cov.parse_ledger(block), "legacy": False,
            "check": c, "prose": prose,
            "statement": cov.statement(c["figures"]), "figures": c["figures"]}


def claim_surface(run: Path) -> str:
    """The frozen claim surface, recovered from the working record.

    THE CLIENT CANNOT READ A CLAIM NUMBER IT WAS NEVER GIVEN. METHOD §16 makes
    `CLAIM SURFACE` one of four delivered blocks, but claims_audit/runner.py
    never writes it to a file — it survives only inside the reasoning trace. So
    a report reaches the client saying "three claims are [delta] (22, 8, 34)",
    and its appendix lists entries numbered by claim, against a surface that is
    not in the envelope. Every one of those numbers indexes nothing.

    Recovering it here delivers what §16 already specifies, without touching the
    audit. It is also the context a coverage statement needs to be read at all:
    "47 of 48 resolved" means little until you can see what the 48 were.
    """
    trace = run / "working_record" / "reasoning_trace.jsonl"
    if not trace.is_file():
        return ""
    for line in trace.read_text(errors="replace").splitlines():
        try:
            reply = json.loads(line).get("raw_response") or ""
        except ValueError:
            continue
        # Anchored by blocks.opened: a leg that MENTIONS the marker mid-sentence
        # is not the leg that emitted it.
        if blocks.opened(reply, "CLAIM SURFACE"):
            return blocks.span(reply, "CLAIM SURFACE", blocks.BLOCKS) or ""
    return ""


def _strip_markers(text: str) -> str:
    """Delivery markers out of a client's document.

    `=== LIMITATIONS ===` is protocol between the agent and the runner. It means
    something to blocks.py and nothing to a buyer, and four of them were sitting
    in the document a client reads.
    """
    return re.sub(r"(?m)^[ \t]*===[^\n]*===[ \t]*$\n?", "", text)


def header(run: Path) -> str:
    """Title, target and date. Every professional document identifies itself.

    Facts from run_meta only — nothing about the engagement is invented here.
    """
    meta = json.loads((run / "run_meta.json").read_text()) \
        if (run / "run_meta.json").is_file() else {}
    # The gap map opens with the target's name in bold — the auditor's own
    # naming, which beats a directory name ("chattermate" for ChatterMate).
    gap = (run / "gap_map.md").read_text(errors="replace") \
        if (run / "gap_map.md").is_file() else ""
    # Two shapes seen: `**ChatterMate** — open-source…` and `**Target:**
    # flowmetrics — small SaaS…`. A bold span ending in a colon is a label, so
    # the name is what follows it, not the label itself — which is how a
    # deliverable came out titled "Technical claims audit — Target:".
    named = re.match(r"\s*(?:===[^\n]*===\s*)?\*\*([^*\n]{2,60})\*\*\s*([^\n—,-]{0,60})",
                     gap)
    target = Path(meta.get("external_repo") or "").name or "the target"
    if named:
        lead, after = named.group(1).strip(), named.group(2).strip()
        target = after or lead if lead.endswith(":") else lead
    when = (meta.get("captured_at_utc") or "")[:10] or "undated"
    return (f"# Technical claims audit — {target}\n\n"
            f"Materials as of {when}. Limited assurance; see Limitations.\n")


# Verdict classes, in the order a reader should meet them: what is not true,
# what is true with a material gap, what could not be settled, what holds with
# a caveat, what holds. §16 asks for highest to lowest consequence and verdict
# is the best proxy available without asking anyone to rank.
_GROUP_ORDER = (("delta",), ("partial",), ("unverifiable",),
                ("real, minor caveat", "real, operational caveat"), ("real",))


def finding_groups(run: Path) -> List[Dict[str, Any]]:
    """The report's findings, grouped by verdict, keyed for SECTION NOTES.

    GROUPED BY VERDICT, NOT BY THE REPORT'S HEADINGS. It read `##` sections
    until 2026-08-31, when a fixture run whose report carried no headings at
    all produced zero groups — the agent's section notes had nothing to attach
    to, no appendix was built, and the reorganisation silently did nothing.
    METHOD §16 never specified markup, so the same method produced a
    heading-structured report on one model and a flat one on another.

    Depending on those headings was wrong twice over: they are unspecified, and
    delivery REPLACES them anyway. Verdict class is deterministic, available
    whatever the model wrote, and is the partition the headings were encoding.

    A key is positional (`group1`, `group2`) rather than named after a verdict,
    so the agent is not handed the audit's vocabulary to echo back.
    """
    report = (run / "report.md").read_text(errors="replace")
    body = blocks.content(report, "REPORT", blocks.REVIEW_BLOCKS) or report
    titles = {int(n): ti.strip() for n, ti, _ in _FINDING_TITLE.findall(report)}
    # THE CLAIM IDS TRAVEL WITH THE FINDING. §5 puts them in the header and
    # this file used to drop them, so the brief below could state a finding
    # count and never a claim count — and the delivery agent, given four
    # findings covering five claims, wrote "Four of the seller's claims are not
    # true as written" under a heading beginning with the word Claims.
    found = [{"n": f["n"], "verdict": f["verdict"],
              "title": titles.get(f["n"], ""), "claims": f["claims"]}
             for f in finding_claims(body)]
    out, i = [], 0
    known = {v for k in _GROUP_ORDER for v in k}
    # A verdict outside §6 still gets a group. Nothing a report states may
    # disappear because this file did not recognise it.
    classes = list(_GROUP_ORDER) + [tuple(sorted({f["verdict"] for f in found}
                                                 - known))]
    for klass in classes:
        members = [f for f in found if f["verdict"] in klass]
        if not members:
            continue
        i += 1
        out.append({"key": f"group{i}",
                    "heading": ", ".join(f"[{k}]" for k in klass),
                    "findings": members})
    return out


def _finding_groups_by_heading(run: Path) -> List[Dict[str, Any]]:
    """Superseded. Kept only so the shape it produced is on the record."""
    report = (run / "report.md").read_text(errors="replace")
    body = blocks.content(report, "REPORT", blocks.REVIEW_BLOCKS) or report
    titles = {int(n): ti.strip() for n, ti, _ in _FINDING_TITLE.findall(report)}
    out, i = [], 0
    for heading, text in sections(body):
        if is_inventory(text) or not heading:
            continue
        found = [{"n": int(n), "verdict": v.strip(), "title": titles.get(int(n), "")}
                 for n, v in _FINDING.findall(text)]
        # A finding group carries findings. The unverifiable section carries
        # none in §5's format but opens its line with a claim, which is the
        # shape that separates it from the coverage statement — where a verdict
        # appears mid-sentence and no line begins with one.
        if not found and not re.search(r"(?m)^\s*\**\s*Claim\s+\d+", text):
            continue
        i += 1
        out.append({"key": f"group{i}", "heading": heading.lstrip("# ").strip(),
                    "findings": found})
    return out


def _all_finding_blocks(text: str) -> List[Tuple[int, str]]:
    """Each §5 finding block, from its header to the next one.

    A TRAILING HEADING BELONGS TO THE NEXT GROUP, NOT THIS FINDING. Where the
    auditor wrote `##` headings they sit BETWEEN findings, so they land at the
    end of the preceding block — and re-emitting blocks under the delivery
    agent's headings then interleaved both sets, leaving the audit's orphaned
    in the body. Findings are regrouped by verdict here, so a heading that only
    labelled the old grouping has nothing left to label.
    """
    cuts = [(int(m.group(1)), m.start()) for m in _FINDING.finditer(text)]
    out = []
    for i, (n, s) in enumerate(cuts):
        e = cuts[i + 1][1] if i + 1 < len(cuts) else len(text)
        seg = text[s:e]
        # A finding ends at the first heading after its own header. What
        # follows is somebody else's — another group's label, or a section like
        # the unverifiable claim, which is report content and keeps its place.
        h = re.search(r"(?m)^#{2,3} ", seg)
        out.append((n, seg[:h.start()] if h else seg))
    return out


def _interstitial(text: str) -> str:
    """Report content that sits between findings and is not one.

    On a heading-structured report this is where the `[unverifiable]` claim
    lives: a section, not a §5 finding, so nothing collects it as one. It must
    still reach the client, and it must not be swallowed into the finding above
    it — which is what happened until the block boundary moved to the first
    heading.
    """
    parts, cuts = [], [(int(m.group(1)), m.start())
                       for m in _FINDING.finditer(text)]
    for i, (_, s) in enumerate(cuts):
        e = cuts[i + 1][1] if i + 1 < len(cuts) else len(text)
        seg = text[s:e]
        h = re.search(r"(?m)^#{2,3} ", seg)
        if not h:
            continue
        tail = seg[h.start():].rstrip()
        # A heading with nothing under it only labelled the old grouping and
        # has nothing left to label. A heading with a body — the unverifiable
        # claim — is content and keeps its heading.
        body = "\n".join(tail.splitlines()[1:]).strip()
        if body:
            # A kept segment can end on the NEXT group's bare label, since the
            # block runs to the following finding. Same rule at the other end.
            parts.append(re.sub(r"(?m)\n#{2,3} .*$\s*\Z", "", tail).rstrip())
    return "\n\n".join(p for p in parts if p.strip())


def _finding_block(text: str, n: int) -> str:
    for num, blk in _all_finding_blocks(text):
        if num == n:
            return blk
    return ""


# The two appendix labels this script emits. A CLOSED SET THIS FILE OWNS, so
# looking for them in the agent's prose is a token check over its own
# vocabulary — the same standing as blocks.py looking for `=== REPORT ===` —
# and not a keyword read of what the sentence means.
_APPENDIX = re.compile(r"\bAppendix\s+([AB])\b")


def promised_appendices(written: Dict[str, Any], emitted: set) -> List[str]:
    """Appendices the agent's prose names that the document will not carry.

    DELIVERY.md §6 tells the cover to name the appendices the brief lists and
    to promise no others, and §7's assembly list makes Appendix A conditional
    on the report having carried an inventory to move. On the grok fixture
    delivery of 2026-08-31 the brief named only Appendix B, the document
    carried only Appendix B, and the cover promised Appendix A anyway — the
    second run to do so after that instruction was rewritten twice.

    So this is not a wording defect and a third rewrite will not settle it.
    The script already knows which appendices it is about to emit; a client
    receiving a document that promises one it does not contain is a defect a
    person must see, and it costs one comparison to see it.
    """
    prose = " ".join(str(v) for k, v in (written or {}).items() if k != "sections")
    for _h, body in ((written or {}).get("sections") or {}).values():
        prose += " " + str(body)
    return sorted({f"Appendix {m}" for m in _APPENDIX.findall(prose)} - emitted)


def assemble(run: Path, written: Optional[Dict[str, Any]] = None) -> str:
    """The report with its inventory moved to an appendix. Nothing reworded.

    The only structural change: a section that is an inventory by shape moves
    below the narrative, keeping its heading and its contents verbatim. On the
    ChatterMate report that moves 35 of 47 entries out of the body, which is
    what stops the document reading as tool output.
    """
    report = (run / "report.md").read_text(errors="replace")
    body = blocks.content(report, "REPORT", blocks.REVIEW_BLOCKS) or report
    limits = blocks.span(report, "LIMITATIONS", blocks.BLOCKS) or ""

    written = written or {}
    gap = (run / "gap_map.md").read_text(errors="replace") \
        if (run / "gap_map.md").is_file() else ""

    # THE REPORT IS FINDINGS PLUS WHAT SURROUNDS THEM, not a set of sections.
    # §16 gives the report three parts and specifies no markup, so one model
    # writes `##` headings and another writes none. Findings are located by
    # their own §5 header, which every report has by construction; whatever
    # precedes the first is the conclusion, whatever follows the last is the
    # questions. Nothing depends on a heading the auditor may not have written.
    cuts = [m.start() for m in _FINDING.finditer(body)]
    if cuts:
        # Whatever precedes the first finding is the conclusion. A heading on
        # its last line only labelled the findings that follow, and those are
        # regrouped, so it goes.
        head = re.sub(r"(?m)^#{2,3} .*$\s*\Z", "", body[:cuts[0]]).rstrip()
        # The findings end where the next heading or inventory line begins.
        after = body[cuts[-1]:]
        stop = re.search(r"(?m)^#{2,3} |^\s*\d+\.\s+(?!\*)[^\n\[]{3,}?\s*[—-]\s*\[",
                         after[1:])
        body_findings = body[cuts[0]:cuts[-1]] + (
            after[:1 + stop.start()] if stop else after)
        rest = after[1 + stop.start():] if stop else ""
    else:
        head, body_findings, rest = body, "", ""

    # The remainder still has its own sections — coverage, questions — and one
    # of them may be the supported inventory. Split THAT, not the whole body:
    # slicing the document at headings is what broke when a report had none.
    keep, moved = [], []
    for heading, text in sections(rest):
        # A moved heading is demoted one level so it nests under the appendix
        # rather than sitting beside it. Depth is layout; the text is untouched.
        if is_inventory(text):
            moved.append((("#" + heading + "\n") if heading else "") + text)
        else:
            keep.append((heading + "\n" if heading else "") + text)

    notes = written.get("sections") or {}
    groups = finding_groups(run) if notes else []
    out = [header(run)]
    if gap:
        out.append("\n\n## Summary\n")
        out.append(_strip_markers(gap).strip())
    if written.get("cover"):
        out.append("\n\n## About this audit\n")
        out.append(written["cover"].strip())
    if head.strip():
        # The label moves INTO the heading rather than being printed twice, so
        # the verdict is more prominent and the sentence after it is untouched.
        m = re.match(r"\s*\*\*Conclusion:\s*([^.*]{2,40})\.?\*\*\s*", head)
        if m:
            out.append(f"\n\n## Conclusion — {m.group(1).strip()}\n\n"
                       + head[m.end():].strip())
        else:
            out.append("\n\n## Conclusion\n\n" + head.strip())
    out.append("\n\n" + how_to_read().rstrip())

    if groups:
        placed = set()
        for g in groups:
            n = notes.get(g["key"])
            heading = ("## " + n[0]) if n and n[0] else f"## {g['heading']}"
            out.append("\n\n" + heading + "\n")
            if n and n[1]:
                out.append(n[1].strip() + "\n")
            for f in g["findings"]:
                blk = _finding_block(body_findings, f["n"])
                if blk:
                    out.append("\n" + blk.strip() + "\n")
                    placed.add(f["n"])
        # Anything the grouping did not place still ships, unheaded.
        leftover = [b for n, b in _all_finding_blocks(body_findings)
                    if n not in placed]
        if leftover:
            out.append("\n\n" + "\n\n".join(b.strip() for b in leftover))
        between = _interstitial(body_findings)
        if between.strip():
            out.append("\n\n" + between.strip())
    elif body_findings.strip():
        out.append("\n\n" + body_findings.strip())
    for seg in keep:
        if seg.strip():
            out.append("\n\n" + seg.strip())

    cv = coverage_of(run)
    if not cv["legacy"]:
        # The figures are computed; the agent's prose, if any, sits under them.
        body_text = (written.get("coverage") or cv["prose"]).strip()
        out.append("\n\n## Coverage\n\n" + cv["statement"]
                   + (("\n\n" + body_text) if body_text else ""))
    elif written.get("coverage"):
        out = [_swap_coverage(s, written["coverage"]) for s in out]
    if limits:
        out.append("\n\n## Limitations\n")
        out.append(_strip_markers(limits).strip())

    if moved:
        out.append("\n\n## Appendix A — supported claims, with citations\n")
        out.append("Every remaining resolved claim, with the evidence that "
                   "resolves it. These are listed rather than written up "
                   "because none of them is a gap.\n")
        out.append("\n".join(seg.strip() for seg in moved).rstrip())
    surface = claim_surface(run)
    if surface:
        out.append("\n\n## Appendix B — the claim surface\n")
        out.append("Every assertion identified in the claim sources and frozen "
                   "before verification began. The claim numbers used above, "
                   "and in the Coverage section, index this list.\n")
        out.append(_strip_markers(surface).strip())
        if cv["ledger"]:
            out.append("\n\nThe verdict each claim received:\n")
            out.append("\n".join(f"{n}. [{v}]" for n, v in cv["ledger"]))
    # Sections are split at their headings, so each body already begins with
    # the newline that ended the heading line — which printed two blank lines
    # under every `##`. Layout only; no character of content moves.
    return re.sub(r"\n{3,}", "\n\n", "\n".join(out)).strip() + "\n"


def _swap_coverage(chunk: str, prose: str) -> str:
    """Replace the audit's coverage paragraph with the agent's version of it.

    The figures are the audit's either way — DELIVERY.md forbids recalculating
    — so this swaps presentation, not facts. Matched on the heading the audit
    wrote; left alone if that heading is not in this chunk.
    """
    m = re.search(r"(?m)^## Coverage[^\n]*\n", chunk)
    if not m:
        return chunk
    end = re.search(r"(?m)^## ", chunk[m.end():])
    tail = chunk[m.end() + end.start():] if end else ""
    return chunk[:m.start()] + "## Coverage\n\n" + prose.strip() + "\n\n" + tail


def editor_notes(run: Path, a: Dict[str, Any]) -> str:
    """What a person must decide, and nothing this script could have decided."""
    L: List[str] = [
        f"# Editor notes — {run.name}",
        "",
        "Produced by workflowsv2/audit_postprocess/deliver.py. Every item below "
        "needs a human decision; none of them is a defect this script is "
        "entitled to settle.",
        "",
        "## 1. Citations that did not resolve",
        "",
    ]
    if not a["citations_unresolved"]:
        L.append(f"None. All {a['citations_total']} citations resolved against "
                 "the materials.")
    else:
        L += [f"{len(a['citations_unresolved'])} of {a['citations_total']} "
              "citations did not resolve. METHOD §12 step 6b says a finding "
              "with an invalid citation does not ship — but the index resolves "
              "only `docN:NN` references and quoted spans, and it can resolve a "
              "name to the wrong file, so **check each against the materials "
              "before acting**. This script does not repair citations.", ""]
        seen = set(a["review"].get("flagged_citations_mentioned") or [])
        for c in a["citations_unresolved"]:
            note = ("an independent review examined this one — read its finding "
                    "before deciding" if c.get("cited") in seen else
                    "not mentioned by any review" if a["review"].get("ran") else
                    "no review has been run on this audit")
            L.append(f"- `{c.get('cited')}` — {c.get('why','did not resolve')} "
                     f"({note})")
    r = a["review"]
    if r.get("ran"):
        L += ["", "## 1b. What the independent review said", "",
              (f"**INADMISSIBLE — the reviewer could not establish what the "
               f"report's citations refer to. Do not deliver.**" if r["inadmissible"]
               else f"Admissible; {r['exceptions_raised']} exception block(s) "
                    f"raised. A finding a reviewer says does not hold should not "
                    f"ship as though it does — check the review before signing "
                    f"off." if r["exceptions_raised"]
               else "Admissible, no exceptions raised. Note that a review "
                    "finding nothing is the least discriminating outcome "
                    "available, not a guarantee.")]
    else:
        L += ["", "## 1b. Independent review", "",
              "No review has been run on this audit. Every citation flag above "
              "is unexamined."]
    if a["parked_notes"]:
        L += ["", "## 1c. Observations parked in an appendix", "",
              "The auditor wrote these into the claim surface rather than as "
              "findings, so they sit at the end of the last appendix where a "
              "client will not reach them. Decide whether each belongs in the "
              "body — this script will not promote an observation into a "
              "finding.", ""]
        L += [f"- {n}" for n in a["parked_notes"]]
    L += ["", "## 1d. Everything this run recorded for a person", "",
          iss.render(run), ""]
    cvk = (a.get("coverage") or {}).get("check")
    if cvk and not cvk.get("ok"):
        L += ["## 1e. The coverage ledger does not account for the surface", "",
              "Every coverage figure is computed from the ledger, so a ledger "
              "that does not cover the frozen surface exactly once makes them "
              "wrong. This is the report's assurance figure. Do not ship it "
              "until the ledger is repaired — and repair the ledger, not the "
              "figures.", ""]
        L += [f"- {p}" for p in cvk["problems"]]
    L += ["", "## 2. Consequence ordering", "",
          "The audit orders findings by verdict class, which is not the same as "
          "consequence to this buyer — and consequence to this buyer is not "
          "derivable from the materials, so neither the audit nor this script "
          "can rank it. Re-order the findings below if the client's priorities "
          "differ from the order the verdicts imply.", ""]
    for f in a["findings"]:
        L.append(f"- Finding {f['n']} — [{f['verdict']}] — {f['title']}")
    L += ["", "## 3. Claim numbers", ""]
    if a.get("appendices_promised_not_emitted"):
        L += ["", "## 1e. The cover promises an appendix the document does not carry",
              "",
              "The deliverable names " + ", ".join(a["appendices_promised_not_emitted"])
              + " in its prose and does not contain it. Either strike the "
                "promise or supply the appendix; a client counting the "
                "appendices will find one missing.", ""]
    if a["claim_surface_recovered"]:
        L.append("The claim surface is appended to the deliverable, so every "
                 "claim number in the coverage block and the supported "
                 "appendix resolves. Nothing to do.")
    else:
        L.append("**The claim surface could not be recovered from the working "
                 "record, so no claim number in this report resolves for the "
                 "reader.** The coverage block cites claims by number and "
                 "the supported appendix is keyed by them. Recover it before "
                 "delivery, or strike the numbers.")
    L += ["", f"The supported appendix holds {len(a['inventory_entries'])} "
              "entries, keyed by claim number rather than finding number.",
          "", "## 4. Structure", "",
          f"- report as delivered: {a['report_chars']:,} characters",
          f"- narrative findings: {len(a['findings'])}",
          f"- appendix entries: {len(a['inventory_entries'])}",
          f"- gap map present: {a['gap_map_present']} (delivered separately)",
          f"- blocks closed: {a['blocks_closed']}",
          ""]
    return "\n".join(L)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--run", type=Path, required=True,
                    help="a finished claims_audit run directory")
    ap.add_argument("--out", type=Path, default=None,
                    help="output directory (default: <run>/delivery)")
    args = ap.parse_args()
    run = args.run.resolve()
    if not (run / "report.md").is_file():
        raise SystemExit(f"{run}: no report.md — not a finished audit run")

    a = audit(run)
    out = (args.out or run / "delivery").resolve()
    out.mkdir(parents=True, exist_ok=True)
    (out / "deliverable.md").write_text(assemble(run))
    (out / "editor_notes.md").write_text(editor_notes(run, a))
    (out / "checks.json").write_text(json.dumps(a, indent=2))

    n = len(a["citations_unresolved"])
    print(f"{run.name}")
    print(f"  citations        {a['citations_total']} resolved, {n} NOT RESOLVED"
          if n else f"  citations        {a['citations_total']}, all resolved")
    for c in a["citations_unresolved"]:
        print(f"                   {c.get('cited')} — {c.get('why','')}")
    print(f"  findings         {len(a['findings'])} narrative, "
          f"{len(a['inventory_entries'])} moved to appendix")
    print(f"  claim ids        {len(a['findings']) - len(a['findings_without_claim_id'])}"
          f" of {len(a['findings'])} findings carry one")
    print(f"  -> {out}/deliverable.md")
    print(f"  -> {out}/editor_notes.md")
    return 1 if n else 0


if __name__ == "__main__":
    raise SystemExit(main())
