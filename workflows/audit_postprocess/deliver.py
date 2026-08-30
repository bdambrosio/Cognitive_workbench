#!/usr/bin/env python3
"""Turn a finished audit run into a client deliverable and an editor's worklist.

    python3 workflows/audit_postprocess/deliver.py --run <audit run directory>

WHAT THIS IS. The audit produces evidence; this produces a document. It
REORGANISES AND CHECKS. It does not re-judge: no finding is added, removed,
reworded or re-verdicted, and every citation passes through byte-for-byte.

NO MODEL. Bruce's call, 2026-08-30. Everything here is either a file operation
or markdown surgery over a format METHOD §16 already specifies, and the one
judgement anyone wanted — ranking findings by consequence — is going to a human
editor because it is not derivable from the materials at all. The audit cannot
know whether this buyer weighs security posture above feature completeness.

WHY THE CHECKS ARE THE POINT, and the reformatting is the bonus. METHOD §12
step 6b says "a finding with a missing or invalid citation does not ship". On
2026-08-30 a ChatterMate report shipped three citations naming lines past the
end of their files — ticket_investigator.py:628-635 in a 398-line file,
ticket.py:755-767 in 287, package.json:17-18 in 12. The audit's own check
missed them and the independent review reported none. For a document whose
whole value is "every claim is checkable against cited evidence", a citation
that resolves to nothing is the most damaging defect available: it discredits
the findings that are right.

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

from workflows import blocks                                    # noqa: E402
from workflows.citations import resolve_citations               # noqa: E402

# A §5 finding header. Same shape audit_review and score.py both key on, so a
# report that parses for them parses here.
_FINDING = re.compile(r"^\s*\**\s*Finding\s+(\d+)[^\n\[]*\[([^\]]+)\]",
                      re.M | re.I)
# The claim id §5 will carry once the header change lands: `Finding 1 (claim 22)`.
# Absent today, which is itself an editor note rather than an error.
_CLAIM_ID = re.compile(r"^\s*\**\s*Finding\s+(\d+)\s*\(claims?\s+([^)]+)\)",
                       re.M | re.I)
# An inventory line: a bare ordinal, a title, a verdict, then citations. This is
# the 35-line block that made up the bulk of the ChatterMate report body.
_INVENTORY = re.compile(r"^\s*(\d+)\.\s+(?!\*)([^\n\[]{3,}?)\s*[—-]\s*\[([^\]]+)\]",
                        re.M)
# The same header, with the title captured. Kept separate from `_FINDING` so
# that pattern stays byte-identical to the one audit_review and score.py use —
# an editor ranking findings needs titles, a parser checking verdicts does not.
_FINDING_TITLE = re.compile(
    r"^\s*\**\s*Finding\s+(\d+)\s*(?:\(claims?[^)]*\))?\s*:?\s*"
    r"(.*?)\s*[—-]\s*\[([^\]]+)\]", re.M)
_H2 = re.compile(r"^## .*$", re.M)


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
    findings = [{"n": int(n), "verdict": v.strip(), "title": titles.get(int(n), "")}
                for n, v in _FINDING.findall(report)]
    with_claim = {int(n): c.strip() for n, c in _CLAIM_ID.findall(report)}
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
                                      if f["n"] not in with_claim],
        "inventory_entries": inv,
        "claim_surface_recovered": bool(claim_surface(run)),
        "parked_notes": parked_notes(claim_surface(run)),
        "review": review_signal(run, unresolved),
        "blocks_closed": {n: blocks.closed(report if n != "GAP MAP" else gap, n)
                          for n in ("REPORT", "LIMITATIONS", "GAP MAP")},
    }


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
    named = re.match(r"\s*(?:===[^\n]*===\s*)?\*\*([^*\n]{2,60})\*\*", gap)
    target = (named.group(1).strip() if named
              else Path(meta.get("external_repo") or "").name or "the target")
    when = (meta.get("captured_at_utc") or "")[:10] or "undated"
    return (f"# Technical claims audit — {target}\n\n"
            f"Materials as of {when}. Limited assurance; see Limitations.\n")


def assemble(run: Path) -> str:
    """The report with its inventory moved to an appendix. Nothing reworded.

    The only structural change: a section that is an inventory by shape moves
    below the narrative, keeping its heading and its contents verbatim. On the
    ChatterMate report that moves 35 of 47 entries out of the body, which is
    what stops the document reading as tool output.
    """
    report = (run / "report.md").read_text(errors="replace")
    body = blocks.content(report, "REPORT", blocks.REVIEW_BLOCKS) or report
    limits = blocks.span(report, "LIMITATIONS", blocks.BLOCKS) or ""

    keep, moved = [], []
    for heading, text in sections(body):
        (moved if is_inventory(text) else keep).append((heading, text))

    # The conclusion sits above the first heading, so it is in no contents list
    # and does not look like the thing the document exists to say. Where the
    # auditor already opened with `**Conclusion: X.**` the label moves INTO the
    # heading rather than being printed twice — the verdict ends up more
    # prominent, not less, and the sentence after it is untouched. Strictly
    # matched, and left alone if it does not match.
    if keep and not keep[0][0]:
        lead = keep[0][1]
        m = re.match(r"\s*\*\*Conclusion:\s*([^.*]{2,40})\.?\*\*\s*", lead)
        if m:
            keep[0] = (f"## Conclusion — {m.group(1).strip()}",
                       "\n" + lead[m.end():].lstrip())
        else:
            keep[0] = ("## Conclusion", lead)

    out = [header(run), "".join(h + "\n" + t for h, t in keep).rstrip()]

    # LIMITATIONS TRAVELS WITH THE REPORT, NOT AFTER THE APPENDICES. blocks.py
    # keeps them separate blocks so nothing has to nest, and says why they are
    # one document: ISAE 3000 / AT-C 205 require the limitations to travel with
    # the report. A reader stops at the appendices.
    if limits:
        out.append("\n\n## Limitations\n")
        out.append(_strip_markers(limits).strip())

    if moved:
        out.append("\n\n## Appendix A — supported claims, with citations\n")
        out.append("Every remaining resolved claim, with the evidence that "
                   "resolves it. These are listed rather than written up "
                   "because none of them is a gap.\n")
        # Demoted one level so a moved heading nests UNDER the appendix
        # rather than sitting beside it. Heading depth is layout; the heading
        # text and everything below it are untouched.
        out.append("".join("#" + h + "\n" + t for h, t in moved).rstrip())
    surface = claim_surface(run)
    if surface:
        out.append("\n\n## Appendix B — the claim surface\n")
        out.append("Every assertion identified in the claim sources and frozen "
                   "before verification began. The claim numbers used above, "
                   "and in the coverage statement, index this list.\n")
        out.append(_strip_markers(surface).strip())
    # Sections are split at their headings, so each body already begins with
    # the newline that ended the heading line — which printed two blank lines
    # under every `##`. Layout only; no character of content moves.
    return re.sub(r"\n{3,}", "\n\n", "\n".join(out)).strip() + "\n"


def editor_notes(run: Path, a: Dict[str, Any]) -> str:
    """What a person must decide, and nothing this script could have decided."""
    L: List[str] = [
        f"# Editor notes — {run.name}",
        "",
        "Produced by workflows/audit_postprocess/deliver.py. Every item below "
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
    L += ["", "## 2. Consequence ordering", "",
          "The audit orders findings by verdict class, which is not the same as "
          "consequence to this buyer — and consequence to this buyer is not "
          "derivable from the materials, so neither the audit nor this script "
          "can rank it. Re-order the findings below if the client's priorities "
          "differ from the order the verdicts imply.", ""]
    for f in a["findings"]:
        L.append(f"- Finding {f['n']} — [{f['verdict']}] — {f['title']}")
    L += ["", "## 3. Claim numbers", ""]
    if a["claim_surface_recovered"]:
        L.append("The claim surface is appended to the deliverable, so every "
                 "claim number in the coverage statement and the supported "
                 "appendix resolves. Nothing to do.")
    else:
        L.append("**The claim surface could not be recovered from the working "
                 "record, so no claim number in this report resolves for the "
                 "reader.** The coverage statement cites claims by number and "
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
