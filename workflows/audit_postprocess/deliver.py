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

    out = ["".join(h + "\n" + t for h, t in keep).rstrip()]
    if moved:
        out.append("\n\n## Appendix — supported claims, with citations\n")
        out.append("Every remaining resolved claim, with the evidence that "
                   "resolves it. These are listed rather than written up "
                   "because none of them is a gap.\n")
        # Demoted one level so a moved heading nests UNDER the appendix
        # rather than sitting beside it. Heading depth is layout; the heading
        # text and everything below it are untouched.
        out.append("".join("#" + h + "\n" + t for h, t in moved).rstrip())
    surface = claim_surface(run)
    if surface:
        out.append("\n\n## Appendix — the claim surface\n")
        out.append("Every assertion identified in the claim sources and frozen "
                   "before verification began. The claim numbers used above, "
                   "and in the coverage statement, index this list.\n")
        out.append(surface.strip())
    if limits:
        out.append("\n\n" + limits.strip())
    return "\n".join(out) + "\n"


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
        for c in a["citations_unresolved"]:
            L.append(f"- `{c.get('cited')}` — {c.get('why', 'did not resolve')}")
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
