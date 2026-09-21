"""The record that a person read a report and released it.

The site says a person confirms and signs every report. Until 2026-09-20 the
only trace of that was the `release` stage mark in state.json, which names no
report: the practice's own claims review of its site found the statement
contradicted, because nothing in the delivered report or beside it said who
stood behind it.

`sign` is called by the release step. It writes `signoff.json` beside the
report: who released it, when, the statement they agreed to, the SHA-256 of
the `report.md` they released, and how many worklist items of each severity
stood open when they did. report.md is not changed: it is the text that was
read, and the hash says which text. The printable report (report.html and
report.pdf) is rendered again with the sign-off as its last section, and the
client's report page shows the same block.

The statement is what the site says the person does (how-it-works, "A person
confirms and signs"). Releasing is signing: there is no separate step.
"""
from __future__ import annotations

import hashlib
import json
import logging
from pathlib import Path
from typing import Any, Dict, Optional

from utils.file_utils import atomic_write_text
from workflowsv2 import engagement_state as state
from workflowsv2 import issues

logger = logging.getLogger("signoff")

FILENAME = "signoff.json"
STATEMENT = ("I have read this report, the ratings marked borderline, the citations the check "
             "flagged and the findings the independent check questioned. I release it to the "
             "client and answer for it on behalf of the practice.")


def load(merged_dir: Path) -> Optional[Dict[str, Any]]:
    p = Path(merged_dir) / FILENAME
    return json.loads(p.read_text(encoding="utf-8")) if p.is_file() else None


def block_md(rec: Dict[str, Any]) -> str:
    """The sign-off as the last section of the report a reader sees."""
    open_items = ", ".join(f"{n} {sev}" for sev, n in (rec.get("worklist") or {}).items() if n) or "none"
    return ("\n\n## Sign-off\n\n"
            f"Read and released by {rec['by']} on {rec['at'][:10]}.\n\n"
            f"> {rec['statement']}\n\n"
            f"Worklist items open at release: {open_items}. "
            f"The report released is the text whose SHA-256 is `{rec['report_sha256']}`.\n")


def worklist_counts(merged_dir: Path) -> Dict[str, int]:
    """How many worklist items of each severity stand against this report,
    gathered from the places worklist.md is gathered from (render.worklist):
    each run directory's issues, its review's, and the merged directory's."""
    merged = json.loads((merged_dir / "merged.json").read_text(encoding="utf-8")) \
        if (merged_dir / "merged.json").is_file() else {}
    places = [merged_dir]
    for r in merged.get("runs") or []:
        d = Path(r.get("dir") or "")
        places += [d, d / "review"]
    counts = {sev: 0 for sev in issues.SEVERITIES}
    for place in places:
        for row in issues.read(place):
            counts[row.get("severity") if row.get("severity") in counts else "check"] += 1
    return counts


def sign(merged_dir: Path, by: str) -> Dict[str, Any]:
    """Write the sign-off for the report in `merged_dir` and render the
    printable report with it. Raises SystemExit when there is no report or no
    named person: an unsigned release is what this exists to prevent."""
    merged_dir = Path(merged_dir)
    report = merged_dir / "report.md"
    if not report.is_file():
        raise SystemExit(f"{merged_dir.name} has no report.md to sign")
    if not (by or "").strip():
        raise SystemExit("a sign-off needs the person's name or address")
    text = report.read_text(encoding="utf-8")
    counts = worklist_counts(merged_dir)
    rec = {"by": by.strip(), "at": state.stamp(), "run": merged_dir.name, "statement": STATEMENT,
           "report_sha256": hashlib.sha256(text.encode("utf-8")).hexdigest(), "worklist": counts}
    atomic_write_text(merged_dir / FILENAME, json.dumps(rec, indent=1, ensure_ascii=False) + "\n")
    try:
        from workflowsv2.audit_report import printable
        html = merged_dir / "report.html"
        atomic_write_text(html, printable.to_html(text + block_md(rec)))
        printable.to_pdf(html)
    except Exception as e:                                     # noqa: BLE001
        # The record is written; the printable copy is a rendering of it.
        logger.warning("sign-off recorded, but the printable report was not rendered again: %s", e)
        issues.note(merged_dir, "signoff", "printable_not_rendered",
                    f"the printable report does not carry the sign-off: {e}")
    return rec
