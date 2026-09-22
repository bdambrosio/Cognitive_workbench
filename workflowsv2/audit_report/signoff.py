"""Two records a person makes about a report, kept beside it.

**Release** is the practice making the report visible to the client. It is
the default step of every engagement. `release` writes `release.json`: who
released the report, when, the SHA-256 of the `report.md` released, and how
many worklist items of each severity stood open. Nothing is added to the
report: a released report carries no statement.

**Signing** is separate, and is not part of the default engagement. It is the
practice's expert reading the whole record and putting their name to the
attestation the site's About page gives. `sign` writes `signature.json` with
the signer's name, the attestation, the time and the report's hash, and
renders the printable report (report.html and report.pdf) again with the
signature as its last section; the client's report page shows the same
block. report.md is not changed by either: it is the text that was read, and
the hash says which text.

Until 2026-09-21 releasing wrote a sign-off in the releaser's name. Bruce:
signing is an extra step at extra cost, and means he personally reviewed the
report; releasing means only that the practice let the client see it.
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

RELEASE = "release.json"
SIGNATURE = "signature.json"
#: The attestation on the site's About page (site/about.html, "Attested due
#: diligence"). The two are kept the same by hand.
ATTESTATION = ("I have read every finding and the record behind it. Each finding cites evidence that "
               "exists as quoted and supports its verdict. Every claim in the documents named at intake "
               "was listed, every claim rated as bearing on the decision received a finding, and what the "
               "materials could not settle is reported as unsettled, not as false. I do not attest to the "
               "truth of unsettled claims, to the seller's conduct, to the target's value, or to anything "
               "outside the materials supplied, and nothing here is legal advice.")


def _load(merged_dir: Path, name: str) -> Optional[Dict[str, Any]]:
    p = Path(merged_dir) / name
    return json.loads(p.read_text(encoding="utf-8")) if p.is_file() else None


def load_release(merged_dir: Path) -> Optional[Dict[str, Any]]:
    return _load(merged_dir, RELEASE)


def load_signature(merged_dir: Path) -> Optional[Dict[str, Any]]:
    return _load(merged_dir, SIGNATURE)


def block_md(rec: Dict[str, Any]) -> str:
    """The signature as the last section of the report a reader sees."""
    return ("\n\n## Signature\n\n"
            f"Signed by {rec['by']} on {rec['at'][:10]}.\n\n"
            f"> {rec['attestation']}\n\n"
            f"The report signed is the text whose SHA-256 is `{rec['report_sha256']}`.\n")


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


def _report_text(merged_dir: Path, what: str, by: str) -> str:
    report = Path(merged_dir) / "report.md"
    if not report.is_file():
        raise SystemExit(f"{Path(merged_dir).name} has no report.md to {what}")
    if not (by or "").strip():
        raise SystemExit(f"a {what} needs the person's name or address")
    return report.read_text(encoding="utf-8")


def release(merged_dir: Path, by: str) -> Dict[str, Any]:
    """Record that `by` released the report in `merged_dir` to the client.
    Raises SystemExit when there is no report or no named person."""
    merged_dir = Path(merged_dir)
    text = _report_text(merged_dir, "release", by)
    rec = {"by": by.strip(), "at": state.stamp(), "run": merged_dir.name,
           "report_sha256": hashlib.sha256(text.encode("utf-8")).hexdigest(),
           "worklist": worklist_counts(merged_dir)}
    atomic_write_text(merged_dir / RELEASE, json.dumps(rec, indent=1, ensure_ascii=False) + "\n")
    return rec


def sign(merged_dir: Path, by: str) -> Dict[str, Any]:
    """Record that `by` signed the attestation for the report in `merged_dir`,
    and render the printable report with the signature. Raises SystemExit
    when there is no report or no named person."""
    merged_dir = Path(merged_dir)
    text = _report_text(merged_dir, "signature", by)
    rec = {"by": by.strip(), "at": state.stamp(), "run": merged_dir.name, "attestation": ATTESTATION,
           "report_sha256": hashlib.sha256(text.encode("utf-8")).hexdigest()}
    atomic_write_text(merged_dir / SIGNATURE, json.dumps(rec, indent=1, ensure_ascii=False) + "\n")
    try:
        from workflowsv2.audit_report import printable
        html = merged_dir / "report.html"
        atomic_write_text(html, printable.to_html(text + block_md(rec)))
        printable.to_pdf(html)
    except Exception as e:                                     # noqa: BLE001
        # The record is written; the printable copy is a rendering of it.
        logger.warning("signature recorded, but the printable report was not rendered again: %s", e)
        issues.note(merged_dir, "signoff", "printable_not_rendered",
                    f"the printable report does not carry the signature: {e}")
    return rec
