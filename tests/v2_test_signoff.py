"""The release record and the signature, kept beside a report.

    python3 -m pytest tests/v2_test_signoff.py -q
"""
import hashlib
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from workflowsv2 import issues                                      # noqa: E402
from workflowsv2.audit_report import signoff                        # noqa: E402


def _merged(tmp_path):
    run = tmp_path / "run1"
    (run / "review").mkdir(parents=True)
    merged = tmp_path / "merged"
    merged.mkdir()
    (merged / "report.md").write_text("# Report\n\nBody.\n")
    (merged / "merged.json").write_text(json.dumps({"runs": [{"dir": str(run), "claim_source": "README.md"}]}))
    issues.note(run, "claims_audit", "surface_check", "a quote is elsewhere", severity="blocking")
    issues.note(run / "review", "audit_review", "adverse_recall", "something adverse", severity="check")
    issues.note(merged, "audit_materiality", "borderline_rating", "split", severity="check")
    return merged


def test_release_records_who_what_and_the_open_worklist_and_adds_nothing_to_the_report(tmp_path):
    merged = _merged(tmp_path)
    rec = signoff.release(merged, "bruce@tuuyi.com")
    assert rec["by"] == "bruce@tuuyi.com" and rec["run"] == "merged"
    assert rec["worklist"] == {"blocking": 1, "check": 2, "note": 0}     # counted where the worklist is gathered
    assert rec["report_sha256"] == hashlib.sha256(b"# Report\n\nBody.\n").hexdigest()
    assert signoff.load_release(merged) == rec and signoff.load_signature(merged) is None
    assert "statement" not in rec and "attestation" not in rec         # a released report carries no statement
    assert not (merged / "report.html").exists() and (merged / "report.md").read_text() == "# Report\n\nBody.\n"


def test_sign_records_the_attestation_and_renders_it_at_the_end_of_the_report(tmp_path):
    merged = _merged(tmp_path)
    rec = signoff.sign(merged, "Bruce D'Ambrosio")
    assert rec["attestation"] == signoff.ATTESTATION and rec["by"] == "Bruce D'Ambrosio"
    assert signoff.load_signature(merged) == rec and signoff.load_release(merged) is None   # the two are independent
    block = signoff.block_md(rec)
    assert "Signed by Bruce D'Ambrosio" in block and "I have read every finding" in block and rec["report_sha256"] in block
    assert (merged / "report.md").read_text() == "# Report\n\nBody.\n"    # the text that was read is not changed
    assert "Signature" in (merged / "report.html").read_text()


def test_no_report_or_no_person_is_refused(tmp_path):
    for f in (signoff.release, signoff.sign):
        with pytest.raises(SystemExit):
            f(tmp_path, "bruce@tuuyi.com")
    (tmp_path / "report.md").write_text("# r\n")
    for f in (signoff.release, signoff.sign):
        with pytest.raises(SystemExit):
            f(tmp_path, "  ")
    assert signoff.load_release(tmp_path) is None and signoff.load_signature(tmp_path) is None
