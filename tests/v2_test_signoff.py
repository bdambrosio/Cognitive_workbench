"""The sign-off written when a report is released.

    python3 -m pytest tests/v2_test_signoff.py -q
"""
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from workflowsv2 import issues                                      # noqa: E402
from workflowsv2.audit_report import signoff                        # noqa: E402


def test_sign_counts_the_worklist_where_the_worklist_is_gathered(tmp_path):
    run = tmp_path / "run1"
    (run / "review").mkdir(parents=True)
    merged = tmp_path / "merged"
    merged.mkdir()
    (merged / "report.md").write_text("# Report\n\nBody.\n")
    (merged / "merged.json").write_text(json.dumps({"runs": [{"dir": str(run), "claim_source": "README.md"}]}))
    issues.note(run, "claims_audit", "surface_check", "a quote is elsewhere", severity="blocking")
    issues.note(run / "review", "audit_review", "adverse_recall", "something adverse", severity="check")
    issues.note(merged, "audit_materiality", "borderline_rating", "split", severity="check")
    rec = signoff.sign(merged, "bruce@tuuyi.com")
    assert rec["worklist"] == {"blocking": 1, "check": 2, "note": 0}
    assert signoff.load(merged) == rec
    block = signoff.block_md(rec)
    assert "Read and released by bruce@tuuyi.com" in block and "1 blocking, 2 check" in block
    assert rec["report_sha256"] in block


def test_no_report_or_no_person_is_refused(tmp_path):
    with pytest.raises(SystemExit):
        signoff.sign(tmp_path, "bruce@tuuyi.com")
    (tmp_path / "report.md").write_text("# r\n")
    with pytest.raises(SystemExit):
        signoff.sign(tmp_path, "  ")
    assert signoff.load(tmp_path) is None
