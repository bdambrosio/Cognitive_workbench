"""An adverse observation without its exception is asked for once more."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from workflowsv2.audit_review.runner import _unexplained          # noqa: E402


def test_unexplained_names_adverse_reviews_with_no_quoted_exception():
    rows = [
        {"claim_id": 1, "evidence_relevant": "yes", "evidence_supports": "yes",
         "verdict_calibration": "correct", "searches_adequate": "not_applicable"},
        {"claim_id": 2, "evidence_relevant": "yes", "evidence_supports": "no",
         "verdict_calibration": "overstated", "searches_adequate": "not_applicable"},
        {"claim_id": 3, "evidence_relevant": "no", "evidence_supports": "yes",
         "verdict_calibration": "correct", "searches_adequate": "not_applicable",
         "exception": "x", "materials_show": ""},
        {"claim_id": 4, "evidence_relevant": "no", "evidence_supports": "yes",
         "verdict_calibration": "correct", "searches_adequate": "not_applicable",
         "exception": "x", "finding_says": "a", "materials_show": "b"}]
    assert _unexplained(rows) == [2, 3]
    assert _unexplained([]) == []
