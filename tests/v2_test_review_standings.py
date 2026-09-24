"""REVIEW.md §9: which observations decide whether a retested exception stands.

Added 2026-09-23. `standings` compared all four observations, so a retest
that agreed the evidence did not support README #51 (chhoto-full, gpt-6-sol)
and differed only on `verdict_calibration`, which §9 does not retest, marked
the exception as not standing.
"""
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "src"))

from workflowsv2.audit_review.runner import standings   # noqa: E402


def _obs(supports, calibration):
    return {"evidence_relevant": "yes", "evidence_supports": supports,
            "verdict_calibration": calibration, "searches_adequate": "not_applicable"}


def _run(first, second):
    return standings({"finding_reviews": [{"claim_id": 51, **first}]},
                     {"ran": True, "results": {51: {"observations": second}}},
                     candidates=[51], sampled=[])["per_finding"][51]["standing"]


def test_a_calibration_difference_alone_does_not_unseat_an_exception():
    assert _run(_obs("no", "understated"), _obs("no", "correct")) == "stands"


def test_a_retested_observation_that_differs_does():
    assert _run(_obs("no", "correct"), _obs("yes", "correct")) == "does not stand"
