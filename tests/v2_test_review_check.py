"""REVIEW §5 check 5 is answered for every finding that records a search,
whatever its verdict, and for no other (2026-09-03, ChatterMate claim 91:
a `partial` resting on a search confined to one directory)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from workflowsv2.audit_review.schemas import check_review          # noqa: E402


def _finding(cid, verdict, search):
    ev = [{"form": "citation", "document": "a.py", "lines": [1, 1], "quote": "x", "shows": "y"}]
    if search:
        ev.append({"form": "search", "kind": "lexical", "performed": "p", "result": "r", "candidates": []})
    return {"claim_id": cid, "adjudication": {"verdict": verdict}, "evidence": ev}


def _review(cid, sa):
    return {"claim_id": cid, "evidence_relevant": "yes", "evidence_supports": "yes",
            "verdict_calibration": "correct", "searches_adequate": sa,
            "exception": "", "finding_says": "", "materials_show": ""}


def test_searches_are_judged_when_a_finding_made_any():
    frozen = [{"id": i} for i in (1, 2, 3)]
    findings = [_finding(1, "unverifiable", True), _finding(2, "partial", True),
                _finding(3, "partial", False)]
    obj = {"claim_checks": [{"claim_id": i, "fidelity": "faithful"} for i in (1, 2, 3)],
           "finding_reviews": [_review(1, "not_applicable"), _review(2, "not_applicable"),
                               _review(3, "yes")],
           "record_check": "ok"}
    text = "\n".join(check_review(obj, frozen, findings)["problems"])
    assert "finding_review 1: the `unverifiable` finding records a search" in text
    assert "finding_review 2: the `partial` finding records a search" in text
    assert "finding_review 3: `searches_adequate` is 'yes' on a `partial` finding, which made no searches" in text
    obj["finding_reviews"] = [_review(1, "yes"), _review(2, "no") | {"exception": "narrow", "finding_says": "a", "materials_show": "b"},
                              _review(3, "not_applicable")]
    assert check_review(obj, frozen, findings)["ok"], check_review(obj, frozen, findings)["problems"]
