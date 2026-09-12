"""A review part that does not parse is asked for once more, same request."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from workflowsv2.audit_review import runner                       # noqa: E402


def test_unparsed_part_is_retried_once(monkeypatch):
    seen = []

    def fake_emit(loop, method_text, user, schema, max_tokens, **kw):
        seen.append(user)
        if len(seen) == 1:                      # first claim_checks call: cut off
            return {"obj": None, "parse": "failed", "parse_error": "Unterminated string",
                    "finish": "length", "raw": "{", "attempts": [], "response_format_dropped": []}
        body = {"claim_checks": [{"claim_id": 1, "fidelity": "faithful"}]} \
            if "claim_checks" in user else (
                {"finding_reviews": [{"claim_id": 1, "evidence_relevant": "yes",
                                      "evidence_supports": "yes",
                                      "verdict_calibration": "correct",
                                      "searches_adequate": "not_applicable"}]}
                if "finding_reviews" in user else {"record_check": "borne out"})
        return {"obj": body, "parse": "parsed", "parse_error": None, "finish": "stop",
                "raw": "", "attempts": [], "response_format_dropped": []}

    monkeypatch.setattr(runner, "emit", fake_emit)
    frozen = [{"id": 1, "quote": "q", "lines": [1, 1], "statement": "s", "about": "target"}]
    findings = [{"claim_id": 1, "adjudication": {"verdict": "real"}, "evidence": []}]
    out = runner.emit_parts(None, "METHOD", {}, frozen, findings, max_tokens=100)
    parts = [c["part"] for c in out["calls"]]
    assert parts == ["claim_checks[1/1]", "claim_checks[1/1]", "finding_reviews[1/1]", "record_check"]
    assert [c.get("retry") for c in out["calls"]] == [None, True, None, None]
    assert out["obj"]["claim_checks"] == [{"claim_id": 1, "fidelity": "faithful"}]
    assert seen[0] == seen[1]                   # the retry is the same request
    # A part that fails twice is not retried again.
    seen.clear()

    def always_fail(loop, method_text, user, schema, max_tokens, **kw):
        seen.append(user)
        return {"obj": None, "parse": "failed", "parse_error": "x", "finish": "length",
                "raw": "", "attempts": [], "response_format_dropped": []}

    monkeypatch.setattr(runner, "emit", always_fail)
    out = runner.emit_parts(None, "METHOD", {}, frozen, findings, max_tokens=100)
    assert len(out["calls"]) == 6 and out["obj"]["claim_checks"] == []
