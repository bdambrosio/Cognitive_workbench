"""The adverse-evidence recall check beside the review (audit_review.runner.
adverse_recall): subjects are the `real` findings, the traces filed under
each claim are what the call reads, hits become issues."""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from workflowsv2.audit_review import runner as rv                 # noqa: E402


def _run(tmp_path):
    run = tmp_path / "run"
    tr = run / "working_record" / "inspect_traces"
    tr.mkdir(parents=True)
    (tr / "inspect_external_2026-09-07T00-00-01Z.txt").write_text(
        "Query: [claims 1, 2] read frontend/index.html\n<script src=\"https://cdn/qrcode.js\">\n")
    (tr / "inspect_external_2026-09-07T00-00-02Z.txt").write_text(
        "Query: [claims 3] read a.py\nx = 1\n")
    return run


def test_subjects_are_real_findings_and_hits_become_issues(tmp_path, monkeypatch):
    run = _run(tmp_path)
    frozen = [{"id": i, "quote": f"q{i}", "statement": f"s{i}"} for i in (1, 2, 3, 4)]
    findings = [
        {"claim_id": 1, "adjudication": {"verdict": "real"}, "evidence": []},
        {"claim_id": 2, "adjudication": {"verdict": "real_with_caveat", "gap": "g"}, "evidence": []},
        {"claim_id": 3, "adjudication": {"verdict": "real"}, "evidence": []},
        {"claim_id": 4, "adjudication": {"verdict": "real"}, "evidence": []},   # no trace filed
    ]
    seen = []

    def fake_emit(loop, method_text, user, schema, max_tokens):
        ids = [int(x) for x in __import__("re").findall(r"--- claim (\d+)", user)]
        seen.append((ids, "cdn/qrcode.js" in user))
        return {"obj": {"adverse_recall": [
            {"claim_id": i, "uncited_adverse": i == 1,
             "where": "frontend/index.html:1" if i == 1 else "",
             "what": "loaded from a CDN" if i == 1 else ""} for i in ids]},
            "raw": "", "parse": "parsed", "parse_error": None, "finish": "stop",
            "attempts": [], "response_format_dropped": []}
    monkeypatch.setattr(rv, "emit", fake_emit)
    out = rv.adverse_recall(None, "method", run, frozen, findings, 1000, batch=10)
    assert out["subjects"] == [1, 3, 4]
    assert out["untagged"] == [4]
    assert sorted(i for ids, _ in seen for i in ids) == [1, 3]
    assert all(has for ids, has in seen if 1 in ids)
    assert [h["claim_id"] for h in out["hits"]] == [1]
    rev = run / "review"; rev.mkdir()
    rv._write_recall(rev, out)
    rows = [json.loads(l) for l in (rev / "issues.jsonl").read_text().splitlines()]
    assert len(rows) == 1 and rows[0]["code"] == "adverse_recall" and "claim 1" in rows[0]["text"]
    assert json.loads((rev / "adverse_recall.json").read_text())["hits"][0]["claim_id"] == 1
