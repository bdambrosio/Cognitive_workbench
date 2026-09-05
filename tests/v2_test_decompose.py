"""Decomposing a broad claim (DECOMPOSE.md): the proposal call, the rows it
becomes, the surface check on `implied_by`, the report's mark, and the
surface editor's endpoint. No model: the emission is faked."""
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from workflowsv2.claims_audit import decompose, schemas         # noqa: E402

PARENT = {"id": 3, "quote": "The target is a simple, self-hosted URL shortener.",
          "lines": [5, 5], "statement": "A simple self-hosted URL shortener.", "about": "target"}
OTHERS = [{"id": 1, "statement": "MIT licensed.", "quote": "MIT", "lines": [2, 2], "about": "document"}, PARENT]


def test_propose_uses_the_method_and_returns_proposals(monkeypatch):
    seen = {}
    def fake_emit(loop, system, user, schema, max_tokens, salvage=None):
        seen.update(system=system, user=user, schema=schema)
        return {"obj": {"subclaims": [{"statement": "A short code resolves to its stored URL.", "property": "resolves codes"},
                                      {"statement": "", "property": "blank"}],
                        "declined": [{"text": "simple", "why": "vague"}]},
                "parse": "parsed", "parse_error": None, "raw": "{}"}
    monkeypatch.setattr(decompose, "emit", fake_emit)
    res = decompose.propose(object(), PARENT, "   5>|The target is ...", OTHERS, category_note="A URL shortener resolves codes.")
    assert [s["property"] for s in res["subclaims"]] == ["resolves codes"]       # the blank one dropped
    assert res["declined"][0]["why"] == "vague" and res["parse"] == "parsed"
    assert "Decomposing a broad claim" in seen["system"] and "DECOMPOSE.md" in seen["user"]
    assert "1. MIT licensed." in seen["user"] and "3. A simple" not in seen["user"]   # others listed, not the parent
    assert "The practice's note on this category" in seen["user"]
    assert seen["schema"]["properties"]["subclaims"]["maxItems"] == decompose.MAX_SUBCLAIMS


def test_append_subclaims_numbers_after_the_surface_and_marks_the_parent():
    claims = [dict(c) for c in OTHERS]
    new = decompose.append_subclaims(claims, PARENT, [{"statement": "Resolves codes.", "property": "resolves codes"},
                                                      {"statement": "Runs from its own container.", "property": "self-hostable"}], "p@x.test")
    assert [c["id"] for c in new] == [4, 5] and len(claims) == 4
    assert new[0]["quote"] == PARENT["quote"] and new[0]["lines"] == [5, 5]
    assert new[0]["implied_by"] == 3 and new[0]["about"] == "target" and new[0]["approved_by"] == "p@x.test"
    # the surface check accepts a parent that exists and refuses one that does not
    src = Path(__file__).parent / "_decompose_src.md"
    src.write_text("a\nMIT\nb\nc\nThe target is a simple, self-hosted URL shortener.\n")
    try:
        ok = schemas.check_surface({"claim_source": src.name, "claims": claims}, src.parent, src.name)
        assert not [p for p in ok["problems"] if "implied_by" in p]
        claims[-1]["implied_by"] = 99
        bad = schemas.check_surface({"claim_source": src.name, "claims": claims}, src.parent, src.name)
        assert any("implied_by 99 names no claim" in p for p in bad["problems"])
    finally:
        src.unlink()


def test_report_marks_an_implied_finding():
    from workflowsv2.audit_report.render import _finding
    f = {"claim_source": "README.md", "claim_id": 4, "quote": PARENT["quote"], "lines": [5, 5],
         "statement": "A short code resolves to its stored URL.", "about": "target", "implied_by": 3,
         "adjudication": {"verdict": "real"}, "evidence": []}
    text = "\n".join(_finding(f, None, "materiality"))
    assert "the practice's reading of claim 3" in text and "resolves to its stored URL" in text


def test_surface_endpoint_proposes_rows(tmp_path, monkeypatch):
    pytest.importorskip("fastapi")
    from fastapi.testclient import TestClient
    from workflowsv2 import engagement_state as st
    from client_ui import site, mail
    from client_ui.access import Access
    monkeypatch.setattr(st, "ENGAGEMENTS", tmp_path)
    monkeypatch.setenv("MAIL_DRY_RUN", "1"); monkeypatch.setenv("PRACTICE_EMAILS", "p@x.test"); mail.sent.clear()
    eng = st.new_engagement(tmp_path / "e1")
    (eng / "target").mkdir(); (eng / "target" / "README.md").write_text("a\nMIT\nb\nc\nThe target is a simple, self-hosted URL shortener.\n")
    (eng / "engagement.yaml").write_text("target: target\nclaim_sources: [README.md]\nclient_emails: []\n")
    monkeypatch.setattr(decompose, "backend_from_model", lambda m: object())
    monkeypatch.setattr(decompose, "propose", lambda backend, claim, section, others, note=None: {
        "subclaims": [{"statement": "A short code resolves to its stored URL.", "property": "resolves codes"}],
        "declined": [{"text": "simple", "why": "vague"}], "parse": "parsed"})
    c = TestClient(site.make_site_app(Access(["p@x.test"], no_access=True), None, build=lambda k: None))
    r = c.post("/p/surface/e1/api/decompose?as=p@x.test", json={"source": "README.md", "claim_id": 3, "claims": OTHERS})
    assert r.status_code == 200, r.text
    j = r.json()
    assert j["subclaims"][0]["id"] == 4 and j["subclaims"][0]["implied_by"] == 3 and j["subclaims"][0]["approved_by"] == "p@x.test"
    assert j["declined"][0]["text"] == "simple"
    assert c.post("/p/surface/e1/api/decompose?as=p@x.test", json={"source": "README.md", "claim_id": 9, "claims": OTHERS}).status_code == 400
    assert c.post("/p/surface/e1/api/decompose?as=nobody@x.test", json={"source": "README.md", "claim_id": 3, "claims": OTHERS}).status_code == 403
