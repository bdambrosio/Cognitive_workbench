"""The report stage: three classes from the record, a document that assembles
without the agent, and the prose check."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from workflowsv2.audit_report import render, schemas          # noqa: E402


def _record():
    cite = [{"form": "citation", "document": "app/x.py", "lines": [3, 4],
             "quote": "x", "shows": "y"}]
    search = [{"form": "search", "kind": "lexical", "performed": "p", "result": "r",
               "candidates": []},
              {"form": "search", "kind": "structural", "performed": "p", "result": "r",
               "candidates": ["app/h.py"]}]

    def f(src, cid, verdict, **adj):
        return {"claim_source": src, "claim_id": cid, "quote": f"claim {cid} text",
                "lines": [cid, cid], "statement": "s",
                "adjudication": {"verdict": verdict, **adj},
                "evidence": search if verdict == "unverifiable" else cite,
                "review": {"outcome": "holds", "adverse_observations": []},
                "citation_problems": []}
    merged = {"engagement": "eng", "runs": [
        {"dir": "runs/x", "claim_source": "README.md", "reviewed": True,
         "claims": 5, "findings": 5, "files_read": 12, "gathering_legs": 3,
         "resolved_model": "m", "unopened_candidates": ["app/h.py"],
         "captured_at_utc": "2026-09-02T16-12-41Z", "target_rev": "abcdef0123456789"}],
        "findings": [
            f("README.md", 1, "contradicted", gap="g1"),
            f("README.md", 2, "real"),
            f("README.md", 3, "unverifiable", unresolved_because="not_in_the_materials"),
            f("README.md", 4, "unverifiable", unresolved_because="not_examined"),
            f("README.md", 5, "real_with_caveat", gap="g5")],
        "questions": [{"claim_source": "README.md", "question": "who?"}],
        "unclaimed": [],
        "figures": {"runs": 1, "claims": 5, "findings": 5, "reviewed": 5,
                    "unreviewed": 0,
                    "verdicts": {"contradicted": 1, "real": 1, "unverifiable": 2,
                                 "real_with_caveat": 1}}}
    ratings = {"ratings": [
        {"claim_source": "README.md", "claim_id": 1, "materiality": "material", "basis": "b1"},
        {"claim_source": "README.md", "claim_id": 5, "materiality": "decisive", "basis": "b5"}],
        "exposures": [
        {"claim_source": "README.md", "claim_id": 3, "exposure": "not_material", "basis": "e3"},
        {"claim_source": "README.md", "claim_id": 4, "exposure": "decisive", "basis": "e4"}],
        "figures": {"materiality": {"material": 1, "decisive": 1},
                    "exposure": {"not_material": 1, "decisive": 1}}}
    return {"merged": merged, "ratings": ratings, "dir": Path(".")}


def test_classify_sorts_by_verdict_and_disposition():
    c = render.classify(_record()["merged"])
    assert [f["claim_id"] for f in c["shown"]] == [1, 5]
    assert [f["claim_id"] for f in c["unsettled"]] == [3]
    assert [f["claim_id"] for f in c["not_examined"]] == [4]
    assert [f["claim_id"] for f in c["holds"]] == [2]


def test_assemble_without_prose_leaves_markers_and_orders_by_rating():
    doc = render.assemble(_record(), None, "Buyer buys.", "eng", "walks if no backups")
    for f in schemas.FIELDS:
        assert f"[[{f}]]" in doc
    # front matter, transaction, thresholds, key findings computed
    assert doc.splitlines()[2].startswith("Materials as of 2026-09-02 at commit abcdef012345. Claim sources: `README.md`.")
    assert "**The assurance given is limited.**" in doc and "was not consulted" in doc
    assert "## The transaction\n\nBuyer buys.\n\n**The buyer's thresholds.** walks if no backups" in doc
    ex = doc[doc.index("## Executive summary"):doc.index("## Scope and approach")]
    assert "[[summary]]" in ex
    assert "- **README.md, claim 5** (true, with something the buyer must know; decisive): g5" in ex
    assert "- **README.md, claim 1** (contradicted; material): g1" in ex
    assert ex.index("claim 5") < ex.index("claim 1")
    # order of sections
    order = ["## The transaction", "## Executive summary", "## Scope and approach",
             "## How to read a finding", "## What the audit showed", "## Unsettled claims",
             "## Claims not examined", "## Claims that hold", "## Questions for the seller",
             "## Coverage", "## Limitations", "## Appendix"]
    idx = [doc.index(h) for h in order]
    assert idx == sorted(idx)
    scope = doc[doc.index("## Scope and approach"):doc.index("## How to read a finding")]
    assert "| README.md | 5 | 5 | yes | 12 | 3 | m |" in scope and "[[scope_note]]" in scope
    assert "| unsettled, or not examined |" in doc                    # the crosswalk
    # decisive before material within the shown section
    shown = doc[doc.index("## What the audit showed"):doc.index("## Unsettled claims")]
    assert shown.index("claim 5 — materiality: decisive") < shown.index("claim 1 — materiality: material")
    assert "**The gap:** g5" in shown and "`app/x.py`, lines 3–4" in shown
    unsettled = doc[doc.index("## Unsettled claims"):doc.index("## Claims not examined")]
    assert "claim 3 — exposure: not_material" in unsettled
    assert "material of the right kind was supplied" in unsettled
    ne = doc[doc.index("## Claims not examined"):doc.index("## Claims that hold")]
    assert "claim 4 — exposure: decisive" in ne and "Files named: `app/h.py`" in ne
    assert "| README.md | 2 | claim 2 text | `app/x.py` lines 3–4 |" in doc
    cov = doc[doc.index("## Coverage"):doc.index("## Limitations")]
    assert "| contradicted | shown | 1 |" in cov and "| unverifiable | unsettled | 2 |" in cov
    assert "Not examined: 1." in cov and "- `app/h.py`" in cov
    lim = doc[doc.index("## Limitations"):doc.index("## Appendix")]
    assert "[[limitations]]" in lim and "has not confirmed the audit's interpretation" in lim
    assert "- (README.md) who?" in doc


def test_assemble_places_prose_and_drops_the_not_examined_section_when_empty():
    rec = _record()
    prose = {f: f"<{f}>" for f in schemas.FIELDS}
    doc = render.assemble(rec, prose, None, "eng")
    assert "[[" not in doc and "<summary>" in doc and "<limitations>" in doc
    assert "The engagement states nothing about the transaction" in doc
    assert "**The buyer's thresholds.** None recorded." in doc
    assert "## Claims not examined" in doc
    rec["merged"]["findings"] = [f for f in rec["merged"]["findings"] if f["claim_id"] != 4]
    doc = render.assemble(rec, None, None, "eng")
    assert "## Claims not examined" not in doc and "[[not_examined_note]]" not in doc


def test_check_prose():
    m = _record()["merged"]
    good = {f: "Claim 1 matters." for f in schemas.FIELDS}
    assert "scope_note" in schemas.FIELDS and "coverage_note" not in schemas.FIELDS
    assert schemas.check_prose(good, m, True)["ok"]
    bad = dict(good, summary="=== COVER ===\nsee claim 99", not_examined_note="")
    res = schemas.check_prose(bad, m, True)
    text = "\n".join(res["problems"])
    assert "summary: carries a `===` marker" in text
    assert "names claim 99" in text
    assert "not_examined_note: empty" in text
    res = schemas.check_prose(good, m, False)
    assert res["problems"] == ["not_examined_note: written, and the document has "
                               "no such claims (REPORT.md §6)"]
    assert schemas.check_prose(dict(good, not_examined_note=""), m, False)["ok"]


def test_header_index_and_worklist(tmp_path):
    rec = _record()
    doc = render.assemble(rec, None, None, "eng")
    assert doc.splitlines()[2].startswith("Materials as of 2026-09-02 at commit abcdef012345.")
    apx = doc[doc.index("## Appendix — every claim and its verdict"):]
    rows = [l for l in apx.splitlines() if l.startswith("| README.md")]
    assert len(rows) == 5
    assert "| README.md | 1 | line 1 | claim 1 text | contradicted | material |" in rows[0]
    assert "| README.md | 4 | line 4 | claim 4 text | unverifiable | decisive |" in rows[3]
    # worklist gathers every stage's issues, blocking first
    run = tmp_path / "runs" / "x"; (run / "review").mkdir(parents=True)
    (run / "issues.jsonl").write_text(
        '{"ts":"t","stage":"claims_audit","code":"output_check","severity":"blocking","text":"finding 1: bad quote"}\n'
        '{"ts":"t","stage":"claims_audit","code":"not_examined","severity":"check","text":"2 files unopened"}\n')
    (run / "review" / "issues.jsonl").write_text(
        '{"ts":"t","stage":"audit_review","code":"review_check","severity":"blocking","text":"finding 3 no exception"}\n')
    md = tmp_path / "merged"; md.mkdir()
    (md / "issues.jsonl").write_text(
        '{"ts":"t","stage":"audit_materiality","code":"unreviewed_run","severity":"check","text":"no review"}\n')
    rec["merged"]["runs"][0]["dir"] = str(run)
    wl = render.worklist(rec["merged"], md)
    assert "4 item(s)" in wl
    assert wl.index("## blocking") < wl.index("## check")
    assert "**claims_audit / output_check** (README.md): finding 1: bad quote" in wl
    assert "**audit_materiality / unreviewed_run** (merged): no review" in wl
    assert render.worklist({"runs": []}).strip().endswith("Nothing recorded.")
