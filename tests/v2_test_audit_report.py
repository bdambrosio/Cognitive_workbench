"""The report stage: three classes from the record, a document that assembles
without the agent, and the prose check."""
import json
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

    def f(src, cid, verdict, about="target", **adj):
        return {"claim_source": src, "claim_id": cid, "quote": f"claim {cid} text",
                "lines": [cid, cid], "statement": "s", "about": about,
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
            f("README.md", 5, "real_with_caveat", gap="g5"),
            # the seller's promises: one the materials cannot reach, one they settle
            f("README.md", 6, "unverifiable", about="seller", unresolved_because="outside_the_materials"),
            f("README.md", 7, "real", about="seller")],
        "questions": [{"claim_source": "README.md", "question": "who?"},
                      {"claim_source": "README.md", "claim_id": 6, "question": "for how long?"},
                      {"claim_source": "README.md", "claim_id": 3, "question": "where?"}],
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
        {"claim_source": "README.md", "claim_id": 4, "exposure": "decisive", "basis": "e4"},
        {"claim_source": "README.md", "claim_id": 6, "exposure": "material", "basis": "e6"}],
        "figures": {"materiality": {"material": 1, "decisive": 1},
                    "exposure": {"not_material": 1, "decisive": 1, "material": 1}}}
    return {"merged": merged, "ratings": ratings, "dir": Path(".")}


def test_classify_sorts_by_verdict_and_disposition():
    c = render.classify(_record()["merged"])
    assert [f["claim_id"] for f in c["shown"]] == [1, 5]
    assert [f["claim_id"] for f in c["unsettled"]] == [3]
    assert [f["claim_id"] for f in c["not_examined"]] == [4]
    assert [f["claim_id"] for f in c["seller_unsettled"]] == [6]
    assert [f["claim_id"] for f in c["holds"]] == [2, 7]          # settled seller claim stays with its verdict


def test_assemble_without_prose_leaves_markers_and_orders_by_rating():
    doc = render.assemble(_record(), None, "Buyer buys.", "eng", "walks if no backups")
    for f in schemas.FIELDS:
        if f == "conclusion":
            continue                    # opt-in: absent unless asked for
        assert f"[[{f}]]" in doc
    # front matter, transaction, thresholds, key findings computed
    assert doc.splitlines()[2].startswith("Materials as of 2026-09-02 at commit abcdef012345. Claim sources: `README.md`.")
    assert "**The assurance given is limited.**" in doc and "was not consulted" in doc
    assert "## The transaction\n\nBuyer buys.\n\n**The buyer's thresholds.**  \nwalks if no backups" in doc
    ex = doc[doc.index("## Executive summary"):doc.index("## Scope and approach")]
    assert "[[summary]]" in ex
    # a rating with no plain account (made before MATERIALITY §5b) falls back to the statement
    assert "- **README.md, claim 5** (decisive): s — true, with something the buyer must know" in ex
    assert "- **README.md, claim 1** (material): s — contradicted" in ex
    assert ex.index("claim 5") < ex.index("claim 1")
    # order of sections
    order = ["## The transaction", "## Executive summary", "## What this document is",
             "## Scope and approach",
             "## How to read a finding", "## What the review showed", "## Unsettled claims",
             "## Unsettled claims about the seller",
             "## Unexamined claims", "## Claims that hold", "## Questions for the seller",
             "## Coverage", "## Limitations", "## Appendix"]
    idx = [doc.index(h) for h in order]
    assert idx == sorted(idx)
    scope = doc[doc.index("## Scope and approach"):doc.index("## How to read a finding")]
    assert "| README.md | 5 | 5 | yes | 12 | 3 | m |" in scope and "[[scope_note]]" in scope
    assert "| unsettled, or unexamined |" in doc                    # the crosswalk
    # decisive before material within the shown section
    shown = doc[doc.index("## What the review showed"):doc.index("## Unsettled claims")]
    assert shown.index("README.md, claim 5 · materiality: decisive") < shown.index("README.md, claim 1 · materiality: material")
    # the heading is the claim's statement and the verdict in words
    assert "### s — true, with something the buyer must know\n\n*README.md, claim 5 · materiality: decisive*" in shown
    assert "**The gap:** g5" in shown and "`app/x.py`, lines 3–4" in shown
    # outside the definitions and the appendices, no rating appears as a code
    body = doc[doc.index("## What the review showed"):doc.index("## Appendix")]
    assert "not_material" not in body and "real_with_caveat" not in body
    unsettled = doc[doc.index("## Unsettled claims"):doc.index("## Unsettled claims about the seller")]
    assert "README.md, claim 3 · exposure: not material" in unsettled
    assert "material of the right kind was supplied" in unsettled
    assert "**Question for the seller:** where?" in unsettled           # a question rides with its finding
    assert "claim 6" not in unsettled
    seller = doc[doc.index("## Unsettled claims about the seller"):doc.index("## Unexamined claims")]
    assert seller.startswith("## Unsettled claims about the seller\n\n" + render.SELLER_LINE)
    assert "README.md, claim 6 · exposure: material" in seller
    assert "**Question for the seller:** for how long?" in seller
    assert "claim 7" not in seller and "claim 3" not in seller
    ne = doc[doc.index("## Unexamined claims"):doc.index("## Claims that hold")]
    assert "README.md, claim 4 · exposure: decisive" in ne
    assert "Files named: `app/h.py`" not in ne and "- searched: r" in ne     # in full in the evidence document
    assert "| README.md | 2 | claim 2 text | `app/x.py` lines 3–4 |" in doc
    assert "| README.md | 7 | claim 7 text | `app/x.py` lines 3–4 |" in doc
    cov = doc[doc.index("## Coverage"):doc.index("## Limitations")]
    assert "| contradicted | 1 |" in cov and "| unsettled | 3 |" in cov
    assert "of which 1 about the seller. Unexamined: 1." in cov and "- `app/h.py`" in cov
    assert "2. (README.md, claim 6) for how long?" in doc               # the checklist stays complete, numbered for the writer
    lim = doc[doc.index("## Limitations"):doc.index("## Appendix")]
    assert "[[limitations]]" in lim and "has not confirmed the review's interpretation" in lim
    assert "1. (README.md) who?" in doc


def test_assemble_places_prose_and_drops_the_not_examined_section_when_empty():
    rec = _record()
    prose = {f: f"<{f}>" for f in schemas.FIELDS}
    doc = render.assemble(rec, prose, None, "eng")
    assert "[[" not in doc and "<summary>" in doc and "<limitations>" in doc
    assert "The engagement states nothing about the transaction" in doc
    assert "**The buyer's thresholds.**  \nNone recorded." in doc
    assert "## Unexamined claims" in doc
    rec["merged"]["findings"] = [f for f in rec["merged"]["findings"] if f["claim_id"] not in (4, 6)]
    doc = render.assemble(rec, None, None, "eng")
    assert "## Unexamined claims" not in doc and "[[not_examined_note]]" not in doc
    assert "## Unsettled claims about the seller" not in doc and render.SELLER_LINE not in doc


def test_check_prose():
    m = _record()["merged"]
    good = {f: "Claim 1 matters." for f in schemas.FIELDS}
    assert "scope_note" in schemas.FIELDS and "coverage_note" not in schemas.FIELDS
    assert schemas.check_prose(good, m, True, wants_conclusion=True)["ok"]
    bad = dict(good, summary="=== COVER ===\nsee claim 99", not_examined_note="")
    res = schemas.check_prose(bad, m, True, wants_conclusion=True)
    text = "\n".join(res["problems"])
    assert "summary: carries a `===` marker" in text
    assert "names claim 99" in text
    assert "not_examined_note: empty" in text
    res = schemas.check_prose(good, m, False, wants_conclusion=True)
    assert res["problems"] == ["not_examined_note: written, and the document has "
                               "no such claims (REPORT.md §6)"]
    assert schemas.check_prose(dict(good, not_examined_note=""), m, False, True)["ok"]
    # the conclusion is written only when asked for
    res = schemas.check_prose(dict(good, not_examined_note=""), m, False)
    assert res["problems"] == ["conclusion: written, and the document does not "
                               "ask for a conclusion (REPORT.md §6)"]
    assert schemas.check_prose(dict(good, not_examined_note="", conclusion=""), m, False)["ok"]
    res = schemas.check_prose(dict(good, not_examined_note="", conclusion=""), m, False, True)
    assert res["problems"] == ["conclusion: empty (REPORT.md §6)"]


def test_assemble_carries_a_conclusion_only_when_asked_and_thresholds_exist():
    rec = _record()
    prose = {f: f"[{f}]" for f in schemas.FIELDS}
    doc = render.assemble(rec, prose, "buys it", "eng", "walks if no backups", True)
    head = "## Against the buyer's thresholds"
    assert head in doc and "[conclusion]" in doc
    # the answer comes first: before the summary and before the front matter
    assert doc.index("## The transaction") < doc.index(head) < doc.index("## Executive summary") \
        < doc.index("## What this document is")
    assert head not in render.assemble(rec, prose, "buys it", "eng", "walks", False)
    assert head not in render.assemble(rec, prose, "buys it", "eng", None, True)


def test_header_index_and_worklist(tmp_path):
    rec = _record()
    doc = render.assemble(rec, None, None, "eng")
    assert doc.splitlines()[2].startswith("Materials as of 2026-09-02 at commit abcdef012345.")
    apx = doc[doc.index("## Appendix — every claim and its verdict"):]
    rows = [l for l in apx.splitlines() if l.startswith("| README.md")]
    assert len(rows) == 7
    assert "| README.md | 1 | 1 | claim 1 text | contradicted | material |" in rows[0]
    assert "| README.md | 4 | 4 | claim 4 text | unverifiable | decisive |" in rows[3]
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


def test_an_outside_claim_shows_the_links_the_claim_source_gives():
    """B, 2026-09-03: a claim resting on a listing or a package shows the
    claim source's own links, and only those — nothing is inferred from a
    package name."""
    def f(cid, quote, locations=(), because="outside_the_materials"):
        return {"claim_source": "README.md", "claim_id": cid, "quote": quote,
                "lines": [cid, cid], "statement": "s", "locations": list(locations),
                "adjudication": {"verdict": "unverifiable",
                                 "unresolved_because": because},
                "evidence": [], "review": {"outcome": "holds",
                                           "adverse_observations": []},
                "citation_problems": []}
    linked = f(1, "**[Shopify App](https://apps.shopify.com/chattermate-chat)**",
               [{"quote": "Install it from the [Shopify App Store]"
                          "(https://apps.shopify.com/chattermate-chat).", "lines": [9, 9]},
                {"quote": "see https://docs.example.com/x, then", "lines": [12, 12]}])
    out = "\n".join(render._finding(linked, None, "exposure"))
    assert ("**Where the claim source points:** <https://apps.shopify.com/chattermate-chat>, "
            "<https://docs.example.com/x>. The practice did not follow") in out
    unlinked = f(2, "pip install chattermate-cli")
    assert "Where the claim source points" not in "\n".join(render._finding(unlinked, None, "exposure"))
    badge = f(4, "[![npm version](https://img.shields.io/npm/v/x.svg)]"
                 "(https://www.npmjs.com/package/x) then open http://localhost/")
    assert ("**Where the claim source points:** <https://www.npmjs.com/package/x>. "
            in "\n".join(render._finding(badge, None, "exposure")))
    not_outside = f(3, "see https://x.example", because="not_in_the_materials")
    assert "Where the claim source points" not in "\n".join(render._finding(not_outside, None, "exposure"))


def test_a_covered_claim_names_the_wider_claim_and_keeps_its_own_verdict(tmp_path):
    """A claim the duplicates pass marked as within a wider one is audited on
    its own (Bruce, 2026-09-17: never derived); the report shows the wider
    claim and its verdict beside it, once in the finding and once in the
    appendix, and defines the term."""
    eng = tmp_path / "eng"
    (eng / "surface").mkdir(parents=True)
    (eng / "merged" / "m1").mkdir(parents=True)
    (eng / "surface" / "duplicates.json").write_text(json.dumps({"pairs": [
        {"source": "README.md", "id": 5, "relation": "within",
         "other": {"source": "README.md", "id": 2, "statement": "the wider claim"}},
        {"source": "README.md", "id": 3, "relation": "same",
         "other": {"source": "README.md", "id": 1, "statement": "x"}}]}), encoding="utf-8")
    rec = _record()
    rec["covered"] = render.covered_by(eng / "merged" / "m1")
    assert list(rec["covered"]) == ["README.md#5"]
    text = render.assemble(rec, None, "t", "eng", None, False)
    assert ("**Covered by:** README.md, claim 2, a wider claim of which this one is a part: "
            "\"the wider claim\". That claim's verdict: holds. This claim was tested on its own; "
            "the verdict above is its own.") in text
    assert "| README.md | 5 | 5 | claim 5 text | real_with_caveat | decisive | README.md 2 |" in text
    assert "| README.md | 2 | 2 | claim 2 text | real |  |  |" in text
    assert "marked *covered by* names a wider claim" in text
    assert render.covered_by(tmp_path / "nowhere" / "merged" / "m") == {}


def test_a_material_finding_is_summed_up_in_the_raters_plain_words_not_its_gap():
    # Bruce, 2026-10-14: the summary carried a second copy of every material
    # gap, a page and a half, and "true, with something the buyer must know"
    # did not say what that something is. The line is the rater's plain
    # account (MATERIALITY §5b), which also opens the finding.
    rec = _record()
    next(f for f in rec["merged"]["findings"] if f["claim_id"] == 5)["adjudication"]["gap"] = "a long gap"
    next(r for r in rec["ratings"]["ratings"] if r["claim_id"] == 5)["in_short"] = \
        "The listing says backups are kept for 30 days. The job keeps them for 7."
    doc = render.assemble(rec)
    ex = doc[doc.index("## Executive summary"):doc.index("## What this document is")]
    assert ("- **README.md, claim 5** (decisive): The listing says backups are kept for 30 days. "
            "The job keeps them for 7.") in ex
    assert "a long gap" not in ex
    shown = doc[doc.index("## What the review showed"):]
    assert shown.index("**In short:** The listing says backups") < shown.index("**The gap:** a long gap")


def test_the_report_shows_evidence_in_short_and_the_evidence_document_in_full():
    rec = _record()
    long_quote = "let Ok(mut statement) = db.prepare_cached(queries::FIND_URL) else { return Err(ServerError); };"
    f1 = next(f for f in rec["merged"]["findings"] if f["claim_id"] == 1)
    f1["evidence"] = [{"form": "citation", "document": "app/x.py", "lines": [3, 4], "quote": long_quote, "shows": "binds it"},
                      {"form": "search", "kind": "lexical", "performed": "grep -rn format! src/", "result": "no SQL built",
                       "candidates": ["src/db.rs"]}]
    doc = render.assemble(rec)
    assert '"let Ok(mut statement) = db.prepare_cached(queries::FIND_URL) else { return …" — binds it' in doc
    assert long_quote not in doc and "grep -rn format!" not in doc and "- searched: no SQL built" in doc
    ev = render.evidence_document(rec, "eng")
    part = ev[ev.index("## README.md, claim 1\n"):ev.index("## README.md, claim 2\n")]
    assert long_quote.replace("<", "\\<") in part and "grep -rn format! src/" in part and "`src/db.rs`" in part


def test_claims_listed_and_not_tested_are_counted_and_given_their_own_appendix(tmp_path):
    rec = _record()
    assert "Every claim received one finding." in render.assemble(rec)
    surface = tmp_path / "e" / "surface"
    surface.mkdir(parents=True)
    (surface / "docs_cli_md.surface.json").write_text(json.dumps({"claim_source": "docs/CLI.md", "claims": [
        {"id": 8, "lines": [39, 39], "quote": "POST /api/new", "tier": 1, "tier_basis": "x"},
        {"id": 9, "lines": [40, 40], "quote": "page_size defaults to 10", "tier": 2,
         "tier_basis": "A default a developer codes against. Rated on the scale alone."}]}))
    rec["not_tested"] = render.not_tested(tmp_path / "e" / "merged" / "m1")
    assert [c["id"] for c in rec["not_tested"]] == [9]
    text = render.assemble(rec)
    assert "Every claim that was tested received one finding. 1 further claim(s)" in text
    about = text[text.index("## What this document is"):text.index("## Scope and approach")]
    assert "the others are listed as not tested" in about and "about each tested claim" in about
    assert "## Appendix — claims listed, not tested" in text
    assert "| docs/CLI.md | 9 | 40 | page_size defaults to 10 | 2 | A default a developer codes against." in text


def test_the_scope_table_counts_what_was_not_tested_and_names_a_source_with_no_run(tmp_path):
    rec = _record()
    plain = render.assemble(rec)
    assert "| claim source | claims | findings |" in plain and "not tested" not in plain.split("## How to read")[0]
    src = rec["merged"]["runs"][0]["claim_source"]
    rec["not_tested"] = [{"id": 9, "claim_source": src, "tier": 2, "tier_basis": "x", "quote": "q", "lines": [1, 1]},
                         {"id": 1, "claim_source": "docs/SECURITY.md", "tier": 3, "tier_basis": "y", "quote": "q", "lines": [3, 6]}]
    text = render.assemble(rec)
    assert "| claim source | tested | not tested | findings |" in text
    assert "| docs/SECURITY.md | 0 | 1 | 0 | | | | |" in text
    assert "`docs/SECURITY.md`" in text.split("## The transaction")[0]
    row = next(l for l in text.splitlines() if l.startswith(f"| {src} | "))
    assert row.split(" | ")[2] == "1"


def test_a_verdict_resting_on_dependency_knowledge_is_marked_and_no_other_is():
    """B, 2026-09-24: a finding whose evidence states what a dependency does
    from knowledge (`from_knowledge`) is marked under its verdict and on its
    evidence line; a derivation from the materials is not."""
    def f(from_knowledge):
        item = {"form": "derived", "derivation": "d", "consequence": "c",
                "basis": [{"document": "backend/src/main.rs", "lines": [99, 99], "quote": "q"}]}
        if from_knowledge:
            item["from_knowledge"] = True
        return {"claim_source": "docs/INSTALLATION.md", "claim_id": 1, "quote": "q",
                "lines": [1, 1], "statement": "s",
                "adjudication": {"verdict": "contradicted", "gap": "g"},
                "evidence": [item],
                "review": {"outcome": "holds", "adverse_observations": []},
                "citation_problems": []}
    marked = "\n".join(render._finding(f(True), None, "materiality"))
    plain = "\n".join(render._finding(f(False), None, "materiality"))
    assert "Rests in part on knowledge of a dependency." in marked
    assert "from knowledge of the dependency, not from the supplied materials" in marked
    assert "knowledge of" not in plain and "- derived from `backend/src/main.rs`" in plain


def test_a_statement_the_buyer_asked_is_reported_apart_from_the_sellers_claims():
    """A gap in a statement the buyer asked is not a mark against the seller:
    it has its own section, and is still listed among the material findings."""
    rec = _record()
    rec["merged"]["findings"].append({
        "claim_source": "questions/buyer.md", "claim_id": 1,
        "quote": "Passwords are hashed.", "lines": [2, 2],
        "statement": "Passwords are hashed.", "about": "target", "asked_by": "buyer",
        "adjudication": {"verdict": "contradicted", "gap": "stored in plain text"},
        "evidence": [{"form": "citation", "document": "app/u.py", "lines": [9, 9],
                      "quote": "x", "shows": "y"}],
        "review": {"outcome": "holds", "adverse_observations": []},
        "citation_problems": []})
    rec["ratings"]["ratings"].append({"claim_source": "questions/buyer.md", "claim_id": 1,
                                      "materiality": "material", "basis": "b"})
    doc = render.assemble(rec, None, "t", "eng", None, False)
    head = "questions/buyer.md, claim 1 ·"
    shown = doc[doc.index("## What the review showed"):doc.index("## Unsettled claims")]
    assert head not in shown
    asked = doc[doc.index("## Questions the buyer asked"):doc.index("## Claims that hold")]
    assert head in asked and "### Passwords are hashed. — contradicted" in asked
    assert "Asked by the buyer." in asked
    summary = doc[doc.index("## Executive summary"):doc.index("## Scope and approach")]
    assert "questions/buyer.md, claim 1" in summary
    # The front matter names the buyer's questions apart from the seller's
    # claim sources, and says the seller did not make them.
    front = doc[:doc.index("## The transaction")]
    claim_line = next(l for l in front.splitlines() if l.startswith("Materials as of"))
    assert "Questions tested: `questions/buyer.md` (the buyer's questions)" in claim_line
    assert "questions/buyer.md" not in claim_line.split("Questions tested:")[0]
    about = doc[doc.index("## What this document is"):doc.index("## Scope and approach")]
    assert "statements the seller did not make" in about
    plain = render.assemble(_record(), None, "t", "eng", None, False)
    assert "Questions tested" not in plain and "statements the seller did not make" not in plain


def test_findings_are_shown_under_their_subjects_only_when_the_groups_hold_each_once():
    rec = _record()
    sections = render.grouped_sections(render.classify(rec["merged"]))
    good = {f: f"<{f}>" for f in schemas.FIELDS}
    good["groups"] = [{"section": "shown", "title": "Backups", "findings": ["README.md#1"]},
                      {"section": "shown", "title": "Logging", "findings": ["README.md#5"]}]
    good["duplicate_questions"] = []
    assert schemas.check_prose(dict(good, not_examined_note="x"), rec["merged"], True,
                               True, sections)["ok"]
    doc = render.assemble(rec, good, None, "eng")
    shown = doc[doc.index("## What the review showed"):doc.index("## Unsettled claims")]
    # groups follow their highest-rated finding; findings sit one level down
    assert shown.index("### Logging") < shown.index("#### s — true, with something") \
        < shown.index("### Backups") < shown.index("#### s — contradicted")
    bad = dict(good, groups=[{"section": "shown", "title": "Backups", "findings": ["README.md#1", "README.md#1"]}])
    res = schemas.check_prose(dict(bad, not_examined_note="x"), rec["merged"], True, True, sections)
    assert "groups (shown): README.md#1 is in two groups; the section is shown ungrouped" in res["problems"]
    assert "groups (shown): README.md#5 is in no group; the section is shown ungrouped" in res["problems"]
    shown = render.assemble(rec, bad, None, "eng")
    assert "### Backups" not in shown and "### s — contradicted" in shown


def test_a_withdrawn_finding_left_out_of_the_groups_keeps_its_section_grouped():
    # chhoto-full 2026-10-14: the writer left the withdrawn practice question
    # out of the groups, and the whole section was shown ungrouped.
    rec = _record()
    f5 = next(f for f in rec["merged"]["findings"] if render._key(f) == "README.md#5")
    f5["disposition"] = {"disposition": "withdrawn", "note": "n"}
    classes = render.classify(rec["merged"])
    prose = {f: f"<{f}>" for f in schemas.FIELDS}
    prose["groups"] = [{"section": "shown", "title": "Backups", "findings": ["README.md#1"]}]
    prose["duplicate_questions"] = []
    res = schemas.check_prose(dict(prose, not_examined_note="x"), rec["merged"], True, True,
                              render.grouped_sections(classes), render.ungrouped_allowed(classes))
    assert res["ok"], res["problems"]
    doc = render.assemble(rec, prose, None, "eng")
    shown = doc[doc.index("## What the review showed"):doc.index("## Unsettled claims")]
    assert shown.index("### Backups") < shown.index("### Withdrawn by the practice") \
        < shown.index("#### s — true, with something")


def test_a_question_that_repeats_another_is_left_out_everywhere():
    rec = _record()
    rec["merged"]["questions"].append({"claim_source": "README.md", "claim_id": 3, "question": "where, exactly?"})
    prose = {f: f"<{f}>" for f in schemas.FIELDS}
    prose["duplicate_questions"] = [[3, 4]]
    doc = render.assemble(rec, prose, None, "eng")
    assert "where?" in doc and "where, exactly?" not in doc
    # a set that names a question twice drops nothing, and is reported
    prose["duplicate_questions"] = [[3, 4], [4]]
    assert "where, exactly?" in render.assemble(rec, prose, None, "eng")
    res = schemas.check_prose(dict(prose, not_examined_note="x"), rec["merged"], True, True)
    assert "duplicate_questions: question 4 is in two entries; no question was dropped" in res["problems"]


def test_rated_observations_get_their_own_section_and_a_material_one_is_listed():
    rec = _record()
    ev = {"form": "citation", "document": "app/c.py", "lines": [5, 5], "quote": "key", "shows": "logs the key"}
    rec["merged"]["unclaimed"] = [
        {"claim_source": "README.md", "note": "The API key is written to the log.", "evidence": ev},
        {"claim_source": "docs/CLI.md", "note": "The key is logged in plain text.", "evidence": ev},
        {"claim_source": "README.md", "note": "A volume name does not match.", "evidence": dict(ev, lines=[9, 9])}]
    rec["ratings"]["observations"] = [
        {"members": [3], "materiality": "not_material", "basis": "minor",
         "remedy": {"change": "rename it", "band": "setting"}},
        {"members": [1, 2], "materiality": "material", "basis": "a credential in logs",
         "remedy": {"change": "stop logging the key", "band": "one_place"}}]
    doc = render.assemble(rec, None, None, "eng")
    obs = doc[doc.index("## Found without a claim"):doc.index("## Unsettled claims")]
    assert obs.index("### Observation 1 — material") < obs.index("### Observation 2 — not material")
    assert "The API key is written to the log." in obs and "The key is logged in plain text." not in obs
    assert obs.count("`app/c.py`, line 5") == 1                  # the same citation once
    assert "Recorded while testing `README.md`, `docs/CLI.md`." in obs
    assert "**What would close it:** stop logging the key — scope: one place in the code." in obs
    ex = doc[doc.index("## Executive summary"):doc.index("## What this document is")]
    assert "**Observation 1** (material; the seller claimed nothing about it)" in ex
    assert "## Observations the seller did not claim" not in doc


def test_a_findings_searches_follow_what_settles_it():
    f = {"claim_source": "README.md", "claim_id": 1, "quote": "q", "lines": [1, 1], "statement": "s",
         "adjudication": {"verdict": "contradicted", "gap": "g"},
         "evidence": [{"form": "search", "kind": "lexical", "performed": "grep", "result": "none", "candidates": []},
                      {"form": "citation", "document": "a.py", "lines": [2, 2], "quote": "x", "shows": "y"}],
         "review": {"outcome": "holds"}, "citation_problems": []}
    out = "\n".join(render._finding(f, None, "materiality"))
    assert out.index("`a.py`, line 2") < out.index("searched:")


def test_a_material_finding_the_check_did_not_uphold_is_held_back_until_the_practice_retains_it():
    # Bruce, 2026-10-14: a finding the check rejected was named on the answer
    # page and in the summary, contradicting two findings that were right.
    rec = _record()
    f1 = next(f for f in rec["merged"]["findings"] if f["claim_id"] == 1)
    f1["review"] = {"outcome": "does_not_hold", "adverse_observations": ["evidence_supports"]}
    doc = render.assemble(rec)
    ex = doc[doc.index("## Executive summary"):doc.index("## What this document is")]
    assert "- **README.md, claim 1**" not in ex
    assert "Not listed: README.md, claim 1, rated material. The check did not uphold this finding" in ex
    assert "the check did not uphold this finding" in doc[doc.index("## What the review showed"):]   # still shown
    f1["disposition"] = {"disposition": "retained", "note": "n"}
    ex = render.assemble(rec)
    ex = ex[ex.index("## Executive summary"):ex.index("## What this document is")]
    assert "- **README.md, claim 1**" in ex and "Not listed" not in ex


def test_a_finding_the_practice_withdrew_is_marked_where_it_was_and_named_in_no_summary(tmp_path):
    rec = _record()
    md = tmp_path / "m"; md.mkdir()
    (md / "dispositions.json").write_text(json.dumps(
        {"README.md#1": {"disposition": "withdrawn", "note": "the shipped default turns it off"}}))
    render.apply_dispositions(rec["merged"], md)
    doc = render.assemble(rec)
    ex = doc[doc.index("## Executive summary"):doc.index("## What this document is")]
    assert "README.md, claim 1" not in ex                       # neither listed nor "Not listed"
    shown = doc[doc.index("## What the review showed"):doc.index("## Unsettled claims")]
    assert ("Practice review: withdrawn by the practice after review; it is not counted among the "
            "findings of this document. the shipped default turns it off") in shown
    assert "Withdrawn by the practice, and counted above as the review recorded them: README.md, claim 1." in doc
