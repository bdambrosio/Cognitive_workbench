"""Question sources: statements the buyer or the practice wrote, kept out of
the seller's materials, become a surface the audit can test."""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from workflowsv2 import engagement_state as state              # noqa: E402
from workflowsv2.claims_audit import questions, schemas        # noqa: E402

TEXT = """# Questions the buyer asked
<!-- The buyer: "Does it store passwords safely?
     And can we run it ourselves?" -->
Passwords are stored as salted hashes made with a password-hashing function.

# More
The software can be built from the repository without network access.
"""


def test_a_question_source_outside_the_target_becomes_a_tested_surface(tmp_path):
    eng = tmp_path / "e"
    (eng / "target").mkdir(parents=True)
    (eng / "target" / "README.md").write_text("A URL shortener.\n")
    (eng / "questions").mkdir()
    (eng / "questions" / "buyer.md").write_text(TEXT)
    (eng / "engagement.yaml").write_text(
        "target: target\nclaim_sources:\n  - README.md\n  - questions/buyer.md\n"
        "questions:\n  questions/buyer.md: buyer\n")
    src = "questions/buyer.md"
    f = state.claim_source_file(eng, src)
    # The seller can read every file under the target; the buyer's questions
    # are not there.
    assert not f.resolve().is_relative_to(state.target_dir(eng).resolve())

    assert questions.build(eng) == {src: 2}
    draft = json.loads((eng / "surface" / "questions_buyer_md.draft.json").read_text())
    rows = draft["claims"]
    assert [(c["statement"], c["lines"]) for c in rows] == [
        ("Passwords are stored as salted hashes made with a password-hashing function.", [4, 4]),
        ("The software can be built from the repository without network access.", [7, 7])]
    assert all(c["asked_by"] == "buyer" and c["tier"] == 1 for c in rows)
    check = schemas.check_surface(draft, state.target_dir(eng), src, src_file=f)
    assert check["ok"], check["problems"]
    # A seller source has no question kind and is read from the target.
    assert state.question_kind(eng, "README.md") is None
    assert state.claim_source_file(eng, "README.md") == state.target_dir(eng) / "README.md"


def test_an_item_that_does_not_apply_is_listed_untested_with_its_reason():
    """A standard item moved under `# Not applicable` stays in the record and
    reaches the report's list of claims not tested, with its reason."""
    from workflowsv2.claims_audit.schemas import split_by_tier
    rows = questions.claims(
        "# Credentials\nNo key is in the source.\n\n# Not applicable\n"
        "User passwords are hashed. | The software has no user accounts.\n", "standard")
    tested, untested = split_by_tier(rows)
    assert [c["statement"] for c in tested] == ["No key is in the source."]
    assert [(c["statement"], c["tier_basis"]) for c in untested] == [
        ("User passwords are hashed.",
         "Not applicable to this target: The software has no user accounts.")]


def test_the_standard_list_is_copied_under_its_version_and_a_practice_edit_survives(tmp_path):
    """The copy's name carries the list's version, which is how the report
    names it; adding it again keeps the practice's edits to the copy."""
    eng = tmp_path / "e"
    eng.mkdir()
    src = questions.add_standard(eng)
    assert src == "questions/standard-v1.md"
    copy = eng / src
    assert questions.statements(copy.read_text())
    copy.write_text("# edited\nOne statement.\n")
    assert questions.add_standard(eng) == src
    assert copy.read_text() == "# edited\nOne statement.\n"


def test_a_question_source_batch_is_not_shown_the_statements_of_other_batches(tmp_path, monkeypatch):
    """A question source's text is its statements; shown whole to a batch,
    the model adjudicated claims of the next batch too, and those claims got
    two findings each (chhoto-questions, 2026-10-01)."""
    from workflowsv2.claims_audit import runner
    src = tmp_path / "standard.md"
    src.write_text("# Q\nFirst statement.\nSecond statement.\nThird statement.\n")
    batch = questions.claims(src.read_text(), "standard")[:2]
    seen = {}

    def fake_emit(loop, method, user, schema, max_tokens, salvage=None):
        seen["user"] = user
        return {"obj": {"findings": []}, "parse": "parsed"}
    monkeypatch.setattr(runner, "emit", fake_emit)
    monkeypatch.setattr(runner, "gathered_evidence", lambda t, b: {"text": ""})
    runner.emit_findings(None, "METHOD", src, batch, [], 1000)
    assert "First statement." in seen["user"] and "Third statement." not in seen["user"]


def test_the_buyers_rating_heading_rates_the_statements_under_it():
    rows = questions.claims(
        "# Rated by the buyer: material\nCustomers can be kept apart.\n"
        "# Rated by the buyer: decisive\nThe licence is permissive.\n"
        "# Other\nRequests are rate-limited.\n", "buyer")
    assert [(c["statement"], c.get("buyer_rating")) for c in rows] == [
        ("Customers can be kept apart.", "material"),
        ("The licence is permissive.", "decisive"),
        ("Requests are rate-limited.", None)]
    import pytest
    with pytest.raises(SystemExit):
        questions.claims("# Rated by the buyer: urgent\nX.\n", "buyer")


def test_a_buyer_rating_replaces_the_models_and_is_marked_as_the_buyers():
    """The model is not asked to rate what the buyer rated at intake; the
    buyer's rating is recorded, passes the ratings check, and the report
    says whose it is."""
    from workflowsv2.audit_materiality import runner as mr, schemas as ms
    from workflowsv2.audit_report import render

    def f(cid, verdict, buyer=None):
        return {"claim_source": "questions/buyer.md", "claim_id": cid,
                "quote": "s", "lines": [cid, cid], "statement": "s", "about": "target",
                "asked_by": "buyer", "buyer_rating": buyer,
                "adjudication": {"verdict": verdict, "gap": "g"} if verdict != "unverifiable"
                else {"verdict": verdict, "unresolved_because": "not_in_the_materials"},
                "evidence": [], "review": {"outcome": "holds"}, "citation_problems": []}
    merged = {"findings": [f(1, "contradicted", "material"), f(2, "unverifiable", "decisive"),
                           f(3, "contradicted")]}
    rateable, exposable = mr.to_rate(merged)
    assert [x["claim_id"] for x in rateable] == [3] and exposable == []
    ratings = {"ratings": [{"claim_source": "questions/buyer.md", "claim_id": 3,
                            "materiality": "not_material", "basis": "model"}], "exposures": []}
    mr.add_buyer_ratings(ratings, merged)
    assert ms.check_ratings(ratings, merged)["ok"]
    by = {(r["claim_id"]): r for r in ratings["ratings"] + ratings["exposures"]}
    assert by[1]["materiality"] == "material" and by[2]["exposure"] == "decisive"
    text = "\n".join(render._finding(merged["findings"][0], by[1], "materiality"))
    assert "material (the buyer's own rating, given at intake)" in text
