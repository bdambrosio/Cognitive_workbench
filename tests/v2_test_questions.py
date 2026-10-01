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
