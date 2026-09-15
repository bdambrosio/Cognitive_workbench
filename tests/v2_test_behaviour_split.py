"""Splitting a claim about behaviour (BEHAVIOUR_SPLIT.md): the proposal call,
what it filters, and what a split does to the surface. No model: the
emission is faked."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from workflowsv2.claims_audit import behaviour_split as bs, schemas   # noqa: E402

CLAIMS = [
    {"id": 1, "quote": "MIT licensed.", "lines": [2, 2], "statement": "The licence is MIT.", "about": "document"},
    {"id": 2, "quote": "The assistant answers common customer questions on its own.", "lines": [5, 5],
     "statement": "The assistant answers common customer questions without a person.", "about": "target"},
    {"id": 3, "quote": "Hosted plans start at $10.", "lines": [7, 7], "statement": "Hosted plans start at $10.", "about": "seller"},
    {"id": 4, "quote": "Users sign in with their email address.", "lines": [9, 9],
     "statement": "Users sign in with their email address.", "about": "target"},
]


def test_propose_lists_only_eligible_claims_and_keeps_only_valid_splits(monkeypatch):
    seen = {}
    def fake_emit(loop, system, user, schema, max_tokens, salvage=None):
        seen.update(system=system, user=user, schema=schema)
        return {"obj": {"splits": [
            {"claim_id": 2, "mechanism": "A handler sends the question to a model and returns its answer.",
             "behaviour": "The answers returned resolve common customer questions."},
            {"claim_id": 3, "mechanism": "x", "behaviour": "y"},          # not eligible: about the seller
            {"claim_id": 2, "mechanism": "again", "behaviour": "again"},  # duplicate id
            {"claim_id": 4, "mechanism": "", "behaviour": "z"},           # empty statement
            {"claim_id": 99, "mechanism": "a", "behaviour": "b"}]},       # not in the batch
            "parse": "parsed", "parse_error": None, "raw": "{}"}
    monkeypatch.setattr(bs, "emit", fake_emit)
    res = bs.propose(object(), "   5|The assistant answers ...", CLAIMS)
    assert [s["claim_id"] for s in res["splits"]] == [2]
    assert "Splitting a claim about behaviour" in seen["system"] and "BEHAVIOUR_SPLIT.md" in seen["user"]
    assert "2. quote: The assistant" in seen["user"] and "4. quote: Users sign in" in seen["user"]
    assert "1. quote: MIT" not in seen["user"] and "3. quote: Hosted" not in seen["user"]
    assert seen["schema"]["required"] == ["splits"]


def test_propose_makes_no_call_when_nothing_is_eligible(monkeypatch):
    monkeypatch.setattr(bs, "emit", lambda *a, **k: (_ for _ in ()).throw(AssertionError("called")))
    res = bs.propose(object(), "", [CLAIMS[0], CLAIMS[2]])
    assert res["splits"] == [] and res["parse"] is None


def test_apply_narrows_the_parent_and_appends_a_behaviour_subclaim(tmp_path):
    claims = [dict(c) for c in CLAIMS]
    applied = bs.apply(claims, [{"claim_id": 2, "mechanism": "A handler sends the question to a model and returns its answer.",
                                 "behaviour": "The answers returned resolve common customer questions."},
                                {"claim_id": 3, "mechanism": "x", "behaviour": "y"}])
    assert [a["claim_id"] for a in applied] == [2]
    assert applied[0]["statement_before"] == "The assistant answers common customer questions without a person."
    assert claims[1]["statement"] == "A handler sends the question to a model and returns its answer."
    assert len(claims) == 5 and applied[0]["behaviour_id"] == 5
    new = claims[-1]
    assert new["implied_by"] == 2 and new["quote"] == CLAIMS[1]["quote"] and new["lines"] == [5, 5]
    assert new["property"] == bs.PROPERTY and new["approved_by"] == bs.BY and new["about"] == "target"
    # a second application does not split the subclaim or the narrowed parent again
    assert bs.apply(claims, [{"claim_id": 5, "mechanism": "m", "behaviour": "b"}]) == []
    # the surface check accepts the result
    src = tmp_path / "README.md"
    src.write_text("a\nMIT licensed.\nb\nc\nThe assistant answers common customer questions on its own.\nd\n"
                   "Hosted plans start at $10.\ne\nUsers sign in with their email address.\n")
    ok = schemas.check_surface({"claim_source": src.name, "claims": claims}, tmp_path, src.name)
    assert ok["problems"] == []          # a shared quote with its own statement is not a duplicate
    claims.append(dict(claims[-1], id=6))
    dup = schemas.check_surface({"claim_source": src.name, "claims": claims}, tmp_path, src.name)
    assert any("same quote and statement as claim 5" in p for p in dup["problems"])
