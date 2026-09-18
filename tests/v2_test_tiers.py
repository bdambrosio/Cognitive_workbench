"""Rating claims into tiers (TIERS.md) against a reliance statement
(RELIANCE.md): the two proposal calls, what each filters, and how the
statement reaches the tier call. No model: the emission is faked."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from workflowsv2.claims_audit import reliance, tiers   # noqa: E402

CLAIMS = [
    {"id": 1, "quote": "No tracking.", "statement": "Visitors are not tracked.", "about": "target"},
    {"id": 2, "quote": "page_size defaults to 10.", "statement": "page_size defaults to 10.", "about": "target"},
    {"id": 3, "quote": "MIT licensed docs.", "statement": "The manual is MIT licensed.", "about": "document"},
]


def _fake(obj, seen):
    def fake_emit(loop, system, user, schema, max_tokens, salvage=None):
        seen.update(system=system, user=user, schema=schema)
        return {"obj": obj, "parse": "parsed", "parse_error": None, "raw": "{}"}
    return fake_emit


def test_tier_propose_keeps_valid_entries_and_names_the_unrated(monkeypatch):
    seen = {}
    monkeypatch.setattr(tiers, "emit", _fake({"tiers": [
        {"claim_id": 1, "tier": 1, "basis": "Privacy is named. Rated on the scale alone."},
        {"claim_id": 1, "tier": 3, "basis": "a second entry for the same claim"},
        {"claim_id": 2, "tier": 2, "basis": ""},                  # empty basis
        {"claim_id": 3, "tier": 4, "basis": "not a tier"},
        {"claim_id": 99, "tier": 1, "basis": "not in the batch"}]}, seen))
    res = tiers.propose(object(), "Client: a buyer", "Paying for: privacy", "README.md", CLAIMS)
    assert [(t["claim_id"], t["tier"]) for t in res["tiers"]] == [(1, 1)]
    assert res["unrated"] == [2, 3]
    assert "Tiers" in seen["system"] and "TIERS.md §7" in seen["user"]
    assert "Paying for: privacy" in seen["user"] and "2. about: target" in seen["user"]
    assert "The reliance statement" not in seen["user"]


def test_tier_propose_passes_the_reliance_statement_when_there_is_one(monkeypatch):
    seen = {}
    monkeypatch.setattr(tiers, "emit", _fake({"tiers": []}, seen))
    text = reliance.render({"use": "One instance per customer.", "items": [
        {"item": "Hit count only", "reliance": "depends", "if_it_failed": "The plan ends.",
         "source": "buyer", "buyer_words": "nothing beyond a count of hits"},
        {"item": "QR codes", "reliance": "does_not_use", "if_it_failed": "Nothing.",
         "source": "inference", "buyer_words": ""}]})
    tiers.propose(object(), "", "", "README.md", CLAIMS, text)
    assert "The reliance statement:\n\nOne instance per customer." in seen["user"]
    assert '- Hit count only — depends (the buyer said: "nothing beyond a count of hits"). If it failed: The plan ends.' in seen["user"]
    assert "- QR codes — does_not_use (the practice's inference)." in seen["user"]


def test_reliance_propose_drops_items_outside_the_schema(monkeypatch):
    seen = {}
    monkeypatch.setattr(reliance, "emit", _fake({"use": " The buyer hosts it. ", "items": [
        {"item": "The API", "reliance": "depends", "if_it_failed": "Costly.", "source": "buyer", "buyer_words": "the API"},
        {"item": "", "reliance": "uses", "if_it_failed": "x", "source": "buyer", "buyer_words": ""},
        {"item": "Helm chart", "reliance": "ignores", "if_it_failed": "x", "source": "buyer", "buyer_words": ""},
        {"item": "Dark mode", "reliance": "does_not_use", "if_it_failed": "x", "source": "guess", "buyer_words": ""}]}, seen))
    res = reliance.propose(object(), "The intake form:\n\n{}", "README.md (1 claims):\n  - It shortens links.")
    assert res["use"] == "The buyer hosts it." and [x["item"] for x in res["items"]] == ["The API"]
    assert "Reliance statement" in seen["system"] and "It shortens links." in seen["user"]
