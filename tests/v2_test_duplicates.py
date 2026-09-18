"""The duplicates pass: which pairings are kept, for both relations. No
model is called."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from workflowsv2.claims_audit import duplicates as dup            # noqa: E402

ORDER = ["README.md", "site.md"]
CLAIMS = {("README.md", 1): {"id": 1}, ("README.md", 2): {"id": 2},
          ("README.md", 3): {"id": 3, "about": "document"},
          ("site.md", 1): {"id": 1}, ("site.md", 2): {"id": 2}, ("site.md", 3): {"id": 3},
          ("site.md", 4): {"id": 4, "about": "document"}}


def pair(cid, src, oid, relation="same"):
    return {"claim_id": cid, "other_source": src, "other_id": oid, "relation": relation}


def test_a_same_pairing_points_backwards_to_a_claim_that_stands():
    same, within = dup.accept(
        [pair(1, "README.md", 2),      # earlier source: kept
         pair(2, "site.md", 1),        # smaller id, but that claim is now marked: followed
         pair(3, "site.md", 3),        # itself: refused
         pair(9, "README.md", 1),      # no such new claim: refused
         pair(1, "README.md", 1)],     # second pairing for one claim: refused
        "site.md", ORDER, CLAIMS, {}, {})
    assert same == {("site.md", 1): ("README.md", 2), ("site.md", 2): ("README.md", 2)}
    assert within == {}


def test_forward_same_pairings_and_unknown_relations_are_refused():
    assert dup.accept([pair(1, "site.md", 1), pair(2, "other.md", 1), pair(1, "README.md", 2, "wider")],
                      "README.md", ORDER, CLAIMS, {}, {}) == ({}, {})
    assert dup.accept([pair(2, "README.md", 1)], "README.md", ORDER, CLAIMS, {}, {}) == \
        ({("README.md", 2): ("README.md", 1)}, {})


def test_a_claim_about_a_document_is_about_its_own_file():
    assert dup.accept([pair(4, "README.md", 3), pair(4, "README.md", 3, "new_within_earlier")],
                      "site.md", ORDER, CLAIMS, {}, {}) == ({}, {})
    assert dup.accept([pair(4, "site.md", 1)], "site.md", ORDER, CLAIMS, {}, {}) == \
        ({("site.md", 4): ("site.md", 1)}, {})


def test_within_points_either_way_and_an_absorbed_claim_absorbs_nothing():
    same, within = dup.accept(
        [pair(1, "README.md", 1, "new_within_earlier"),   # new is narrower: kept
         pair(2, "README.md", 2, "earlier_within_new"),   # earlier is narrower: kept
         pair(3, "README.md", 2, "earlier_within_new")],  # README 2 already marked: refused
        "site.md", ORDER, CLAIMS, {}, {})
    assert same == {}
    assert within == {("site.md", 1): ("README.md", 1), ("README.md", 2): ("site.md", 2)}
    # A claim marked the same as another cannot be the wider claim; the one it
    # points at is used instead.
    same2, within2 = dup.accept([pair(3, "site.md", 1, "new_within_earlier")],
                                "site.md", ORDER, CLAIMS, {("site.md", 1): ("README.md", 2)}, {})
    assert within2 == {("site.md", 3): ("README.md", 2)}
    # A claim already the same as another takes no mark of its own.
    assert dup.accept([pair(1, "README.md", 2, "new_within_earlier")],
                      "site.md", ORDER, CLAIMS, {("site.md", 1): ("README.md", 2)}, {}) == ({}, {})
