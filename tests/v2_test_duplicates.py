"""The duplicates pass: which pairings are kept. No model is called."""
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


def pair(cid, src, sid):
    return {"claim_id": cid, "same_as_source": src, "same_as_id": sid}


def test_a_pairing_points_backwards_to_a_claim_that_stands():
    got = dup.accept([pair(1, "README.md", 2),      # earlier source: kept
                      pair(2, "site.md", 1),        # smaller id, but that claim is now marked: followed
                      pair(3, "site.md", 3),        # itself: refused
                      pair(9, "README.md", 1),      # no such new claim: refused
                      pair(1, "README.md", 1)],     # second pairing for one claim: refused
                     "site.md", ORDER, CLAIMS, {})
    assert got == {("site.md", 1): ("README.md", 2), ("site.md", 2): ("README.md", 2)}


def test_forward_and_unknown_pairings_are_refused():
    assert dup.accept([pair(1, "site.md", 1), pair(2, "other.md", 1), pair(1, "README.md", 2)],
                      "README.md", ORDER, CLAIMS, {}) == {}
    assert dup.accept([pair(2, "README.md", 1)], "README.md", ORDER, CLAIMS, {}) == \
        {("README.md", 2): ("README.md", 1)}


def test_a_claim_about_a_document_is_about_its_own_file():
    assert dup.accept([pair(4, "README.md", 3)], "site.md", ORDER, CLAIMS, {}) == {}
    assert dup.accept([pair(4, "site.md", 1)], "site.md", ORDER, CLAIMS, {}) == \
        {("site.md", 4): ("site.md", 1)}
