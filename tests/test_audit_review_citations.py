"""Tests for the audit reviewer's citation resolver.

On 2026-08-26 a review of `m1_qwen_2` returned "supported 1 of 15, FAIL" on a
report whose findings were, 14 of 15, defensible from the materials. Three
causes, and the resolver owns two of them: it matched only `docN:NN`, so the
report's evidence half — cited by section name — produced no entries at all;
and it resolved the claim half as line numbers when the report was citing claim
ordinals, which is why four references "did not exist" in a five-line document.

These pin the two facts that stop that recurring. Quoted evidence must resolve,
and a reference past the end of its document must be visible as such, per
document, so §4.0 can tell one stray reference from a different coordinate
system.

Everything here runs against the real corpus and real reports on disk. No model
calls. Do not call score.py's trace-reading functions from a test — its
module-global `_RECORD` is set only inside `main()`.
"""

import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "src"))

from workflows.audit_review.runner import (resolve_citations, resolve_quotes,
                                           unpointed_fields)

CORPUS = REPO / "measure" / "fixtures" / "dataroom" / "corpus"

# Reports are COPIED here rather than read from a run directory. Run output is
# not tracked — measure/fixtures/dataroom/results/ and engagements/*/runs/ are
# both gitignored — so a test reading from there passes on the machine that
# made the run and errors on a fresh clone. The corpus stays tracked: it is
# fixture source, not output.
FIX = REPO / "tests" / "fixtures" / "audit_review"

# The report that produced the bad review of 2026-08-26: cites claim ordinals,
# evidence by section name. Four of its seven doc2 references name a line past
# the end of a five-line document.
ORDINALS = FIX / "m1_qwen_2_report.md"
# The current-METHOD run, which does cite line numbers and resolves clean. The
# negative control: a check that fires on this one is worse than what it checks.
LINE_NUMBERED = FIX / "w1_grok_2_report.md"
# One stray reference in eight, not a scheme mismatch — §4.0's other case.
ONE_STRAY = FIX / "m1_grok_1_report.md"

GOLDEN = FIX / "w1_grok_2_citations_golden.json"


def _scheme(report):
    return resolve_citations(report.read_text(), CORPUS)["scheme"]


def test_corpus_is_the_one_these_tests_assume():
    """doc2 really is five lines. Every assertion below rests on it."""
    doc2 = CORPUS / "doc2_tech_stack_description_as_provided_by_se.md"
    assert len(doc2.read_text().splitlines()) == 5


def test_reference_past_end_of_document_is_visible_per_document():
    s = _scheme(ORDINALS)
    assert s["line_refs_exceeding_file_length"] == 4
    assert s["consistent_with_line_numbering"] is False
    doc2 = s["by_document"]["doc2"]
    assert (doc2["refs"], doc2["exceeding"], doc2["max_cited"]) == (7, 4, 13)


def test_line_numbered_report_is_not_flagged():
    """The negative control. Without it the signal could fire on everything."""
    s = _scheme(LINE_NUMBERED)
    assert s["line_refs"] > 0
    assert s["consistent_with_line_numbering"] is True
    assert s["by_document"] == {}


def test_one_stray_reference_is_distinguishable_from_a_scheme_mismatch():
    """§4.0 divides on the rate, not the count — both cases reach it as False."""
    stray = _scheme(ONE_STRAY)["by_document"]["doc7"]
    assert stray["exceeding"] == 1 and stray["refs"] == 8
    ordinals = _scheme(ORDINALS)["by_document"]["doc2"]
    assert ordinals["exceeding"] / ordinals["refs"] > 0.5


def test_evidence_quotes_resolve_although_they_are_cited_by_section():
    """The half the old resolver could not see.

    These four were hand-checked against doc3 and doc4 on 2026-08-26. Each is
    quoted in the report under a section-named citation — `(doc4, Backups
    section)` — which `_CITE` produces no entry for.
    """
    quotes = resolve_quotes(ORDINALS.read_text(), CORPUS)
    found = {q["quote"]: q for q in quotes}
    for text in ("Status: Failures recorded for the last 21 days. "
                 "Last Successful Backup: 2026-07-30. Alerting: None "
                 "configured for backup failures.",
                 "Uptime Monitor: None.",
                 "CI/CD Pipeline: None configured.",
                 "DNS Management: Managed personally by 'dave'. "
                 "No secondary DNS provider."):
        assert text in found, f"quote not extracted: {text[:60]}"
        assert found[text]["resolved"], f"quote did not resolve: {text[:60]}"


def test_composite_quote_resolves_by_segment():
    """A quote assembled from several bullets, reordered, is not fabrication.

    The report joins four of doc3's testing bullets into one sentence in an
    order doc3 does not use. Whole-span matching calls that a miss; every fact
    in it is verbatim.
    """
    quotes = resolve_quotes(ORDINALS.read_text(), CORPUS)
    composite = next(q for q in quotes if q["quote"].startswith("Unit Tests: 12"))
    assert composite["resolved"]
    assert composite["segments"] >= 4
    assert composite["segments_found"] == composite["segments"]
    assert composite["how"] == "segments"       # found, but not contiguous


def test_report_prose_between_two_quoted_words_is_not_a_quote():
    """`"Streamlined" is not a fair description of "not present."`

    This yielded ` is not a fair description of ` as a quotation of the
    materials, from two directions at once: the old extractor scanned for an
    opening mark followed by a closing one, sliding past a pair too short to
    keep and matching from ITS closing mark to the next pair's opening one;
    and it read the whole report, including `Gap:`, where quoting a word back
    at the seller is legitimate writing rather than a citation. Marks now pair
    in document order, and quotes are read only from the evidence fields.
    """
    quotes = resolve_quotes(ORDINALS.read_text(), CORPUS)
    assert not [q for q in quotes if "fair description" in q["quote"]]
    assert all(q["resolved"] for q in quotes), \
        [q["quote"] for q in quotes if not q["resolved"]]


def test_a_quote_needs_one_document_to_hold_all_of_it():
    """Fragments from two documents, joined into one sentence, is fabrication.

    Every fragment here is verbatim — `Dyno:` from doc4, the MRR from doc1 —
    and the sentence is true of neither document. Resolving each segment
    wherever it happens to appear reports this as evidence.
    """
    report = ('Evidence (doc4): "Dyno: `standard-1x` (1GB RAM, 0.5 CPU). '
              'Blended MRR: $40,000."\n')
    quote = resolve_quotes(report, CORPUS)[0]
    assert not quote["resolved"]
    assert quote["how"] == "split"          # both found, never together
    assert quote["segments_found"] == quote["segments"]
    assert quote["documents"] == []


def test_separated_passages_of_one_document_joined_by_ellipsis_resolve():
    """Separated passages of one document, joined, resolve segment by segment.

    Not a licence to paraphrase: each passage is matched exactly. This is what
    stops a legitimate multi-passage quote being reported as fabrication.
    """
    report = ('Evidence (doc1): "loyal customer base of 120 active accounts '
              '... driven by a mix of high-value enterprise contracts"\n')
    quote = resolve_quotes(report, CORPUS)[0]
    assert quote["resolved"] and quote["how"] == "segments"
    assert quote["documents"] == ["doc1_seller_listing_description.md"]


def test_a_field_that_wraps_is_still_one_field():
    """A long Evidence quote may wrap, and `Basis:` is two lines by design.

    Reading only the label's own line truncates the quote and reports it as a
    miss. A field runs to the next blank line or the next §5 label.
    """
    report = ('Evidence (doc1): "loyal customer base of 120 active accounts '
              '...\ndriven by a mix of high-value enterprise contracts" — the '
              'listing.\n\nGap: "active" is not "paying".\n')
    quotes = resolve_quotes(report, CORPUS)
    assert len(quotes) == 1                 # Gap: prose is not evidence
    assert quotes[0]["resolved"] and quotes[0]["how"] == "segments"


def test_a_paraphrase_is_not_a_quote():
    """doc9 says "across ALL critical paths". The report dropped the word.

    Verbatim is exact. The dropped qualifier is the auditor softening a
    seller's claim while presenting it as quoted, which is the reason there is
    no near-match tier.
    """
    quotes = resolve_quotes(LINE_NUMBERED.read_text(), CORPUS)
    bad = [q for q in quotes if not q["resolved"]]
    assert [q["quote"] for q in bad] == [
        "well-documented with comprehensive test coverage across critical paths"]
    assert bad[0]["how"] == "miss"


def test_a_field_pointing_nowhere_is_counted():
    """§5: "without both, a reader cannot check the finding."

    `resolve_citations` and `resolve_quotes` score the pointers a report makes.
    Neither notices a field that makes none, so nothing counted the gap this
    test names. On a report written to a quote-only contract this reached 10 of
    33 evidence fields while the quote ratio still read 16 of 23.
    """
    unpointed = unpointed_fields(LINE_NUMBERED.read_text())
    # The one legitimate case: a field asserting that no evidence exists.
    assert len(unpointed) == 1
    assert "Cannot confirm" in unpointed[0]

    # A field naming a document and then writing prose points nowhere.
    assert unpointed_fields(
        "Evidence (doc4): Single standard-1x dyno, no read replicas.\n")
    # Either pointer alone is enough for a reader to go and look.
    assert not unpointed_fields("Evidence: doc4:5 — a single dyno.\n")
    assert not unpointed_fields('Evidence (doc4): "Dyno: `standard-1x`"\n')


def test_line_reference_half_is_unchanged_against_the_golden():
    """The quote resolver is additive. Existing `docN:NN` behaviour is pinned.

    The golden was written by the resolver as it stood before the 2026-08-26
    change, against the same report and the same corpus.
    """
    golden = json.loads(GOLDEN.read_text())
    fresh = resolve_citations(LINE_NUMBERED.read_text(), CORPUS)
    assert fresh["citations"] == golden["citations"]
    assert (fresh["total"], fresh["broken"]) == (golden["total"],
                                                 golden["broken"])
