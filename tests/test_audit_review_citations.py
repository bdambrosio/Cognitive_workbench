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

from workflows.audit_review.runner import resolve_citations, resolve_quotes

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


def test_quotes_that_are_not_quotations_do_not_resolve():
    """Report prose captured between two quoted words. Known noise, not a bug."""
    quotes = resolve_quotes(ORDINALS.read_text(), CORPUS)
    prose = next(q for q in quotes
                 if q["quote"] == "is not a fair description of")
    assert not prose["resolved"] and prose["how"] == "miss"


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
