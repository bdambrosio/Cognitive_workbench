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

from workflowsv2.audit_review.runner import (resolve_citations, resolve_quotes,
                                           unpointed_fields,
                                           retestable_exceptions,
                                           _confirmation_note)

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


def test_only_failures_a_second_reviewer_can_check_are_retested():
    """§9: `[unsupported]`, `[indeterminate]` and `[broken citation]` are
    retested; `[uncited]` and `[overstated]` are not.

    `[broken citation]` joined the retested set 2026-08-29. It had been excluded
    as settled by a file operation — true, but that set severity by how the
    defect was detected rather than by what it was, and a citation naming a line
    that exists but holds something else was retestable while one naming a line
    past end-of-file was fatal on sight. Two spellings of one defect.

    `[uncited]` stays out because a finding citing nothing gives a second
    reviewer no reference to open, and `[overstated]` never fails a report.
    """
    review = (
        "**Exception 1: F9 (report:120-131) — [unsupported]**\n"
        "**Exception 2: F5 (report:88) — [uncited]**\n"
        "**Exception 3: F3 (report:40-44) — [overstated]**\n"
        "**Exception 4: F7 (report:96 – 104) — [indeterminate]**\n"
        "**Exception 5: F2 (report:12-15) — [broken citation]**\n")
    assert retestable_exceptions(review) == [
        {"finding": "F9", "lines": "120-131", "verdict": "unsupported"},
        {"finding": "F7", "lines": "96-104", "verdict": "indeterminate"},
        {"finding": "F2", "lines": "12-15", "verdict": "broken citation"}]
    assert retestable_exceptions("") == []

    # THE LABEL IS NOT THE REFERENT. `F9` is the reviewer's own (REVIEW.md §4)
    # and the retest never sees the review that defined it, so an Exception
    # line carrying only a label is not actionable and must not be treated as
    # if it were. Every one of these was emitted by a real reviewer before
    # §6 required the range; each would have sent the retest after a finding
    # it could not identify.
    for unresolvable in ("**Exception 1: report Finding 7 — [unsupported]**",
                         "**Exception 1: Finding 9 — [unsupported]**",
                         "**Exception 1: F1 — [unsupported]**"):
        assert retestable_exceptions(unresolvable) == []


def test_unread_exceptions_are_counted_but_a_clean_review_is_quiet():
    """The alarm fires on format drift, never on a healthy review.

    The first version of this warned whenever no judgement fails were parsed
    — which is the normal case, since `[broken citation]` and `[uncited]` are
    retest-exempt by design. It would have fired on 11 of the 17 reviews on
    disk, all of them fine. The condition that actually means something is
    narrower: the review wrote Exception blocks and the strict §6 pattern
    read none of them.
    """
    from workflowsv2.audit_review.runner import unparsed_exceptions

    # Healthy: nothing written, nothing unread.
    assert unparsed_exceptions("Supported by their citations: 28 of 28.") == 0

    # Healthy: written in the §6 format and read.
    assert unparsed_exceptions(
        "**Exception 1: F1 (report:42-48) — [unsupported]**\n"
        "**Exception 2: F2 (report:51-60) — [broken citation]**") == 0

    # Drift: every one of these was emitted by a real reviewer, and none
    # carries a referent the retest could resolve.
    assert unparsed_exceptions(
        "**Exception 1: report Finding 7 — [uncited]**\n"
        "**Exception 2: Finding 9 — [unsupported]**\n"
        "**Exception 3: F1 — [unsupported]**") == 3

    # Mixed: only the unreadable one counts.
    assert unparsed_exceptions(
        "**Exception 1: F1 (report:42-48) — [unsupported]**\n"
        "**Exception 2: F2 — [unsupported]**") == 1


def test_a_failed_finding_stands_only_if_the_retest_agrees():
    """One retest. A single disagreement means the fail does not stand.

    A retest that returned no verdict for a finding has not agreed to fail
    it, so silence cannot uphold a fail.
    """
    from workflowsv2.audit_review.runner import (REVIEWERS_REQUIRED,
                                               CONFIRMING_REVIEWERS)
    assert (REVIEWERS_REQUIRED, CONFIRMING_REVIEWERS) == (2, 1)

    def tally(mine, others):
        agreeing = 1 + sum(1 for v in others if v == mine)
        return agreeing, agreeing == REVIEWERS_REQUIRED

    assert tally("unsupported", ["unsupported"]) == (2, True)
    assert tally("unsupported", ["supported"]) == (1, False)
    assert tally("unsupported", [None]) == (1, False)
    assert tally("indeterminate", ["indeterminate"]) == (2, True)


def test_the_summary_is_told_what_the_retest_found():
    """The runner reports the tally. §9 carries the rule.

    An exception that stands and one that does not must be distinguishable, and
    one that does not stand must survive into the review rather than vanish.

    The note says "disposition", never "verdict" or "fail". REVIEW.md §6
    reserves `verdict` for what the AUDIT concluded about a claim, and this
    note is the last thing the reviewer reads before writing §9 — the two
    wordings disagreeing there is what the reservation exists to prevent.
    """
    note = _confirmation_note({"ran": True, "results": [
        {"finding": "9", "verdict": "unsupported",
         "other_verdicts": ["supported"],
         "agreeing": 1, "of": 2, "accepted_as_failed": False},
        {"finding": "7", "verdict": "indeterminate",
         "other_verdicts": ["indeterminate"],
         "agreeing": 2, "of": 2, "accepted_as_failed": True}]})
    assert "Finding 9" in note and "1 of 2" in note and "DOES NOT STAND" in note
    assert "Finding 7" in note and "2 of 2" in note and "STANDS" in note
    assert "Do not delete an exception that does not stand" in note
    # There is no grade to report: §9 stopped returning one 2026-08-29. Every
    # exception carries a standing instead, and one that does not stand is
    # reported rather than dropped.
    assert "no grade to report" in note
    assert "stands, does not stand, or not retested" in note
    # §6's reservation, enforced where it is easiest to lose.
    assert "verdict" not in note and "fail" not in note

    # A retest that could not be run leaves the standing unknown. That is not
    # an exception that did not stand, and an infrastructure failure must not
    # clear a report.
    failed = _confirmation_note({"ran": False, "error": "boom"})
    assert "could not obtain the retest" in failed
    assert "NOT RETESTED" in failed
    assert "must not be reported as one that does not stand" in failed
    assert "STANDS" not in failed


def test_every_name_the_runner_uses_at_run_time_resolves():
    """A NameError inside the retest reads as "could not obtain the retest".

    `latest_reply` was imported inside main(), so the module-level
    confirm_exceptions could not see it. The retest raised NameError, every
    failed finding became one that did not stand, and a report with five
    [unsupported] findings came back PASS. The failure path was correct; the
    name was not there to begin with.

    Static, because the alternative needs a live model: every global a
    function reads must exist in the module or be imported in its own body.
    """
    import ast
    import builtins
    import inspect
    from workflowsv2.audit_review import runner as mod

    src = ast.parse(inspect.getsource(mod))
    problems = []
    for fn in [n for n in src.body if isinstance(n, ast.FunctionDef)]:
        local = set()
        for node in ast.walk(fn):
            if isinstance(node, (ast.Import, ast.ImportFrom)):
                local.update(a.asname or a.name.split(".")[0]
                             for a in node.names)
            elif isinstance(node, ast.Name) and isinstance(node.ctx, ast.Store):
                local.add(node.id)
            elif isinstance(node, ast.arg):
                local.add(node.arg)
            elif isinstance(node, ast.ExceptHandler) and node.name:
                local.add(node.name)          # `except X as e` binds a str
            elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                local.add(node.name)          # a nested `def` binds its name
        for node in ast.walk(fn):
            if not (isinstance(node, ast.Name)
                    and isinstance(node.ctx, ast.Load)):
                continue
            n = node.id
            if n in local or hasattr(mod, n) or hasattr(builtins, n):
                continue
            problems.append(f"{fn.name}: {n} (line {node.lineno})")
    assert not problems, "names that will not resolve at run time: " + \
        "; ".join(sorted(set(problems)))
