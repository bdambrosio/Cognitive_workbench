"""The coverage figures are arithmetic over the ledger, and the checks run.

The second half matters as much as the first. Both post-run checks were inline
against a variable assigned later in the function, so every run since they were
added raised UnboundLocalError and skipped them — silently, because each is
wrapped so a failed check cannot lose a report. The citation check had never
executed once. These tests call the checks directly, which is only possible
because they take what they read as arguments.
"""
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from workflowsv2 import coverage as cov                       # noqa: E402
from workflowsv2.claims_audit.runner import post_run_checks   # noqa: E402

SURFACE = "=== CLAIM SURFACE ===\n4 claims\n\n1. a\n2. b\n3. c\n4. d\n=== END CLAIM SURFACE ==="
LEDGER = ("=== COVERAGE ===\n1. [real]\n2. [real, minor caveat]\n"
          "3. [partial]\n4. [unverifiable]\n\nWhy 4 could not be settled.\n"
          "=== END COVERAGE ===")


def test_every_figure_is_arithmetic_over_the_ledger():
    c = cov.check(SURFACE, LEDGER)
    f = c["figures"]
    assert c["ok"], c["problems"]
    assert (f["identified"], f["resolved"], f["supported"]) == (4, 3, 2)
    assert f["unverifiable"] == 1
    assert cov.statement(f) == ("Coverage: 3 of 4 identified claims resolved; "
                               "2 of 3 resolved claims supported.")


def test_a_claim_counted_twice_is_named_not_merely_detected():
    """'The numbers disagree' sends a person back through the whole report.
    Claim 31 on the ChatterMate medium run was counted once as a [partial]
    finding and again as '31-poll' among the supported claims."""
    doubled = LEDGER.replace("4. [unverifiable]", "4. [unverifiable]\n3. [real]")
    c = cov.check(SURFACE, doubled)
    assert not c["ok"]
    assert c["duplicated"] == [3]
    assert any("more than once" in p and "3" in p for p in c["problems"])


def test_a_claim_missing_from_the_ledger_is_named():
    c = cov.check(SURFACE, LEDGER.replace("2. [real, minor caveat]\n", ""))
    assert not c["ok"] and c["missing"] == [2]


def test_a_verdict_outside_method_6_is_caught():
    c = cov.check(SURFACE, LEDGER.replace("[partial]", "[mostly true]"))
    assert not c["ok"] and c["bad_verdicts"] == ["mostly true"]


# --- The runner's post-run check, over the v2 output --------------------------
#
# These pin the regression that motivated extracting the checks in the first
# place: inline code referencing a variable assigned twenty lines later,
# wrapped so the failure was silent. The checks now read typed output instead
# of a transcript, and "did they run at all" is the same question.

def _corpus(tmp_path):
    d = tmp_path / "corpus"
    d.mkdir()
    (d / "src.md").write_text("alpha claim one\nbeta claim two\n")
    (d / "ev.md").write_text("the evidence line\n")
    return d


_FROZEN = [{"id": 1, "quote": "alpha claim one", "lines": [1, 1],
            "statement": "one"},
           {"id": 2, "quote": "beta claim two", "lines": [2, 2],
            "statement": "two"}]


def _finding(cid):
    return {"claim_id": cid,
            "adjudication": {"verdict": "contradicted", "gap": "g"},
            "evidence": [{"form": "citation", "document": "ev.md",
                          "lines": [1, 1], "quote": "the evidence line",
                          "shows": "s"}]}


def test_the_checks_actually_execute_on_v2_output(tmp_path):
    corpus = _corpus(tmp_path)
    obj = {"claim_source": "src.md", "findings": [_finding(1), _finding(2)]}
    got = post_run_checks(obj, corpus, "src.md", _FROZEN, tmp_path)
    assert got is not None, "the output check did not run"
    assert got["ok"], got["problems"]
    assert got["figures"]["frozen_claims"] == 2
    assert got["figures"]["adjudicated"] == 2


def test_a_claim_with_no_finding_is_written_to_the_issue_log(tmp_path):
    """The check freezing buys: v1 could not tell an unenumerated claim from
    an unadjudicated one, because one pass produced both."""
    from workflowsv2 import issues
    corpus = _corpus(tmp_path)
    obj = {"claim_source": "src.md", "findings": [_finding(1)]}
    got = post_run_checks(obj, corpus, "src.md", _FROZEN, tmp_path)
    assert not got["ok"]
    assert any("claim 2" in p and "no finding" in p for p in got["problems"]), \
        got["problems"]
    rows = issues.read(tmp_path)
    assert any(r["code"] == "output_check" and r["severity"] == "blocking"
               for r in rows), rows


def test_an_unparseable_emission_is_not_silently_clean(tmp_path):
    got = post_run_checks(None, _corpus(tmp_path), "src.md", _FROZEN, tmp_path)
    assert not got["ok"] and got["problems"]




# --- Every claim in the ledger is backed by a finding -------------------------
#
# `check` above proves the LEDGER accounts for the surface. These prove no
# outcome was asserted without grounds. A finding is the statement of the
# verdict of adjudicating cited evidence with respect to a claim, so a claim
# has one verdict and therefore one finding — an [unverifiable] claim included,
# since METHOD §6 makes that a finding like any other outcome.

REPORT = ("=== REPORT ===\n"
          "**Finding 1 (claim 1): a holds — [real]**\n\n"
          "**Finding 2 (claim 2): b holds with a caveat — [real, minor caveat]**\n\n"
          "**Finding 3 (claim 3): c partly holds — [partial]**\n\n"
          "**Finding 4 (claim 4): d cannot be settled — [unverifiable]**\n\n"
          "=== END REPORT ===")


def test_a_report_that_backs_every_claim_passes():
    c = cov.findings_check(REPORT, LEDGER)
    assert c["ok"], c["problems"]
    assert (c["ledger_claims"], c["claim_findings"]) == (4, 4)


def test_an_unverifiable_claim_owes_a_finding_like_any_other_outcome():
    """It was a defect to state one as a finding until 2026-08-31; §5's absence
    pattern had always described exactly that, and §6 denied it. Now the denial
    is gone and the omission is the defect."""
    c = cov.findings_check(
        REPORT.replace("**Finding 4 (claim 4): d cannot be settled — [unverifiable]**\n\n", ""),
        LEDGER)
    assert not c["ok"] and c["missing"] == [4]


def test_a_resolved_claim_named_by_no_finding_is_named():
    """Claim 29 on cm_ledger_glm_1 carried a ledger verdict and appeared in no
    finding. The fixture run of 2026-08-31 did the same to claim 19 — adjudicated
    [partial] in the coverage block's prose, with grounds, and given no finding."""
    c = cov.findings_check(
        REPORT.replace("**Finding 2 (claim 2): b holds with a caveat — [real, minor caveat]**\n\n", ""),
        LEDGER)
    assert not c["ok"] and c["missing"] == [2]


def test_one_finding_may_adjudicate_several_claims_that_share_a_verdict():
    """One determination, however many claims it settles: `Finding 1 (claims
    19, 20)` over a single body of backup evidence."""
    ledger = LEDGER.replace("2. [real, minor caveat]", "2. [real]")
    merged = REPORT.replace(
        "**Finding 1 (claim 1): a holds — [real]**\n\n"
        "**Finding 2 (claim 2): b holds with a caveat — [real, minor caveat]**",
        "**Finding 1 (claims 1-2): a and b hold — [real]**")
    c = cov.findings_check(merged, ledger)
    assert c["ok"], c["problems"]


def test_a_finding_over_claims_whose_verdicts_differ_is_caught():
    """The defect the fixture smoke run shipped with: Finding 1 adjudicated
    claims 19 and 20 together as [delta] while the ledger recorded claim 20 as
    [partial], so one claim's verdict was misrepresented in the report body."""
    merged = REPORT.replace(
        "**Finding 2 (claim 2): b holds with a caveat — [real, minor caveat]**\n\n"
        "**Finding 3 (claim 3): c partly holds — [partial]**",
        "**Finding 2 (claims 2-3): b and c — [real, minor caveat]**")
    c = cov.findings_check(merged, LEDGER)
    assert not c["ok"]
    assert c["verdict_mismatch"] == [(3, "real, minor caveat", "partial")]


def test_two_findings_adjudicating_one_claim_is_a_defect():
    """A claim has one verdict, so it has one finding. Two are redundant or
    contradictory, and neither is a state an audit should express."""
    twice = REPORT.replace("=== END REPORT ===",
                           "**Finding 5 (claim 3): more evidence — [partial]**\n\n=== END REPORT ===")
    c = cov.findings_check(twice, LEDGER)
    assert not c["ok"] and c["duplicated"] == [3]


def test_a_header_verdict_that_contradicts_the_ledger_is_caught():
    """Claim 2 read [real] in its header and [real, minor caveat] in the ledger
    on cm_ledger_glm_1. The ledger line is the index of the verdict, not a
    second copy, so a disagreement is a defect rather than a tie."""
    c = cov.findings_check(
        REPORT.replace("b holds with a caveat — [real, minor caveat]",
                       "b holds — [real]"), LEDGER)
    assert not c["ok"]
    assert c["verdict_mismatch"] == [(2, "real", "real, minor caveat")]


def test_a_finding_naming_a_claim_outside_the_ledger_is_named():
    c = cov.findings_check(
        REPORT.replace("=== END REPORT ===",
                       "**Finding 5 (claim 9): stray — [real]**\n\n=== END REPORT ==="),
        LEDGER)
    assert not c["ok"] and c["unknown"] == [9]


def test_a_derived_statement_is_evidence_and_carries_no_header():
    """A derivation sits in a finding's Evidence (METHOD §5), so nothing new
    wears a `Finding N — [derived]` bracket. The filter stays for the 28 reports
    on disk that predate that."""
    legacy = REPORT.replace("=== END REPORT ===",
                            "**Finding 5: a consequence — [derived]**\n\n=== END REPORT ===")
    c = cov.findings_check(legacy, LEDGER)
    assert c["ok"], c["problems"]
    assert c["untraceable"] == [] and c["claim_findings"] == 4


def test_a_finding_naming_no_claim_is_named():
    c = cov.findings_check(
        REPORT.replace("**Finding 3 (claim 3): c partly holds — [partial]**",
                       "**Finding 3: c partly holds — [partial]**"), LEDGER)
    assert not c["ok"] and c["untraceable"] == [3] and c["missing"] == [3]


def test_a_title_parenthesis_is_not_read_as_a_claim_list():
    c = cov.findings_check(
        REPORT.replace("c partly holds", "builder (drag-and-drop) partly holds"),
        LEDGER)
    assert c["ok"], c["problems"]


def test_a_finding_naming_an_unfrozen_claim_is_caught(tmp_path):
    """A finding may only adjudicate a claim the frozen surface holds."""
    corpus = _corpus(tmp_path)
    obj = {"claim_source": "src.md",
           "findings": [_finding(1), _finding(2), _finding(7)]}
    got = post_run_checks(obj, corpus, "src.md", _FROZEN, tmp_path)
    assert not got["ok"]
    assert any("claim_id 7" in p for p in got["problems"]), got["problems"]



# --- Quote normalisation: decoration is ignored, meaning is not ---------------
#
# The check's first run called seven of twelve findings on doc9 fabricated,
# and every one was a faithful quote of `*   **Dyno:** ...` written down as
# `Dyno: ...`. The risk in fixing that is the other direction — normalising
# until a wrong quote matches — so both directions are pinned here.

def _md_corpus(tmp_path):
    d = tmp_path / "corpus"
    d.mkdir()
    (d / "src.md").write_text("alpha claim one\n")
    (d / "doc4.md").write_text(
        "**Backups:**\n"
        "*   **Schedule:** Daily at 2:00 AM via `heroku pg:backups schedule`\n"
        "*   **Status:** Failures recorded for the last 21 days.\n"
        "1.  **Acme Retail**\n"
        "    *   **Value:** $8,000/mo\n")
    return d


def _obj(quote, lines):
    return {"claim_source": "src.md", "findings": [{
        "claim_id": 1, "adjudication": {"verdict": "contradicted", "gap": "g"},
        "evidence": [{"form": "citation", "document": "doc4.md",
                      "lines": lines, "quote": quote, "shows": "s"}]}]}


_ONE = [{"id": 1, "quote": "alpha claim one", "lines": [1, 1],
         "statement": "one"}]


def test_a_quote_stripped_of_markdown_still_matches(tmp_path):
    corpus = _md_corpus(tmp_path)
    got = post_run_checks(
        _obj("Schedule: Daily at 2:00 AM via heroku pg:backups schedule",
             [2, 2]), corpus, "src.md", _ONE, tmp_path)
    assert got["ok"], got["problems"]


def test_a_quote_spanning_an_ordered_list_marker_still_matches(tmp_path):
    """The join asymmetry: line-anchored markers stripped from the quote and
    not from a source span joined with spaces."""
    corpus = _md_corpus(tmp_path)
    got = post_run_checks(
        _obj("Acme Retail\n    *   **Value:** $8,000/mo", [4, 5]),
        corpus, "src.md", _ONE, tmp_path)
    assert got["ok"], got["problems"]


def test_a_quote_that_says_something_else_still_fails(tmp_path):
    corpus = _md_corpus(tmp_path)
    got = post_run_checks(
        _obj("Schedule: Hourly at 2:00 AM", [2, 2]),
        corpus, "src.md", _ONE, tmp_path)
    assert not got["ok"]
    assert any("quote is not at" in p for p in got["problems"]), got["problems"]


def test_a_quote_from_the_wrong_lines_is_still_located(tmp_path):
    """Right document, wrong line range — the message must say it is elsewhere,
    not that it is absent, or a reader chases the wrong defect."""
    corpus = _md_corpus(tmp_path)
    got = post_run_checks(
        _obj("Status: Failures recorded for the last 21 days.", [2, 2]),
        corpus, "src.md", _ONE, tmp_path)
    assert not got["ok"]
    assert any("elsewhere in that document" in p for p in got["problems"]), \
        got["problems"]
