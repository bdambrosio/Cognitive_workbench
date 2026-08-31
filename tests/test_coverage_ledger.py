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

from workflows import coverage as cov                       # noqa: E402
from workflows.claims_audit.runner import post_run_checks   # noqa: E402

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


def test_an_unattempted_claim_is_a_valid_ledger_status():
    """Section 1a and Section 12 step 5 both permit the work to stop with claims
    unexamined, while Section 16 requires every frozen claim on exactly one
    ledger line. Before `[unattempted]` existed a run that stopped early could
    not write a conforming ledger at all."""
    stopped = LEDGER.replace("3. [partial]", "3. [unattempted]")
    c = cov.check(SURFACE, stopped)
    assert c["ok"], c["problems"]
    f = c["figures"]
    assert (f["resolved"], f["unattempted"], f["unverifiable"]) == (2, 1, 1)


def test_an_unattempted_claim_owes_no_finding():
    """It reached no verdict, so nothing is asserted about it to back."""
    stopped = LEDGER.replace("3. [partial]", "3. [unattempted]")
    c = cov.findings_check(REPORT.replace(
        "**Finding 3 (claim 3): c title — [partial]**\n\n", ""), stopped)
    assert c["ok"], c["problems"]


def test_a_verdict_outside_method_6_is_caught():
    c = cov.check(SURFACE, LEDGER.replace("[partial]", "[mostly true]"))
    assert not c["ok"] and c["bad_verdicts"] == ["mostly true"]


def test_the_checks_actually_execute_on_a_delivered_transcript(tmp_path):
    """The regression that motivated extracting them: inline code referencing a
    variable assigned twenty lines later, wrapped so the failure was silent."""
    whole = SURFACE + "\n\n=== REPORT ===\nbody\n=== END REPORT ===\n\n" + LEDGER
    got = post_run_checks(tmp_path, whole, str(tmp_path))
    assert got["coverage"] is not None, "the coverage check did not run"
    assert got["coverage"]["ok"], got["coverage"]["problems"]
    assert got["citations"] is not None, "the citation check did not run"


def test_a_bad_ledger_is_written_to_the_issue_log(tmp_path):
    from workflows import issues
    whole = SURFACE + "\n\n" + LEDGER.replace("3. [partial]", "3. [partial]\n3. [real]")
    post_run_checks(tmp_path, whole, str(tmp_path))
    rows = issues.read(tmp_path)
    assert any(r["code"] == "coverage_ledger" and r["severity"] == "blocking"
               for r in rows), rows



# --- Every claim with a verdict is backed by a finding ------------------------
#
# `check` above proves the LEDGER accounts for the surface. These prove no
# verdict was asserted without evidence. They test traceability and NOT ratios:
# a finding may bear on several claims and a claim on several findings, which
# is the shape of the work rather than a defect.

REPORT = ("=== REPORT ===\n"
          "**Finding 1 (claim 1): a title — [real]**\n\n"
          "**Finding 2 (claim 2): b title — [real, minor caveat]**\n\n"
          "**Finding 3 (claim 3): c title — [partial]**\n\n"
          "=== END REPORT ===")


def test_a_report_that_backs_every_resolved_claim_passes():
    c = cov.findings_check(REPORT, LEDGER)
    assert c["ok"], c["problems"]
    assert (c["resolved"], c["claim_findings"]) == (3, 3)


def test_a_resolved_claim_named_by_no_finding_is_named():
    """Claim 29 carried [real, minor caveat] in the ledger and appeared in no
    finding, so nothing cited it and nothing noticed."""
    c = cov.findings_check(
        REPORT.replace("**Finding 2 (claim 2): b title — [real, minor caveat]**\n\n", ""),
        LEDGER)
    assert not c["ok"] and c["missing"] == [2]


def test_one_finding_may_bear_on_several_claims():
    """Finding 8 resolved the seven SQL-guardrail claims from one body of
    evidence. A finding is a relevant subset of the source, and the mapping to
    claims is many-to-many; an earlier version of this check called it a defect."""
    merged = REPORT.replace(
        "**Finding 2 (claim 2): b title — [real, minor caveat]**\n\n"
        "**Finding 3 (claim 3): c title — [partial]**",
        "**Finding 2 (claims 2-3): b and c — [real]**")
    c = cov.findings_check(merged, LEDGER)
    assert c["ok"], c["problems"]


def test_one_claim_may_be_carried_by_several_findings():
    """Claims 13 and 15 sat in Finding 8's range and again in Finding 9's list.
    Two findings bearing on one claim is evidence, not duplication."""
    twice = REPORT.replace("=== END REPORT ===",
                           "**Finding 4 (claim 3): more evidence — [partial]**\n\n=== END REPORT ===")
    c = cov.findings_check(twice, LEDGER)
    assert c["ok"], c["problems"]


def test_a_finding_naming_a_claim_outside_the_ledger_is_named():
    c = cov.findings_check(
        REPORT.replace("=== END REPORT ===",
                       "**Finding 4 (claim 9): stray — [real]**\n\n=== END REPORT ==="),
        LEDGER)
    assert not c["ok"] and c["unknown"] == [9]


def test_an_unverifiable_claim_stated_as_a_finding_is_named():
    """METHOD §6 reports it in the coverage statement. Finding 7 stated claims
    4, 32 and 33 as a numbered finding; the reviewer caught it by reading."""
    c = cov.findings_check(
        REPORT.replace("=== END REPORT ===",
                       "**Finding 4 (claim 4): tried — [unverifiable]**\n\n=== END REPORT ==="),
        LEDGER)
    assert not c["ok"] and c["unresolved_as_finding"] == [4]


def test_a_derived_finding_names_no_claim_and_owes_none():
    c = cov.findings_check(
        REPORT.replace("=== END REPORT ===",
                       "**Finding 4: a consequence — [derived]**\n\n=== END REPORT ==="),
        LEDGER)
    assert c["ok"], c["problems"]
    assert c["untraceable"] == [] and c["claim_findings"] == 3


def test_a_claim_finding_naming_no_claim_is_named():
    c = cov.findings_check(
        REPORT.replace("**Finding 3 (claim 3): c title — [partial]**",
                       "**Finding 3: c title — [partial]**"), LEDGER)
    assert not c["ok"] and c["untraceable"] == [3] and c["missing"] == [3]


def test_a_title_parenthesis_is_not_read_as_a_claim_list():
    c = cov.findings_check(
        REPORT.replace("**Finding 3 (claim 3): c title — [partial]**",
                       "**Finding 3 (claim 3): builder (drag-and-drop) — [partial]**"),
        LEDGER)
    assert c["ok"], c["problems"]


def test_the_findings_check_runs_and_logs_a_blocking_issue(tmp_path):
    from workflows import issues
    whole = SURFACE + "\n\n" + REPORT.replace(
        "**Finding 2 (claim 2): b title — [real, minor caveat]**\n\n", "") + "\n\n" + LEDGER
    got = post_run_checks(tmp_path, whole, str(tmp_path))
    assert got["findings"] is not None, "the findings check did not run"
    assert not got["findings"]["ok"]
    rows = issues.read(tmp_path)
    assert any(r["code"] == "findings_ledger" and r["severity"] == "blocking"
               for r in rows), rows
