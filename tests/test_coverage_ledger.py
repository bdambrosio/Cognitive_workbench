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
