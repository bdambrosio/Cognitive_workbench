"""The workflow linter, validated against the defects it exists to catch.

A linter written by the same author who injected the defects is worth nothing
on its author's say-so. What makes it trustworthy is that every check can be
run against a revision where the defect actually existed, and seen to fire.

Those revisions are in git with commit messages describing what they fixed, so
this suite is the regression suite that already existed and had not been used.
"""
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from workflows import lint_workflow as lw                      # noqa: E402


def _at(rev: str, path: str, tmp: Path, sub: str) -> Path:
    """That file as it stood at `rev`, written where the linter can read it.

    The destination keeps `claims_audit` or `audit_review` in its path because
    check_block_vocab picks the expected block set from the directory name.
    """
    try:
        raw = subprocess.check_output(["git", "show", f"{rev}:{path}"],
                                      cwd=REPO, text=True,
                                      stderr=subprocess.DEVNULL)
    except subprocess.CalledProcessError:
        pytest.skip(f"{rev} not in this clone")
    out = tmp / sub / Path(path).name
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(raw, encoding="utf-8")
    return out


def _problems(doc: Path) -> dict:
    return lw.lint(str(doc))


def test_both_documents_are_clean_now():
    """HEAD must pass every check. This is the assertion that regresses."""
    for path in lw.DOCS:
        for check, problems in lw.lint(path).items():
            assert not problems, f"{path}: {check}: {problems}"


def test_method_vocabularies_match_the_scorer():
    assert not lw.check_code_vocab()


def test_retired_token_check_fires_on_the_revision_that_had_it(tmp_path):
    """§9 named `Walk` three lines under the table that forbids it, and §6 named
    `non-delta` after renaming it. Fixed 032f056b."""
    doc = _at("032f056b~1", "workflows/claims_audit/method/METHOD.md",
              tmp_path, "claims_audit")
    found = _problems(doc)["retired tokens in the prompt"]
    assert any("Walk" in p for p in found), found
    assert any("non-delta" in p for p in found), found


def test_changelog_in_the_prompt_is_caught_by_the_date_check(tmp_path):
    """REVIEW.md carried no audience marker, so every incident loaded into every
    review. Fixed bb4613b9."""
    doc = _at("bb4613b9~1", "workflows/audit_review/method/REVIEW.md",
              tmp_path, "audit_review")
    assert _problems(doc)["dates in the prompt"]


def test_block_vocabulary_check_fires_before_blocks_were_specified(tmp_path):
    """`=== GAP MAP ===` was declared in two runners and specified to the agent
    in neither. Fixed 038a9dab."""
    doc = _at("038a9dab~1", "workflows/claims_audit/method/METHOD.md",
              tmp_path, "claims_audit")
    found = _problems(doc)["block vocabulary"]
    assert any("GAP MAP" in p and "never specified" in p for p in found), found


def test_cross_document_references_are_not_flagged():
    """REVIEW cites METHOD's sections legitimately. Treating "METHOD §16" as a
    dangling intra-document reference was this linter's own first false
    positive, found on its first run."""
    agent = lw.load_workflow(REPO / lw.DOCS[1])
    assert "METHOD §16" in agent
    assert not lw.lint(lw.DOCS[1])["section references"]


def test_a_dangling_reference_is_still_caught(tmp_path):
    """The tolerance above must not have disabled the check it lives in."""
    doc = tmp_path / "audit_review" / "REVIEW.md"
    doc.parent.mkdir(parents=True, exist_ok=True)
    doc.write_text("## 1. A\n\nSee §7, which does not exist.\n", encoding="utf-8")
    assert _problems(doc)["section references"]


def test_a_dead_section_reference_in_runner_code_is_caught(tmp_path, monkeypatch):
    """The class this check exists for: renaming a section is a substitution
    across the document and a silent trap in the code, because a runner emits
    §N to the agent at run time. Line 884 of audit_review/runner.py says
    "See REVIEW.md §4.0." — rename §4.0 and that sentence points nowhere."""
    fake = tmp_path / "runner.py"
    fake.write_text('MSG = "Settle admissibility first, per REVIEW.md §99."\n',
                    encoding="utf-8")
    # The documents are named rather than indexed since 2026-08-30, when a third
    # was added and positional lookup repointed every cross-check.
    method_abs = str(REPO / lw.METHOD_DOC)
    review_abs = str(REPO / lw.REVIEW_DOC)
    monkeypatch.setattr(lw, "REPO", tmp_path)
    monkeypatch.setattr(lw, "METHOD_DOC", method_abs)
    monkeypatch.setattr(lw, "REVIEW_DOC", review_abs)
    found = lw.check_code_refs("runner.py", review_abs)
    assert any("§99" in f for f in found), found


def test_runner_code_references_resolve_now():
    for runner, doc in lw.RUNNERS.items():
        assert not lw.check_code_refs(runner, doc), runner
