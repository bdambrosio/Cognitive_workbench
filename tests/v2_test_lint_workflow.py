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

from workflowsv2 import lint_workflow as lw                      # noqa: E402


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


@pytest.mark.xfail(reason="v1 consumer: score.py scores a text report and a \u00a79 conclusion still reads v2's METHOD. Rewritten with the downstream stages; the xfail is the reminder.",
                   strict=False)
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
    # CALLED DIRECTLY, NOT THROUGH lint(). claims_audit's contract is the
    # schema now, so lint() routes that path to check_schema_vocab and this
    # key no longer exists there. The check itself is unchanged and still
    # guards REVIEW and DELIVERY, which is what this regression pins.
    found = lw.check_block_vocab(str(doc), doc.read_text())
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


@pytest.mark.xfail(reason="v1 consumer: audit_review/runner.py cites METHOD \u00a712a still reads v2's METHOD. Rewritten with the downstream stages; the xfail is the reminder.",
                   strict=False)
def test_runner_code_references_resolve_now():
    for runner, doc in lw.RUNNERS.items():
        assert not lw.check_code_refs(runner, doc), runner


@pytest.mark.xfail(reason="v1 consumer: deliver.py's glossary uses bracketed verdicts still reads v2's METHOD. Rewritten with the downstream stages; the xfail is the reminder.",
                   strict=False)
def test_a_verdict_missing_from_the_client_glossary_is_caught(monkeypatch):
    """The delivery script explains `[delta]` and its siblings to a reader who
    has never seen METHOD. That makes the glossary a second home for a closed
    vocabulary, and a second place it can drift. Drop one and the lint must
    fail, or a buyer reads an undefined bracket."""
    import sys
    sys.path.insert(0, str(REPO / "workflowsv2" / "audit_postprocess"))
    import deliver
    short = tuple(g for g in deliver.VERDICT_GLOSS if g[0] != "[delta]")
    monkeypatch.setattr(deliver, "VERDICT_GLOSS", short)
    found = lw.check_delivery_gloss()
    assert any("[delta]" in f for f in found), found


@pytest.mark.xfail(reason="v1 consumer: deliver.py's glossary uses bracketed verdicts still reads v2's METHOD. Rewritten with the downstream stages; the xfail is the reminder.",
                   strict=False)
def test_the_client_glossary_matches_method_now():
    assert not lw.check_delivery_gloss()
