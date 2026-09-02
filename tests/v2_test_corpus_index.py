"""The corpus index resolves a cited `document` by relative path, and by bare
filename only when that name is unique.

Added 2026-09-02 with the index itself. The first `check_output` keyed files
by basename, which is exact for the nine-document fixture and wrong for a
repository: ChatterMate has two `README.md`, and a basename index checked a
quote against whichever sorted last.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from workflowsv2.claims_audit.schemas import corpus_index, resolve_document  # noqa: E402


def _tree(tmp_path):
    (tmp_path / "README.md").write_text("root readme\n")
    (tmp_path / "backend" / "app").mkdir(parents=True)
    (tmp_path / "backend" / "app" / "README.md").write_text("app readme\n")
    (tmp_path / "backend" / "app" / "only.py").write_text("x = 1\n")
    (tmp_path / ".git").mkdir()
    (tmp_path / ".git" / "HEAD").write_text("ref: refs/heads/main\n")
    (tmp_path / ".env.example").write_text("KEY=\n")
    return corpus_index(tmp_path)


def test_keys_are_relative_posix_paths_and_git_is_skipped(tmp_path):
    docs = _tree(tmp_path)
    assert set(docs) == {"README.md", "backend/app/README.md",
                         "backend/app/only.py", ".env.example"}


def test_a_path_resolves_exactly(tmp_path):
    docs = _tree(tmp_path)
    assert resolve_document(docs, "backend/app/README.md") == \
        ("backend/app/README.md", None)
    assert resolve_document(docs, "./README.md") == ("README.md", None)


def test_a_unique_basename_resolves_and_a_duplicate_does_not(tmp_path):
    docs = _tree(tmp_path)
    assert resolve_document(docs, "only.py") == ("backend/app/only.py", None)
    key, why = resolve_document(docs, "readme.md")
    assert key is None and "not in the materials" in why
    # `README.md` is both an exact root path and a duplicated basename: the
    # exact match wins, which is what a citation of the root file means.
    assert resolve_document(docs, "README.md") == ("README.md", None)


def test_an_ambiguous_basename_names_the_candidates(tmp_path):
    docs = _tree(tmp_path)
    (tmp_path / "frontend").mkdir()
    (tmp_path / "frontend" / "only.py").write_text("y = 2\n")
    docs = corpus_index(tmp_path)
    key, why = resolve_document(docs, "only.py")
    assert key is None
    assert "2 files" in why and "cite the path" in why


def test_a_missing_document_is_a_problem_not_an_exception(tmp_path):
    docs = _tree(tmp_path)
    assert resolve_document(docs, None)[0] is None
    assert resolve_document(docs, "nowhere.md") == \
        (None, "document 'nowhere.md' is not in the materials")


def test_a_quote_joining_lines_of_the_cited_range_is_counted_not_failed(tmp_path):
    """METHOD §7 asks for one contiguous span. A quote that joins lines from
    inside the cited range with newlines is recorded as joined and is not a
    problem; a quote with a piece that is not at those lines still is."""
    from workflowsv2.claims_audit.schemas import check_output
    (tmp_path / "d.md").write_text(
        "1. **Vendor**\n   * sub one\n   * **Dependency:** 40%\n   * sub two\n   * **Term:** monthly\n")
    frozen = [{"id": 1}]
    def out(quote):
        return check_output({"findings": [{"claim_id": 1,
                    "adjudication": {"verdict": "real"},
                    "evidence": [{"form": "citation", "document": "d.md",
                                  "lines": [1, 5], "quote": quote,
                                  "shows": "x"}]}]}, tmp_path, "d.md", frozen)
    res = out("1. **Vendor**\n   * **Dependency:** 40%\n   * **Term:** monthly")
    assert res["ok"], res["problems"]
    assert res["figures"]["joined_quotes"] == ["finding 1 evidence 1"]
    res = out("1. **Vendor**\n   * **Dependency:** 90%")
    assert not res["ok"]
    assert "this part is not at those lines" in res["problems"][0]
    assert "Dependency: 90%" in res["problems"][0]
