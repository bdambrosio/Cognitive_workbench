"""Nothing under .git is readable or citable, by name or through a symlink."""
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from chat.subagents import code_subagent as cs                   # noqa: E402

pytestmark = pytest.mark.skipif(shutil.which("git") is None, reason="needs git")


def _repo(tmp_path):
    subprocess.run(["git", "init", "-q", str(tmp_path)], check=True)
    (tmp_path / "a.txt").write_text("line 1\nline 2\n")
    os.symlink(tmp_path / ".git" / "HEAD", tmp_path / "head_link")
    return tmp_path


def test_git_paths_are_refused_with_a_reason_and_symlinks_at_resolve(tmp_path):
    repo = _repo(tmp_path)
    assert (repo / ".git" / "HEAD").is_file()
    out = cs._tool_read(repo, ".git/HEAD", None, None)
    assert out.startswith("ERROR: read refused") and "repository metadata" in out
    assert cs._safe_resolve(repo, ".git/HEAD", must_be_file=True) is None
    assert cs._safe_resolve(repo, ".git/packed-refs") is None
    # A symlink whose target is under .git resolves into it and is refused
    # there; the by-name message does not fire, the resolve does.
    assert cs._safe_resolve(repo, "head_link", must_be_file=True) is None
    assert cs._tool_read(repo, "head_link", None, None).startswith("ERROR: read invalid")
    # Ordinary files are untouched.
    assert cs._tool_read(repo, "a.txt", None, None).startswith("OK: ")
    assert cs._safe_resolve(repo, "a.txt", must_be_file=True) is not None
    traces = tmp_path.parent / f"{tmp_path.name}_world" / "inspect_traces"
    sa = cs.CodeSubagent(repo, None, traces, mode='external')
    out = sa._tool_cite({'file': '.git/HEAD', 'start_line': 1, 'end_line': 1})
    assert out.startswith("ERROR: cite refused") and "repository metadata" in out
    assert sa._tool_cite({'file': 'a.txt', 'start_line': 1, 'end_line': 1}).startswith("OK: cited")
