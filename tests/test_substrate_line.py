"""Unit tests for the startup substrate line (harness provenance):
git HEAD + dirty count + commits-since-last-session via the
<memory>/substrate_marker.json roundtrip, and the no-git fallback.

Uses object.__new__(ChatLoop) with only the attributes the method
touches and a throwaway temp git repo — never the real repo or live
world state.
"""

import json
import os
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from chat.chat_loop import ChatLoop


def _git(repo, *args):
    subprocess.run(
        ["git", "-C", str(repo), "-c", "user.email=t@t", "-c", "user.name=t",
         *args],
        check=True, capture_output=True, text=True)


@pytest.fixture
def repo(tmp_path):
    r = tmp_path / "repo"
    r.mkdir()
    _git(r, "init", "-q")
    (r / "a.txt").write_text("one\n")
    _git(r, "add", "a.txt")
    _git(r, "commit", "-q", "-m", "first commit")
    return r


@pytest.fixture
def loop(tmp_path):
    inst = object.__new__(ChatLoop)
    inst.character_name = "Tester"
    inst.backend = SimpleNamespace(model="test-model-1")
    inst._memory_dir = lambda: tmp_path / "memory"
    return inst


def test_first_session_head_and_model(loop, repo, tmp_path):
    line = loop._compute_substrate_line(repo_root=repo)
    assert line.startswith("running commit ")
    assert "backend model test-model-1" in line
    assert "since my last session" not in line  # no marker yet
    marker = json.loads(
        (tmp_path / "memory" / "substrate_marker.json").read_text())
    assert marker["head"] in line


def test_same_head_reports_no_commits(loop, repo):
    loop._compute_substrate_line(repo_root=repo)
    line = loop._compute_substrate_line(repo_root=repo)
    assert "no commits since my last session" in line


def test_new_commits_listed_by_subject(loop, repo):
    loop._compute_substrate_line(repo_root=repo)
    (repo / "b.txt").write_text("two\n")
    _git(repo, "add", "b.txt")
    _git(repo, "commit", "-q", "-m", "add feature B")

    line = loop._compute_substrate_line(repo_root=repo)
    assert "1 commit(s) since my last session: add feature B" in line


def test_dirty_tree_counted(loop, repo):
    (repo / "a.txt").write_text("changed\n")
    line = loop._compute_substrate_line(repo_root=repo)
    assert "(+1 file(s) uncommitted)" in line


def test_non_repo_returns_empty(loop, tmp_path):
    plain = tmp_path / "plain"
    plain.mkdir()
    assert loop._compute_substrate_line(repo_root=plain) == ""


def test_unreachable_prev_head_omits_delta(loop, repo, tmp_path):
    loop._compute_substrate_line(repo_root=repo)
    marker_path = tmp_path / "memory" / "substrate_marker.json"
    marker_path.write_text(json.dumps({"head": "deadbeef"}))
    (repo / "c.txt").write_text("three\n")
    _git(repo, "add", "c.txt")
    _git(repo, "commit", "-q", "-m", "add C")

    line = loop._compute_substrate_line(repo_root=repo)
    # prev..HEAD fails on the bogus sha — delta clause omitted, no crash.
    assert line.startswith("running commit ")
    assert "since my last session" not in line
