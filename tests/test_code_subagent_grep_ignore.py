"""grep applies git's ignore rules only when its root is itself a checkout,
the boundary list and read already use. Added 2026-09-03: an engagement
directory is an ignored subdirectory of this repo, and ripgrep's default
made its record grep as empty."""
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from chat.subagents import code_subagent as cs                   # noqa: E402

pytestmark = pytest.mark.skipif(shutil.which("rg") is None or shutil.which("git") is None,
                                reason="needs ripgrep and git")


def _repo(tmp_path):
    subprocess.run(["git", "init", "-q", str(tmp_path)], check=True)
    (tmp_path / ".gitignore").write_text("record/\n")
    (tmp_path / "src.txt").write_text("needle in source\n")
    (tmp_path / "eng" / "record").mkdir(parents=True)
    (tmp_path / "eng" / "record" / "merged.json").write_text('{"x": "needle in record"}\n')
    return tmp_path


def test_grep_at_a_checkout_root_hides_ignored_files(tmp_path):
    repo = _repo(tmp_path)
    out = cs._tool_grep(repo, "needle", None)
    assert "src.txt" in out and "merged.json" not in out


def test_grep_below_a_checkout_root_searches_everything(tmp_path):
    repo = _repo(tmp_path)
    out = cs._tool_grep(repo / "eng", "needle", None)
    assert "record/merged.json" in out, out
