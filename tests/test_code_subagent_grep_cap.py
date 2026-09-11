"""A capped grep says what it did not show: every matching file with its
hit count, at most five lines per file, and a blunt note past 200 files.
An uncapped grep is unchanged."""
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from chat.subagents import code_subagent as cs                   # noqa: E402

pytestmark = pytest.mark.skipif(shutil.which("rg") is None or shutil.which("git") is None,
                                reason="needs ripgrep and git")


def _repo(tmp_path, n_files, noisy_lines=0):
    subprocess.run(["git", "init", "-q", str(tmp_path)], check=True)
    for i in range(n_files):
        (tmp_path / f"f{i:03d}.txt").write_text(f"needle {i}\nplain\n")
    if noisy_lines:
        (tmp_path / "noisy.txt").write_text("needle\n" * noisy_lines)
    return tmp_path


def _hit_lines(out):
    body = out[4:] if out.startswith("OK: ") else out
    return [ln for ln in body.splitlines() if ":" in ln and ln.split(":", 1)[1][:1].isdigit()]


def test_small_result_is_unchanged(tmp_path):
    out = cs._tool_grep(_repo(tmp_path, 3), "needle", None)
    assert out.startswith("OK: ") and len(_hit_lines(out)) == 3
    assert "capped" not in out and "Every matching file" not in out


def test_capped_result_lists_every_matching_file_with_counts(tmp_path):
    out = cs._tool_grep(_repo(tmp_path, 60, noisy_lines=30), "needle", None)
    hits = _hit_lines(out)
    assert len(hits) <= cs._MAX_GREP_HITS
    assert sum(1 for h in hits if h.startswith("noisy.txt:")) <= cs._MAX_GREP_HITS_PER_FILE
    assert "capped at 50 lines, 5 per file; 90 hits in 61 file(s) in all" in out
    assert "noisy.txt  30" in out
    listed = [ln for ln in out.splitlines() if ln.endswith("  1")]
    assert len(listed) == 60                       # every single-hit file named
    assert "too broad" not in out


def test_one_file_over_the_per_file_cap_says_so_without_claiming_a_line_cap(tmp_path):
    out = cs._tool_grep(_repo(tmp_path, 0, noisy_lines=11), "needle", None)
    assert len(_hit_lines(out)) == cs._MAX_GREP_HITS_PER_FILE
    assert "5 hits per file shown; 11 hits in 1 file(s) in all, 6 hits and 0 file(s) not shown" in out
    assert "capped at 50" not in out


def test_exactly_at_the_cap_is_not_reported_as_capped(tmp_path):
    out = cs._tool_grep(_repo(tmp_path, cs._MAX_GREP_HITS), "needle", None)
    assert len(_hit_lines(out)) == cs._MAX_GREP_HITS
    assert "Every matching file" not in out


def test_over_two_hundred_files_gets_the_blunt_note(tmp_path):
    out = cs._tool_grep(_repo(tmp_path, 230), "needle", None)
    assert "230 file(s) in all" in out
    assert "too broad to enumerate" in out
    listed = [ln for ln in out.splitlines() if ln.endswith("  1")]
    assert len(listed) == cs._MAX_GREP_FILES_LISTED
