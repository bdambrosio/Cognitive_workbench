"""grep shares its line budget across matching files by distribution and
says what it did not show: totals first, "(+N more in this file)" on a file
cut short, files that got no line listed after with counts, a blunt note
past 200 such files. Three states, each marked only when true."""
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from chat.subagents import code_subagent as cs                   # noqa: E402

pytestmark = pytest.mark.skipif(shutil.which("rg") is None or shutil.which("git") is None,
                                reason="needs ripgrep and git")


def _repo(tmp_path, n_files, noisy=None):
    """n_files single-hit files f000.., plus optional {name: hits}."""
    subprocess.run(["git", "init", "-q", str(tmp_path)], check=True)
    for i in range(n_files):
        (tmp_path / f"f{i:03d}.txt").write_text(f"needle {i}\nplain\n")
    for name, hits in (noisy or {}).items():
        (tmp_path / name).write_text("needle\n" * hits)
    return tmp_path


def _body(out):
    assert out.startswith("OK: ")
    return out[4:].splitlines()


def _hits(lines):
    return [ln for ln in lines if ":" in ln and ln.split(":", 1)[1][:1].isdigit()]


def _hits_in(lines, name):
    return [ln for ln in _hits(lines) if ln.startswith(name + ":")]


def test_small_result_is_complete_and_only_carries_the_header(tmp_path):
    lines = _body(cs._tool_grep(_repo(tmp_path, 3), "needle", None))
    assert lines[0] == "3 hits across 3 file(s)"
    assert len(_hits(lines)) == 3
    assert not any("more in this file" in ln or "not shown" in ln for ln in lines)


def test_one_dense_file_gets_the_whole_budget_and_says_what_is_left(tmp_path):
    lines = _body(cs._tool_grep(_repo(tmp_path, 0, {"big.txt": 189}), "needle", None))
    assert lines[0] == "189 hits across 1 file(s)"
    assert len(_hits_in(lines, "big.txt")) == cs._MAX_GREP_HITS
    assert lines[-1] == "(+139 more in this file)"


def test_budget_is_shared_by_distribution_densest_first(tmp_path):
    noisy = {f"n{i}.txt": 12 for i in range(16)}          # 192 hits, 16 files
    lines = _body(cs._tool_grep(_repo(tmp_path, 0, noisy), "needle", None))
    assert lines[0] == "192 hits across 16 file(s)"
    hits = _hits(lines)
    assert len(hits) == 48                                 # limit 3 x 16, 2 lines unspent
    for name in noisy:
        assert len(_hits_in(lines, name)) == 3
    assert lines.count("(+9 more in this file)") == 16
    assert not any("not shown" in ln for ln in lines)


def test_more_files_than_budget_lists_the_rest_by_count(tmp_path):
    lines = _body(cs._tool_grep(_repo(tmp_path, 60, {"noisy.txt": 30}), "needle", None))
    assert lines[0] == "90 hits across 61 file(s)"
    hits = _hits(lines)
    assert len(hits) == cs._MAX_GREP_HITS
    assert hits[0].startswith("noisy.txt:")               # densest first
    assert lines[lines.index(hits[0]) + 1] == "(+29 more in this file)"
    assert "(11 matching file(s) not shown above, with hits:)" in lines
    tail = lines[lines.index("(11 matching file(s) not shown above, with hits:)") + 1:]
    assert len(tail) == 11 and all(t.endswith("  1") for t in tail)
    assert tail == sorted(tail)                            # path tie-break, deterministic


def test_exactly_at_the_budget_is_complete(tmp_path):
    lines = _body(cs._tool_grep(_repo(tmp_path, cs._MAX_GREP_HITS), "needle", None))
    assert len(_hits(lines)) == cs._MAX_GREP_HITS
    assert not any("more in this file" in ln or "not shown" in ln for ln in lines)


def test_far_too_many_files_gets_the_blunt_note(tmp_path):
    lines = _body(cs._tool_grep(_repo(tmp_path, 260), "needle", None))
    assert lines[0] == "260 hits across 260 file(s)"
    assert "(210 matching file(s) not shown above, with hits:)" in lines
    assert sum(1 for ln in lines if ln.endswith("  1")) == cs._MAX_GREP_FILES_LISTED
    assert lines[-1].startswith("(pattern matches over 200 files")
