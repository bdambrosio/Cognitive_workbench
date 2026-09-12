"""The repository map: keyed on the tree's shape, cached outside the tree,
rendered with its hash and the calibration line, and reachable as the code
subagent's `map` primitive."""
import json
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from utils import repo_map as rm                                  # noqa: E402
from chat.subagents import code_subagent as cs                    # noqa: E402

pytestmark = pytest.mark.skipif(shutil.which("git") is None, reason="needs git")


class _Backend:
    """Labels every directory it is asked about; counts its calls."""
    def __init__(self):
        self.calls = 0

    def chat(self, messages, **kw):
        self.calls += 1
        body = messages[1]['content']
        paths = [ln.split('/  (')[0] for ln in body.splitlines() if '/  (' in ln]
        return json.dumps({p: "tests" if p.endswith("tests") else "source files"
                           for p in paths})


def _repo(tmp_path):
    subprocess.run(["git", "init", "-q", str(tmp_path)], check=True)
    (tmp_path / ".gitignore").write_text("ignored/\n")
    (tmp_path / "src").mkdir()
    (tmp_path / "src" / "main.py").write_text("print(1)\nprint(2)\n")
    (tmp_path / "src" / "tests").mkdir()
    (tmp_path / "src" / "tests" / "test_a.py").write_text("assert True\n")
    (tmp_path / "ignored").mkdir()
    (tmp_path / "ignored" / "junk.txt").write_text("x\n")
    (tmp_path / "README.md").write_text("# hi\n")
    return tmp_path


def test_listing_follows_git_and_hash_tracks_the_shape(tmp_path):
    repo = _repo(tmp_path)
    entries = rm.list_tree(repo)
    assert [p for p, _ in entries] == [".gitignore", "README.md", "src/main.py",
                                       "src/tests/test_a.py"]
    h1 = rm.tree_hash(entries)
    (repo / "src" / "main.py").write_text("print(1)\n")
    assert rm.tree_hash(rm.list_tree(repo)) != h1
    (repo / "src" / "main.py").write_text("print(1)\nprint(2)\n")
    assert rm.tree_hash(rm.list_tree(repo)) == h1


def test_get_map_caches_by_hash_and_renders_with_hash_and_calibration(tmp_path):
    repo = _repo(tmp_path)
    cache = tmp_path.parent / f"{tmp_path.name}_cache"
    be = _Backend()
    m = rm.get_map(repo, cache, be)
    assert be.calls == 1
    assert m['dirs']['src']['role'] == "source files"
    assert m['dirs']['src/tests']['role'] == "tests"
    assert m['dirs']['']['n_files'] == 4 and m['dirs']['src']['n_lines'] == 3
    assert (cache / f"{m['tree_hash']}.json").is_file()
    assert not list(repo.rglob("*.json"))          # nothing written in the tree
    rm.get_map(repo, cache, be)
    assert be.calls == 1                              # a hit costs no call
    (repo / "new.txt").write_text("n\n")
    m2 = rm.get_map(repo, cache, be)
    assert be.calls == 2 and m2['tree_hash'] != m['tree_hash']
    text = rm.render_map(m2)
    assert m2['tree_hash'] in text and "ORGANIZED" in text
    assert "never where not to look" in text
    assert "src/  (2 files, 3 lines)" in text and "role: source files" in text
    assert "README.md" in text
    sub = rm.render_map(m2, "src")
    assert "main.py" in sub and "src/tests/" in sub
    assert rm.render_map(m2, "nope").startswith("ERROR:")


def test_role_shape_is_clipped_not_judged():
    assert rm._clean_role("  route   handlers. ") == "route handlers"
    assert rm._clean_role("one two three four five six") == "one two three four"
    assert rm._clean_role("") is None and rm._clean_role(3) is None


def test_map_is_the_first_primitive_and_answers(tmp_path):
    repo = _repo(tmp_path)
    traces = tmp_path.parent / f"{tmp_path.name}_world" / "inspect_traces"
    sa = cs.CodeSubagent(repo, _Backend(), traces, mode='external')
    assert list(sa.primitives())[0] == 'map'
    assert sa.map_cache_dir == traces.parent / 'repo_maps'
    out = sa._tool_map({})
    assert out.startswith("OK: Repository map:") and "role: tests" in out
    assert sa._tool_map({'path': 'src'}).startswith("OK: ")
    assert sa._tool_map({'path': '../'}).startswith("ERROR:")
    assert "map" in sa.system_prompt() and "call `map` first" in sa.system_prompt()


def test_map_can_be_switched_off_for_a_run(tmp_path):
    """The claims workflow runs with `subagent_map: false`: no primitive, no
    tool entry, no sentence sending the model to it; Jill's default keeps all
    three."""
    repo = _repo(tmp_path)
    traces = tmp_path.parent / f"{tmp_path.name}_world2" / "inspect_traces"
    off = cs.CodeSubagent(repo, _Backend(), traces, mode='external', map_enabled=False)
    assert 'map' not in off.primitives()
    p = off.system_prompt()
    assert '"tool": "map"' not in p and "call `map` first" not in p
    assert "list the root and read README.md" in p
    assert p.count("\n1. {") == 1 and '"tool": "list"' in p.split("\n1. {")[1][:60]
    on = cs.CodeSubagent(repo, _Backend(), traces, mode='external')
    q = on.system_prompt()
    assert '"tool": "map"' in q and "call `map` first" in q
    assert "list the root and read README.md" not in q
