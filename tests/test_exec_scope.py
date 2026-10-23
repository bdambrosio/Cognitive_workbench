"""exec-script's sandbox shows a scoped run only what its exec_scope names.

The contract: with a scope set, a script can read the scope's read paths,
cannot read anything outside them (the rest of the checkout, the answer
keys, other engagements), and cannot read a masked path inside them; with
no scope, the whole checkout is readable as before. These run the real
bubblewrap sandbox; they are skipped where bwrap is not installed.
"""

import importlib.util
import shutil
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src"))

from chat.tools import resolve_exec_scope  # noqa: E402

pytestmark = pytest.mark.skipif(shutil.which("bwrap") is None, reason="bwrap not installed")

AGENT = "pytest_exec_scope"


def _tool():
    spec = importlib.util.spec_from_file_location(
        "_exec_script_under_test", REPO / "src/tools/exec-script/tool.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _run(script, scope=None):
    out = _tool().react_invoke({"script": script}, character_name=AGENT, exec_scope=scope)
    return out["text"]


@pytest.fixture
def target(tmp_path):
    t = tmp_path / "target"
    (t / ".git").mkdir(parents=True)
    (t / ".git" / "HEAD").write_text("HISTORY\n")
    (t / "docs").mkdir()
    (t / "docs" / "README.md").write_text("CLAIM SOURCE\n")
    (t / "main.rs").write_text("THE CODE\n")
    (tmp_path / "other").mkdir()
    (tmp_path / "other" / "key.txt").write_text("OTHER ENGAGEMENT\n")
    yield t
    shutil.rmtree(REPO / "scratch" / AGENT, ignore_errors=True)


def test_scoped_run_reads_its_target_and_nothing_else(target):
    scope = resolve_exec_scope({"read": [str(target)]}, AGENT)
    out = _run(f"cat main.rs; cat {target.parent}/other/key.txt; "
               f"cat {REPO}/measure/known_answers/chhoto.yaml", scope)
    assert "Known answers" not in out
    assert "OTHER ENGAGEMENT" not in out
    assert "THE CODE" in out


def test_masked_paths_inside_the_scope_cannot_be_read(target):
    scope = resolve_exec_scope(
        {"read": [str(target)], "mask": [str(target / ".git"), str(target / "docs/README.md")]},
        AGENT)
    out = _run("cat .git/HEAD; cat docs/README.md; cat main.rs", scope)
    assert "HISTORY" not in out
    assert "CLAIM SOURCE" not in out
    assert "THE CODE" in out


def test_materials_scope_shows_the_target_and_run_and_hides_git_and_excludes(target, tmp_path):
    run = tmp_path / "run"
    run.mkdir()
    (run / "findings.json").write_text("FINDINGS\n")
    scope = resolve_exec_scope("materials", AGENT, (target, run, ["docs/README.md"]))
    out = _run(f"cat main.rs; cat {run}/findings.json; cat .git/HEAD; cat docs/README.md; "
               f"cat {target.parent}/other/key.txt", scope)
    assert "HISTORY" not in out and "CLAIM SOURCE" not in out and "OTHER ENGAGEMENT" not in out
    assert "THE CODE" in out and "FINDINGS" in out


def test_write_paths_are_writable_and_read_paths_are_not(target, tmp_path):
    tools = tmp_path / "tools_dir"
    scope = resolve_exec_scope({"read": [str(target)], "write": [str(tools)]}, AGENT)
    _run(f"echo made > {tools}/new.py; echo changed > main.rs", scope)
    assert (tools / "new.py").read_text() == "made\n"
    assert (target / "main.rs").read_text() == "THE CODE\n"


def test_no_scope_keeps_the_whole_checkout_readable():
    out = _run("head -1 src/tools/exec-script/Skill.md")
    assert out.startswith("---")


def test_a_missing_read_path_stops_the_run(tmp_path):
    with pytest.raises(SystemExit):
        resolve_exec_scope({"read": [str(tmp_path / "absent")]}, AGENT)
