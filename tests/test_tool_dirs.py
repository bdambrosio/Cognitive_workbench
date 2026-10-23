"""A scenario's `tool_dirs` adds tools to the catalog; src/tools/ keeps its names.

The contract a scenario author relies on: a tool directory (Skill.md with a
description, tool.py with react_invoke) placed in a directory the scenario
lists under `tool_dirs` is discovered at startup, and a tool there with the
name of a built-in does not replace the built-in.
"""

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src"))

from chat.tools import ToolsMixin, _TOOLS_DIR  # noqa: E402


class _Loop(ToolsMixin):
    def __init__(self, tool_dirs):
        self.character_name = "pytest_tool_dirs"
        self._tool_dirs = tool_dirs


def _make_tool(root: Path, name: str, description: str) -> None:
    d = root / name
    d.mkdir(parents=True)
    (d / "Skill.md").write_text(f"---\nname: {name}\ndescription: {description}\nargs: {{}}\n---\n")
    (d / "tool.py").write_text("def react_invoke(args, **kw):\n    return {'status': 'ok', 'text': ''}\n")


def test_a_tool_in_an_added_directory_is_discovered(tmp_path):
    _make_tool(tmp_path, "her-new-tool", "a tool she wrote")
    registry = _Loop([_TOOLS_DIR, str(tmp_path)])._discover_tools()
    assert registry["her-new-tool"]["module_path"] == str(tmp_path / "her-new-tool" / "tool.py")


def test_a_built_in_name_is_not_replaced(tmp_path):
    _make_tool(tmp_path, "exec-script", "an impostor")
    registry = _Loop([_TOOLS_DIR, str(tmp_path)])._discover_tools()
    assert registry["exec-script"]["module_path"].startswith(_TOOLS_DIR)
