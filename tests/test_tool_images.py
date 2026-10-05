"""A tool's image reaching the model's visual input.

    python3 -m pytest tests/test_tool_images.py -q
"""
import base64
import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src"))

from chat.tools import ToolsMixin                                     # noqa: E402

PNG = b"\x89PNG\r\n\x1a\n" + b"not a real picture"


def _generate_image(monkeypatch, tmp_path, args):
    spec = importlib.util.spec_from_file_location(
        "_generate_image_under_test", REPO / "src/tools/generate-image/tool.py")
    tool = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tool)
    monkeypatch.setattr(tool, "_ensure_server", lambda: True)
    monkeypatch.setattr(tool, "_OUT_DIR", tmp_path)
    monkeypatch.setattr(tool.requests, "post", lambda *a, **k: SimpleNamespace(
        status_code=200, content=PNG, text="", headers={"content-type": "image/png"}))
    return tool.react_invoke(args)


def test_a_generated_image_is_returned_for_the_model_to_see_unless_attach_is_false(monkeypatch, tmp_path):
    seen = _generate_image(monkeypatch, tmp_path, {"prompt": "a clock"})
    assert seen["image"]["label"] == "generated image"
    assert base64.b64decode(seen["image"]["data_uri"].split(",", 1)[1]) == PNG
    unseen = _generate_image(monkeypatch, tmp_path, {"prompt": "a clock", "attach": False})
    assert "image" not in unseen
    # The URL the display tool needs is in the text either way.
    assert "/local?path=" in seen["text"] and "/local?path=" in unseen["text"]
    # The same tag goes to the user's canvas unless `show` is false.
    assert "/local?path=" in seen["display"]
    assert "display" not in _generate_image(monkeypatch, tmp_path, {"prompt": "a clock", "show": False})


class _Loop(ToolsMixin):
    """The dispatcher with one tool that returns an image."""
    character_name = "Test"

    def __init__(self, accepts_images):
        self.backend = SimpleNamespace(supports_image_input=accepts_images)
        self._pending_tool_image = None
        self._tool_displayed = False
        self.canvas = []
        self._tool_module_cache = {
            "camera": SimpleNamespace(react_invoke=lambda *a, **k: {
                "status": "ok", "text": "captured",
                "image": {"data_uri": "data:image/png;base64,AAAA", "label": "view"}}),
            "painter": SimpleNamespace(react_invoke=lambda *a, **k: {
                "status": "ok", "text": "painted", "display": "<img src='x'>"})}

    def _run_display(self, content, fmt):
        self.canvas.append((content, fmt))
        return "OK: rendered"


def test_a_tool_image_is_sent_only_on_a_route_that_accepts_images():
    yes, no = _Loop(True), _Loop(False)
    assert "attached as an image" in yes._dispatch_discovered_tool("camera", {"tool": "camera"}, [])
    assert yes._pending_tool_image["data_uri"] == "data:image/png;base64,AAAA"
    said = no._dispatch_discovered_tool("camera", {"tool": "camera"}, [])
    assert no._pending_tool_image is None
    assert "captured" in said and "not attached" in said


def test_what_a_tool_returns_for_the_canvas_is_shown_without_a_display_call():
    loop = _Loop(True)
    said = loop._dispatch_discovered_tool("painter", {"tool": "painter"}, [])
    assert loop.canvas == [("<img src='x'>", "html")]
    assert loop._tool_displayed and "shown on the user's canvas" in said
    loop._dispatch_discovered_tool("camera", {"tool": "camera"}, [])
    assert len(loop.canvas) == 1          # a tool that returns none shows nothing
