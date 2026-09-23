"""emit() reports a dropped response_format and nothing else as unconstrained."""
import sys
import types
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO)); sys.path.insert(0, str(REPO / "src"))

from workflowsv2.emit import emit                                  # noqa: E402


class Backend:
    def __init__(self, drop):
        self._param_drops, self._drop = set(), drop
        self.last_finish_reason, self.last_reasoning_chars = "stop", None

    def chat(self, *a, **k):
        self._param_drops |= self._drop
        return '{"a": 1}'


def _run(drop):
    return emit(types.SimpleNamespace(backend=Backend(drop)), "s", "u",
                {"type": "object"}, 100)["response_format_dropped"]


def test_a_dropped_top_p_is_not_a_dropped_response_format():
    assert _run({"top_p"}) == []


def test_a_dropped_response_format_is_reported():
    assert _run({"response_format", "top_p"}) == ["response_format"]
