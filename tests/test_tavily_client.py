"""The tavily tool and the shared client under it: an answer that cannot be
read is an error, and only a readable empty answer is "nothing found".

    python3 -m pytest tests/test_tavily_client.py -q
"""
import importlib.util
import sys
from pathlib import Path

import requests

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src"))

from utils import tavily_client                                  # noqa: E402

_spec = importlib.util.spec_from_file_location("tavily_tool", REPO / "src/tools/tavily/tool.py")
tool = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(tool)


class Resp:
    def __init__(self, body, status=200):
        self._body, self.status_code, self.text = body, status, str(body)

    def json(self):
        if isinstance(self._body, Exception):
            raise self._body
        return self._body


def _post(resp, seen=None):
    def post(url, headers=None, json=None, timeout=None):
        if seen is not None:
            seen.update(url=url, json=json, timeout=timeout)
        if isinstance(resp, Exception):
            raise resp
        return resp
    return post


def _invoke(monkeypatch, resp, seen=None):
    monkeypatch.setenv("TAVILY_API_KEY", "k")
    monkeypatch.setattr(requests, "post", _post(resp, seen))
    return tool.react_invoke({"query": "q", "max_results": 3})


def test_a_200_without_results_is_an_error(monkeypatch):
    out = _invoke(monkeypatch, Resp({"detail": "rate limited"}))
    assert out["status"] == "error" and "rate limited" in out["text"]


def test_a_200_with_no_hits_is_still_empty(monkeypatch):
    assert _invoke(monkeypatch, Resp({"results": []}))["status"] == "empty"


def test_the_request_is_unchanged_and_results_reach_the_observation(monkeypatch):
    seen = {}
    out = _invoke(monkeypatch, Resp({"results": [
        {"url": "https://a.example/x", "title": "T", "content": "snippet words",
         "raw_content": "page words"}]}), seen)
    assert seen["url"] == "https://api.tavily.com/search" and seen["timeout"] == 60.0
    assert seen["json"] == {"query": "q", "max_results": 3, "search_depth": "advanced",
                            "include_answer": False, "include_raw_content": "markdown"}
    assert out.get("status") != "error" and "a.example" in out["text"]


def test_errors_keep_the_tools_error_shape(monkeypatch):
    for resp in (Resp("boom", 500), Resp(ValueError("bad json")), requests.exceptions.Timeout()):
        assert _invoke(monkeypatch, resp)["status"] == "error"
    monkeypatch.delenv("TAVILY_API_KEY")
    assert "use search-web instead" in tool.react_invoke({"query": "q"})["text"]
