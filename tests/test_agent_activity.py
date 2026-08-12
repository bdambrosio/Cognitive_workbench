"""agent-activity — the audit tool Sentinel uses to review another
character's outbound-capable tool calls.

The properties that matter: it selects by tool identity (never by reading
the argument text), it renders the argument verbatim because that is the
channel a payload would travel in, and it survives the several timestamp
formats this codebase writes.
"""
import importlib.util
import json
import sys
import time
from pathlib import Path

import pytest

SRC = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SRC))


def _tool():
    spec = importlib.util.spec_from_file_location(
        "agent_activity", SRC / "tools" / "agent-activity" / "tool.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


mod = _tool()


def _rec(ts, turn, source, actions):
    log = "\n".join(f"ACTION: {json.dumps(a)}" for a in actions)
    return {"ts": ts, "turn_seq": turn, "source": source,
            "working_log": f"--- iter 1 ---\n{log}\n$step1: OK: fine"}


@pytest.fixture
def trace(tmp_path, monkeypatch):
    """A trace file the tool will find, with a caller-supplied record set."""
    p = tmp_path / "reasoning_trace.jsonl"
    monkeypatch.setattr(mod, "_trace_path", lambda character: p)

    def write(records):
        p.write_text("\n".join(json.dumps(r) for r in records) + "\n")
    return write


def _iso(offset_s=0):
    from datetime import datetime, timezone
    return datetime.fromtimestamp(time.time() + offset_s,
                                  timezone.utc).isoformat()


def test_reports_outbound_calls_and_skips_local_ones(trace):
    trace([_rec(_iso(-600), 12, "sensor:rss-watcher", [
        {"tool": "fetch-text", "url": "https://example.com/feed"},
        {"tool": "world-look"},              # local perception: no channel out
        {"tool": "search-web", "query": "who is example"},
    ])])
    r = mod.react_invoke({"character": "Jill"})
    assert r["status"] == "ok"
    assert "fetch-text" in r["text"] and "search-web" in r["text"]
    assert "world-look" not in r["text"]
    # The argument is the point of the whole tool — it must appear whole.
    assert "https://example.com/feed" in r["text"]
    # And what sourced the turn, since feed-sourced turns are the risk.
    assert "sensor:rss-watcher" in r["text"]


def test_silence_is_a_result_not_a_gap(trace):
    trace([_rec(_iso(-600), 12, "User", [{"tool": "world-move", "x": 1}])])
    r = mod.react_invoke({"character": "Jill"})
    assert r["status"] == "empty"
    assert "not a gap" in r["text"], \
        "an auditor told 'nothing found' must not read it as a failed probe"


def test_window_excludes_older_calls(trace):
    trace([
        _rec(_iso(-4 * 3600), 10, "Jill", [{"tool": "search-web", "query": "old"}]),
        _rec(_iso(-600), 11, "Jill", [{"tool": "search-web", "query": "recent"}]),
    ])
    r = mod.react_invoke({"character": "Jill", "hours": 1})
    assert "recent" in r["text"] and "old" not in r["text"]


@pytest.mark.parametrize("stamp", [
    _iso(-600),                                  # aware ISO — what traces write
    time.time() - 600,                           # epoch float
    str(time.time() - 600),                      # epoch as string
])
def test_accepts_the_timestamp_formats_this_codebase_writes(trace, stamp):
    """Aware-UTC ISO and naive-local stamps both exist here. Parsing only
    one of them made every record undated, and the tool reported a clean
    bill of health over a trace it had entirely skipped."""
    trace([_rec(stamp, 12, "Jill", [{"tool": "fetch-text", "url": "https://x.test"}])])
    assert mod.react_invoke({"character": "Jill"})["status"] == "ok"


def test_undated_records_are_skipped_not_crashed(trace):
    trace([_rec(None, 12, "Jill", [{"tool": "fetch-text", "url": "https://x.test"}])])
    assert mod.react_invoke({"character": "Jill"})["status"] == "empty"


def test_malformed_action_lines_do_not_stop_the_scan(trace):
    p_rec = _rec(_iso(-600), 12, "Jill",
                 [{"tool": "fetch-text", "url": "https://good.test"}])
    p_rec["working_log"] = ("ACTION: {truncated json\n"
                           + p_rec["working_log"])
    trace([p_rec])
    r = mod.react_invoke({"character": "Jill"})
    assert r["status"] == "ok" and "https://good.test" in r["text"]


def test_missing_character_and_unknown_character(trace, monkeypatch):
    assert mod.react_invoke({})["status"] == "error"
    monkeypatch.setattr(mod, "_trace_path", lambda c: None)
    assert mod.react_invoke({"character": "Nobody"})["status"] == "empty"
