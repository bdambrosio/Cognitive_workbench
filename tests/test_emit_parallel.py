"""Concurrent schema-constrained calls (workflowsv2/emit.py): each call's
result carries its own finish reason, results come back in task order, and
the retries counted during the calls reach the loop's backend."""
import json
import sys
import threading
import time
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from workflowsv2 import emit as em                              # noqa: E402


class _Backend:
    """Answers with the request's own tag; the finish reason is the tag too.
    Like the real backend, it records the finish on itself, and returns a
    moment later: a call sharing the backend would read another's finish."""

    def __init__(self):
        self.transient_events, self.transient_exhausted = {}, 0
        self._param_drops = set()
        self.last_finish_reason = None
        # A dict, so the per-call copies of the backend count into it too.
        self.calls = {"in_flight": 0, "peak": 0}
        self.lock = threading.Lock()

    def chat(self, messages, **kw):
        tag = messages[1]["content"]
        with self.lock:
            self.calls["in_flight"] += 1
            self.calls["peak"] = max(self.calls["peak"], self.calls["in_flight"])
        time.sleep(0.05 if tag == "slow" else 0.01)
        self.transient_events[429] = self.transient_events.get(429, 0) + 1
        self.last_finish_reason = tag
        time.sleep(0.02)
        with self.lock:
            self.calls["in_flight"] -= 1
        return json.dumps({"tag": tag})


def _ask(tag):
    return lambda lp: em.emit(lp, "system", tag, {}, 100)


def test_concurrent_calls_keep_their_own_finish_and_order():
    backend = _Backend()
    loop = types.SimpleNamespace(backend=backend)
    em.set_parallel(3)
    try:
        tags = ["slow", "a", "b", "c", "d", "e"]
        out = em.run_tasks(loop, [_ask(t) for t in tags])
    finally:
        em.set_parallel(1)
    assert [o["obj"]["tag"] for o in out] == tags
    assert [o["finish"] for o in out] == tags
    assert backend.transient_events == {429: len(tags)}
    assert 1 < backend.calls["peak"] <= 3
