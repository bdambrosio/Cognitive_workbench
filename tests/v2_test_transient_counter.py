"""The backend counts every transient status it retries, by code, and every
call that spends the retry budget. Stubbed transport: a live route on a
funded account would not produce a 429 on demand (2026-09-02)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import chat.backend as be                                        # noqa: E402


class _Resp:
    def __init__(self, code):
        self.status_code, self.ok = code, code < 400

    def json(self):
        return {}


def _backend():
    return be._ChatBackend(server="local", model="glm-5.3-flash",
                           base_url="http://127.0.0.1:1")


def test_counts_each_retried_status_and_exhaustion(monkeypatch):
    monkeypatch.setattr(be, "_HTTP_TIMEOUT_S", 1, raising=False)
    monkeypatch.setattr(be._ChatBackend, "_TRANSIENT_BASE_SLEEP_S", 0)
    monkeypatch.setattr(be.time if hasattr(be, "time") else __import__("time"),
                        "sleep", lambda s: None)
    codes = iter([429, 429, 503, 200])
    monkeypatch.setattr(be.requests, "post",
                        lambda *a, **k: _Resp(next(codes)))
    b = _backend()
    r = b._post_transient_retrying("http://x", {}, {})
    assert r.status_code == 200
    assert b.transient_events == {429: 2, 503: 1}
    assert b.transient_exhausted == 0

    always = iter([429] * 10)
    monkeypatch.setattr(be.requests, "post",
                        lambda *a, **k: _Resp(next(always)))
    r = b._post_transient_retrying("http://x", {}, {})
    assert r.status_code == 429
    assert b.transient_events[429] == 2 + b._TRANSIENT_MAX_RETRIES
    assert b.transient_exhausted == 1

    # a clean call adds nothing
    monkeypatch.setattr(be.requests, "post", lambda *a, **k: _Resp(200))
    b._post_transient_retrying("http://x", {}, {})
    assert b.transient_exhausted == 1
