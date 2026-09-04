"""The demo server over fake sessions: a cookie per visitor, a world per
cookie, eviction above the cap, the turn limit, redaction on the way out."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
pytest.importorskip("fastapi")
from fastapi.testclient import TestClient                       # noqa: E402

from demo.app import make_app, COOKIE                             # noqa: E402
from demo.redact import Redactor                                  # noqa: E402

built, closed = [], []


class FakeSession:
    def __init__(self, sid):
        self.sid = sid; self.hist = []
        built.append(sid)

    def turn(self, text):
        self.hist += [{"who": "client", "text": text}, {"who": "agent", "text": f"about chhoto-url: {text}"}]
        return {"reply": f"about chhoto-url: {text}"}

    def history(self):
        return list(self.hist)

    def close(self):
        closed.append(self.sid)


def _app(tmp_path, **over):
    cfg = {"display_name": "the target", "max_live": 2, "max_turns": 2, "max_chars": 50,
           "sessions_per_ip_per_hour": 3, "turns_in_flight": 2, "keep_days": 7,
           "model_label": "m", "redact": ["chhoto-url"]}
    cfg.update(over)
    doc = {"kind": "report", "engagement": "the target", "html": "<p>x</p>", "findings": {}, "banner": "b"}
    return TestClient(make_app(FakeSession, doc, Redactor("the target", ["chhoto-url"]), cfg, tmp_path / "log.jsonl"))


def _visit(c):
    r = c.get("/"); assert r.status_code == 200
    return r.cookies.get(COOKIE) or c.cookies.get(COOKIE)


def test_cookie_per_visitor_and_redacted_replies(tmp_path):
    built.clear(); closed.clear()
    c = _app(tmp_path)
    sid = _visit(c)
    assert sid and c.get("/").cookies.get(COOKIE) is None            # a known cookie is kept
    with c.websocket_connect("/ws") as ws:
        assert ws.receive_json()["type"] == "history"
        assert ws.receive_json()["type"] == "document"
        assert ws.receive_json()["type"] == "status"
        ws.send_json({"type": "turn", "text": "what about chhoto-url?"})
        got = [ws.receive_json() for _ in range(4)]
        say = [g for g in got if g["type"] == "say" and g.get("who") != "client"][0]
        assert "chhoto" not in say["text"] and "the target" in say["text"]
        ws.send_json({"type": "turn", "text": "x" * 60})
        assert "limited to 50" in ws.receive_json()["text"]
        ws.send_json({"type": "turn", "text": "second"})
        [ws.receive_json() for _ in range(4)]
        ws.send_json({"type": "turn", "text": "third"})
        assert "2 questions per visitor" in ws.receive_json()["text"]
    assert built == [sid]


def test_no_cookie_is_refused_and_ip_rate_limited(tmp_path):
    c = _app(tmp_path)
    with pytest.raises(Exception):
        with c.websocket_connect("/ws") as ws:
            ws.receive_json()
    for _ in range(3):
        c.cookies.clear(); assert c.get("/").status_code == 200
    c.cookies.clear(); assert c.get("/").status_code == 429


def test_eviction_above_the_cap_and_two_visitors_two_sessions(tmp_path):
    built.clear(); closed.clear()
    c = _app(tmp_path)
    sids = []
    for _ in range(3):
        c.cookies.clear(); sids.append(_visit(c))
        with c.websocket_connect("/ws") as ws:
            for _ in range(3): ws.receive_json()
    assert built == sids and closed == [sids[0]]                      # cap 2: the first was evicted
