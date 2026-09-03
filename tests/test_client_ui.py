"""The client page's server: token gate, history and document, an upload
that becomes a turn, a websocket round trip. A fake session stands in for
the ChatLoop."""
import sys
import threading
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

pytest.importorskip("fastapi")
from fastapi.testclient import TestClient                       # noqa: E402

from client_ui.app import make_app                               # noqa: E402


class FakeSession:
    def __init__(self, eng_dir):
        self.eng_dir = eng_dir
        self.turns = []
        self.hist = []
        self.opened = 0
        self.closed = False
        self.turned = threading.Event()

    def open(self):
        self.opened += 1
        self.hist.append({"who": "agent", "text": "hello, who are you?"})
        return "hello, who are you?"

    def turn(self, text):
        self.turns.append(text)
        self.hist += [{"who": "client", "text": text}, {"who": "agent", "text": f"got: {text}"}]
        self.turned.set()
        return {"reply": f"got: {text}"}

    def history(self):
        return list(self.hist)

    def document(self):
        return {"kind": "form", "engagement": "e", "form": {}, "check": {}, "ledger": "[form: x]",
                "slots": {"identify": ["client"]}, "uploads_dir": str(self.uploads_dir())}

    def uploads_dir(self):
        d = self.eng_dir / "uploads"; d.mkdir(exist_ok=True); return d

    def close(self):
        self.closed = True


def _client(tmp_path, kind="intake"):
    s = FakeSession(tmp_path)
    return s, TestClient(make_app(s, kind, "secret"))


def test_token_gate(tmp_path):
    s, c = _client(tmp_path)
    assert c.get("/").status_code == 403
    assert c.get("/?token=wrong").status_code == 403
    assert c.get("/?token=secret").status_code == 200
    assert c.get("/api/document?token=secret").json()["kind"] == "form"
    assert c.get("/static/client.css").status_code == 200


def test_websocket_opens_then_turns(tmp_path):
    s, c = _client(tmp_path)
    with c.websocket_connect("/ws?token=secret") as ws:
        assert ws.receive_json()["type"] == "history"
        assert ws.receive_json()["type"] == "document"
        assert ws.receive_json() == {"type": "status", "state": "idle"}
        # the opening turn, once
        assert ws.receive_json() == {"type": "status", "state": "thinking"}
        assert ws.receive_json() == {"type": "say", "text": "hello, who are you?"}
        assert ws.receive_json() == {"type": "status", "state": "idle"}
        ws.send_json({"type": "turn", "text": "Acme"})
        got = [ws.receive_json() for _ in range(5)]
        types = [g["type"] for g in got]
        assert got[0] == {"type": "say", "who": "client", "text": "Acme"}
        assert {"type": "say", "text": "got: Acme"} in got
        assert "document" in types and types[-1] == "status"
    assert s.turns == ["Acme"] and s.opened == 1


def test_upload_saves_and_becomes_a_turn(tmp_path):
    s, c = _client(tmp_path)
    with c.websocket_connect("/ws?token=secret") as ws:
        for _ in range(6):
            ws.receive_json()
        r = c.post("/api/upload?token=secret", files={"file": ("../evil name.txt", b"hi")})
        assert r.status_code == 200 and r.json()["saved"] == "uploads/evil_name.txt"
        assert (tmp_path / "uploads" / "evil_name.txt").read_bytes() == b"hi"
        assert s.turned.wait(5)
        assert s.turns == ["[The client uploaded a file into the engagement: "
                           "uploads/evil_name.txt (2 bytes). Its contents:]\n\nhi"]
        r = c.post("/api/upload?token=secret", files={"file": ("evil name.txt", b"again")})
        assert r.json()["saved"] == "uploads/evil_name_2.txt"
    assert c.post("/api/upload", files={"file": ("a.txt", b"x")}).status_code == 403


def test_post_kind_has_no_opening_and_no_uploads(tmp_path):
    s, c = _client(tmp_path, kind="post")
    with c.websocket_connect("/ws?token=secret") as ws:
        assert [ws.receive_json()["type"] for _ in range(3)] == ["history", "document", "status"]
    assert s.opened == 0
    assert c.post("/api/upload?token=secret", files={"file": ("a.txt", b"x")}).status_code == 400


def test_announce_text_binary_and_long():
    from client_ui.app import _announce, _INLINE_CHARS
    assert _announce("uploads/a.txt", b"x").endswith("Its contents:]\n\nx")
    assert _announce("uploads/a.png", b"\x89PNG\x00\x01").endswith("It is not a text file.]")
    long = _announce("uploads/a.md", b"y" * (_INLINE_CHARS + 5))
    assert f"first {_INLINE_CHARS} characters" in long and long.endswith("y" * _INLINE_CHARS)
