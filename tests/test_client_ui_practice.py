"""The practice page's server over a temporary engagements directory."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

pytest.importorskip("fastapi")
from fastapi.testclient import TestClient                       # noqa: E402

from workflowsv2 import engagement_state as st                  # noqa: E402
from client_ui.practice import make_practice_app, commands       # noqa: E402


def _client(tmp_path, monkeypatch):
    monkeypatch.setattr(st, "ENGAGEMENTS", tmp_path)
    return TestClient(make_practice_app("secret"))


def test_token_and_listing(tmp_path, monkeypatch):
    c = _client(tmp_path, monkeypatch)
    assert c.get("/api/engagements").status_code == 403
    assert c.get("/api/engagements?token=secret").json() == []
    assert c.get("/?token=secret").status_code == 200


def test_new_current_and_cancel(tmp_path, monkeypatch):
    c = _client(tmp_path, monkeypatch)
    r = c.post("/api/engagements?token=secret", json={"name": "e1"})
    assert r.status_code == 200 and r.json()[0]["name"] == "e1"
    assert (tmp_path / "e1" / "engagement.yaml").is_file()
    assert c.post("/api/engagements?token=secret", json={"name": "../x"}).status_code == 400
    assert c.post("/api/engagements?token=secret", json={"name": "e1"}).status_code == 400
    a = st.new_intake(tmp_path / "e1"); b = st.new_intake(tmp_path / "e1")
    r = c.post("/api/engagements/e1/intake/current?token=secret", json={"id": a})
    e = r.json()[0]
    assert e["current_intake"] == a and [i["current"] for i in e["intakes"]] == [True, False]
    r = c.post("/api/engagements/e1/intake/cancel?token=secret", json={"id": a})
    e = r.json()[0]
    assert e["current_intake"] == b and e["intakes"][0]["cancelled"]
    assert c.post("/api/engagements/e1/run/current?token=secret", json={"id": "nope"}).status_code == 400
    assert c.post("/api/engagements/zz/intake/current?token=secret", json={"id": a}).status_code == 404
    cmds = e["commands"]
    assert "--engagement e1" in cmds["intake"] and "<merged dir>" in cmds["report"]


def test_commands_fill_in_the_current_run():
    s = {"current_intake": "I1", "intakes": [{"id": "I1", "runs": [
        {"name": "2026_x", "current": False}, {"name": "2026_y", "current": True}]}]}
    assert "merged/2026_y" in commands("e", s)["report"]
