"""Mocked-bridge tests for the factorio-telemetry sensor: baseline
initialization, each event class, quiet cycles, bridge-down, and the
Jill self-chat skip (loop prevention)."""
import importlib.util
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from utils import factorio_link  # noqa: E402


def load_sensor():
    name = "factorio_telemetry_sensor"
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(
        name, SRC / "sensors" / "factorio-telemetry" / "sensor.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


CTX = {"parameters": {"watch_items": ["iron-plate"]}}


def make_bridge(chat_entries, last_seq, players=(), production=None, alerts=()):
    def fake_request(method, path, **kwargs):
        if path == "/chat":
            return {"ok": True, "entries": list(chat_entries), "last_seq": last_seq}
        if path == "/telemetry":
            return {"ok": True, "tick": 1, "players": list(players),
                    "production": production or {}, "alerts": list(alerts)}
        raise AssertionError(f"unexpected path {path}")
    return fake_request


@pytest.fixture
def sensor(monkeypatch, tmp_path):
    module = load_sensor()
    monkeypatch.setattr(module, "_STATE_PATH", tmp_path / "state.json")
    return module


def test_baseline_then_events_then_quiet(sensor, monkeypatch):
    # run 1: baseline is silent even with history present
    monkeypatch.setattr(factorio_link, "_request", make_bridge(
        [{"seq": 1, "speaker": "Bruce", "message": "old line"}], last_seq=1,
        players=["Bruce"],
        production={"iron-plate": {"produced_1m": 5, "produced_10m": 20}}))
    assert sensor.run(CTX)["status"] == "nothing"

    # run 2: new chat + a leave + flow stop + new alert, all in one cycle
    monkeypatch.setattr(factorio_link, "_request", make_bridge(
        [{"seq": 2, "speaker": "Bruce", "message": "Jill, take the NE patch"}],
        last_seq=2, players=[],
        production={"iron-plate": {"produced_1m": 0, "produced_10m": 0}},
        alerts=[{"entity": "stone-furnace", "issue": "out of fuel", "x": 1, "y": 2}]))
    out = sensor.run(CTX)
    assert out["status"] == "ok"
    text = out["content"]
    assert "Bruce: Jill, take the NE patch" in text
    assert "Bruce left the game" in text
    assert "iron-plate production has stopped" in text
    assert "new alert: stone-furnace: out of fuel — at (1, 2)" in text

    # run 3: same world state again -> quiet (no re-reporting)
    out = sensor.run(CTX)
    assert out["status"] == "nothing"


def test_jill_chat_skipped(sensor, monkeypatch):
    monkeypatch.setattr(factorio_link, "_request", make_bridge([], last_seq=0))
    assert sensor.run(CTX)["status"] == "nothing"  # baseline
    monkeypatch.setattr(factorio_link, "_request", make_bridge(
        [{"seq": 1, "speaker": "Jill", "message": "on my way"}], last_seq=1))
    assert sensor.run(CTX)["status"] == "nothing"


def test_bridge_down_is_quiet(sensor, monkeypatch):
    def down(method, path, **kwargs):
        raise factorio_link.BridgeError("unreachable")
    monkeypatch.setattr(factorio_link, "_request", down)
    assert sensor.run(CTX)["status"] == "nothing"


def test_player_join(sensor, monkeypatch):
    monkeypatch.setattr(factorio_link, "_request", make_bridge([], 0))
    sensor.run(CTX)  # baseline: nobody online
    monkeypatch.setattr(factorio_link, "_request", make_bridge([], 0, players=["Bruce"]))
    out = sensor.run(CTX)
    assert out["status"] == "ok"
    assert "Bruce joined the game" in out["content"]
