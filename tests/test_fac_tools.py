"""Step-4 checks for the fac-* Factorio chat tools (build order:
"discovery scan check, mocked-bridge smoke tests").

- discovery: every fac-* tool is picked up by the standard tool loader
  with valid frontmatter.
- mocked bridge: each tool's react_invoke runs against canned bridge
  responses (no server needed) and returns the {status, text} contract,
  with deviation reports surfaced verbatim.
"""
import importlib.util
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from utils import factorio_link  # noqa: E402
from utils.tool_loader import load_tools  # noqa: E402

FAC_TOOLS = [
    "fac-status", "fac-observe", "fac-inventory",
    "fac-walk", "fac-place", "fac-connect", "fac-insert", "fac-extract",
    "fac-craft", "fac-harvest", "fac-rotate",
    "fac-say", "fac-stop",
]


def load_tool_module(name: str):
    """Import src/tools/<name>/tool.py under a collision-free module name."""
    mod_name = name.replace("-", "_") + "_tool"
    if mod_name in sys.modules:
        return sys.modules[mod_name]
    spec = importlib.util.spec_from_file_location(mod_name, SRC / "tools" / name / "tool.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[mod_name] = module
    spec.loader.exec_module(module)
    return module


# ------------------------------------------------------------- discovery

def test_discovery_scan():
    tools = load_tools(str(SRC / "tools"))
    for name in FAC_TOOLS:
        assert name in tools, f"{name} not discovered by tool_loader"
        meta = tools[name]
        assert meta.get("description"), f"{name} has no description"


def test_react_invoke_present():
    for name in FAC_TOOLS:
        module = load_tool_module(name)
        assert callable(getattr(module, "react_invoke", None)), name


# ----------------------------------------------------------- mocked bridge

CANNED = {
    ("GET", "/status"): {
        "ok": True, "position": {"x": 10.0, "y": 20.0}, "walking": False,
        "tick": 12345, "players": ["Bruce"],
        "last_activity": {"endpoint": "/act/place", "params": {}, "ok": True},
        "last_hard_stop_s_ago": None,
    },
    ("GET", "/chat"): {
        "ok": True, "last_seq": 2,
        "entries": [{"seq": 1, "speaker": "Bruce", "message": "Hi Jill"},
                    {"seq": 2, "speaker": "Jill", "message": "Hi Bruce!"}],
    },
    ("GET", "/observe"): {
        "ok": True, "center": {"x": 10, "y": 20}, "radius": 15,
        "entities": [
            {"name": "stone-furnace", "position": {"x": 12, "y": 20},
             "status": "working", "warnings": [],
             "fuel": {"coal": 4}, "furnace_result": {"iron-plate": 5}},
        ] + [
            {"name": "transport-belt", "position": {"x": 10 + i, "y": 24}, "direction": 4}
            for i in range(9)
        ],
    },
    ("GET", "/telemetry"): {
        "ok": True, "tick": 12345, "players": ["Bruce"],
        "production": {"iron-plate": {"produced_1m": 10, "produced_10m": 40, "consumed_1m": 0}},
        "alerts": [],
    },
    ("GET", "/inventory"): {"ok": True, "inventory": {"coal": 40, "transport-belt": 12}},
    ("POST", "/act/walk"): {"ok": True, "position": {"x": 30.0, "y": 40.0}},
    ("POST", "/act/place"): {
        "ok": True,
        "result": {"name": "stone-furnace", "position": {"x": 30, "y": 40},
                   "status": "no_fuel", "warnings": ["'out of fuel'"]},
    },
    ("POST", "/act/connect"): {
        "ok": True,
        "result": {"connected": True, "number_of_entities": 9,
                   "entities": [{"name": "transport-belt",
                                 "position": {"x": 30 + i, "y": 44}} for i in range(9)]},
    },
    ("POST", "/act/insert"): {
        "ok": True,
        "result": {"name": "stone-furnace", "position": {"x": 30, "y": 40},
                   "fuel": {"coal": 10}},
    },
    ("POST", "/act/extract"): {"ok": True, "result": 5},
    ("POST", "/act/craft"): {"ok": True, "result": 10},
    ("POST", "/act/harvest"): {"ok": True, "result": 20},
    ("POST", "/act/rotate"): {
        "ok": True,
        "result": {"name": "burner-inserter", "position": {"x": 31, "y": 40},
                   "direction": 12},
    },
    ("POST", "/say"): {"ok": True, "result": True},
    ("POST", "/act/stop"): {"ok": True, "position": {"x": 30.0, "y": 40.0}},
}


@pytest.fixture
def mock_bridge(monkeypatch):
    calls = []

    def fake_request(method, path, **kwargs):
        calls.append((method, path, kwargs))
        assert (method, path) in CANNED, f"unexpected bridge call: {method} {path}"
        return CANNED[(method, path)]

    monkeypatch.setattr(factorio_link, "_request", fake_request)
    return calls


SMOKE = [
    ("fac-status", {}, ["(10.0, 20.0)", "Bruce", "Hi Jill"]),
    ("fac-observe", {"radius": 10}, ["stone-furnace", "transport-belt x9", "iron-plate +10"]),
    ("fac-inventory", {}, ["coal:40", "transport-belt:12"]),
    ("fac-walk", {"x": 30, "y": 40}, ["Arrived at (30.0, 40.0)"]),
    ("fac-place", {"prototype": "stone-furnace", "x": 30, "y": 40, "direction": "east"},
     ["Placed", "stone-furnace", "out of fuel"]),
    ("fac-connect", {"from_x": 30, "from_y": 44, "to_x": 38, "to_y": 44},
     ["Connected", "9 transport-belt"]),
    ("fac-insert", {"item": "coal", "count": 10, "x": 30, "y": 40, "target": "stone-furnace"},
     ["Inserted 10 coal", "coal:10"]),
    ("fac-extract", {"item": "iron-plate", "count": 5, "x": 30, "y": 40,
                     "target": "stone-furnace"}, ["Extracted 5 iron-plate"]),
    ("fac-craft", {"item": "transport-belt", "count": 10}, ["Crafted 10 transport-belt"]),
    ("fac-harvest", {"x": 5, "y": 5, "count": 20}, ["Harvested 20"]),
    ("fac-rotate", {"prototype": "burner-inserter", "x": 31, "y": 40, "direction": "west"},
     ["Rotated", "burner-inserter"]),
    ("fac-say", {"message": "on it"}, ["Said in game chat: on it"]),
    ("fac-stop", {}, ["Stopped at (30.0, 40.0)"]),
]


@pytest.mark.parametrize("name,args,expect", SMOKE, ids=[s[0] for s in SMOKE])
def test_mocked_smoke(mock_bridge, name, args, expect):
    module = load_tool_module(name)
    out = module.react_invoke(args)
    assert out["status"] == "ok", out
    for fragment in expect:
        assert fragment in out["text"], f"{fragment!r} not in:\n{out['text']}"


def test_deviation_surfaces_verbatim(monkeypatch):
    def fake_request(method, path, **kwargs):
        return {"ok": False, "error": "collision at target",
                "deviation": {"kind": "world_changed",
                              "detail": "target area changed since it was observed 12s ago",
                              "observed_now": ["iron-chest@(30.0,40.0)"],
                              "last_observed": []}}

    monkeypatch.setattr(factorio_link, "_request", fake_request)
    out = load_tool_module("fac-place").react_invoke(
        {"prototype": "stone-furnace", "x": 30, "y": 40})
    assert out["status"] == "error"
    assert "deviation [world_changed]" in out["text"]
    assert "iron-chest@(30.0,40.0)" in out["text"]
    assert "collision at target" in out["text"]


def test_bridge_down_is_clean_error(monkeypatch):
    def fake_request(method, path, **kwargs):
        raise factorio_link.BridgeError("factorio bridge unreachable at http://localhost:3004")

    monkeypatch.setattr(factorio_link, "_request", fake_request)
    for name in ("fac-status", "fac-walk", "fac-say"):
        module = load_tool_module(name)
        out = module.react_invoke({"x": 1, "y": 2, "message": "hi"})
        assert out["status"] == "error"
        assert "unreachable" in out["text"]


def test_bad_args_rejected_without_bridge(monkeypatch):
    def fake_request(method, path, **kwargs):
        raise AssertionError("bridge should not be called on bad args")

    monkeypatch.setattr(factorio_link, "_request", fake_request)
    cases = [
        ("fac-walk", {"x": "north"}),
        ("fac-place", {"x": 1, "y": 2}),                      # no prototype
        ("fac-place", {"prototype": "chest", "x": 1, "y": 2, "direction": "up"}),
        ("fac-insert", {"item": "coal", "count": 5}),          # no target/x/y
        ("fac-say", {}),
    ]
    for name, args in cases:
        out = load_tool_module(name).react_invoke(args)
        assert out["status"] == "error", (name, out)
