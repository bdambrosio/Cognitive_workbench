"""Tests for the shared world (src/world/) and its agent tools.

No live server and no live world state — the World object is constructed
per test, and the tools run against a patched bridge.
"""

import importlib.util
import math
import os
import sys
from pathlib import Path

import pytest

SRC = Path(__file__).resolve().parent.parent / "src"
sys.path.insert(0, str(SRC))

from utils import world_link  # noqa: E402
from world import terrain as terrain_mod  # noqa: E402
from world.server import World, parse_occupants  # noqa: E402
from world.state import Occupant  # noqa: E402

SEED = 4242


def _tool(name):
    spec = importlib.util.spec_from_file_location(
        name.replace("-", "_"), SRC / "tools" / name / "tool.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# --------------------------------------------------------------- terrain

def test_same_seed_same_terrain():
    """The load-bearing property: Python and the browser only agree
    because the browser is sent this exact terrain."""
    assert terrain_mod.generate(SEED).fingerprint() == \
        terrain_mod.generate(SEED).fingerprint()


def test_different_seed_different_terrain():
    assert terrain_mod.generate(SEED).fingerprint() != \
        terrain_mod.generate(SEED + 1).fingerprint()


def test_height_at_matches_grid_nodes():
    """Bilinear sampling must return the grid value exactly on a node —
    that is the contract the renderer's mesh interpolation relies on."""
    t = terrain_mod.generate(SEED)
    half = terrain_mod.EXTENT_M / 2
    step = terrain_mod.EXTENT_M / (terrain_mod.GRID_N - 1)
    for ix, iz in ((0, 0), (10, 40), (128, 128), (terrain_mod.GRID_N - 1,) * 2):
        x, z = -half + ix * step, -half + iz * step
        assert t.height_at(x, z) == pytest.approx(t.heights[iz, ix], abs=1e-6)


def test_walkable_rejects_water_and_out_of_bounds():
    t = terrain_mod.generate(SEED)
    assert not t.walkable(10_000, 0)
    assert not t.walkable(0, -10_000)
    # Every point below the water line is unwalkable, by construction.
    wet = [(x, z) for x in range(-100, 101, 7) for z in range(-100, 101, 7)
           if t.height_at(x, z) < terrain_mod.WATER_Y]
    for x, z in wet:
        assert not t.walkable(x, z)


def test_walkable_rejects_steep_slope(monkeypatch):
    t = terrain_mod.generate(SEED)
    monkeypatch.setattr(t, "height_at", lambda x, z: 50.0)
    monkeypatch.setattr(t, "slope_at",
                        lambda x, z: terrain_mod.MAX_WALK_SLOPE + 0.1)
    assert not t.walkable(0, 0)


def test_payload_carries_what_the_renderer_needs():
    p = terrain_mod.generate(SEED).payload()
    assert set(p) >= {"grid_n", "extent_m", "water_y", "heights_b64",
                      "trees", "rocks"}
    import base64
    raw = base64.b64decode(p["heights_b64"])
    assert len(raw) == terrain_mod.GRID_N ** 2 * 4      # float32 per node


# ----------------------------------------------------------------- state

def test_bearing_and_facing():
    me = Occupant("Me", x=0, z=0, heading=0.0)          # heading 0 = +z
    ahead, behind = Occupant("A", x=0, z=10), Occupant("B", x=0, z=-10)
    assert me.distance_to(ahead) == pytest.approx(10)
    assert me.facing_offset(ahead) == pytest.approx(0, abs=1e-6)
    assert me.facing_offset(behind) == pytest.approx(math.pi, abs=1e-6)


def test_parse_occupants_defaults_to_agent():
    assert parse_occupants("Bruce:human, Jill:agent,Sentinel") == [
        ("Bruce", "human"), ("Jill", "agent"), ("Sentinel", "agent")]
    assert parse_occupants("") == []


# ---------------------------------------------------------------- server

def _world():
    w = World(seed=SEED)
    for i, (name, kind) in enumerate(parse_occupants("Bruce:human,Jill:agent")):
        w.add_occupant(name, kind, i)
    return w


def test_occupants_spawn_on_walkable_ground():
    w = _world()
    for occ in w.state.occupants.values():
        assert w.terrain.walkable(occ.x, occ.z)


def test_goal_walk_arrives_and_stops():
    """A goal is stepped over many ticks, not applied instantly — this is
    what keeps a seconds-long agent turn off the walking path."""
    w = _world()
    jill = w.state.occupants["Jill"]
    target = w.resolve_target("Jill", "Bruce")
    accepted = w.set_goal("Jill", target["x"], target["z"])
    assert accepted["accepted"] and accepted["eta_s"] > 1.0

    start = (jill.x, jill.z)
    for _ in range(20):                 # one second of ticks
        w.step(1 / 20)
    assert (jill.x, jill.z) != start
    assert jill.goal is not None, "should still be walking after 1 s"

    for _ in range(20 * 60):
        w.step(1 / 20)
        if jill.goal is None:
            break
    assert jill.goal is None and jill.gait == "idle"
    assert jill.distance_to(w.state.occupants["Bruce"]) < 4.0


def test_move_rejections():
    w = _world()
    assert "error" in w.set_goal("Jill", 10_000, 0)
    assert "error" in w.set_goal("Ghost", 0, 0)
    assert "error" in w.resolve_target("Jill", "Ghost")


def test_human_pose_is_validated():
    w = _world()
    bruce = w.state.occupants["Bruce"]
    before = (bruce.x, bruce.z)
    w.set_human_pose("Bruce", 10_000, 10_000, 1.0)
    assert (bruce.x, bruce.z) == before, "out-of-bounds pose must be ignored"
    assert bruce.heading == 1.0, "heading still tracks the client"


def test_agents_cannot_be_driven_as_humans():
    w = _world()
    jill = w.state.occupants["Jill"]
    before = (jill.x, jill.z)
    w.set_human_pose("Jill", jill.x + 5, jill.z, 0.0)
    assert (jill.x, jill.z) == before


def test_look_reports_others_and_ground():
    w = _world()
    d = w.look("Jill", 30)
    assert d["me"]["name"] == "Jill"
    assert "elevation" in d["ground"]
    assert [o["name"] for o in d["occupants"]] == ["Bruce"]
    assert d["occupants"][0]["distance_m"] > 0


def test_look_unknown_occupant_lists_who_is_present():
    d = _world().look("Nobody", 30)
    assert "Bruce" in d["error"] and "Jill" in d["error"]


def test_human_name_picks_the_human():
    assert _world().human_name() == "Bruce"


# ----------------------------------------------------------------- tools

@pytest.fixture
def fake_bridge(monkeypatch):
    """Stand in for the world server, mirroring the seam
    tests/test_fac_tools.py uses for the Factorio bridge."""
    sent = {}

    def fake_request(method, path, **kwargs):
        sent["method"], sent["path"] = method, path
        sent["params"] = kwargs.get("params")
        sent["json"] = kwargs.get("json")
        if path == "/look":
            return {
                "me": {"name": "Jill", "pos": [1.0, 2.0], "heading_deg": 0,
                       "gait": "idle", "goal": None},
                "ground": "forest, flat ground, elevation 3.0 m",
                "tree_count": 12, "radius_m": 30, "extent_m": 250,
                "occupants": [{
                    "name": "Bruce", "kind": "human", "distance_m": 9.4,
                    "bearing_deg": 90, "pos": [10.0, 2.0], "gait": "walk",
                    "looking_at_me": True, "in_my_view": False}],
            }
        return {"accepted": True, "distance_m": 9.4, "eta_s": 5.5,
                "from": [1.0, 2.0]}

    monkeypatch.setattr(world_link, "_request", fake_request)
    return sent


def test_world_look_renders_perception(fake_bridge):
    r = _tool("world-look").react_invoke({}, character_name="Jill")
    assert r["status"] == "ok"
    assert "(1.0, 2.0)" in r["text"]
    assert "Bruce (the human)" in r["text"]
    assert "9.4 m away to the E" in r["text"]      # bearing 90 -> East
    assert "looking at you" in r["text"]
    assert "behind you" in r["text"]               # in_my_view False
    assert fake_bridge["params"]["name"] == "Jill"


def test_world_look_clamps_radius(fake_bridge):
    _tool("world-look").react_invoke({"radius": 9999}, character_name="Jill")
    assert fake_bridge["params"]["radius"] == 120.0


def test_world_move_toward_states_it_does_not_block(fake_bridge):
    r = _tool("world-move").react_invoke({"toward": "Bruce"},
                                         character_name="Jill")
    assert r["status"] == "ok"
    assert fake_bridge["json"] == {"name": "Jill", "toward": "Bruce"}
    assert "does not wait" in r["text"]


def test_world_move_needs_a_destination(fake_bridge):
    r = _tool("world-move").react_invoke({}, character_name="Jill")
    assert r["status"] == "error" and "toward" in r["text"]


def test_tools_need_a_character(fake_bridge):
    for name in ("world-look", "world-move"):
        r = _tool(name).react_invoke({}, character_name=None)
        assert r["status"] == "error"


def test_world_offline_is_a_clean_error(monkeypatch):
    def boom(method, path, **kwargs):
        raise world_link.BridgeError("bridge unreachable at http://x")
    monkeypatch.setattr(world_link, "_request", boom)
    r = _tool("world-look").react_invoke({}, character_name="Jill")
    assert r["status"] == "error" and "unreachable" in r["text"]


# --- embodiment selection (turn 2358 regression) ----------------------
# "come to me at -18, 25" went to fac-status: fifteen fac-* tools against
# two world-* ones, and nothing told her which body she actually had.

def test_tool_family_lookup_uses_declared_names():
    from launcher import _tool_names_with_prefix
    fac = _tool_names_with_prefix("fac-")
    world = _tool_names_with_prefix("world-")
    assert set(world) == {"world-look", "world-move", "world-mark"}
    assert len(fac) > 10
    # get-financial-statements declares an underscored name; the family
    # lookup must key on the declaration, not the directory.
    assert not (set(fac) & set(world))


@pytest.mark.parametrize("world,factorio,dropped,kept", [
    (True, False, "fac-", "world-"),
    (False, True, "world-", "fac-"),
])
def test_embodiment_selection_drops_the_other_family(world, factorio,
                                                     dropped, kept):
    from launcher import apply_embodiment_selection
    chars = [("Jill", {})]
    apply_embodiment_selection(chars, world, factorio)
    omitted = chars[0][1]["chat"]["omitted_tools"]
    assert any(t.startswith(dropped) for t in omitted)
    assert not any(t.startswith(kept) for t in omitted)


def test_embodiment_selection_preserves_existing_omissions():
    from launcher import apply_embodiment_selection
    chars = [("Sentinel", {"chat": {"omitted_tools": ["exec-script"]}})]
    apply_embodiment_selection(chars, True, False)
    assert "exec-script" in chars[0][1]["chat"]["omitted_tools"]


def test_no_flag_leaves_catalogs_untouched():
    """Silently removing tools on an unrelated launch would be worse than
    the ambiguity; the launcher warns instead."""
    from launcher import apply_embodiment_selection
    chars = [("Jill", {})]
    apply_embodiment_selection(chars, False, False)
    assert (chars[0][1].get("chat") or {}).get("omitted_tools") in (None, [])


def test_embodiment_probe_reports_down_surfaces(monkeypatch):
    import requests
    from chat.chat_loop import ChatLoop

    class Fake:
        _EMBODIMENT_SURFACES = ChatLoop._EMBODIMENT_SURFACES

    def refuse(*a, **k):
        raise requests.ConnectionError("refused")
    monkeypatch.setattr(requests, "get", refuse)
    line = ChatLoop._compute_embodiment_line(Fake())
    assert "LIVE" not in line
    assert "shared world: not running" in line


def test_omitted_tools_are_unreachable_not_just_unadvertised():
    """Turn 2360: fac-status ran under --world with the fac-* family
    already dropped. Working logs from recent turns are re-injected into
    every prompt, so a tool demonstrated before an ablation gets re-emitted
    from memory — the catalog filter alone does not stop it."""
    from chat.tools import ToolsMixin

    class Loop(ToolsMixin):
        def __init__(self, omitted):
            self._discovered_tools = {"fac-status": {}, "world-look": {},
                                      "fetch-text": {}}
            self._omitted_tools = omitted

    omitted = Loop(["fac-status"])
    assert omitted._canonical_tool_name("fac-status") is None
    assert omitted._canonical_tool_name("fac_status") is None, \
        "separator drift must not route around the omission"
    assert omitted._canonical_tool_name("world-look") == "world-look"
    assert omitted._canonical_tool_name("fetch_text") == "fetch-text"
    assert Loop([])._canonical_tool_name("fac-status") == "fac-status"


# --- world-presence sensor -------------------------------------------
# Edge-triggered, level-carrying. The two properties that matter are that
# a quiet world costs nothing, and that someone loitering at the boundary
# cannot flap the agent into a turn every tick.

def _sensor():
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "world_presence", SRC / "sensors" / "world-presence" / "sensor.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _look_at(distance_m):
    return {
        "me": {"name": "Jill", "pos": [0.0, 0.0], "heading_deg": 0,
               "gait": "idle", "goal": None},
        "ground": "plains, flat ground, elevation 3.0 m",
        "tree_count": 4, "radius_m": 40, "extent_m": 250,
        "occupants": [{
            "name": "Bruce", "kind": "human", "distance_m": distance_m,
            "bearing_deg": 0, "pos": [0.0, distance_m], "gait": "idle",
            "looking_at_me": False, "in_my_view": True}],
    }


@pytest.fixture
def sensor(monkeypatch):
    mod = _sensor()
    state = {"distance": 40.0}
    monkeypatch.setattr(mod, "world_get",
                        lambda *a, **k: _look_at(state["distance"]))
    return mod, state


def _run(mod):
    return mod.run({"character_name": "Jill", "parameters": {}})


def test_cold_start_is_silent(sensor):
    mod, state = sensor
    state["distance"] = 5.0            # already standing next to her
    assert _run(mod)["status"] == "nothing", \
        "someone already present at launch is not an arrival"


def test_quiescent_world_costs_nothing(sensor):
    mod, state = sensor
    _run(mod)
    for _ in range(5):
        assert _run(mod)["status"] == "nothing"


def test_arrival_fires_once_and_carries_state(sensor):
    mod, state = sensor
    _run(mod)                           # seed at 40 m
    state["distance"] = 8.0
    r = _run(mod)
    assert r["status"] == "ok"
    assert r["metadata"]["arrived"] == ["Bruce"]
    # Level carried with the edge: no world-look needed to act.
    assert "8.0 m" in r["content"] and "plains" in r["content"]
    assert "stay silent" in r["content"], "must not force a reply"
    assert _run(mod)["status"] == "nothing", "must not re-fire while near"


def test_hysteresis_band_does_not_flap(sensor):
    mod, state = sensor
    _run(mod)
    state["distance"] = 8.0
    assert _run(mod)["status"] == "ok"
    for d in (13.0, 15.0, 19.5, 14.0):   # between enter 12 and exit 20
        state["distance"] = d
        assert _run(mod)["status"] == "nothing", f"flapped at {d} m"


def test_departure_needs_the_wider_threshold(sensor):
    mod, state = sensor
    _run(mod)
    state["distance"] = 8.0
    _run(mod)
    state["distance"] = 25.0
    r = _run(mod)
    assert r["status"] == "ok" and r["metadata"]["departed"] == ["Bruce"]
    state["distance"] = 6.0
    assert _run(mod)["metadata"]["arrived"] == ["Bruce"]


def test_world_down_is_silent_not_an_error(monkeypatch):
    mod = _sensor()

    def boom(*a, **k):
        raise mod.BridgeError("bridge unreachable")
    monkeypatch.setattr(mod, "world_get", boom)
    assert _run(mod)["status"] == "nothing", \
        "a chat-only session must not error every cycle"


# --- fog of war -------------------------------------------------------
# Without a sight limit, world-look hands every agent every position and
# there is never a reason to ask anyone anything.

def test_sight_is_shorter_in_forest():
    t = terrain_mod.generate(SEED)
    assert terrain_mod.FOREST_SIGHT_M < terrain_mod.OPEN_SIGHT_M
    forest = next((x, z) for x in range(-100, 101, 5)
                  for z in range(-100, 101, 5) if t.biome_at(x, z) == "forest")
    plains = next((x, z) for x in range(-100, 101, 5)
                  for z in range(-100, 101, 5) if t.biome_at(x, z) == "plains")
    assert t.sight_range(*forest) < t.sight_range(*plains)


def test_range_alone_hides_distant_occupants():
    t = terrain_mod.generate(SEED)
    plains = next((x, z) for x in range(-80, 81, 5)
                  for z in range(-80, 81, 5) if t.biome_at(x, z) == "plains")
    far = (plains[0], plains[1] + terrain_mod.OPEN_SIGHT_M + 20)
    assert not t.can_see(plains[0], plains[1], far[0], far[1])


def test_terrain_occludes_inside_sight_range():
    """The radius is not doing all the work — a rise between two points
    inside range must break line of sight."""
    t = terrain_mod.generate(SEED)
    step = 25
    pts = [(x, z) for x in range(-100, 101, step)
           for z in range(-100, 101, step)]
    blocked = [(a, b) for a in pts for b in pts if a != b
               and math.hypot(a[0] - b[0], a[1] - b[1]) <= t.sight_range(*a)
               and not t.line_of_sight(a[0], a[1], b[0], b[1])]
    assert blocked, "no pair in range was occluded — fog is only a radius"


def test_look_hides_the_unseen_but_admits_they_exist():
    w = _world()
    me = w.state.occupants["Jill"]
    bruce = w.state.occupants["Bruce"]
    bruce.x, bruce.z = me.x, me.z + terrain_mod.OPEN_SIGHT_M + 40
    d = w.look("Jill", 30)
    assert d["occupants"] == []
    assert d["out_of_sight"] == 1, \
        "presence is knowable; position is what must be asked for"


def test_cannot_walk_toward_someone_unseen():
    w = _world()
    me = w.state.occupants["Jill"]
    bruce = w.state.occupants["Bruce"]
    bruce.x, bruce.z = me.x, me.z + terrain_mod.OPEN_SIGHT_M + 40
    err = w.resolve_target("Jill", "Bruce")["error"]
    assert "cannot see" in err
    # ...but a coordinate someone told you still works. That asymmetry is
    # the point: saying where you are does real work.
    assert w.set_goal("Jill", bruce.x, bruce.z).get("accepted")


# --- markers ----------------------------------------------------------
# A thing left in the world rather than said. Persistence is the point:
# it survives the gap between how fast Bruce moves and how slowly Jill
# takes turns.

def test_marker_defaults_to_the_placers_feet():
    w = _world()
    jill = w.state.occupants["Jill"]
    r = w.add_marker("Jill", "waiting here")
    assert r["placed"] and r["pos"] == [round(jill.x, 1), round(jill.z, 1)]


def test_marker_at_a_coordinate_and_bounds_checked():
    w = _world()
    assert w.add_marker("Jill", "there", 10.0, 10.0)["placed"]
    assert "error" in w.add_marker("Jill", "nowhere", 9999.0, 0.0)
    assert "error" in w.add_marker("Ghost", "x")


def test_markers_obey_fog():
    w = _world()
    jill = w.state.occupants["Jill"]
    w.add_marker("Jill", "near me")
    far_z = jill.z + terrain_mod.OPEN_SIGHT_M + 40
    w.add_marker("Jill", "over the ridge", jill.x, far_z)
    labels = [m["label"] for m in w.look("Jill", 30)["markers"]]
    assert "near me" in labels
    assert "over the ridge" not in labels, \
        "a marker you cannot see must not appear"


def test_markers_are_capped_oldest_first():
    from world.state import MAX_MARKERS
    w = _world()
    for i in range(MAX_MARKERS + 5):
        w.add_marker("Jill", f"m{i}")
    labels = [m.label for m in w.state.markers]
    assert len(labels) == MAX_MARKERS
    assert "m0" not in labels and f"m{MAX_MARKERS + 4}" in labels


def test_marker_survives_the_placer_walking_away():
    """The whole reason to leave one rather than say one."""
    w = _world()
    jill = w.state.occupants["Jill"]
    w.add_marker("Jill", "meet here")
    where = (w.state.markers[0].x, w.state.markers[0].z)
    jill.x, jill.z = jill.x + 5, jill.z + 5
    for _ in range(40):
        w.step(1 / 20)
    assert (w.state.markers[0].x, w.state.markers[0].z) == where


@pytest.fixture
def mark_bridge(monkeypatch):
    sent = {}

    def fake_request(method, path, **kwargs):
        sent["path"], sent["json"] = path, kwargs.get("json")
        return {"placed": True, "id": 3, "label": "spot", "pos": [1.0, 2.0]}
    monkeypatch.setattr(world_link, "_request", fake_request)
    return sent


def test_world_mark_tool_requires_a_label(mark_bridge):
    r = _tool("world-mark").react_invoke({}, character_name="Jill")
    assert r["status"] == "error" and "label" in r["text"]


def test_world_mark_tool_rejects_half_a_coordinate(mark_bridge):
    r = _tool("world-mark").react_invoke({"label": "x", "x": 5},
                                         character_name="Jill")
    assert r["status"] == "error" and "both" in r["text"]


def test_world_mark_tool_places_at_feet(mark_bridge):
    r = _tool("world-mark").react_invoke({"label": "spot"},
                                         character_name="Jill")
    assert r["status"] == "ok"
    assert mark_bridge["json"] == {"name": "Jill", "label": "spot"}
    assert "stays there" in r["text"]
