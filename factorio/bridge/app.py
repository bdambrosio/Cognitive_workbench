"""factorio_bridge HTTP server — the CW side of the peripheral pattern.

Endpoints per docs/factorio-bridge-architecture.md ("Bridge HTTP
contract"): JSON everywhere, {ok: true, ...} / {ok: false, error,
deviation?}. Single agent (Jill = agent character 1).
"""
import json
import logging
import os
import threading
import time
from collections import deque
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI
from pydantic import BaseModel

from .deviation import ObservationCache, classify, entity_keys
from .policy import STOP_PHRASE
from .rcon_link import ActionError, RconLink, lua_table

log = logging.getLogger("bridge")

AGENT = 1

_FACTORIO_DIR = Path(__file__).resolve().parent.parent
RCON_HOST = os.environ.get("FACTORIO_RCON_HOST", "127.0.0.1")
RCON_PORT = int(os.environ.get("FACTORIO_RCON_PORT", "27015"))
RCON_PW_FILE = Path(os.environ.get("FACTORIO_RCON_PW_FILE", str(_FACTORIO_DIR / "data/config/rconpw")))
WALK_TIMEOUT_S = float(os.environ.get("BRIDGE_WALK_TIMEOUT_S", "120"))
PATH_TIMEOUT_S = 8.0

link = RconLink(RCON_HOST, RCON_PORT, RCON_PW_FILE.read_text().strip())
cache = ObservationCache()

_lock = threading.Lock()
state = {
    "chat": deque(maxlen=500),      # bridge-side archive of the mod's chat buffer
    "chat_seq": 0,                  # highest mod chat seq seen
    "activity": deque(maxlen=500),  # Jill's action log (the audit control)
    "activity_seq": 0,
    "telemetry": {},
    "alerts": {},                   # (entity, issue, x, y) -> alert dict, expired by ALERT_TTL_S
    "stop_ts": 0.0,                 # last hard stop (chat phrase or /act/stop)
}
ALERT_TTL_S = 600.0


def _log_activity(endpoint, params, ok, error=None, deviation=None, result=None):
    with _lock:
        state["activity_seq"] += 1
        entry = {
            "seq": state["activity_seq"],
            "ts": round(time.time(), 1),
            "endpoint": endpoint,
            "params": params,
            "ok": ok,
        }
        if error is not None:
            entry["error"] = error
        if deviation is not None:
            entry["deviation"] = deviation
        if result is not None:
            entry["result"] = result
        state["activity"].append(entry)
    return entry


def _summary(result):
    """Compact form of an action result for the activity log."""
    if isinstance(result, dict):
        return {k: result[k] for k in ("name", "position", "status") if k in result}
    if isinstance(result, list):
        return {"count": len(result)}
    return result


def _observe(x, y, radius):
    """Observe via the mod and record into the deviation cache."""
    raw = link.call("get_entities", AGENT, radius, "[]", x, y)
    entities = lua_table(raw)
    if not isinstance(entities, list):
        entities = []
    entry = cache.record(x, y, radius, entity_keys(entities))
    return entities, entry


def _act(endpoint, params, fn, target=None):
    """Run a mod action, classify failure against the observation cache, log."""
    try:
        result = fn()
    except ActionError as e:
        deviation = None
        if target is not None:
            try:
                deviation = classify(cache, target[0], target[1], str(e), _observe)
            except Exception as ce:
                log.warning("deviation classification failed for %s: %s", endpoint, ce)
            if deviation is not None and deviation.get("kind") == "infeasible":
                _add_tile_blocker(deviation, [(target[0], target[1])])
        _log_activity(endpoint, params, ok=False, error=str(e), deviation=deviation)
        out = {"ok": False, "error": str(e)}
        if deviation is not None:
            out["deviation"] = deviation
        return out
    _log_activity(endpoint, params, ok=True, result=_summary(result))
    return {"ok": True, "result": result}


def _position():
    return link.call("agent_position", AGENT)


def _path(sx, sy, gx, gy, radius=1.0, timeout_s=PATH_TIMEOUT_S, entity_size=1):
    """request_path -> poll get_path until success. Returns (handle, status_str).

    move_to must only run after get_path reports success — calling early
    hits the mod's 'path not yet computed' race (observed live)."""
    rid = link.call("request_path", AGENT, sx, sy, gx, gy, radius, False, entity_size)
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        raw = link.call("get_path", rid)
        status = json.loads(raw) if isinstance(raw, str) else raw
        st = status.get("status")
        if st == "success":
            return rid, st
        if st in ("not_found", "invalid_request"):
            return None, st
        time.sleep(0.3)
    return None, "path_timeout"


# ---------------------------------------------------------------- background

def _hard_stop(reason):
    link.call("stop", AGENT)
    with _lock:
        state["stop_ts"] = time.time()
    _log_activity("hard_stop", {"reason": reason}, ok=True)
    try:
        link.call("say", "stopped.")
    except ActionError as e:
        log.warning("say after hard stop failed: %s", e)


def _poll_chat():
    new = link.call("get_chat", state["chat_seq"])
    if not isinstance(new, list):
        return
    for entry in sorted(new, key=lambda e: e.get("seq", 0)):
        with _lock:
            state["chat"].append(entry)
            state["chat_seq"] = max(state["chat_seq"], entry.get("seq", 0))
        if entry.get("speaker") != "Jill" and STOP_PHRASE in str(entry.get("message", "")).lower():
            _hard_stop("chat: %s said %r" % (entry.get("speaker"), entry.get("message")))


_TELEMETRY_LUA = """
local surface = game.surfaces[1]
local force = game.forces.player
local stats = force.get_item_production_statistics(surface)
local out = {tick = game.tick, players = {}, production = {}}
for _, p in pairs(game.connected_players) do table.insert(out.players, p.name) end
local pi = defines.flow_precision_index
for name, _ in pairs(stats.input_counts) do
    out.production[name] = {
        produced_1m = stats.get_flow_count{name = name, category = "input", precision_index = pi.one_minute},
        produced_10m = stats.get_flow_count{name = name, category = "input", precision_index = pi.ten_minutes},
        consumed_1m = stats.get_flow_count{name = name, category = "output", precision_index = pi.one_minute},
    }
end
rcon.print(helpers.table_to_json(out))
""".strip().replace("\n", " ")


# Current status of every player-force entity, for re-validating latched
# alerts (the mod's `on_tick` writes an issue once and never clears it, so
# a working, producing line still reports stale "out of fuel / no sink /
# result full" alerts). Read-only /sc; keyed by position on the Python side.
_STATUS_SCAN_LUA = """
local surf = game.surfaces[1]
local W = defines.entity_status.working
local WD = defines.entity_status.waiting_for_space_in_destination
local out = {}
for _, e in pairs(surf.find_entities_filtered{force = 'player'}) do
    local st = e.status
    if st ~= nil then
        local rec = {x = e.position.x, y = e.position.y, working = (st == W)}
        if e.type == 'mining-drill' and e.drop_position then
            rec.waiting_dest = (st == WD)
            local sink = surf.find_entities_filtered{position = e.drop_position, force = 'player'}[1]
            if sink and sink.status ~= nil then rec.sink_working = (sink.status == W) end
        end
        out[#out + 1] = rec
    end
end
rcon.print(helpers.table_to_json(out))
""".strip().replace("\n", " ")


def _revalidate_alerts():
    """Drop held alerts contradicted by current entity status: (1) the
    entity is working, or (2) a mining-drill is over-feeding a working sink
    (drill in `waiting_for_space_in_destination` while its drop-target is
    working — the healthy steady state of a direct-insert line, not a
    fault). Read-only scan; skipped silently if the server is unreachable."""
    resp = link.sc(_STATUS_SCAN_LUA)
    if not resp or resp.startswith("Cannot execute"):
        log.warning("alert revalidation scan failed: %s", (resp or "")[:200])
        return
    try:
        recs = json.loads(resp)
    except (ValueError, TypeError) as e:
        log.warning("alert revalidation parse failed: %s", e)
        return
    if not isinstance(recs, list):
        return

    def _k(x, y):
        return (round(x * 2) / 2, round(y * 2) / 2)

    working, overfeed = set(), set()
    for r in recs:
        if not isinstance(r, dict) or r.get("x") is None or r.get("y") is None:
            continue
        k = _k(r["x"], r["y"])
        if r.get("working"):
            working.add(k)
        if r.get("waiting_dest") and r.get("sink_working"):
            overfeed.add(k)
    with _lock:
        for key, a in list(state["alerts"].items()):
            if a.get("x") is None or a.get("y") is None:
                continue
            if _k(a["x"], a["y"]) in working or _k(a["x"], a["y"]) in overfeed:
                del state["alerts"][key]


def _poll_telemetry():
    resp = link.sc(_TELEMETRY_LUA)
    if resp and not resp.startswith("Cannot execute"):
        data = json.loads(resp)
        if isinstance(data.get("production"), list):  # empty Lua table -> []
            data["production"] = {}
        if isinstance(data.get("players"), dict):
            data["players"] = list(data["players"].values())
        with _lock:
            state["telemetry"] = {**data, "updated": round(time.time(), 1)}
    else:
        log.warning("telemetry poll failed: %s", (resp or "")[:200])
    alerts = link.call("get_alerts", 30)
    now = time.time()
    with _lock:
        if isinstance(alerts, list):
            # The mod emits one entry per (entity, tick); dedupe to one live
            # alert per (entity, issue, position), refreshed while it recurs.
            for a in alerts:
                if not isinstance(a, dict):
                    continue
                pos = a.get("position") or {}
                for issue in a.get("issues") or []:
                    issue = str(issue).strip("'")
                    key = (a.get("entity_name"), issue, pos.get("x"), pos.get("y"))
                    state["alerts"][key] = {
                        "entity": a.get("entity_name"), "issue": issue,
                        "x": pos.get("x"), "y": pos.get("y"),
                        "tick": a.get("tick"), "seen": round(now, 1),
                    }
        state["alerts"] = {k: v for k, v in state["alerts"].items()
                           if now - v["seen"] < ALERT_TTL_S}
    # Re-validate against live status (outside the state lock: does its own
    # RCON scan, then re-acquires the lock only to drop stale entries).
    _revalidate_alerts()


def _poller():
    # Log on health transitions only: with Jill long-lived and the game
    # server routinely down, a warning every 2s is pure spam.
    n = 0
    healthy = True
    while True:
        try:
            _poll_chat()
            if n % 5 == 0:
                _poll_telemetry()
            if not healthy:
                log.info("poller: game server reachable again")
                healthy = True
        except Exception as e:
            if healthy:
                log.warning("poller: game server unreachable (will log again on recovery): %s", e)
                healthy = False
        n += 1
        time.sleep(2)


@asynccontextmanager
async def _lifespan(app):
    try:
        _position()
    except ActionError:
        log.info("no agent character; creating one")
        link.call("create_agent_characters", 1)
    threading.Thread(target=_poller, daemon=True, name="bridge-poller").start()
    yield


app = FastAPI(title="factorio-bridge", lifespan=_lifespan)


# ---------------------------------------------------------------------- read

# Connected players WITH positions (live read; the telemetry cache keeps
# names only — the sensor's join/leave diff depends on that shape).
_PLAYERS_LUA = (
    "local out = {} "
    "for _, p in pairs(game.connected_players) do "
    "local e = {name = p.name} "
    "if p.character then e.x = p.character.position.x e.y = p.character.position.y end "
    "table.insert(out, e) end "
    "rcon.print(helpers.table_to_json(out))"
)


def _players_with_positions():
    resp = link.sc(_PLAYERS_LUA)
    if not resp or resp.startswith("Cannot execute"):
        log.warning("player position read failed: %s", (resp or "")[:200])
        return []
    data = json.loads(resp)
    return data if isinstance(data, list) else []


@app.get("/status")
def status():
    pos = _position()
    with _lock:
        telem = state["telemetry"]
        last = state["activity"][-1] if state["activity"] else None
        stop_ts = state["stop_ts"]
    return {
        "ok": True,
        "position": {"x": pos["x"], "y": pos["y"]},
        "walking": pos["walking"],
        "tick": telem.get("tick"),
        "players": _players_with_positions(),
        "last_activity": last,
        "last_hard_stop_s_ago": round(time.time() - stop_ts, 1) if stop_ts else None,
    }


# Ore patches are neutral-force entities; the mod's get_entities filters by
# player force and cannot see them (observed live: Jill was ore-blind). A
# data-only /sc read, aggregated per resource type; surfaced through
# /observe as summary pseudo-entities so the existing tools show them
# without agent-side changes.
_RESOURCES_LUA = (
    "local out = {} "
    "for _, e in pairs(game.surfaces[1].find_entities_filtered{"
    "area = {{%.1f, %.1f}, {%.1f, %.1f}}, type = 'resource'}) do "
    "local r = out[e.name] "
    "if not r then r = {count = 0, amount = 0, sx = 0, sy = 0} out[e.name] = r end "
    "r.count = r.count + 1 r.amount = r.amount + e.amount "
    "r.sx = r.sx + e.position.x r.sy = r.sy + e.position.y "
    "end rcon.print(helpers.table_to_json(out))"
)


def _resource_pseudo_entities(x, y, radius):
    resp = link.sc(_RESOURCES_LUA % (x - radius, y - radius, x + radius, y + radius))
    if not resp or resp.startswith("Cannot execute"):
        log.warning("resource scan failed: %s", (resp or "")[:200])
        return []
    data = json.loads(resp)
    if not isinstance(data, dict):
        return []
    out = []
    for name, r in sorted(data.items()):
        out.append({
            "name": "%s patch (~%d tiles, ~%d units)" % (name, r["count"], r["amount"]),
            "position": {"x": round(r["sx"] / r["count"], 1),
                         "y": round(r["sy"] / r["count"], 1)},
            "resource": name,
            "tiles": r["count"],
            "amount": r["amount"],
        })
    return out


# Loose items on the ground are neutral-force item-entities — invisible to
# the mod's player-force get_entities, but they block building placement
# (observed live 2026-07-17: Jill fought "item-on-ground" placement
# failures she could not see). Same data-only /sc pattern as resources,
# aggregated per item type; the span shows whether debris overlaps an
# intended footprint.
_GROUND_ITEMS_LUA = (
    "local out = {} "
    "for _, e in pairs(game.surfaces[1].find_entities_filtered{"
    "area = {{%.1f, %.1f}, {%.1f, %.1f}}, type = 'item-entity'}) do "
    "local n = e.stack.name "
    "local r = out[n] "
    "if not r then r = {piles = 0, count = 0, sx = 0, sy = 0, "
    "minx = 1e9, miny = 1e9, maxx = -1e9, maxy = -1e9} out[n] = r end "
    "r.piles = r.piles + 1 r.count = r.count + e.stack.count "
    "local p = e.position "
    "r.sx = r.sx + p.x r.sy = r.sy + p.y "
    "if p.x < r.minx then r.minx = p.x end if p.y < r.miny then r.miny = p.y end "
    "if p.x > r.maxx then r.maxx = p.x end if p.y > r.maxy then r.maxy = p.y end "
    "end rcon.print(helpers.table_to_json(out))"
)


def _ground_item_pseudo_entities(x, y, radius):
    resp = link.sc(_GROUND_ITEMS_LUA % (x - radius, y - radius, x + radius, y + radius))
    if not resp or resp.startswith("Cannot execute"):
        log.warning("ground-item scan failed: %s", (resp or "")[:200])
        return []
    try:
        data = json.loads(resp)
    except (ValueError, TypeError) as e:
        log.warning("ground-item scan parse failed: %s", e)
        return []
    if not isinstance(data, dict):
        return []
    out = []
    for name, r in sorted(data.items()):
        if r["piles"] > 1:
            label = ("loose %s on ground x%d (%d piles spanning "
                     "(%.1f, %.1f)..(%.1f, %.1f) — blocks placement)" % (
                         name, r["count"], r["piles"],
                         r["minx"], r["miny"], r["maxx"], r["maxy"]))
        else:
            label = "loose %s on ground x%d (blocks placement)" % (name, r["count"])
        out.append({
            "name": label,
            "position": {"x": round(r["sx"] / r["piles"], 1),
                         "y": round(r["sy"] / r["piles"], 1)},
            "item": name,
            "count": r["count"],
        })
    return out


# Water/cliffs are tiles, not entities, so an entity-empty target can still
# be impassable — and the entity-based deviation classifier cannot see why.
# A data-only tile read (same `collides_with('player')` predicate the verify
# script and mod use) turns "infeasible/short of goal" into "blocked by
# deepwater at (x, y)". Read-only /sc, so it cannot crash the server.
_TILE_PROBE_LUA = (
    "local pts = %s local out = {} "
    "for _, p in ipairs(pts) do "
    "local t = game.surfaces[1].get_tile(p[1], p[2]) "
    "if t.collides_with('player') then "
    "out[#out + 1] = {name = t.name, x = p[1], y = p[2]} break end "
    "end rcon.print(helpers.table_to_json(out))"
)


def _ray_samples(x0, y0, x1, y1, cap=8):
    """Points one tile apart from (x0,y0) toward (x1,y1), nearest first."""
    d = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
    if d < 1e-6:
        return [(round(x0, 1), round(y0, 1))]
    ux, uy = (x1 - x0) / d, (y1 - y0) / d
    n = min(cap, max(1, int(d)))
    return [(round(x0 + ux * s, 1), round(y0 + uy * s, 1)) for s in range(1, n + 1)]


def _blocking_tile(points):
    """First impassable tile among `points` (data-only read), or None."""
    if not points:
        return None
    lua_pts = "{" + ",".join("{%.1f,%.1f}" % (x, y) for x, y in points) + "}"
    resp = link.sc(_TILE_PROBE_LUA % lua_pts)
    if not resp or resp.startswith("Cannot execute"):
        log.warning("tile probe failed: %s", (resp or "")[:200])
        return None
    try:
        hits = json.loads(resp)
    except (ValueError, TypeError) as e:
        log.warning("tile probe parse failed: %s", e)
        return None
    if not isinstance(hits, list) or not hits:  # empty Lua table -> {} (dict)
        return None
    h = hits[0]
    return {"tile": h["name"], "position": {"x": h["x"], "y": h["y"]}}


def _add_tile_blocker(deviation, points):
    """Annotate an infeasible deviation with the tile that blocks `points`."""
    blk = _blocking_tile(points)
    if blk is not None:
        deviation["blocked_by"] = blk
        deviation["detail"] += " — blocked by %s at (%.1f, %.1f)" % (
            blk["tile"], blk["position"]["x"], blk["position"]["y"])
    return deviation


@app.get("/observe")
def observe(x: float | None = None, y: float | None = None, radius: float = 15):
    if x is None or y is None:
        pos = _position()
        x, y = pos["x"], pos["y"]
    radius = min(radius, 50)
    entities, _ = _observe(x, y, radius)
    # Resources and ground items are appended AFTER the deviation-cache
    # record: pseudo-entities must not perturb world-change comparisons.
    entities = (entities + _resource_pseudo_entities(x, y, radius)
                + _ground_item_pseudo_entities(x, y, radius))
    return {"ok": True, "center": {"x": x, "y": y}, "radius": radius, "entities": entities}


@app.get("/nearest")
def nearest(resource: str):
    """Nearest resource patch by name ('iron-ore', 'coal', also 'water',
    'wood'); the mod scans 500 tiles around the agent."""
    return _act("/nearest", {"resource": resource},
                lambda: link.call("nearest", AGENT, resource))


@app.get("/inventory")
def inventory():
    raw = link.call("inspect_inventory", AGENT, True, 0, 0, None, False)
    return {"ok": True, "inventory": lua_table(raw) or {}}


@app.get("/entity")
def entity(x: float, y: float, prototype: str):
    return _act("/entity", {"x": x, "y": y, "prototype": prototype},
                lambda: link.call("get_entity", AGENT, prototype, x, y), target=(x, y))


@app.get("/telemetry")
def telemetry():
    with _lock:
        return {"ok": True, **state["telemetry"], "alerts": list(state["alerts"].values())}


@app.get("/chat")
def chat(since_seq: int = 0):
    with _lock:
        entries = [e for e in state["chat"] if e.get("seq", 0) > since_seq]
    return {"ok": True, "entries": entries, "last_seq": state["chat_seq"]}


@app.get("/log")
def activity_log(since_seq: int = 0):
    with _lock:
        entries = [e for e in state["activity"] if e["seq"] > since_seq]
    return {"ok": True, "entries": entries, "last_seq": state["activity_seq"]}


# ---------------------------------------------------------------------- act

class Walk(BaseModel):
    x: float
    y: float
    timeout_s: float | None = None
    arrive_radius: float = 1.0


@app.post("/act/walk")
def act_walk(req: Walk):
    params = req.model_dump()
    pos = _position()
    with _lock:
        stop_before = state["stop_ts"]
    # Clearance ladder (as for connect): a start position adjacent to
    # entities fails pathfinding at full character size.
    for size in (1, 0.5, 0.25):
        handle, path_status = _path(pos["x"], pos["y"], req.x, req.y,
                                    req.arrive_radius, entity_size=size)
        if handle is not None:
            break
    if handle is None:
        deviation = {"kind": "infeasible", "detail": "pathfinder: %s" % path_status,
                     "target": {"x": req.x, "y": req.y}}
        _add_tile_blocker(deviation, _ray_samples(pos["x"], pos["y"], req.x, req.y))
        _log_activity("/act/walk", params, ok=False, error=path_status, deviation=deviation)
        return {"ok": False, "error": "no path: %s" % path_status, "deviation": deviation}
    link.call("move_to", AGENT, handle, None, False)
    deadline = time.time() + (req.timeout_s or WALK_TIMEOUT_S)
    while time.time() < deadline:
        time.sleep(0.5)
        pos = _position()
        if not pos["walking"]:
            break
    dist = ((pos["x"] - req.x) ** 2 + (pos["y"] - req.y) ** 2) ** 0.5
    position = {"x": pos["x"], "y": pos["y"]}
    if dist <= req.arrive_radius + 1.5:
        _log_activity("/act/walk", params, ok=True, result=position)
        return {"ok": True, "position": position}
    with _lock:
        stopped = state["stop_ts"] > stop_before
    if stopped:
        deviation = {"kind": "stopped", "detail": "hard stop during walk", "position": position}
    elif pos["walking"]:
        link.call("stop", AGENT)
        deviation = {"kind": "timeout", "detail": "walk exceeded timeout; halted", "position": position}
    else:
        deviation = {"kind": "infeasible", "detail": "walk ended %.1f tiles short of goal" % dist,
                     "position": position}
        _add_tile_blocker(deviation, _ray_samples(pos["x"], pos["y"], req.x, req.y))
    _log_activity("/act/walk", params, ok=False, error=deviation["detail"], deviation=deviation)
    return {"ok": False, "error": deviation["detail"], "deviation": deviation}


class Place(BaseModel):
    prototype: str
    x: float
    y: float
    direction: int = 0
    exact: bool = True


@app.post("/act/place")
def act_place(req: Place):
    return _act("/act/place", req.model_dump(),
                lambda: link.call("place_entity", AGENT, req.prototype, req.direction,
                                  req.x, req.y, req.exact),
                target=(req.x, req.y))


class PlaceNextTo(BaseModel):
    prototype: str
    ref_x: float
    ref_y: float
    direction: int = 0
    gap: int = 0


@app.post("/act/place_next_to")
def act_place_next_to(req: PlaceNextTo):
    return _act("/act/place_next_to", req.model_dump(),
                lambda: link.call("place_entity_next_to", AGENT, req.prototype,
                                  req.ref_x, req.ref_y, req.direction, req.gap),
                target=(req.ref_x, req.ref_y))


class Point(BaseModel):
    x: float
    y: float


class Connect(BaseModel):
    source: Point
    target: Point
    prototype: str = "transport-belt"
    dry_run: bool = False


@app.post("/act/connect")
def act_connect(req: Connect):
    """THE macro: positions-only v1 (entity-endpoint resolution deferred)."""
    params = req.model_dump()

    def run():
        inv = lua_table(link.call("inspect_inventory", AGENT, True, 0, 0, None, False)) or {}
        available = int(inv.get(req.prototype, 0))
        # FLE's clearance ladder: retry the path + connect with shrinking
        # entity sizes until one placement validates.
        last_error = None
        for size in (1.5, 1, 0.5, 0.25):
            handle, path_status = _path(req.source.x, req.source.y,
                                        req.target.x, req.target.y,
                                        radius=1, entity_size=size)
            if handle is None:
                last_error = ActionError("no path between source and target: %s" % path_status)
                continue
            try:
                return link.call("connect_entities", AGENT, req.source.x, req.source.y,
                                 req.target.x, req.target.y, handle, req.prototype,
                                 req.dry_run, available)
            except ActionError as e:
                last_error = e
        raise last_error

    return _act("/act/connect", params, run, target=(req.target.x, req.target.y))


class Transfer(BaseModel):
    item: str
    count: int
    x: float
    y: float
    target: str


@app.post("/act/insert")
def act_insert(req: Transfer):
    return _act("/act/insert", req.model_dump(),
                lambda: link.call("insert_item", AGENT, req.item, req.count,
                                  req.x, req.y, req.target),
                target=(req.x, req.y))


@app.post("/act/extract")
def act_extract(req: Transfer):
    return _act("/act/extract", req.model_dump(),
                lambda: link.call("extract_item", AGENT, req.item, req.count,
                                  req.x, req.y, req.target),
                target=(req.x, req.y))


class Craft(BaseModel):
    item: str
    count: int = 1


@app.post("/act/craft")
def act_craft(req: Craft):
    return _act("/act/craft", req.model_dump(),
                lambda: link.call("craft_item", AGENT, req.item, req.count))


class Harvest(BaseModel):
    x: float
    y: float
    count: int = 1
    radius: float = 3


@app.post("/act/harvest")
def act_harvest(req: Harvest):
    return _act("/act/harvest", req.model_dump(),
                lambda: link.call("harvest_resource", AGENT, req.x, req.y,
                                  req.count, req.radius),
                target=(req.x, req.y))


class Pickup(BaseModel):
    prototype: str
    x: float
    y: float


@app.post("/act/pickup")
def act_pickup(req: Pickup):
    return _act("/act/pickup", req.model_dump(),
                lambda: link.call("pickup_entity", AGENT, req.x, req.y, req.prototype),
                target=(req.x, req.y))


class Rotate(BaseModel):
    prototype: str
    x: float
    y: float
    direction: int


@app.post("/act/rotate")
def act_rotate(req: Rotate):
    return _act("/act/rotate", req.model_dump(),
                lambda: link.call("rotate_entity", AGENT, req.x, req.y,
                                  req.direction, req.prototype),
                target=(req.x, req.y))


class Say(BaseModel):
    message: str


@app.post("/say")
def say(req: Say):
    return _act("/say", req.model_dump(), lambda: link.call("say", req.message))


@app.post("/act/stop")
def act_stop():
    _hard_stop("/act/stop")
    pos = _position()
    return {"ok": True, "position": {"x": pos["x"], "y": pos["y"]}}
