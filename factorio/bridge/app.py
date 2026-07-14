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
    "alerts": deque(maxlen=200),
    "stop_ts": 0.0,                 # last hard stop (chat phrase or /act/stop)
}


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
    if isinstance(alerts, list) and alerts:
        with _lock:
            state["alerts"].extend(alerts)


def _poller():
    n = 0
    while True:
        try:
            _poll_chat()
            if n % 5 == 0:
                _poll_telemetry()
        except Exception as e:
            log.warning("poller error: %s", e)
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
        "players": telem.get("players", []),
        "last_activity": last,
        "last_hard_stop_s_ago": round(time.time() - stop_ts, 1) if stop_ts else None,
    }


@app.get("/observe")
def observe(x: float | None = None, y: float | None = None, radius: float = 15):
    if x is None or y is None:
        pos = _position()
        x, y = pos["x"], pos["y"]
    radius = min(radius, 50)
    entities, _ = _observe(x, y, radius)
    return {"ok": True, "center": {"x": x, "y": y}, "radius": radius, "entities": entities}


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
        return {"ok": True, **state["telemetry"], "alerts": list(state["alerts"])}


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
