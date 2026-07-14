"""Step-3 acceptance (architecture note, build order): drive the bridge over
HTTP to build a small smelting setup end-to-end, then force one deviation of
each class (world_changed / stale_model / infeasible).

Run with the server AND the bridge up:
    .venv/bin/python -m bridge &
    .venv/bin/python verify_bridge.py

Uses RconLink directly only for test fixtures (stocking the agent's
inventory, simulating another player's edit); everything agent-shaped goes
through HTTP.
"""
import json
import time
from pathlib import Path

import requests

from bridge.rcon_link import RconLink

BASE = "http://localhost:3004"
HERE = Path(__file__).parent

link = RconLink("127.0.0.1", 27015, (HERE / "data/config/rconpw").read_text().strip())

PASS, FAIL = [], []


def check(name, ok, detail=""):
    (PASS if ok else FAIL).append(name)
    print(("PASS  " if ok else "FAIL  ") + name + (f"  — {detail}" if detail else ""))


def get(path, **params):
    return requests.get(BASE + path, params=params, timeout=30).json()


def post(path, body=None, timeout=150):
    return requests.post(BASE + path, json=body or {}, timeout=timeout).json()


def clear_site(cx, cy, half=7):
    ents = get("/observe", x=cx, y=cy, radius=half + 2)["entities"]
    if [e for e in ents if isinstance(e, dict)]:
        return False
    # water and cliffs are tiles, not entities — check walkability too
    blocked = link.sc(
        "local c = 0 "
        "for x = -%d, %d do for y = -%d, %d do "
        "if game.surfaces[1].get_tile(%d + x, %d + y).collides_with('player') then c = c + 1 end "
        "end end rcon.print(c)" % (half, half, half, half, cx, cy)
    )
    return blocked == "0"


# ---- 0. pick a clear build site near the agent
pos = get("/status")["position"]
site = None
for dx, dy in [(12, 8), (24, 8), (12, 20), (-16, 12), (24, 24), (-24, -12), (0, 24)]:
    cx, cy = round(pos["x"]) + dx, round(pos["y"]) + dy
    if clear_site(cx, cy):
        site = (cx, cy)
        break
assert site, "no clear build site found near agent"
bx, by = site
print(f"build site: {site}")

# ---- fixture: stock the agent
link.call("set_inventory", 1, json.dumps({
    "stone-furnace": 2, "iron-chest": 2, "transport-belt": 40,
    "coal": 50, "iron-ore": 50,
}))

# ---- 1. walk to site
r = post("/act/walk", {"x": bx, "y": by})
check("walk to site", r.get("ok"), str(r.get("position", r))[:80])

# ---- 2. build: chest, furnace next to it, a belt run alongside
r = post("/act/place", {"prototype": "iron-chest", "x": bx, "y": by})
check("place chest", r.get("ok"), str(r.get("error", ""))[:100])

# Factorio 2.0 direction encoding: 0=N, 4=E, 8=S, 12=W
r = post("/act/place_next_to", {"prototype": "stone-furnace", "ref_x": bx, "ref_y": by,
                                "direction": 4, "gap": 2})
check("place furnace next_to chest", r.get("ok"), str(r.get("error", ""))[:100])
furnace_pos = (r.get("result") or {}).get("position", {})
fx, fy = furnace_pos.get("x", bx + 3), furnace_pos.get("y", by)

r = post("/act/connect", {"source": {"x": bx, "y": by + 4}, "target": {"x": bx + 8, "y": by + 4},
                          "prototype": "transport-belt", "dry_run": True})
check("connect dry_run", r.get("ok"), str(r.get("result", r.get("error")))[:100])

r = post("/act/connect", {"source": {"x": bx, "y": by + 4}, "target": {"x": bx + 8, "y": by + 4},
                          "prototype": "transport-belt", "dry_run": False})
check("connect belt run", r.get("ok"), str(r.get("result", r.get("error")))[:100])

# ---- 3. smelt: fuel + ore in, wait, plates out, into the chest
r = post("/act/insert", {"item": "coal", "count": 10, "x": fx, "y": fy, "target": "stone-furnace"})
check("insert coal", r.get("ok"), str(r.get("error", ""))[:100])
r = post("/act/insert", {"item": "iron-ore", "count": 10, "x": fx, "y": fy, "target": "stone-furnace"})
check("insert ore", r.get("ok"), str(r.get("error", ""))[:100])

print("smelting 10 ore (~35s)...")
plates = 0
for _ in range(20):
    time.sleep(4)
    e = get("/entity", x=fx, y=fy, prototype="stone-furnace")
    plates = ((e.get("result") or {}).get("furnace_result") or {}).get("iron-plate", 0)
    if plates >= 10:
        break
check("furnace produced plates", plates >= 10, f"{plates} iron-plate")

r = post("/act/extract", {"item": "iron-plate", "count": plates, "x": fx, "y": fy,
                          "target": "stone-furnace"})
check("extract plates", r.get("ok"), str(r.get("error", ""))[:100])
r = post("/act/insert", {"item": "iron-plate", "count": plates, "x": bx, "y": by,
                         "target": "iron-chest"})
check("deposit plates in chest", r.get("ok"), str(r.get("error", ""))[:100])

def world_write_chest(x, y):
    """Test fixture: simulate ANOTHER PLAYER building — a raw world write,
    no agent reach checks (placing via the agent hits its 10-tile limit)."""
    link.sc("game.surfaces[1].create_entity{name='iron-chest', "
            "position={x=%s, y=%s}, force=game.forces.player}" % (x, y))


# ---- 4. forced deviations, one per class
# world_changed: observe a clear tile, another player builds there, then we place
post("/act/walk", {"x": bx, "y": by - 2})
wx, wy = bx, by - 4
get("/observe", x=wx, y=wy, radius=4)                       # fresh observation: clear
world_write_chest(wx, wy)                                    # fixture: "Bruce" builds
r = post("/act/place", {"prototype": "stone-furnace", "x": wx, "y": wy})
dev = (r.get("deviation") or {}).get("kind")
check("deviation world_changed", not r.get("ok") and dev == "world_changed", f"kind={dev}")

# stale_model: place onto a tile we've NEVER observed that has something on it
# (the chest the fixture just placed is in cache now — use a second fixture
# chest far to the south, outside every observation, after walking into reach)
sx, sy = bx, by + 30
world_write_chest(sx, sy)
post("/act/walk", {"x": sx + 3, "y": sy})
r = post("/act/place", {"prototype": "stone-furnace", "x": sx, "y": sy})
dev = (r.get("deviation") or {}).get("kind")
check("deviation stale_model", not r.get("ok") and dev == "stale_model", f"kind={dev}")

# infeasible: fresh observation, unchanged world, impossible request
# (place onto our own furnace, just observed)
post("/act/walk", {"x": bx, "y": by + 2})
get("/observe", x=fx, y=fy, radius=4)
r = post("/act/place", {"prototype": "iron-chest", "x": fx, "y": fy})
dev = (r.get("deviation") or {}).get("kind")
check("deviation infeasible", not r.get("ok") and dev == "infeasible", f"kind={dev}")

# ---- 5. surfaces: log, chat, telemetry
log = get("/log")
check("activity log populated", log.get("ok") and len(log["entries"]) >= 10,
      f"{len(log.get('entries', []))} entries")
post("/say", {"message": "bridge verify complete."})
chat = get("/chat")
check("chat surface", chat.get("ok") and any(e.get("speaker") == "Jill" for e in chat["entries"]))
tel = get("/telemetry")
check("telemetry cached", tel.get("ok") and tel.get("tick") is not None,
      f"tick={tel.get('tick')} production_items={len(tel.get('production', {}))}")

print(f"\n{len(PASS)} passed, {len(FAIL)} failed" + (f": {FAIL}" if FAIL else ""))
raise SystemExit(1 if FAIL else 0)
