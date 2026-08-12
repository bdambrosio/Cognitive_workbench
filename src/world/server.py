"""World authority — terrain, occupants, and the tick that moves them.

One process owns the world. The browser is a view plus an input device;
the agent tools are HTTP clients. Positions live here so a query from Jill
and a frame drawn for Bruce can never disagree.

Two listeners, both loopback, following the canvas bridge's split:
    WORLD_WS_PORT    default 8790   browser: state out, input in
    WORLD_HTTP_PORT  default 8791   agent tools + static files

Occupants come from WORLD_OCCUPANTS, e.g. "Bruce:human,Jill:agent" —
config-driven so adding Sentinel costs no code.
"""
from __future__ import annotations

import asyncio
import json
import logging
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, List, Optional, Set
from urllib.parse import parse_qs, urlparse

import numpy as np

from world import terrain as terrain_mod
from world.state import (ARRIVE_TOL_M, MAX_MARKERS, WALK_SPEED_MS,
                         Marker, Occupant, WorldState)

logger = logging.getLogger('world.server')

WS_PORT = int(os.environ.get('WORLD_WS_PORT', '8790'))
HTTP_PORT = int(os.environ.get('WORLD_HTTP_PORT', '8791'))
TICK_HZ = 20.0
# Recompute per-viewer visibility every N ticks (20 Hz tick -> ~4 Hz).
_VIS_EVERY_TICKS = 5
SEED = int(os.environ.get('WORLD_SEED', '20260808'))
STATIC_DIR = Path(__file__).resolve().parent / 'display' / 'static'

# Distinguishable at a glance across a clearing.
_PALETTE = ['#f2b544', '#8ab4f8', '#7ddc94', '#e0708a', '#c39bf5']

_MIME = {'.html': 'text/html; charset=utf-8', '.js': 'text/javascript',
         '.css': 'text/css', '.json': 'application/json',
         '.png': 'image/png', '.svg': 'image/svg+xml'}


class World:
    """Authoritative state. Every mutation goes through the lock — the
    tick loop, the WS input handler and the HTTP API all touch it."""

    def __init__(self, seed: int = SEED):
        self.terrain = terrain_mod.generate(seed)
        self.state = WorldState()
        self._lock = threading.Lock()
        self._terrain_payload = self.terrain.payload()
        self._tree_xz = None          # lazily built for spawn clearance
        self._marker_seq = 0
        self._vis_cache: Dict[str, tuple] = {}

    # -- setup ----------------------------------------------------------

    def add_occupant(self, name: str, kind: str, index: int) -> Occupant:
        x, z = self._spawn_point(index)
        occ = Occupant(name=name, kind=kind, x=x, z=z,
                       y=self.terrain.height_at(x, z),
                       color=_PALETTE[index % len(_PALETTE)])
        with self._lock:
            self.state.occupants[name] = occ
        return occ

    def _spawn_point(self, index: int) -> tuple:
        """Spread occupants around the origin on walkable, open ground.

        Clearance matters more than it sounds: spawning inside a thicket
        puts the third-person camera inside a trunk, and the first thing
        you see is the inside of a tree.
        """
        taken = [(o.x, o.z) for o in self.state.occupants.values()]

        def apart(x: float, z: float) -> float:
            """Distance to the nearest already-placed occupant."""
            if not taken:
                return 1e9
            return min(math.hypot(x - tx, z - tz) for tx, tz in taken)

        best = None
        for radius in (10.0, 18.0, 30.0, 46.0, 64.0):
            for step in range(16):
                angle = (index * 2.4) + step * (math.pi / 8)
                x = math.sin(angle) * radius
                z = math.cos(angle) * radius
                if not self.terrain.walkable(x, z):
                    continue
                # The angle sequence can bring two indices around to nearly
                # the same bearing, so distance from whoever is already
                # placed is a hard requirement, not a preference.
                gap = apart(x, z)
                if gap < 6.0:
                    continue
                clear = self._nearest_tree_m(x, z)
                if clear >= 7.0 and self.terrain.biome_at(x, z) == 'plains':
                    return x, z
                score = min(clear, gap)
                if best is None or score > best[0]:
                    best = (score, x, z)
        return (best[1], best[2]) if best else (0.0, 0.0)

    def _nearest_tree_m(self, x: float, z: float) -> float:
        trees = self.terrain.trees
        if not trees:
            return 1e9
        if self._tree_xz is None:
            self._tree_xz = np.array([[t['x'], t['z']] for t in trees])
        d = np.hypot(self._tree_xz[:, 0] - x, self._tree_xz[:, 1] - z)
        return float(d.min())

    # -- tick -----------------------------------------------------------

    def step(self, dt: float) -> None:
        """Advance goal-driven occupants. Humans move themselves; agents
        post a goal and get walked here, which is what keeps a multi-second
        agent turn from having to block on a multi-second walk."""
        with self._lock:
            self.state.tick += 1
            self.state.ts = time.time()
            for occ in self.state.occupants.values():
                if occ.goal is None:
                    if occ.gait != 'idle':
                        occ.gait = 'idle'
                    continue
                gx, gz = occ.goal
                dx, dz = gx - occ.x, gz - occ.z
                dist = math.hypot(dx, dz)
                if dist <= ARRIVE_TOL_M:
                    occ.goal, occ.gait = None, 'idle'
                    continue
                # Nothing blocks: the ground under you only sets your pace,
                # so a straight line always gets there and the cost of the
                # route is what varies. Sampling the factor where you stand
                # rather than where you are going keeps this a plain
                # integration of cost along the path.
                speed = WALK_SPEED_MS * self.terrain.speed_factor(occ.x, occ.z)
                stride = min(speed * dt, dist)
                nx, nz = occ.x + dx / dist * stride, occ.z + dz / dist * stride
                occ.x, occ.z = nx, nz
                occ.y = self.terrain.height_at(nx, nz)
                occ.heading = math.atan2(dx, dz)
                occ.gait = 'walk'

    # -- mutations from clients ------------------------------------------

    def set_human_pose(self, name: str, x: float, z: float,
                       heading: float) -> None:
        """Browser-driven movement. The browser integrates input for
        responsiveness — including the terrain speed factor, from the same
        constants — so the server takes the pose and owns ground height.
        Bounds are still the server's call; pace is not."""
        with self._lock:
            occ = self.state.occupants.get(name)
            if occ is None or occ.kind != 'human':
                return
            if self.terrain.in_bounds(x, z):
                occ.x, occ.z = x, z
                occ.y = self.terrain.height_at(x, z)
            occ.heading = heading
            occ.gait = 'walk' if occ.goal is None else occ.gait

    def set_goal(self, name: str, x: float, z: float) -> Dict[str, Any]:
        with self._lock:
            occ = self.state.occupants.get(name)
            if occ is None:
                return {'error': f"no occupant named {name!r} in this world"}
            if not self.terrain.in_bounds(x, z):
                return {'error': f"({x:.1f}, {z:.1f}) is outside the world"}
            dist = math.hypot(x - occ.x, z - occ.z)
            occ.goal = [x, z]
            occ.gait = 'walk'
            # Best case, not a forecast: the ground between here and there
            # can cost up to twice this, and finding that out by walking it
            # is what gives an occupant something worth telling a partner.
            # Integrating the real cost here would hand out that answer for
            # free and there would be nothing to report.
            return {'accepted': True, 'distance_m': round(dist, 1),
                    'eta_s_best_case': round(dist / WALK_SPEED_MS, 1),
                    'from': [round(occ.x, 1), round(occ.z, 1)]}

    def add_marker(self, by: str, label: str,
                   x: Optional[float] = None,
                   z: Optional[float] = None) -> Dict[str, Any]:
        """Leave a marker. Defaults to where the placer is standing."""
        with self._lock:
            placer = self.state.occupants.get(by)
            if placer is None:
                return {'error': f"no occupant named {by!r} in this world"}
            mx = placer.x if x is None else float(x)
            mz = placer.z if z is None else float(z)
            if not self.terrain.in_bounds(mx, mz):
                return {'error': f"({mx:.1f}, {mz:.1f}) is outside the world"}
            self._marker_seq += 1
            marker = Marker(id=self._marker_seq, label=str(label)[:60],
                            by=by, x=mx, y=self.terrain.height_at(mx, mz),
                            z=mz, ts=time.time())
            self.state.markers.append(marker)
            # Oldest fall off rather than accumulating forever; the world
            # has no persistence yet, so a cap is the only bound.
            if len(self.state.markers) > MAX_MARKERS:
                self.state.markers = self.state.markers[-MAX_MARKERS:]
            return {'placed': True, 'id': marker.id, 'label': marker.label,
                    'pos': [round(mx, 1), round(mz, 1)]}

    def resolve_target(self, name: str, toward: str) -> Dict[str, Any]:
        """Turn 'toward: Bruce' into a coordinate just short of them."""
        with self._lock:
            me = self.state.occupants.get(name)
            them = self.state.occupants.get(toward)
        if me is None:
            return {'error': f"no occupant named {name!r}"}
        if them is None:
            return {'error': f"no occupant named {toward!r} in this world"}
        if not self.terrain.can_see(me.x, me.z, them.x, them.z):
            # You cannot walk to someone you cannot see. Ask them where they
            # are and move to the coordinate — which is the point: it makes
            # saying so out loud do real work.
            return {'error': f"you cannot see {toward} from here, so you "
                             f"cannot walk to them. Ask where they are and "
                             f"move to the coordinate."}
        dist = me.distance_to(them)
        if dist <= ARRIVE_TOL_M * 2:
            return {'error': f"already standing with {toward}"}
        # Stop a conversational distance short rather than inside them.
        keep = min(2.5, dist - ARRIVE_TOL_M)
        f = (dist - keep) / dist
        return {'x': me.x + (them.x - me.x) * f,
                'z': me.z + (them.z - me.z) * f}

    # -- reads ------------------------------------------------------------

    def snapshot_json(self) -> str:
        with self._lock:
            return self.state.to_json()

    def snapshot_for(self, viewer: Optional[str]) -> str:
        """State as one occupant can perceive it.

        The browser is a viewer like any other. Without this it receives
        every position and every marker while the agents' tools are fogged
        — which hands the human an information advantage in exactly the
        experiments the fog exists to make possible.

        Visibility is recomputed on a slower cadence than the 20 Hz tick
        and cached: a ray-march per entity per client per tick is real CPU,
        and someone appearing a fraction of a second late is imperceptible.
        Positions still stream at full rate; only the visible *set* is
        cached.
        """
        with self._lock:
            me = self.state.occupants.get(viewer or '')
            if me is None:
                return self.state.to_json()      # unbound viewer: no fog
            cached = self._vis_cache.get(me.name)
            if cached is None or self.state.tick - cached[0] >= _VIS_EVERY_TICKS:
                names = {o.name for o in self.state.occupants.values()
                         if o.name == me.name
                         or self.terrain.can_see(me.x, me.z, o.x, o.z)}
                marks = {m.id for m in self.state.markers
                         if self.terrain.can_see(me.x, me.z, m.x, m.z)}
                self._vis_cache[me.name] = (self.state.tick, names, marks)
            else:
                _, names, marks = cached
            visible = WorldState(
                occupants={o.name: o for o in self.state.occupants.values()
                           if o.name in names},
                markers=[m for m in self.state.markers if m.id in marks],
                tick=self.state.tick, ts=self.state.ts)
            return visible.to_json()

    def terrain_payload(self) -> Dict[str, Any]:
        return self._terrain_payload

    def human_name(self) -> Optional[str]:
        """Which occupant the browser drives. First human wins."""
        with self._lock:
            for occ in self.state.occupants.values():
                if occ.kind == 'human':
                    return occ.name
        return None

    def look(self, name: str, radius: float) -> Dict[str, Any]:
        """Structured perception for one occupant. Formatting lives in the
        tool; this stays machine-shaped."""
        with self._lock:
            me = self.state.occupants.get(name)
            if me is None:
                return {'error': f"no occupant named {name!r} in this world. "
                                 f"Present: "
                                 f"{', '.join(self.state.occupants) or '(none)'}"}
            others = [o for o in self.state.occupants.values()
                      if o.name != name and o.online]
            # Fog: you report what you can see, not what exists. Anyone out
            # of range or behind a rise is simply absent from the report.
            hidden = 0
            seen = []
            for o in others:
                if not self.terrain.can_see(me.x, me.z, o.x, o.z):
                    hidden += 1
                    continue
                d = me.distance_to(o)
                seen.append({
                    'name': o.name, 'kind': o.kind,
                    'distance_m': round(d, 1),
                    'bearing_deg': round(math.degrees(me.bearing_to(o)) % 360, 0),
                    'pos': [round(o.x, 1), round(o.z, 1)],
                    'gait': o.gait,
                    'looking_at_me': o.facing_offset(me) < 0.5,
                    'in_my_view': me.facing_offset(o) < 1.0,
                })
            seen.sort(key=lambda s: s['distance_m'])
            me_pos = [round(me.x, 1), round(me.z, 1)]
            me_head = round(math.degrees(me.heading) % 360, 0)
            me_gait, me_goal = me.gait, me.goal
            # Markers obey the same fog as people — one left over the hill
            # is not visible, which is what makes going to look worthwhile.
            marks = []
            for mk in self.state.markers:
                if not self.terrain.can_see(me.x, me.z, mk.x, mk.z):
                    continue
                marks.append({
                    'id': mk.id, 'label': mk.label, 'by': mk.by,
                    'pos': [round(mk.x, 1), round(mk.z, 1)],
                    'distance_m': round(math.hypot(mk.x - me.x,
                                                   mk.z - me.z), 1),
                    'bearing_deg': round(math.degrees(
                        math.atan2(mk.x - me.x, mk.z - me.z)) % 360, 0),
                })
            marks.sort(key=lambda m: m['distance_m'])

        nearby_trees = sum(
            1 for t in self.terrain.trees
            if abs(t['x'] - me_pos[0]) < radius and abs(t['z'] - me_pos[1]) < radius)
        return {
            'me': {'name': name, 'pos': me_pos, 'heading_deg': me_head,
                   'gait': me_gait, 'goal': me_goal},
            'ground': self.terrain.describe(me_pos[0], me_pos[1]),
            'tree_count': nearby_trees,
            'radius_m': radius,
            'occupants': seen,
            'extent_m': terrain_mod.EXTENT_M,
            'sight_m': round(self.terrain.sight_range(me_pos[0], me_pos[1]), 0),
            # How many others exist but cannot be seen from here. The count
            # only — knowing someone is out there is a different thing from
            # knowing where, and the second is what has to be asked for.
            'out_of_sight': hidden,
            'markers': marks,
        }


# --------------------------------------------------------------- HTTP API

def _make_handler(world: World):
    class _Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, fmt, *args):     # quiet; we have a logger
            logger.debug(f"http: {fmt % args}")

        def _send(self, code: int, body: bytes, ctype: str) -> None:
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj: Any, code: int = 200) -> None:
            self._send(code, json.dumps(obj).encode('utf-8'),
                       'application/json')

        def do_GET(self):
            u = urlparse(self.path)
            q = parse_qs(u.query)
            if u.path == '/health':
                return self._json({'ok': True, 'ws_port': WS_PORT})
            if u.path == '/state':
                return self._send(200, world.snapshot_json().encode(),
                                  'application/json')
            if u.path == '/terrain':
                return self._json(world.terrain_payload())
            if u.path == '/look':
                name = (q.get('name') or [''])[0]
                radius = float((q.get('radius') or ['30'])[0])
                return self._json(world.look(name, radius))
            return self._static(u.path)

        def do_POST(self):
            u = urlparse(self.path)
            length = int(self.headers.get('Content-Length') or 0)
            try:
                body = json.loads(self.rfile.read(length) or b'{}')
            except ValueError:
                return self._json({'error': 'body must be JSON'}, 400)
            if u.path == '/move':
                name = str(body.get('name') or '')
                if body.get('toward'):
                    target = world.resolve_target(name, str(body['toward']))
                    if 'error' in target:
                        return self._json(target)
                    return self._json(world.set_goal(
                        name, target['x'], target['z']))
                try:
                    x, z = float(body['x']), float(body['z'])
                except (KeyError, TypeError, ValueError):
                    return self._json(
                        {'error': 'move needs x and z, or toward'}, 400)
                return self._json(world.set_goal(name, x, z))
            if u.path == '/mark':
                name = str(body.get('name') or '')
                label = str(body.get('label') or '').strip()
                if not label:
                    return self._json({'error': 'mark needs a label'}, 400)
                x = body.get('x')
                z = body.get('z')
                try:
                    x = None if x is None else float(x)
                    z = None if z is None else float(z)
                except (TypeError, ValueError):
                    return self._json({'error': 'x and z must be numbers'}, 400)
                if (x is None) != (z is None):
                    return self._json(
                        {'error': 'give both x and z, or neither'}, 400)
                return self._json(world.add_marker(name, label, x, z))
            return self._json({'error': f'no route {u.path}'}, 404)

        def _static(self, path: str) -> None:
            rel = 'index.html' if path in ('/', '') else path.lstrip('/')
            target = (STATIC_DIR / rel).resolve()
            if not str(target).startswith(str(STATIC_DIR.resolve())) \
                    or not target.is_file():
                return self._json({'error': f'not found: {path}'}, 404)
            ctype = _MIME.get(target.suffix, 'application/octet-stream')
            self._send(200, target.read_bytes(), ctype)

    return _Handler


# ------------------------------------------------------------------ WS + tick

async def _serve(world: World) -> None:
    import websockets

    clients: Dict[Any, Optional[str]] = {}

    async def handler(ws):
        # Each connection is bound to the occupant it drives, so its view
        # can be fogged from that body's vantage.
        clients[ws] = world.human_name()
        logger.info(f"world: browser connected ({len(clients)} total)")
        try:
            await ws.send(json.dumps({'type': 'terrain',
                                      'you': world.human_name(),
                                      **world.terrain_payload()}))
            async for raw in ws:
                try:
                    msg = json.loads(raw)
                except ValueError:
                    continue
                if msg.get('type') == 'input':
                    if msg.get('name'):
                        clients[ws] = str(msg['name'])
                    world.set_human_pose(
                        str(msg.get('name') or ''), float(msg.get('x', 0)),
                        float(msg.get('z', 0)), float(msg.get('heading', 0)))
                elif msg.get('type') == 'mark':
                    world.add_marker(str(msg.get('name') or ''),
                                     str(msg.get('label') or 'mark'))
        except Exception as e:
            logger.debug(f"world: ws closed: {e}")
        finally:
            clients.pop(ws, None)

    async with websockets.serve(handler, '127.0.0.1', WS_PORT,
                                max_size=2 ** 22):
        logger.info(f"world: ws listening on ws://127.0.0.1:{WS_PORT}")
        dt = 1.0 / TICK_HZ
        last = time.monotonic()
        while True:
            await asyncio.sleep(dt)
            now = time.monotonic()
            world.step(min(now - last, 0.25))   # clamp after a stall
            last = now
            for ws, viewer in list(clients.items()):
                try:
                    await ws.send(world.snapshot_for(viewer))
                except Exception:
                    clients.pop(ws, None)


def parse_occupants(spec: str) -> List[tuple]:
    """"Bruce:human,Jill:agent" -> [("Bruce","human"), ("Jill","agent")]"""
    out = []
    for entry in (spec or '').split(','):
        entry = entry.strip()
        if not entry:
            continue
        name, _, kind = entry.partition(':')
        out.append((name.strip(), (kind.strip() or 'agent')))
    return out


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s %(levelname)s %(name)s: %(message)s')
    world = World()
    spec = os.environ.get('WORLD_OCCUPANTS', 'Bruce:human,Jill:agent')
    for i, (name, kind) in enumerate(parse_occupants(spec)):
        occ = world.add_occupant(name, kind, i)
        logger.info(f"world: {name} ({kind}) at "
                    f"({occ.x:.0f}, {occ.z:.0f})")
    logger.info(f"world: terrain seed={world.terrain.seed} "
                f"fingerprint={world.terrain.fingerprint()} "
                f"trees={len(world.terrain.trees)}")

    httpd = ThreadingHTTPServer(('127.0.0.1', HTTP_PORT), _make_handler(world))
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    logger.info(f"world: http listening on http://127.0.0.1:{HTTP_PORT}")

    try:
        asyncio.run(_serve(world))
    except KeyboardInterrupt:
        logger.info("world: shutting down")


if __name__ == '__main__':
    main()
