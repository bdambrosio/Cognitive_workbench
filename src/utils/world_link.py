"""world_link — shared HTTP client for the shared-world server.

One canonical implementation for every world-* tool (src/tools/world-*/):
thin GET/POST against the world process (src/world/server.py, default
http://127.0.0.1:8791), plus the perception formatting both tools render.

Transport plumbing and argument coercion come from utils.tool_helpers,
which factorio_link also builds on.
"""
import logging
import os
from typing import Any, Dict, Optional

from utils.tool_helpers import (  # noqa: F401  (re-exported for the tools)
    BridgeError, parse_number, request_json, run_tool,
)

logger = logging.getLogger(__name__)

_HTTP_TIMEOUT_S = float(os.environ.get("WORLD_HTTP_TIMEOUT_S", "10"))

_OFFLINE_HINT = ("the world is not running (start the agent with --world, "
                 "or run `python -m world.display`)")

_COMPASS = ['N', 'NNE', 'NE', 'ENE', 'E', 'ESE', 'SE', 'SSE',
            'S', 'SSW', 'SW', 'WSW', 'W', 'WNW', 'NW', 'NNW']


def base_url() -> str:
    return os.environ.get("WORLD_URL", "http://127.0.0.1:8791").rstrip("/")


def _request(method: str, path: str, **kwargs) -> Dict[str, Any]:
    # Module-level indirection so tests can patch the bridge out, matching
    # the seam tests/test_fac_tools.py uses for factorio_link.
    return request_json(base_url(), method, path,
                        timeout_s=_HTTP_TIMEOUT_S,
                        offline_hint=_OFFLINE_HINT, **kwargs)


def world_get(path: str, params: Optional[dict] = None) -> Dict[str, Any]:
    return _request("GET", path, params=params or {})


def world_post(path: str, body: Optional[dict] = None) -> Dict[str, Any]:
    return _request("POST", path, json=body or {})


def compass(bearing_deg: float) -> str:
    """Compass point for a bearing — 'NE' reads better than '045°' in a
    perception line the agent has to reason over."""
    return _COMPASS[int((float(bearing_deg) % 360) / 22.5 + 0.5) % 16]


def fmt_occupant(o: dict) -> str:
    gait = 'walking' if o.get('gait') == 'walk' else 'standing still'
    bits = [f"{o['name']}"]
    if o.get('kind') == 'human':
        bits.append("(the human)")
    line = (f"  {' '.join(bits)} — {o['distance_m']} m away to the "
            f"{compass(o['bearing_deg'])}, at "
            f"({o['pos'][0]}, {o['pos'][1]}), {gait}")
    notes = []
    if o.get('in_my_view'):
        notes.append("in your view")
    else:
        notes.append("behind you")
    if o.get('looking_at_me'):
        notes.append("looking at you")
    return line + f" — {', '.join(notes)}"


def fmt_look(d: dict) -> str:
    """Render a /look response as the agent's situational read."""
    me = d['me']
    lines = [
        f"You are at ({me['pos'][0]}, {me['pos'][1]}) facing "
        f"{compass(me['heading_deg'])} — {d['ground']}.",
        f"About {d['tree_count']} trees within {int(d['radius_m'])} m. "
        f"The world is {int(d['extent_m'])} m across.",
    ]
    if d.get('sight_m'):
        lines.append(f"You can see about {int(d['sight_m'])} m from here; "
                     f"rising ground blocks the rest.")
    if me.get('goal'):
        lines.append(f"You are walking toward "
                     f"({me['goal'][0]:.1f}, {me['goal'][1]:.1f}).")
    others = d.get('occupants') or []
    hidden = int(d.get('out_of_sight') or 0)
    lines.append("")
    if others:
        lines.append("In sight:")
        lines.extend(fmt_occupant(o) for o in others)
    else:
        lines.append("You cannot see anyone from here.")
    if hidden:
        # Presence without position: enough to know to ask, not enough to
        # act on. Asking is the point.
        who = "someone" if hidden == 1 else f"{hidden} others"
        lines.append(f"({who} else is in the world but out of sight — you "
                     f"would have to ask, or go and look.)")

    marks = d.get('markers') or []
    if marks:
        lines.append("")
        lines.append("Markers in sight:")
        for m in marks:
            placer = "you left" if m['by'] == me['name'] else f"{m['by']} left"
            lines.append(f"  \"{m['label']}\" — {placer} it, "
                         f"{m['distance_m']} m to the "
                         f"{compass(m['bearing_deg'])}, at "
                         f"({m['pos'][0]}, {m['pos'][1]})")
    return "\n".join(lines)
