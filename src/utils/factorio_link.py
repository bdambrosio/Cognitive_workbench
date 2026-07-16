"""factorio_link — shared HTTP client for the Factorio game bridge.

One canonical implementation for every fac-* tool (src/tools/fac-*/):
thin GET/POST against the bridge process (factorio/bridge/, FastAPI at
FACTORIO_URL, default http://localhost:3004), plus the shared response
formatting — compact entity lines and the verbatim deviation block the
ReAct loop reasons over (docs/factorio-bridge-architecture.md,
"CW-side tools").
"""
import logging
import os
from typing import Any, Dict, Optional

import requests

logger = logging.getLogger(__name__)

# Walks block in the bridge until arrival (up to its 120s walk timeout),
# so the HTTP timeout sits above that.
_HTTP_TIMEOUT_S = float(os.environ.get("FACTORIO_HTTP_TIMEOUT_S", "150"))

# Factorio 2.0 direction encoding is 16-step; the four cardinals are all v1 uses.
DIRECTIONS = {"north": 0, "east": 4, "south": 8, "west": 12}
_DIRECTION_NAMES = {v: k for k, v in DIRECTIONS.items()}


class BridgeError(Exception):
    """Bridge unreachable or returned a non-JSON/non-200 transport failure."""


def base_url() -> str:
    return os.environ.get("FACTORIO_URL", "http://localhost:3004").rstrip("/")


def _request(method: str, path: str, **kwargs) -> Dict[str, Any]:
    url = base_url() + path
    try:
        resp = requests.request(method, url, timeout=_HTTP_TIMEOUT_S, **kwargs)
    except requests.ConnectionError as e:
        raise BridgeError(
            f"factorio bridge unreachable at {base_url()} — the factory is "
            f"offline (start it with factorio/up.sh) [{e}]"
        )
    except requests.RequestException as e:
        raise BridgeError(f"factorio bridge request failed: {e}")
    if resp.status_code != 200:
        raise BridgeError(f"bridge HTTP {resp.status_code} on {path}: {resp.text[:300]}")
    return resp.json()


def bridge_get(path: str, params: Optional[dict] = None) -> Dict[str, Any]:
    return _request("GET", path, params=params or {})


def bridge_post(path: str, body: Optional[dict] = None) -> Dict[str, Any]:
    return _request("POST", path, json=body or {})


def run_tool(fn, logger_=None):
    """Standard react_invoke wrapper: fn() -> {status, text}, with transport
    failures surfaced as ERROR text instead of a raised exception."""
    log = logger_ or logger
    try:
        return fn()
    except BridgeError as e:
        log.warning(f"factorio tool: {e}")
        return {"status": "error", "text": str(e)}


# ----------------------------------------------------------------- coercion

def parse_direction(value, default: Optional[int] = None):
    """Accept a cardinal name or a Factorio 2.0 direction number (0/4/8/12).
    Returns (direction, error_text)."""
    if value is None:
        return default, None
    if isinstance(value, str) and value.strip().lower() in DIRECTIONS:
        return DIRECTIONS[value.strip().lower()], None
    try:
        d = int(value)
    except (TypeError, ValueError):
        return None, f"direction must be north/east/south/west (or 0/4/8/12), got {value!r}"
    if d not in _DIRECTION_NAMES:
        return None, f"direction number must be one of 0/4/8/12 (N/E/S/W), got {d}"
    return d, None


def parse_number(args: dict, name: str, required: bool = True, default=None):
    """Returns (value, error_text)."""
    raw = args.get(name)
    if raw is None:
        if required:
            return None, f"missing required arg: {name}"
        return default, None
    try:
        return float(raw), None
    except (TypeError, ValueError):
        return None, f"{name} must be a number, got {raw!r}"


def parse_transfer_args(args: dict):
    """Shared arg contract of fac-insert/fac-extract: item, count, x, y,
    target. Returns (body, error_text)."""
    item = args.get("item")
    target = args.get("target")
    if not item or not target:
        missing = [n for n, v in (("item", item), ("target", target)) if not v]
        return None, "missing required arg(s): " + ", ".join(missing)
    count, err = parse_number(args, "count")
    if err:
        return None, err
    x, err = parse_number(args, "x")
    if err:
        return None, err
    y, err = parse_number(args, "y")
    if err:
        return None, err
    return {"item": str(item), "count": int(count), "x": x, "y": y,
            "target": str(target)}, None


# --------------------------------------------------------------- formatting

def fmt_direction(d) -> str:
    return _DIRECTION_NAMES.get(d, str(d))


def fmt_pos(x, y=None) -> str:
    if isinstance(x, dict):
        x, y = x.get("x"), x.get("y")
    try:
        return f"({float(x):.1f}, {float(y):.1f})"
    except (TypeError, ValueError):
        return f"({x}, {y})"


def fmt_deviation(dev: dict) -> str:
    """The deviation report, verbatim enough for the ReAct loop to reason over."""
    lines = [f"deviation [{dev.get('kind', '?')}]: {dev.get('detail', '')}"]
    if dev.get("last_observed") is not None:
        lines.append(f"  previously observed here: {dev['last_observed'] or 'nothing'}")
    if dev.get("observed_now") is not None:
        lines.append(f"  observed there just now: {dev['observed_now'] or 'nothing'}")
    return "\n".join(lines)


def fail_text(resp: dict) -> str:
    """Uniform failure text: bridge error + deviation block when present."""
    parts = [str(resp.get("error", "unknown bridge failure"))]
    if resp.get("deviation"):
        parts.append(fmt_deviation(resp["deviation"]))
    return "\n".join(parts)


def _contents(e: dict) -> str:
    """Merge the entity's item-holding fields into 'name:count, ...'."""
    merged: Dict[str, int] = {}
    for field in ("inventory", "fuel", "furnace_source", "furnace_result",
                  "assembling_machine_input", "assembling_machine_output"):
        val = e.get(field)
        if isinstance(val, dict):
            for item, count in val.items():
                if isinstance(count, (int, float)) and count:
                    merged[item] = max(merged.get(item, 0), int(count))
    return ", ".join(f"{k}:{v}" for k, v in sorted(merged.items()))


def fmt_entity(e: dict) -> str:
    parts = [f"{e.get('name', '?')} @ {fmt_pos(e.get('position', {}))}"]
    if e.get("direction") not in (None, 0):
        parts.append(f"facing {fmt_direction(e['direction'])}")
    # Drills/inserters eject output at drop_position — the one tile a
    # receiving belt/chest must occupy. Surface it so the agent never
    # guesses (guessing put belts inside the drill's own footprint).
    if isinstance(e.get("drop_position"), dict):
        parts.append(f"outputs to {fmt_pos(e['drop_position'])}")
    if e.get("status") and e["status"] not in ("normal",):
        parts.append(f"status={e['status']}")
    warnings = e.get("warnings")
    if isinstance(warnings, list) and warnings:
        parts.append("warnings: " + "; ".join(str(w).strip("'") for w in warnings))
    contents = _contents(e)
    if contents:
        parts.append(f"contains {contents}")
    return "  ".join(parts)


def fmt_entities(entities, limit: int = 40) -> str:
    """Compact multi-entity listing; same-name swarms (belts, poles) are
    summarized as one line with count and extent."""
    entities = [e for e in entities or [] if isinstance(e, dict)]
    if not entities:
        return "nothing"
    by_name: Dict[str, list] = {}
    for e in entities:
        by_name.setdefault(str(e.get("name", "?")), []).append(e)
    lines = []
    for name, group in sorted(by_name.items()):
        if len(group) > 4:
            xs, ys = [], []
            for e in group:
                pos = e.get("position") or {}
                try:
                    xs.append(float(pos.get("x")))
                    ys.append(float(pos.get("y")))
                except (TypeError, ValueError):
                    continue
            extent = (f" spanning {fmt_pos(min(xs), min(ys))} to {fmt_pos(max(xs), max(ys))}"
                      if xs else "")
            lines.append(f"{name} x{len(group)}{extent}")
        else:
            lines.extend(fmt_entity(e) for e in group)
    if len(lines) > limit:
        lines = lines[:limit] + [f"... and {len(lines) - limit} more lines (narrow the radius)"]
    return "\n".join(lines)
