"""fac-observe — entities near a point + telemetry, one situational read."""
import logging
import os
import sys

_THIS = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.dirname(os.path.dirname(_THIS))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from utils.factorio_link import (  # noqa: E402
    bridge_get, fmt_entities, fmt_pos, parse_number, run_tool,
)

_log = logging.getLogger(__name__)


def _impl(args):
    x, err = parse_number(args, "x", required=False)
    if err:
        return {"status": "error", "text": err}
    y, err = parse_number(args, "y", required=False)
    if err:
        return {"status": "error", "text": err}
    radius, err = parse_number(args, "radius", required=False, default=15)
    if err:
        return {"status": "error", "text": err}

    st = bridge_get("/status")
    if x is None or y is None:
        x, y = st["position"]["x"], st["position"]["y"]
    obs = bridge_get("/observe", {"x": x, "y": y, "radius": radius})

    lines = [
        f"You are at {fmt_pos(st['position'])}"
        + (" (walking)." if st.get("walking") else "."),
        f"Entities within {obs.get('radius')} of {fmt_pos(x, y)}:",
        fmt_entities(obs.get("entities")),
    ]

    tel = bridge_get("/telemetry")
    production = tel.get("production") or {}
    flows = {
        item: f
        for item, f in production.items()
        if isinstance(f, dict) and (f.get("produced_1m") or f.get("consumed_1m"))
    }
    if flows:
        lines.append("Production (last minute): " + "; ".join(
            f"{item} +{f.get('produced_1m', 0)}/-{f.get('consumed_1m', 0)}"
            for item, f in sorted(flows.items())
        ))
    else:
        lines.append("No production flows in the last minute.")
    alerts = tel.get("alerts") or []
    if alerts:
        lines.append(f"Alerts: {len(alerts)} recent.")
    return {"status": "ok", "text": "\n".join(lines)}


def react_invoke(args, *, character_name=None, backend=None, logger=None):
    return run_tool(lambda: _impl(args or {}), logger or _log)
