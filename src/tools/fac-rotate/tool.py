"""fac-rotate — rotate a placed entity."""
import logging
import os
import sys

_THIS = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.dirname(os.path.dirname(_THIS))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from utils.factorio_link import (  # noqa: E402
    bridge_post, fail_text, fmt_entity, parse_direction, parse_number, run_tool,
)

_log = logging.getLogger(__name__)


def _impl(args):
    prototype = args.get("prototype")
    if not prototype:
        return {"status": "error", "text": "missing required arg: prototype"}
    x, err = parse_number(args, "x")
    if err:
        return {"status": "error", "text": err}
    y, err = parse_number(args, "y")
    if err:
        return {"status": "error", "text": err}
    direction, err = parse_direction(args.get("direction"))
    if err or direction is None:
        return {"status": "error", "text": err or "missing required arg: direction"}
    resp = bridge_post("/act/rotate", {
        "prototype": str(prototype), "x": x, "y": y, "direction": direction,
    })
    if resp.get("ok"):
        result = resp.get("result")
        text = ("Rotated: " + fmt_entity(result)) if isinstance(result, dict) else "Rotated."
        return {"status": "ok", "text": text}
    return {"status": "error", "text": fail_text(resp)}


def react_invoke(args, *, character_name=None, backend=None, logger=None):
    return run_tool(lambda: _impl(args or {}), logger or _log)
