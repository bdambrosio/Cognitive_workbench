"""fac-walk — walk the agent character to a position."""
import logging
import os
import sys

_THIS = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.dirname(os.path.dirname(_THIS))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from utils.factorio_link import (  # noqa: E402
    bridge_post, fail_text, fmt_pos, parse_number, run_tool,
)

_log = logging.getLogger(__name__)


def _impl(args):
    x, err = parse_number(args, "x")
    if err:
        return {"status": "error", "text": err}
    y, err = parse_number(args, "y")
    if err:
        return {"status": "error", "text": err}
    resp = bridge_post("/act/walk", {"x": x, "y": y})
    if resp.get("ok"):
        return {"status": "ok", "text": f"Arrived at {fmt_pos(resp['position'])}."}
    return {"status": "error", "text": fail_text(resp)}


def react_invoke(args, *, character_name=None, backend=None, logger=None):
    return run_tool(lambda: _impl(args or {}), logger or _log)
