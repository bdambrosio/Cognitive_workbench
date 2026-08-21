"""fac-pickup — pick up a placed entity or loose ground items into the agent's inventory."""
import logging
import os
import sys

_THIS = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.dirname(os.path.dirname(_THIS))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from utils.factorio_link import bridge_post, fail_text, parse_number, run_tool  # noqa: E402

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
    resp = bridge_post("/act/pickup", {"prototype": str(prototype), "x": x, "y": y})
    if resp.get("ok"):
        return {"status": "ok", "text": f"Picked up {prototype}."}
    return {"status": "error", "text": fail_text(resp)}


def react_invoke(args, *, character_name=None, backend=None, logger=None):
    return run_tool(lambda: _impl(args or {}), logger or _log)
