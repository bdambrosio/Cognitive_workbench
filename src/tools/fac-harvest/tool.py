"""fac-harvest — mine resources / chop trees into the agent's inventory."""
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
    x, err = parse_number(args, "x")
    if err:
        return {"status": "error", "text": err}
    y, err = parse_number(args, "y")
    if err:
        return {"status": "error", "text": err}
    count, err = parse_number(args, "count", required=False, default=1)
    if err:
        return {"status": "error", "text": err}
    resp = bridge_post("/act/harvest", {"x": x, "y": y, "count": int(count)})
    if resp.get("ok"):
        got = resp.get("result")
        got = int(got) if isinstance(got, (int, float)) else count
        return {"status": "ok", "text": f"Harvested {got}."}
    return {"status": "error", "text": fail_text(resp)}


def react_invoke(args, *, character_name=None, backend=None, logger=None):
    return run_tool(lambda: _impl(args or {}), logger or _log)
