"""fac-nearest — nearest resource patch within 500 tiles."""
import logging
import os
import sys

_THIS = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.dirname(os.path.dirname(_THIS))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from utils.factorio_link import bridge_get, fail_text, fmt_pos, run_tool  # noqa: E402

_log = logging.getLogger(__name__)


def _impl(args):
    resource = args.get("resource")
    if not resource:
        return {"status": "error", "text": "missing required arg: resource"}
    resp = bridge_get("/nearest", {"resource": str(resource).strip()})
    if not resp.get("ok"):
        return {"status": "error", "text": fail_text(resp)}
    pos = resp.get("result") or {}
    text = f"Nearest {str(resource).strip()} is at {fmt_pos(pos)}"
    st = bridge_get("/status")
    me = st.get("position") or {}
    try:
        dist = ((me["x"] - pos["x"]) ** 2 + (me["y"] - pos["y"]) ** 2) ** 0.5
        text += f", {dist:.0f} tiles from you"
    except (KeyError, TypeError):
        pass
    return {"status": "ok", "text": text + "."}


def react_invoke(args, *, character_name=None, backend=None, logger=None):
    return run_tool(lambda: _impl(args or {}), logger or _log)
