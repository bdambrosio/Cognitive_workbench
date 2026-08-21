"""fac-say — speak in Factorio multiplayer chat."""
import logging
import os
import sys

_THIS = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.dirname(os.path.dirname(_THIS))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from utils.factorio_link import bridge_post, fail_text, run_tool  # noqa: E402

_log = logging.getLogger(__name__)


def _impl(args):
    message = args.get("message")
    if not message or not str(message).strip():
        return {"status": "error", "text": "missing required arg: message"}
    resp = bridge_post("/say", {"message": str(message).strip()})
    if resp.get("ok"):
        return {"status": "ok", "text": f"Said in game chat: {str(message).strip()}"}
    return {"status": "error", "text": fail_text(resp)}


def react_invoke(args, *, character_name=None, backend=None, logger=None):
    return run_tool(lambda: _impl(args or {}), logger or _log)
