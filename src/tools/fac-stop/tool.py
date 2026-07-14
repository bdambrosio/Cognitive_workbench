"""fac-stop — abort the agent character's current movement."""
import logging
import os
import sys

_THIS = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.dirname(os.path.dirname(_THIS))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from utils.factorio_link import bridge_post, fmt_pos, run_tool  # noqa: E402

_log = logging.getLogger(__name__)


def _impl():
    resp = bridge_post("/act/stop")
    return {"status": "ok", "text": f"Stopped at {fmt_pos(resp.get('position', {}))}."}


def react_invoke(args, *, character_name=None, backend=None, logger=None):
    return run_tool(_impl, logger or _log)
