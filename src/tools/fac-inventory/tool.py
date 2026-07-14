"""fac-inventory — the agent character's inventory."""
import logging
import os
import sys

_THIS = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.dirname(os.path.dirname(_THIS))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from utils.factorio_link import bridge_get, run_tool  # noqa: E402

_log = logging.getLogger(__name__)


def _impl():
    inv = bridge_get("/inventory").get("inventory") or {}
    if not inv:
        return {"status": "ok", "text": "You are carrying nothing."}
    items = ", ".join(f"{k}:{v}" for k, v in sorted(inv.items()))
    return {"status": "ok", "text": f"Carrying: {items}"}


def react_invoke(args, *, character_name=None, backend=None, logger=None):
    return run_tool(_impl, logger or _log)
