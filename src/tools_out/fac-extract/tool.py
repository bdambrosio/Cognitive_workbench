"""fac-extract — move items from an entity into the agent's inventory."""
import logging
import os
import sys

_THIS = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.dirname(os.path.dirname(_THIS))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from utils.factorio_link import (  # noqa: E402
    bridge_post, fail_text, parse_transfer_args, run_tool,
)

_log = logging.getLogger(__name__)


def _impl(args):
    body, err = parse_transfer_args(args)
    if err:
        return {"status": "error", "text": err}
    resp = bridge_post("/act/extract", body)
    if resp.get("ok"):
        return {"status": "ok",
                "text": f"Extracted {body['count']} {body['item']} from {body['target']} "
                        f"into your inventory."}
    return {"status": "error", "text": fail_text(resp)}


def react_invoke(args, *, character_name=None, backend=None, logger=None):
    return run_tool(lambda: _impl(args or {}), logger or _log)
