"""fac-insert — move items from the agent's inventory into an entity."""
import logging
import os
import sys

_THIS = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.dirname(os.path.dirname(_THIS))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from utils.factorio_link import (  # noqa: E402
    bridge_post, fail_text, fmt_entity, parse_transfer_args, run_tool,
)

_log = logging.getLogger(__name__)


def _impl(args):
    body, err = parse_transfer_args(args)
    if err:
        return {"status": "error", "text": err}
    resp = bridge_post("/act/insert", body)
    if resp.get("ok"):
        result = resp.get("result")
        tail = ("\nTarget now: " + fmt_entity(result)) if isinstance(result, dict) else ""
        return {"status": "ok",
                "text": f"Inserted {body['count']} {body['item']} into {body['target']}." + tail}
    return {"status": "error", "text": fail_text(resp)}


def react_invoke(args, *, character_name=None, backend=None, logger=None):
    return run_tool(lambda: _impl(args or {}), logger or _log)
