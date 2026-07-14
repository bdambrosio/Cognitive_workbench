"""fac-connect — build a validated belt/pipe/pole run between two positions."""
import logging
import os
import sys

_THIS = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.dirname(os.path.dirname(_THIS))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from utils.factorio_link import (  # noqa: E402
    bridge_post, fail_text, fmt_entities, fmt_pos, parse_number, run_tool,
)

_log = logging.getLogger(__name__)


def _impl(args):
    coords = {}
    for name in ("from_x", "from_y", "to_x", "to_y"):
        coords[name], err = parse_number(args, name)
        if err:
            return {"status": "error", "text": err}
    prototype = str(args.get("prototype") or "transport-belt")
    dry_run = str(args.get("dry_run", "")).lower() in ("true", "1", "yes")

    resp = bridge_post("/act/connect", {
        "source": {"x": coords["from_x"], "y": coords["from_y"]},
        "target": {"x": coords["to_x"], "y": coords["to_y"]},
        "prototype": prototype,
        "dry_run": dry_run,
    })
    if not resp.get("ok"):
        return {"status": "error", "text": fail_text(resp)}
    result = resp.get("result") or {}
    span = f"{fmt_pos(coords['from_x'], coords['from_y'])} to {fmt_pos(coords['to_x'], coords['to_y'])}"
    needed = result.get("number_of_entities")
    if dry_run:
        return {"status": "ok",
                "text": f"Dry run: connection {span} is feasible, needs {needed} {prototype}."}
    placed = result.get("entities") or []
    return {"status": "ok",
            "text": f"Connected {span} with {needed} {prototype}:\n" + fmt_entities(placed)}


def react_invoke(args, *, character_name=None, backend=None, logger=None):
    return run_tool(lambda: _impl(args or {}), logger or _log)
