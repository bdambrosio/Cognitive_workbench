"""world-mark — leave something in the world instead of saying it."""
import logging
import os
import sys

_THIS = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.dirname(os.path.dirname(_THIS))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from utils.world_link import (  # noqa: E402
    parse_number, run_tool, world_post,
)

_log = logging.getLogger(__name__)


def _impl(args, character_name):
    label = str(args.get("label") or "").strip()
    if not label:
        return {"status": "error",
                "text": "world-mark needs a `label` — an unlabelled marker "
                        "tells nobody anything"}

    body = {"name": character_name, "label": label}
    has_x, has_z = args.get("x") is not None, args.get("z") is not None
    if has_x != has_z:
        return {"status": "error",
                "text": "give both `x` and `z` to place a marker away from "
                        "you, or neither to drop it at your feet"}
    if has_x:
        x, err = parse_number(args, "x")
        if err:
            return {"status": "error", "text": err}
        z, err = parse_number(args, "z")
        if err:
            return {"status": "error", "text": err}
        body["x"], body["z"] = x, z

    resp = world_post("/mark", body)
    if resp.get("error"):
        return {"status": "error", "text": resp["error"]}

    pos = resp["pos"]
    where = "at your feet" if not has_x else f"at ({pos[0]}, {pos[1]})"
    return {"status": "ok",
            "text": (f"Left a marker {where}: \"{resp['label']}\" "
                     f"(#{resp['id']}). It stays there. Anyone who can see "
                     f"that spot will find it in their next world-look.")}


def react_invoke(args, *, character_name=None, backend=None, logger=None):
    """ReAct entry-point — see Skill.md for the args contract."""
    if not character_name:
        return {"status": "error",
                "text": "world-mark needs a character name to know who is "
                        "leaving the marker"}
    return run_tool(lambda: _impl(args or {}, character_name),
                    logger or _log)
