"""world-look — where you are, and who else is here."""
import logging
import os
import sys

_THIS = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.dirname(os.path.dirname(_THIS))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from utils.world_link import (  # noqa: E402
    fmt_look, parse_number, run_tool, world_get,
)

_log = logging.getLogger(__name__)

_MAX_RADIUS = 120.0


def _impl(args, character_name):
    radius, err = parse_number(args, "radius", required=False, default=30.0)
    if err:
        return {"status": "error", "text": err}
    radius = max(5.0, min(float(radius), _MAX_RADIUS))

    data = world_get("/look", {"name": character_name, "radius": radius})
    if data.get("error"):
        return {"status": "error", "text": data["error"]}
    return {"status": "ok", "text": fmt_look(data)}


def react_invoke(args, *, character_name=None, backend=None, logger=None):
    """ReAct entry-point — see Skill.md for the args contract. The occupant
    is the calling character: each agent looks out of its own body."""
    if not character_name:
        return {"status": "error",
                "text": "world-look needs a character name to know whose "
                        "eyes to look through"}
    return run_tool(lambda: _impl(args or {}, character_name),
                    logger or _log)
