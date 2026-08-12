"""world-move — post a walking goal; the body walks while the agent thinks."""
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
    toward = args.get("toward")
    body = {"name": character_name}

    if toward:
        body["toward"] = str(toward)
    else:
        x, err = parse_number(args, "x")
        if err:
            return {"status": "error",
                    "text": "world-move needs `toward` (an occupant's name) "
                            "or both `x` and `z`"}
        z, err = parse_number(args, "z")
        if err:
            return {"status": "error", "text": err}
        body["x"], body["z"] = x, z

    resp = world_post("/move", body)
    if resp.get("error"):
        return {"status": "error", "text": resp["error"]}

    dest = f"toward {toward}" if toward else f"to ({body['x']:.1f}, {body['z']:.1f})"
    return {"status": "ok",
            "text": (f"Walking {dest} — {resp['distance_m']} m, "
                     f"{resp['eta_s_best_case']} s at best. Rough or wooded "
                     f"ground on the way can take up to twice that, and you "
                     f"will only know which by walking it. You are moving "
                     f"now; this call does not wait for you to arrive. Use "
                     f"world-look on a later turn to see where you got to.")}


def react_invoke(args, *, character_name=None, backend=None, logger=None):
    """ReAct entry-point — see Skill.md for the args contract."""
    if not character_name:
        return {"status": "error",
                "text": "world-move needs a character name to know which "
                        "body to move"}
    return run_tool(lambda: _impl(args or {}, character_name),
                    logger or _log)
