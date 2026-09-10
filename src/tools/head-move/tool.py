"""head-move — aim the ChatterBot pan/tilt camera head.

ReAct entry-point. Publishes one chatter/head/cmd over the shared ChatterLink
(isolated Zenoh session to the Pi) and reports the settled pose. See Skill.md
for the args contract and src/utils/chatter_link.py for the wire details.

Aim only: no gestures. Nod/shake/scan were removed 2026-09-10 at Jill's
request (a head that nods when she replies is theatre, not something she is
doing). Turning toward a talker is done by the voice sensor, not here.
"""
import logging
import os
import sys

# Make sibling src/ modules importable when launched from src/ as cwd.
_THIS = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.dirname(os.path.dirname(_THIS))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from utils.chatter_link import (  # noqa: E402
    get_link, router, clamp_pose, NEUTRAL_PAN, NEUTRAL_TILT,
    PAN_MIN, PAN_MAX, TILT_MIN, TILT_MAX)

_log = logging.getLogger(__name__)


def _as_angle(v, name, lo, hi):
    """Coerce v to a float in [lo, hi]; returns (angle, error_text)."""
    try:
        a = float(v)
    except (TypeError, ValueError):
        return None, f"{name} must be a number {lo}-{hi}, got {v!r}"
    if not lo <= a <= hi:
        return None, f"{name} must be within {lo}-{hi}, got {a}"
    return a, None


def react_invoke(args, *, character_name=None, backend=None, logger=None):
    """See Skill.md for the args contract. Returns {status, text}."""
    pan_in = args.get("pan")
    tilt_in = args.get("tilt")
    if args.get("center"):
        pan_in, tilt_in = NEUTRAL_PAN, NEUTRAL_TILT
    if pan_in is None and tilt_in is None:
        return {"status": "error",
                "text": "head-move needs pan and/or tilt, or center: true"}
    pan = tilt = None
    if pan_in is not None:
        pan, err = _as_angle(pan_in, "pan", PAN_MIN, PAN_MAX)
        if err:
            return {"status": "error", "text": err}
    if tilt_in is not None:
        tilt, err = _as_angle(tilt_in, "tilt", TILT_MIN, TILT_MAX)
        if err:
            return {"status": "error", "text": err}
    if pan is not None and tilt is not None:
        pan, tilt = clamp_pose(pan, tilt)

    smooth = args.get("smooth", True)

    link = get_link()
    err = link.ensure()
    if err is not None:
        return {"status": "error",
                "text": f"ChatterBot not reachable at {router()} ({err})"}

    try:
        result = link.send_head_cmd(pan=pan, tilt=tilt, smooth=bool(smooth))
    except Exception as e:
        _log.exception("head-move: send failed")
        return {"status": "error",
                "text": f"head command failed: {type(e).__name__}: {e}"}

    what = f"pan={pan} tilt={tilt}"
    pose = f"pan={result.get('pan')} tilt={result.get('tilt')}"
    if result.get("confirmed"):
        return {"status": "ok", "text": f"head moved ({what}); settled at {pose}."}
    # Command went out but no arrival confirmation — head_service may be slow,
    # absent, or only the router is up. Report honestly rather than claim done.
    return {"status": "ok",
            "text": (f"head command sent ({what}) but no arrival confirmation; "
                     f"last known pose {pose} (head_service may be down).")}
