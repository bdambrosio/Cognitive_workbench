"""head-move — aim the ChatterBot pan/tilt head or play a gesture.

ReAct entry-point. Publishes one chatter/head/cmd over the shared ChatterLink
(isolated Zenoh session to the Pi) and reports the settled pose. See Skill.md
for the args contract and src/utils/chatter_link.py for the wire details.
"""
import logging
import os
import sys

# Make sibling src/ modules importable when launched from src/ as cwd.
_THIS = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.dirname(os.path.dirname(_THIS))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from utils.chatter_link import get_link, router  # noqa: E402

_log = logging.getLogger(__name__)

_GESTURES = ("nod", "shake", "scan", "center")


def _as_angle(v, name):
    """Coerce v to a float in [0,180]; returns (angle, error_text)."""
    try:
        a = float(v)
    except (TypeError, ValueError):
        return None, f"{name} must be a number 0-180, got {v!r}"
    if not 0 <= a <= 180:
        return None, f"{name} must be within 0-180, got {a}"
    return a, None


def react_invoke(args, *, character_name=None, backend=None, logger=None):
    """See Skill.md for the args contract. Returns {status, text}."""
    gesture = args.get("gesture")
    pan_in = args.get("pan")
    tilt_in = args.get("tilt")

    if gesture is not None:
        gesture = str(gesture).strip().lower()
        if gesture not in _GESTURES:
            return {"status": "error",
                    "text": f"unknown gesture {gesture!r}; use one of "
                            f"{', '.join(_GESTURES)}"}
        pan = tilt = None
    else:
        if pan_in is None and tilt_in is None:
            return {"status": "error",
                    "text": "head-move needs pan, tilt, and/or gesture"}
        pan = tilt = None
        if pan_in is not None:
            pan, err = _as_angle(pan_in, "pan")
            if err:
                return {"status": "error", "text": err}
        if tilt_in is not None:
            tilt, err = _as_angle(tilt_in, "tilt")
            if err:
                return {"status": "error", "text": err}

    smooth = args.get("smooth", True)

    link = get_link()
    err = link.ensure()
    if err is not None:
        return {"status": "error",
                "text": f"ChatterBot not reachable at {router()} ({err})"}

    try:
        result = link.send_head_cmd(
            pan=pan, tilt=tilt, gesture=gesture, smooth=bool(smooth))
    except Exception as e:
        _log.exception("head-move: send failed")
        return {"status": "error",
                "text": f"head command failed: {type(e).__name__}: {e}"}

    what = f"gesture {gesture}" if gesture else f"pan={pan} tilt={tilt}"
    pose = f"pan={result.get('pan')} tilt={result.get('tilt')}"
    if result.get("confirmed"):
        return {"status": "ok", "text": f"head moved ({what}); settled at {pose}."}
    # Command went out but no arrival confirmation — head_service may be slow,
    # absent, or only the router is up. Report honestly rather than claim done.
    return {"status": "ok",
            "text": (f"head command sent ({what}) but no arrival confirmation; "
                     f"last known pose {pose} (head_service may be down).")}
