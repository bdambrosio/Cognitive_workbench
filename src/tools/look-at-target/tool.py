"""look-at-target — closed-loop gaze (Tier 2).

ReAct entry-point. Given a natural-language target, drives the ChatterBot head
(over the shared ChatterLink) until the target is centered in the camera, using
the character's multimodal backend to judge the target's position each cycle.

Control law (see ChatterBot docs/gaze-support.md §1 for the geometry):
- Per cycle, ask the vision model for the target's position relative to frame
  center as coarse buckets (left/right + up/down, a little / a lot). Coarse
  buckets are what a general VLM does reliably; precise pixel/degree regression
  is not.
- Map each bucket to a bounded pan/tilt delta and apply it relative to the
  current pose, clamped to the safe envelope. Signs (hflip/vflip false):
  target left -> pan + (toward 170); target up -> tilt - (toward 30).
- Bounded acquire sweep if the target isn't already visible; bounded centering
  loop with a halve-on-flip anti-oscillation guard. Centers once; not a tracker.

Assessment frames are downscaled (the loop doesn't need full res); the final
returned frame is full res for display.
"""
import logging
import os
import sys
import urllib.parse
import uuid
from pathlib import Path

# Make sibling src/ modules importable when launched from src/ as cwd.
_THIS = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.dirname(os.path.dirname(_THIS))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from utils.chatter_link import (  # noqa: E402
    NEUTRAL_PAN, NEUTRAL_TILT, PAN_MAX, PAN_MIN, TILT_MAX, TILT_MIN,
    get_link, jpeg_to_data_uri, router)
from utils.json_utils import repair_json_string  # noqa: E402

_log = logging.getLogger(__name__)

_OUT_DIR = Path("~/.cache/cognitive/chatterbot").expanduser()
_CANVAS_PORT = os.environ.get("CANVAS_HTTP_PORT", "8789")

# Safe head envelope + neutral pose (ChatterBot docs/gaze-support.md §1).
_PAN_MIN, _PAN_MAX = PAN_MIN, PAN_MAX
_TILT_MIN, _TILT_MAX = TILT_MIN, TILT_MAX
_HOME_PAN, _HOME_TILT = NEUTRAL_PAN, NEUTRAL_TILT

# Bounded per-cycle deltas (degrees) by bucket. left -> pan+, up -> tilt-.
_H_STEP = {"left_a_lot": +18, "left_a_little": +7, "centered": 0,
           "right_a_little": -7, "right_a_lot": -18}
_V_STEP = {"up_a_lot": -18, "up_a_little": -7, "centered": 0,
           "down_a_little": +7, "down_a_lot": +18}

# Bounded acquire sweep — all poses inside the safe envelope, near horizontal
# where people/animals usually are. Capped count by construction.
_SEARCH_WAYPOINTS = [(p, t) for t in (113, 95) for p in (50, 90, 130)]
_MAX_CENTER_ITERS = 5
_ASSESS_WH = (640, 360)                   # downscale for assessment frames

_ASSESS_SCHEMA = {
    "type": "object",
    "properties": {
        "visible": {"type": "boolean"},
        "h_pos": {"type": "string", "enum": list(_H_STEP.keys())},
        "v_pos": {"type": "string", "enum": list(_V_STEP.keys())},
    },
    "required": ["visible", "h_pos", "v_pos"],
    "additionalProperties": True,
}

_ASSESS_SYS = (
    "You position a camera to center a target. You are shown the current camera "
    "view. Report whether the target is visible in THIS image, and if so where "
    "it sits relative to the CENTER of the frame: horizontally "
    "(left_a_lot/left_a_little/centered/right_a_little/right_a_lot) and "
    "vertically (up_a_lot/up_a_little/centered/down_a_little/down_a_lot). "
    "'left' means the target is toward the left edge; 'up' means toward the top. "
    "Judge only what you see; if the target is not present, set visible=false. "
    "Output only the structured fields."
)


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _assess(backend, jpeg, target):
    """One vision judgement. Returns {visible,h_pos,v_pos} or None on failure."""
    data_uri = jpeg_to_data_uri(jpeg, max_wh=_ASSESS_WH)
    messages = [
        {"role": "system", "content": _ASSESS_SYS},
        {"role": "user", "content": [
            {"type": "text", "text": f"Target to center: {target}"},
            {"type": "image_url", "image_url": {"url": data_uri}},
        ]},
    ]
    try:
        raw = backend.chat(messages, max_tokens=256, temperature=0.2,
                           response_schema=_ASSESS_SCHEMA)
    except Exception as e:
        _log.warning(f"look-at-target: assess LLM call failed: {e}")
        return None
    obj = repair_json_string(raw or "")
    if not isinstance(obj, dict) or "h_pos" not in obj or "v_pos" not in obj:
        _log.warning(f"look-at-target: unparseable assessment: {raw[:160]!r}")
        return None
    return obj


def _centered(a):
    return a["h_pos"] == "centered" and a["v_pos"] == "centered"


def react_invoke(args, *, character_name=None, backend=None, logger=None):
    """See Skill.md for the args contract. Returns {status, text, image?}."""
    target = args.get("target")
    if not isinstance(target, str) or not target.strip():
        return {"status": "error", "text": "look-at-target needs a `target`"}
    target = target.strip()

    if backend is None or not getattr(backend, "supports_image_input", False):
        return {"status": "error",
                "text": ("look-at-target needs a vision-capable backend "
                         "(the active model can't take image input)")}

    link = get_link()
    err = link.ensure()
    if err is not None:
        return {"status": "error",
                "text": f"ChatterBot not reachable at {router()} ({err})"}

    def _capture():
        return link.capture_image()          # raises RuntimeError on failure

    # --- acquire ---------------------------------------------------------
    try:
        frame = _capture()
    except Exception as e:
        return {"status": "empty",
                "text": f"could not capture from the camera: {e}"}
    assess = _assess(backend, frame["jpeg"], target)

    searched = 0
    while (assess is None or not assess.get("visible")) and \
            searched < len(_SEARCH_WAYPOINTS):
        p, t = _SEARCH_WAYPOINTS[searched]
        searched += 1
        link.send_head_cmd(pan=p, tilt=t, smooth=True)
        try:
            frame = _capture()
        except Exception as e:
            _log.warning(f"look-at-target: capture during search failed: {e}")
            continue
        assess = _assess(backend, frame["jpeg"], target)

    if assess is None or not assess.get("visible"):
        return {"status": "empty",
                "text": (f"couldn't find {target} after searching "
                         f"{searched} view(s).")}

    # --- center ----------------------------------------------------------
    pan_scale = tilt_scale = 1.0
    last_h = last_v = 0
    iters = 0
    for iters in range(1, _MAX_CENTER_ITERS + 1):
        if _centered(assess):
            iters -= 1            # already centered; no move this iter
            break
        dpan = _H_STEP[assess["h_pos"]]
        dtilt = _V_STEP[assess["v_pos"]]
        # Anti-oscillation: if an axis reversed direction (overshoot), damp it.
        if dpan and last_h and (dpan > 0) != (last_h > 0):
            pan_scale = max(0.25, pan_scale * 0.5)
        if dtilt and last_v and (dtilt > 0) != (last_v > 0):
            tilt_scale = max(0.25, tilt_scale * 0.5)
        last_h, last_v = dpan or last_h, dtilt or last_v

        pose = link.latest_head_status() or {}
        cur_pan = pose.get("pan", _HOME_PAN)
        cur_tilt = pose.get("tilt", _HOME_TILT)
        new_pan = _clamp(round(cur_pan + dpan * pan_scale), _PAN_MIN, _PAN_MAX)
        new_tilt = _clamp(round(cur_tilt + dtilt * tilt_scale), _TILT_MIN, _TILT_MAX)
        link.send_head_cmd(pan=new_pan, tilt=new_tilt, smooth=True)
        try:
            frame = _capture()
        except Exception as e:
            _log.warning(f"look-at-target: capture during centering failed: {e}")
            break
        a = _assess(backend, frame["jpeg"], target)
        if a is None:
            break
        assess = a

    # --- result ----------------------------------------------------------
    pose = link.latest_head_status() or {}
    pan, tilt = pose.get("pan"), pose.get("tilt")
    centered = _centered(assess)

    # Save the final frame for canvas display, and attach it to the model's view.
    _OUT_DIR.mkdir(parents=True, exist_ok=True)
    out_path = _OUT_DIR / f"{uuid.uuid4().hex}.jpg"
    out_path.write_bytes(frame["jpeg"])
    url = (f"http://127.0.0.1:{_CANVAS_PORT}/local?path="
           + urllib.parse.quote(str(out_path)))
    img_tag = f'<img src="{url}" style="max-width:100%">'
    data_uri = jpeg_to_data_uri(frame["jpeg"])

    if centered:
        head = f"centered on {target} at pan={pan} tilt={tilt} (in {iters} adjustment(s))."
    else:
        head = (f"could not fully center {target} after {iters} adjustment(s); "
                f"best pose pan={pan} tilt={tilt}.")
    return {"status": "ok",
            "text": (f"{head} You can see the final frame; to also show it to the "
                     f"user, use the display tool, format=html: {img_tag}"),
            "image": {"data_uri": data_uri, "label": "camera view"}}
