"""camera-capture — grab a still from the ChatterBot head camera.

ReAct entry-point. Requests a frame over the shared ChatterLink (isolated Zenoh
session to the Pi), saves the JPEG under the canvas server's allowlisted local
root, and returns a ready-to-display URL — mirroring generate-image's output so
the `display` tool can render it. The image bytes never travel through the ReAct
text log.
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

from utils.chatter_link import get_link, jpeg_to_data_uri, router  # noqa: E402

_log = logging.getLogger(__name__)

# Captured frames land under the canvas server's allowlisted local root so the
# /local route can serve them to <img> tags. Mirrors generate-image.
_OUT_DIR = Path("~/.cache/cognitive/chatterbot").expanduser()
_CANVAS_PORT = os.environ.get("CANVAS_HTTP_PORT", "8789")


def react_invoke(args, *, character_name=None, backend=None, logger=None):
    """Takes no args (see Skill.md). Returns {status, text}."""
    link = get_link()
    err = link.ensure()
    if err is not None:
        return {"status": "error",
                "text": f"ChatterBot not reachable at {router()} ({err})"}

    try:
        frame = link.capture_image()
    except RuntimeError as e:
        # Connected but the capture didn't complete (timeout / no data).
        return {"status": "empty",
                "text": (f"connected to {router()} but no frame arrived: {e} "
                         f"(camera_service running?)")}
    except Exception as e:
        _log.exception("camera-capture: unexpected failure")
        return {"status": "error",
                "text": f"capture failed: {type(e).__name__}: {e}"}

    _OUT_DIR.mkdir(parents=True, exist_ok=True)
    out_path = _OUT_DIR / f"{uuid.uuid4().hex}.jpg"
    out_path.write_bytes(frame["jpeg"])

    # Data-URI for the model's vision input — inline base64, like /paste. The
    # /local URL below is for the canvas display only; a local model server
    # generally can't fetch a 127.0.0.1 URL, so vision goes via the data-URI.
    data_uri = jpeg_to_data_uri(frame["jpeg"])

    url = (f"http://127.0.0.1:{_CANVAS_PORT}/local?path="
           + urllib.parse.quote(str(out_path)))
    dims = f"{frame['width']}x{frame['height']}"
    pose = frame.get("head_pose")
    pose_str = f" at head pose {pose}" if pose else ""
    img = f'<img src="{url}" style="max-width:100%">'
    return {"status": "ok",
            "text": (f"captured {dims} frame{pose_str}. You can see it now; to "
                     f"also show it to the user, use the display tool, "
                     f"format=html: {img}"),
            "image": {"data_uri": data_uri, "label": "camera view"}}
