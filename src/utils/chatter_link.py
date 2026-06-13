"""ChatterLink — long-lived, isolated Zenoh session to the ChatterBot Pi.

Shared by the `head-move` and `camera-capture` drop-in tools (src/tools/).
Both need the same session to the Pi's `chatter/...` bus, so it lives here as
one canonical link rather than being re-opened per tool.

Design (mirrors the retired body.py BodyLink, repointed at ChatterBot):
- A separate session from the chat loop's localhost mesh. We connect straight
  to the Pi router (CHATTER_ROUTER), multicast off, gossip on — the chat loop's
  make_localhost_config() is loopback-only and must NOT be federated with the
  Pi (see docs/jill-integration.md §3).
- Singleton, lazily opened on first use, reused across tool calls. Thread-safe:
  Zenoh subscriber callbacks run on Zenoh's pool, tool calls on the chat thread.

Wire contract (chatterbot/lib/topics.py + ChatterBot DESIGN.md §5):
- head/cmd   (Jill->Pi) : {ts, pan?, tilt?, gesture?: nod|shake|scan|center, smooth?}
- head/status(Pi->Jill) : {ts, pan, tilt, state: idle|moving|arrived, mode} ~5Hz
- camera/capture (Jill->Pi): {ts, request_id}
- camera/image  (Pi->Jill) : {ts, request_id, format:"jpeg_base64", data_base64,
                              width, height, head_pose:{pan,tilt}, settled}
camera/image is broadcast, so capture filters by request_id.
"""

from __future__ import annotations

import base64
import json
import logging
import os
import threading
import time
import uuid
from typing import Any, Dict, Optional

logger = logging.getLogger(__name__)

# Pi router. Default is the ChatterBot Pi's on-LAN zenohd address; override
# with CHATTER_ROUTER (or the generic ZENOH_CONNECT) at launch.
_CHATTER_ROUTER = (
    os.environ.get("CHATTER_ROUTER")
    or os.environ.get("ZENOH_CONNECT")
    or "tcp/192.168.68.78:7447"
)

# Safe head envelope + neutral/attentive pose. Source of truth: ChatterBot
# docs/gaze-support.md §1 (measured for this mount). Mirrored by every agent-side
# consumer (look-at-target, head_aliveness) so the geometry lives in one place.
PAN_MIN, PAN_MAX = 10, 170          # 0=cam right, 170=cam left
TILT_MIN, TILT_MAX = 30, 150        # 30=up, 115=horizontal, 150=~45° down
NEUTRAL_PAN, NEUTRAL_TILT = 90, 113  # forward, horizontal ("attentive")


def clamp_pose(pan, tilt):
    """Clamp a (pan, tilt) pair into the safe envelope."""
    return (max(PAN_MIN, min(PAN_MAX, pan)),
            max(TILT_MIN, min(TILT_MAX, tilt)))


# Topics.
_T_HEAD_CMD = "chatter/head/cmd"
_T_HEAD_STATUS = "chatter/head/status"
_T_CAMERA_CAPTURE = "chatter/camera/capture"
_T_CAMERA_IMAGE = "chatter/camera/image"

# How long to wait for the camera/image reply before giving up.
_CAPTURE_TIMEOUT_S = float(os.environ.get("CHATTER_CAPTURE_TIMEOUT_S", "8.0"))
# How long send_head_cmd waits for an "arrived" head/status confirmation.
_MOVE_TIMEOUT_S = float(os.environ.get("CHATTER_MOVE_TIMEOUT_S", "6.0"))


def _normalize_endpoint(ep: str) -> str:
    """Accept either `tcp/host:port` or a bare `host:port` and return the
    Zenoh `tcp/host:port` form."""
    ep = (ep or "").strip()
    if "/" not in ep:
        return f"tcp/{ep}"
    return ep


def _payload_bytes(sample: Any) -> bytes:
    """Extract raw bytes from a Zenoh sample across API shapes."""
    payload = getattr(sample, "payload", sample)
    to_bytes = getattr(payload, "to_bytes", None)
    if callable(to_bytes):
        return bytes(to_bytes())
    return bytes(payload)


def jpeg_to_data_uri(jpeg: bytes, max_wh: Optional[tuple] = None) -> str:
    """Build a `data:image/jpeg;base64,...` URI from JPEG bytes for use as LLM
    vision input. If max_wh=(w,h) is given, downscale first (preserving aspect)
    to cut size/latency — used for the gaze loop's assessment frames. Downscale
    failure falls back to the full frame (logged, never silent)."""
    if max_wh is not None:
        try:
            import io
            from PIL import Image
            im = Image.open(io.BytesIO(jpeg))
            im.thumbnail(max_wh)
            buf = io.BytesIO()
            im.convert("RGB").save(buf, format="JPEG", quality=85)
            jpeg = buf.getvalue()
        except Exception as e:
            logger.warning(f"chatter: downscale failed, using full frame: {e}")
    return "data:image/jpeg;base64," + base64.b64encode(jpeg).decode("ascii")


class ChatterLink:
    """Owns the Zenoh session to the ChatterBot Pi: head/camera publishers,
    a latest-value cache for head/status, and the camera/image request/reply
    plumbing. One instance per process, reused across tool invocations."""

    def __init__(self) -> None:
        self._init_lock = threading.Lock()
        self._session: Optional[Any] = None
        self._head_pub: Optional[Any] = None
        self._capture_pub: Optional[Any] = None
        self._image_sub: Optional[Any] = None
        self._status_sub: Optional[Any] = None

        # camera/image: request_id -> {"event": Event, "result": dict|None}
        self._pending_lock = threading.Lock()
        self._pending: Dict[str, Dict[str, Any]] = {}

        # latest head/status payload + arrival monotonic time
        self._status_lock = threading.Lock()
        self._status: Optional[Dict[str, Any]] = None
        self._status_ts: float = 0.0
        # Set by the status callback whenever an "arrived" status lands, so
        # send_head_cmd can block until a move completes.
        self._arrived_event: Optional[threading.Event] = None

    # -- connection -------------------------------------------------------

    def ensure(self) -> Optional[str]:
        """Open the session and declare pub/subs once. Returns None on
        success, or an error string the caller surfaces as ERROR."""
        with self._init_lock:
            if self._session is not None:
                return None
            try:
                import zenoh
                config = zenoh.Config()
                ep = _normalize_endpoint(_CHATTER_ROUTER)
                config.insert_json5("connect/endpoints", json.dumps([ep]))
                config.insert_json5("scouting/multicast/enabled", "false")
                config.insert_json5("scouting/gossip/enabled", "true")
                self._session = zenoh.open(config)
                self._head_pub = self._session.declare_publisher(_T_HEAD_CMD)
                self._capture_pub = self._session.declare_publisher(
                    _T_CAMERA_CAPTURE)
                self._image_sub = self._session.declare_subscriber(
                    _T_CAMERA_IMAGE, self._on_image)
                self._status_sub = self._session.declare_subscriber(
                    _T_HEAD_STATUS, self._on_status)
                logger.info(f"chatter: zenoh connected to {ep}")
            except Exception as e:
                logger.exception("chatter: zenoh init failed")
                self._teardown_locked()
                return f"{type(e).__name__}: {e}"
        return None

    def _teardown_locked(self) -> None:
        """Best-effort cleanup after a failed open. Each step logs on failure
        rather than silently swallowing."""
        for obj, name in ((self._status_sub, "status_sub"),
                          (self._image_sub, "image_sub"),
                          (self._capture_pub, "capture_pub"),
                          (self._head_pub, "head_pub")):
            if obj is not None:
                try:
                    obj.undeclare()
                except Exception as e:
                    logger.warning(f"chatter: {name} undeclare failed: {e}")
        self._status_sub = None
        self._image_sub = None
        self._capture_pub = None
        self._head_pub = None
        if self._session is not None:
            try:
                self._session.close()
            except Exception as e:
                logger.warning(f"chatter: session close failed: {e}")
            self._session = None

    # -- subscriber callbacks --------------------------------------------

    def _on_image(self, sample: Any) -> None:
        try:
            msg = json.loads(_payload_bytes(sample).decode("utf-8"))
        except Exception as e:
            logger.warning(f"chatter: undecodable camera/image dropped: {e}")
            return
        req_id = msg.get("request_id")
        with self._pending_lock:
            entry = self._pending.get(req_id)
            if entry is not None:
                entry["result"] = msg
                entry["event"].set()

    def _on_status(self, sample: Any) -> None:
        try:
            msg = json.loads(_payload_bytes(sample).decode("utf-8"))
        except Exception as e:
            logger.warning(f"chatter: undecodable head/status dropped: {e}")
            return
        with self._status_lock:
            self._status = msg
            self._status_ts = time.monotonic()
            if msg.get("state") == "arrived" and self._arrived_event is not None:
                self._arrived_event.set()

    # -- primitives -------------------------------------------------------

    def send_head_cmd(self, *, pan: Optional[float] = None,
                      tilt: Optional[float] = None,
                      gesture: Optional[str] = None,
                      smooth: bool = True,
                      timeout: float = _MOVE_TIMEOUT_S) -> Dict[str, Any]:
        """Publish a head command and wait for the Pi to report arrival.

        Returns {"confirmed": bool, "pan", "tilt", "state"} — confirmed is
        True if an "arrived" status landed within timeout; pan/tilt/state come
        from the latest head/status (may be None if none has arrived)."""
        cmd: Dict[str, Any] = {"ts": time.time()}
        if gesture is not None:
            # Pi gives gesture precedence over pan/tilt; send only the gesture.
            cmd["gesture"] = gesture
        else:
            if pan is not None:
                cmd["pan"] = pan
            if tilt is not None:
                cmd["tilt"] = tilt
            cmd["smooth"] = smooth

        event = threading.Event()
        with self._status_lock:
            self._arrived_event = event
        try:
            self._head_pub.put(json.dumps(cmd).encode("utf-8"))
            confirmed = event.wait(timeout)
        finally:
            with self._status_lock:
                self._arrived_event = None
                status = self._status or {}
        return {
            "confirmed": confirmed,
            "pan": status.get("pan"),
            "tilt": status.get("tilt"),
            "state": status.get("state"),
        }

    def capture_image(self, timeout: float = _CAPTURE_TIMEOUT_S) -> Dict[str, Any]:
        """Request a frame and return the decoded reply. Raises RuntimeError
        with a terse reason on timeout or a malformed reply.

        Returns {"jpeg": bytes, "width", "height", "head_pose", "settled"}."""
        req_id = str(uuid.uuid4())
        event = threading.Event()
        with self._pending_lock:
            self._pending[req_id] = {"event": event, "result": None}
        try:
            payload = json.dumps(
                {"ts": time.time(), "request_id": req_id}).encode("utf-8")
            self._capture_pub.put(payload)
            if not event.wait(timeout):
                raise RuntimeError(f"capture_timeout_{timeout}s")
            msg = self._pending.get(req_id, {}).get("result") or {}
        finally:
            with self._pending_lock:
                self._pending.pop(req_id, None)

        data = msg.get("data_base64")
        if not data:
            raise RuntimeError("capture_failed: reply had no image data")
        try:
            jpeg = base64.b64decode(data)
        except Exception as e:
            raise RuntimeError(f"capture_failed: bad base64 ({e})")
        return {
            "jpeg": jpeg,
            "width": msg.get("width", "?"),
            "height": msg.get("height", "?"),
            "head_pose": msg.get("head_pose"),
            "settled": msg.get("settled"),
        }

    def latest_head_status(self) -> Optional[Dict[str, Any]]:
        """Latest cached head/status with an age_s field, or None if none yet."""
        with self._status_lock:
            if self._status is None:
                return None
            age = round(time.monotonic() - self._status_ts, 2)
            return {"age_s": age, **self._status}


_link: Optional[ChatterLink] = None
_link_lock = threading.Lock()


def get_link() -> ChatterLink:
    """Process-wide ChatterLink singleton (lazy)."""
    global _link
    with _link_lock:
        if _link is None:
            _link = ChatterLink()
        return _link


def router() -> str:
    """The configured Pi endpoint, for error messages."""
    return _CHATTER_ROUTER
