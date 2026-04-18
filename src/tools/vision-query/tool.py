"""vision-query — ask a VLM a question about the Body's camera.

Captures a fresh RGB frame from the Body over zenoh (publish to
`body/oakd/config`, wait on `body/oakd/rgb`) and runs a VLM call via
`vision_service`. Returns the VLM's text answer.

An optional `image_path` overrides capture — useful when the operator has
pushed a specific frame via body_stub's Vision dock and wants Jill to
analyse that exact image.
"""

import base64
import json
import logging
import os
import pathlib
import threading
import time
import uuid
from typing import Any, Dict, Optional, Tuple

from infospace_executor import InfospaceExecutor

import vision_service
from utils.zenoh_utils import make_localhost_config


logger = logging.getLogger(__name__)


BODY_ROUTER = (
    os.environ.get("BODY_ROUTER")
    or os.environ.get("ZENOH_CONNECT")
    or "tcp/127.0.0.1:7447"
)
OAKD_CONFIG_TOPIC = "body/oakd/config"
OAKD_RGB_TOPIC = "body/oakd/rgb"
CAPTURE_TIMEOUT_S = float(os.environ.get("VISION_CAPTURE_TIMEOUT_S", "5.0"))


# Module-level zenoh state. One session/subscriber/publisher shared across
# tool calls. A pending-request dict dispatches RGB replies back to the
# waiting caller by request_id, so concurrent vision-query calls don't
# collide.
_init_lock = threading.Lock()
_session: Optional[Any] = None
_rgb_sub: Optional[Any] = None
_config_pub: Optional[Any] = None
_pending_lock = threading.Lock()
_pending: Dict[str, Dict[str, Any]] = {}


def _on_rgb(sample: Any) -> None:
    try:
        raw = bytes(sample.payload.to_bytes())
    except AttributeError:
        raw = bytes(sample.payload)
    try:
        msg = json.loads(raw.decode("utf-8"))
    except Exception:
        return
    req_id = msg.get("request_id") or ""
    if not req_id:
        return
    with _pending_lock:
        entry = _pending.get(req_id)
        if entry is None:
            return
        entry["result"] = msg
        entry["event"].set()


def _ensure_zenoh() -> Optional[str]:
    global _session, _rgb_sub, _config_pub
    with _init_lock:
        if _session is not None:
            return None
        try:
            import zenoh
            if BODY_ROUTER.startswith("tcp/127.") or BODY_ROUTER.startswith("tcp/localhost"):
                config = make_localhost_config()
            else:
                config = zenoh.Config()
            config.insert_json5(
                "connect/endpoints", json.dumps([BODY_ROUTER]),
            )
            _session = zenoh.open(config)
            _rgb_sub = _session.declare_subscriber(OAKD_RGB_TOPIC, _on_rgb)
            _config_pub = _session.declare_publisher(OAKD_CONFIG_TOPIC)
        except Exception as e:
            logger.exception("vision-query: zenoh init failed")
            _teardown_zenoh_locked()
            return f"{type(e).__name__}: {e}"
    return None


def _teardown_zenoh_locked() -> None:
    global _session, _rgb_sub, _config_pub
    for obj in (_rgb_sub, _config_pub):
        if obj is not None:
            try:
                obj.undeclare()
            except Exception:
                pass
    _rgb_sub = None
    _config_pub = None
    if _session is not None:
        try:
            _session.close()
        except Exception:
            pass
        _session = None


def _capture_rgb() -> Tuple[Optional[bytes], Optional[str]]:
    err = _ensure_zenoh()
    if err is not None:
        return None, f"zenoh_init: {err}"
    req_id = str(uuid.uuid4())
    event = threading.Event()
    with _pending_lock:
        _pending[req_id] = {"event": event, "result": None}
    t0 = time.time()
    try:
        payload = json.dumps({
            "action": "capture_rgb", "request_id": req_id,
        }).encode("utf-8")
        _config_pub.put(payload)
    except Exception as e:
        with _pending_lock:
            _pending.pop(req_id, None)
        return None, f"publish_failed: {e}"
    got = event.wait(CAPTURE_TIMEOUT_S)
    dt_ms = int((time.time() - t0) * 1000)
    if not got:
        with _pending_lock:
            _pending.pop(req_id, None)
        logger.info("vision-query: capture TIMEOUT after %d ms", dt_ms)
        return None, f"capture_timeout_{CAPTURE_TIMEOUT_S}s"
    with _pending_lock:
        msg = _pending.pop(req_id, {}).get("result")
    if not msg:
        return None, "capture_no_result"
    if not msg.get("ok"):
        return None, f"capture_failed: {msg.get('error', 'unknown')}"
    data = msg.get("data")
    if not data:
        return None, "capture_no_data"
    try:
        jpeg = base64.b64decode(data)
    except Exception as e:
        return None, f"b64_decode: {e}"
    logger.info(
        "vision-query: captured %dx%d %d KB in %d ms",
        msg.get("width", 0), msg.get("height", 0), len(jpeg) // 1024, dt_ms,
    )
    return jpeg, None


def _fail(executor: InfospaceExecutor, reason: str,
          extra: Optional[Dict[str, Any]] = None):
    return executor._create_uniform_return(
        "failed", value=reason, reason=reason, extra=extra,
    )


def _success(executor: InfospaceExecutor, value: str,
             extra: Optional[Dict[str, Any]] = None):
    return executor._create_uniform_return(
        "success", value=value, extra=extra,
    )


def tool(input_value, **kwargs):
    executor: InfospaceExecutor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available",
                "value": None, "resource_id": None}

    question = kwargs.get("question") or ""
    if not question or not isinstance(question, str):
        # Fall back to input_value if caller passed the question positionally
        if isinstance(input_value, str) and input_value:
            question = input_value
        else:
            return _fail(executor, "question required")

    image_path = kwargs.get("image_path")
    logger.info(
        "vision-query: q=%r source=%s",
        question[:120] + ("…" if len(question) > 120 else ""),
        f"path:{image_path}" if image_path else "capture",
    )

    if image_path:
        path = pathlib.Path(image_path)
        if not path.is_file():
            return _fail(executor, f"image_not_found: {image_path}")
        try:
            jpeg = path.read_bytes()
        except OSError as e:
            return _fail(executor, f"image_read_failed: {e}")
        captured_path = image_path
    else:
        jpeg, err = _capture_rgb()
        if jpeg is None:
            return _fail(executor, err or "capture_failed")
        try:
            captured_path = vision_service.cache_jpeg(jpeg)
        except Exception:
            captured_path = None

    t0 = time.time()
    try:
        answer = vision_service.chat(
            [{"role": "user", "content": question}],
            images=[jpeg],
            temperature=0.2,
        )
    except Exception as e:
        logger.exception("vision-query: VLM call failed")
        return _fail(executor, f"vlm_error: {type(e).__name__}: {e}")
    vlm_ms = int((time.time() - t0) * 1000)
    logger.info(
        "vision-query: vlm %d ms, %d chars — %s",
        vlm_ms, len(answer),
        answer[:160].replace("\n", " ") + ("…" if len(answer) > 160 else ""),
    )

    return _success(
        executor, answer,
        {"question": question, "image_path": captured_path,
         "chars": len(answer)},
    )
