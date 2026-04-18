"""Vision service — friendly client for a local OpenAI-compatible VLM server.

Calls vLLM (or any OpenAI-compatible server) at VISION_BASE_URL. Model choice
is decoupled: any VL model exposing chat.completions with image content parts
should work. Per-model tuning, if needed, lives in _DETECT_TEMPLATE.
"""

import base64
import hashlib
import logging
import os
import pathlib
import threading
import time
from dataclasses import dataclass, field
from typing import List, Optional

import requests

from utils.json_utils import repair_json_string


logger = logging.getLogger(__name__)


VISION_BASE_URL = os.environ.get("VISION_BASE_URL", "http://localhost:5005/v1")
# If VISION_MODEL is set, it's an explicit override — bypasses auto-detection.
# If unset, we query /v1/models on first use and cache whatever's served.
VISION_MODEL_OVERRIDE = os.environ.get("VISION_MODEL")
VISION_TIMEOUT_S = float(os.environ.get("VISION_TIMEOUT_S", "60"))

_model_lock = threading.Lock()
_resolved_model: Optional[str] = None


def _resolve_model() -> str:
    """Return the model id to send with each chat request.

    Precedence: explicit VISION_MODEL env override > auto-detect from
    /v1/models (cached) > literal "default" as a last-resort fallback that
    will make the failure mode explicit in the server response.
    """
    global _resolved_model
    if VISION_MODEL_OVERRIDE:
        return VISION_MODEL_OVERRIDE
    with _model_lock:
        if _resolved_model is not None:
            return _resolved_model
        try:
            resp = requests.get(f"{VISION_BASE_URL}/models", timeout=5.0)
            resp.raise_for_status()
            data = resp.json().get("data") or []
            if data:
                _resolved_model = data[0].get("id") or "default"
                logger.info("vision_service: auto-detected model %r", _resolved_model)
                return _resolved_model
        except Exception as e:
            logger.warning("vision_service: /v1/models probe failed: %s", e)
    return "default"
VISION_CACHE_DIR = pathlib.Path(
    os.environ.get("VISION_CACHE_DIR", "/tmp/body_vision")
)


def cache_jpeg(jpeg_bytes: bytes) -> str:
    """Write a JPEG into the shared cache dir, return its absolute path.

    Used when body_stub routes chat through Jill: the image is written to
    disk so Jill's vision-query tool can read it by path (no payload push
    through zenoh). Filename is ms-timestamp + short content hash.
    """
    VISION_CACHE_DIR.mkdir(parents=True, exist_ok=True)
    digest = hashlib.sha1(jpeg_bytes).hexdigest()[:8]
    fname = f"{int(time.time() * 1000)}-{digest}.jpg"
    path = VISION_CACHE_DIR / fname
    path.write_bytes(jpeg_bytes)
    return str(path)

# Canonical detect prompt. Per-model tuning lives here — swap the wording
# if a new VLM responds better to a different grounding format.
_DETECT_TEMPLATE = (
    "Analyze the image and locate distinct objects. Return ONLY a JSON "
    "object inside a ```json fenced block with a single field `boxes`, "
    "a list of entries each having:\n"
    "  - label: short object name (string)\n"
    "  - bbox: [x1, y1, x2, y2] integer pixel coordinates\n"
    "  - confidence: float between 0 and 1\n"
    "{constraint}"
    "Do not output any text outside the fenced block."
)


@dataclass
class Box:
    label: str
    bbox: tuple  # (x1, y1, x2, y2)
    confidence: Optional[float] = None


@dataclass
class DetectResult:
    text: str
    boxes: List[Box] = field(default_factory=list)


@dataclass
class Face:
    name: str
    bbox: tuple
    confidence: Optional[float] = None


def _jpeg_to_data_url(jpeg_bytes: bytes) -> str:
    b64 = base64.b64encode(jpeg_bytes).decode("ascii")
    return f"data:image/jpeg;base64,{b64}"


def _attach_images(messages, images):
    """Return a copy of messages with images appended to the last user turn."""
    if not images:
        return messages
    out = [dict(m) for m in messages]
    for i in range(len(out) - 1, -1, -1):
        if out[i].get("role") == "user":
            parts = [{"type": "text", "text": out[i].get("content", "")}]
            for img in images:
                parts.append({
                    "type": "image_url",
                    "image_url": {"url": _jpeg_to_data_url(img)},
                })
            out[i] = {**out[i], "content": parts}
            return out
    raise ValueError("no user message to attach images to")


def _post_chat(messages, model=None, temperature=0.2, max_tokens=1024) -> str:
    payload = {
        "model": model or _resolve_model(),
        "messages": messages,
        "temperature": temperature,
        "max_tokens": max_tokens,
    }
    resp = requests.post(
        f"{VISION_BASE_URL}/chat/completions",
        json=payload,
        timeout=VISION_TIMEOUT_S,
    )
    resp.raise_for_status()
    return resp.json()["choices"][0]["message"]["content"]


def chat(messages, images=None, model=None, temperature=0.2, max_tokens=1024) -> str:
    """Call the VLM with a chat transcript and optional JPEG images.

    messages: list of {"role", "content"} dicts (content is str).
    images: optional list of JPEG byte strings, attached to the last user turn.
    Returns assistant text.
    """
    return _post_chat(
        _attach_images(messages, images),
        model=model, temperature=temperature, max_tokens=max_tokens,
    )


def detect(jpeg_bytes: bytes, labels=None, model=None) -> DetectResult:
    """Run open-vocab (or label-restricted) object detection on a JPEG.

    Returns DetectResult with raw text and parsed boxes. boxes is empty
    when the response doesn't contain a recognisable JSON block.
    """
    constraint = f"Restrict detection to: {', '.join(labels)}.\n" if labels else ""
    prompt = _DETECT_TEMPLATE.format(constraint=constraint)
    text = chat(
        [{"role": "user", "content": prompt}],
        images=[jpeg_bytes],
        model=model,
        temperature=0.1,
    )
    return DetectResult(text=text, boxes=_parse_boxes(text))


def _parse_boxes(text: str) -> List[Box]:
    parsed = repair_json_string(text)
    if not isinstance(parsed, dict):
        return []
    raw = parsed.get("boxes")
    if not isinstance(raw, list):
        return []
    out = []
    for entry in raw:
        if not isinstance(entry, dict):
            continue
        label = entry.get("label")
        bbox = entry.get("bbox")
        if not isinstance(label, str):
            continue
        if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
            continue
        try:
            xyxy = tuple(int(v) for v in bbox)
        except (TypeError, ValueError):
            continue
        conf = entry.get("confidence")
        if not isinstance(conf, (int, float)):
            conf = None
        out.append(Box(label=label, bbox=xyxy, confidence=conf))
    return out


def recognize(jpeg_bytes: bytes) -> List[Face]:
    """Stub: person recognition against an enrolled library.

    Returns empty list until the face-recognition pipeline is wired up.
    The signature is final — callers can integrate now and get real
    results once enrollment + matching land.
    """
    return []
