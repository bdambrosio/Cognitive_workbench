"""Body-sensing subagent — a thin, persona-less ReAct loop that observes
the Body robot over Zenoh via a small set of typed, read-only primitives.
Mirrors the security.py / inspect.py pattern: typed primitives, geofenced
to the Body's Zenoh endpoints, read-only operations, OK/EMPTY/ERROR
observations.

Used by Jill's chat ReAct loop via the `body` tool. From Jill's vantage
a `body` call is one step: she emits a query string, gets a synthesized
answer back. The subagent's internal multi-iter reasoning is opaque to
her trace but written to body_traces/ for debugging.

Architectural notes:
- Backend is whatever the caller passes — by default this is the main
  character's backend (self.backend in chat_loop), so per-scenario YAML
  decides the model. No per-subagent backend overrides.
- Primitives are intentionally minimal and READ-ONLY:
    capture_rgb  — request a fresh OAK-D RGB frame, save it to disk, and
                   return a canvas-displayable /local URL.
    status       — snapshot the latest driving/pose/health telemetry.
    respond      — final synthesized answer, exits the loop.
- v1 is sensing only. It does NOT publish cmd_vel / cmd_direct / heartbeat
  — a sensing client must not announce itself as a live motion controller
  to the Body watchdog. Actuation is deliberately out of scope.
- The Body lives on a different Zenoh router than the localhost cognitive
  bus, so this module owns its own long-lived session (BodyLink), distinct
  from chat_loop's localhost session. The session and its standing
  subscribers are created lazily on first use and reused across calls.

Body wire contract (all JSON over UTF-8; pub/sub only, no queryables):
- RGB capture: PUT body/oakd/config {"action":"capture_rgb","request_id":<uuid>}
  then await a body/oakd/rgb message with the matching request_id:
  {"ok":true,"format":"jpeg","encoding":"base64","data":<b64>,"width","height"}.
- Status: standing subscriptions to body/status, body/odom,
  body/motor_state, body/emergency_stop, each carrying a JSON dict.
"""

from __future__ import annotations

import base64
import json
import logging
import os
import threading
import time
import urllib.parse
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

from utils.json_utils import repair_json_string

logger = logging.getLogger(__name__)

_MAX_ITERS = 8

# Zenoh router for the Body. Default is the robot's on-LAN address; override
# with BODY_ROUTER (or the more generic ZENOH_CONNECT) at launch.
_BODY_ROUTER = (
    os.environ.get("BODY_ROUTER")
    or os.environ.get("ZENOH_CONNECT")
    or "tcp/192.168.8.60:7447"
)

# RGB capture: how long to wait for the body/oakd/rgb reply before giving up.
_CAPTURE_TIMEOUT_S = float(os.environ.get("BODY_CAPTURE_TIMEOUT_S", "5.0"))
# How long the first status snapshot waits for body/status to arrive after a
# fresh connect (it publishes at ~1 Hz, so a cold call can race the first msg).
_STATUS_WARMUP_S = 1.5

# Topics (match Body desktop.nav defaults).
_T_OAKD_CONFIG = "body/oakd/config"
_T_OAKD_RGB = "body/oakd/rgb"
_STATUS_TOPICS = ("body/status", "body/odom", "body/motor_state",
                  "body/emergency_stop")

# Captured frames land under the canvas server's allowlisted local root so
# the /local route can serve them to <img> tags. Mirrors generate-image.
_OUT_DIR = Path("~/.cache/cognitive/body").expanduser()
_CANVAS_PORT = os.environ.get("CANVAS_HTTP_PORT", "8789")


# ---------------------------------------------------------------------------
# BodyLink — long-lived Zenoh session + standing subscribers (singleton)
# ---------------------------------------------------------------------------

class BodyLink:
    """Owns the Zenoh session to the Body, the RGB request/reply plumbing,
    and a latest-value cache for the status topics. One instance per process,
    reused across subagent invocations. Thread-safe: Zenoh subscriber
    callbacks run on Zenoh's thread pool, primitives run on the chat thread."""

    def __init__(self) -> None:
        self._init_lock = threading.Lock()
        self._session: Optional[Any] = None
        self._config_pub: Optional[Any] = None
        self._rgb_sub: Optional[Any] = None
        self._status_subs: List[Any] = []

        # request_id -> {"event": Event, "result": dict|None}
        self._pending_lock = threading.Lock()
        self._pending: Dict[str, Dict[str, Any]] = {}

        # topic -> (payload_dict, arrival_monotonic)
        self._cache_lock = threading.Lock()
        self._cache: Dict[str, Any] = {}

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
                ep = _normalize_endpoint(_BODY_ROUTER)
                config.insert_json5("connect/endpoints", json.dumps([ep]))
                config.insert_json5("scouting/multicast/enabled", "false")
                config.insert_json5("scouting/gossip/enabled", "true")
                self._session = zenoh.open(config)
                self._config_pub = self._session.declare_publisher(_T_OAKD_CONFIG)
                self._rgb_sub = self._session.declare_subscriber(
                    _T_OAKD_RGB, self._on_rgb)
                for topic in _STATUS_TOPICS:
                    self._status_subs.append(
                        self._session.declare_subscriber(
                            topic, self._make_status_cb(topic)))
                logger.info(f"body: zenoh connected to {ep}")
            except Exception as e:
                logger.exception("body: zenoh init failed")
                self._teardown_locked()
                return f"{type(e).__name__}: {e}"
        return None

    def _teardown_locked(self) -> None:
        """Best-effort cleanup after a failed open. Each undeclare/close is
        logged on failure rather than silently swallowed."""
        for sub in self._status_subs:
            try:
                sub.undeclare()
            except Exception as e:
                logger.warning(f"body: status sub undeclare failed: {e}")
        self._status_subs = []
        for obj, name in ((self._rgb_sub, "rgb_sub"),
                          (self._config_pub, "config_pub")):
            if obj is not None:
                try:
                    obj.undeclare()
                except Exception as e:
                    logger.warning(f"body: {name} undeclare failed: {e}")
        self._rgb_sub = None
        self._config_pub = None
        if self._session is not None:
            try:
                self._session.close()
            except Exception as e:
                logger.warning(f"body: session close failed: {e}")
            self._session = None

    # -- subscriber callbacks --------------------------------------------

    def _on_rgb(self, sample: Any) -> None:
        try:
            msg = json.loads(_payload_bytes(sample).decode("utf-8"))
        except Exception as e:
            logger.warning(f"body: undecodable RGB reply dropped: {e}")
            return
        req_id = msg.get("request_id")
        with self._pending_lock:
            entry = self._pending.get(req_id)
            if entry is not None:
                entry["result"] = msg
                entry["event"].set()

    def _make_status_cb(self, topic: str):
        def _cb(sample: Any) -> None:
            try:
                msg = json.loads(_payload_bytes(sample).decode("utf-8"))
            except Exception as e:
                logger.warning(f"body: undecodable {topic} msg dropped: {e}")
                return
            with self._cache_lock:
                self._cache[topic] = (msg, time.monotonic())
        return _cb

    # -- primitives backends ---------------------------------------------

    def capture_rgb(self) -> Optional[str]:
        """Request a frame, write it to disk, return the saved Path string.
        Raises RuntimeError with a terse reason on any failure."""
        req_id = str(uuid.uuid4())
        event = threading.Event()
        with self._pending_lock:
            self._pending[req_id] = {"event": event, "result": None}
        try:
            payload = json.dumps(
                {"action": "capture_rgb", "request_id": req_id}).encode("utf-8")
            self._config_pub.put(payload)
            if not event.wait(_CAPTURE_TIMEOUT_S):
                raise RuntimeError(f"capture_timeout_{_CAPTURE_TIMEOUT_S}s")
            msg = self._pending.get(req_id, {}).get("result") or {}
        finally:
            with self._pending_lock:
                self._pending.pop(req_id, None)

        if not msg.get("ok"):
            raise RuntimeError(f"capture_failed: {msg.get('error', 'unknown')}")
        data = msg.get("data")
        if not data:
            raise RuntimeError("capture_failed: reply had no image data")
        jpeg = base64.b64decode(data)
        _OUT_DIR.mkdir(parents=True, exist_ok=True)
        out_path = _OUT_DIR / f"{uuid.uuid4().hex}.jpg"
        out_path.write_bytes(jpeg)
        w, h = msg.get("width", "?"), msg.get("height", "?")
        return f"{out_path}\t{w}x{h}"

    def status_snapshot(self) -> Dict[str, Any]:
        """Return the latest cached sample for each status topic, tagged with
        age_s. Waits briefly for body/status on a cold session."""
        deadline = time.monotonic() + _STATUS_WARMUP_S
        while time.monotonic() < deadline:
            with self._cache_lock:
                if "body/status" in self._cache:
                    break
            time.sleep(0.1)
        now = time.monotonic()
        out: Dict[str, Any] = {}
        with self._cache_lock:
            for topic in _STATUS_TOPICS:
                entry = self._cache.get(topic)
                key = topic.split("/", 1)[1].replace("/", "_")  # body/odom -> odom
                if entry is None:
                    out[key] = None
                else:
                    payload, ts = entry
                    out[key] = {"age_s": round(now - ts, 2), **payload}
        return out


_link: Optional[BodyLink] = None
_link_lock = threading.Lock()


def _get_link() -> BodyLink:
    global _link
    with _link_lock:
        if _link is None:
            _link = BodyLink()
        return _link


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

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


# ---------------------------------------------------------------------------
# System prompt
# ---------------------------------------------------------------------------

def _build_system_prompt() -> str:
    """Static, persona-less prompt: role, the read-only geofence, the
    primitives, and discipline. Stable across calls (caches well)."""
    return (
        "You are a body-sensing subagent. You observe a physical robot "
        "(the Body) through a small set of typed, read-only primitives over "
        "its Zenoh bus. You have no persona, no goals beyond the current "
        "query, and no memory of your own past calls — each invocation is "
        "independent.\n"
        "\n"
        "This is sensing only. You cannot move, turn, or otherwise actuate "
        "the robot — there are no motion primitives. If asked to drive or "
        "act, say plainly that actuation is not available.\n"
        "\n"
        "You cannot interpret image content yourself. `capture_rgb` returns "
        "a file URL for a frame; it does NOT describe the scene. To 'show' "
        "or 'capture' what the camera sees, capture the frame and return its "
        "display URL verbatim in your answer so the caller can display it.\n"
        "\n"
        "## Tools (one JSON object per emission)\n"
        "\n"
        '1. {"thought": "<one sentence>", "tool": "capture_rgb"} — request a '
        "fresh RGB frame from the Body camera, save it, and get back a "
        "ready-to-display URL plus the pixel dimensions.\n"
        '2. {"thought": "...", "tool": "status"} — snapshot the latest '
        "driving/pose/health telemetry (e-stop, odometry pose x/y/theta and "
        "velocities, motor PWM/direction, stall and command-timeout flags). "
        "Each field is tagged with age_s; large age_s or null means that "
        "stream is stale or absent.\n"
        '3. {"thought": "...", "tool": "respond", "text": "<answer>"} — final '
        "answer to the query, exits the loop. When you captured a frame, "
        "include its display URL in the text unchanged.\n"
        "\n"
        "## Discipline\n"
        "\n"
        "- One capture or one status read is usually enough — do not loop.\n"
        "- If a primitive returns ERROR (e.g. the Body is unreachable or the "
        "  capture timed out), report that plainly in `respond`; do not "
        "  retry the same failing call repeatedly.\n"
        "- Answer only from what the primitives return — do not invent pose "
        "  values, scene content, or robot state.\n"
        "- Output ONLY one JSON object per emission. No prose, no markdown "
        "  fences."
    )


# ---------------------------------------------------------------------------
# Primitive observation wrappers (OK/EMPTY/ERROR convention)
# ---------------------------------------------------------------------------

def _tool_capture_rgb() -> str:
    link = _get_link()
    err = link.ensure()
    if err is not None:
        return f"ERROR: Body not reachable at {_BODY_ROUTER} ({err})"
    try:
        result = link.capture_rgb()
    except RuntimeError as e:
        return f"ERROR: capture_rgb failed: {e}"
    except Exception as e:
        logger.exception("body: capture_rgb unexpected failure")
        return f"ERROR: capture_rgb failed: {type(e).__name__}: {e}"
    path_str, dims = result.split("\t", 1)
    url = (f"http://127.0.0.1:{_CANVAS_PORT}/local?path="
           + urllib.parse.quote(path_str))
    img_tag = f'<img src="{url}" style="max-width:100%">'
    return (f"OK: captured {dims} RGB frame. Display it with the display "
            f"tool, format=html: {img_tag}")


def _tool_status() -> str:
    link = _get_link()
    err = link.ensure()
    if err is not None:
        return f"ERROR: Body not reachable at {_BODY_ROUTER} ({err})"
    try:
        snap = link.status_snapshot()
    except Exception as e:
        logger.exception("body: status snapshot failed")
        return f"ERROR: status failed: {type(e).__name__}: {e}"
    if all(v is None for v in snap.values()):
        return (f"EMPTY: connected to {_BODY_ROUTER} but no telemetry has "
                f"arrived yet (Body publishing? topics quiet)")
    return "OK: " + json.dumps(snap, separators=(",", ":"))


# ---------------------------------------------------------------------------
# Loop core
# ---------------------------------------------------------------------------

def _parse_action(raw: str) -> Optional[Dict[str, Any]]:
    obj = repair_json_string(raw or "")
    return obj if isinstance(obj, dict) else None


def _write_trace(trace_dir: Path, query: str,
                 iters: List[Dict[str, Any]], answer: str,
                 exit_reason: str) -> Optional[Path]:
    try:
        trace_dir.mkdir(parents=True, exist_ok=True)
        ts = datetime.now(timezone.utc).strftime('%Y-%m-%dT%H-%M-%SZ')
        path = trace_dir / f'body_{ts}.txt'
        lines = [
            '=' * 80,
            f'[body] {ts} exit={exit_reason} iters={len(iters)}',
            '=' * 80,
            f'Query: {query}',
            '',
        ]
        for i, it in enumerate(iters, start=1):
            lines.append(f'--- iter {i} ---')
            lines.append('ACTION:')
            if it.get('action') is not None:
                lines.append(json.dumps(it['action'], indent=2))
            else:
                lines.append('(unparseable; raw follows)')
                lines.append(it.get('raw', ''))
            obs = it.get('observation', '')
            if obs:
                lines.append('OBSERVATION:')
                lines.append(obs)
            lines.append('')
        lines.append('FINAL ANSWER:')
        lines.append(answer)
        path.write_text('\n'.join(lines), encoding='utf-8')
        return path
    except Exception as e:
        logger.warning(f"body: trace write failed: {e}")
        return None


def body(query: str, llm_backend, trace_dir: Path) -> str:
    """Run the body-sensing subagent. Returns the synthesized answer string,
    suitable for binding to a $stepN observation in Jill's parent ReAct loop.
    Side effect: writes a per-call trace file under trace_dir.

    Args:
        query: natural-language request about what the Body senses.
        llm_backend: a _ChatBackend instance used to generate actions.
            By convention this is the main character backend; the
            per-scenario llm_config decides the model.
        trace_dir: where to write the per-call subagent trace.
    """
    if not query or not query.strip():
        return "(body: empty query)"
    sys_prompt = _build_system_prompt()
    user_prefix = f"Query: {query.strip()}\n\n## Working log\n"
    log_lines: List[str] = []
    iters: List[Dict[str, Any]] = []

    def _build_user_msg() -> str:
        body_msg = user_prefix + ('\n'.join(log_lines) + '\n' if log_lines else '')
        return body_msg + '\nEmit next action:\n'

    answer = ''
    exit_reason = 'max_iters'
    for i in range(_MAX_ITERS):
        messages = [
            {'role': 'system', 'content': sys_prompt},
            {'role': 'user', 'content': _build_user_msg()},
        ]
        try:
            raw = llm_backend.chat(messages, max_tokens=4096, temperature=0.2)
        except Exception as e:
            logger.warning(f"body: llm call failed at iter {i+1}: {e}")
            answer = f"(body: llm error at iter {i+1}: {e})"
            exit_reason = 'llm_error'
            break

        action = _parse_action(raw)
        iter_rec: Dict[str, Any] = {'raw': raw, 'action': action}
        iters.append(iter_rec)
        if action is None:
            log_lines.append(
                "NOTE: previous output was unparseable; emit ONE JSON action now.")
            iter_rec['observation'] = '(unparseable)'
            continue

        tool = action.get('tool')
        if tool == 'respond':
            answer = str(action.get('text', '') or '').strip() or '(no answer)'
            exit_reason = 'respond'
            iter_rec['observation'] = '(respond)'
            break

        binding = f'$step{i+1}'
        if tool == 'capture_rgb':
            obs = _tool_capture_rgb()
        elif tool == 'status':
            obs = _tool_status()
        else:
            obs = (f"ERROR: unknown tool {tool!r}; available: "
                   "capture_rgb, status, respond")

        iter_rec['observation'] = obs
        log_lines.append(f"ACTION {i+1}: {json.dumps(action)}")
        log_lines.append(f"{binding}:")
        log_lines.append(obs)
        log_lines.append('')

    if exit_reason == 'max_iters' and not answer:
        answer = ("(body: hit max iterations without responding; "
                  "consider narrowing the query)")

    _write_trace(trace_dir, query, iters, answer, exit_reason)
    return answer
