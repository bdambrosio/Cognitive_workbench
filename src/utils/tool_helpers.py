"""tool_helpers — shared plumbing for ReAct tools that talk to a local bridge.

One canonical implementation of the pieces every bridge-backed tool family
needs: the transport error type, the JSON request wrapper, the react_invoke
wrapper that turns transport failures into ERROR text, and the argument
coercion / position formatting used in tool output.

`utils/factorio_link.py` and `utils/world_link.py` both build on this; each
keeps only what is genuinely specific to its bridge (base URL, timeout, the
"service is offline" hint, and its own response formatting).
"""
from __future__ import annotations

import logging
from typing import Any, Callable, Dict, Optional

import requests

logger = logging.getLogger(__name__)


class BridgeError(Exception):
    """Bridge unreachable or returned a non-JSON/non-200 transport failure."""


def request_json(base_url: str, method: str, path: str, *,
                 timeout_s: float, offline_hint: str = '',
                 **kwargs) -> Dict[str, Any]:
    """Issue one request against a local bridge and return parsed JSON.

    Transport problems raise BridgeError with a message the agent can act
    on — `offline_hint` is where a caller says how to start the service.
    """
    url = base_url.rstrip('/') + path
    try:
        resp = requests.request(method, url, timeout=timeout_s, **kwargs)
    except requests.ConnectionError as e:
        hint = f" — {offline_hint}" if offline_hint else ''
        raise BridgeError(f"bridge unreachable at {base_url}{hint} [{e}]")
    except requests.RequestException as e:
        raise BridgeError(f"bridge request failed: {e}")
    if resp.status_code != 200:
        raise BridgeError(
            f"bridge HTTP {resp.status_code} on {path}: {resp.text[:300]}")
    try:
        return resp.json()
    except ValueError as e:
        raise BridgeError(f"bridge returned non-JSON on {path}: {e}")


def run_tool(fn: Callable[[], Dict[str, Any]], logger_=None) -> Dict[str, Any]:
    """Standard react_invoke wrapper: fn() -> {status, text}, with transport
    failures surfaced as ERROR text instead of a raised exception."""
    log = logger_ or logger
    try:
        return fn()
    except BridgeError as e:
        log.warning(f"tool: {e}")
        return {"status": "error", "text": str(e)}


def parse_number(args: dict, name: str, required: bool = True, default=None):
    """Returns (value, error_text)."""
    raw = args.get(name)
    if raw is None:
        if required:
            return None, f"missing required arg: {name}"
        return default, None
    try:
        return float(raw), None
    except (TypeError, ValueError):
        return None, f"{name} must be a number, got {raw!r}"


def fmt_pos(x, y=None) -> str:
    """Format a 2-D position. Accepts (x, y) or a dict with x/y keys."""
    if isinstance(x, dict):
        x, y = x.get("x"), x.get("y")
    try:
        return f"({float(x):.1f}, {float(y):.1f})"
    except (TypeError, ValueError):
        return f"({x}, {y})"
