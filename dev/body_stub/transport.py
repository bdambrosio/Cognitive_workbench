"""Zenoh session helpers for the body stub.

Mirrors the pattern in src/utils/zenoh_utils.make_localhost_config, but
connects to an explicit remote router endpoint rather than scouting
localhost. Multicast scouting is disabled so the stub binds only to the
configured router (predictable behavior on a LAN with other Zenoh peers).
"""
from __future__ import annotations

import json
import logging
from typing import Any

logger = logging.getLogger(__name__)


def normalize_endpoint(endpoint: str) -> str:
    """Zenoh wants `<protocol>/<address>`. If the caller passed a bare
    `host:port` (no `/`), assume tcp. Whitespace is stripped.
    """
    s = endpoint.strip()
    if "/" in s:
        return s
    return f"tcp/{s}"


def make_remote_config(endpoint: str) -> Any:
    """Build a zenoh.Config that connects to one explicit router."""
    import zenoh
    ep = normalize_endpoint(endpoint)
    config = zenoh.Config()
    config.insert_json5("connect/endpoints", json.dumps([ep]))
    config.insert_json5("scouting/multicast/enabled", "false")
    config.insert_json5("scouting/gossip/enabled", "true")
    return config


def open_session(endpoint: str) -> Any:
    """Open a Zenoh session to the given router. Raises on failure."""
    import zenoh
    ep = normalize_endpoint(endpoint)
    logger.info(f"opening zenoh session → {ep}")
    return zenoh.open(make_remote_config(ep))
