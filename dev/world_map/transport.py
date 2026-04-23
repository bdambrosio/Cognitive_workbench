"""Zenoh session helpers. Same pattern as dev/body_stub/transport.py —
single explicit remote endpoint, no multicast scouting.
"""
from __future__ import annotations

import json
import logging
from typing import Any

logger = logging.getLogger(__name__)


def normalize_endpoint(endpoint: str) -> str:
    s = endpoint.strip()
    if "/" in s:
        return s
    return f"tcp/{s}"


def make_remote_config(endpoint: str) -> Any:
    import zenoh
    ep = normalize_endpoint(endpoint)
    config = zenoh.Config()
    config.insert_json5("connect/endpoints", json.dumps([ep]))
    config.insert_json5("scouting/multicast/enabled", "false")
    config.insert_json5("scouting/gossip/enabled", "true")
    return config


def open_session(endpoint: str) -> Any:
    import zenoh
    ep = normalize_endpoint(endpoint)
    logger.info(f"opening zenoh session → {ep}")
    return zenoh.open(make_remote_config(ep))
