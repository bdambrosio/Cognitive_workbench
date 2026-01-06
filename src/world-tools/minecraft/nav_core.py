"""
Shared navigation helpers for Minecraft world-tools.

This module is intentionally world-specific (Minecraft) and is meant to be imported by
multiple tools under src/world-tools/minecraft/*/tool.py.
"""

import logging
import math
import os
import time
from typing import Any, Dict, Optional, Tuple

import requests

logger = logging.getLogger(__name__)

DEFAULT_MINECRAFT_URL = os.getenv("MINECRAFT_URL", "http://localhost:3003")
MAX_NAV_HISTORY = 100


def calculate_snap_position_and_yaw(
    start_pos: Dict[str, float],
    end_pos: Dict[str, float],
    current_yaw: Optional[float] = None,
) -> Tuple[Dict[str, float], float, float]:
    """
    Calculate block-center position and a yaw that faces the movement direction.

    Returns (snap_pos, yaw, pitch) where pitch is always 0.
    """
    block_x = math.floor(end_pos["x"])
    block_z = math.floor(end_pos["z"])

    center_x = block_x + 0.5
    center_z = block_z + 0.5

    dx = end_pos["x"] - start_pos["x"]
    dz = end_pos["z"] - start_pos["z"]

    if abs(dx) < 0.001 and abs(dz) < 0.001:
        yaw = current_yaw if current_yaw is not None else 0.0
    else:
        yaw_rad = math.atan2(-dx, -dz)
        yaw = math.degrees(yaw_rad) % 360
        if yaw < 0:
            yaw += 360

    pitch = 0.0
    return {"x": center_x, "y": end_pos["y"], "z": center_z}, float(yaw), float(pitch)


def call_move_endpoint(
    minecraft_url: str,
    *,
    forward: bool = True,
    duration: float = 0.2,
    check_collision: bool = True,
) -> Optional[Dict[str, Any]]:
    """
    Call bridge /act/move endpoint directly.
    Returns the JSON dict, or None on error.
    """
    move_params: Dict[str, Any] = {"forward": bool(forward), "check_collision": bool(check_collision)}
    if duration is not None:
        move_params["duration_ms"] = int(float(duration) * 1000)

    try:
        timeout_seconds = float(duration) + 5.0
        url = f"{minecraft_url}/act/move"
        logger.debug(f"nav_core: POST {url} body={move_params}")
        response = requests.post(url, json=move_params, timeout=max(10.0, timeout_seconds))
        response.raise_for_status()
        return response.json()
    except Exception as e:
        logger.warning(f"nav_core: move request failed: {e}")
        return None


def snap_to_position(
    minecraft_url: str,
    *,
    x: float,
    y: float,
    z: float,
    yaw: Optional[float] = None,
    pitch: Optional[float] = None,
    timeout_s: float = 5.0,
) -> bool:
    """
    Snap player to an exact position (and optionally orientation) via /act/snapto.

    If yaw/pitch are omitted, bridge preserves current orientation (see handle_snapto).
    """
    body: Dict[str, Any] = {"x": float(x), "y": float(y), "z": float(z)}
    if yaw is not None:
        body["yaw"] = float(yaw)
    if pitch is not None:
        body["pitch"] = float(pitch)

    try:
        url = f"{minecraft_url}/act/snapto"
        logger.debug(f"nav_core: POST {url} body={body}")
        response = requests.post(url, json=body, timeout=float(timeout_s))
        response.raise_for_status()
        data = response.json()
        if not data.get("ok"):
            logger.warning(f"nav_core: snapto failed: {data.get('error', 'unknown error')}")
            return False
        return True
    except Exception as e:
        logger.warning(f"nav_core: snapto request failed: {e}")
        return False


def update_nav_state(
    executor: Any,
    *,
    pose: Dict[str, float],
    support_here: str,
    fell: bool,
    was_fall: bool = False,
) -> None:
    """
    Update executor world_state('nav') by prepending a new entry.
    """
    nav_list = executor.get_world_state("nav")
    if not isinstance(nav_list, list):
        nav_list = []

    nav_entry = {
        "pose": pose.copy(),
        "support_here": support_here,
        "fell": bool(fell),
        "was_fall": bool(was_fall),
        "timestamp": time.time(),
    }

    nav_list.insert(0, nav_entry)
    if len(nav_list) > MAX_NAV_HISTORY:
        nav_list = nav_list[:MAX_NAV_HISTORY]

    executor.set_world_state("nav", nav_list)

