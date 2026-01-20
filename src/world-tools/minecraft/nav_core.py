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

# Session-local rolling occupancy grid (agent-owned)
try:
    from local_grid import set_center_from_pose
except Exception:
    set_center_from_pose = None  # optional; do not break nav_core

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


def _round_to_cardinal(yaw: float) -> float:
    """
    Round yaw to nearest cardinal direction (0°, 90°, 180°, 270°).
    """
    normalized = yaw % 360
    if normalized < 0:
        normalized += 360
    
    # Thresholds at midpoints: 45°, 135°, 225°, 315°
    if normalized < 45 or normalized >= 315:
        return 0.0
    elif normalized < 135:
        return 90.0
    elif normalized < 225:
        return 180.0
    else:  # normalized < 315
        return 270.0


def dx_dy_dz_to_absolute(dx: float, dy: float, dz: float, agent_pos: Dict[str, float]) -> Tuple[int, int, int]:
    """
    Convert world-relative coordinates (dx, dy, dz) to absolute block coordinates.
    
    Args:
        dx: World-relative X offset (positive = east, negative = west)
        dy: World-relative Y offset (positive = up, negative = down)
        dz: World-relative Z offset (positive = south, negative = north)
        agent_pos: Agent position dict with 'x', 'y', 'z' keys
    
    Returns:
        Tuple of (absolute_x, absolute_y, absolute_z) as integers (block coordinates)
    
    Note: dx, dy, dz are world-relative (absolute axis directions), NOT egocentric.
    This is different from 'forward' which depends on yaw.
    """
    px = agent_pos.get('x', 0.0)
    py = agent_pos.get('y', 0.0)
    pz = agent_pos.get('z', 0.0)
    
    # Convert to block coordinates (floor) then add offset
    bx = int(math.floor(px))
    by = int(math.floor(py))
    bz = int(math.floor(pz))
    
    return (bx + int(round(dx)), by + int(round(dy)), bz + int(round(dz)))


def ensure_grid_aligned(
    executor: Any,
    minecraft_url: str,
    *,
    status_data: Optional[Dict[str, Any]] = None,
) -> Tuple[Dict[str, float], float]:
    """
    Ensure agent is grid-aligned before movement: block center, cardinal yaw, pitch 0.
    
    MUST be called before any movement command to bridge.
    
    Args:
        executor: InfospaceExecutor instance
        minecraft_url: Minecraft server URL
        status_data: Optional pre-fetched status data (if None, fetches via mc-status)
    
    Returns:
        (normalized_pose, normalized_yaw) where:
        - normalized_pose: {"x": block_center_x, "y": current_y, "z": block_center_z}
        - normalized_yaw: cardinal yaw (0, 90, 180, or 270)
    
    On error: logs warning and returns current pose/yaw (proceeds anyway).
    """
    # Get current status if not provided
    if status_data is None:
        status_result = executor.execute_action_with_log({"type": "mc-status"}, "ensure_grid_aligned")
        if status_result.get("status") != "success":
            logger.warning("ensure_grid_aligned: Failed to get status, proceeding without alignment")
            # Return dummy pose - caller should handle this
            return {"x": 0.0, "y": 0.0, "z": 0.0}, 0.0
        status_data = status_result.get("data", {})
    
    current_pos = status_data.get("position", {})
    current_yaw = status_data.get("yaw", 0.0)
    current_pitch = status_data.get("pitch", 0.0)
    
    # Calculate block center
    block_x = math.floor(current_pos.get("x", 0.0))
    block_z = math.floor(current_pos.get("z", 0.0))
    center_x = block_x + 0.5
    center_z = block_z + 0.5
    
    # Round yaw to nearest cardinal (no-op if already cardinal)
    normalized_yaw = _round_to_cardinal(current_yaw)
    
    # Always set pitch to 0
    normalized_pitch = 0.0
    
    # Snap to grid-aligned position and orientation
    normalized_pose = {
        "x": center_x,
        "y": current_pos.get("y", 0.0),  # Preserve Y (vertical position)
        "z": center_z,
    }
    
    success = snap_to_position(
        minecraft_url,
        x=normalized_pose["x"],
        y=normalized_pose["y"],
        z=normalized_pose["z"],
        yaw=normalized_yaw,
        pitch=normalized_pitch,
    )
    
    if not success:
        logger.warning(
            f"ensure_grid_aligned: snap_to_position failed, proceeding anyway. "
            f"Target: ({normalized_pose['x']:.2f}, {normalized_pose['y']:.2f}, {normalized_pose['z']:.2f}), "
            f"yaw={normalized_yaw:.1f}, pitch={normalized_pitch:.1f}"
        )

    # Side-effect: update session-local grid center and prune (tool moved/realigned pose)
    try:
        if set_center_from_pose is not None:
            set_center_from_pose(executor, normalized_pose)
    except Exception:
        pass
    
    return normalized_pose, normalized_yaw


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

