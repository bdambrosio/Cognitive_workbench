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


def agent_rel_to_world(dx: float, dy: float, dz: float, yaw: float) -> Tuple[float, float, float]:
    """
    Convert agent-relative coordinates to world-relative coordinates.
    
    Args:
        dx: Agent-relative X offset (positive = right, negative = left)
        dy: Agent-relative Y offset (positive = up, negative = down) - unchanged
        dz: Agent-relative Z offset (positive = forward, negative = back)
        yaw: Agent yaw in degrees (cardinal: 0, 90, 180, 270)
    
    Returns:
        Tuple of (world_dx, world_dy, world_dz) as floats
        - world_dx: World-relative X (positive = east, negative = west)
        - world_dy: World-relative Y (same as dy, always vertical)
        - world_dz: World-relative Z (positive = south, negative = north)
    """
    yaw_rad = math.radians(yaw)
    # Transform agent-relative (right/forward) to world-relative (east/south)
    # Fixed: forward (dz=+1) at yaw=0° maps to south (world_dz=+1), not north
    world_dx = -math.sin(yaw_rad) * dz - math.cos(yaw_rad) * dx
    world_dz = math.cos(yaw_rad) * dz - math.sin(yaw_rad) * dx
    return world_dx, dy, world_dz


def world_to_agent_rel(world_dx: float, world_dy: float, world_dz: float, yaw: float) -> Tuple[float, float, float]:
    """
    Convert world-relative coordinates to agent-relative coordinates.
    
    Args:
        world_dx: World-relative X (positive = east, negative = west)
        world_dy: World-relative Y (positive = up, negative = down)
        world_dz: World-relative Z (positive = south, negative = north)
        yaw: Agent yaw in degrees (cardinal: 0, 90, 180, 270)
    
    Returns:
        Tuple of (dx, dy, dz) as floats
        - dx: Agent-relative X (positive = right, negative = left)
        - dy: Agent-relative Y (same as world_dy, always vertical)
        - dz: Agent-relative Z (positive = forward, negative = back)
    """
    yaw_rad = math.radians(yaw)
    # Inverse transform: world-relative (east/south) to agent-relative (right/forward)
    # Fixed to match corrected forward transform
    dx = -math.cos(yaw_rad) * world_dx - math.sin(yaw_rad) * world_dz
    dz = -math.sin(yaw_rad) * world_dx + math.cos(yaw_rad) * world_dz
    return dx, world_dy, dz


# Agent-relative face names (egocentric)
RELATIVE_FACES = {"forward", "back", "left", "right", "up", "down"}

# Absolute face names (world cardinals)
ABSOLUTE_FACES = {"north", "south", "east", "west", "up", "down", "top", "bottom"}


def relative_face_to_absolute(face: str, yaw: float) -> str:
    """
    Convert agent-relative face to absolute world face.
    
    Agent-relative faces:
        - forward: direction agent is facing
        - back: opposite of forward
        - left: 90° counter-clockwise from forward
        - right: 90° clockwise from forward
        - up/down: unchanged (always vertical)
    
    Absolute faces (Minecraft world coordinates):
        - north: -Z direction
        - south: +Z direction
        - east: +X direction
        - west: -X direction
        - up/down: unchanged
    
    Yaw convention (Minecraft):
        - 0° = facing South (+Z)
        - 90° = facing West (-X)
        - 180° = facing North (-Z)
        - 270° = facing East (+X)
    
    Args:
        face: Agent-relative face name (forward, back, left, right, up, down)
              OR already-absolute face (north, south, east, west, up, down, top, bottom)
        yaw: Agent yaw in degrees (will be cardinal-snapped)
    
    Returns:
        Absolute face name (north, south, east, west, up, down)
        Returns input unchanged if already absolute or unrecognized.
    """
    face_lower = face.lower().strip()
    
    # Handle vertical faces (unchanged)
    if face_lower in ("up", "down", "top", "bottom"):
        return "up" if face_lower in ("up", "top") else "down"
    
    # If already absolute, return as-is
    if face_lower in ("north", "south", "east", "west"):
        return face_lower
    
    # Cardinal-snap yaw
    yaw = _round_to_cardinal(yaw)
    
    # Mapping from yaw to absolute forward direction
    # yaw 0° = south, 90° = west, 180° = north, 270° = east
    yaw_to_forward = {
        0.0: "south",
        90.0: "west",
        180.0: "north",
        270.0: "east",
    }
    
    # Derive all directions from forward
    forward = yaw_to_forward.get(yaw, "south")
    
    # Clockwise rotation: south -> west -> north -> east -> south
    rotation_order = ["south", "west", "north", "east"]
    fwd_idx = rotation_order.index(forward)
    
    direction_map = {
        "forward": rotation_order[fwd_idx],
        "right": rotation_order[(fwd_idx + 1) % 4],
        "back": rotation_order[(fwd_idx + 2) % 4],
        "left": rotation_order[(fwd_idx + 3) % 4],
    }
    
    result = direction_map.get(face_lower)
    if result:
        return result
    
    # Unrecognized face - return as-is (let caller handle error)
    return face


def dx_dy_dz_to_absolute(dx: float, dy: float, dz: float, agent_pos: Dict[str, float], yaw: Optional[float] = None) -> Tuple[int, int, int]:
    """
    Convert agent-relative coordinates (dx, dy, dz) to absolute block coordinates.
    
    Args:
        dx: Agent-relative X offset (positive = right, negative = left)
        dy: Agent-relative Y offset (positive = up, negative = down)
        dz: Agent-relative Z offset (positive = forward, negative = back)
        agent_pos: Agent position dict with 'x', 'y', 'z', optionally 'yaw'
        yaw: Optional agent yaw in degrees (if None, uses agent_pos.get('yaw', 0.0))
    
    Returns:
        Tuple of (absolute_x, absolute_y, absolute_z) as integers (block coordinates)
    
    Note: dx, dy, dz are agent-relative (right/forward/up), transformed using yaw.
    Yaw is cardinal-snapped (0°, 90°, 180°, 270°) for consistency.
    """
    px = agent_pos.get('x', 0.0)
    py = agent_pos.get('y', 0.0)
    pz = agent_pos.get('z', 0.0)
    
    # Get yaw (cardinal-snapped)
    agent_yaw = yaw if yaw is not None else agent_pos.get('yaw', 0.0)
    agent_yaw = _round_to_cardinal(agent_yaw)
    
    # Convert agent-relative to world-relative
    world_dx, world_dy, world_dz = agent_rel_to_world(dx, dy, dz, agent_yaw)
    
    # Convert to block coordinates (floor) then add world-relative offset
    bx = int(math.floor(px))
    by = int(math.floor(py))
    bz = int(math.floor(pz))
    
    return (bx + int(round(world_dx)), by + int(round(world_dy)), bz + int(round(world_dz)))


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


if __name__ == "__main__":
    # Unit tests for relative_face_to_absolute
    print("Testing relative_face_to_absolute:")
    
    # Expected mappings for each yaw
    expected = {
        0.0: {"forward": "south", "back": "north", "left": "east", "right": "west"},
        90.0: {"forward": "west", "back": "east", "left": "south", "right": "north"},
        180.0: {"forward": "north", "back": "south", "left": "west", "right": "east"},
        270.0: {"forward": "east", "back": "west", "left": "north", "right": "south"},
    }
    
    errors = []
    for yaw, mappings in expected.items():
        for rel_face, expected_abs in mappings.items():
            actual = relative_face_to_absolute(rel_face, yaw)
            if actual != expected_abs:
                errors.append(f"yaw={yaw}, face={rel_face}: expected {expected_abs}, got {actual}")
            else:
                print(f"  ✓ yaw={yaw}° {rel_face} -> {actual}")
    
    # Test vertical faces (unchanged)
    for face in ["up", "down"]:
        for yaw in [0.0, 90.0, 180.0, 270.0]:
            actual = relative_face_to_absolute(face, yaw)
            if actual != face:
                errors.append(f"yaw={yaw}, face={face}: expected {face}, got {actual}")
    print("  ✓ up/down pass through unchanged")
    
    # Test absolute faces pass through
    for face in ["north", "south", "east", "west"]:
        actual = relative_face_to_absolute(face, 0.0)
        if actual != face:
            errors.append(f"absolute face={face}: expected {face}, got {actual}")
    print("  ✓ absolute faces pass through unchanged")
    
    # Test top/bottom normalization
    assert relative_face_to_absolute("top", 0.0) == "up"
    assert relative_face_to_absolute("bottom", 0.0) == "down"
    print("  ✓ top/bottom normalized to up/down")
    
    if errors:
        print(f"\n❌ {len(errors)} errors:")
        for e in errors:
            print(f"  {e}")
        exit(1)
    else:
        print("\n✓ All tests passed!")

