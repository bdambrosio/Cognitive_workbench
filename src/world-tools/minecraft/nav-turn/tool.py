"""
Navigation turn primitive.

Change orientation to a cardinal direction (0, 90, 180, 270).
Snaps to block center and sets pitch to 0.

Atomic, bounded, composable.
"""

import logging
import math
import os
import requests
import time
from typing import Dict
from infospace_executor import InfospaceExecutor

logger = logging.getLogger(__name__)

DEFAULT_MINECRAFT_URL = os.getenv("MINECRAFT_URL", "http://localhost:3003")
MAX_NAV_HISTORY = 100


def _round_to_cardinal(yaw_deg: float) -> float:
    """
    Round yaw to nearest cardinal direction (0, 90, 180, 270).
    
    Args:
        yaw_deg: Yaw in degrees
    
    Returns:
        Nearest cardinal yaw (0, 90, 180, or 270)
    """
    cardinals = [0, 90, 180, 270]
    normalized = yaw_deg % 360
    
    # Find nearest cardinal
    nearest = min(cardinals, key=lambda c: min(
        abs(normalized - c),
        abs(normalized - c + 360),
        abs(normalized - c - 360)
    ))
    
    return nearest


def _snap_to_position(executor: InfospaceExecutor, snap_pos: Dict, yaw: float, pitch: float, minecraft_url: str) -> bool:
    """
    Snap player to exact position and orientation via bridge endpoint.
    
    Returns:
        True if successful, False otherwise
    """
    try:
        url = f"{minecraft_url}/act/snapto"
        body = {
            "x": snap_pos['x'],
            "y": snap_pos['y'],
            "z": snap_pos['z'],
            "yaw": yaw,
            "pitch": pitch
        }
        logger.debug(f"nav-turn: POST {url} body={body}")
        response = requests.post(url, json=body, timeout=5.0)
        response.raise_for_status()
        data = response.json()
        if not data.get("ok"):
            logger.warning(f"nav-turn: snapto failed: {data.get('error', 'unknown error')}")
            return False
        return True
    except Exception as e:
        logger.warning(f"nav-turn: snapto request failed: {e}")
        return False


def _update_nav_state(executor: InfospaceExecutor, pose: Dict, support_here: str, fell: bool, was_fall: bool = False):
    """
    Update navigation state by prepending a new entry to the nav list.
    
    Args:
        executor: InfospaceExecutor instance
        pose: Dict with x, y, z, yaw
        support_here: "solid", "unsafe", "unknown", etc.
        fell: Whether the agent fell
        was_fall: Whether this was a fall event (for nav-backtrack safety check)
    """
    nav_list = executor.get_world_state("nav")
    if not isinstance(nav_list, list):
        nav_list = []
    
    nav_entry = {
        "pose": pose.copy(),
        "support_here": support_here,
        "fell": fell,
        "was_fall": was_fall,
        "timestamp": time.time()
    }
    
    # Prepend new entry
    nav_list.insert(0, nav_entry)
    
    # Limit to MAX_NAV_HISTORY
    if len(nav_list) > MAX_NAV_HISTORY:
        nav_list = nav_list[:MAX_NAV_HISTORY]
    
    executor.set_world_state("nav", nav_list)


def tool(input_value=None, **kwargs):
    """
    Turn to a cardinal direction and snap to block center.

    Args:
        direction: one of {"right", "left", "back", "forward"} (required)
        
    Returns:
        Uniform return with structured data describing the turn.
        
        Success:
            success=True, yaw=<cardinal>
            
        Failure:
            success=False
            failure_reason in {
                "invalid_direction",
                "status_failed",
                "snapto_failed"
            }
    """
    executor: InfospaceExecutor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}

    direction = kwargs.get("direction")
    if direction not in {"right", "left", "back", "forward"}:
        return executor._create_uniform_return(
            "failed",
            value=f"Invalid direction: {direction}. Must be one of: right, left, back, forward",
            data={"success": False, "failure_reason": "invalid_direction"}
        )

    minecraft_url = kwargs.get("world_url") or kwargs.get("minecraft_url") or DEFAULT_MINECRAFT_URL

    # --- Get current status ---
    status = executor.execute_action_with_log({"type": "mc-status"}, "nav-turn")
    if status.get("status") != "success":
        return executor._create_uniform_return(
            "failed",
            value="Failed to obtain current status",
            data={"success": False, "failure_reason": "status_failed"}
        )

    current_pos = status["data"]["position"]
    current_yaw = status["data"].get("yaw", 0.0)
    
    # Normalize current yaw to 0-360
    current_yaw = current_yaw % 360
    if current_yaw < 0:
        current_yaw += 360

    # --- Calculate new yaw based on direction ---
    if direction == "right":
        new_yaw = current_yaw + 90
    elif direction == "left":
        new_yaw = current_yaw - 90
    elif direction == "back":
        new_yaw = current_yaw + 180
    else:  # forward
        new_yaw = current_yaw  # Will round to nearest cardinal

    # Normalize to 0-360
    new_yaw = new_yaw % 360
    if new_yaw < 0:
        new_yaw += 360

    # --- Round to nearest cardinal ---
    cardinal_yaw = _round_to_cardinal(new_yaw)

    # --- Calculate block center position ---
    block_x = math.floor(current_pos['x'])
    block_z = math.floor(current_pos['z'])
    snap_pos = {
        "x": block_x + 0.5,
        "y": current_pos['y'],
        "z": block_z + 0.5
    }

    # --- Snap to position and orientation ---
    pitch = 0.0
    if not _snap_to_position(executor, snap_pos, cardinal_yaw, pitch, minecraft_url):
        return executor._create_uniform_return(
            "failed",
            value="Failed to snap to position and orientation",
            data={
                "success": False,
                "failure_reason": "snapto_failed",
                "yaw": cardinal_yaw,
            },
        )

    # Update nav state (pose change - yaw changed)
    pose = {"x": snap_pos["x"], "y": snap_pos["y"], "z": snap_pos["z"], "yaw": cardinal_yaw}
    # Get current support_here from previous nav state if available
    nav_list = executor.get_world_state("nav")
    support_here = "unknown"
    if isinstance(nav_list, list) and len(nav_list) > 0:
        support_here = nav_list[0].get("support_here", "unknown")
    _update_nav_state(executor, pose, support_here, fell=False, was_fall=False)

    # --- Success ---
    return executor._create_uniform_return(
        "success",
        value=f"Turned to {direction}, yaw={cardinal_yaw}°",
        data={
            "success": True,
            "yaw": cardinal_yaw,
            "direction": direction,
        },
    )


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    print("nav-turn module loaded")

