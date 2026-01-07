"""
Navigation climb primitive (v2).

Attempt exactly ONE adjacent climb with automatic handling of Minecraft ascent semantics:
- Try walk-up (for slabs/stairs/partial rises)
- If needed, try jump-up (for full 1-block ledges)
- Accept walkable landings (e.g., snow layers) by default

Atomic, bounded, composable.
"""

import logging
import math
import os
import requests
import time
from typing import Dict, Tuple, Optional
from infospace_executor import InfospaceExecutor

logger = logging.getLogger(__name__)

DEFAULT_MINECRAFT_URL = os.getenv("MINECRAFT_URL", "http://localhost:3003")
MAX_NAV_HISTORY = 100


def _calculate_snap_position_and_yaw(start_pos: Dict, end_pos: Dict, current_yaw: Optional[float] = None) -> Tuple[Dict, float, float]:
    """
    Calculate block center position and yaw from movement.
    
    Args:
        start_pos: Starting position dict with x, y, z
        end_pos: Ending position dict with x, y, z
        current_yaw: Current yaw in degrees (used if no horizontal movement)
    
    Returns:
        (snap_pos, yaw, pitch) where:
        - snap_pos: {"x": center_x, "y": end_y, "z": center_z}
        - yaw: degrees (0-360)
        - pitch: degrees (always 0 for horizontal movement)
    """
    # Get block coordinates (floor)
    block_x = math.floor(end_pos['x'])
    block_z = math.floor(end_pos['z'])
    
    # Block center (keep actual Y)
    center_x = block_x + 0.5
    center_z = block_z + 0.5
    
    # Calculate movement vector
    dx = end_pos['x'] - start_pos['x']
    dz = end_pos['z'] - start_pos['z']
    
    # Calculate yaw from movement direction
    if abs(dx) < 0.001 and abs(dz) < 0.001:
        # No horizontal movement - preserve current yaw
        if current_yaw is not None:
            yaw = current_yaw
        else:
            yaw = 0.0
    else:
        # Calculate yaw from movement vector (Minecraft convention)
        yaw_rad = math.atan2(-dx, -dz)
        yaw = math.degrees(yaw_rad)
        # Normalize to 0-360
        yaw = yaw % 360
        if yaw < 0:
            yaw += 360
    
    # Pitch: always 0 for horizontal movement
    pitch = 0.0
    
    return {"x": center_x, "y": end_pos['y'], "z": center_z}, yaw, pitch


def _call_move_endpoint(minecraft_url: str, forward: bool = True, duration: float = 0.6, jump: bool = False, check_collision: bool = True) -> Dict:
    """
    Call bridge /act/move endpoint directly.
    
    Returns:
        Dict with {"ok": bool, "status": str, "final_position": dict, ...} or None on error
    """
    move_params = {"forward": True, "check_collision": check_collision}
    if jump:
        move_params["jump"] = True
    if duration is not None:
        move_params["duration_ms"] = int(duration * 1000)
    
    try:
        timeout_seconds = duration + 5.0
        url = f"{minecraft_url}/act/move"
        logger.debug(f"nav-climb: POST {url} body={move_params}")
        response = requests.post(url, json=move_params, timeout=max(10.0, timeout_seconds))
        response.raise_for_status()
        data = response.json()
        return data
    except Exception as e:
        logger.warning(f"nav-climb: move request failed: {e}")
        return None


def _snap_to_position(executor: InfospaceExecutor, snap_pos: Dict, minecraft_url: str) -> bool:
    """
    Snap player to exact position via bridge endpoint (preserves current orientation).
    
    Returns:
        True if successful, False otherwise
    """
    try:
        url = f"{minecraft_url}/act/snapto"
        body = {
            "x": snap_pos['x'],
            "y": snap_pos['y'],
            "z": snap_pos['z'],
        }
        logger.debug(f"nav-climb: POST {url} body={body}")
        response = requests.post(url, json=body, timeout=5.0)
        response.raise_for_status()
        data = response.json()
        if not data.get("ok"):
            logger.warning(f"nav-climb: snapto failed: {data.get('error', 'unknown error')}")
            return False
        return True
    except Exception as e:
        logger.warning(f"nav-climb: snapto request failed: {e}")
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
    Attempt a single climb (+~1Y) forward.

    Args:
        step_duration: float seconds for each attempt (default: 0.6)
        allow_walkable_landing: bool (default: True)
        min_delta_y: float minimum Y gain to count as climb (default: 0.9)

    Returns:
        Uniform return with structured data describing the transition.

        Success:
            success=True, climbed=True, delta_y>=min_delta_y

        Failure:
            success=False
            failure_reason in {
                "collision",
                "fell",
                "observation_failed",
                "support_ambiguous",
                "not_elevated"
            }
    """
    executor: InfospaceExecutor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}

    step_duration = float(kwargs.get("step_duration", 0.6))
    allow_walkable_landing = bool(kwargs.get("allow_walkable_landing", True))
    min_delta_y = float(kwargs.get("min_delta_y", 0.9))
    minecraft_url = kwargs.get("world_url") or kwargs.get("minecraft_url") or DEFAULT_MINECRAFT_URL

    # --- Capture starting status ---
    status_before = executor.execute_action_with_log({"type": "mc-status"}, "nav-climb")
    if status_before.get("status") != "success":
        return executor._create_uniform_return(
            "failed",
            value="Failed to obtain initial status",
            reason="status_failed"
        )

    start_pos = status_before["data"]["position"]
    start_y = start_pos.get("y")
    current_yaw = status_before["data"].get("yaw")  # Get current yaw for preservation if no movement

    def attempt(move_kwargs, label):
        jump = move_kwargs.get("jump", False)
        move_data = _call_move_endpoint(minecraft_url, forward=True, duration=step_duration, jump=jump, check_collision=True)
        if move_data is None or not move_data.get("ok"):
            return {"ok": False, "reason": "move_failed"}

        move_status = move_data.get("status", "success")
        if move_status == "collision":
            return {"ok": False, "reason": "collision"}
        if move_status == "fell":
            return {"ok": False, "reason": "fell"}

        # Observe
        obs = executor.execute_action_with_log({"type": "mc-observe-blocks"}, f"nav-climb:{label}")
        if obs.get("status") != "success":
            return {"ok": False, "reason": "observation_failed"}

        # Status after
        status_after = executor.execute_action_with_log({"type": "mc-status"}, f"nav-climb:{label}")
        if status_after.get("status") != "success":
            return {"ok": False, "reason": "observation_failed"}

        end_pos = status_after["data"]["position"]
        end_y = end_pos.get("y")
        delta_y = None
        if isinstance(start_y, (int, float)) and isinstance(end_y, (int, float)):
            delta_y = end_y - start_y

        support_here = obs.get("data", {}).get("support", {}).get("here", {})
        support_type = support_here.get("type", "unknown")

        return {
            "ok": True,
            "end_pos": end_pos,
            "delta_y": delta_y,
            "support_here": support_type,
        }

    # --- Attempt A: walk-up ---
    res = attempt({}, "walk")
    if res.get("ok"):
        dy = res.get("delta_y")
        if dy is not None and dy >= min_delta_y:
            if allow_walkable_landing or res.get("support_here") == "solid":
                end_pos = res.get("end_pos")
                # Snap to block center (preserves current yaw)
                if end_pos:
                    snap_pos, _, _ = _calculate_snap_position_and_yaw(start_pos, end_pos, current_yaw)
                    _snap_to_position(executor, snap_pos, minecraft_url)
                    end_pos = snap_pos
                    
                    # Update nav state (successful climb) - preserve current_yaw
                    pose = {"x": end_pos["x"], "y": end_pos["y"], "z": end_pos["z"], "yaw": current_yaw}
                    _update_nav_state(executor, pose, res.get("support_here", "unknown"), fell=False, was_fall=False)
                
                return executor._create_uniform_return(
                    "success",
                    value=f"Climbed successfully via walk (delta_y={dy:.2f})",
                    extra={
                        "from": start_pos,
                        "to": end_pos,
                        "delta_y": dy,
                        "mode": "walk",
                        "support_here": res.get("support_here"),
                    },
                )

    # --- Attempt B: jump-up ---
    res = attempt({"jump": True}, "jump")
    if res.get("ok"):
        dy = res.get("delta_y")
        if dy is not None and dy >= min_delta_y:
            if allow_walkable_landing or res.get("support_here") == "solid":
                end_pos = res.get("end_pos")
                # Snap to block center (preserves current yaw)
                if end_pos:
                    snap_pos, _, _ = _calculate_snap_position_and_yaw(start_pos, end_pos, current_yaw)
                    _snap_to_position(executor, snap_pos, minecraft_url)
                    end_pos = snap_pos
                    
                    # Update nav state (successful climb) - preserve current_yaw
                    pose = {"x": end_pos["x"], "y": end_pos["y"], "z": end_pos["z"], "yaw": current_yaw}
                    _update_nav_state(executor, pose, res.get("support_here", "unknown"), fell=False, was_fall=False)
                
                return executor._create_uniform_return(
                    "success",
                    value=f"Climbed successfully via jump (delta_y={dy:.2f})",
                    extra={
                        "from": start_pos,
                        "to": end_pos,
                        "delta_y": dy,
                        "mode": "jump",
                        "support_here": res.get("support_here"),
                    },
                )

    # --- Failure classification ---
    reason = res.get("reason", "not_elevated")
    end_pos = res.get("end_pos") if isinstance(res, dict) else None
    
    # Snap to block center even on failure if we have a position (preserves current yaw)
    if end_pos:
        snap_pos, _, _ = _calculate_snap_position_and_yaw(start_pos, end_pos, current_yaw)
        _snap_to_position(executor, snap_pos, minecraft_url)
        end_pos = snap_pos
        
        # Update nav state (climb failure) - preserve current_yaw
        pose = {"x": end_pos["x"], "y": end_pos["y"], "z": end_pos["z"], "yaw": current_yaw}
        support_here = res.get("support_here", "unknown") if isinstance(res, dict) else "unknown"
        _update_nav_state(executor, pose, support_here, fell=False, was_fall=False)
    
    return executor._create_uniform_return(
        "failed",
        value=f"Climb failed: {reason}",
        reason=reason,
    )


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    print("nav-climb v2 module loaded")
