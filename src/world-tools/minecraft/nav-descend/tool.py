"""
Navigation descend primitive.

Attempt exactly ONE controlled descent into an adjacent lower cell.
This covers stepping down slopes, edges, and shallow drops (≤ 1 block).

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


def _call_move_endpoint(minecraft_url: str, forward: bool = True, duration: float = 0.25, check_collision: bool = True) -> Dict:
    """
    Call bridge /act/move endpoint directly.
    
    Returns:
        Dict with {"ok": bool, "status": str, "final_position": dict, ...} or None on error
    """
    move_params = {"forward": True, "check_collision": check_collision}
    if duration is not None:
        move_params["duration_ms"] = int(duration * 1000)
    
    try:
        timeout_seconds = duration + 5.0
        url = f"{minecraft_url}/act/move"
        logger.debug(f"nav-descend: POST {url} body={move_params}")
        response = requests.post(url, json=move_params, timeout=max(10.0, timeout_seconds))
        response.raise_for_status()
        data = response.json()
        return data
    except Exception as e:
        logger.warning(f"nav-descend: move request failed: {e}")
        return None


def _snap_to_position(executor: InfospaceExecutor, snap_pos: Dict, minecraft_url: str) -> bool:
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
        }
        logger.debug(f"nav-descend: POST {url} body={body}")
        response = requests.post(url, json=body, timeout=5.0)
        response.raise_for_status()
        data = response.json()
        if not data.get("ok"):
            logger.warning(f"nav-descend: snapto failed: {data.get('error', 'unknown error')}")
            return False
        return True
    except Exception as e:
        logger.warning(f"nav-descend: snapto request failed: {e}")
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
    Attempt a single controlled descent (-~1Y) forward.

    Args:
        step_duration: float seconds for the move (default: 0.25)
        max_drop: float maximum allowed drop in Y (default: 1.2)
        min_drop: float minimum Y decrease to count as descend (default: 0.2)

    Returns:
        Uniform return with structured data describing the transition.

        Success:
            success=True, descended=True

        Failure:
            success=False
            failure_reason in {
                "collision",
                "fell",
                "observation_failed",
                "support_ambiguous",
                "no_descent",
                "excessive_drop"
            }
    """
    executor: InfospaceExecutor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}

    step_duration = float(kwargs.get("step_duration", 0.25))
    max_drop = float(kwargs.get("max_drop", 1.2))
    min_drop = float(kwargs.get("min_drop", 0.2))
    minecraft_url = kwargs.get("world_url") or kwargs.get("minecraft_url") or DEFAULT_MINECRAFT_URL

    # --- Initial status ---
    status_before = executor.execute_action_with_log({"type": "mc-status"}, "nav-descend")
    if status_before.get("status") != "success":
        return executor._create_uniform_return(
            "failed",
            value="Failed to obtain initial status",
            reason="Failed to obtain initial status",
            data={"success": False, "failure_reason": "status_failed"}
        )

    start_pos = status_before["data"]["position"]
    start_y = start_pos.get("y")
    current_yaw = status_before["data"].get("yaw")  # Get current yaw for preservation if no movement

    # --- Attempt move ---
    move_data = _call_move_endpoint(minecraft_url, forward=True, duration=step_duration, check_collision=True)
    
    if move_data is None or not move_data.get("ok"):
        return executor._create_uniform_return(
            "failed",
            value="Move request failed",
            reason="Move request failed",
            data={
                "success": False,
                "failure_reason": "move_failed",
                "from": start_pos,
                "to": None,
            },
        )

    move_status = move_data.get("status", "success")

    if move_status == "collision":
        return executor._create_uniform_return(
            "failed",
            value="Movement blocked by collision",
            reason="Movement blocked by collision",
            data={
                "success": False,
                "failure_reason": "collision",
                "from": start_pos,
                "to": start_pos,
            },
        )

    if move_status == "fell":
        # Fall detected - validate if it's a controlled descent within bounds
        # Get final position after fall
        status_after = executor.execute_action_with_log({"type": "mc-status"}, "nav-descend")
        if status_after.get("status") != "success":
            return executor._create_uniform_return(
                "failed",
                value="Failed to obtain position after fall",
                reason="Failed to obtain position after fall",
                data={
                    "success": False,
                    "failure_reason": "observation_failed",
                    "from": start_pos,
                    "to": None,
                },
            )
        
        end_pos = status_after.get("data", {}).get("position")
        fall_yaw = status_after.get("data", {}).get("yaw", current_yaw)
        end_y = end_pos.get("y") if end_pos else None
        
        # Calculate delta_y
        delta_y = None
        if isinstance(start_y, (int, float)) and isinstance(end_y, (int, float)):
            delta_y = end_y - start_y
        
        # Observe landing support
        obs = executor.execute_action_with_log({"type": "mc-observe-blocks"}, "nav-descend")
        if obs.get("status") != "success":
            # Snap to position even if observation failed
            if end_pos:
                snap_pos, yaw, pitch = _calculate_snap_position_and_yaw(start_pos, end_pos, current_yaw)
                _snap_to_position(executor, snap_pos, minecraft_url)
                end_pos = snap_pos
                pose = {"x": end_pos["x"], "y": end_pos["y"], "z": end_pos["z"], "yaw": fall_yaw}
                _update_nav_state(executor, pose, "unknown", fell=True, was_fall=True)
            
            return executor._create_uniform_return(
                "failed",
                value="Failed to observe blocks after fall",
                reason="Failed to observe blocks after fall",
                data={
                    "success": False,
                    "failure_reason": "observation_failed",
                    "from": start_pos,
                    "to": end_pos,
                    "delta_y": delta_y,
                },
            )
        
        support_here = obs.get("data", {}).get("support", {}).get("here", {})
        support_type = support_here.get("type", "unknown")
        
        # Snap to fallen block center and set yaw
        if end_pos:
            snap_pos, yaw, pitch = _calculate_snap_position_and_yaw(start_pos, end_pos, current_yaw)
            _snap_to_position(executor, snap_pos, minecraft_url)
            end_pos = snap_pos
        
        # Validate descent bounds
        if delta_y is None or delta_y >= -min_drop:
            # Fall was insufficient descent
            pose = {"x": end_pos["x"], "y": end_pos["y"], "z": end_pos["z"], "yaw": fall_yaw}
            _update_nav_state(executor, pose, support_type, fell=False, was_fall=False)
            
            reason_text = f"Fall insufficient for descent (delta_y={delta_y:.2f})"
            return executor._create_uniform_return(
                "failed",
                value=reason_text,
                reason=reason_text,
                data={
                    "success": False,
                    "failure_reason": "no_descent",
                    "from": start_pos,
                    "to": end_pos,
                    "delta_y": delta_y,
                },
            )
        
        if abs(delta_y) > max_drop:
            # Fall exceeded maximum allowed drop
            pose = {"x": end_pos["x"], "y": end_pos["y"], "z": end_pos["z"], "yaw": fall_yaw}
            _update_nav_state(executor, pose, support_type, fell=True, was_fall=True)
            
            reason_text = f"Fall exceeded maximum drop (delta_y={delta_y:.2f})"
            return executor._create_uniform_return(
                "failed",
                value=reason_text,
                reason=reason_text,
                data={
                    "success": False,
                    "failure_reason": "excessive_drop",
                    "from": start_pos,
                    "to": end_pos,
                    "delta_y": delta_y,
                },
            )
        
        # Validate landing support
        if support_type not in {"solid", "unsafe"}:
            # Landing support is not walkable
            pose = {"x": end_pos["x"], "y": end_pos["y"], "z": end_pos["z"], "yaw": fall_yaw}
            _update_nav_state(executor, pose, support_type, fell=False, was_fall=False)
            
            reason_text = f"Fall landed on non-walkable support: {support_type}"
            return executor._create_uniform_return(
                "failed",
                value=reason_text,
                reason=reason_text,
                data={
                    "success": False,
                    "failure_reason": "support_ambiguous",
                    "from": start_pos,
                    "to": end_pos,
                    "delta_y": delta_y,
                    "support_here": support_type,
                },
            )
        
        # Success: Fall was a controlled descent within bounds with walkable landing
        pose = {"x": end_pos["x"], "y": end_pos["y"], "z": end_pos["z"], "yaw": fall_yaw}
        _update_nav_state(executor, pose, support_type, fell=False, was_fall=False)
        
        return executor._create_uniform_return(
            "success",
            value=f"Controlled descent completed via fall (delta_y={delta_y:.2f})",
            data={
                "success": True,
                "descended": True,
                "from": start_pos,
                "to": end_pos,
                "delta_y": delta_y,
                "support_here": support_type,
            },
        )

    # --- Observe landing ---
    obs = executor.execute_action_with_log({"type": "mc-observe-blocks"}, "nav-descend")
    if obs.get("status") != "success":
        return executor._create_uniform_return(
            "failed",
            value="Failed to observe blocks at destination",
            reason="Failed to observe blocks at destination",
            data={
                "success": False,
                "failure_reason": "observation_failed",
                "from": start_pos,
                "to": start_pos,
            },
        )

    support_here = obs.get("data", {}).get("support", {}).get("here", {})
    support_type = support_here.get("type", "unknown")

    # --- Final status ---
    status_after = executor.execute_action_with_log({"type": "mc-status"}, "nav-descend")
    if status_after.get("status") != "success":
        return executor._create_uniform_return(
            "failed",
            value="Failed to obtain final status",
            reason="Failed to obtain final status",
            data={
                "success": False,
                "failure_reason": "observation_failed",
                "from": start_pos,
                "to": start_pos,
            },
        )

    end_pos = status_after["data"]["position"]
    final_yaw = status_after["data"].get("yaw", current_yaw)
    end_y = end_pos.get("y")

    delta_y = None
    if isinstance(start_y, (int, float)) and isinstance(end_y, (int, float)):
        delta_y = end_y - start_y

    # --- Validate descent ---
    if delta_y is None or delta_y >= -min_drop:
        # Snap to block center even on no_descent failure
        snap_pos, yaw, pitch = _calculate_snap_position_and_yaw(start_pos, end_pos, current_yaw)
        _snap_to_position(executor, snap_pos, minecraft_url)
        end_pos = snap_pos
        
        # Update nav state (no descent)
        pose = {"x": end_pos["x"], "y": end_pos["y"], "z": end_pos["z"], "yaw": final_yaw}
        _update_nav_state(executor, pose, support_type, fell=False, was_fall=False)
        
        reason_text = f"Descent insufficient (delta_y={delta_y:.2f})"
        return executor._create_uniform_return(
            "failed",
            value=reason_text,
            reason=reason_text,
            data={
                "success": False,
                "failure_reason": "no_descent",
                "from": start_pos,
                "to": end_pos,
                "delta_y": delta_y,
            },
        )

    if abs(delta_y) > max_drop:
        # Snap to block center even on excessive_drop failure
        snap_pos, yaw, pitch = _calculate_snap_position_and_yaw(start_pos, end_pos, current_yaw)
        _snap_to_position(executor, snap_pos, minecraft_url)
        end_pos = snap_pos
        
        # Update nav state (excessive drop)
        pose = {"x": end_pos["x"], "y": end_pos["y"], "z": end_pos["z"], "yaw": final_yaw}
        _update_nav_state(executor, pose, support_type, fell=True, was_fall=True)
        
        reason_text = f"Excessive drop detected (delta_y={delta_y:.2f})"
        return executor._create_uniform_return(
            "failed",
            value=reason_text,
            reason=reason_text,
            data={
                "success": False,
                "failure_reason": "excessive_drop",
                "from": start_pos,
                "to": end_pos,
                "delta_y": delta_y,
            },
        )

    # --- Support check ---
    # Accept solid or walkable landings (snow, carpet), reject air/void
    if support_type not in {"solid", "unsafe"}:
        # Snap to block center even on support_ambiguous failure
        snap_pos, yaw, pitch = _calculate_snap_position_and_yaw(start_pos, end_pos, current_yaw)
        _snap_to_position(executor, snap_pos, minecraft_url)
        end_pos = snap_pos
        
        # Update nav state (support ambiguous)
        pose = {"x": end_pos["x"], "y": end_pos["y"], "z": end_pos["z"], "yaw": final_yaw}
        _update_nav_state(executor, pose, support_type, fell=False, was_fall=False)
        
        reason_text = f"Support ambiguous: {support_type}"
        return executor._create_uniform_return(
            "failed",
            value=reason_text,
            reason=reason_text,
            data={
                "success": False,
                "failure_reason": "support_ambiguous",
                "from": start_pos,
                "to": end_pos,
                "delta_y": delta_y,
                "support_here": support_type,
            },
        )

    # --- Success: Snap to destination block center and set yaw ---
    snap_pos, yaw, pitch = _calculate_snap_position_and_yaw(start_pos, end_pos, current_yaw)
    _snap_to_position(executor, snap_pos, minecraft_url)
    end_pos = snap_pos
    
    # Update nav state (successful descent)
    pose = {"x": end_pos["x"], "y": end_pos["y"], "z": end_pos["z"], "yaw": final_yaw}
    _update_nav_state(executor, pose, support_type, fell=False, was_fall=False)
    
    return executor._create_uniform_return(
        "success",
        value=f"Descent completed successfully (delta_y={delta_y:.2f})",
        data={
            "success": True,
            "descended": True,
            "from": start_pos,
            "to": end_pos,
            "delta_y": delta_y,
            "support_here": support_type,
        },
    )


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    print("nav-descend module loaded")
