"""
Minecraft initialization tool.

Normalizes agent pose to block center, cardinal yaw, and pitch zero,
then initializes navigation state in world_state.
"""

import logging
import time
from typing import Dict, List
from infospace_executor import InfospaceExecutor

logger = logging.getLogger(__name__)


def tool(input_value=None, **kwargs):
    """
    Initialize navigation state by normalizing current pose.
    
    Steps:
    1. Get current status (position, yaw, pitch)
    2. Use nav-turn with "forward" to normalize to block center, cardinal yaw, pitch 0
    3. Get normalized status
    4. Initialize nav state in world_state with normalized pose
    
    Returns:
        Uniform return format dict with success status and normalized pose.
    """
    executor: InfospaceExecutor = kwargs.get("executor")
    if not executor:
        return {
            "status": "failed",
            "reason": "executor not available",
            "value": None,
            "resource_id": None
        }
    
    # Step 1: Get current status
    status_result = executor.execute_action_with_log({"type": "mc-status"}, "init")
    if status_result.get("status") != "success":
        return executor._create_uniform_return(
            "failed",
            value="Failed to get initial status",
            reason="status_failed"
        )
    
    status_data = status_result.get("data", {})
    initial_pos = status_data.get("position", {})
    initial_yaw = status_data.get("yaw", 0.0)
    initial_pitch = status_data.get("pitch", 0.0)
    
    logger.info(f"Initial pose: ({initial_pos.get('x', 0):.2f}, {initial_pos.get('y', 0):.2f}, {initial_pos.get('z', 0):.2f}), yaw={initial_yaw:.1f}°, pitch={initial_pitch:.1f}°")
    
    # Step 2: Normalize using nav-turn with "forward" direction
    # This will round yaw to nearest cardinal, snap to block center, and set pitch to 0
    turn_result = executor.execute_action_with_log(
        {"type": "nav-turn", "direction": "forward"},
        "init"
    )
    
    if turn_result.get("status") != "success":
        failure_reason = turn_result.get("reason", "unknown")
        return executor._create_uniform_return(
            "failed",
            value=f"Failed to normalize pose with nav-turn: {failure_reason}",
            reason=f"turn_failed_{failure_reason}"
        )
    
    # Step 3: Get normalized status
    normalized_status = executor.execute_action_with_log({"type": "mc-status"}, "init")
    if normalized_status.get("status") != "success":
        return executor._create_uniform_return(
            "failed",
            value="Failed to get normalized status",
            reason="normalized_status_failed"
        )
    
    normalized_data = normalized_status.get("data", {})
    normalized_pos = normalized_data.get("position", {})
    normalized_yaw = normalized_data.get("yaw", 0.0)
    normalized_pitch = normalized_data.get("pitch", 0.0)
    
    # Extract pose components
    pose = {
        "x": normalized_pos.get("x", 0.0),
        "y": normalized_pos.get("y", 0.0),
        "z": normalized_pos.get("z", 0.0),
        "yaw": normalized_yaw
    }
    
    logger.info(f"Normalized pose: ({pose['x']:.2f}, {pose['y']:.2f}, {pose['z']:.2f}), yaw={pose['yaw']:.1f}°, pitch={normalized_pitch:.1f}°")
    
    # Step 4: Initialize nav state in world_state (replace first element or create new list)
    nav_entry = {
        "pose": pose,
        "support_here": "unknown",  # Will be updated by navigation tools
        "fell": False,
        "was_fall": False
    }
    
    # Get existing nav list or initialize empty list
    nav_list = executor.get_world_state("nav")
    if not isinstance(nav_list, list):
        nav_list = []
    
    # Replace first element or set entire list
    if nav_list:
        nav_list[0] = nav_entry
    else:
        nav_list = [nav_entry]
    
    executor.set_world_state("nav", nav_list)
    
    logger.info(f"Initialized nav state: pose=({pose['x']:.2f}, {pose['y']:.2f}, {pose['z']:.2f}), yaw={pose['yaw']:.1f}°")
    
    
    # Return success with normalized pose
    return executor._create_uniform_return(
        "success",
        value=f"Initialized nav state at ({pose['x']:.2f}, {pose['y']:.2f}, {pose['z']:.2f}), yaw={pose['yaw']:.1f}°",
        extra={
            "pose": pose,
            "normalized_from": {
                "x": initial_pos.get("x", 0.0),
                "y": initial_pos.get("y", 0.0),
                "z": initial_pos.get("z", 0.0),
                "yaw": initial_yaw,
                "pitch": initial_pitch
            }
        }
    )


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    print("init tool module loaded")

