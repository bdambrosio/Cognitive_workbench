"""
Minecraft respawn tool.
Reset embodiment after death - lifecycle/recovery.
"""

import logging
import os
import requests
import time
from typing import Any, Dict
from infospace_executor import InfospaceExecutor

logger = logging.getLogger(__name__)

# Default Minecraft bot server URL (can be overridden via environment variable or config)
DEFAULT_MINECRAFT_URL = os.getenv("MINECRAFT_URL", "http://localhost:3003")
MAX_NAV_HISTORY = 100


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
    Respawn bot after death - reset embodiment.
    
    Args:
        input_value: ignored
        minecraft_url: Optional URL override for Minecraft bot server (default: http://localhost:3003)
        
    Returns:
        Dict with result using uniform return format.
    """
    executor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}
    
    minecraft_url = kwargs.get("world_url") or kwargs.get("minecraft_url") or DEFAULT_MINECRAFT_URL
    
    try:
        url = f"{minecraft_url}/act/respawn"
        logger.debug(f"🔍 mc-respawn: POST {url}")
        response = requests.post(url, json={}, timeout=10.0)
        logger.debug(f"📥 mc-respawn: Response status={response.status_code}")
        response.raise_for_status()
        data = response.json()
        
        # Get new position after respawn
        status_result = executor.execute_action_with_log({"type": "mc-status"}, "mc-respawn")
        if status_result.get("status") == "success":
            status_data = status_result.get("data", {})
            position = status_data.get("position", {})
            yaw = status_data.get("yaw", 0.0)
            
            # Update nav state (respawn changes position)
            pose = {
                "x": position.get("x", 0.0),
                "y": position.get("y", 0.0),
                "z": position.get("z", 0.0),
                "yaw": yaw
            }
            _update_nav_state(executor, pose, "unknown", fell=False, was_fall=False)
        
        # Format acknowledgement
        ack_text = "Respawn command accepted - bot reset"
        
        # Pass API response data as value (stored in Note)
        return executor._create_uniform_return('success', value=data)
    except requests.exceptions.RequestException as e:
        logger.error(f"Minecraft respawn request failed: {e}")
        return executor._create_uniform_return(
            'failed',
            value=f"API request failed: {e}",
            reason="api_failed"
        )


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    result = tool()
    print(result)

