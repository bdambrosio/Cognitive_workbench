"""
Minecraft use tool.
Right-click style interaction using equipped item.
"""

import logging
import os
import requests
import sys
from pathlib import Path
from typing import Any, Dict

logger = logging.getLogger(__name__)

# Import nav_core helper for coordinate conversion
_THIS_DIR = os.path.dirname(__file__)
_NAV_CORE_DIR = os.path.abspath(os.path.join(_THIS_DIR, ".."))
if _NAV_CORE_DIR not in sys.path:
    sys.path.insert(0, _NAV_CORE_DIR)
from nav_core import dx_dy_dz_to_absolute

DEFAULT_MINECRAFT_URL = os.getenv("MINECRAFT_URL", "http://localhost:3003")


def tool(input_value=None, **kwargs):
    """
    Right-click style interaction using equipped item.
    
    Args:
        input_value: ignored
        dx: float - agent-relative X offset from agent (positive = right, negative = left)
        dy: float - agent-relative Y offset from agent (positive = up, negative = down)
        dz: float - agent-relative Z offset from agent (positive = forward, negative = back)
        minecraft_url: Optional URL override for Minecraft bot server (default: http://localhost:3003)
        
    Returns:
        Dict with result using uniform return format.
    """
    executor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}
    
    minecraft_url = kwargs.get("world_url") or kwargs.get("minecraft_url") or DEFAULT_MINECRAFT_URL
    
    # Get dx, dy, dz (world-relative coordinates, origin at agent)
    dx = kwargs.get("dx")
    dy = kwargs.get("dy")
    dz = kwargs.get("dz")
    
    if dx is None or dy is None or dz is None:
        return executor._create_uniform_return(
            'failed',
            value="position required (dx, dy, dz - agent-relative offsets from agent)",
            reason="missing_position"
        )
    
    dx = float(dx)
    dy = float(dy)
    dz = float(dz)
    
    # Get agent position to convert dx,dy,dz to absolute
    try:
        status_result = executor.execute_action_with_log({"type": "mc-status"}, "mc-use")
        if status_result.get("status") != "success":
            return executor._create_uniform_return(
                'failed',
                value="Failed to get agent position for coordinate conversion",
                reason="status_failed"
            )
        status_data = status_result.get("data", {})
        agent_pos = status_data.get("position", {})
        agent_yaw = status_data.get("yaw", 0.0)
        if not isinstance(agent_pos, dict):
            return executor._create_uniform_return(
                'failed',
                value="Invalid agent position data",
                reason="invalid_position"
            )
    except Exception as e:
        logger.error(f"mc-use: Failed to get agent position: {e}")
        return executor._create_uniform_return(
            'failed',
            value=f"Failed to get agent position: {e}",
            reason="status_failed"
        )
    
    # Convert agent-relative dx,dy,dz to absolute block coordinates
    abs_x, abs_y, abs_z = dx_dy_dz_to_absolute(dx, dy, dz, agent_pos, yaw=agent_yaw)
    
    # Build position parameters - pass absolute to bridge
    use_params = {
        "pos": {
            "x": float(abs_x),
            "y": float(abs_y),
            "z": float(abs_z)
        }
    }
    
    try:
        url = f"{minecraft_url}/act/use"
        logger.debug(f"🔍 mc-use: POST {url} body={use_params}")
        response = requests.post(url, json=use_params, timeout=10.0)
        logger.debug(f"📥 mc-use: Response status={response.status_code}")
        response.raise_for_status()
        data = response.json()
        
        if not data.get("ok"):
            error = data.get("error", "unknown failure")
            result_text = f"Use failed: {error}"
            return executor._create_uniform_return(
                'failed',
                value=result_text,
                reason="use_failed"
            )
        
        result_text = "Use interaction successful"
        
        # Pass API response data as value (stored in Note)
        return executor._create_uniform_return('success', value=data)
    except requests.exceptions.RequestException as e:
        logger.error(f"Minecraft use request failed: {e}")
        return executor._create_uniform_return(
            'failed',
            value=f"API request failed: {e}",
            reason="api_failed"
        )


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    result = tool(dx=0, dy=-1, dz=0)
    print(result)

