"""
Minecraft open tool.
Open block-based UI (crafting table, chest, furnace).
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
    Open block-based UI (crafting table, chest, furnace).
    
    Args:
        input_value: ignored
        dx: float - world-relative X offset from agent (positive = east, negative = west)
        dy: float - world-relative Y offset from agent (positive = up, negative = down)
        dz: float - world-relative Z offset from agent (positive = south, negative = north)
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
            value="position required (dx, dy, dz - world-relative offsets from agent)",
            reason="missing_position"
        )
    
    dx = float(dx)
    dy = float(dy)
    dz = float(dz)
    
    # Get agent position to convert dx,dy,dz to absolute
    try:
        status_result = executor.execute_action_with_log({"type": "mc-status"}, "mc-open")
        if status_result.get("status") != "success":
            return executor._create_uniform_return(
                'failed',
                value="Failed to get agent position for coordinate conversion",
                reason="status_failed"
            )
        agent_pos = status_result.get("data", {}).get("position", {})
        if not isinstance(agent_pos, dict):
            return executor._create_uniform_return(
                'failed',
                value="Invalid agent position data",
                reason="invalid_position"
            )
    except Exception as e:
        logger.error(f"mc-open: Failed to get agent position: {e}")
        return executor._create_uniform_return(
            'failed',
            value=f"Failed to get agent position: {e}",
            reason="status_failed"
        )
    
    # Convert dx,dy,dz to absolute block coordinates
    abs_x, abs_y, abs_z = dx_dy_dz_to_absolute(dx, dy, dz, agent_pos)
    
    # Build position parameters - pass absolute to bridge
    open_params = {
        "pos": {
            "x": float(abs_x),
            "y": float(abs_y),
            "z": float(abs_z)
        }
    }
    
    try:
        url = f"{minecraft_url}/act/open"
        logger.debug(f"🔍 mc-open: POST {url} body={open_params}")
        response = requests.post(url, json=open_params, timeout=10.0)
        logger.debug(f"📥 mc-open: Response status={response.status_code}")
        response.raise_for_status()
        data = response.json()
        
        if not data.get("ok"):
            error = data.get("error", "unknown failure")
            result_text = f"Open failed: {error}"
            return executor._create_uniform_return(
                'failed',
                value=result_text,
                reason="open_failed"
            )
        
        ui_type = data.get("ui_type", "unknown")
        result_text = f"Opened {ui_type}"
        
        # Build structured data dict
        # Pass API response data as value (stored in Note)
        return executor._create_uniform_return('success', value=data)
    except requests.exceptions.RequestException as e:
        logger.error(f"Minecraft open request failed: {e}")
        return executor._create_uniform_return(
            'failed',
            value=f"API request failed: {e}",
            reason="api_failed"
        )


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    result = tool(dx=0, dy=0, dz=1)
    print(result)

