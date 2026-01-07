"""
Minecraft open tool.
Open block-based UI (crafting table, chest, furnace).
"""

import logging
import os
import requests
from typing import Any, Dict

logger = logging.getLogger(__name__)

DEFAULT_MINECRAFT_URL = os.getenv("MINECRAFT_URL", "http://localhost:3003")


def tool(input_value=None, **kwargs):
    """
    Open block-based UI (crafting table, chest, furnace).
    
    Args:
        input_value: ignored
        x: float - absolute x coordinate (if using absolute)
        y: float - absolute y coordinate (if using absolute)
        z: float - absolute z coordinate (if using absolute)
        forward: float - blocks forward (egocentric)
        right: float - blocks right (egocentric)
        up: float - blocks up (egocentric)
        rel_x: float - relative x offset (legacy)
        rel_y: float - relative y offset (legacy)
        rel_z: float - relative z offset (legacy)
        minecraft_url: Optional URL override for Minecraft bot server (default: http://localhost:3003)
        
    Returns:
        Dict with result using uniform return format.
    """
    executor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}
    
    minecraft_url = kwargs.get("world_url") or kwargs.get("minecraft_url") or DEFAULT_MINECRAFT_URL
    
    open_params = {}
    
    if kwargs.get("x") is not None and kwargs.get("y") is not None and kwargs.get("z") is not None:
        open_params["pos"] = {
            "x": float(kwargs.get("x")),
            "y": float(kwargs.get("y")),
            "z": float(kwargs.get("z"))
        }
    # Check for relative position (Egocentric or Cartesian)
    elif (kwargs.get("forward") is not None or kwargs.get("right") is not None or kwargs.get("up") is not None or
          kwargs.get("rel_x") is not None or kwargs.get("rel_y") is not None or kwargs.get("rel_z") is not None):
        
        rel_params = {}
        # Egocentric (preferred)
        if kwargs.get("forward") is not None: rel_params["forward"] = float(kwargs.get("forward"))
        if kwargs.get("right") is not None: rel_params["right"] = float(kwargs.get("right"))
        if kwargs.get("up") is not None: rel_params["up"] = float(kwargs.get("up"))
        if kwargs.get("down") is not None: rel_params["up"] = -float(kwargs.get("down"))
        if kwargs.get("left") is not None: rel_params["right"] = -float(kwargs.get("left"))

        # Cartesian (legacy/fallback)
        if kwargs.get("rel_x") is not None: rel_params["dx"] = float(kwargs.get("rel_x"))
        if kwargs.get("rel_y") is not None: rel_params["dy"] = float(kwargs.get("rel_y"))
        if kwargs.get("rel_z") is not None: rel_params["dz"] = float(kwargs.get("rel_z"))
        
        open_params["rel"] = rel_params
    else:
        return executor._create_uniform_return(
            'failed',
            value="position required (absolute x,y,z OR relative forward,right,up)",
            reason="missing_position"
        )
    
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
    result = tool(rel_x=0, rel_y=0, rel_z=1)
    print(result)

