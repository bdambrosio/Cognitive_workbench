"""
Minecraft dig tool.
Remove a block - world manipulation.
"""

import logging
import os
import requests
from typing import Any, Dict

logger = logging.getLogger(__name__)

# Default Minecraft bot server URL (can be overridden via environment variable or config)
DEFAULT_MINECRAFT_URL = os.getenv("MINECRAFT_URL", "http://localhost:3003")


def tool(input_value=None, **kwargs):
    """
    Dig/remove a block - world manipulation.
    
    Args:
        input_value: ignored
        x: float - absolute x coordinate (if using absolute)
        y: float - absolute y coordinate (if using absolute)
        z: float - absolute z coordinate (if using absolute)
        forward: float - blocks forward (egocentric)
        right: float - blocks right (egocentric)
        up: float - blocks up (egocentric)
        rel_x: float - relative x offset (legacy cartesian)
        rel_y: float - relative y offset (legacy cartesian)
        rel_z: float - relative z offset (legacy cartesian)
        minecraft_url: Optional URL override for Minecraft bot server (default: http://localhost:3003)
        
    Returns:
        Dict with result using uniform return format.
        Note: Digging is asynchronous - the request may be accepted but digging may fail
        due to Minecraft physics (e.g., block too hard, cannot reach, already air).
    """
    executor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}
    
    minecraft_url = kwargs.get("world_url") or kwargs.get("minecraft_url") or DEFAULT_MINECRAFT_URL
    
    # Build position parameters - bot expects pos or rel
    dig_params = {}
    
    # Check for absolute position
    if kwargs.get("x") is not None and kwargs.get("y") is not None and kwargs.get("z") is not None:
        dig_params["pos"] = {
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
        if kwargs.get("down") is not None: rel_params["up"] = -float(kwargs.get("down")) # Helper alias
        if kwargs.get("left") is not None: rel_params["right"] = -float(kwargs.get("left")) # Helper alias

        # Cartesian (legacy/fallback)
        if kwargs.get("rel_x") is not None: rel_params["dx"] = float(kwargs.get("rel_x"))
        if kwargs.get("rel_y") is not None: rel_params["dy"] = float(kwargs.get("rel_y"))
        if kwargs.get("rel_z") is not None: rel_params["dz"] = float(kwargs.get("rel_z"))
        
        dig_params["rel"] = rel_params
    else:
        return executor._create_uniform_return(
            'failed',
            value="position required (absolute x,y,z OR relative forward,right,up)",
            reason="missing_position"
        )
    
    try:
        url = f"{minecraft_url}/act/dig"
        logger.info(f"🔍 mc-dig: POST {url} body={dig_params}")
        # Increased timeout to 60s - digging blocks can take time, especially hard blocks
        # Bridge returns immediately after initiating dig, but some blocks take 10-30+ seconds to break
        response = requests.post(url, json=dig_params, timeout=60.0)
        logger.info(f"📥 mc-dig: Response status={response.status_code}, headers={dict(response.headers)}")
        response.raise_for_status()
        data = response.json()
        logger.debug(f"📥 mc-dig: Response body={data}")
        
        if not data.get("ok"):
            error = data.get("error", "unknown failure")
            result_text = f"Dig failed: {error}"
            return executor._create_uniform_return(
                'failed',
                value=result_text,
                reason="dig_failed"
            )
        
        # Dig request accepted - note that actual digging is asynchronous
        # and may fail due to Minecraft physics (block too hard, cannot reach, etc.)
        dug = data.get("dug", {})
        block_name = dug.get("name", "unknown")
        dug_pos = dug.get("position", {})
        
        if block_name == "air":
            result_text = f"Dig request accepted: target block is already air"
        elif dug_pos:
            pos_str = f"({dug_pos.get('x', 0)}, {dug_pos.get('y', 0)}, {dug_pos.get('z', 0)})"
            result_text = f"Dig request accepted: {block_name} at {pos_str}"
        else:
            result_text = f"Dig request accepted: {block_name}"
        
        # Build structured data dict
        # Extract metadata fields for extra
        extra_metadata = {
            "status": "accepted",  # Request accepted, but digging may still fail asynchronously
            "dug": dug
        }
        # Merge API response data into extra
        extra_metadata.update(data)
        
        return executor._create_uniform_return('success', value=result_text, extra=extra_metadata)
    except requests.exceptions.RequestException as e:
        logger.error(f"Minecraft dig request failed: {e}")
        return executor._create_uniform_return(
            'failed',
            value=f"API request failed: {e}",
            reason="api_failed"
        )


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    result = tool(rel_x=0, rel_y=-1, rel_z=1)
    print(result)

