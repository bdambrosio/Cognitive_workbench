"""
Minecraft use tool.
Right-click style interaction using equipped item.
"""

import logging
import os
import requests
from typing import Any, Dict

logger = logging.getLogger(__name__)

DEFAULT_MINECRAFT_URL = os.getenv("MINECRAFT_URL", "http://localhost:3003")


def tool(input_value=None, **kwargs):
    """
    Right-click style interaction using equipped item.
    
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
    
    use_params = {}
    
    if kwargs.get("x") is not None and kwargs.get("y") is not None and kwargs.get("z") is not None:
        use_params["pos"] = {
            "x": float(kwargs.get("x")),
            "y": float(kwargs.get("y")),
            "z": float(kwargs.get("z"))
        }
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

        use_params["rel"] = rel_params
    else:
        return executor._create_uniform_return('failed', reason="position required (absolute x,y,z OR relative forward,right,up)")
    
    try:
        url = f"{minecraft_url}/act/use"
        logger.debug(f"🔍 mc-use: POST {url} body={use_params}")
        response = requests.post(url, json=use_params, timeout=10.0)
        logger.debug(f"📥 mc-use: Response status={response.status_code}")
        response.raise_for_status()
        data = response.json()
        
        if not data.get("ok"):
            error = data.get("error", "unknown failure")
            error_code = data.get("error_code", "unknown_error")
            result_text = f"Use failed: {error}"
            return executor._create_uniform_return('failed', reason=result_text, data={
                "error": error,
                "error_code": error_code,
                **data
            })
        
        result_text = "Use interaction successful"
        
        # Build structured data dict
        structured_data = dict(data)
        
        return executor._create_uniform_return('success', value=result_text, data=structured_data)
    except requests.exceptions.RequestException as e:
        logger.error(f"Minecraft use request failed: {e}")
        return executor._create_uniform_return('failed', reason=f"API request failed: {e}")


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    result = tool(rel_x=0, rel_y=-1, rel_z=0)
    print(result)

