"""
Minecraft place tool.
Build / modify world - place blocks.
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
    Place a block - build/modify world.
    
    Args:
        input_value: Item/block name to place (preferred)
        item: Item/block name to place (alternative to input_value)
        x: float - absolute x coordinate of reference block (if using absolute)
        y: float - absolute y coordinate of reference block (if using absolute)
        z: float - absolute z coordinate of reference block (if using absolute)
        forward: float - blocks forward to ref block (egocentric)
        right: float - blocks right to ref block (egocentric)
        up: float - blocks up to ref block (egocentric)
        rel_x: float - relative x offset to reference block (legacy)
        rel_y: float - relative y offset to reference block (legacy)
        rel_z: float - relative z offset to reference block (legacy)
        face: string - face of reference block to place against (e.g., "top", "bottom", "north", "south", "east", "west")
        minecraft_url: Optional URL override for Minecraft bot server (default: http://localhost:3003)
        
    Returns:
        Dict with result using uniform return format.
        Note: Placement is asynchronous - the request may be accepted but placement may fail
        due to Minecraft physics (e.g., block already exists, insufficient space, item not in inventory).
    """
    executor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}
    
    minecraft_url = kwargs.get("world_url") or kwargs.get("minecraft_url") or DEFAULT_MINECRAFT_URL
    
    item = kwargs.get("item") or input_value or ""
    if not isinstance(item, str) or not item.strip():
        return executor._create_uniform_return('failed', reason="item/block name required (string)")
    
    # Build position parameters - bot expects ref.pos or ref.rel
    place_params = {"item": item}
    
    ref = {}
    # Check for absolute position
    if kwargs.get("x") is not None and kwargs.get("y") is not None and kwargs.get("z") is not None:
        ref["pos"] = {
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
        
        ref["rel"] = rel_params
    
    if not ref:
        return executor._create_uniform_return('failed', reason="reference block position required (absolute x,y,z OR relative forward,right,up)")
    
    place_params["ref"] = ref
    
    # Face is required (convert "top"/"bottom" to "up"/"down" if needed)
    face = kwargs.get("face")
    if not face:
        return executor._create_uniform_return('failed', reason="face required (top, bottom, north, south, east, west)")
    if face == "top":
        face = "up"
    elif face == "bottom":
        face = "down"
    place_params["face"] = face
    
    try:
        url = f"{minecraft_url}/act/place"
        logger.info(f"🔍 mc-place: POST {url} body={place_params}")
        response = requests.post(url, json=place_params, timeout=30.0)
        logger.info(f"📥 mc-place: Response status={response.status_code}, headers={dict(response.headers)}")
        response.raise_for_status()
        data = response.json()
        logger.debug(f"📥 mc-place: Response body={data}")
        
        if not data.get("ok"):
            error = data.get("error", "unknown failure")
            error_code = data.get("error_code", "unknown_error")
            result_text = f"Place failed: {error}"
            return executor._create_uniform_return('failed', reason=result_text, data={
                "error": error,
                "error_code": error_code,
                **data
            })
        
        # Placement request accepted - note that actual placement is asynchronous
        # and may fail due to Minecraft physics (block already exists, insufficient space, etc.)
        placed_info = data.get("placed", {})
        placed_pos = placed_info.get("position", {})
        
        if placed_pos:
            pos_str = f"({placed_pos.get('x', 0)}, {placed_pos.get('y', 0)}, {placed_pos.get('z', 0)})"
            result_text = f"Place request accepted: {item} at {pos_str} (face={face})"
        else:
            result_text = f"Place request accepted: {item} (face={face})"
        
        # Build structured data dict
        structured_data = dict(data)
        structured_data["status"] = "accepted"  # Request accepted, but placement may still fail asynchronously
        structured_data["placed"] = placed_info
        
        return executor._create_uniform_return('success', value=result_text, data=structured_data)
    except requests.exceptions.RequestException as e:
        logger.error(f"Minecraft place request failed: {e}")
        return executor._create_uniform_return('failed', reason=f"API request failed: {e}")


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    result = tool("dirt", rel_x=0, rel_y=0, rel_z=1, face="north")
    print(result)

