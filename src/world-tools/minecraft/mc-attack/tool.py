"""
Minecraft attack tool.
Left-click style interaction (entities or blocks).
"""

import logging
import os
import requests
from typing import Any, Dict

logger = logging.getLogger(__name__)

DEFAULT_MINECRAFT_URL = os.getenv("MINECRAFT_URL", "http://localhost:3003")


def tool(input_value=None, **kwargs):
    """
    Left-click style interaction (entities or blocks).
    
    Args:
        input_value: ignored
        target: dict with either:
            - entity_id: string (for entity attack)
            - forward, right, up: floats (egocentric block attack)
            - rel_x, rel_y, rel_z: floats (legacy cartesian block attack)
        forward: float - blocks forward (alternative top-level arg)
        right: float - blocks right (alternative top-level arg)
        up: float - blocks up (alternative top-level arg)
        minecraft_url: Optional URL override for Minecraft bot server (default: http://localhost:3003)
        
    Returns:
        Dict with result (text, format, metadata, char_count).
        Executor will create Note from this content.
    """
    minecraft_url = kwargs.get("world_url") or kwargs.get("minecraft_url") or DEFAULT_MINECRAFT_URL
    
    target = kwargs.get("target")
    if not isinstance(target, dict):
        return {"status": "failed", "reason": "target required (dict with entity_id or rel_x/rel_y/rel_z)"}
    
    attack_params = {}
    
    if "entity_id" in target:
        attack_params["entity_id"] = target["entity_id"]
    elif (kwargs.get("forward") is not None or kwargs.get("right") is not None or kwargs.get("up") is not None or
          "rel_x" in target or "rel_y" in target or "rel_z" in target):
        
        rel_params = {}
        # Egocentric (preferred)
        if kwargs.get("forward") is not None: rel_params["forward"] = float(kwargs.get("forward"))
        if kwargs.get("right") is not None: rel_params["right"] = float(kwargs.get("right"))
        if kwargs.get("up") is not None: rel_params["up"] = float(kwargs.get("up"))
        if kwargs.get("down") is not None: rel_params["up"] = -float(kwargs.get("down"))
        if kwargs.get("left") is not None: rel_params["right"] = -float(kwargs.get("left"))
        
        # Also check inside 'target' dict for direct keys if passed that way
        if "forward" in target: rel_params["forward"] = float(target["forward"])
        if "right" in target: rel_params["right"] = float(target["right"])
        if "up" in target: rel_params["up"] = float(target["up"])

        # Cartesian (legacy/fallback)
        if "rel_x" in target: rel_params["dx"] = float(target["rel_x"])
        if "rel_y" in target: rel_params["dy"] = float(target["rel_y"])
        if "rel_z" in target: rel_params["dz"] = float(target["rel_z"])

        attack_params["rel"] = rel_params
    else:
        return {"status": "failed", "reason": "target must have either entity_id, absolute pos, or relative (forward,right,up)"}
    
    try:
        response = requests.post(
            f"{minecraft_url}/act/attack",
            json=attack_params,
            timeout=10.0
        )
        response.raise_for_status()
        data = response.json()
        
        if not data.get("ok"):
            error = data.get("error", "unknown failure")
            result_text = f"Attack failed: {error}"
        else:
            result_text = "Attack successful"
        
        return {
            "text": result_text,
            "format": "text",
            "metadata": {
                "success": data.get("ok", False),
                **data
            },
            "char_count": len(result_text)
        }
    except requests.exceptions.RequestException as e:
        logger.error(f"Minecraft attack request failed: {e}")
        return {"status": "failed", "reason": f"API request failed: {e}"}


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    result = tool(target={"rel_x": 0, "rel_y": 0, "rel_z": 1})
    print(result)

