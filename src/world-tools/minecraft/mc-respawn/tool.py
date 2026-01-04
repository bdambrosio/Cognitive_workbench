"""
Minecraft respawn tool.
Reset embodiment after death - lifecycle/recovery.
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
        
        # Format acknowledgement
        ack_text = "Respawn command accepted - bot reset"
        
        # Build structured data dict
        structured_data = dict(data)
        
        return executor._create_uniform_return('success', value=ack_text, data=structured_data)
    except requests.exceptions.RequestException as e:
        logger.error(f"Minecraft respawn request failed: {e}")
        return executor._create_uniform_return('failed', reason=f"API request failed: {e}")


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    result = tool()
    print(result)

