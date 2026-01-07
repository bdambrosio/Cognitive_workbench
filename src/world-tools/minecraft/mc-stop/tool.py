"""
Minecraft stop tool.
Cancel embodied motion - critical for safety, interrupts, reflection pauses.
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
    Stop all movement - cancel embodied motion.
    
    Args:
        input_value: ignored
        minecraft_url: Optional URL override for Minecraft bot server (default: http://localhost:3003)
        
    Returns:
        Dict with acknowledgement using uniform return format.
        Executor will create Note from this content.
    """
    executor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}
    
    minecraft_url = kwargs.get("world_url") or kwargs.get("minecraft_url") or DEFAULT_MINECRAFT_URL
    
    try:
        response = requests.post(
            f"{minecraft_url}/act/stop",
            json={},
            timeout=10.0
        )
        response.raise_for_status()
        data = response.json()
        
        # Format acknowledgement
        ack_text = "Stop command accepted - all movement cancelled"
        
        # Pass API response data as value (stored in Note)
        return executor._create_uniform_return('success', value=data)
    except requests.exceptions.RequestException as e:
        logger.error(f"Minecraft stop request failed: {e}")
        return executor._create_uniform_return(
            'failed',
            value=f"API request failed: {e}",
            reason="api_failed"
        )


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    result = tool()
    print(result)

