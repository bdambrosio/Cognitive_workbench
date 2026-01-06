"""
Minecraft close tool.
Close currently open UI.
"""

import logging
import os
import requests
from typing import Any, Dict

logger = logging.getLogger(__name__)

DEFAULT_MINECRAFT_URL = os.getenv("MINECRAFT_URL", "http://localhost:3003")


def tool(input_value=None, **kwargs):
    """
    Close currently open UI.
    
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
        url = f"{minecraft_url}/act/close"
        logger.debug(f"🔍 mc-close: POST {url}")
        response = requests.post(url, json={}, timeout=10.0)
        logger.debug(f"📥 mc-close: Response status={response.status_code}")
        response.raise_for_status()
        data = response.json()
        
        if not data.get("ok"):
            error = data.get("error", "unknown failure")
            result_text = f"Close failed: {error}"
            return executor._create_uniform_return(
                'failed',
                value=result_text,
                data={
                    "success": False,
                    "failure_reason": "close_failed",
                    "error": error,
                    **data
                }
            )
        
        result_text = "UI closed"
        
        # Build structured data dict
        structured_data = dict(data)
        structured_data["success"] = True
        
        return executor._create_uniform_return('success', value=result_text, data=structured_data)
    except requests.exceptions.RequestException as e:
        logger.error(f"Minecraft close request failed: {e}")
        return executor._create_uniform_return(
            'failed',
            value=f"API request failed: {e}",
            data={"success": False, "failure_reason": "api_failed"}
        )


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    result = tool()
    print(result)

