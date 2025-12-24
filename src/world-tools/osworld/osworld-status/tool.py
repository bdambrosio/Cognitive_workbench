"""
OSWorld status tool.
Gets the current status of the OSWorld server.
"""

import logging
import os
import requests
from typing import Any, Dict

logger = logging.getLogger(__name__)

# Default OSWorld server URL (can be overridden via environment variable or config)
DEFAULT_OSWORLD_URL = os.getenv("OSWORLD_URL", "http://localhost:3002")


def tool(value=None, **kwargs):
    """
    Get OSWorld server status.
    
    Args:
        value: ignored
        osworld_url: Optional URL override for OSWorld server (default: http://localhost:3002)
        
    Returns:
        Dict with status information (text, format, metadata, char_count).
        Executor will create Note from this content.
    """
    osworld_url = kwargs.get("world_url") or kwargs.get("osworld_url") or DEFAULT_OSWORLD_URL
    
    try:
        response = requests.get(f"{osworld_url}/status", timeout=10.0)
        response.raise_for_status()
        data = response.json()
        
        # Format status as readable text
        status_text = f"""OSWorld Server Status:
Ready: {data.get('ready', False)}
Uptime: {data.get('uptime_sec', 0)} seconds
Step Counter: {data.get('step_counter', 0)}
Provider: {data.get('provider', 'unknown')}
Headless: {data.get('headless', False)}
Screen: {data.get('screen', {}).get('width', 0)}x{data.get('screen', {}).get('height', 0)}"""
        
        return {
            "text": status_text,
            "format": "text",
            "metadata": data,
            "char_count": len(status_text)
        }
    except requests.exceptions.RequestException as e:
        logger.error(f"OSWorld status request failed: {e}")
        return {"status": "failed", "reason": f"API request failed: {e}"}


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    result = tool()
    print(result)

