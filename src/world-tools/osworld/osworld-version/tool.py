"""
OSWorld version tool.
Gets the API version information from the OSWorld server.
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
    Get OSWorld server version information.
    
    Args:
        value: ignored
        osworld_url: Optional URL override for OSWorld server (default: http://localhost:3002)
        
    Returns:
        Dict with version information (text, format, metadata, char_count).
        Executor will create Note from this content.
    """
    osworld_url = kwargs.get("world_url") or kwargs.get("osworld_url") or DEFAULT_OSWORLD_URL
    
    try:
        response = requests.get(f"{osworld_url}/version", timeout=10.0)
        response.raise_for_status()
        data = response.json()
        
        # Format version as readable text
        version_text = f"""OSWorld API Version:
API Version: {data.get('api_version', 'unknown')}
Server Version: {data.get('server_version', 'unknown')}
Protocol: {data.get('protocol', 'unknown')}"""
        
        return {
            "text": version_text,
            "format": "text",
            "metadata": data,
            "char_count": len(version_text)
        }
    except requests.exceptions.RequestException as e:
        logger.error(f"OSWorld version request failed: {e}")
        return {"status": "failed", "reason": f"API request failed: {e}"}


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    result = tool()
    print(result)

