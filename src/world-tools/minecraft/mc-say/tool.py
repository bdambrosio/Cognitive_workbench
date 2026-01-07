"""
Minecraft say tool.
Social / debugging / alignment communication.
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
    Send chat message in Minecraft - social/debugging/alignment.
    
    Args:
        input_value: Message text to send (preferred)
        message: Message text to send (alternative to input_value)
        minecraft_url: Optional URL override for Minecraft bot server (default: http://localhost:3003)
        
    Returns:
        Dict with result using uniform return format.
    """
    executor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}
    
    message = kwargs.get("message") or input_value or ""
    
    if not isinstance(message, str) or not message.strip():
        return executor._create_uniform_return(
            'failed',
            value="message text required (string)",
            reason="missing_message"
        )
    
    minecraft_url = kwargs.get("world_url") or kwargs.get("minecraft_url") or DEFAULT_MINECRAFT_URL
    
    try:
        url = f"{minecraft_url}/say"
        logger.debug(f"🔍 mc-say: POST {url} body={{\"message\": \"{message[:50]}...\"}}")
        response = requests.post(url, json={"message": message}, timeout=10.0)
        logger.debug(f"📥 mc-say: Response status={response.status_code}")
        response.raise_for_status()
        data = response.json()
        
        # Format acknowledgement
        ack_text = f"Message sent: {message}"
        
        # Build structured data dict
        # Extract metadata fields for extra
        extra_metadata = {
            "message": message
        }
        # Merge API response data into extra
        extra_metadata.update(data)
        
        return executor._create_uniform_return('success', value=ack_text, extra=extra_metadata)
    except requests.exceptions.RequestException as e:
        logger.error(f"Minecraft say request failed: {e}")
        return executor._create_uniform_return(
            'failed',
            value=f"API request failed: {e}",
            reason="api_failed"
        )


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    result = tool("Hello, world!")
    print(result)

