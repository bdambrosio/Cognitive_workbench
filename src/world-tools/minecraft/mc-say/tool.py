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
        Dict with acknowledgement (text, format, metadata, char_count).
        Executor will create Note from this content.
    """
    message = kwargs.get("message") or input_value or ""
    
    if not isinstance(message, str) or not message.strip():
        return {"status": "failed", "reason": "message text required (string)"}
    
    minecraft_url = kwargs.get("world_url") or kwargs.get("minecraft_url") or DEFAULT_MINECRAFT_URL
    
    try:
        response = requests.post(
            f"{minecraft_url}/say",
            json={"message": message},
            timeout=10.0
        )
        response.raise_for_status()
        data = response.json()
        
        # Format acknowledgement
        ack_text = f"Message sent: {message}"
        
        return {
            "text": ack_text,
            "format": "text",
            "metadata": data,
            "char_count": len(ack_text)
        }
    except requests.exceptions.RequestException as e:
        logger.error(f"Minecraft say request failed: {e}")
        return {"status": "failed", "reason": f"API request failed: {e}"}


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    result = tool("Hello, world!")
    print(result)

