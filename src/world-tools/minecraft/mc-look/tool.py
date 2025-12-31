"""
Minecraft look tool.
Reorient perception - adjust yaw and pitch.
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
    Reorient bot's view - adjust yaw and pitch.
    
    Args:
        input_value: ignored
        yaw: float - yaw angle in radians (required)
        pitch: float - pitch angle in radians (required)
        minecraft_url: Optional URL override for Minecraft bot server (default: http://localhost:3003)
        
    Returns:
        Dict with acknowledgement (text, format, metadata, char_count).
        Executor will create Note from this content.
    """
    minecraft_url = kwargs.get("world_url") or kwargs.get("minecraft_url") or DEFAULT_MINECRAFT_URL
    
    yaw = kwargs.get("yaw")
    pitch = kwargs.get("pitch")
    
    if yaw is None or pitch is None:
        return {"status": "failed", "reason": "yaw and pitch required (radians)"}
    
    url = f"{minecraft_url}/act/look"
    body = {"yaw": float(yaw), "pitch": float(pitch)}
    
    try:
        logger.info(f"🔍 mc-look: POST {url} body={body}")
        response = requests.post(url, json=body, timeout=10.0)
        logger.info(f"📥 mc-look: Response status={response.status_code}, headers={dict(response.headers)}")
        response.raise_for_status()
        data = response.json()
        logger.debug(f"📥 mc-look: Response body={data}")
        
        # Format acknowledgement
        ack_text = f"Look command accepted (yaw: {yaw:.2f}, pitch: {pitch:.2f})"
        
        return {
            "text": ack_text,
            "format": "text",
            "metadata": data,
            "char_count": len(ack_text)
        }
    except requests.exceptions.RequestException as e:
        logger.error(f"Minecraft look request failed: {e}")
        return {"status": "failed", "reason": f"API request failed: {e}"}


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    result = tool(yaw=1.57, pitch=0.0)
    print(result)

