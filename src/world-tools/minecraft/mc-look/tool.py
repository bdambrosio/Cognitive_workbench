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
        yaw: float - yaw angle in degrees (required)
        pitch: float - pitch angle in degrees (required)
        minecraft_url: Optional URL override for Minecraft bot server (default: http://localhost:3003)
        
    Returns:
        Dict with acknowledgement using uniform return format.
        Executor will create Note from this content.
    """
    executor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}
    
    minecraft_url = kwargs.get("world_url") or kwargs.get("minecraft_url") or DEFAULT_MINECRAFT_URL
    
    yaw = kwargs.get("yaw")
    pitch = kwargs.get("pitch")
    
    if yaw is None or pitch is None:
        return executor._create_uniform_return('failed', reason="yaw and pitch required (degrees)")
    
    url = f"{minecraft_url}/act/look"
    body = {"yaw": float(yaw), "pitch": float(pitch)}
    
    try:
        logger.info(f"🔍 mc-look: POST {url} body={body}")
        response = requests.post(url, json=body, timeout=10.0)
        logger.info(f"📥 mc-look: Response status={response.status_code}, headers={dict(response.headers)}")
        response.raise_for_status()
        data = response.json()
        logger.debug(f"📥 mc-look: Response body={data}")
        
        # Format acknowledgement (yaw and pitch are in degrees)
        ack_text = f"Look command accepted (yaw: {yaw:.2f}°, pitch: {pitch:.2f}°)"
        
        # Build structured data dict
        structured_data = dict(data)
        structured_data["yaw"] = yaw
        structured_data["pitch"] = pitch
        
        return executor._create_uniform_return('success', value=ack_text, data=structured_data)
    except requests.exceptions.RequestException as e:
        logger.error(f"Minecraft look request failed: {e}")
        return executor._create_uniform_return('failed', reason=f"API request failed: {e}")


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    result = tool(yaw=90.0, pitch=0.0)  # 90 degrees = east
    print(result)

