"""
Minecraft status tool.
Fast heartbeat + sanity check for bot connection and state.
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
    Get Minecraft bot status - fast heartbeat + sanity check.
    
    Args:
        input_value: ignored
        minecraft_url: Optional URL override for Minecraft bot server (default: http://localhost:3003)
        
    Returns:
        Dict with status information (text, format, metadata, char_count).
        Executor will create Note from this content.
    """
    minecraft_url = kwargs.get("world_url") or kwargs.get("minecraft_url") or DEFAULT_MINECRAFT_URL
    
    try:
        response = requests.get(f"{minecraft_url}/status", timeout=10.0)
        response.raise_for_status()
        data = response.json()
        
        # Format status as readable text
        status_parts = []
        status_parts.append("Minecraft Bot Status:")
        status_parts.append(f"Connected: {data.get('connected', False)}")
        
        position = data.get('position')
        if position:
            if isinstance(position, dict):
                status_parts.append(f"Position: ({position.get('x', 0):.2f}, {position.get('y', 0):.2f}, {position.get('z', 0):.2f})")
            elif isinstance(position, (list, tuple)) and len(position) >= 3:
                status_parts.append(f"Position: ({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f})")
        
        orientation = data.get('orientation')
        if orientation:
            if isinstance(orientation, dict):
                status_parts.append(f"Yaw: {orientation.get('yaw', 0):.2f}, Pitch: {orientation.get('pitch', 0):.2f}")
            elif isinstance(orientation, (list, tuple)) and len(orientation) >= 2:
                status_parts.append(f"Yaw: {orientation[0]:.2f}, Pitch: {orientation[1]:.2f}")
        
        health = data.get('health')
        if health is not None and isinstance(health, (int, float)):
            status_parts.append(f"Health: {health}/20")
        
        food = data.get('food')
        if food is not None and isinstance(food, (int, float)):
            status_parts.append(f"Food: {food}/20")
        
        action = data.get('action', {})
        if isinstance(action, dict) and action.get('type'):
            action_type = action.get('type', 'idle')
            action_note = action.get('note', '')
            if action_note:
                status_parts.append(f"Current Action: {action_type} ({action_note})")
            else:
                status_parts.append(f"Current Action: {action_type}")
        else:
            status_parts.append("Current Action: idle")
        
        status_text = "\n".join(status_parts)
        
        return {
            "text": status_text,
            "format": "text",
            "metadata": data,
            "char_count": len(status_text)
        }
    except requests.exceptions.RequestException as e:
        logger.error(f"Minecraft status request failed: {e}")
        return {"status": "failed", "reason": f"API request failed: {e}"}


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    result = tool()
    print(result)

