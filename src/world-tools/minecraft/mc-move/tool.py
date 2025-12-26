"""
Minecraft move tool.
Locomotion - non-blocking action.
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
    Initiate movement - non-blocking locomotion.
    
    Args:
        input_value: ignored
        forward: bool - move forward (default: False)
        back: bool - move backward (default: False)
        left: bool - strafe left (default: False)
        right: bool - strafe right (default: False)
        jump: bool - jump while moving (default: False)
        sprint: bool - sprint while moving (default: False)
        duration: float - movement duration in seconds (bounded, default: server default)
        check_collision: bool - stop on collision (default: True)
        minecraft_url: Optional URL override for Minecraft bot server (default: http://localhost:3003)
        
    Returns:
        Dict with result (text, format, metadata, char_count).
        Executor will create Note from this content.
    """
    minecraft_url = kwargs.get("world_url") or kwargs.get("minecraft_url") or DEFAULT_MINECRAFT_URL
    
    # Build movement parameters
    move_params = {}
    if kwargs.get("forward"):
        move_params["forward"] = True
    if kwargs.get("back"):
        move_params["back"] = True
    if kwargs.get("left"):
        move_params["left"] = True
    if kwargs.get("right"):
        move_params["right"] = True
    if kwargs.get("jump"):
        move_params["jump"] = True
    if kwargs.get("sprint"):
        move_params["sprint"] = True
    if kwargs.get("check_collision") is False:
        move_params["check_collision"] = False

    if kwargs.get("duration") is not None:
        # Convert seconds to milliseconds
        move_params["duration_ms"] = int(float(kwargs.get("duration")) * 1000)
    
    if not move_params:
        return {"status": "failed", "reason": "at least one direction flag required"}
    
    try:
        # Increase timeout significantly as the server now blocks until movement completes (or collides)
        timeout_seconds = float(kwargs.get("duration", 0)) + 5.0
        response = requests.post(
            f"{minecraft_url}/act/move",
            json=move_params,
            timeout=max(10.0, timeout_seconds)
        )
        response.raise_for_status()
        data = response.json()
        
        if not data.get("ok"):
            return {"status": "failed", "reason": data.get("error", "unknown error")}
        
        # Parse result
        status = data.get("status", "unknown")
        actual_duration = data.get("actual_duration_ms", 0) / 1000.0
        
        if status == "collision":
            reason = data.get("reason", "unknown obstruction")
            result_text = f"Movement stopped by collision after {actual_duration:.2f}s: {reason}"
        else:
            result_text = f"Movement completed successfully ({actual_duration:.2f}s)"
        
        return {
            "text": result_text,
            "format": "text",
            "metadata": {
                "status": status,
                "duration_seconds": actual_duration,
                "final_position": data.get("final_position"),
                **data
            },
            "char_count": len(result_text)
        }
    except requests.exceptions.RequestException as e:
        logger.error(f"Minecraft move request failed: {e}")
        return {"status": "failed", "reason": f"API request failed: {e}"}


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    result = tool(forward=True, duration=2.0)
    print(result)

