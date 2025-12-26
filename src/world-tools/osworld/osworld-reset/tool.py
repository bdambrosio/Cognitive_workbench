"""
OSWorld reset tool.
Resets the OSWorld environment (soft or hard mode).
"""

import logging
import os
import requests
from typing import Any, Dict

logger = logging.getLogger(__name__)

# Default OSWorld server URL (can be overridden via environment variable or config)
DEFAULT_OSWORLD_URL = os.getenv("OSWORLD_URL", "http://localhost:3002")


def tool(input_value=None, **kwargs):
    """
    Reset OSWorld environment.
    
    Args:
        input_value: mode string ("soft" or "hard", preferred)
        mode: mode string ("soft" or "hard", alternative to input_value)
        osworld_url: Optional URL override for OSWorld server (default: http://localhost:3002)
        
    Returns:
        Dict with reset result (text, format, metadata, char_count).
        Executor will create Note from this content.
    """
    mode = kwargs.get("mode") or input_value or "soft"
    
    if mode not in ("soft", "hard"):
        return {"status": "failed", "reason": f"mode must be 'soft' or 'hard', got: {mode}"}
    
    osworld_url = kwargs.get("world_url") or kwargs.get("osworld_url") or DEFAULT_OSWORLD_URL
    
    try:
        response = requests.post(
            f"{osworld_url}/reset",
            json={"mode": mode},
            timeout=30.0
        )
        response.raise_for_status()
        data = response.json()
        
        # Build result text
        result_text = f"""Reset Result:
Mode: {data.get('mode', mode)}
Success: {data.get('success', False)}
Step Counter: {data.get('step_counter', 0)}"""
        
        if mode == "hard":
            result_text += "\n\nHard reset: DesktopEnv.reset() called - environment state reverted to snapshot"
        else:
            result_text += "\n\nSoft reset: Step counter cleared, no state change"
        
        return {
            "text": result_text,
            "format": "text",
            "metadata": {
                "success": data.get("success"),
                "mode": data.get("mode", mode),
                "step_counter": data.get("step_counter", 0)
            },
            "char_count": len(result_text)
        }
    except requests.exceptions.RequestException as e:
        logger.error(f"OSWorld reset request failed: {e}")
        return {"status": "failed", "reason": f"API request failed: {e}"}


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    result = tool("hard")
    print(result)

