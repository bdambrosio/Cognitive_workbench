"""
OSWorld observe tool.
Gets the current observation from the OSWorld environment (screenshot and accessibility tree).
"""

import logging
import os
import requests
from typing import Any, Dict, Optional

logger = logging.getLogger(__name__)

# Default OSWorld server URL (can be overridden via environment variable or config)
DEFAULT_OSWORLD_URL = os.getenv("OSWORLD_URL", "http://localhost:3002")


def tool(input_value=None, **kwargs):
    """
    Get current observation from OSWorld environment.
    
    Args:
        input_value: ignored
        include_screenshot: bool (default: True) - include screenshot in observation
        include_a11y: bool (default: True) - include accessibility tree in observation
        osworld_url: Optional URL override for OSWorld server (default: http://localhost:3002)
        
    Returns:
        Dict with observation content (text, format, metadata, char_count).
        Executor will create Note from this content.
    """
    osworld_url = kwargs.get("world_url") or kwargs.get("osworld_url") or DEFAULT_OSWORLD_URL
    include_screenshot = kwargs.get("include_screenshot", True)
    include_a11y = kwargs.get("include_a11y", True)
    
    try:
        response = requests.post(
            f"{osworld_url}/observe",
            json={
                "include_screenshot": include_screenshot,
                "include_a11y": include_a11y
            },
            timeout=30.0
        )
        response.raise_for_status()
        data = response.json()
        
        # Build observation text
        obs_parts = []
        obs_parts.append(f"Observation #{data.get('observation_counter', data.get('step_counter', 0))}")
        obs_parts.append(f"Timestamp: {data.get('timestamp', 0)}")
        obs_parts.append(f"Step Counter: {data.get('step_counter', 0)}")
        
        observation = data.get("observation", {})
        if observation.get("screenshot"):
            obs_parts.append("Screenshot: available (base64 PNG)")
        if observation.get("accessibility_tree"):
            a11y = observation["accessibility_tree"]
            if isinstance(a11y, dict):
                obs_parts.append(f"Accessibility Tree: {len(str(a11y))} characters")
            else:
                obs_parts.append(f"Accessibility Tree: available")
        
        obs_text = "\n".join(obs_parts)
        
        return {
            "text": obs_text,
            "format": "json",
            "metadata": {
                "timestamp": data.get("timestamp"),
                "step_counter": data.get("step_counter"),
                "observation": observation
            },
            "char_count": len(obs_text)
        }
    except requests.exceptions.RequestException as e:
        logger.error(f"OSWorld observe request failed: {e}")
        return {"status": "failed", "reason": f"API request failed: {e}"}


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    result = tool()
    print(result)

