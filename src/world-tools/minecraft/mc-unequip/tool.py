"""
Minecraft unequip tool.
Clear or swap equipped item.
"""

import logging
import os
import requests
from typing import Any, Dict

logger = logging.getLogger(__name__)

DEFAULT_MINECRAFT_URL = os.getenv("MINECRAFT_URL", "http://localhost:3003")


def tool(input_value=None, **kwargs):
    """
    Clear or swap equipped item.
    
    Args:
        input_value: ignored
        slot: "hand" or "offhand" (default: "hand")
        minecraft_url: Optional URL override for Minecraft bot server (default: http://localhost:3003)
        
    Returns:
        Dict with result (text, format, metadata, char_count).
        Executor will create Note from this content.
    """
    minecraft_url = kwargs.get("world_url") or kwargs.get("minecraft_url") or DEFAULT_MINECRAFT_URL
    
    slot = kwargs.get("slot", "hand")
    if slot not in ["hand", "offhand"]:
        return {"status": "failed", "reason": "slot must be 'hand' or 'offhand'"}
    
    unequip_params = {"slot": slot}
    
    try:
        response = requests.post(
            f"{minecraft_url}/act/unequip",
            json=unequip_params,
            timeout=10.0
        )
        response.raise_for_status()
        data = response.json()
        
        if not data.get("ok"):
            error = data.get("error", "unknown failure")
            result_text = f"Unequip failed: {error}"
        else:
            result_text = f"Unequipped {slot}"
        
        return {
            "text": result_text,
            "format": "text",
            "metadata": {
                "success": data.get("ok", False),
                "slot": slot,
                **data
            },
            "char_count": len(result_text)
        }
    except requests.exceptions.RequestException as e:
        logger.error(f"Minecraft unequip request failed: {e}")
        return {"status": "failed", "reason": f"API request failed: {e}"}


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    result = tool(slot="hand")
    print(result)

