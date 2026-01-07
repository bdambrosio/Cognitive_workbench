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
        Dict with result using uniform return format.
    """
    executor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}
    
    minecraft_url = kwargs.get("world_url") or kwargs.get("minecraft_url") or DEFAULT_MINECRAFT_URL
    
    slot = kwargs.get("slot", "hand")
    if slot not in ["hand", "offhand"]:
        return executor._create_uniform_return(
            'failed',
            value="slot must be 'hand' or 'offhand'",
            reason="invalid_slot"
        )
    
    unequip_params = {"slot": slot}
    
    try:
        url = f"{minecraft_url}/act/unequip"
        logger.debug(f"🔍 mc-unequip: POST {url} body={unequip_params}")
        response = requests.post(url, json=unequip_params, timeout=10.0)
        logger.debug(f"📥 mc-unequip: Response status={response.status_code}")
        response.raise_for_status()
        data = response.json()
        
        if not data.get("ok"):
            error = data.get("error", "unknown failure")
            result_text = f"Unequip failed: {error}"
            return executor._create_uniform_return(
                'failed',
                value=result_text,
                reason="unequip_failed"
            )
        
        result_text = f"Unequipped {slot}"
        
        # Build structured data dict
        # Extract metadata fields for extra
        extra_metadata = {
            "slot": slot
        }
        # Merge API response data into extra
        extra_metadata.update(data)
        
        return executor._create_uniform_return('success', value=result_text, extra=extra_metadata)
    except requests.exceptions.RequestException as e:
        logger.error(f"Minecraft unequip request failed: {e}")
        return executor._create_uniform_return(
            'failed',
            value=f"API request failed: {e}",
            reason="api_failed"
        )


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    result = tool(slot="hand")
    print(result)

