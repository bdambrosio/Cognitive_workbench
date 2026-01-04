"""
Minecraft equip tool.
Equip an item from inventory into hand or offhand.
"""

import logging
import os
import requests
from typing import Any, Dict

logger = logging.getLogger(__name__)

DEFAULT_MINECRAFT_URL = os.getenv("MINECRAFT_URL", "http://localhost:3003")


def tool(input_value=None, **kwargs):
    """
    Equip an item from inventory into hand or offhand.
    
    Args:
        input_value: Item name (preferred)
        item: Item name (alternative to input_value)
        slot: "hand" or "offhand" (default: "hand")
        minecraft_url: Optional URL override for Minecraft bot server (default: http://localhost:3003)
        
    Returns:
        Dict with result using uniform return format.
    """
    executor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}
    
    minecraft_url = kwargs.get("world_url") or kwargs.get("minecraft_url") or DEFAULT_MINECRAFT_URL
    
    item = kwargs.get("item") or input_value or ""
    if not isinstance(item, str) or not item.strip():
        return executor._create_uniform_return('failed', reason="item name required (string)")
    
    slot = kwargs.get("slot", "hand")
    if slot not in ["hand", "offhand"]:
        return executor._create_uniform_return('failed', reason="slot must be 'hand' or 'offhand'")
    
    equip_params = {"item": item, "slot": slot}
    
    try:
        url = f"{minecraft_url}/act/equip"
        logger.debug(f"🔍 mc-equip: POST {url} body={equip_params}")
        response = requests.post(url, json=equip_params, timeout=10.0)
        logger.debug(f"📥 mc-equip: Response status={response.status_code}")
        response.raise_for_status()
        data = response.json()
        
        if not data.get("ok"):
            error = data.get("error", "unknown failure")
            error_code = data.get("error_code", "unknown_error")
            result_text = f"Equip failed: {error}"
            return executor._create_uniform_return('failed', reason=result_text, data={
                "error": error,
                "error_code": error_code,
                **data
            })
        
        equipped_item = data.get("equipped", item)
        result_text = f"Equipped {equipped_item} in {slot}"
        
        # Build structured data dict
        structured_data = {
            "equipped": equipped_item,
            "slot": slot,
            **data
        }
        
        return executor._create_uniform_return('success', value=result_text, data=structured_data)
    except requests.exceptions.RequestException as e:
        logger.error(f"Minecraft equip request failed: {e}")
        return executor._create_uniform_return('failed', reason=f"API request failed: {e}")


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    result = tool("stone", slot="hand")
    print(result)

