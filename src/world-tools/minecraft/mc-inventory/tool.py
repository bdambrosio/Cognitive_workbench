"""
Minecraft inventory tool.
Observe inventory contents and equipped items - epistemic, read-only.
"""

import logging
import os
import requests
from typing import Any, Dict

logger = logging.getLogger(__name__)

DEFAULT_MINECRAFT_URL = os.getenv("MINECRAFT_URL", "http://localhost:3003")


def tool(input_value=None, **kwargs):
    """
    Get inventory contents and equipped items - epistemic observation.
    
    Args:
        input_value: ignored
        minecraft_url: Optional URL override for Minecraft bot server (default: http://localhost:3003)
        
    Returns:
        Dict with result using uniform return format.
    """
    executor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}
    
    minecraft_url = kwargs.get("world_url") or kwargs.get("minecraft_url") or DEFAULT_MINECRAFT_URL
    
    try:
        url = f"{minecraft_url}/inventory"
        logger.debug(f"🔍 mc-inventory: GET {url}")
        response = requests.get(url, timeout=10.0)
        logger.debug(f"📥 mc-inventory: Response status={response.status_code}")
        response.raise_for_status()
        data = response.json()
        
        if not data.get("ok"):
            error = data.get("error", "unknown failure")
            return executor._create_uniform_return(
                'failed',
                value=f"Inventory request failed: {error}",
                reason="inventory_failed"
            )
        
        slots = data.get("slots", [])
        equipped = data.get("equipped", {})
        
        inv_parts = []
        inv_parts.append("Minecraft Inventory:")
        
        if slots:
            inv_parts.append(f"\nInventory Slots ({len(slots)} items):")
            for slot_data in slots:
                slot_num = slot_data.get("slot", "?")
                item = slot_data.get("item", "empty")
                count = slot_data.get("count", 0)
                if item and item != "air":
                    inv_parts.append(f"  Slot {slot_num}: {item} x{count}")
        else:
            inv_parts.append("\nInventory: empty")
        
        hand_item = equipped.get("hand")
        offhand_item = equipped.get("offhand")
        inv_parts.append("\nEquipped:")
        inv_parts.append(f"  Hand: {hand_item if hand_item else 'empty'}")
        inv_parts.append(f"  Offhand: {offhand_item if offhand_item else 'empty'}")
        
        inv_text = "\n".join(inv_parts)
        
        # Build structured data dict
        # Extract metadata fields for extra
        extra_metadata = {
            "slots": slots,
            "equipped": equipped
        }
        # Merge API response data into extra
        extra_metadata.update(data)
        
        return executor._create_uniform_return('success', value=inv_text, extra=extra_metadata)
    except requests.exceptions.RequestException as e:
        logger.error(f"Minecraft inventory request failed: {e}")
        return executor._create_uniform_return(
            'failed',
            value=f"API request failed: {e}",
            reason="api_failed"
        )


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    result = tool()
    print(result)

