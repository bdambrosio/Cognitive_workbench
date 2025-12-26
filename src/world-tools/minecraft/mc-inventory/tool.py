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
        Dict with inventory information (text, format, metadata, char_count).
        Executor will create Note from this content.
    """
    minecraft_url = kwargs.get("world_url") or kwargs.get("minecraft_url") or DEFAULT_MINECRAFT_URL
    
    try:
        response = requests.get(f"{minecraft_url}/inventory", timeout=10.0)
        response.raise_for_status()
        data = response.json()
        
        if not data.get("ok"):
            error = data.get("error", "unknown failure")
            return {"status": "failed", "reason": f"Inventory request failed: {error}"}
        
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
        
        return {
            "text": inv_text,
            "format": "text",
            "metadata": {
                "slots": slots,
                "equipped": equipped,
                **data
            },
            "char_count": len(inv_text)
        }
    except requests.exceptions.RequestException as e:
        logger.error(f"Minecraft inventory request failed: {e}")
        return {"status": "failed", "reason": f"API request failed: {e}"}


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    result = tool()
    print(result)

