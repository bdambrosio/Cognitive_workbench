"""
Minecraft craft tool.
Perform crafting via Mineflayer recipe system.
"""

import logging
import os
import requests
from typing import Any, Dict

logger = logging.getLogger(__name__)

DEFAULT_MINECRAFT_URL = os.getenv("MINECRAFT_URL", "http://localhost:3003")


def tool(input_value=None, **kwargs):
    """
    Perform crafting via Mineflayer recipe system.
    
    Args:
        input_value: Recipe/item name (preferred)
        recipe: Recipe/item name (alternative to input_value)
        count: int - number of items to craft (default: 1)
        minecraft_url: Optional URL override for Minecraft bot server (default: http://localhost:3003)
        
    Returns:
        Dict with result (text, format, metadata, char_count).
        Executor will create Note from this content.
    """
    minecraft_url = kwargs.get("world_url") or kwargs.get("minecraft_url") or DEFAULT_MINECRAFT_URL
    
    recipe = kwargs.get("recipe") or input_value or ""
    if not isinstance(recipe, str) or not recipe.strip():
        return {"status": "failed", "reason": "recipe/item name required (string)"}
    
    count = kwargs.get("count", 1)
    try:
        count = int(count)
    except (ValueError, TypeError):
        count = 1
    
    craft_params = {"recipe": recipe, "count": count}
    
    try:
        response = requests.post(
            f"{minecraft_url}/act/craft",
            json=craft_params,
            timeout=30.0
        )
        response.raise_for_status()
        data = response.json()
        
        if not data.get("ok"):
            error = data.get("error", "unknown failure")
            error_code = data.get("error_code", "unknown_error")
            result_text = f"Craft failed: {error}"
            return {
                "text": result_text,
                "format": "text",
                "metadata": {
                    "success": False,
                    "error_code": error_code,
                    **data
                },
                "char_count": len(result_text)
            }
        
        crafted_item = data.get("item", recipe)
        crafted_count = data.get("count", count)
        result_text = f"Crafted {crafted_count}x {crafted_item}"
        
        return {
            "text": result_text,
            "format": "text",
            "metadata": {
                "success": True,
                "item": crafted_item,
                "count": crafted_count,
                **data
            },
            "char_count": len(result_text)
        }
    except requests.exceptions.RequestException as e:
        logger.error(f"Minecraft craft request failed: {e}")
        return {"status": "failed", "reason": f"API request failed: {e}"}


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    result = tool("minecraft:stick", count=4)
    print(result)

