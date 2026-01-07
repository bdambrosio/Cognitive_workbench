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
        Dict with result using uniform return format.
    """
    executor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}
    
    minecraft_url = kwargs.get("world_url") or kwargs.get("minecraft_url") or DEFAULT_MINECRAFT_URL
    
    recipe = kwargs.get("recipe") or input_value or ""
    if not isinstance(recipe, str) or not recipe.strip():
        return executor._create_uniform_return(
            'failed',
            value="recipe/item name required (string)",
            reason="missing_recipe"
        )
    
    count = kwargs.get("count", 1)
    try:
        count = int(count)
    except (ValueError, TypeError):
        count = 1
    
    craft_params = {"recipe": recipe, "count": count}
    
    try:
        url = f"{minecraft_url}/act/craft"
        logger.debug(f"🔍 mc-craft: POST {url} body={craft_params}")
        response = requests.post(url, json=craft_params, timeout=30.0)
        logger.debug(f"📥 mc-craft: Response status={response.status_code}")
        response.raise_for_status()
        data = response.json()
        
        if not data.get("ok"):
            error = data.get("error", "unknown failure")
            result_text = f"Craft failed: {error}"
            return executor._create_uniform_return(
                'failed',
                value=result_text,
                reason="craft_failed"
            )
        
        crafted_item = data.get("item", recipe)
        crafted_count = data.get("count", count)
        result_text = f"Crafted {crafted_count}x {crafted_item}"
        
        # Build structured data dict
        # Extract metadata fields for extra
        extra_metadata = {
            "item": crafted_item,
            "count": crafted_count
        }
        # Merge API response data into extra
        extra_metadata.update(data)
        
        return executor._create_uniform_return('success', value=result_text, extra=extra_metadata)
    except requests.exceptions.RequestException as e:
        logger.error(f"Minecraft craft request failed: {e}")
        return executor._create_uniform_return(
            'failed',
            value=f"API request failed: {e}",
            reason="api_failed"
        )


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    result = tool("minecraft:stick", count=4)
    print(result)

