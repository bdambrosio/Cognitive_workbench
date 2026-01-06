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
            data={"success": False, "failure_reason": "missing_recipe"}
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
            error_code = data.get("error_code", "unknown_error")
            result_text = f"Craft failed: {error}"
            return executor._create_uniform_return(
                'failed',
                value=result_text,
                data={
                    "success": False,
                    "failure_reason": "craft_failed",
                    "error": error,
                    "error_code": error_code,
                    **data
                }
            )
        
        crafted_item = data.get("item", recipe)
        crafted_count = data.get("count", count)
        result_text = f"Crafted {crafted_count}x {crafted_item}"
        
        # Build structured data dict
        structured_data = {
            "success": True,
            "item": crafted_item,
            "count": crafted_count,
            **data
        }
        
        return executor._create_uniform_return('success', value=result_text, data=structured_data)
    except requests.exceptions.RequestException as e:
        logger.error(f"Minecraft craft request failed: {e}")
        return executor._create_uniform_return(
            'failed',
            value=f"API request failed: {e}",
            data={"success": False, "failure_reason": "api_failed"}
        )


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    result = tool("minecraft:stick", count=4)
    print(result)

