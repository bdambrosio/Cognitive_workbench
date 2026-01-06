"""
Minecraft drop tool.
Drop items from inventory into the world as entities - embodied manipulation.
"""

import logging
import os
import requests
from typing import Any, Dict

logger = logging.getLogger(__name__)

DEFAULT_MINECRAFT_URL = os.getenv("MINECRAFT_URL", "http://localhost:3003")


def tool(input_value=None, **kwargs):
    """
    Drop items from inventory into the world as entities.
    
    Args:
        input_value: Item name (preferred)
        item: Item name (alternative to input_value)
        count: int - number of items to drop (optional, default: all)
        scatter: bool - scatter items (optional, default: false)
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
        return executor._create_uniform_return(
            'failed',
            value="item name required (string)",
            data={"success": False, "failure_reason": "missing_item"}
        )
    
    count = kwargs.get("count")
    if count is not None:
        try:
            count = int(count)
        except (ValueError, TypeError):
            count = None
    
    scatter = kwargs.get("scatter", False)
    if isinstance(scatter, str):
        scatter = scatter.lower() in ("true", "1", "yes")
    
    drop_params = {"item": item}
    if count is not None:
        drop_params["count"] = count
    if scatter:
        drop_params["scatter"] = True
    
    try:
        url = f"{minecraft_url}/act/drop"
        logger.debug(f"🔍 mc-drop: POST {url} body={drop_params}")
        response = requests.post(url, json=drop_params, timeout=10.0)
        logger.debug(f"📥 mc-drop: Response status={response.status_code}")
        response.raise_for_status()
        data = response.json()
        
        if not data.get("ok"):
            error = data.get("error", "unknown failure")
            error_code = data.get("error_code", "unknown_error")
            result_text = f"Drop failed: {error}"
            return executor._create_uniform_return(
                'failed',
                value=result_text,
                data={
                    "success": False,
                    "failure_reason": "drop_failed",
                    "error": error,
                    "error_code": error_code,
                    **data
                }
            )
        
        dropped = data.get("dropped", [])
        
        result_parts = []
        if dropped:
            result_parts.append(f"Dropped: {len(dropped)} item(s)")
            for item_data in dropped:
                result_parts.append(f"  - {item_data.get('item', 'unknown')} x{item_data.get('count', 0)}")
        else:
            result_parts.append("No items dropped")
        
        result_text = "\n".join(result_parts)
        
        # Build structured data dict
        structured_data = {
            "success": True,
            "dropped": dropped,
            **data
        }
        
        return executor._create_uniform_return('success', value=result_text, data=structured_data)
    except requests.exceptions.RequestException as e:
        logger.error(f"Minecraft drop request failed: {e}")
        return executor._create_uniform_return(
            'failed',
            value=f"API request failed: {e}",
            data={"success": False, "failure_reason": "api_failed"}
        )


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    result = tool("stone", count=1)
    print(result)

