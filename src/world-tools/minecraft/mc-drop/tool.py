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
        Dict with result (text, format, metadata, char_count).
        Executor will create Note from this content.
    """
    minecraft_url = kwargs.get("world_url") or kwargs.get("minecraft_url") or DEFAULT_MINECRAFT_URL
    
    item = kwargs.get("item") or input_value or ""
    if not isinstance(item, str) or not item.strip():
        return {"status": "failed", "reason": "item name required (string)"}
    
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
        response = requests.post(
            f"{minecraft_url}/act/drop",
            json=drop_params,
            timeout=10.0
        )
        response.raise_for_status()
        data = response.json()
        
        if not data.get("ok"):
            error = data.get("error", "unknown failure")
            error_code = data.get("error_code", "unknown_error")
            result_text = f"Drop failed: {error}"
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
        
        dropped = data.get("dropped", [])
        
        result_parts = []
        if dropped:
            result_parts.append(f"Dropped: {len(dropped)} item(s)")
            for item_data in dropped:
                result_parts.append(f"  - {item_data.get('item', 'unknown')} x{item_data.get('count', 0)}")
        else:
            result_parts.append("No items dropped")
        
        result_text = "\n".join(result_parts)
        
        return {
            "text": result_text,
            "format": "text",
            "metadata": {
                "success": True,
                "dropped": dropped,
                **data
            },
            "char_count": len(result_text)
        }
    except requests.exceptions.RequestException as e:
        logger.error(f"Minecraft drop request failed: {e}")
        return {"status": "failed", "reason": f"API request failed: {e}"}


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    result = tool("stone", count=1)
    print(result)

