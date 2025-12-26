"""
Minecraft observe-items tool.
Exhaustive enumeration of ALL item entities within radius R.
Only reports item entities (dropped items), not blocks or other entities.
"""

import logging
import os
import requests
from typing import Any, Dict

logger = logging.getLogger(__name__)

# Default Minecraft bot server URL (can be overridden via environment variable or config)
DEFAULT_MINECRAFT_URL = os.getenv("MINECRAFT_URL", "http://localhost:3003")


def tool(input_value=None, **kwargs):
    """
    Get exhaustive enumeration of ALL item entities within radius R.
    Only reports item entities (dropped items from digging/dropping).
    
    Args:
        input_value: ignored
        minecraft_url: Optional URL override for Minecraft bot server (default: http://localhost:3003)
        radius: Optional observation radius for items (default: 5, max: 12)
        entities_radius: Optional entities observation radius (default: 5, max: 12)
        
    Returns:
        Dict with observation information (text, format, metadata, char_count).
        Executor will create Note from this content.
    """
    minecraft_url = kwargs.get("world_url") or kwargs.get("minecraft_url") or DEFAULT_MINECRAFT_URL
    
    try:
        params = {}
        if kwargs.get("entities_radius") is not None:
            params["entities_radius"] = kwargs.get("entities_radius")
        # Support legacy "radius" param for convenience
        if kwargs.get("radius") is not None:
            params["entities_radius"] = kwargs.get("radius")
        
        # Filter to items only
        params["entity_filter"] = "items"
        
        response = requests.get(f"{minecraft_url}/observe", params=params, timeout=10.0)
        response.raise_for_status()
        data = response.json()
        
        # Bot returns { ok: true, status: {...}, perception: {...} }
        status = data.get('status', {})
        perception = data.get('perception', {})
        
        # Format observation as readable text
        obs_parts = []
        obs_parts.append("Minecraft Items Observation:")
        
        position = status.get('position')
        if position:
            if isinstance(position, dict):
                obs_parts.append(f"Bot Position: ({position.get('x', 0):.2f}, {position.get('y', 0):.2f}, {position.get('z', 0):.2f})")
            elif isinstance(position, (list, tuple)) and len(position) >= 3:
                obs_parts.append(f"Bot Position: ({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f})")
        
        nearby_items = perception.get('nearby_entities', [])
        entities_complete = perception.get('entities_complete', True)
        entities_elapsed_ms = perception.get('entities_elapsed_ms', 0)
        visibility_distances = perception.get('visibility_distances', {})
        
        # Format visibility distances
        if visibility_distances:
            obs_parts.append("\nVisibility Distances:")
            radius = kwargs.get("entities_radius") or kwargs.get("radius") or 5
            for direction in ['forward', 'back', 'left', 'right', 'up', 'down']:
                dist = visibility_distances.get(direction)
                if dist is not None:
                    obs_parts.append(f"  {direction.capitalize()}: {dist:.2f} blocks")
                else:
                    obs_parts.append(f"  {direction.capitalize()}: >{radius} blocks (no block found)")
        
        if nearby_items:
            status_text = "found" if entities_complete else "found (partial, timeout)"
            obs_parts.append(f"\nItems ({len(nearby_items)} {status_text}):")
            for item in nearby_items[:30]:  # Show more since it's exhaustive
                if not isinstance(item, dict):
                    continue
                pos = item.get('position')
                item_name = item.get('item_name') or item.get('name') or item.get('type') or 'unknown'
                item_count = item.get('item_count', 1)
                distance = item.get('distance', 0)
                if isinstance(pos, dict):
                    obs_parts.append(f"  {item_count}x {item_name} at ({pos.get('x', 0):.2f}, {pos.get('y', 0):.2f}, {pos.get('z', 0):.2f}), distance: {distance:.2f}")
                elif isinstance(pos, (list, tuple)) and len(pos) >= 3:
                    obs_parts.append(f"  {item_count}x {item_name} at ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}), distance: {distance:.2f}")
                else:
                    obs_parts.append(f"  {item_count}x {item_name}, distance: {distance:.2f}")
            if len(nearby_items) > 30:
                obs_parts.append(f"  ... and {len(nearby_items) - 30} more")
            
            if not entities_complete:
                obs_parts.append(f"\nNote: Item enumeration incomplete (timeout after {entities_elapsed_ms:.1f}ms)")
        else:
            obs_parts.append("\nNo items found within radius")
        
        obs_text = "\n".join(obs_parts)
        
        return {
            "text": obs_text,
            "format": "text",
            "metadata": data,
            "char_count": len(obs_text)
        }
    except requests.exceptions.RequestException as e:
        logger.error(f"Minecraft observe-items request failed: {e}")
        return {"status": "failed", "reason": f"API request failed: {e}"}


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    result = tool()
    print(result)

