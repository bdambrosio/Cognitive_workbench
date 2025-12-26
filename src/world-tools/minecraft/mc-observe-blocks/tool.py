"""
Minecraft observe-blocks tool.
Exhaustive enumeration of ALL visible non-air blocks within radius R.
Visibility = within radius R AND line-of-sight from bot's eye position.
Does NOT report items (use mc-observe-items for that).
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
    Get exhaustive enumeration of ALL visible non-air blocks within radius R.
    Visibility = within radius R AND line-of-sight from bot's eye position.
    Does NOT report items (use mc-observe-items for that).
    
    Args:
        input_value: ignored
        minecraft_url: Optional URL override for Minecraft bot server (default: http://localhost:3003)
        radius: Optional observation radius (default: 5)
        blocks_radius: Optional blocks observation radius (default: 5, max: 6)
        entities_radius: Optional entities observation radius (default: 5, max: 12)
        
    Returns:
        Dict with observation information (text, format, metadata, char_count).
        Executor will create Note from this content.
    """
    minecraft_url = kwargs.get("world_url") or kwargs.get("minecraft_url") or DEFAULT_MINECRAFT_URL
    
    try:
        params = {}
        # Bot uses separate params: blocks_radius, entities_radius
        if kwargs.get("blocks_radius") is not None:
            params["blocks_radius"] = kwargs.get("blocks_radius")
        if kwargs.get("entities_radius") is not None:
            params["entities_radius"] = kwargs.get("entities_radius")
        # Support legacy "radius" param for convenience
        if kwargs.get("radius") is not None:
            params["blocks_radius"] = kwargs.get("radius")
        
        # Filter out items - only show blocks and non-item entities
        params["entity_filter"] = "non-items"
        
        response = requests.get(f"{minecraft_url}/observe", params=params, timeout=10.0)
        response.raise_for_status()
        data = response.json()
        
        # Bot returns { ok: true, status: {...}, perception: {...} }
        status = data.get('status', {})
        perception = data.get('perception', {})
        
        # Format observation as readable text
        obs_parts = []
        obs_parts.append("Minecraft World Observation:")
        
        position = status.get('position')
        if position:
            if isinstance(position, dict):
                obs_parts.append(f"Bot Position: ({position.get('x', 0):.2f}, {position.get('y', 0):.2f}, {position.get('z', 0):.2f})")
            elif isinstance(position, (list, tuple)) and len(position) >= 3:
                obs_parts.append(f"Bot Position: ({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f})")
        
        yaw = status.get('yaw')
        pitch = status.get('pitch')
        if yaw is not None and pitch is not None:
            obs_parts.append(f"Yaw: {yaw:.2f}, Pitch: {pitch:.2f}")
        
        visibility_distances = perception.get('visibility_distances', {})
        
        # Format visibility distances
        if visibility_distances:
            obs_parts.append("\nVisibility Distances:")
            radius = kwargs.get("blocks_radius") or kwargs.get("radius") or 5
            for direction in ['forward', 'back', 'left', 'right', 'up', 'down']:
                dist = visibility_distances.get(direction)
                if dist is not None:
                    obs_parts.append(f"  {direction.capitalize()}: {dist:.2f} blocks")
                else:
                    obs_parts.append(f"  {direction.capitalize()}: >{radius} blocks (no block found)")
        
        nearby_blocks = perception.get('nearby_blocks', [])
        blocks_complete = perception.get('blocks_complete', True)
        blocks_elapsed_ms = perception.get('blocks_elapsed_ms', 0)
        
        if nearby_blocks:
            status_text = "visible" if blocks_complete else "visible (partial, timeout)"
            obs_parts.append(f"\nVisible Blocks ({len(nearby_blocks)} {status_text}):")
            for block in nearby_blocks[:30]:  # Show more since it's exhaustive
                if not isinstance(block, dict):
                    continue
                pos = block.get('position')
                block_name = block.get('name', 'unknown')
                if isinstance(pos, dict):
                    obs_parts.append(f"  {block_name} at ({pos.get('x', 0)}, {pos.get('y', 0)}, {pos.get('z', 0)})")
                elif isinstance(pos, (list, tuple)) and len(pos) >= 3:
                    obs_parts.append(f"  {block_name} at ({pos[0]}, {pos[1]}, {pos[2]})")
                else:
                    obs_parts.append(f"  {block_name} (position unknown)")
            if len(nearby_blocks) > 30:
                obs_parts.append(f"  ... and {len(nearby_blocks) - 30} more")
            
            if not blocks_complete:
                obs_parts.append(f"\nNote: Block enumeration incomplete (timeout after {blocks_elapsed_ms:.1f}ms)")
        else:
            obs_parts.append("\nNo visible blocks found within radius")
        
        nearby_entities = perception.get('nearby_entities', [])
        if nearby_entities:
            obs_parts.append(f"\nNearby Entities ({len(nearby_entities)}):")
            for entity in nearby_entities[:10]:  # Limit display
                if not isinstance(entity, dict):
                    continue
                pos = entity.get('position')
                entity_type = entity.get('type', 'unknown')
                distance = entity.get('distance', 0)
                if isinstance(pos, dict):
                    obs_parts.append(f"  {entity_type} at ({pos.get('x', 0):.2f}, {pos.get('y', 0):.2f}, {pos.get('z', 0):.2f}), distance: {distance:.2f}")
                elif isinstance(pos, (list, tuple)) and len(pos) >= 3:
                    obs_parts.append(f"  {entity_type} at ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}), distance: {distance:.2f}")
                else:
                    obs_parts.append(f"  {entity_type}, distance: {distance:.2f}")
            if len(nearby_entities) > 10:
                obs_parts.append(f"  ... and {len(nearby_entities) - 10} more")
        
        obs_text = "\n".join(obs_parts)
        
        return {
            "text": obs_text,
            "format": "text",
            "metadata": data,
            "char_count": len(obs_text)
        }
    except requests.exceptions.RequestException as e:
        logger.error(f"Minecraft observe request failed: {e}")
        return {"status": "failed", "reason": f"API request failed: {e}"}


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    result = tool()
    print(result)

