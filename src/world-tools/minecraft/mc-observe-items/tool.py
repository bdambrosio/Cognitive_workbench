"""
Minecraft observe-items tool.
Exhaustive enumeration of ALL item entities within radius R.
Only reports item entities (dropped items), not blocks or other entities.
"""

import logging
import os
import requests
from typing import Any, Dict, List, Set
from collections import defaultdict

logger = logging.getLogger(__name__)

# Default Minecraft bot server URL (can be overridden via environment variable or config)
DEFAULT_MINECRAFT_URL = os.getenv("MINECRAFT_URL", "http://localhost:3003")

# Pickup range thresholds (in blocks)
PICKUP_RANGE = 1.5  # Automatic pickup within this distance
NEARBY_RANGE = 3.0  # Considered nearby for pickup planning


def compute_confidence(entities_complete: bool, entities_elapsed_ms: float, item_count: int) -> str:
    """
    Compute confidence level based on data completeness.
    
    Returns:
        'high', 'med', or 'low'
    """
    if not entities_complete:
        return 'low'
    if entities_elapsed_ms > 8.0:  # Near timeout
        return 'med'
    if item_count > 50:  # Many items might indicate complex environment
        return 'med'
    return 'high'


def tool(input_value=None, **kwargs):
    """
    Get exhaustive enumeration of ALL item entities within radius R.
    Only reports item entities (dropped items from digging/dropping).
    
    Returns structured observation summary following simplified schema.
    """
    minecraft_url = kwargs.get("world_url") or kwargs.get("minecraft_url") or DEFAULT_MINECRAFT_URL
    
    try:
        params = {}
        if kwargs.get("entities_radius") is not None:
            params["entities_radius"] = kwargs.get("entities_radius")
        if kwargs.get("radius") is not None:
            params["entities_radius"] = kwargs.get("radius")
        
        params["entity_filter"] = "items"
        
        response = requests.get(f"{minecraft_url}/observe", params=params, timeout=10.0)
        response.raise_for_status()
        data = response.json()
        
        status = data.get('status', {})
        perception = data.get('perception', {})
        
        # Extract position and orientation
        position = status.get('position', {})
        if isinstance(position, (list, tuple)) and len(position) >= 3:
            position = {'x': position[0], 'y': position[1], 'z': position[2]}
        elif not isinstance(position, dict):
            position = {}
        
        yaw = status.get('yaw', 0.0)
        pitch = status.get('pitch', 0.0)
        
        nearby_items = perception.get('nearby_entities', [])
        entities_complete = perception.get('entities_complete', True)
        entities_elapsed_ms = perception.get('entities_elapsed_ms', 0)
        radius = kwargs.get("entities_radius") or kwargs.get("radius") or 5
        
        # Build structured summary
        summary_parts = []
        summary_parts.append("SUMMARY:")
        summary_parts.append("")
        
        # pose
        px = position.get('x', 0)
        py = position.get('y', 0)
        pz = position.get('z', 0)
        summary_parts.append(f"pose: ({px:.2f},{py:.2f},{pz:.2f}, yaw:{yaw:.2f}, pitch:{pitch:.2f})")
        summary_parts.append("")
        
        # items: summary and details
        summary_parts.append("items:")
        
        if not nearby_items:
            summary_parts.append("  total: 0")
            summary_parts.append("  types: {}")
            summary_parts.append("  nearest: {}")
            summary_parts.append("  by_distance: []")
        else:
            # Group items by type and compute distances
            items_by_type = defaultdict(int)
            items_with_distance = []
            
            for item in nearby_items:
                if not isinstance(item, dict):
                    continue
                
                item_name = item.get('item_name') or item.get('name') or item.get('type') or 'unknown'
                item_count = item.get('item_count', 1)
                distance = item.get('distance', 0.0)
                
                pos = item.get('position')
                if isinstance(pos, dict):
                    item_pos = (pos.get('x', 0), pos.get('y', 0), pos.get('z', 0))
                elif isinstance(pos, (list, tuple)) and len(pos) >= 3:
                    item_pos = (pos[0], pos[1], pos[2])
                else:
                    item_pos = None
                
                items_by_type[item_name] += item_count
                items_with_distance.append({
                    'name': item_name,
                    'count': item_count,
                    'distance': distance,
                    'position': item_pos
                })
            
            # Sort by distance
            items_with_distance.sort(key=lambda x: x['distance'])
            
            # Build summary
            summary_parts.append(f"  total: {len(nearby_items)}")
            
            # Types summary
            types_dict = dict(items_by_type)
            types_str = "{" + ", ".join([f"{k}: {v}" for k, v in sorted(types_dict.items())]) + "}"
            summary_parts.append(f"  types: {types_str}")
            
            # Nearest items (one per type, closest)
            nearest_by_type = {}
            for item in items_with_distance:
                item_name = item['name']
                if item_name not in nearest_by_type:
                    nearest_by_type[item_name] = item['distance']
            nearest_str = "{" + ", ".join([f"{k}: {v:.2f}" for k, v in sorted(nearest_by_type.items())]) + "}"
            summary_parts.append(f"  nearest: {nearest_str}")
            
            # By distance (sorted list)
            summary_parts.append("  by_distance:")
            for item in items_with_distance[:20]:  # Limit to 20 for readability
                pos_str = ""
                if item['position']:
                    pos_str = f" at ({item['position'][0]:.2f},{item['position'][1]:.2f},{item['position'][2]:.2f})"
                count_str = f" ({item['count']}x)" if item['count'] > 1 else ""
                summary_parts.append(f"    - {item['name']}{count_str}{pos_str} dist: {item['distance']:.2f}")
            if len(items_with_distance) > 20:
                summary_parts.append(f"    ... and {len(items_with_distance) - 20} more")
        
        summary_parts.append("")
        
        # pickup: feasibility categorization
        summary_parts.append("pickup:")
        
        if not nearby_items:
            summary_parts.append("  in_range: []")
            summary_parts.append("  nearby: []")
            summary_parts.append("  far: []")
        else:
            in_range: Set[str] = set()
            nearby: Set[str] = set()
            far: Set[str] = set()
            
            for item in nearby_items:
                if not isinstance(item, dict):
                    continue
                item_name = item.get('item_name') or item.get('name') or item.get('type') or 'unknown'
                distance = item.get('distance', float('inf'))
                
                if distance <= PICKUP_RANGE:
                    in_range.add(item_name)
                elif distance <= NEARBY_RANGE:
                    nearby.add(item_name)
                else:
                    far.add(item_name)
            
            summary_parts.append(f"  in_range: {sorted(list(in_range))}")
            summary_parts.append(f"  nearby: {sorted(list(nearby))}")
            summary_parts.append(f"  far: {sorted(list(far))}")
        
        summary_parts.append("")
        
        # conf: confidence
        conf = compute_confidence(entities_complete, entities_elapsed_ms, len(nearby_items))
        summary_parts.append(f"conf: {conf}")
        
        # note: human-readable summary
        note_parts = []
        if not nearby_items:
            note_parts.append("No items found")
        else:
            total_count = sum(items_by_type.values()) if nearby_items else 0
            unique_types = len(items_by_type) if nearby_items else 0
            
            if total_count == 1:
                note_parts.append("1 item found")
            else:
                note_parts.append(f"{total_count} items found")
            
            if unique_types == 1:
                note_parts.append(f"({list(items_by_type.keys())[0]})")
            elif unique_types > 1:
                note_parts.append(f"({unique_types} types)")
            
            if in_range:
                note_parts.append(f"{len(in_range)} type(s) in pickup range")
            elif nearby:
                note_parts.append(f"{len(nearby)} type(s) nearby")
        
        note = ", ".join(note_parts) if note_parts else "Standard observation"
        summary_parts.append(f"note: \"{note}\"")
        
        summary_text = "\n".join(summary_parts)
        
        return {
            "text": summary_text,
            "format": "text",
            "metadata": data,
            "char_count": len(summary_text)
        }
    except requests.exceptions.RequestException as e:
        logger.error(f"Minecraft observe-items request failed: {e}")
        return {"status": "failed", "reason": f"API request failed: {e}"}


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    result = tool()
    print(result)
