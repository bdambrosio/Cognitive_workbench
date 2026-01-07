"""
Minecraft observe-entities tool.
Exhaustive enumeration of ALL entities within radius R.
Reports all entity types: mobs, players, items, etc.
"""

import logging
import os
import requests
from typing import Any, Dict, List, Set
from collections import defaultdict

logger = logging.getLogger(__name__)

# Default Minecraft bot server URL (can be overridden via environment variable or config)
DEFAULT_MINECRAFT_URL = os.getenv("MINECRAFT_URL", "http://localhost:3003")


def categorize_entity(entity_type: str) -> str:
    """
    Categorize entity by type.
    
    Returns:
        'item', 'mob', 'player', or 'other'
    """
    if not entity_type:
        return 'other'
    
    entity_type_lower = str(entity_type).lower()
    
    if 'item' in entity_type_lower:
        return 'item'
    if 'player' in entity_type_lower:
        return 'player'
    # Common mob types
    mob_indicators = ['zombie', 'skeleton', 'creeper', 'spider', 'cow', 'pig', 'chicken', 
                      'sheep', 'wolf', 'villager', 'enderman', 'slime', 'witch', 'witch',
                      'guardian', 'shulker', 'phantom', 'ghast', 'blaze', 'wither',
                      'dragon', 'horse', 'donkey', 'mule', 'llama', 'panda', 'fox',
                      'bee', 'dolphin', 'turtle', 'cat', 'ocelot', 'parrot', 'rabbit',
                      'polar_bear', 'iron_golem', 'snow_golem', 'vex', 'evoker', 'vindicator',
                      'pillager', 'ravager', 'hoglin', 'piglin', 'strider', 'zoglin']
    
    if any(indicator in entity_type_lower for indicator in mob_indicators):
        return 'mob'
    
    return 'other'


def compute_confidence(entities_complete: bool, entities_elapsed_ms: float, entity_count: int) -> str:
    """
    Compute confidence level based on data completeness.
    
    Returns:
        'high', 'med', or 'low'
    """
    if not entities_complete:
        return 'low'
    if entities_elapsed_ms > 8.0:  # Near timeout
        return 'med'
    if entity_count > 50:  # Many entities might indicate complex environment
        return 'med'
    return 'high'


def tool(input_value=None, **kwargs):
    """
    Get exhaustive enumeration of ALL entities within radius R.
    Reports all entity types: mobs, players, items, etc.
    
    Returns structured observation summary following simplified schema.
    """
    executor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}
    
    minecraft_url = kwargs.get("world_url") or kwargs.get("minecraft_url") or DEFAULT_MINECRAFT_URL
    
    try:
        params = {}
        if kwargs.get("entities_radius") is not None:
            params["entities_radius"] = kwargs.get("entities_radius")
        if kwargs.get("radius") is not None:
            params["entities_radius"] = kwargs.get("radius")
        
        # Explicitly request all entities (empty string triggers scanning, scan_entities only filters for "items")
        params["entity_filter"] = ""
        url = f"{minecraft_url}/observe"
        logger.debug(f"🔍 mc-observe-entities: GET {url} params={params}")
        response = requests.get(url, params=params, timeout=10.0)
        logger.debug(f"📥 mc-observe-entities: Response status={response.status_code}, headers={dict(response.headers)}")
        response.raise_for_status()
        data = response.json()
        logger.debug(f"📥 mc-observe-entities: Response body keys={list(data.keys())}, entities_count={len(data.get('perception', {}).get('nearby_entities', []))}")
        
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
        
        nearby_entities = perception.get('nearby_entities', [])
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
        
        # entities: summary and details
        summary_parts.append("entities:")
        
        # Initialize variables for structured data
        entities_by_category = defaultdict(list)
        entities_by_type = defaultdict(int)
        entities_with_distance = []
        nearest_by_type = {}
        
        if not nearby_entities:
            summary_parts.append("  total: 0")
            summary_parts.append("  by_category: {}")
            summary_parts.append("  types: {}")
            summary_parts.append("  nearest: {}")
            summary_parts.append("  by_distance: []")
        else:
            # Group entities by category and type
            for entity in nearby_entities:
                if not isinstance(entity, dict):
                    continue
                
                entity_type = entity.get('type') or entity.get('name') or 'unknown'
                distance = entity.get('distance', 0.0)
                
                pos = entity.get('position')
                if isinstance(pos, dict):
                    entity_pos = (pos.get('x', 0), pos.get('y', 0), pos.get('z', 0))
                elif isinstance(pos, (list, tuple)) and len(pos) >= 3:
                    entity_pos = (pos[0], pos[1], pos[2])
                else:
                    entity_pos = None
                
                category = categorize_entity(entity_type)
                entities_by_category[category].append(entity)
                entities_by_type[entity_type] += 1
                
                entity_data = {
                    'type': entity_type,
                    'category': category,
                    'distance': distance,
                    'position': entity_pos
                }
                
                # Add item-specific fields if available
                if category == 'item':
                    item_name = entity.get('item_name') or entity.get('name') or entity_type
                    item_count = entity.get('item_count', 1)
                    entity_data['item_name'] = item_name
                    entity_data['item_count'] = item_count
                
                # Add health if available (for mobs/players)
                health = entity.get('health') or entity.get('hp')
                if health is not None:
                    entity_data['health'] = health
                
                entities_with_distance.append(entity_data)
            
            # Sort by distance
            entities_with_distance.sort(key=lambda x: x['distance'])
            
            # Build summary
            summary_parts.append(f"  total: {len(nearby_entities)}")
            
            # By category summary
            category_counts = {cat: len(ents) for cat, ents in entities_by_category.items()}
            category_str = "{" + ", ".join([f"{k}: {v}" for k, v in sorted(category_counts.items())]) + "}"
            summary_parts.append(f"  by_category: {category_str}")
            
            # Types summary
            types_dict = dict(entities_by_type)
            types_str = "{" + ", ".join([f"{k}: {v}" for k, v in sorted(types_dict.items())]) + "}"
            summary_parts.append(f"  types: {types_str}")
            
            # Nearest entities (one per type, closest)
            for entity in entities_with_distance:
                entity_type = entity['type']
                if entity_type not in nearest_by_type:
                    nearest_by_type[entity_type] = entity['distance']
            nearest_str = "{" + ", ".join([f"{k}: {v:.2f}" for k, v in sorted(nearest_by_type.items())]) + "}"
            summary_parts.append(f"  nearest: {nearest_str}")
            
            # By distance (sorted list)
            summary_parts.append("  by_distance:")
            for entity in entities_with_distance[:20]:  # Limit to 20 for readability
                pos_str = ""
                if entity['position']:
                    pos_str = f" at ({entity['position'][0]:.2f},{entity['position'][1]:.2f},{entity['position'][2]:.2f})"
                
                entity_desc = entity['type']
                if entity.get('item_name'):
                    count_str = f" ({entity['item_count']}x)" if entity.get('item_count', 1) > 1 else ""
                    entity_desc = f"{entity['item_name']}{count_str}"
                
                health_str = ""
                if entity.get('health') is not None:
                    health_str = f" hp:{entity['health']:.1f}"
                
                summary_parts.append(f"    - {entity_desc}{health_str}{pos_str} dist: {entity['distance']:.2f}")
            if len(entities_with_distance) > 20:
                summary_parts.append(f"    ... and {len(entities_with_distance) - 20} more")
        
        summary_parts.append("")
        
        # conf: confidence
        conf = compute_confidence(entities_complete, entities_elapsed_ms, len(nearby_entities))
        summary_parts.append(f"conf: {conf}")
        
        # note: human-readable summary
        note_parts = []
        if not nearby_entities:
            note_parts.append("No entities found")
        else:
            total_count = len(nearby_entities)
            unique_types = len(entities_by_type)
            
            if total_count == 1:
                note_parts.append("1 entity found")
            else:
                note_parts.append(f"{total_count} entities found")
            
            if unique_types == 1:
                note_parts.append(f"({list(entities_by_type.keys())[0]})")
            elif unique_types > 1:
                note_parts.append(f"({unique_types} types)")
            
            # Add category breakdown
            category_list = []
            for cat in ['mob', 'player', 'item', 'other']:
                if cat in category_counts:
                    count = category_counts[cat]
                    if count == 1:
                        category_list.append(f"1 {cat}")
                    else:
                        category_list.append(f"{count} {cat}s")
            
            if category_list:
                note_parts.append(", ".join(category_list))
        
        note = ", ".join(note_parts) if note_parts else "Standard observation"
        summary_parts.append(f"note: \"{note}\"")
        
        summary_text = "\n".join(summary_parts)
        
        # Build structured data dict
        structured_data = {
            "pose": {
                "x": px,
                "y": py,
                "z": pz,
                "yaw": yaw,
                "pitch": pitch
            },
            "entities": {
                "total": len(nearby_entities),
                "by_category": {cat: len(ents) for cat, ents in entities_by_category.items()} if nearby_entities else {},
                "types": dict(entities_by_type) if nearby_entities else {},
                "nearest": nearest_by_type if nearby_entities else {},
                "by_distance": entities_with_distance if nearby_entities else [],
                "nearby": nearby_entities  # Full list of all entities
            },
            "conf": conf,
            "note": note
        }
        
        # Pass structured dict as value (stored in Note), summary_text will be formatted for LLM context
        return executor._create_uniform_return('success', value=structured_data, extra={"summary": summary_text})
    except requests.exceptions.RequestException as e:
        logger.error(f"Minecraft observe-entities request failed: {e}")
        return executor._create_uniform_return(
            'failed',
            value=f"API request failed: {e}",
            reason="api_failed"
        )


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    result = tool()
    print(result)
