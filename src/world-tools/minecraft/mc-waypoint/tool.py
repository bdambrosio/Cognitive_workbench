"""
Minecraft waypoint tool.
Labels coordinates for reasoning about spatial relationships.
"""

import logging
from typing import Any, Dict, Optional

logger = logging.getLogger(__name__)

# Default map name
DEFAULT_MAP_NAME = "minecraft_map"


def _get_content(resource_id: str, resource_manager) -> Any:
    """Fetch content for a resource ID."""
    if resource_id == "Note_null":
        return None
    
    if not resource_manager:
        logger.error("Resource manager not available")
        return None
    
    resource = resource_manager.get_resource(resource_id)
    if not resource:
        return None
    
    return resource.get('properties', {}).get('content')


def _round_coordinate(coord: float) -> int:
    """Round coordinate to nearest block (integer)."""
    return int(round(coord))


def tool(input_value=None, **kwargs):
    """
    Label a coordinate with a waypoint name.
    
    Args:
        input_value: Ignored
        name: Waypoint name (required)
        x: Optional x coordinate (defaults to current position from status)
        y: Optional y coordinate (defaults to current position from status)
        z: Optional z coordinate (defaults to current position from status)
        map_name: Optional map Collection name (default: "minecraft_map")
        resource_manager: Resource manager instance (from executor)
        agent_name: Agent name (for creating resources)
        
    Returns:
        Dict with status and result information.
    """
    resource_manager = kwargs.get('resource_manager')
    agent_name = kwargs.get('agent_name', 'system')
    map_name = kwargs.get('map_name', DEFAULT_MAP_NAME)
    waypoint_name = kwargs.get('name')
    
    if not resource_manager:
        return {
            "status": "failed",
            "reason": "Resource manager not available",
            "text": "Failed to create waypoint: Resource manager not available",
            "format": "text",
            "char_count": 0
        }
    
    if not waypoint_name:
        return {
            "status": "failed",
            "reason": "Waypoint name required",
            "text": "Failed to create waypoint: Name parameter required",
            "format": "text",
            "char_count": 0
        }
    
    # Get coordinates
    x = kwargs.get('x')
    y = kwargs.get('y')
    z = kwargs.get('z')
    
    # If coordinates not provided, try to get from status (would need status Note ID)
    # For now, require explicit coordinates
    if x is None or y is None or z is None:
        return {
            "status": "failed",
            "reason": "Coordinates required (x, y, z)",
            "text": "Failed to create waypoint: Coordinates (x, y, z) required",
            "format": "text",
            "char_count": 0
        }
    
    # Round coordinates to block positions
    x_block = _round_coordinate(x)
    y_block = _round_coordinate(y)
    z_block = _round_coordinate(z)
    
    # Load or create map Collection
    map_collection_id = resource_manager.named_collections.get(map_name)
    map_content = []
    
    if map_collection_id:
        # Load existing map
        map_resource = resource_manager.get_resource(map_collection_id)
        if map_resource:
            map_content = map_resource.get('properties', {}).get('content', [])
            if not isinstance(map_content, list):
                map_content = []
    else:
        # Create new map Collection
        success, collection_id, error_msg, location = resource_manager.create_collection(
            agent_name, [], 'list', 'mc-waypoint', 'Initial map creation', map_name, {}
        )
        if success:
            map_collection_id = collection_id
            # Mark as persistent
            resource_manager.mark_persistent(collection_id, agent_name)
            logger.info(f"Created new map Collection: {map_name} = {collection_id}")
        else:
            return {
                "status": "failed",
                "reason": f"Failed to create map Collection: {error_msg}",
                "text": f"Failed to create map Collection: {error_msg}",
                "format": "text",
                "char_count": 0
            }
    
    # Find existing entry for this coordinate
    existing_entry = None
    existing_index = None
    
    for i, entry in enumerate(map_content):
        if isinstance(entry, dict):
            entry_x = _round_coordinate(entry.get('x', 0))
            entry_y = _round_coordinate(entry.get('y', 0))
            entry_z = _round_coordinate(entry.get('z', 0))
            if entry_x == x_block and entry_y == y_block and entry_z == z_block:
                existing_entry = entry
                existing_index = i
                break
    
    # Add waypoint label
    if existing_entry:
        # Update existing entry
        waypoints = existing_entry.get('waypoints', [])
        if not isinstance(waypoints, list):
            waypoints = []
        if waypoint_name not in waypoints:
            waypoints.append(waypoint_name)
        existing_entry['waypoints'] = waypoints
        map_content[existing_index] = existing_entry
    else:
        # Create new entry with waypoint
        from datetime import datetime
        new_entry = {
            'x': x_block,
            'y': y_block,
            'z': z_block,
            'observed': {},
            'first_visit': datetime.now().isoformat(),
            'last_visit': datetime.now().isoformat(),
            'visit_count': 0,
            'waypoints': [waypoint_name]
        }
        map_content.append(new_entry)
    
    # Update Collection content
    map_resource = resource_manager.get_resource(map_collection_id)
    if map_resource:
        map_resource['properties']['content'] = map_content
        map_resource['properties']['item_count'] = len(map_content)
        # Mark as persistent
        resource_manager.mark_persistent(map_collection_id, agent_name)
        
        result_text = f"Waypoint '{waypoint_name}' created at ({x_block}, {y_block}, {z_block})"
        
        return {
            "status": "success",
            "text": result_text,
            "format": "text",
            "metadata": {
                "map_name": map_name,
                "map_id": map_collection_id,
                "waypoint": waypoint_name,
                "location": {"x": x_block, "y": y_block, "z": z_block}
            },
            "char_count": len(result_text)
        }
    else:
        return {
            "status": "failed",
            "reason": "Map Collection not found after update",
            "text": "Failed to create waypoint: Collection not found",
            "format": "text",
            "char_count": 0
        }


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    # Test with mock data
    result = tool(
        name="Base_Camp",
        x=-112,
        y=71,
        z=-123,
        resource_manager=None,
        agent_name="test"
    )
    print(result)

