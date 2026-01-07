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
        Dict with result using uniform return format.
    """
    executor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}
    
    resource_manager = kwargs.get('resource_manager')
    agent_name = kwargs.get('agent_name', 'system')
    # Default to agent-specific map name
    default_map_name = f"{agent_name}-minecraft_map"
    map_name = kwargs.get('map_name') or default_map_name
    waypoint_name = kwargs.get('name')
    
    if not resource_manager:
        return executor._create_uniform_return(
            'failed',
            value="Resource manager not available",
            reason="no_resource_manager"
        )
    
    if not waypoint_name:
        return executor._create_uniform_return(
            'failed',
            value="Waypoint name required",
            reason="missing_name"
        )
    
    # Get coordinates
    x = kwargs.get('x')
    y = kwargs.get('y')
    z = kwargs.get('z')
    
    # If coordinates not provided, try to get from status (would need status Note ID)
    # For now, require explicit coordinates
    if x is None or y is None or z is None:
        return executor._create_uniform_return(
            'failed',
            value="Coordinates required (x, y, z)",
            reason="missing_coordinates"
        )
    
    # Round coordinates to block positions
    x_block = _round_coordinate(x)
    y_block = _round_coordinate(y)
    z_block = _round_coordinate(z)
    
    # Load or create map Collection
    map_collection_id = resource_manager.named_collections.get(map_name)
    
    if not map_collection_id:
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
            return executor._create_uniform_return(
                'failed',
                value=f"Failed to create map Collection: {error_msg}",
                reason="collection_creation_failed"
            )
    
    # Create entry with waypoint label
    from datetime import datetime
    timestamp = datetime.now().isoformat()
    entry = {
        'x': x_block,
        'y': y_block,
        'z': z_block,
        'waypoints': [waypoint_name],
        'timestamp': timestamp
    }
    
    # Create a Note with this waypoint entry
    success, note_id, error_msg, location = resource_manager.create_note(
        agent_name, entry, 'json', 'mc-waypoint', f"Waypoint {waypoint_name} at ({x_block}, {y_block}, {z_block})", '', {}
    )
    
    if not success:
        return executor._create_uniform_return(
            'failed',
            value=f"Failed to create waypoint Note: {error_msg}",
            reason="note_creation_failed"
        )
    
    # Add Note to Collection
    success, item_count, error_msg = resource_manager.add_to_collection(
        map_collection_id, note_id, agent_name, 'add', None
    )
    
    if not success:
        return executor._create_uniform_return(
            'failed',
            value=f"Failed to add Note to Collection: {error_msg}",
            reason="add_to_collection_failed"
        )
    
    # Mark Collection as persistent
    resource_manager.mark_persistent(map_collection_id, agent_name)
    
    result_text = f"Waypoint '{waypoint_name}' created at ({x_block}, {y_block}, {z_block})"
    
    # Build structured data dict
    structured_data = {
        "map_name": map_name,
        "map_id": map_collection_id,
        "note_id": note_id,
        "waypoint": waypoint_name,
        "location": {"x": x_block, "y": y_block, "z": z_block}
    }
    
    return executor._create_uniform_return('success', value=result_text, extra=structured_data)


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

