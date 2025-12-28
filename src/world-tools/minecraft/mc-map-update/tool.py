"""
Minecraft map-update tool.
Converts ephemeral observation data into persistent spatial memory.
Stores observation data in a persistent Collection named "minecraft_map".
"""

import logging
import json
from datetime import datetime
from typing import Any, Dict, List, Optional

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
    Update persistent spatial map with observation data.
    
    Args:
        input_value: Observation data from mc-observe-blocks (Note ID or dict)
        observation: Alternative parameter name for observation data
        x: Optional x coordinate (defaults to extracting from observation)
        y: Optional y coordinate (defaults to extracting from observation)
        z: Optional z coordinate (defaults to extracting from observation)
        map_name: Optional map Collection name (default: "minecraft_map")
        resource_manager: Resource manager instance (from executor)
        agent_name: Agent name (for creating resources)
        
    Returns:
        Dict with status and result information.
    """
    resource_manager = kwargs.get('resource_manager')
    agent_name = kwargs.get('agent_name', 'system')
    map_name = kwargs.get('map_name', DEFAULT_MAP_NAME)
    
    if not resource_manager:
        return {
            "status": "failed",
            "reason": "Resource manager not available",
            "text": "Failed to update map: Resource manager not available",
            "format": "text",
            "char_count": 0
        }
    
    # Get observation data
    observation_data = kwargs.get('observation') or input_value
    
    # Resolve observation if it's a Note ID
    if isinstance(observation_data, str) and (observation_data.startswith('Note_') or observation_data.startswith('$')):
        # Try to resolve as Note ID
        if observation_data.startswith('$'):
            # Variable reference - would need executor context, skip for now
            observation_data = None
        else:
            observation_data = _get_content(observation_data, resource_manager)
    
    if not observation_data:
        return {
            "status": "failed",
            "reason": "No observation data provided",
            "text": "Failed to update map: No observation data provided",
            "format": "text",
            "char_count": 0
        }
    
    # Parse observation data
    if isinstance(observation_data, str):
        try:
            # Try to parse as JSON
            observation_data = json.loads(observation_data)
        except json.JSONDecodeError:
            # Try to extract from structured text format
            # Look for "pose: (x.xx,y.yy,z.zz, yaw:y.yy, pitch:p.pp)" pattern
            import re
            pose_match = re.search(r'pose:\s*\(([-\d.]+),([-\d.]+),([-\d.]+)', observation_data)
            if pose_match:
                x, y, z = float(pose_match.group(1)), float(pose_match.group(2)), float(pose_match.group(3))
            else:
                return {
                    "status": "failed",
                    "reason": "Could not parse observation data",
                    "text": "Failed to update map: Could not parse observation data",
                    "format": "text",
                    "char_count": 0
                }
    
    # Extract coordinates
    x = kwargs.get('x')
    y = kwargs.get('y')
    z = kwargs.get('z')
    
    if x is None or y is None or z is None:
        # Try to extract from observation data
        if isinstance(observation_data, dict):
            # Check for pose field
            if 'pose' in observation_data:
                pose = observation_data['pose']
                if isinstance(pose, dict):
                    x = pose.get('x')
                    y = pose.get('y')
                    z = pose.get('z')
                elif isinstance(pose, str):
                    # Parse "pose: (x.xx,y.yy,z.zz, yaw:y.yy, pitch:p.pp)"
                    import re
                    match = re.search(r'\(([-\d.]+),([-\d.]+),([-\d.]+)', pose)
                    if match:
                        x, y, z = float(match.group(1)), float(match.group(2)), float(match.group(3))
            # Check for direct x/y/z fields
            if x is None:
                x = observation_data.get('x')
            if y is None:
                y = observation_data.get('y')
            if z is None:
                z = observation_data.get('z')
            # Check metadata for position
            if x is None and 'metadata' in observation_data:
                metadata = observation_data['metadata']
                if isinstance(metadata, dict) and 'status' in metadata:
                    status = metadata['status']
                    if isinstance(status, dict) and 'position' in status:
                        pos = status['position']
                        if isinstance(pos, dict):
                            x = pos.get('x')
                            y = pos.get('y')
                            z = pos.get('z')
                        elif isinstance(pos, (list, tuple)) and len(pos) >= 3:
                            x, y, z = pos[0], pos[1], pos[2]
    
    if x is None or y is None or z is None:
        return {
            "status": "failed",
            "reason": "Could not determine coordinates from observation",
            "text": "Failed to update map: Could not determine coordinates (x, y, z) from observation data",
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
            agent_name, [], 'list', 'mc-map-update', 'Initial map creation', map_name, {}
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
    entry_key = f"{x_block},{y_block},{z_block}"
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
    
    # Prepare observation summary (extract structured data if available)
    observed_data = {}
    if isinstance(observation_data, dict):
        # Extract structured fields from observation
        for field in ['pose', 'dirs', 'support', 'clear', 'blocks', 'geom', 'aff', 'conf', 'note']:
            if field in observation_data:
                observed_data[field] = observation_data[field]
        # If no structured fields, store full observation
        if not observed_data:
            observed_data = observation_data
    elif isinstance(observation_data, str):
        # Store text observation
        observed_data = {'text': observation_data}
    
    # Create or update entry
    timestamp = datetime.now().isoformat()
    
    if existing_entry:
        # Update existing entry
        existing_entry['observed'] = observed_data
        existing_entry['last_visit'] = timestamp
        existing_entry['visit_count'] = existing_entry.get('visit_count', 0) + 1
        # Preserve waypoints if they exist
        if 'waypoints' not in existing_entry:
            existing_entry['waypoints'] = []
        map_content[existing_index] = existing_entry
    else:
        # Create new entry
        new_entry = {
            'x': x_block,
            'y': y_block,
            'z': z_block,
            'observed': observed_data,
            'first_visit': timestamp,
            'last_visit': timestamp,
            'visit_count': 1,
            'waypoints': []
        }
        map_content.append(new_entry)
    
    # Update Collection content
    map_resource = resource_manager.get_resource(map_collection_id)
    if map_resource:
        map_resource['properties']['content'] = map_content
        map_resource['properties']['item_count'] = len(map_content)
        # Mark as persistent
        resource_manager.mark_persistent(map_collection_id, agent_name)
        
        result_text = f"Map updated: ({x_block}, {y_block}, {z_block}) - {len(map_content)} locations total"
        
        return {
            "status": "success",
            "text": result_text,
            "format": "text",
            "metadata": {
                "map_name": map_name,
                "map_id": map_collection_id,
                "location": {"x": x_block, "y": y_block, "z": z_block},
                "total_locations": len(map_content),
                "visit_count": existing_entry['visit_count'] if existing_entry else 1
            },
            "char_count": len(result_text)
        }
    else:
        return {
            "status": "failed",
            "reason": "Map Collection not found after update",
            "text": "Failed to update map: Collection not found",
            "format": "text",
            "char_count": 0
        }


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    # Test with mock data
    result = tool(
        observation={"x": -112.12, "y": 71.0, "z": -123.67, "observed": {"note": "test"}},
        resource_manager=None,
        agent_name="test"
    )
    print(result)

