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
    # Default to agent-specific map name
    default_map_name = f"{agent_name}-minecraft_map"
    provided_map_name = kwargs.get('map_name')
    
    if not provided_map_name:
        # Not provided - use agent-specific default
        map_name = default_map_name
    elif provided_map_name in ['minecraft', 'infolab', 'scienceworld', 'osworld']:
        # Executor set world name - use agent-specific default instead
        map_name = default_map_name
    else:
        # Explicitly provided - use it
        map_name = provided_map_name
    
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
    
    if not map_collection_id:
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
    
    # Prepare observation summary (extract structured data if available)
    observed_data = {}
    if isinstance(observation_data, dict):
        # Check metadata first (where mc-observe-blocks puts structured fields)
        metadata = observation_data.get('metadata', {})
        if isinstance(metadata, dict):
            # Extract structured fields from metadata
            for field in ['pose', 'dirs', 'support', 'clear', 'blocks', 'geom', 'aff', 'conf', 'note']:
                if field in metadata:
                    observed_data[field] = metadata[field]
            
            # Extract nearby_blocks and filter to y-1, y, y+1 (walkable layers)
            perception = metadata.get('perception', {})
            if isinstance(perception, dict):
                nearby_blocks_raw = perception.get('nearby_blocks', [])
                if isinstance(nearby_blocks_raw, list):
                    # Filter to blocks at dy = -1, 0, or +1 (relative to agent Y)
                    nearby_blocks_filtered = []
                    for block in nearby_blocks_raw:
                        if isinstance(block, dict):
                            dy = block.get('dy')
                            if isinstance(dy, (int, float)) and abs(dy) <= 1:
                                nearby_blocks_filtered.append(block)
                    if nearby_blocks_filtered:
                        observed_data['nearby_blocks'] = nearby_blocks_filtered
        
        # Also check top-level dict for structured fields (backward compatibility)
        if not observed_data:
            for field in ['pose', 'dirs', 'support', 'clear', 'blocks', 'geom', 'aff', 'conf', 'note']:
                if field in observation_data:
                    observed_data[field] = observation_data[field]
        
        # If no structured fields found, store full observation
        if not observed_data:
            observed_data = observation_data
    elif isinstance(observation_data, str):
        # Store text observation
        observed_data = {'text': observation_data}
    
    # Create entry for this observation
    timestamp = datetime.now().isoformat()
    entry = {
        'x': x_block,
        'y': y_block,
        'z': z_block,
        'observed': observed_data,
        'timestamp': timestamp
    }
    
    # Create a Note with this entry
    success, note_id, error_msg, location = resource_manager.create_note(
        agent_name, entry, 'json', 'mc-map-update', f"Map entry ({x_block}, {y_block}, {z_block})", '', {}
    )
    
    if not success:
        return {
            "status": "failed",
            "reason": f"Failed to create map Note: {error_msg}",
            "text": f"Failed to create map Note: {error_msg}",
            "format": "text",
            "char_count": 0
        }
    
    # Add Note to Collection
    success, item_count, error_msg = resource_manager.add_to_collection(
        map_collection_id, note_id, agent_name, 'add', None
    )
    
    if not success:
        return {
            "status": "failed",
            "reason": f"Failed to add Note to Collection: {error_msg}",
            "text": f"Failed to add Note to Collection: {error_msg}",
            "format": "text",
            "char_count": 0
        }
    
    # Mark Collection as persistent
    resource_manager.mark_persistent(map_collection_id, agent_name)
    
    result_text = f"Map updated: ({x_block}, {y_block}, {z_block}) - {item_count} observations total"
    
    return {
        "status": "success",
        "value": result_text,
        "text": result_text,  # Keep for backward compatibility
        "format": "text",
        "metadata": {
            "map_name": map_name,
            "map_id": map_collection_id,
            "note_id": note_id,
            "location": {"x": x_block, "y": y_block, "z": z_block},
            "total_observations": item_count
        },
        "char_count": len(result_text)
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

