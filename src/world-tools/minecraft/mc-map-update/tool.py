"""
Minecraft map-update tool.
Converts ephemeral observation data into persistent spatial memory.
Updates both the Collection-based log and the cell-based SpatialMap.
"""

import logging
import json
import os
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

logger = logging.getLogger(__name__)

# Default map name
DEFAULT_MAP_NAME = "minecraft_map"

# Import SpatialMap
try:
    from ..spatial_map import SpatialMap
except ImportError:
    # Fallback for direct execution
    import sys
    sys.path.insert(0, str(Path(__file__).parent.parent))
    from spatial_map import SpatialMap


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


# Cache for SpatialMap instances (one per agent)
_spatial_map_cache: Dict[str, SpatialMap] = {}


def _get_spatial_map(agent_name: str, world_name: str, base_dir: Optional[Path] = None) -> SpatialMap:
    """Get or create SpatialMap for agent."""
    cache_key = f"{agent_name}:{world_name}"
    if cache_key not in _spatial_map_cache:
        _spatial_map_cache[cache_key] = SpatialMap(agent_name, world_name, base_dir)
    return _spatial_map_cache[cache_key]


def tool(input_value=None, **kwargs):
    """
    Update persistent spatial map with observation data.
    
    Updates both:
    1. Collection-based log (raw observations as Notes)
    2. Cell-based SpatialMap (compiled spatial memory)
    
    Args:
        input_value: Observation data from mc-observe-blocks (Note ID or dict)
        observation: Alternative parameter name for observation data
        map_name: Optional map Collection name (default: agent-specific)
        
    Note: Coordinates are automatically extracted from observation['pose'] field.
          If pose is missing, mc-status is queried as fallback.
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
    world_name = kwargs.get('world_name', 'minecraft')
    
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
        return executor._create_uniform_return(
            'failed',
            value="Resource manager not available",
            reason="no_resource_manager"
        )
    
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
    
    # If no observation data provided, automatically invoke mc-observe-blocks
    if not observation_data:
        # Optionally allow radius parameter to be passed through
        radius = kwargs.get('radius') or kwargs.get('blocks_radius')
        observe_action = {"type": "mc-observe-blocks"}
        if radius is not None:
            observe_action["radius"] = radius
        
        obs_result = executor.execute_action_with_log(observe_action, "mc-map-update")
        
        if obs_result.get("status") != "success":
            error_msg = obs_result.get("value", "Unknown error")
            return executor._create_uniform_return(
                'failed',
                value=f"Failed to obtain observation data: {error_msg}",
                reason="observation_failed"
            )
        
        # Extract structured observation data from result
        observation_data = obs_result.get("data", {})
        
        # If data is still empty, try value field as fallback
        if not observation_data:
            observation_data = obs_result.get("value")
        
        if not observation_data:
            return executor._create_uniform_return(
                'failed',
                value="Failed to extract observation data from mc-observe-blocks result",
                reason="observation_extraction_failed"
            )
    
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
                return executor._create_uniform_return(
                    'failed',
                    value="Could not parse observation data",
                    reason="parse_error"
                )
    
    # Extract coordinates from observation data
    # mc-observe-blocks always includes pose with x, y, z coordinates
    x = None
    y = None
    z = None
    
    if isinstance(observation_data, dict):
        # Extract from pose field (primary source - mc-observe-blocks always includes this)
        if 'pose' in observation_data:
            pose = observation_data['pose']
            if isinstance(pose, dict):
                x = pose.get('x')
                y = pose.get('y')
                z = pose.get('z')
            elif isinstance(pose, str):
                # Parse "pose: (x.xx,y.yy,z.zz, yaw:y.yy, pitch:p.pp)" format (legacy text)
                import re
                match = re.search(r'\(([-\d.]+),([-\d.]+),([-\d.]+)', pose)
                if match:
                    x, y, z = float(match.group(1)), float(match.group(2)), float(match.group(3))
        
        # Fallback: check direct x/y/z fields (legacy)
        if x is None:
            x = observation_data.get('x')
        if y is None:
            y = observation_data.get('y')
        if z is None:
            z = observation_data.get('z')
    
    # If still missing, query mc-status as last resort
    if x is None or y is None or z is None:
        try:
            status_result = executor.execute_action_with_log({"type": "mc-status"}, "mc-map-update")
            if status_result.get("status") == "success":
                status_data = status_result.get("data", {})
                position = status_data.get("position")
                if isinstance(position, dict):
                    x = position.get('x')
                    y = position.get('y')
                    z = position.get('z')
                elif isinstance(position, (list, tuple)) and len(position) >= 3:
                    x, y, z = position[0], position[1], position[2]
        except Exception as e:
            logger.warning(f"Failed to query mc-status for coordinates: {e}")
    
    if x is None or y is None or z is None:
        return executor._create_uniform_return(
            'failed',
            value="Could not determine coordinates from observation or mc-status",
            reason="missing_coordinates"
        )
    
    # Round coordinates to block positions
    x_block = _round_coordinate(x)
    y_block = _round_coordinate(y)
    z_block = _round_coordinate(z)
    
    # =========================================================================
    # Update Cell-based SpatialMap
    # =========================================================================
    
    # Get base_dir from resource_manager if available
    base_dir = None
    if resource_manager and hasattr(resource_manager, 'base_dir'):
        base_dir = resource_manager.base_dir
    
    spatial_map = _get_spatial_map(agent_name, world_name, base_dir)
    
    # Update cells from observation
    cells_updated = 0
    if isinstance(observation_data, dict):
        cells_updated = spatial_map.update_cell_from_observation(
            x, y, z, observation_data, update_reason="mc-observe-blocks"
        )
        # Save after update
        spatial_map.save()
    
    # =========================================================================
    # Update Collection-based Log (existing behavior)
    # =========================================================================
    
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
            return executor._create_uniform_return(
                'failed',
                value=f"Failed to create map Collection: {error_msg}",
                reason="collection_creation_failed"
            )
    
    # Prepare observation summary (extract structured data if available)
    # Note: observation_data is data['value'] from the Note (which contains structured dict from mc-observe-blocks)
    # Structure: {'pose': {...}, 'support': {...}, 'blocks': {...}, 'geom': {...}, 'aff': {...}, ...}
    observed_data = {}
    if isinstance(observation_data, dict):
        # Check if this is the structured observation dict (has pose, support, blocks, etc.)
        has_structured_fields = any(field in observation_data for field in ['pose', 'support', 'blocks', 'geom', 'aff'])
        
        if has_structured_fields:
            # Extract structured fields from dict (current format - structured data is in data['value'])
            for field in ['pose', 'dirs', 'support', 'clear', 'blocks', 'geom', 'aff', 'conf', 'note']:
                if field in observation_data:
                    observed_data[field] = observation_data[field]
        else:
            # Check if nested under 'value' (legacy or alternative structure)
            if 'value' in observation_data and isinstance(observation_data['value'], dict):
                value_dict = observation_data['value']
                for field in ['pose', 'dirs', 'support', 'clear', 'blocks', 'geom', 'aff', 'conf', 'note']:
                    if field in value_dict:
                        observed_data[field] = value_dict[field]
            
            # Check metadata (legacy format where fields were nested)
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
        return executor._create_uniform_return(
            'failed',
            value=f"Failed to create map Note: {error_msg}",
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
    
    # Get spatial map stats
    map_stats = spatial_map.get_stats()
    
    result_text = f"Map updated: ({x_block}, {y_block}, {z_block}) - {item_count} observations, {map_stats['cell_count']} cells ({cells_updated} updated)"
    
    # Extract metadata fields for extra
    extra_metadata = {
        "map_name": map_name,
        "map_id": map_collection_id,
        "note_id": note_id,
        "location": {"x": x_block, "y": y_block, "z": z_block},
        "total_observations": item_count,
        "spatial_map": {
            "cells_updated": cells_updated,
            "total_cells": map_stats['cell_count'],
            "bounds": map_stats.get('bounds')
        }
    }
    
    return executor._create_uniform_return('success', value=result_text, extra=extra_metadata)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    # Test with mock data
    result = tool(
        observation={"x": -112.12, "y": 71.0, "z": -123.67, "observed": {"note": "test"}},
        resource_manager=None,
        agent_name="test"
    )
    print(result)
