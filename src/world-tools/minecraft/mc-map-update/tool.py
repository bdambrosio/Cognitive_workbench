"""
Minecraft map-update tool.
Converts ephemeral observation data into persistent spatial memory.
Updates the cell-based SpatialMap with observation data.
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
    
    Updates the cell-based SpatialMap with observation data from mc-observe-blocks.
    Automatically checks if current location is already mapped before observing.
    
    Args:
        input_value: Observation data from mc-observe-blocks (Note ID or dict)
        observation: Alternative parameter name for observation data
        radius: Optional radius for mc-observe-blocks (default: 7, only used when observation is not provided)
        blocks_radius: Alternative parameter name for radius
        x, y, z: Optional coordinates (extracted from observation if not provided)
        
    Note: Coordinates are automatically extracted from observation['pose'] field.
          If pose is missing, mc-status is queried as fallback.
          If no observation provided and current location is already mapped, auto-observation is skipped.
          
        resource_manager: Resource manager instance (from executor)
        agent_name: Agent name (for creating resources)
        world_name: World name (default: 'minecraft')
        
    Returns:
        Dict with result using uniform return format.
    """
    executor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}
    
    resource_manager = kwargs.get('resource_manager')
    agent_name = kwargs.get('agent_name', 'system')
    world_name = kwargs.get('world_name', 'minecraft')
    
    if not resource_manager:
        return executor._create_uniform_return(
            'failed',
            value="Resource manager not available",
            reason="no_resource_manager"
        )
    
    # Get base_dir from resource_manager if available (needed for SpatialMap)
    base_dir = None
    if resource_manager and hasattr(resource_manager, 'base_dir'):
        base_dir = resource_manager.base_dir
    
    spatial_map = _get_spatial_map(agent_name, world_name, base_dir)
    
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
    
    # If no observation data provided, check if we need to observe
    if not observation_data:
        # Try to get current coordinates first
        current_x = kwargs.get('x')
        current_y = kwargs.get('y')
        current_z = kwargs.get('z')
        
        # If coordinates not provided, get from mc-status
        if current_x is None or current_y is None or current_z is None:
            try:
                status_result = executor.execute_action_with_log({"type": "mc-status"}, "mc-map-update")
                if status_result.get("status") == "success":
                    status_data = status_result.get("data", {})
                    position = status_data.get("position")
                    if isinstance(position, dict):
                        current_x = current_x or position.get('x')
                        current_y = current_y or position.get('y')
                        current_z = current_z or position.get('z')
                    elif isinstance(position, (list, tuple)) and len(position) >= 3:
                        current_x = current_x or position[0]
                        current_y = current_y or position[1]
                        current_z = current_z or position[2]
            except Exception as e:
                logger.warning(f"Failed to query mc-status for coordinates: {e}")
        
        # Check if current location is already mapped
        if current_x is not None and current_z is not None:
            x_block_check = _round_coordinate(current_x)
            z_block_check = _round_coordinate(current_z)
            existing_cell = spatial_map.get_cell(x_block_check, z_block_check)
            
            # If cell exists and has been observed, skip auto-observation
            if existing_cell and existing_cell.get('observability', {}).get('last_observed_at'):
                logger.info(f"Location ({x_block_check}, {z_block_check}) already mapped, skipping auto-observation")
                # Still proceed with update using existing cell data if needed
                # But for now, we'll still observe to get fresh data
                # (User can explicitly pass observation to skip)
        
        # Automatically invoke mc-observe-blocks
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
    
    # Update cells from observation
    cells_updated = 0
    if isinstance(observation_data, dict):
        cells_updated = spatial_map.update_cell_from_observation(
            x, y, z, observation_data, update_reason="mc-observe-blocks"
        )
        # Save after update
        spatial_map.save()
    
    # Get spatial map stats
    map_stats = spatial_map.get_stats()
    
    result_text = f"Map updated: ({x_block}, {y_block}, {z_block}) - {map_stats['cell_count']} cells ({cells_updated} updated)"
    
    # Extract metadata fields for extra
    extra_metadata = {
        "location": {"x": x_block, "y": y_block, "z": z_block},
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
