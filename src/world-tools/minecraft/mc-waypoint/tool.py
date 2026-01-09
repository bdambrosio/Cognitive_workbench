"""
Minecraft waypoint tool.
Labels coordinates for reasoning about spatial relationships.
"""

import logging
from pathlib import Path
from typing import Any, Dict, Optional

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


# Cache for SpatialMap instances (one per agent)
_spatial_map_cache: Dict[str, SpatialMap] = {}


def _get_spatial_map(agent_name: str, world_name: str, base_dir: Optional[Path] = None) -> SpatialMap:
    """Get or create SpatialMap for agent."""
    cache_key = f"{agent_name}:{world_name}"
    if cache_key not in _spatial_map_cache:
        _spatial_map_cache[cache_key] = SpatialMap(agent_name, world_name, base_dir)
    return _spatial_map_cache[cache_key]


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
    waypoint_name = kwargs.get('name')
    
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
    
    # If coordinates not provided, get from mc-status
    if x is None or y is None or z is None:
        status_result = executor.execute_action_with_log({"type": "mc-status"}, "mc-waypoint")
        if status_result.get("status") == "success":
            status_data = status_result.get("data", {})
            position = status_data.get("position")
            if isinstance(position, dict):
                x = x or position.get('x')
                y = y or position.get('y')
                z = z or position.get('z')
            elif isinstance(position, (list, tuple)) and len(position) >= 3:
                x = x or position[0]
                y = y or position[1]
                z = z or position[2]
    
    if x is None or y is None or z is None:
        return executor._create_uniform_return(
            'failed',
            value="Coordinates required (x, y, z). Provide explicitly or ensure mc-status is available.",
            reason="missing_coordinates"
        )
    
    # Round coordinates to block positions
    x_block = _round_coordinate(x)
    z_block = _round_coordinate(z)
    
    # Get base_dir from resource_manager if available
    base_dir = None
    if resource_manager and hasattr(resource_manager, 'base_dir'):
        base_dir = resource_manager.base_dir
    
    # Get SpatialMap instance
    spatial_map = _get_spatial_map(agent_name, world_name, base_dir)
    
    # Get or create cell at this location
    cell = spatial_map.get_cell(x_block, z_block)
    if not cell:
        # Cell doesn't exist - create empty cell
        # Use SpatialMap's internal empty_cell function via import
        try:
            from ..spatial_map import empty_cell
        except ImportError:
            import sys
            from pathlib import Path
            sys.path.insert(0, str(Path(__file__).parent.parent))
            from spatial_map import empty_cell
        cell = empty_cell(x_block, z_block)
        spatial_map.set_cell(x_block, z_block, cell)
        logger.info(f"Created new cell at ({x_block}, {z_block}) for waypoint")
    
    # Add waypoint to cell
    success = spatial_map.add_waypoint(x_block, z_block, waypoint_name)
    if not success:
        return executor._create_uniform_return(
            'failed',
            value=f"Failed to add waypoint to cell at ({x_block}, {z_block})",
            reason="waypoint_add_failed"
        )
    
    # Save SpatialMap
    spatial_map.save()
    
    # Get updated waypoints list
    waypoints = spatial_map.get_waypoints(x_block, z_block)
    
    result_text = f"Waypoint '{waypoint_name}' added at ({x_block}, {y}, {z_block})"
    if len(waypoints) > 1:
        result_text += f" (total waypoints at this location: {len(waypoints)})"
    
    # Build structured data dict
    structured_data = {
        "waypoint": waypoint_name,
        "location": {"x": x_block, "y": y, "z": z_block},
        "all_waypoints": waypoints
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

