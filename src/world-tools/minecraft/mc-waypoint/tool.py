"""
Minecraft waypoint tool.
Labels coordinates for reasoning about spatial relationships.
"""

import logging
import os
import sys
from pathlib import Path
from typing import Any, Dict, Optional

logger = logging.getLogger(__name__)

# Import nav_core helper for coordinate conversion
_THIS_DIR = os.path.dirname(__file__)
_NAV_CORE_DIR = os.path.abspath(os.path.join(_THIS_DIR, ".."))
if _NAV_CORE_DIR not in sys.path:
    sys.path.insert(0, _NAV_CORE_DIR)
from nav_core import dx_dy_dz_to_absolute

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
        dx: Optional agent-relative X offset from agent (defaults to 0 = current position)
        dy: Optional agent-relative Y offset from agent (defaults to 0 = current position)
        dz: Optional agent-relative Z offset from agent (defaults to 0 = current position)
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
    
    # Get dx, dy, dz (default to 0,0,0 = current position)
    dx = kwargs.get('dx', 0)
    dy = kwargs.get('dy', 0)
    dz = kwargs.get('dz', 0)
    
    dx = float(dx)
    dy = float(dy)
    dz = float(dz)
    
    # Get agent position to convert dx,dy,dz to absolute
    try:
        status_result = executor.execute_action_with_log({"type": "mc-status"}, "mc-waypoint")
        if status_result.get("status") != "success":
            return executor._create_uniform_return(
                'failed',
                value="Failed to get agent position for coordinate conversion",
                reason="status_failed"
            )
        status_data = status_result.get("data", {})
        agent_pos = status_data.get("position", {})
        agent_yaw = status_data.get("yaw", 0.0)
        if not isinstance(agent_pos, dict):
            return executor._create_uniform_return(
                'failed',
                value="Invalid agent position data",
                reason="invalid_position"
            )
    except Exception as e:
        logger.error(f"mc-waypoint: Failed to get agent position: {e}")
        return executor._create_uniform_return(
            'failed',
            value=f"Failed to get agent position: {e}",
            reason="status_failed"
        )
    
    # Convert agent-relative dx,dy,dz to absolute block coordinates
    abs_x, abs_y, abs_z = dx_dy_dz_to_absolute(dx, dy, dz, agent_pos, yaw=agent_yaw)
    
    # Round coordinates to block positions
    x_block = abs_x
    z_block = abs_z
    y_block = abs_y  # Keep Y for display
    
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
    
    # Format result text with relative coordinates
    if dx == 0 and dy == 0 and dz == 0:
        result_text = f"Waypoint '{waypoint_name}' added at [0,0,0] (current position)"
    else:
        result_text = f"Waypoint '{waypoint_name}' added at [dx={dx},dy={dy},dz={dz}]"
    if len(waypoints) > 1:
        result_text += f" (total waypoints at this location: {len(waypoints)})"
    
    # Build structured data dict (store absolute for spatial_map, but also include relative)
    structured_data = {
        "waypoint": waypoint_name,
        "location": {"x": x_block, "y": y_block, "z": z_block},
        "relative": {"dx": dx, "dy": dy, "dz": dz},
        "all_waypoints": waypoints
    }
    
    return executor._create_uniform_return('success', value=result_text, extra=structured_data)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    # Test with mock data
    result = tool(
        name="Base_Camp",
        dx=0,
        dy=0,
        dz=0,
        resource_manager=None,
        agent_name="test"
    )
    print(result)

