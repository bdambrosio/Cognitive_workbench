"""
Minecraft map-query tool.
Queries persistent spatial memory for strategic decision-making.
Supports both Collection-based queries and cell-based SpatialMap queries.
"""

import logging
import json
import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

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


def _distance(x1: float, y1: float, z1: float, x2: float, y2: float, z2: float) -> float:
    """Calculate Euclidean distance between two 3D points."""
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)


def _distance_2d(x1: float, z1: float, x2: float, z2: float) -> float:
    """Calculate Euclidean distance between two 2D points."""
    return math.sqrt((x2 - x1)**2 + (z2 - z1)**2)


# Cache for SpatialMap instances (one per agent)
_spatial_map_cache: Dict[str, SpatialMap] = {}


def _get_spatial_map(agent_name: str, world_name: str, base_dir: Optional[Path] = None) -> SpatialMap:
    """Get or create SpatialMap for agent."""
    cache_key = f"{agent_name}:{world_name}"
    if cache_key not in _spatial_map_cache:
        _spatial_map_cache[cache_key] = SpatialMap(agent_name, world_name, base_dir)
    return _spatial_map_cache[cache_key]


def _compile_map_forward(map_collection_id: str, resource_manager) -> List[Dict]:
    """
    Compile forward: Load all Notes from Collection, deduplicate by (x,y,z), keep latest.
    
    Args:
        map_collection_id: Collection ID containing map Notes
        resource_manager: Resource manager instance
        
    Returns:
        List of compiled map entries (one per unique location, latest observation)
    """
    if not resource_manager:
        return []
    
    # Load Collection
    map_resource = resource_manager.get_resource(map_collection_id)
    if not map_resource:
        return []
    
    # Get Note IDs from Collection content
    note_ids = map_resource.get('properties', {}).get('content', [])
    if not isinstance(note_ids, list):
        return []
    
    # Load all Notes and compile
    location_map = {}  # (x,y,z) -> entry with latest timestamp
    waypoint_map = {}  # (x,y,z) -> set of waypoint names
    
    for note_id in note_ids:
        if not isinstance(note_id, str) or not note_id.startswith('Note_'):
            continue
        
        note_resource = resource_manager.get_resource(note_id)
        if not note_resource:
            continue
        
        entry = note_resource.get('properties', {}).get('content')
        if not isinstance(entry, dict):
            continue
        
        # Extract coordinates
        x = _round_coordinate(entry.get('x', 0))
        y = _round_coordinate(entry.get('y', 0))
        z = _round_coordinate(entry.get('z', 0))
        key = (x, y, z)
        
        # Collect waypoints
        waypoints = entry.get('waypoints', [])
        if isinstance(waypoints, list) and waypoints:
            if key not in waypoint_map:
                waypoint_map[key] = set()
            waypoint_map[key].update(waypoints)
        
        # Get timestamp
        timestamp = entry.get('timestamp', '')
        
        # Keep latest entry for each location
        if key not in location_map:
            location_map[key] = entry
        else:
            existing_timestamp = location_map[key].get('timestamp', '')
            if timestamp > existing_timestamp:
                location_map[key] = entry
    
    # Merge waypoints into compiled entries
    compiled = []
    for key, entry in location_map.items():
        compiled_entry = entry.copy()
        if key in waypoint_map:
            compiled_entry['waypoints'] = sorted(list(waypoint_map[key]))
        compiled.append(compiled_entry)
    
    return compiled


def _format_cell_summary(cell: Dict[str, Any]) -> str:
    """Format a cell for human-readable output."""
    cell_id = cell.get('cell_id', {})
    x, z = cell_id.get('x', 0), cell_id.get('z', 0)
    support = cell.get('support', {})
    walkable = "walkable" if support.get('walkable') else "blocked"
    movement = support.get('movement_class', 'unknown')
    conf = cell.get('observability', {}).get('confidence', 0)
    return f"({x},{z}): {walkable}, {movement}, conf={conf:.2f}"


def _format_cells_list(cells: List[Dict[str, Any]], max_items: int = 10) -> str:
    """Format a list of cells for human-readable output."""
    if not cells:
        return "none"
    summaries = [_format_cell_summary(c) for c in cells[:max_items]]
    result = "\n  ".join(summaries)
    if len(cells) > max_items:
        result += f"\n  ... and {len(cells) - max_items} more"
    return result


# ============================================================================
# Query Type Handlers
# ============================================================================

QUERY_TYPES = [
    # Coverage / Observability
    "cells-unobserved",
    "cells-low-confidence", 
    "frontier-cells",
    "cells-observed-from-distance",
    "cells-stale",
    # Reachability / Locomotion
    "cells-reachable",
    "cells-blocked",
    "cells-requiring-climb",
    "cells-with-drop-risk",
    # Safety / Survival
    "cells-safe-to-stand",
    "cells-high-hazard",
    "cells-escape-nodes",
    # Resources (basic)
    "cells-with-resource",
    "cells-water-source",
    # Multi-Objective
    "cells-candidate-waypoints",
    "cells-nearest",
    "cells-worth-revisit",
    # Legacy queries
    "location",
    "property",
    "waypoint",
    "unexplored",
    "nearest",
    "stats"
]


def tool(input_value=None, **kwargs):
    """
    Query persistent spatial map for strategic information.
    
    Supports both cell-based SpatialMap queries and legacy Collection queries.
    
    Args:
        input_value: Ignored
        query: Query type (see QUERY_TYPES list)
        
        For spatial queries:
        x, z: Center coordinates (from_x, from_z also accepted)
        radius: Search radius (default: 10)
        threshold: Confidence threshold for low-confidence query (default: 0.5)
        max_age: Maximum age in seconds for stale query (default: 300)
        max_delta_y: Max Y change for reachable query (default: 1)
        min_dist: Minimum distance for observed-from-distance query
        resource_type: Resource type for cells-with-resource query
        
        For legacy queries:
        y: Y coordinate (for location queries)
        property: Property path for property queries
        value: Property value to match
        waypoint: Waypoint name for waypoint queries
        
        map_name: Optional map Collection name (default: agent-specific)
        resource_manager: Resource manager instance (from executor)
        agent_name: Agent name
        
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
    map_name = kwargs.get('map_name') or default_map_name
    query_type = kwargs.get('query', 'stats')
    
    if not resource_manager:
        return executor._create_uniform_return(
            'failed',
            value="Resource manager not available",
            reason="no_resource_manager"
            reason="no_resource_manager"
        )
    
    # Get base_dir from resource_manager if available
    base_dir = None
    if resource_manager and hasattr(resource_manager, 'base_dir'):
        base_dir = resource_manager.base_dir
    
    # Get SpatialMap instance
    spatial_map = _get_spatial_map(agent_name, world_name, base_dir)
    
    # Extract common parameters
    x = kwargs.get('x') or kwargs.get('from_x')
    z = kwargs.get('z') or kwargs.get('from_z')
    y = kwargs.get('y') or kwargs.get('from_y')
    radius = int(kwargs.get('radius', 10))
    
    # Convert coordinates if provided
    if x is not None:
        x = _round_coordinate(x)
    if z is not None:
        z = _round_coordinate(z)
    if y is not None:
        y = _round_coordinate(y)
    
    results = []
    result_text_parts = []
    
    # ========================================================================
    # Cell-based Spatial Queries (new)
    # ========================================================================
    
    if query_type == 'stats':
        # Return map statistics
        stats = spatial_map.get_stats()
        result_text_parts.append(f"Spatial Map Statistics:")
        result_text_parts.append(f"  Total cells: {stats['cell_count']}")
        if stats.get('bounds'):
            b = stats['bounds']
            result_text_parts.append(f"  Bounds: X[{b['min_x']}..{b['max_x']}], Z[{b['min_z']}..{b['max_z']}]")
            result_text_parts.append(f"  Size: {b['range_x']}x{b['range_z']}")
        result_text_parts.append(f"  Walkable: {stats['walkable_count']}")
        result_text_parts.append(f"  Hazards: {stats['hazard_count']}")
        result_text_parts.append(f"  Resources: {stats['resource_count']}")
        
        return executor._create_uniform_return(
            'success',
            value="\n".join(result_text_parts),
            extra={"query_type": query_type, "stats": stats}
        )
    
    elif query_type == 'cells-unobserved':
        if x is None or z is None:
            return executor._create_uniform_return(
                'failed',
                value="cells-unobserved requires x, z coordinates",
                reason="missing_coordinates"
            )
        
        unobserved = spatial_map.cells_unobserved(x, z, radius)
        result_text_parts.append(f"Unobserved cells within radius {radius} of ({x},{z}): {len(unobserved)}")
        if unobserved:
            coords_str = ", ".join([f"({ux},{uz})" for ux, uz in unobserved[:20]])
            result_text_parts.append(f"  {coords_str}")
            if len(unobserved) > 20:
                result_text_parts.append(f"  ... and {len(unobserved) - 20} more")
        
        return executor._create_uniform_return(
            'success',
            value="\n".join(result_text_parts),
            extra={"query_type": query_type, "result_count": len(unobserved), "results": unobserved[:50]}
        )
    
    elif query_type == 'cells-low-confidence':
        if x is None or z is None:
            return executor._create_uniform_return(
                'failed',
                value="cells-low-confidence requires x, z coordinates",
                reason="missing_coordinates"
            )
        
        threshold = float(kwargs.get('threshold', 0.5))
        cells = spatial_map.cells_low_confidence(x, z, radius, threshold)
        result_text_parts.append(f"Low-confidence cells (< {threshold}) within radius {radius}: {len(cells)}")
        result_text_parts.append(f"  {_format_cells_list(cells)}")
        
        return executor._create_uniform_return(
            'success',
            value="\n".join(result_text_parts),
            extra={"query_type": query_type, "result_count": len(cells), "results": cells[:20]}
        )
    
    elif query_type == 'frontier-cells':
        if x is None or z is None:
            return executor._create_uniform_return(
                'failed',
                value="frontier-cells requires x, z coordinates",
                reason="missing_coordinates"
            )
        
        cells = spatial_map.frontier_cells(x, z, radius)
        result_text_parts.append(f"Frontier cells within radius {radius}: {len(cells)}")
        result_text_parts.append(f"  {_format_cells_list(cells)}")
        
        return executor._create_uniform_return(
            'success',
            value="\n".join(result_text_parts),
            extra={"query_type": query_type, "result_count": len(cells), "results": cells[:20]}
        )
    
    elif query_type == 'cells-observed-from-distance':
        min_dist = float(kwargs.get('min_dist', 2.0))
        cells = spatial_map.cells_observed_from_distance(min_dist)
        result_text_parts.append(f"Cells observed from distance >= {min_dist}: {len(cells)}")
        result_text_parts.append(f"  {_format_cells_list(cells)}")
        
        return executor._create_uniform_return(
            'success',
            value="\n".join(result_text_parts),
            extra={"query_type": query_type, "result_count": len(cells), "results": cells[:20]}
        )
    
    elif query_type == 'cells-stale':
        max_age = float(kwargs.get('max_age', 300))  # Default 5 minutes
        cells = spatial_map.cells_stale(max_age)
        result_text_parts.append(f"Stale cells (age > {max_age}s): {len(cells)}")
        result_text_parts.append(f"  {_format_cells_list(cells)}")
        
        return executor._create_uniform_return(
            'success',
            value="\n".join(result_text_parts),
            extra={"query_type": query_type, "result_count": len(cells), "results": cells[:20]}
        )
    
    elif query_type == 'cells-reachable':
        if x is None or z is None:
            return executor._create_uniform_return(
                'failed',
                value="cells-reachable requires x, z coordinates",
                reason="missing_coordinates"
            )
        
        max_delta_y = int(kwargs.get('max_delta_y', 1))
        cells = spatial_map.cells_reachable(x, z, radius, max_delta_y)
        result_text_parts.append(f"Reachable cells within radius {radius} (max_delta_y={max_delta_y}): {len(cells)}")
        result_text_parts.append(f"  {_format_cells_list(cells)}")
        
        return executor._create_uniform_return(
            'success',
            value="\n".join(result_text_parts),
            extra={"query_type": query_type, "result_count": len(cells), "results": cells[:20]}
        )
    
    elif query_type == 'cells-blocked':
        cells = spatial_map.cells_blocked()
        result_text_parts.append(f"Blocked cells: {len(cells)}")
        result_text_parts.append(f"  {_format_cells_list(cells)}")
        
        return executor._create_uniform_return(
            'success',
            value="\n".join(result_text_parts),
            extra={"query_type": query_type, "result_count": len(cells), "results": cells[:20]}
        )
    
    elif query_type == 'cells-requiring-climb':
        cells = spatial_map.cells_requiring_climb()
        result_text_parts.append(f"Cells requiring climb: {len(cells)}")
        result_text_parts.append(f"  {_format_cells_list(cells)}")
        
        return executor._create_uniform_return(
            'success',
            value="\n".join(result_text_parts),
            extra={"query_type": query_type, "result_count": len(cells), "results": cells[:20]}
        )
    
    elif query_type == 'cells-with-drop-risk':
        cells = spatial_map.cells_with_drop_risk()
        result_text_parts.append(f"Cells with drop risk: {len(cells)}")
        result_text_parts.append(f"  {_format_cells_list(cells)}")
        
        return executor._create_uniform_return(
            'success',
            value="\n".join(result_text_parts),
            extra={"query_type": query_type, "result_count": len(cells), "results": cells[:20]}
        )
    
    elif query_type == 'cells-safe-to-stand':
        if x is None or z is None:
            return executor._create_uniform_return(
                'failed',
                value="cells-safe-to-stand requires x, z coordinates",
                reason="missing_coordinates"
            )
        
        cells = spatial_map.cells_safe_to_stand(x, z, radius)
        result_text_parts.append(f"Safe cells within radius {radius}: {len(cells)}")
        result_text_parts.append(f"  {_format_cells_list(cells)}")
        
        return executor._create_uniform_return(
            'success',
            value="\n".join(result_text_parts),
            extra={"query_type": query_type, "result_count": len(cells), "results": cells[:20]}
        )
    
    elif query_type == 'cells-high-hazard':
        cells = spatial_map.cells_high_hazard()
        result_text_parts.append(f"High-hazard cells: {len(cells)}")
        result_text_parts.append(f"  {_format_cells_list(cells)}")
        
        return executor._create_uniform_return(
            'success',
            value="\n".join(result_text_parts),
            extra={"query_type": query_type, "result_count": len(cells), "results": cells[:20]}
        )
    
    elif query_type == 'cells-escape-nodes':
        if x is None or z is None:
            return executor._create_uniform_return(
                'failed',
                value="cells-escape-nodes requires x, z coordinates",
                reason="missing_coordinates"
            )
        
        cells = spatial_map.cells_escape_nodes(x, z, radius)
        result_text_parts.append(f"Escape nodes within radius {radius}: {len(cells)}")
        result_text_parts.append(f"  {_format_cells_list(cells)}")
        
        return executor._create_uniform_return(
            'success',
            value="\n".join(result_text_parts),
            extra={"query_type": query_type, "result_count": len(cells), "results": cells[:20]}
        )
    
    elif query_type == 'cells-with-resource':
        resource_type = kwargs.get('resource_type', kwargs.get('resource'))
        if not resource_type:
            return executor._create_uniform_return(
                'failed',
                value="cells-with-resource requires resource_type parameter",
                reason="missing_resource_type"
            )
        
        cells = spatial_map.cells_with_resource(resource_type)
        result_text_parts.append(f"Cells with resource '{resource_type}': {len(cells)}")
        result_text_parts.append(f"  {_format_cells_list(cells)}")
        
        return executor._create_uniform_return(
            'success',
            value="\n".join(result_text_parts),
            extra={"query_type": query_type, "result_count": len(cells), "results": cells[:20]}
        )
    
    elif query_type == 'cells-water-source':
        cells = spatial_map.cells_water_source()
        result_text_parts.append(f"Water source cells: {len(cells)}")
        result_text_parts.append(f"  {_format_cells_list(cells)}")
        
        return executor._create_uniform_return(
            'success',
            value="\n".join(result_text_parts),
            extra={"query_type": query_type, "result_count": len(cells), "results": cells[:20]}
        )
    
    elif query_type == 'cells-candidate-waypoints':
        if x is None or z is None:
            return executor._create_uniform_return(
                'failed',
                value="cells-candidate-waypoints requires x, z coordinates",
                reason="missing_coordinates"
            )
        
        require_safe = kwargs.get('require_safe', True)
        require_reachable = kwargs.get('require_reachable', True)
        cells = spatial_map.cells_candidate_waypoints(x, z, radius, require_safe, require_reachable)
        result_text_parts.append(f"Candidate waypoints within radius {radius}: {len(cells)}")
        result_text_parts.append(f"  {_format_cells_list(cells)}")
        
        return executor._create_uniform_return(
            'success',
            value="\n".join(result_text_parts),
            extra={"query_type": query_type, "result_count": len(cells), "results": cells[:20]}
        )
    
    elif query_type == 'cells-nearest':
        if x is None or z is None:
            return executor._create_uniform_return(
                'failed',
                value="cells-nearest requires x, z coordinates",
                reason="missing_coordinates"
            )
        
        # Build predicate from parameters
        predicate_type = kwargs.get('predicate', 'walkable')
        
        if predicate_type == 'walkable':
            predicate = lambda c: c.get('support', {}).get('walkable', False)
        elif predicate_type == 'safe':
            predicate = lambda c: (
                c.get('support', {}).get('walkable', False) and
                not c.get('hazards', {}).get('flags', [])
            )
        elif predicate_type == 'hazard':
            predicate = lambda c: len(c.get('hazards', {}).get('flags', [])) > 0
        elif predicate_type == 'resource':
            predicate = lambda c: len(c.get('resources', {}).get('resources_visible', [])) > 0
        else:
            predicate = lambda c: True  # All cells
        
        max_results = int(kwargs.get('max_results', 10))
        nearest = spatial_map.cells_nearest(x, z, predicate, max_results)
        
        result_text_parts.append(f"Nearest {predicate_type} cells from ({x},{z}):")
        for dist, cell in nearest:
            cell_id = cell.get('cell_id', {})
            cx, cz = cell_id.get('x', 0), cell_id.get('z', 0)
            result_text_parts.append(f"  ({cx},{cz}) at distance {dist:.1f}")
        
        return executor._create_uniform_return(
            'success',
            value="\n".join(result_text_parts),
            extra={"query_type": query_type, "result_count": len(nearest), "results": [{"distance": d, "cell": c} for d, c in nearest]}
        )
    
    elif query_type == 'cells-worth-revisit':
        if x is None or z is None:
            return executor._create_uniform_return(
                'failed',
                value="cells-worth-revisit requires x, z coordinates",
                reason="missing_coordinates"
            )
        
        max_confidence = float(kwargs.get('max_confidence', 0.6))
        cells = spatial_map.cells_worth_revisit(x, z, radius, max_confidence)
        result_text_parts.append(f"Cells worth revisiting within radius {radius}: {len(cells)}")
        result_text_parts.append(f"  {_format_cells_list(cells)}")
        
        return executor._create_uniform_return(
            'success',
            value="\n".join(result_text_parts),
            extra={"query_type": query_type, "result_count": len(cells), "results": cells[:20]}
        )
    
    # ========================================================================
    # Legacy Collection-based Queries
    # ========================================================================
    
    # Load map Collection for legacy queries
    map_collection_id = resource_manager.named_collections.get(map_name)
    
    if not map_collection_id and query_type in ['location', 'property', 'waypoint', 'unexplored', 'nearest']:
        result_text = "Map not found - no locations explored yet"
        extra_metadata = {"map_name": map_name, "found": False, "results": []}
        return executor._create_uniform_return('success', value=result_text, extra=extra_metadata)
    
    # Compile forward: Load all Notes, deduplicate by (x,y,z), keep latest
    if map_collection_id:
        map_content = _compile_map_forward(map_collection_id, resource_manager)
    else:
        map_content = []
    
    if query_type == 'location':
        # Query specific location
        if x is None or y is None or z is None:
            return executor._create_uniform_return(
                'failed',
                value="Location query requires x, y, z coordinates",
                reason="missing_coordinates"
            )
        
        for entry in map_content:
            if isinstance(entry, dict):
                entry_x = _round_coordinate(entry.get('x', 0))
                entry_y = _round_coordinate(entry.get('y', 0))
                entry_z = _round_coordinate(entry.get('z', 0))
                if entry_x == x and entry_y == y and entry_z == z:
                    results.append(entry)
                    result_text_parts.append(f"Location ({x}, {y}, {z}) found: visited {entry.get('visit_count', 0)} times")
                    break
        
        if not results:
            result_text_parts.append(f"Location ({x}, {y}, {z}) not found in map")
    
    elif query_type == 'property':
        # Query by property value
        property_path = kwargs.get('property')
        property_value = kwargs.get('value')
        
        if not property_path:
            return executor._create_uniform_return(
                'failed',
                value="Property query requires property path",
                reason="missing_property"
                reason="missing_property"
            )
        
        # Navigate property path (e.g., "geom.stair" -> entry['observed']['geom']['stair'])
        for entry in map_content:
            if isinstance(entry, dict) and 'observed' in entry:
                observed = entry['observed']
                if isinstance(observed, dict):
                    # Navigate nested path
                    parts = property_path.split('.')
                    value = observed
                    for part in parts:
                        if isinstance(value, dict):
                            value = value.get(part)
                        else:
                            value = None
                            break
                    
                    # Check if value matches
                    if property_value is not None:
                        if value == property_value:
                            results.append(entry)
                    else:
                        # Return all entries with this property set
                        if value is not None:
                            results.append(entry)
        
        if results:
            result_text_parts.append(f"Found {len(results)} locations with {property_path}={property_value}")
        else:
            result_text_parts.append(f"No locations found with {property_path}={property_value}")
    
    elif query_type == 'waypoint':
        # Query by waypoint name
        waypoint_name = kwargs.get('waypoint')
        
        if not waypoint_name:
            return executor._create_uniform_return(
                'failed',
                value="Waypoint query requires waypoint name",
                reason="missing_waypoint"
                reason="missing_waypoint"
            )
        
        for entry in map_content:
            if isinstance(entry, dict):
                waypoints = entry.get('waypoints', [])
                if isinstance(waypoints, list) and waypoint_name in waypoints:
                    results.append(entry)
        
        if results:
            result_text_parts.append(f"Found waypoint '{waypoint_name}' at {len(results)} location(s)")
        else:
            result_text_parts.append(f"Waypoint '{waypoint_name}' not found")
    
    elif query_type == 'unexplored':
        # Find nearest unexplored edge (low visit_count or missing observations)
        from_x = x
        from_y = y
        from_z = z
        
        if from_x is None or from_y is None or from_z is None:
            return executor._create_uniform_return(
                'failed',
                value="Unexplored query requires from_x, from_y, from_z coordinates",
                reason="missing_coordinates"
            )
        
        # Find locations with low visit_count or missing observations
        candidates = []
        for entry in map_content:
            if isinstance(entry, dict):
                visit_count = entry.get('visit_count', 0)
                observed = entry.get('observed', {})
                # Consider unexplored if visit_count <= 1 or missing key observation data
                if visit_count <= 1 or not observed or not isinstance(observed, dict) or not observed.get('dirs'):
                    ex = entry.get('x', 0)
                    ey = entry.get('y', 0)
                    ez = entry.get('z', 0)
                    dist = _distance(from_x, from_y, from_z, ex, ey, ez)
                    candidates.append((dist, entry))
        
        # Sort by distance
        candidates.sort(key=lambda x: x[0])
        
        # Return nearest few
        results = [entry for dist, entry in candidates[:5]]
        
        if results:
            result_text_parts.append(f"Found {len(results)} unexplored locations nearby")
        else:
            result_text_parts.append("No unexplored locations found")
    
    else:
        return executor._create_uniform_return(
            'failed',
            value=f"Unknown query type: {query_type}. Valid types: {', '.join(QUERY_TYPES)}",
            reason="invalid_query_type"
            reason="invalid_query_type"
        )
    
    # Format results
    result_text = "\n".join(result_text_parts) if result_text_parts else "Query completed"
    
    # Extract metadata fields for extra
    extra_metadata = {
        "map_name": map_name,
        "query_type": query_type,
        "found": len(results) > 0,
        "result_count": len(results),
        "results": results[:10]  # Limit to first 10 for metadata
    }
    
    return executor._create_uniform_return('success', value=result_text, extra=extra_metadata)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    # Test with mock data
    result = tool(
        query="stats",
        resource_manager=None,
        agent_name="test"
    )
    print(result)
