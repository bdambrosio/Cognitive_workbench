"""
Minecraft map-query tool.
Queries persistent spatial memory for strategic decision-making.
"""

import logging
import json
import math
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


def _distance(x1: float, y1: float, z1: float, x2: float, y2: float, z2: float) -> float:
    """Calculate Euclidean distance between two 3D points."""
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)


def tool(input_value=None, **kwargs):
    """
    Query persistent spatial map for strategic information.
    
    Args:
        input_value: Ignored
        query: Query type - "location", "property", "unexplored", "waypoint", "nearest"
        x: X coordinate (for location/property queries)
        y: Y coordinate (for location/property queries)
        z: Z coordinate (for location/property queries)
        property: Property path (e.g., "geom.stair", "aff.step") for property queries
        value: Property value to match (for property queries)
        waypoint: Waypoint name (for waypoint queries)
        from_x: Starting X coordinate (for nearest/unexplored queries)
        from_y: Starting Y coordinate (for nearest/unexplored queries)
        from_z: Starting Z coordinate (for nearest/unexplored queries)
        map_name: Optional map Collection name (default: "minecraft_map")
        resource_manager: Resource manager instance (from executor)
        agent_name: Agent name (for creating resources)
        
    Returns:
        Dict with query results.
    """
    resource_manager = kwargs.get('resource_manager')
    agent_name = kwargs.get('agent_name', 'system')
    map_name = kwargs.get('map_name', DEFAULT_MAP_NAME)
    query_type = kwargs.get('query', 'location')
    
    if not resource_manager:
        return {
            "status": "failed",
            "reason": "Resource manager not available",
            "text": "Failed to query map: Resource manager not available",
            "format": "text",
            "char_count": 0
        }
    
    # Load map Collection
    map_collection_id = resource_manager.named_collections.get(map_name)
    
    if not map_collection_id:
        return {
            "status": "success",
            "text": "Map not found - no locations explored yet",
            "format": "text",
            "metadata": {
                "map_name": map_name,
                "found": False,
                "results": []
            },
            "char_count": 0
        }
    
    map_resource = resource_manager.get_resource(map_collection_id)
    if not map_resource:
        return {
            "status": "failed",
            "reason": "Map Collection not found",
            "text": "Failed to query map: Collection not found",
            "format": "text",
            "char_count": 0
        }
    
    map_content = map_resource.get('properties', {}).get('content', [])
    if not isinstance(map_content, list):
        map_content = []
    
    results = []
    result_text_parts = []
    
    # Handle different query types
    if query_type == 'location':
        # Query specific location
        x = kwargs.get('x')
        y = kwargs.get('y')
        z = kwargs.get('z')
        
        if x is None or y is None or z is None:
            return {
                "status": "failed",
                "reason": "Location query requires x, y, z coordinates",
                "text": "Failed to query map: Location query requires x, y, z coordinates",
                "format": "text",
                "char_count": 0
            }
        
        x_block = _round_coordinate(x)
        y_block = _round_coordinate(y)
        z_block = _round_coordinate(z)
        
        for entry in map_content:
            if isinstance(entry, dict):
                entry_x = _round_coordinate(entry.get('x', 0))
                entry_y = _round_coordinate(entry.get('y', 0))
                entry_z = _round_coordinate(entry.get('z', 0))
                if entry_x == x_block and entry_y == y_block and entry_z == z_block:
                    results.append(entry)
                    result_text_parts.append(f"Location ({x_block}, {y_block}, {z_block}) found: visited {entry.get('visit_count', 0)} times")
                    break
        
        if not results:
            result_text_parts.append(f"Location ({x_block}, {y_block}, {z_block}) not found in map")
    
    elif query_type == 'property':
        # Query by property value
        property_path = kwargs.get('property')
        property_value = kwargs.get('value')
        
        if not property_path:
            return {
                "status": "failed",
                "reason": "Property query requires property path",
                "text": "Failed to query map: Property query requires property path",
                "format": "text",
                "char_count": 0
            }
        
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
            return {
                "status": "failed",
                "reason": "Waypoint query requires waypoint name",
                "text": "Failed to query map: Waypoint query requires waypoint name",
                "format": "text",
                "char_count": 0
            }
        
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
        from_x = kwargs.get('from_x')
        from_y = kwargs.get('from_y')
        from_z = kwargs.get('from_z')
        
        if from_x is None or from_y is None or from_z is None:
            return {
                "status": "failed",
                "reason": "Unexplored query requires from_x, from_y, from_z coordinates",
                "text": "Failed to query map: Unexplored query requires from_x, from_y, from_z coordinates",
                "format": "text",
                "char_count": 0
            }
        
        # Find locations with low visit_count or missing observations
        candidates = []
        for entry in map_content:
            if isinstance(entry, dict):
                visit_count = entry.get('visit_count', 0)
                observed = entry.get('observed', {})
                # Consider unexplored if visit_count <= 1 or missing key observation data
                if visit_count <= 1 or not observed or not isinstance(observed, dict) or not observed.get('dirs'):
                    x = entry.get('x', 0)
                    y = entry.get('y', 0)
                    z = entry.get('z', 0)
                    dist = _distance(from_x, from_y, from_z, x, y, z)
                    candidates.append((dist, entry))
        
        # Sort by distance
        candidates.sort(key=lambda x: x[0])
        
        # Return nearest few
        results = [entry for dist, entry in candidates[:5]]
        
        if results:
            result_text_parts.append(f"Found {len(results)} unexplored locations nearby")
        else:
            result_text_parts.append("No unexplored locations found")
    
    elif query_type == 'nearest':
        # Find nearest location matching criteria
        from_x = kwargs.get('from_x')
        from_y = kwargs.get('from_y')
        from_z = kwargs.get('from_z')
        property_path = kwargs.get('property')
        property_value = kwargs.get('value')
        
        if from_x is None or from_y is None or from_z is None:
            return {
                "status": "failed",
                "reason": "Nearest query requires from_x, from_y, from_z coordinates",
                "text": "Failed to query map: Nearest query requires from_x, from_y, from_z coordinates",
                "format": "text",
                "char_count": 0
            }
        
        # Filter by property if specified
        candidates = []
        for entry in map_content:
            if isinstance(entry, dict):
                # Check property filter if specified
                if property_path and property_value is not None:
                    observed = entry.get('observed', {})
                    if isinstance(observed, dict):
                        parts = property_path.split('.')
                        value = observed
                        for part in parts:
                            if isinstance(value, dict):
                                value = value.get(part)
                            else:
                                value = None
                                break
                        if value != property_value:
                            continue
                
                x = entry.get('x', 0)
                y = entry.get('y', 0)
                z = entry.get('z', 0)
                dist = _distance(from_x, from_y, from_z, x, y, z)
                candidates.append((dist, entry))
        
        # Sort by distance
        candidates.sort(key=lambda x: x[0])
        
        # Return nearest
        if candidates:
            results = [candidates[0][1]]
            result_text_parts.append(f"Nearest location: ({results[0].get('x')}, {results[0].get('y')}, {results[0].get('z')}) at distance {candidates[0][0]:.2f}")
        else:
            result_text_parts.append("No matching locations found")
    
    else:
        return {
            "status": "failed",
            "reason": f"Unknown query type: {query_type}",
            "text": f"Failed to query map: Unknown query type '{query_type}'",
            "format": "text",
            "char_count": 0
        }
    
    # Format results
    result_text = "\n".join(result_text_parts) if result_text_parts else "Query completed"
    
    return {
        "status": "success",
        "text": result_text,
        "format": "text",
        "metadata": {
            "map_name": map_name,
            "query_type": query_type,
            "found": len(results) > 0,
            "result_count": len(results),
            "results": results[:10]  # Limit to first 10 for metadata
        },
        "char_count": len(result_text)
    }


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    # Test with mock data
    result = tool(
        query="location",
        x=-112,
        y=71,
        z=-123,
        resource_manager=None,
        agent_name="test"
    )
    print(result)

