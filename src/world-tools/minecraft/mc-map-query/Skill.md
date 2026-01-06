---
name: mc-map-query
type: python
description: "Queries persistent spatial memory for strategic decision-making. Enables 'Have I been here before?', 'Where is the nearest unexplored edge?', 'Where was that staircase?'"
---

# Minecraft Map Query Tool

Queries persistent spatial memory for strategic decision-making. Enables location queries, property searches, waypoint lookups, and nearest neighbor searches.

## Purpose

Spatial memory queries for navigation, exploration planning, and location recall. Supports multiple query types for different spatial reasoning needs.

## Input

- `query`: Query type - one of `{"location", "property", "unexplored", "waypoint", "nearest"}` (required)
- `x`, `y`, `z`: Coordinates (for location queries)
- `property`: Property path string (e.g., `"geom.stair"`) (for property/nearest queries)
- `value`: Property value to match (for property/nearest queries)
- `waypoint`: Waypoint name string (for waypoint queries)
- `from_x`, `from_y`, `from_z`: Starting coordinates (for nearest/unexplored queries)
- `map_name`: Optional map Collection name (default: `<agent_name>-minecraft_map`)
- `value`: Ignored

## Output

Returns uniform_return format with:
- `value`: Text summary (human-readable query results)
- `data`: Structured data dict (machine-readable). Key fields:
  - `success`: Boolean
  - `query_type`: String
  - `found`: Boolean
  - `result_count`: Integer
  - `results`: Array of location entries (up to 10)

## Behavior & Performance

- Location query: Check if location exists in map
- Property query: Find locations matching property value
- Unexplored query: Find nearest unexplored edge from starting point
- Waypoint query: Lookup waypoint by name
- Nearest query: Find nearest location matching property from starting point

## Guidelines

- Use location query to check if visited before
- Use property query to find locations with specific features (e.g., stairs, pits)
- Use unexplored query for exploration planning
- Use waypoint query to recall named locations
- Use nearest query to find closest matching location
- Results limited to 10 entries for performance

## Usage Examples

Check location:
```json
{"type":"mc-map-query","query":"location","x":-112,"y":71,"z":-123,"out":"$loc"}
```

Find stairs:
```json
{"type":"mc-map-query","query":"property","property":"geom.stair","value":true,"out":"$stairs"}
```

Find unexplored edge:
```json
{"type":"mc-map-query","query":"unexplored","from_x":-112,"from_y":71,"from_z":-123,"out":"$next"}
```

Lookup waypoint:
```json
{"type":"mc-map-query","query":"waypoint","waypoint":"Base_Camp","out":"$base"}
```

Find nearest stair:
```json
{"type":"mc-map-query","query":"nearest","from_x":-112,"from_y":71,"from_z":-123,"property":"geom.stair","value":true,"out":"$nearest_stair"}
```
