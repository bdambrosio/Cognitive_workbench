---
name: mc-map-query
type: python
description: "Queries persistent spatial memory for strategic decision-making. Enables 'Have I been here before?', 'Where is the nearest unexplored edge?', 'Where was that staircase?'"
schema_hint:
  value: "ignored"
  query: "query type: 'location', 'property', 'unexplored', 'waypoint', 'nearest'"
  x: "x coordinate (for location queries)"
  y: "y coordinate (for location queries)"
  z: "z coordinate (for location queries)"
  property: "property path (e.g., 'geom.stair', 'aff.step') for property/nearest queries"
  value: "property value to match (for property/nearest queries)"
  waypoint: "waypoint name (for waypoint queries)"
  from_x: "starting x coordinate (for nearest/unexplored queries)"
  from_y: "starting y coordinate (for nearest/unexplored queries)"
  from_z: "starting z coordinate (for nearest/unexplored queries)"
  map_name: "optional map Collection name (default: '<agent_name>-minecraft_map', computed automatically)"
  out: "$variable"
examples:
  - '{"type":"mc-map-query","query":"location","x":-112,"y":71,"z":-123,"out":"$loc"}'
  - '{"type":"mc-map-query","query":"property","property":"geom.stair","value":true,"out":"$stairs"}'
  - '{"type":"mc-map-query","query":"unexplored","from_x":-112,"from_y":71,"from_z":-123,"out":"$next"}'
  - '{"type":"mc-map-query","query":"waypoint","waypoint":"Base_Camp","out":"$base"}'
  - '{"type":"mc-map-query","query":"nearest","from_x":-112,"from_y":71,"from_z":-123,"property":"geom.stair","value":true,"out":"$nearest_stair"}'
---

# Minecraft Map-Query Tool

## Input
- `query`: Query type - one of:
  - `"location"`: Check if specific coordinate exists in map
  - `"property"`: Find locations matching property value
  - `"unexplored"`: Find nearest unexplored locations
  - `"waypoint"`: Find location by waypoint name
  - `"nearest"`: Find nearest location matching criteria
- Coordinate parameters: `x`, `y`, `z` (for location queries)
- Property parameters: `property` (path like `"geom.stair"`), `value` (to match)
- Waypoint parameter: `waypoint` (name string)
- From parameters: `from_x`, `from_y`, `from_z` (starting position for nearest/unexplored)
- `map_name`: Optional map Collection name (default: `<agent_name>-minecraft_map`, computed automatically)

## Output
- Note ID (bound to `out` variable) containing:
  - `text`: Human-readable query results
  - `metadata`: Structured results including:
    - `query_type`: Query type executed
    - `found`: Boolean indicating if results found
    - `result_count`: Number of results
    - `results`: Array of matching location entries (up to 10)

## Query Types

### 1. Location Query
Check if a specific coordinate has been explored:
```json
{"type":"mc-map-query","query":"location","x":-112,"y":71,"z":-123,"out":"$loc"}
```
Returns entry if location exists, or empty if not found.

### 2. Property Query
Find locations matching a property value:
```json
{"type":"mc-map-query","query":"property","property":"geom.stair","value":true,"out":"$stairs"}
```
Searches `observed.geom.stair` field. Returns all matching locations.

### 3. Unexplored Query
Find nearest unexplored locations (low visit_count or missing observations):
```json
{"type":"mc-map-query","query":"unexplored","from_x":-112,"from_y":71,"from_z":-123,"out":"$next"}
```
Returns up to 5 nearest locations with visit_count <= 1 or missing observation data.

### 4. Waypoint Query
Find location by waypoint name:
```json
{"type":"mc-map-query","query":"waypoint","waypoint":"Base_Camp","out":"$base"}
```
Searches `waypoints` array in entries. Returns all locations with matching waypoint.

### 5. Nearest Query
Find nearest location matching property criteria:
```json
{"type":"mc-map-query","query":"nearest","from_x":-112,"from_y":71,"from_z":-123,"property":"geom.stair","value":true,"out":"$nearest_stair"}
```
Calculates distance from starting position and returns nearest matching location.

## Common Workflows

**Check if location explored:**
```json
{"type":"mc-status","out":"$status"}
{"type":"mc-map-query","query":"location","x":-112,"y":71,"z":-123,"out":"$loc"}
```

**Find unexplored area:**
```json
{"type":"mc-status","out":"$status"}
{"type":"mc-map-query","query":"unexplored","from_x":-112,"from_y":71,"from_z":-123,"out":"$next"}
```

**Find waypoint:**
```json
{"type":"mc-map-query","query":"waypoint","waypoint":"Base_Camp","out":"$base"}
```

**Find nearest staircase:**
```json
{"type":"mc-map-query","query":"nearest","from_x":-112,"from_y":71,"from_z":-123,"property":"geom.stair","value":true,"out":"$stair"}
```

## Cognitive Contract

- **Strategic decisions**: Enables "Go North" rather than "Wall in front, turn Right"
- **Spatial memory**: Answers "Have I been here before?"
- **Path finding**: Locates waypoints and landmarks
- **Exploration planning**: Identifies unexplored edges for systematic exploration
- **Property search**: Finds locations with specific geometry or affordances

## Important Notes

- **Property paths**: Use dot notation for nested properties (e.g., `"geom.stair"`, `"aff.step"`)
- **Distance calculation**: Uses Euclidean distance in 3D space
- **Result limits**: Metadata includes up to 10 results; use `load` to access full Collection if needed
- **Empty map**: Returns success with empty results if map doesn't exist yet
- **Block coordinates**: All coordinates are rounded to integer block positions

