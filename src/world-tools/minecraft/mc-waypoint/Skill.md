---
name: mc-waypoint
type: python
description: "Labels a coordinate with a waypoint name for reasoning about spatial relationships. Learning requires labeled data."
schema_hint:
  value: "ignored"
  name: "waypoint name (required)"
  x: "x coordinate (required)"
  y: "y coordinate (required)"
  z: "z coordinate (required)"
  map_name: "optional map Collection name (default: '<agent_name>-minecraft_map', computed automatically)"
  out: "$variable"
examples:
  - '{"type":"mc-status","out":"$status"}'
  - '{"type":"mc-waypoint","name":"Base_Camp","x":-112,"y":71,"z":-123,"out":"$result"}'
  - '{"type":"mc-waypoint","name":"Pit_Exit_1","x":-110,"y":65,"z":-120,"out":"$result"}'
---

# Minecraft Waypoint Tool

## Input
- `name`: Waypoint name (required) - e.g., `"Base_Camp"`, `"Pit_Exit_1"`
- `x`, `y`, `z`: Coordinates (required) - block position to label
- `map_name`: Optional map Collection name (default: `<agent_name>-minecraft_map`, computed automatically)

## Output
- Note ID (bound to `out` variable) containing:
  - `text`: Success message with waypoint name and location
  - `metadata`: Waypoint information including name, location, map_name, map_id

## How It Works

1. **Loads or creates map Collection**: Looks for named Collection `"minecraft_map"` (or specified `map_name`)
2. **Finds or creates location entry**: For the specified coordinates
3. **Adds waypoint label**: Appends waypoint name to `waypoints` array in entry
4. **Marks as persistent**: Ensures map Collection persists across plans

## Waypoint Storage

Waypoints are stored in the `waypoints` array of location entries:
```json
{
  "x": -112,
  "y": 71,
  "z": -123,
  "waypoints": ["Base_Camp", "Spawn_Point"],
  ...
}
```

Multiple waypoints can be assigned to the same location.

## Common Workflow

**Label current position:**
```json
{"type":"mc-status","out":"$status"}
{"type":"load","target":"$status","out":"$status_content"}
{"type":"refine","target":"$status_content","query":"Extract x, y, z coordinates from position","out":"$coords"}
{"type":"mc-waypoint","name":"Base_Camp","x":-112,"y":71,"z":-123,"out":"$result"}
{"type":"persist","target":"minecraft_map"}
```

**Label specific location:**
```json
{"type":"mc-waypoint","name":"Pit_Exit_1","x":-110,"y":65,"z":-120,"out":"$result"}
```

**Query waypoint later:**
```json
{"type":"mc-map-query","query":"waypoint","waypoint":"Base_Camp","out":"$base_location"}
```

## Cognitive Contract

- **Learning requires labels**: Waypoints enable reasoning about spatial relationships
- **Persistent labels**: Waypoint names persist across plans and sessions
- **Multiple labels**: Same location can have multiple waypoint names
- **Automatic entry creation**: If location doesn't exist in map, creates entry with waypoint
- **Strategic planning**: Enables "Go to Base_Camp" rather than "Go to (-112, 71, -123)"

## Important Notes

- **Coordinates required**: Must provide explicit x, y, z coordinates
- **Block-level precision**: Coordinates are rounded to integer block positions
- **Waypoint names**: Use descriptive names (e.g., `"Base_Camp"`, `"Pit_Exit_1"`)
- **Must persist**: After creating waypoint, use `persist` primitive to ensure map survives plan completion
- **Query by name**: Use `mc-map-query` with `query="waypoint"` to find waypoint locations

## Use Cases

- **Base camp**: Label spawn point or home base
- **Landmarks**: Label important structures (staircases, exits, resources)
- **Navigation**: Create waypoints for path planning
- **Learning**: Build labeled dataset of spatial relationships

