---
name: mc-map-update
type: python
description: "Converts ephemeral observation data into persistent spatial memory. Stores observation data in a persistent Collection named 'minecraft_map'."
schema_hint:
  value: "observation data from mc-observe-blocks (Note ID or dict)"
  observation: "observation data from mc-observe-blocks (Note ID or dict, alternative to value)"
  x: "optional x coordinate (defaults to extracting from observation)"
  y: "optional y coordinate (defaults to extracting from observation)"
  z: "optional z coordinate (defaults to extracting from observation)"
  map_name: "optional map Collection name (default: '<agent_name>-minecraft_map', computed automatically)"
  out: "$variable"
examples:
  - '{"type":"mc-observe-blocks","out":"$obs"}'
  - '{"type":"mc-map-update","observation":"$obs","out":"$result"}'
  - '{"type":"mc-map-update","observation":"$obs","x":-112,"y":71,"z":-123,"out":"$result"}'
---

# Minecraft Map-Update Tool

## Input
- `value` or `observation`: Observation data from `mc-observe-blocks` (Note ID or dict)
- `x`, `y`, `z`: Optional explicit coordinates (defaults to extracting from observation)
- `map_name`: Optional map Collection name (default: `<agent_name>-minecraft_map`, computed automatically)

## Output
- Note ID (bound to `out` variable) containing:
  - `text`: Success message with location and total locations count
  - `metadata`: Map information including map_name, map_id, location, total_locations, visit_count

## How It Works

1. **Loads or creates map Collection**: Looks for named Collection `"minecraft_map"` (or specified `map_name`)
2. **Extracts coordinates**: From observation data (structured SUMMARY format or metadata)
3. **Rounds to block coordinates**: Converts float coordinates to integer block positions
4. **Updates or creates entry**: 
   - If location exists: Updates observation data, increments visit_count
   - If new location: Creates new entry with observation data
5. **Marks as persistent**: Ensures map Collection persists across plans

## Map Entry Structure

Each location entry in the map Collection:
```json
{
  "x": -112,
  "y": 71,
  "z": -123,
  "observed": {
    "pose": {...},
    "dirs": {...},
    "support": {...},
    "clear": {...},
    "blocks": {...},
    "geom": {...},
    "aff": {...},
    "conf": "high",
    "note": "..."
  },
  "first_visit": "2025-12-28T09:37:50",
  "last_visit": "2025-12-28T09:37:50",
  "visit_count": 1,
  "waypoints": []
}
```

## Common Workflow

```json
{"type":"mc-observe-blocks","out":"$obs"}
{"type":"mc-status","out":"$status"}
{"type":"mc-map-update","observation":"$obs","out":"$result"}
{"type":"persist","target":"minecraft_map"}
```

## Cognitive Contract

- **Converts sight to memory**: Ephemeral observations become persistent spatial knowledge
- **Automatic coordinate extraction**: Parses coordinates from structured observation format
- **Visit tracking**: Increments visit_count for repeated locations
- **Persistent storage**: Map Collection persists across `generate_plan` invocations
- **Block-level precision**: Rounds coordinates to integer block positions for consistency

## Important Notes

- **First use creates map**: If `minecraft_map` doesn't exist, it's created automatically
- **Coordinates rounded**: Float coordinates are rounded to nearest block (integer)
- **Observation data preserved**: Stores structured observation summary from `mc-observe-blocks`
- **Waypoints preserved**: Existing waypoint labels are preserved when updating entries
- **Must persist**: After updating, use `persist` primitive to ensure map survives plan completion

