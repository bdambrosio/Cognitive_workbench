---
name: mc-map-update
type: python
description: "Converts ephemeral observation data into persistent spatial memory. Stores observation data in a persistent Collection named 'minecraft_map'"
---

# Minecraft Map Update Tool

Converts ephemeral observation data into persistent spatial memory. Stores observation data in a persistent Collection for later querying.

## Purpose

Spatial memory persistence for learning and navigation. Converts observation data from `mc-observe-blocks` into persistent map entries that can be queried later.

## Input

- `value`: Observation data from `mc-observe-blocks` (preferred)
- `observation`: Observation data from `mc-observe-blocks` (alternative to value)
- `x`, `y`, `z`: Optional explicit coordinates (defaults to extracting from observation)
- `map_name`: Optional map Collection name (default: `<agent_name>-minecraft_map`)

## Output

Returns uniform_return format with:
- `value`: Text summary (success message with location and total locations count)
- `data`: Structured data dict (machine-readable). Key fields:
  - `success`: Boolean
  - `map_name`: String
  - `location`: `{x: int, y: int, z: int}`
  - `total_locations`: Integer
  - `visit_count`: Integer

## Behavior & Performance

- Extracts location from observation data
- Stores observation in persistent Collection
- Tracks visit counts for locations
- Map name defaults to agent-specific name

## Guidelines

- Use after `mc-observe-blocks` to store observations
- Observation data should come from `mc-observe-blocks` tool
- Coordinates extracted from observation if not explicitly provided
- Stored data enables later queries with `mc-map-query`
- Visit counts track how many times location was observed

## Usage Examples

Update map with observation:
```json
{"type":"mc-observe-blocks","out":"$obs"}
{"type":"mc-map-update","observation":"$obs","out":"$result"}
```

Update map with explicit coordinates:
```json
{"type":"mc-map-update","observation":"$obs","x":-112,"y":71,"z":-123,"out":"$result"}
```
