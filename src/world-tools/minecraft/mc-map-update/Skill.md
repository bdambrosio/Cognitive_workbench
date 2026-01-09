---
name: mc-map-update
type: python
description: "Updates persistent spatial map with observation data. Populates cell-based SpatialMap."
---

# Minecraft Map Update Tool

Converts ephemeral observation data into persistent spatial memory. Updates the cell-based SpatialMap with observation data from mc-observe-blocks.

## Purpose

Store observation data from mc-observe-blocks into persistent spatial memory. Cell data supports spatial queries for planning. Automatically checks if current location is already mapped before observing.

## Input

- `observation`: Observation data from mc-observe-blocks (optional - if not provided, automatically invokes mc-observe-blocks)
- `radius` or `blocks_radius`: Optional radius for mc-observe-blocks (default: 4, only used when observation is not provided)
- `x`, `y`, `z`: Optional coordinates (extracted from observation if not provided)

## Output

Returns uniform_return format with:
- `value`: Summary text of update
- `data`: Structured result with:
  - `location`: `{x, y, z}` block coordinates
  - `spatial_map`: `{cells_updated, total_cells, bounds}`

## Behavior & Performance

- If no observation provided, checks if current location is already mapped
- If location is already mapped (has `last_observed_at`), skips auto-observation unless observation is explicitly provided
- If location not mapped or observation provided, automatically invokes mc-observe-blocks internally
- Populates cell schema from observation: support, surface, hazards, resources, observability, waypoints
- Updates observer cell (direct) and forward cell (inferred)
- Auto-saves SpatialMap after each update
- Coordinates rounded to block integers

## Guidelines

- Can be called directly without observation - will automatically invoke mc-observe-blocks if needed
- Or call after mc-observe-blocks to persist spatial knowledge from a specific observation
- Cell data enables spatial queries via mc-map-query
- Visualization via mc-map-visualize shows mapped cells
- Clear map via FastAPI Controls panel removes SpatialMap

## Usage Examples

Update automatically (invokes mc-observe-blocks internally):
```json
{"type":"mc-map-update"}
```

Update with custom radius:
```json
{"type":"mc-map-update","radius":6}
```

Update from existing observation:
```json
{"type":"mc-map-update","observation":"$obs"}
```

With explicit coordinates:
```json
{"type":"mc-map-update","observation":"$obs","x":-112,"y":71,"z":-123}
```
