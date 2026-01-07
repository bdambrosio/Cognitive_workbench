---
name: mc-map-update
type: python
description: "Updates persistent spatial map with observation data. Populates cell-based SpatialMap and Collection-based observation log."
---

# Minecraft Map Update Tool

Converts ephemeral observation data into persistent spatial memory. Updates both the cell-based SpatialMap (for queries) and Collection-based log (for raw observations).

## Purpose

Store observation data from mc-observe-blocks into persistent spatial memory. Cell data supports spatial queries for planning. Collection log preserves raw observations for reference.

## Input

- `observation`: Observation data from mc-observe-blocks (optional - if not provided, automatically invokes mc-observe-blocks)
- `radius` or `blocks_radius`: Optional radius for mc-observe-blocks (default: 4, only used when observation is not provided)
- `x`, `y`, `z`: Optional coordinates (extracted from observation if not provided)
- `map_name`: Optional map Collection name (default: agent-specific)

## Output

Returns uniform_return format with:
- `value`: Summary text of update
- `data`: Structured result with:
  - `success`: Boolean
  - `map_name`: Collection name
  - `map_id`: Collection ID
  - `note_id`: Created Note ID
  - `location`: `{x, y, z}` block coordinates
  - `total_observations`: Count in Collection
  - `spatial_map`: `{cells_updated, total_cells, bounds}`

## Behavior & Performance

- If no observation provided, automatically invokes mc-observe-blocks internally to obtain current observation
- Populates cell schema from observation: support, surface, hazards, resources, observability
- Updates observer cell (direct) and forward cell (inferred)
- Auto-saves SpatialMap after each update
- Creates/updates Collection for raw observation log
- Coordinates rounded to block integers

## Guidelines

- Can be called directly without observation - will automatically invoke mc-observe-blocks
- Or call after mc-observe-blocks to persist spatial knowledge from a specific observation
- Cell data enables spatial queries via mc-map-query
- Visualization via mc-map-visualize shows mapped cells
- Clear map via FastAPI Controls panel removes both SpatialMap and Collection

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
