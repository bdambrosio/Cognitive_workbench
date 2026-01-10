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
- `radius` or `blocks_radius`: Optional radius for mc-observe-blocks (default: 7, only used when observation is not provided)
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
- Updates observer cell (direct), forward cell (inferred), and neighbor cells (obstructions)
- Auto-saves SpatialMap after each update
- Coordinates rounded to block integers

**Data Storage Philosophy**:
- Stores only **absolute** properties: `support_y` (absolute Y coordinate), `walkable`, `drop` (has pit below)
- Does **NOT** store agent-relative data: `blocked`, `step_up`, `step_down`, `delta_y_from_agent` are computed at query time
- Stores raw obstruction data (`obstructions.blocks_at_y`) for query-time blocking computation
- Resources/hazards only recorded when on surface (Y = `support_y` or `support_y + 1`)

**Cone-Based Visibility Impact**:
- `mc-observe-blocks` uses a forward view cone (yaw ±60°, pitch -60..+90) with line-of-sight, so surface blocks behind taller obstacles may be occluded
- Observer cell and forward cell always have `support_y` (from downward probes, not affected by occlusion)
- Distant cells may be created from `nearby_blocks` but lack `support_y` if their surface blocks are occluded
- Cells without `support_y` cannot have resources/hazards recorded (requires surface Y coordinate for validation)
- This is expected behavior: incomplete data for occluded cells until observed from another angle

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
{"type":"mc-map-update","radius":5}
```

Update from existing observation:
```json
{"type":"mc-map-update","observation":"$obs"}
```

With explicit coordinates:
```json
{"type":"mc-map-update","observation":"$obs","x":-112,"y":71,"z":-123}
```
