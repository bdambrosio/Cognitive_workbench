---
name: mc-map-query
type: python
description: "Queries persistent spatial map for strategic decision-making. Supports coverage, reachability, safety, and resource queries."
---

# Minecraft Map Query Tool

Queries cell-based SpatialMap for spatial planning. Supports multiple query types for coverage analysis, movement planning, safety assessment, and resource discovery.

## Purpose

Query spatial memory for strategic information. Returns cells matching criteria for movement planning, hazard avoidance, exploration targeting, and resource gathering.

## Input

Common parameters:
- `query`: Query type (see list below)
- `x`, `z`: Center coordinates for radius-based queries
- `y`: Y coordinate (optional, used for query-time computations)
- `radius`: Search radius (default: 10)

Query-specific parameters:
- `threshold`: Confidence threshold for `cells-low-confidence` (default: 0.5)
- `max_age`: Maximum age in seconds for `cells-stale` (default: 300)
- `max_delta_y`: Max Y change for `cells-reachable` (default: 1)
- `min_dist`: Minimum distance for `cells-observed-from-distance`
- `resource_type`: Resource type for `cells-with-resource`
- `predicate`: Filter for `cells-nearest` (walkable, safe, hazard, resource)

**Important**: Some queries now support query-time computation when `y` (and optionally `x`, `z`) are provided:
- `cells-blocked`: Computes blocking relative to position (requires `x`, `z`, `y`)
- `cells-reachable`: Computes delta_y and blocking relative to current position (uses `y` if provided)
- `cells-requiring-climb`: Computes step_up relative to Y coordinate (requires `y`)
- `cells-with-drop-risk`: Computes drop risk relative to Y coordinate (uses `y` if provided)

## Query Types

Coverage / Observability:
- `stats`: Map statistics (no coords required)
- `cells-unobserved`: Unmapped cells within radius
- `cells-low-confidence`: Cells with confidence below threshold
- `frontier-cells`: Observed cells adjacent to unobserved
- `cells-observed-from-distance`: Cells observed from far away
- `cells-stale`: Cells with old observations

Reachability / Locomotion:
- `cells-reachable`: Walkable cells within delta_y constraint (uses `y` for query-time delta_y and blocking)
- `cells-blocked`: Blocked cells relative to position (requires `x`, `z`, `y`) or unwalkable cells if no position
- `cells-requiring-climb`: Cells needing upward movement (requires `y` for query-time step_up computation)
- `cells-with-drop-risk`: Cells involving unsafe descent (uses `y` for query-time drop risk computation)

**Note**: "blocked", "step_up", and "step_down" are now query-time attributes computed relative to current position, not static cell properties. Provide `x`, `z`, `y` coordinates for accurate query-time computation.

Safety / Survival:
- `cells-safe-to-stand`: Low hazard, walkable cells
- `cells-high-hazard`: Cells with hazards (lava, fire, etc.)
- `cells-escape-nodes`: Good retreat positions

Resources:
- `cells-with-resource`: Cells with specified resource type
- `cells-water-source`: Cells with water

Multi-Objective:
- `cells-candidate-waypoints`: Safe, reachable waypoint candidates
- `cells-nearest`: Nearest cells matching predicate
- `cells-worth-revisit`: Low confidence or inferred cells

Note: Legacy Collection-based queries (`location`, `property`, `waypoint`, `unexplored`, `nearest`) are deprecated. Use SpatialMap queries instead.

## Output

Returns uniform_return format with:
- `value`: Human-readable summary
- `data`: `{success, query_type, result_count, results: [cell...]}`

## Usage Examples

Map statistics:
```json
{"type":"mc-map-query","query":"stats"}
```

Unobserved cells:
```json
{"type":"mc-map-query","query":"cells-unobserved","x":-112,"z":-123,"radius":15}
```

Safe cells nearby:
```json
{"type":"mc-map-query","query":"cells-safe-to-stand","x":-112,"z":-123,"radius":10}
```

Find water:
```json
{"type":"mc-map-query","query":"cells-water-source"}
```

Nearest walkable:
```json
{"type":"mc-map-query","query":"cells-nearest","x":-112,"z":-123,"predicate":"walkable"}
```
