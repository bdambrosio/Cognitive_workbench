# Minecraft Integration Updates (January 2026)

This document summarizes major updates to the Minecraft world integration, focusing on navigation-first observation, spatial mapping improvements, and path planning enhancements.

## Navigation Surface Scanning (Option B)

### Overview
Replaced visual occlusion-based block scanning with **navigation-first surface scanning** for more reliable terrain mapping.

### Implementation
- **`bridge.py`**: New `scan_nav_surface()` function that:
  - Enumerates (x,z) cells within forward cone (yaw ±60°, pitch -60° to +90°)
  - Probes downward (max 6 blocks) to find supporting blocks
  - Skips non-supporting cover blocks (snow layers, grass, carpet)
  - Caches all `getblock()` calls per request
  - Caps work: max 120 nav cells, max 200 blocks total

- **Response format**: `/observe` endpoint now includes `perception.nav_surface`:
  ```json
  {
    "nav_surface": [
      {
        "x": -122, "z": -165,
        "support_y": 71,
        "support_block": "minecraft:grass_block[snowy=true]",
        "cover_block": "minecraft:snow[layers=1]",
        "walkable": true
      },
      ...
    ]
  }
  ```

### Benefits
- **Reliable terrain mapping**: Snow layers and grass don't occlude ground detection
- **Performance**: 2D cell enumeration + short vertical probes is faster than 3D LOS raycasting
- **Navigation-focused**: Directly provides `support_y` and walkability for path planning

## Spatial Map Architecture Changes

### Query-Time Attributes
Previously stored agent-relative data statically (became stale). Now computed dynamically:

- **"blocked"**: Computed via `SpatialMap.is_blocked_from(current_pos, target_pos)` using stored obstruction data
- **"step_up" / "step_down"**: Computed by comparing `surface_y` values at query time
- **Removed**: `delta_y_from_agent` (was agent-relative, now computed as needed)

### Support Detection
- Uses `adjacent_blocks['down']` from bridge (guaranteed block directly below, independent of cone filtering)
- Falls back to `dirs.down` when `support.here.depth` is missing
- More reliable ground detection even when `nearby_blocks` is incomplete

### Data Storage Philosophy
- **Stored**: Absolute properties (`support_y`, `walkable`, `drop`, `obstructions.blocks_at_y`)
- **Computed at query time**: Agent-relative properties (`blocked`, `step_up`, `step_down`, Y-deltas)
- **Resources/hazards**: Only recorded when on surface (Y = `support_y` or `support_y + 1`)

## Observation Tools Updates

### Cone-Based Visibility
All observation tools (`mc-observe-blocks`, `mc-observe-entities`, `mc-observe-items`) now use:

- **Horizontal cone**: yaw ±60° (120° total)
- **Vertical range**: pitch -60° to +90° (up 60°, down 90°)
- **Line-of-sight**: Occlusion-aware (rays terminate on opaque blocks)
- **Default radius**: 7 blocks (max 7)

### Entity/Item Format
Entities and items now include:
- `name`, `type`, `position` (x, y, z)
- `dx`, `dy`, `dz` (offsets from agent)
- `distance` (Euclidean distance from agent foot position)
- Items: `item_name`, `item_count`

### Block Format
Blocks include:
- `name`, `position`, `dx`, `dy`, `dz`
- `surface` (tri-state: `true` | `false` | `"unknown"`)

## Path Planning Improvements

### path-frontier Updates
- **Default `allow_unknown=True`**: Over-approximates frontier candidates
- **Unknown cells**: Returned as `{"clear_body": true, "clear_head": true, "support": "unknown"}` instead of `None`
- **Start Y alignment**: Snaps `start_state["y"]` to map's `support_y + 1` when available
- **Climb/descend support**: Correctly handles y±1 movements in simulation

### nav_simulation Updates
- **Climb semantics**: Validates destination at y+1, support at y
- **Descend semantics**: Validates destination at y-1, support at y-2
- **Unknown support**: Accepted when `allow_unknown=True` (treated as "unsafe" for support checks)

### Behavior
- **Over-approximate**: Returns frontier positions even with partial map coverage
- **Does not guarantee reachability**: Simulation estimates may fail in execution
- **Considers y±1**: Properly handles step-up/step-down terrain

## Tool Changes Summary

### mc-observe-blocks
- Uses `nav_surface` for forward support detection
- Default/max radius: 7 blocks
- Returns `nav_surface` in structured data

### mc-map-update
- Ingests `nav_surface` to populate nearby cells' `support_y`, `walkable`, `surface_block`
- Preserves existing data when new observations are incomplete
- Only updates fields when explicit data is available

### mc-map-query
- Updated to accept `y` parameter for query-time computations
- "blocked", "step_up", "step_down" are query-time attributes

### mc-map-visualize
- Removed `delta_y_from_agent` from display
- Fixed hover functionality for accurate cell detail display

## Migration Notes

### Breaking Changes
- **Radius limits**: Default changed from 5 to 7, max changed from 12 to 7
- **Observation format**: `nearby_blocks` now synthesized from `nav_surface` (may differ from previous visual-only results)
- **Map schema**: Removed `delta_y_from_agent` field

### Compatibility
- Legacy map files load correctly (missing fields default to `None`/`unknown`)
- `path-frontier` with `allow_unknown=False` still works but may return empty results with partial maps
- Observation tools backward-compatible (still return `nearby_blocks` for compatibility)

## Performance

### Observation Timing
- **nav_surface scan**: ~200-500ms for radius=7 (depends on terrain density)
- **Total `/observe`**: ~500-2000ms (includes visibility_distances, nav_surface, entities, status)
- **Caching**: Block reads cached per request to minimize `getblock()` calls

### Map Updates
- **SpatialMap.save()**: Auto-saves after each `mc-map-update`
- **Cell updates**: Typically 5-20 cells per observation (observer + neighbors + nav_surface cells)

## Future Considerations

- **Dynamic radius**: Could adjust based on terrain complexity
- **Incremental updates**: Only update changed cells to reduce save overhead
- **Multi-level support**: Handle multi-block step-up/step-down (currently limited to ±1)
