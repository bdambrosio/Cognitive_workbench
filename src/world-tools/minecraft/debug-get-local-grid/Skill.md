---
name: debug-get-local-grid
type: python
hidden: true
description: "Debug tool: Get raw local_grid from world_state"
---

# debug-get-local-grid

Debug tool for testing coordinate system. Returns raw local_grid data from world_state.

## Input

None

## Output

Returns uniform return format with:
- `value`: Summary text
- `data`: Raw local_grid dict with:
  - `center`: `{x, y, z, yaw}` (absolute coordinates)
  - `radius`: int
  - `cells`: Dict of cell data (keyed by "x,y,z")

## Notes

- Hidden from planner catalog (debug/testing only)
- Returns raw internal grid structure
- Accessible via direct `{"type": "debug-get-local-grid"}` invocation
