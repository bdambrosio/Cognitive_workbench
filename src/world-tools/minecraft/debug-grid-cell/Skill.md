---
name: debug-grid-cell
type: python
hidden: true
description: "Debug tool: Query specific cell in observed voxel grid"
---

# debug-grid-cell

Debug tool for testing coordinate system. Queries a specific cell in the observed voxel grid.

## Input

- `dx`: int - Agent-relative X coordinate
- `dy`: int - Agent-relative Y coordinate
- `dz`: int - Agent-relative Z coordinate
- `radius`: int - Optional grid radius (default: 2)

## Output

Returns uniform return format with:
- `value`: Summary text
- `data`: Cell dict with:
  - `dx, dy, dz`: Agent-relative coordinates
  - `block_id`: Block name or null
  - `solid`: bool
  - `support`: bool

## Notes

- Hidden from planner catalog (debug/testing only)
- Queries agent-relative coordinates
- Accessible via direct `{"type": "debug-grid-cell"}` invocation
