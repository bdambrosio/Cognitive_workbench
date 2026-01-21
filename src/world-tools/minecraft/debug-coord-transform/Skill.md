---
name: debug-coord-transform
type: python
hidden: true
description: "Debug tool: Transform coordinates between agent-relative and world-relative"
---

# debug-coord-transform

Debug tool for testing coordinate transformations. Converts between agent-relative and world-relative coordinates.

## Input

- `dx`: float - X coordinate
- `dy`: float - Y coordinate (unchanged, always vertical)
- `dz`: float - Z coordinate
- `yaw`: float - Agent yaw in degrees (default: from current status)
- `direction`: string - "agent-to-world" or "world-to-agent" (default: "agent-to-world")

## Output

Returns uniform return format with:
- `value`: Summary text
- `data`: Dict with:
  - `input`: `{dx, dy, dz, yaw, direction}`
  - `output`: `{dx, dy, dz}` (transformed coordinates)

## Notes

- Hidden from planner catalog (debug/testing only)
- Useful for validating coordinate transformations
- Accessible via direct `{"type": "debug-coord-transform"}` invocation
