---
name: nav-approach-wall
type: python
description: "Bring the agent into stable adjacency with a solid vertical surface using nav conventions and nav motion tools."
---

# Navigation Approach Wall Tool

Advance forward until a solid vertical surface is directly ahead (stable adjacency), using nav motion tools and nav conventions.

## Purpose

Higher-level navigation skill to support wall-dependent goals (climb, carve, align, boundary exploration). Uses verified nav motion and can stabilize footing when needed.

## Input

- `target`: Integer maximum blocks to advance before giving up (default: `10`)
- `step_duration`: Float seconds per move (default: `0.2`)
- `placement_item`: Item/block name for stabilization (default: `"dirt"`)
- `value`: Ignored

## Output

Returns uniform_return format with:
- `value`: Text summary with outcome and notes
- `data`: Structured dict. Key fields:
  - `success`: Boolean
  - `outcome`: `"SUCCESS"` | `"NO_WALL_REACHABLE"` | `"SUPPORT_UNSTABILIZABLE"` | `"FALL_DETECTED"`
  - `target`: Integer
  - `step_duration`: Float
  - `placement_item`: String
  - `log`: List of strings

## Behavior & Performance

- Observes forward block each loop.
- Uses `nav-advance` for motion (verified, snap-to-grid, nav history).
- If support becomes ambiguous, attempts `mc-place-until-supported` then continues.

## Guidelines

- Use before `nav-climb` or other wall-dependent operations.
- If `FALL_DETECTED`, treat as safety event and consider `nav-backtrack`.

## Usage Examples

Approach a wall within 10 blocks:
```json
{"type":"nav-approach-wall","target":10,"out":"$aw"}
```

