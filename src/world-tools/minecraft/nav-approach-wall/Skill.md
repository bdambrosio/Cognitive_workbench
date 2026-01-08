---
name: nav-approach-wall
type: python
description: "Bring the agent into stable adjacency with a solid vertical surface using nav conventions and nav motion tools."
---

# nav-approach-wall

Advance forward until a solid vertical surface is directly ahead (stable adjacency), using nav motion tools and nav conventions.

## Input

- `target`: Integer maximum blocks to advance before giving up (default: `10`)
- `step_duration`: Float seconds per move (default: `0.2`)
- `placement_item`: Item/block name for stabilization (default: `"dirt"`)

## Output

Success (`status: "success"`):
- `value`: Summary text with log
- `extra.outcome`: `"SUCCESS"`
- `extra.log`: List of step descriptions

Failure (`status: "failed"`):
- `value`: Summary text with log
- `extra.outcome`: `"NO_WALL_REACHABLE"` | `"SUPPORT_UNSTABILIZABLE"` | `"FALL_DETECTED"`
- `extra.log`: List of step descriptions

## Behavior

- Observes forward block each loop
- Uses `nav-advance` for motion (verified, snap-to-grid, nav history)
- If support becomes ambiguous, attempts `mc-place-until-supported` then continues

## Planning Notes

- Use before `nav-climb` or other wall-dependent operations
- If `FALL_DETECTED`, treat as safety event and consider `nav-backtrack`

## Example

```json
{"type":"nav-approach-wall","target":10,"out":"$aw"}
```
