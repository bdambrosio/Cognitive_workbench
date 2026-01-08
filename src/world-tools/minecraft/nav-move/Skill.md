---
name: nav-move
type: python
description: "Attempt exactly ONE adjacent navigation move forward and report the outcome. Atomic navigation operation suitable for composition."
---

# nav-move

Attempt exactly one adjacent forward move (relative to current facing direction) and report the outcome. Atomic forward navigation primitive.

## Input

- `step_duration`: Float seconds for the move (default: `0.2`)
- `max_abs_delta_y`: Float tolerance for Y noise (default: `0.2`)

## Output

Success (`status: "success"`):
- `value`: Summary text
- `extra.from`: Starting position `{x, y, z}`
- `extra.to`: Ending position `{x, y, z}`
- `extra.delta_y`: Vertical displacement
- `extra.support_here`: Support type at destination

Failure (`status: "failed"`):
- `reason`: One of `"status_failed"`, `"move_failed"`, `"collision"`, `"fell"`, `"observation_failed"`, `"support_ambiguous"`, `"unexpected_vertical_change"`
- `extra.from`, `extra.to`: Positions (if available)

## Behavior

- Calls bridge `/act/move` once (forward) with collision checking
- Validates landing via `mc-status` + `mc-observe-blocks`
- Snaps to block center after any position change
- Updates `world_state("nav")` history

## Planning Notes

- Use for single-step movement with verification
- For multi-step forward motion, prefer `nav-advance`
- If `fell` occurs, treat as safety event and consider `nav-backtrack`

## Examples

```json
{"type":"nav-move","out":"$m1"}
{"type":"nav-move","step_duration":0.25,"out":"$m2"}
```
