---
name: nav-descend
type: python
description: "Attempt exactly ONE controlled descent forward into an adjacent lower cell. Covers stepping down slopes, edges, and shallow drops (≤ 1 block)."
schema_hint:
  value: "ignored"
  step_duration: "float seconds for the move (default: 0.25)"
  max_drop: "float maximum allowed drop in Y (default: 1.2)"
  min_drop: "float minimum Y decrease to count as descend (default: 0.2)"
  out: "$variable"
examples:
  - '{"type":"nav-descend","out":"$descend"}'
---

# nav-descend

Attempt exactly ONE controlled descent forward into an adjacent lower cell. Covers stepping down slopes, edges, and shallow drops (≤ 1 block).

## Input

- `step_duration`: Float seconds for the move (default: `0.25`)
- `max_drop`: Float maximum allowed drop in Y (default: `1.2`)
- `min_drop`: Float minimum Y decrease to count as descend (default: `0.2`)

## Output

Success (`status: "success"`):
- `value`: Summary text
- `extra.from`: Starting position `{x, y, z}`
- `extra.to`: Ending position `{x, y, z}`
- `extra.delta_y`: Vertical displacement (negative, within bounds)
- `extra.support_here`: Support type at destination

Failure (`status: "failed"`):
- `reason`: One of `"status_failed"`, `"move_failed"`, `"collision"`, `"observation_failed"`, `"support_ambiguous"`, `"no_descent"`, `"excessive_drop"`

## Invariants

- Exactly one descent attempt per call (always forward)
- Descent must be within bounds: `min_drop` ≤ `|delta_y|` ≤ `max_drop`
- Landing support must be walkable ("solid" or "unsafe")
- Snaps to block center after any position change
- Updates `world_state("nav")` history

## Planning Notes

- Does not handle multi-block drops; compose multiple calls for that
- Use `nav-turn` before descending to change direction
- Falls within bounds are treated as controlled descent (success if landing is safe)

## Example Workflow

```json
{"type":"nav-turn","direction":"left","out":"$t"}
{"type":"nav-descend","out":"$d1"}
{"type":"nav-descend","out":"$d2"}
```
