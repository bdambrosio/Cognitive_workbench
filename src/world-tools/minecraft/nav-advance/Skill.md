---
name: nav-advance
type: python
description: "Advance forward up to N blocks using nav conventions. Stops early on collision, fall, or ambiguous support."
---

# nav-advance

Planner-friendly multi-step forward motion. Reduces fragile planner loops while preserving nav invariants (verification, snap-to-grid, nav history).

## Input

- `blocks`: Integer number of blocks to advance (required)
- `step_duration`: Float seconds per move (default: `0.2`)
- `max_abs_delta_y`: Float tolerance for Y noise (default: `0.2`)

## Output

Success (`status: "success"`):
- `value`: Summary text (e.g., "Advance completed: TARGET_REACHED (3/3)")

Failure (`status: "failed"`):
- `value`: Summary with steps completed
- `reason`: One of `"INVALID_TARGET"`, `"STATUS_FAILED"`, `"MOVE_FAILED"`, `"COLLISION"`, `"FELL"`, `"OBSERVATION_FAILED"`, `"UNEXPECTED_VERTICAL_CHANGE"`, `"SUPPORT_AMBIGUOUS"`

## Behavior

- Internally performs up to `blocks` single moves with verification
- Snaps to block center after each landed move and updates nav history
- Stops immediately when unsafe conditions are detected

## Planning Notes

- Use instead of planner loops over `nav-move`
- If `reason` is `FELL`, treat as safety event and consider `nav-backtrack`

## Example

```json
{"type":"nav-advance","blocks":3,"out":"$adv"}
```
