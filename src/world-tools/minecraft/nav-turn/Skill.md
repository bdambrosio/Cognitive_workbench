---
name: nav-turn
type: python
description: "Turn to a cardinal direction (0, 90, 180, 270) and snap to block center. Atomic orientation operation suitable for composition."
schema_hint:
  value: "ignored"
  direction: "one of {right, left, back, forward} (required)"
  out: "$variable"
examples:
  - '{"type":"nav-turn","direction":"right","out":"$turn"}'
---

# nav-turn

The only navigation tool that changes orientation. All movement tools (`nav-move`, `nav-climb`, `nav-descend`) move forward relative to current facing.

## Turn Behavior

| Direction | Action |
|-----------|--------|
| `right` | +90° from current yaw → round to cardinal |
| `left` | -90° from current yaw → round to cardinal |
| `back` | +180° from current yaw → round to cardinal |
| `forward` | Round current yaw to cardinal (align only) |

Output yaw is always one of: 0°, 90°, 180°, 270°.

## Output

Success (`status: "success"`):
- `value`: Description string
- `extra.yaw`: Cardinal yaw (0, 90, 180, or 270)
- `extra.direction`: Direction used

Failure (`status: "failed"`):
- `reason`: One of `"invalid_direction"`, `"status_failed"`, `"snapto_failed"`

## Invariants

- Always results in cardinal yaw (0, 90, 180, 270)
- Always snaps to block center: `(floor(x)+0.5, y, floor(z)+0.5)`
- Always sets pitch to 0°
- Never changes block position (orientation + centering only)

## Planning Notes

- Use `nav-turn` before movement to change direction
- Use `forward` to align to nearest cardinal without rotation
- Compose with `nav-move`, `nav-climb`, `nav-descend` for navigation sequences

## Example Workflow

```json
{"type":"nav-turn","direction":"right","out":"$turn"}
{"type":"nav-move","out":"$m1"}
{"type":"nav-move","out":"$m2"}
{"type":"nav-turn","direction":"left","out":"$turn2"}
```
