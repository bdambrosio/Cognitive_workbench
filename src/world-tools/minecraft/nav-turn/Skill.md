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
  - '{"type":"nav-turn","direction":"left","out":"$turn"}'
  - '{"type":"nav-turn","direction":"back","out":"$turn"}'
  - '{"type":"nav-turn","direction":"forward","out":"$turn"}'
---

# Navigation Turn Tool

## Skill: NAV_TURN

### Level
Level 1 — Atomic orientation primitive

### Purpose
Turn to a cardinal direction (0°, 90°, 180°, or 270°) and snap to block center. This is an atomic orientation operation suitable for composition into higher-level navigation skills. This is the only navigation tool that changes orientation; all movement tools (`nav-moveone`, `nav-climb`, `nav-descend`) move forward relative to current facing direction.

### Primitives Used
- `mc-status`: Capture current pose and orientation
- `snapto`: Bridge endpoint for precise positioning and orientation

### Input
- `value`: ignored
- `direction`: one of `{"right", "left", "back", "forward"}` (required)

### Output
- Uniform return format dict (bound to `out` variable if specified):
  - `status`: 'success' or 'failed' (execution status)
  - `value`: None (no text summary)
  - `data`: structured data dict containing:
    - `success`: Boolean indicating if the turn succeeded
    - `yaw`: New yaw value (0, 90, 180, or 270) (always present)
    - `direction`: Direction used for turn (present if `success=True`)
    - `failure_reason`: One of `"invalid_direction"`, `"status_failed"`, `"snapto_failed"` (only present if `success=False`)
  - `resource_id`: None (no resource created)

### Turn Behavior

- right: Rotate +90° from current yaw, then round to nearest cardinal
- left: Rotate -90° from current yaw, then round to nearest cardinal
- back: Rotate +180° from current yaw, then round to nearest cardinal
- forward: Round current yaw to nearest cardinal (no rotation, just alignment)

All results are normalized to one of: 0°, 90°, 180°, or 270° (360° maps to 0°).

### Post-Conditions

After a successful turn, the agent is:
- Snapped to block center: Position is centered at `(floor(x)+0.5, actual_y, floor(z)+0.5)`
- Oriented to cardinal direction: Yaw is set to 0°, 90°, 180°, or 270°
- Horizontal pitch: Pitch is set to 0° (horizontal)

This ensures consistent, block-centered positioning and cardinal orientation for all navigation operations.

### Success Outcomes

When `success=True`:
- `success`: True
- `yaw`: Cardinal yaw value (0, 90, 180, or 270)
- `direction`: Direction used for turn

### Failure Outcomes

When `success=False`:
- `success`: False
- `failure_reason`: Specific reason for failure
- `yaw`: Cardinal yaw value (still calculated and returned even on failure)

Failure Reasons:
- `"invalid_direction"`: Direction parameter was not one of the allowed values
- `"status_failed"`: Could not obtain current status/position
- `"snapto_failed"`: Failed to snap to position and orientation via bridge endpoint

### Invariants

- Always results in a cardinal yaw (0, 90, 180, or 270)
- Always snaps to block center
- Always sets pitch to 0°
- Never changes position (only orientation and centering)

### Notes

- This is the only navigation tool that changes orientation.
- All movement tools (`nav-moveone`, `nav-climb`, `nav-descend`) move forward relative to current facing direction.
- Use `nav-turn` before movement to change direction, or after movement to align to cardinal directions.
- The `forward` direction is useful for aligning to a cardinal direction without changing orientation.
- All turns result in cardinal directions, ensuring predictable orientation for navigation planning.

### Common Workflow

```json
{"type":"mc-status","out":"$status_before"}
{"type":"nav-turn","direction":"right","out":"$turn"}
{"type":"nav-moveone","out":"$m1"}
{"type":"nav-moveone","out":"$m2"}
{"type":"nav-turn","direction":"left","out":"$turn2"}
{"type":"nav-moveone","out":"$m3"}
{"type":"mc-status","out":"$status_after"}
```

### Cognitive Contract

- ATOMIC: Performs exactly one orientation change, no more, no less
- CARDINAL: Always results in a cardinal direction (0, 90, 180, 270)
- BLOCK-CENTERED: Always snaps to block center for consistent positioning
- COMPOSABLE: Designed to be composed with movement tools for navigation sequences
- ORIENTATION-ONLY: Changes orientation only; does not change position (except centering)

