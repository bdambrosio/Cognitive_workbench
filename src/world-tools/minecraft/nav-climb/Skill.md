---
name: nav-climb
type: python
description: "Attempt exactly ONE adjacent climb (+1Y) forward. Atomic navigation operation suitable for composition."
schema_hint:
  value: "ignored"
  step_duration: "float seconds for the move attempt (default: 0.6)"
  allow_walkable_landing: "boolean, if True accepts walkable landings (default: True)"
  min_delta_y: "float minimum Y gain to count as climb (default: 0.9)"
  out: "$variable"
examples:
  - '{"type":"nav-climb","step_duration":0.6,"out":"$climb"}'
  - '{"type":"nav-climb","allow_walkable_landing":false,"out":"$climb"}'
---

# Navigation Climb Tool

## Skill: NAV_CLIMB

### Level
Level 1 — Atomic navigation primitive

### Purpose
Attempt exactly ONE adjacent climb forward: move into a neighbor cell that is approximately +1 block higher. This is an atomic navigation operation suitable for composition into higher-level navigation skills (e.g., nav-traverse).

### Assumptions
- Minecraft "step-up" onto a 1-block rise is achievable via a short forward move when collision/clearance allow it
- Tries walk-up first, then jump-up if needed
- Success is detected via `delta_y >= min_delta_y` (using `mc-status` Y values)
- Walkable landings (e.g., snow layers) are acceptable by default

### Primitives Used
- `mc-status`: Capture starting and ending pose
- `/act/move`: Bridge endpoint for locomotion with bounded duration (always forward, optionally with jump)
- `mc-observe-blocks`: Support assessment at destination
- `snapto`: Bridge endpoint for precise positioning and orientation

### Input
- `value`: ignored
- `step_duration`: float seconds for each move attempt (default: `0.6`)
- `allow_walkable_landing`: boolean, if True accepts walkable landings like snow (default: `True`)
- `min_delta_y`: float minimum Y gain to count as climb (default: `0.9`)

### Output
- Uniform return format dict (bound to `out` variable if specified):
  - `status`: 'success' or 'failed' (execution status)
  - `value`: None (no text summary)
  - `data`: structured data dict containing:
    - `success`: Boolean indicating if the climb succeeded
    - `climbed`: Boolean indicating if elevation gain occurred (`delta_y == 1`)
    - `landed`: Boolean indicating if the agent landed at a position (True even if fell)
    - `fell`: Boolean indicating if a fall occurred
    - `failure_reason`: One of `"move_failed"`, `"collision"`, `"fell"`, `"observation_failed"`, `"support_ambiguous"`, `"not_elevated"`, `"unexpected_delta_y"` (only present if `success=False`)
    - `from`: Starting position dict `{"x": float, "y": float, "z": float}`
    - `to`: Ending position dict `{"x": float, "y": float, "z": float}` (None if move failed)
    - `delta_y`: Vertical displacement `to.y - from.y` (present if landed)
    - `mode`: Climb mode used (`"walk"` or `"jump"`) (present if successful)
    - `support_here`: Support type at destination `{"solid", "unsafe", ...}` (present if observation succeeded)
  - `resource_id`: None (no resource created)

### Success Outcomes

When `success=True`:
- `success`: True
- `climbed`: True (elevation gain confirmed: `delta_y >= min_delta_y`)
- `from`: Starting position
- `to`: Ending position (block-centered)
- `delta_y`: Positive value >= `min_delta_y` (typically ~1.0)
- `mode`: `"walk"` or `"jump"` indicating which method succeeded
- `support_here`: Support type (typically "solid" or "unsafe")

### Failure Outcomes

When `success=False`:
- `success`: False
- `climbed`: False
- `failure_reason`: Specific reason for failure
- `from`: Starting position
- `to`: Ending position (may be same as `from` if collision, or None if move failed, block-centered if position changed)
- `delta_y`: Vertical displacement (may be 0, negative, or insufficient)

Failure Reasons:
- `"move_failed"`: Movement request failed before execution
- `"collision"`: Movement blocked by obstacle (bridge `/act/move` returned `status="collision"`)
- `"fell"`: Fall detected during movement (bridge `/act/move` returned `status="fell"`)
- `"observation_failed"`: Could not observe blocks at destination or obtain final status
- `"support_ambiguous"`: Reached destination but support type is not "solid" or "unsafe"
- `"not_elevated"`: Movement succeeded but elevation gain was insufficient (`delta_y < min_delta_y`)

### Post-Conditions

After any position change (success or failure), the agent is:
- **Snapped to block center**: Position is centered at `(floor(x)+0.5, actual_y, floor(z)+0.5)`
- **Oriented to movement direction**: Yaw is set to face the direction of actual movement (calculated from dx, dz)
- **Horizontal pitch**: Pitch is set to 0° (horizontal)

This ensures consistent, block-centered positioning for all navigation operations.

### Invariants

- Exactly one elevation gain attempt per call (always forward)
- Always captures both starting and ending positions
- Never assumes movement success without verification
- Tries walk-up first, then jump-up if needed
- Elevation gain must be >= `min_delta_y` to succeed
- Block-centered positioning established after any position change

### Notes

- This tool attempts to climb approximately ONE block higher; it does not handle multi-block climbs.
- Success requires both movement completion AND sufficient elevation gain (`delta_y >= min_delta_y`) AND acceptable landing support.
- Falls are detected immediately by the bridge `/act/move` endpoint and reported as failures.
- Collisions prevent movement and are reported as failures.
- The tool tries walk-up first (no jump), then jump-up if walk-up doesn't achieve sufficient elevation.
- The default `step_duration` (0.6s) is longer than `nav-moveone` (0.2s) to allow time for step-up mechanics.
- Higher-level skills can compose multiple `nav-climb` calls for multi-block vertical traversal.
- Orientation changes require `nav-turn` (separate tool, not yet implemented).
- The tool always moves forward relative to the agent's current facing direction.

### Common Workflow

```json
{"type":"mc-status","out":"$status_before"}
{"type":"mc-observe-blocks","out":"$obs"}
{"type":"nav-climb","step_duration":0.6,"out":"$climb1"}
{"type":"nav-climb","step_duration":0.6,"out":"$climb2"}
{"type":"mc-status","out":"$status_after"}
```
- planners should inspect success and failure_reason between steps; repeated climb without verification is unsafe.

### Cognitive Contract

- ATOMIC: Performs exactly one block elevation gain, no more, no less
- VERIFICATION-BASED: Never assumes success without position verification
- RETRY-AWARE: Handles stochastic movement by retrying when elevation gain doesn't occur
- OUTCOME-BASED: Returns detailed structured data for higher-level decision making
- COMPOSABLE: Designed to be composed into multi-step vertical navigation sequences
- POSITION-AWARE: Always tracks position transitions and elevation changes for spatial reasoning

