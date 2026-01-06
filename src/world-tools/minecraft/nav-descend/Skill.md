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
  - '{"type":"nav-descend","step_duration":0.25,"max_drop":1.2,"out":"$descend"}'
  - '{"type":"nav-descend","min_drop":0.2,"out":"$descend"}'
---

# Navigation Descend Tool

## Skill: NAV_DESCEND

### Level
Level 1 — Atomic navigation primitive

### Purpose
Attempt exactly ONE controlled descent forward into an adjacent lower cell. This covers stepping down slopes, edges, and shallow drops (≤ 1 block). This is an atomic navigation operation suitable for composition into higher-level navigation skills.

### Assumptions
- Minecraft allows controlled descent via normal forward movement when stepping down slopes or edges
- Success is detected via negative `delta_y` within acceptable bounds (`min_drop` ≤ `abs(delta_y)` ≤ `max_drop`)
- Walkable blocks (snow, carpet) provide acceptable landing support even if not "solid"

### Primitives Used
- `mc-status`: Capture starting and ending pose
- `/act/move`: Bridge endpoint for locomotion with bounded duration (always forward)
- `mc-observe-blocks`: Support assessment at destination
- `snapto`: Bridge endpoint for precise positioning and orientation

### Input
- `value`: ignored
- `step_duration`: float seconds for the move (default: `0.25`)
- `max_drop`: float maximum allowed drop in Y (default: `1.2`)
- `min_drop`: float minimum Y decrease to count as descend (default: `0.2`)

### Output
- Uniform return format dict (bound to `out` variable if specified):
  - `status`: 'success' or 'failed' (execution status)
  - `value`: None (no text summary)
  - `data`: structured data dict containing:
    - `success`: Boolean indicating if the descent succeeded
    - `descended`: Boolean indicating if descent occurred (only present if `success=True`)
    - `failure_reason`: One of `"move_failed"`, `"collision"`, `"fell"`, `"observation_failed"`, `"support_ambiguous"`, `"no_descent"`, `"excessive_drop"` (only present if `success=False`)
    - `from`: Starting position dict `{"x": float, "y": float, "z": float}`
    - `to`: Ending position dict `{"x": float, "y": float, "z": float}` (None if move failed)
    - `delta_y`: Vertical displacement `to.y - from.y` (present if landed)
    - `support_here`: Support type at destination `{"solid", "unsafe", ...}` (present if observation succeeded)
  - `resource_id`: None (no resource created)

### Success Outcomes

When `success=True`:
- `success`: True
- `descended`: True
- `from`: Starting position
- `to`: Ending position
- `delta_y`: Negative value between `-max_drop` and `-min_drop` (e.g., -0.2 to -1.2)
- `support_here`: Support type ("solid" or "unsafe")

### Failure Outcomes

When `success=False`:
- `success`: False
- `failure_reason`: Specific reason for failure
- `from`: Starting position
- `to`: Ending position (may be same as `from` if collision, or None if move failed)
- `delta_y`: Vertical displacement (may be positive, zero, or excessively negative)

Failure Reasons:
- `"move_failed"`: Movement request failed before execution
- `"collision"`: Movement blocked by obstacle (bridge `/act/move` returned `status="collision"`)
- `"observation_failed"`: Could not observe blocks at destination or obtain final status
- `"support_ambiguous"`: Reached destination but support type is not "solid" or "unsafe" (e.g., air/void)
- `"no_descent"`: Movement succeeded but descent was insufficient (`delta_y >= -min_drop`, meaning not enough downward movement). This includes falls that don't meet minimum descent threshold.
- `"excessive_drop"`: Descent exceeded maximum allowed (`abs(delta_y) > max_drop`, meaning drop was too large). This includes falls that exceed the maximum safe drop distance.

### Post-Conditions

After any position change (success or failure), the agent is:
- **Snapped to block center**: Position is centered at `(floor(x)+0.5, actual_y, floor(z)+0.5)`
- **Oriented to movement direction**: Yaw is set to face the direction of actual movement (calculated from dx, dz)
- **Horizontal pitch**: Pitch is set to 0° (horizontal)

This ensures consistent, block-centered positioning for all navigation operations.

### Invariants

- Exactly one descent attempt per call (always forward, no retry logic)
- Always captures both starting and ending positions
- Never assumes movement success without verification
- Descent must be within acceptable bounds (`min_drop` ≤ `abs(delta_y)` ≤ `max_drop`)
- Landing support must be walkable ("solid" or "unsafe")
- Block-centered positioning established after any position change

### Notes

- This tool attempts a controlled descent of up to 1 block; it does not handle multi-block drops.
- Success requires both movement completion AND descent within acceptable bounds AND walkable landing support.
- Falls detected by the bridge `/act/move` endpoint are validated against descent criteria: if the fall results in a controlled descent (within `min_drop` to `max_drop` bounds) with walkable landing support, it is treated as success. Falls that are too small, too large, or land on non-walkable surfaces are reported as failures with appropriate reasons (`no_descent`, `excessive_drop`, `support_ambiguous`).
- Collisions prevent movement and are reported as failures.
- The default `step_duration` (0.25s) is shorter than `nav-climb` (0.6s) as descent typically requires less time.
- The `max_drop` parameter (default 1.2) allows for slight overshoot in Minecraft physics while preventing dangerous falls.
- The `min_drop` parameter (default 0.2) ensures that actual descent occurred, not just horizontal movement.
- Walkable blocks like snow and carpet are acceptable landing surfaces (classified as "unsafe" but still walkable).
- Higher-level skills can compose multiple `nav-descend` calls for multi-step downward traversal.
- Orientation changes require `nav-turn` (separate tool, not yet implemented).
- The tool always moves forward relative to the agent's current facing direction.

### Common Workflow

```json
{"type":"mc-status","out":"$status_before"}
{"type":"mc-observe-blocks","out":"$obs"}
{"type":"nav-descend","step_duration":0.25,"max_drop":1.2,"out":"$descend1"}
{"type":"nav-descend","step_duration":0.25,"max_drop":1.2,"out":"$descend2"}
{"type":"mc-status","out":"$status_after"}
```
- planners should inspect success and failure_reason between steps; repeated descent without verification is unsafe.

### Cognitive Contract

- ATOMIC: Performs exactly one controlled descent, no more, no less
- VERIFICATION-BASED: Never assumes success without position verification
- BOUNDED: Enforces maximum drop limits to prevent dangerous falls
- OUTCOME-BASED: Returns detailed structured data for higher-level decision making
- COMPOSABLE: Designed to be composed into multi-step downward navigation sequences
- POSITION-AWARE: Always tracks position transitions and elevation changes for spatial reasoning

