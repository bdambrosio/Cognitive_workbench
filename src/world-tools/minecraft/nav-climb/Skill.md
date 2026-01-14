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
---

# nav-climb

Attempt exactly ONE adjacent climb forward: move into a neighbor cell ~+1 block higher. Tries walk-up first, then jump-up if needed.

## Input

- `step_duration`: Float seconds for each move attempt (default: `0.6`)
- `allow_walkable_landing`: Boolean, accepts walkable landings like snow (default: `True`)
- `min_delta_y`: Float minimum Y gain to count as climb (default: `0.9`)

## Output

Success (`status: "success"`):
- `value`: Summary text
- `extra.from`: Starting position `{x, y, z}`
- `extra.to`: Ending position `{x, y, z}`
- `extra.delta_y`: Vertical displacement (≥ `min_delta_y`)
- `extra.mode`: `"walk"` or `"jump"`
- `extra.support_here`: Support type at destination

Failure (`status: "failed"`):
- `reason`: One of `"status_failed"`, `"collision"`, `"fell"`, `"observation_failed"`, `"support_ambiguous"`, `"not_elevated"`
- `extra.diagnostics`: When `reason="collision"`, contains clearance data:
  - `up_block`: Block directly overhead (use `dirs.up.blk`, NOT `nav_surface.cover_block`)
  - `clear_fwd_head`, `clear_up_head`: Head clearance flags
  - `fwd_block`: Block forward

## Clearance Requirements

For nav-climb to succeed, BOTH source and destination must satisfy clearance:

**Source (current position)**:
- Grid cell above head (real y+2) must be clear
- No adjacent overhangs at (x±1, z±1, y+1) or (x±1, z±1, y+2)

**Destination (forward, y+1)**:
- Body space at (fwd+1, y+1) and head space at (fwd+1, y+2) must be clear
- Supporting block at (fwd+1, y) must be walkable

**Verification**: Use `dirs.up.blk` to check overhead (NOT `nav_surface.cover_block` - those are offset cells).

## Invariants

- Exactly one elevation gain attempt per call (always forward)
- Tries walk-up first, then jump-up if walk-up insufficient
- Elevation gain must be ≥ `min_delta_y` to succeed
- Snaps to block center after any position change
- Updates `world_state("nav")` history

## Collision Recovery

When `reason="collision"`, check `extra.diagnostics`:
- `up_block=null` but collision → likely overhang (adjacent cell blocks jump arc)
- `clear_fwd_head=false` → destination head space blocked
- `clear_up_head=false` → overhead clearance insufficient

**Recovery strategies** (after 1 failed climb + 1 failed dig → change topology):
1. Lateral escape: Move sideways to fully open column (`dirs.up.blk=null`)
2. Overhang removal: Dig forward+up with absolute coordinates `(floor(x), y+2, floor(z))`
3. Pillar up: Use `mc-place-until-supported` to build upward

## Planning Notes

- Does not handle multi-block climbs; compose multiple calls for that
- Use `nav-turn` before climbing to change direction
- Default `step_duration` (0.6s) is longer than `nav-move` (0.2s) for step-up mechanics
- After repeated failures, switch strategy rather than retry

## Example Workflow

```json
{"type":"nav-turn","direction":"right","out":"$t"}
{"type":"nav-climb","out":"$climb1"}
{"type":"nav-climb","out":"$climb2"}
```
