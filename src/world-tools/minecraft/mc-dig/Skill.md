---
name: mc-dig
type: python
description: "Removes a block from the Minecraft world at a specified location. ASYNCHRONOUS. Returns request acceptance status. May cause an item entity to spawn, but item spawning, item type, and item location are not guaranteed and are not directly reported by this tool."
schema_hint:
  value: "ignored"
  forward: "blocks forward (float, egocentric, + is front, - is back)"
  right: "blocks right (float, egocentric, + is right, - is left)"
  up: "blocks up (float, egocentric, + is up, - is down)"
  x: "absolute x coordinate (float, if using absolute)"
  y: "absolute y coordinate (float, if using absolute)"
  z: "absolute z coordinate (float, if using absolute)"
  out: "$variable"
examples:
  - '{"type":"mc-dig","forward":1,"up":0,"right":0,"out":"$dig"}'
  - '{"type":"mc-dig","forward":0,"up":-1,"right":0,"out":"$dig"}'
  - '{"type":"mc-dig","x":100,"y":64,"z":200,"out":"$dig"}'
---

# Minecraft Dig Tool

## Input
- `value`: ignored
- Egocentric position (Preferred): `forward`, `right`, `up` (relative to agent facing)
- Absolute position: `x`, `y`, `z` (legacy/global)
- Relative Cartesian (Legacy): `rel_x`, `rel_y`, `rel_z`

## Output
- Uniform return format dict (bound to `out` variable if specified):
  - `status`: 'success' or 'failed' (execution status - whether API call succeeded)
  - `value`: truncated text summary (e.g., "Dig request accepted: stone at (x, y, z)" or "Dig request accepted: target block is already air")
  - `data`: structured data dict containing:
    - `status`: "accepted" (request accepted, but digging may still fail asynchronously)
    - `dug`: dict with `name` (block type) and `position` (x, y, z)
    - All other API response fields
  - `resource_id`: None (no resource created)
  
**Note**: Digging is asynchronous - the request may be accepted but actual digging may fail due to Minecraft physics (e.g., block too hard, cannot reach target, already air, insufficient time to break block).

## Configuration
- Configured via `world_config.port` (defaults to `http://localhost:3003`)
- Can be overridden with `world_config.url` or `MINECRAFT_URL` environment variable

## Common Workflow
```json
{"type":"mc-observe-blocks","out":"$obs"}
{"type":"mc-dig","forward":1,"up":0,"right":0,"out":"$dig"}
{"type":"mc-observe-blocks","out":"$obs_after"}
```

## Cognitive Contract
- ASYNCHRONOUS: Dig request is accepted immediately, but actual digging happens asynchronously.
- RETURNS REQUEST STATUS: The output tells you if the request was accepted, not whether digging actually succeeded.
- Digging may fail due to Minecraft physics even if request is accepted (e.g., block too hard, cannot reach, already air).
- Use `mc-observe-blocks` after digging to verify the block was actually removed.
- Failure is informative, not fatal - Jill learns from failures
- Returns what was targeted for digging - useful for planning and verification

