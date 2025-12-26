---
name: mc-dig
type: python
description: "removes a block from the Minecraft world at a specified location.  May cause an item entity to spawn, but item spawning, item type, and item location are not guaranteed and are not directly reported by this tool**."
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
- Note ID (bound to `out` variable) containing:
  - `text`: success/failure message with block type
  - `metadata`: raw response including:
    - `success`: boolean - whether dig succeeded
    - `block_type`: string - type of block that was dug (or null if failed)
    - `reason`: string - failure reason if unsuccessful

## Configuration
- Configured via `world_config.port` (defaults to `http://localhost:3003`)
- Can be overridden with `world_config.url` or `MINECRAFT_URL` environment variable

## Common Workflow
```json
{"type":"mc-observe-blocks","out":"$obs"}
{"type":"mc-dig","forward":1,"up":0,"right":0,"out":"$dig"}
{"type":"mc-observe","out":"$obs_after"}
```

## Cognitive Contract
- Atomic at human timescale - dig completes before returning
- Failure is informative, not fatal - Jill learns from failures
- Returns what was dug - useful for planning and verification

