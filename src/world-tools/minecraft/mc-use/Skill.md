---
name: mc-use
type: python
description: "Right-click style interaction using equipped item. Returns success/failure."
schema_hint:
  value: "ignored"
  forward: "blocks forward (float, egocentric)"
  right: "blocks right (float, egocentric)"
  up: "blocks up (float, egocentric)"
  x: "absolute x coordinate (float, if using absolute)"
  y: "absolute y coordinate (float, if using absolute)"
  z: "absolute z coordinate (float, if using absolute)"
  rel_x: "relative x offset (float, if using relative)"
  rel_y: "relative y offset (float, if using relative)"
  rel_z: "relative z offset (float, if using relative)"
  out: "$variable"
examples:
  - '{"type":"mc-use","forward":1,"up":0,"right":0,"out":"$use"}'
  - '{"type":"mc-use","x":100,"y":64,"z":200,"out":"$activate"}'
---

# Minecraft Use Tool

## Input
- Egocentric position: `forward`, `right`, `up` (preferred)
- Absolute position: `x`, `y`, `z` (all required if using absolute)
- Relative position: `rel_x`, `rel_y`, `rel_z` (all required if using relative)

## Output
- Note ID (bound to `out` variable) containing:
  - `text`: success/failure message
  - `metadata`: raw response including:
    - `success`: boolean - whether use succeeded
    - `error_code`: string - failure reason code if unsuccessful (e.g., "nothing_equipped", "target_not_interactable", "out_of_range")

## Configuration
- Configured via `world_config.port` (defaults to `http://localhost:3003`)
- Can be overridden with `world_config.url` or `MINECRAFT_URL` environment variable

## Common Workflow
```json
{"type":"mc-inventory","out":"$inv"}
{"type":"mc-equip","item":"bucket","slot":"hand","out":"$equip"}
{"type":"mc-use","forward":1,"up":0,"right":0,"out":"$use"}
```

## Cognitive Contract
- Right-click style interaction (eat food, activate lever, open door, use bucket)
- Requires equipped item for most interactions
- Some interactions may work empty-handed (e.g., opening doors, activating levers)

