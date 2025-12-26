---
name: mc-open
type: python
description: "Open block-based UI (crafting table, chest, furnace). Returns success/failure."
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
  - '{"type":"mc-open","forward":1,"up":0,"right":0,"out":"$open"}'
---

# Minecraft Open Tool

## Input
- Egocentric position: `forward`, `right`, `up` (preferred)
- Absolute position: `x`, `y`, `z` (all required if using absolute)
- Relative position: `rel_x`, `rel_y`, `rel_z` (all required if using relative)

## Output
- Note ID (bound to `out` variable) containing:
  - `text`: success/failure message
  - `metadata`: raw response including:
    - `success`: boolean - whether open succeeded
    - `ui_type`: string - type of UI opened (e.g., "crafting_table", "chest", "furnace")

## Configuration
- Configured via `world_config.port` (defaults to `http://localhost:3003`)
- Can be overridden with `world_config.url` or `MINECRAFT_URL` environment variable

## Common Workflow
```json
{"type":"mc-open","forward":1,"up":0,"right":0,"out":"$open"}
{"type":"mc-craft","recipe":"minecraft:stick","count":4,"out":"$craft"}
{"type":"mc-close","out":"$close"}
```

## Cognitive Contract
- Opens block-based UI (crafting table, chest, furnace)
- Required before mc-craft (if crafting table needed)
- UI state should be checked via mc-inventory or mc-status

