---
name: mc-place
type: python
description: "Build / modify world - place blocks. Returns ok/error."
schema_hint:
  value: "item/block name"
  item: "item/block name (alternative to value)"
  forward: "blocks forward (float, egocentric)"
  right: "blocks right (float, egocentric)"
  up: "blocks up (float, egocentric)"
  x: "absolute x coordinate of reference block (float, if using absolute)"
  y: "absolute y coordinate of reference block (float, if using absolute)"
  z: "absolute z coordinate of reference block (float, if using absolute)"
  rel_x: "relative x offset to reference block (float, if using relative)"
  rel_y: "relative y offset to reference block (float, if using relative)"
  rel_z: "relative z offset to reference block (float, if using relative)"
  face: "face of reference block (string: top, bottom, north, south, east, west)"
  out: "$variable"
examples:
  - '{"type":"mc-place","value":"dirt","forward":1,"up":0,"right":0,"face":"north","out":"$place"}'
  - '{"type":"mc-place","item":"cobblestone","x":100,"y":64,"z":200,"face":"top","out":"$build"}'
---

# Minecraft Place Tool

## Input
- `value`: item/block name to place (preferred)
- `item`: item/block name to place (alternative to value)
- Reference position: either Egocentric (`forward`, `right`, `up`), Absolute (`x`, `y`, `z`), or Relative (`rel_x`, `rel_y`, `rel_z`) - REQUIRED
- `face`: string - face of reference block to place against (top, bottom, north, south, east, west) - REQUIRED

## Output
- Note ID (bound to `out` variable) containing:
  - `text`: success/failure message
  - `metadata`: raw response including:
    - `success`: boolean - whether place succeeded
    - `error_code`: string - failure reason code if unsuccessful (e.g., "no_reference_block", "no_reference_face", "item_not_equipped", "item_not_in_inventory", "out_of_range")

## Configuration
- Configured via `world_config.port` (defaults to `http://localhost:3003`)
- Can be overridden with `world_config.url` or `MINECRAFT_URL` environment variable

## Common Workflow
```json
{"type":"mc-inventory","out":"$inv"}
{"type":"mc-equip","item":"stone","slot":"hand","out":"$equip"}
{"type":"mc-observe-blocks","out":"$obs"}
{"type":"mc-place","value":"stone","forward":1,"up":0,"right":0,"face":"top","out":"$place"}
{"type":"mc-observe","out":"$obs_after"}
```

## Cognitive Contract
- Builds/modifies world - places blocks
- REQUIRES: item must be equipped (use mc-equip first)
- REQUIRES: reference block position and face must be specified
- Returns ok/error - success or failure indication
- Used for construction, modification, building

