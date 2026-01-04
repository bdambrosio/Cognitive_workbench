---
name: mc-place
type: python
description: "Build / modify world - place blocks. ASYNCHRONOUS. Returns request acceptance status."
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
- Uniform return format dict (bound to `out` variable if specified):
  - `status`: 'success' or 'failed' (execution status - whether API call succeeded)
  - `value`: truncated text summary (e.g., "Place request accepted: dirt at (x, y, z) (face=north)")
  - `data`: structured data dict containing:
    - `status`: "accepted" (request accepted, but placement may still fail asynchronously)
    - `placed`: dict with `name`, `position` (x, y, z), and `face`
    - All other API response fields
  - `resource_id`: None (no resource created)
  
**Note**: Placement is asynchronous - the request may be accepted but actual placement may fail due to Minecraft physics (e.g., block already exists at target location, insufficient space, item not in inventory, cannot reach target location).

## Configuration
- Configured via `world_config.port` (defaults to `http://localhost:3003`)
- Can be overridden with `world_config.url` or `MINECRAFT_URL` environment variable

## Common Workflow
```json
{"type":"mc-inventory","out":"$inv"}
{"type":"mc-equip","item":"stone","slot":"hand","out":"$equip"}
{"type":"mc-observe-blocks","out":"$obs"}
{"type":"mc-place","value":"stone","forward":1,"up":0,"right":0,"face":"top","out":"$place"}
{"type":"mc-observe-blocks","out":"$obs_after"}
```

## Cognitive Contract
- ASYNCHRONOUS: Placement request is accepted immediately, but actual placement happens asynchronously.
- RETURNS REQUEST STATUS: The output tells you if the request was accepted, not whether placement actually succeeded.
- Placement may fail due to Minecraft physics even if request is accepted (e.g., block already exists, insufficient space, item not in inventory).
- Use `mc-observe-blocks` after placement to verify the block was actually placed.
- REQUIRES: item must be equipped (use mc-equip first)
- REQUIRES: reference block position and face must be specified
- Used for construction, modification, building

