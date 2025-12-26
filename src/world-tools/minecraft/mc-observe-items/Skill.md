---
name: mc-observe-items
type: python
description: "Exhaustive enumeration of ALL item entities within radius R. Only reports item entities (dropped items), not blocks or other entities."
schema_hint:
  value: "ignored"
  radius: "optional observation radius for items (default: 5, max: 12)"
  entities_radius: "optional entities observation radius (default: 5, max: 12)"
  out: "$variable"
examples:
  - '{"type":"mc-observe-items","out":"$items"}'
  - '{"type":"mc-observe-items","radius":5,"out":"$items"}'
---

# Minecraft Observe-Items Tool

## Input
- `value`: ignored
- `radius`: optional - observation radius for items (default: 5, max: 12)
- `entities_radius`: optional - entities observation radius (default: 5, max: 12)

## Output
- Note ID (bound to `out` variable) containing:
  - `text`: formatted observation information
  - `metadata`: raw observation data including:
    - `position`: dict with x, y, z coordinates
    - `nearby_entities`: list of **ALL item entities** within radius (exhaustive enumeration)
    - `entities_complete`: boolean indicating if enumeration completed (false if timeout)
    - `entities_elapsed_ms`: time taken for entity enumeration
    - `visibility_distances`: dict with distances to nearest opaque block in 6 directions (forward, back, left, right, up, down)
      - Each value is a number (distance in blocks) or `null` if no block found within radius (reported as `">{radius} blocks"`)

## Configuration
- Configured via `world_config.port` (defaults to `http://localhost:3003`)
- Can be overridden with `world_config.url` or `MINECRAFT_URL` environment variable

## Common Workflow
```json
{"type":"mc-dig","forward":1,"up":0,"right":0,"out":"$dig"}
{"type":"mc-observe-items","out":"$items"}
{"type":"mc-move","forward":true,"duration":0.5,"out":"$nudge"}
{"type":"mc-inventory","out":"$inv"}
```

## Cognitive Contract

### Exhaustive Enumeration Guarantee
- **ALL item entities within radius R are returned** - this is exhaustive, not sampled
- **Safe negation**: If an item type is not listed within radius R, Jill can safely infer it is not present nearby
- Items are entities, not blocks - they can be detected through walls (no line-of-sight required)

### Performance
- Default radius: 5 blocks (max: 12 blocks)
- Timeout protection: Enumeration stops if it exceeds ~10ms to prevent blocking
- If timeout occurs, `entities_complete` will be `false` and results are partial

### Visibility Distances
- Reports visibility distances in 6 directions: forward, back, left, right, up, down
- Forward/back/left/right are relative to bot's current yaw (egocentric)
- Up/down are absolute (world Y-axis)
- Distance is to nearest opaque block in that direction
- If no opaque block found within radius, reports `">{radius} blocks (no block found)"`
- Useful for understanding spatial constraints and planning movement

### Other Notes
- **Only reports dropped items** (item entities) - does NOT report blocks or other entities
- Use `mc-observe-blocks` to see blocks and non-item entities
- Items appear after digging blocks or dropping items from inventory
- Items automatically collect into inventory when within ~1.5 blocks (see mc-pickup instruction)
- Useful for verifying drops after digging or checking for items before pickup
- This enables reliable item detection: "If diamond is not listed within radius 5, there are no diamonds nearby"

