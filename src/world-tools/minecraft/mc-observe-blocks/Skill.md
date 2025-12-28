---
name: mc-observe-blocks
type: python
description: "Exhaustive enumeration of ALL visible non-air blocks within radius R. Visibility = within radius R AND line-of-sight from bot's eye position. Does NOT report items (use mc-observe-items for that)."
schema_hint:
  value: "ignored"
  radius: "optional observation radius (default: 5, max: 6)"
  blocks_radius: "optional blocks observation radius (default: 5, max: 6)"
  entities_radius: "optional entities observation radius (default: 12)"
  out: "$variable"
examples:
  - '{"type":"mc-observe-blocks","out":"$obs"}'
  - '{"type":"mc-observe-blocks","radius":5,"out":"$obs"}'
---

# Minecraft Observe-Blocks Tool

## Input
- `value`: ignored
- `radius`: optional - observation radius (default: 5, max: 6)
- `blocks_radius`: optional - blocks observation radius (default: 5, max: 6)
- `entities_radius`: optional - entities observation radius (default: 5, max: 12)

## Output
- Note ID (bound to `out` variable) containing:
  - `text`: formatted observation information
  - `metadata`: raw observation data including:
    - `position`: dict with x, y, z coordinates
    - `orientation`: dict with yaw, pitch (radians)
    - `nearby_blocks`: list of **ALL visible non-air blocks** within radius (exhaustive enumeration)
    - `blocks_complete`: boolean indicating if enumeration completed (false if timeout)
    - `blocks_elapsed_ms`: time taken for block enumeration
    - `nearby_entities`: list of non-item entity observations (mobs, players, etc.) - **items are excluded**
    - `visibility_distances`: dict with distances to nearest opaque block in 6 directions (forward, back, left, right, up, down)
      - Each value is a number (distance in blocks) or `null` if no block found within radius (reported as `">{radius} blocks"`)

**IMPORTANT**: The EXACT_RESULT reported in tool execution traces is **truncated** for display purposes. To see the complete observation data including all blocks and entities, use the `load` primitive to load the Note bound to the `out` variable (e.g., `{"type":"load","target":"$obs","out":"$obs_full"}`).

## Configuration
- Configured via `world_config.port` (defaults to `http://localhost:3003`)
- Can be overridden with `world_config.url` or `MINECRAFT_URL` environment variable

## Common Workflow
```json
{"type":"mc-observe-blocks","out":"$obs"}
{"type":"mc-move","forward":true,"duration":2.0,"out":"$move"}
{"type":"mc-observe-blocks","out":"$obs_after"}
```

## Cognitive Contract

### Exhaustive Visibility Guarantee
- **ALL visible non-air blocks within radius R are returned** - this is exhaustive, not sampled
- **Visibility definition**: A block is visible if:
  - It is within radius R (Euclidean distance from bot position), AND
  - There exists line-of-sight from the bot's eye position to the block (raycast not blocked by opaque blocks)
- **Safe negation**: If a block type is not listed within radius R, Jill can safely infer it is not visible nearby
- **No air blocks**: Air blocks are excluded from results

### Performance
- Default radius: 5 blocks (max: 6 blocks)
- Timeout protection: Enumeration stops if it exceeds ~10ms to prevent blocking
- If timeout occurs, `blocks_complete` will be `false` and results are partial

### Visibility Distances
- Reports visibility distances in 6 directions: forward, back, left, right, up, down
- Forward/back/left/right are relative to bot's current yaw (egocentric)
- Up/down are absolute (world Y-axis)
- Distance is to nearest opaque block in that direction
- If no opaque block found within radius, reports `">{radius} blocks (no block found)"`
- Useful for understanding spatial constraints and planning movement

### Other Notes
- Reports blocks and non-item entities (mobs, players) - **items are NOT reported**
- Use `mc-observe-items` to see dropped items in the environment
- Jill uses this to ground plans, verify action effects, detect hazards/opportunities
- This enables reliable spatial reasoning: "If stone is not listed within radius 5, there is no visible stone nearby"

